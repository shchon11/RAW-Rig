#!/usr/bin/env bash

set -euo pipefail

interface="enp3s0f1"
host_cidr="192.168.1.100/24"
remove_cidr="192.168.1.0/24"
configure_ptp4l=true
tune_receive=true
mtu=""
rx_ring=4096
# Spinnaker needs a large receive socket buffer; the 212 KB default drops GigE
# stream packets as soon as several cameras stream at once.
rmem_bytes=10485760
backlog=5000
persist=true
sysctl_conf="/etc/sysctl.d/60-flir-gige.conf"
dry_run=false

usage() {
  cat <<'USAGE'
Usage: scripts/setup_camera_nic.bash [options]

Prepare the PC camera NIC for the FLIR GigE/PTP launch.

Options:
  --interface IFACE       Camera NIC name. Default: enp3s0f1
  --host-cidr CIDR        PC host address to keep/add. Default: 192.168.1.100/24
  --remove-cidr CIDR      Bad PC alias to remove. Default: 192.168.1.0/24
  --mtu BYTES             Set NIC MTU. Use 9000 only if the switch does jumbo
                          frames too, and then raise camera.GevSCPSPacketSize
                          to 9000 in config/flir_camera.yaml to match.
  --rx-ring N             NIC RX ring buffer entries. Default: 4096
  --rmem BYTES            net.core.rmem_max/rmem_default. Default: 10485760
  --skip-receive-tuning   Do not touch socket buffers, backlog, or RX ring
  --skip-persist          Apply the sysctl values now but do not write them to
                          /etc/sysctl.d/60-flir-gige.conf
  --skip-ptp4l-cap        Do not set ptp4l capabilities
  --dry-run               Print commands without running them
  -h, --help              Show this help

The socket-buffer sysctls are written to /etc/sysctl.d/60-flir-gige.conf so they
survive a reboot. A reboot resets them to the 212 KB kernel default, which drops
GigE stream packets as soon as several cameras stream at once.

The NIC RX ring and the host IP are not sysctls and reset on reboot too, so run
this script once per boot, or install it as a boot-time service:
  sudo cp scripts/flir-camera-nic.service /etc/systemd/system/
  sudo systemctl enable --now flir-camera-nic.service

Example:
  scripts/setup_camera_nic.bash --interface enp3s0f1 --host-cidr 192.168.1.100/24
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interface)
      interface="${2:?missing value for --interface}"
      shift 2
      ;;
    --host-cidr)
      host_cidr="${2:?missing value for --host-cidr}"
      shift 2
      ;;
    --remove-cidr)
      remove_cidr="${2:?missing value for --remove-cidr}"
      shift 2
      ;;
    --mtu)
      mtu="${2:?missing value for --mtu}"
      shift 2
      ;;
    --rx-ring)
      rx_ring="${2:?missing value for --rx-ring}"
      shift 2
      ;;
    --rmem)
      rmem_bytes="${2:?missing value for --rmem}"
      shift 2
      ;;
    --skip-receive-tuning)
      tune_receive=false
      shift
      ;;
    --skip-persist)
      persist=false
      shift
      ;;
    --skip-ptp4l-cap)
      configure_ptp4l=false
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v ip >/dev/null 2>&1; then
  echo "ip command not found." >&2
  exit 1
fi

if ! ip link show "$interface" >/dev/null 2>&1; then
  echo "Interface '$interface' not found." >&2
  exit 1
fi

run() {
  if [[ "$dry_run" == true ]]; then
    printf '+'
    printf ' %q' "$@"
    printf '\n'
    return
  fi

  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

has_cidr() {
  ip -o -4 addr show dev "$interface" | awk '{print $4}' | grep -Fxq "$1"
}

echo "[camera-nic] interface=${interface} host_cidr=${host_cidr} remove_cidr=${remove_cidr}"

run ip link set dev "$interface" up

if [[ -n "$remove_cidr" ]] && has_cidr "$remove_cidr"; then
  run ip addr del "$remove_cidr" dev "$interface"
fi

if ! has_cidr "$host_cidr"; then
  run ip addr add "$host_cidr" dev "$interface"
fi

if [[ -n "$mtu" ]]; then
  run ip link set dev "$interface" mtu "$mtu"
fi

if [[ "$tune_receive" == true ]]; then
  run sysctl -w "net.core.rmem_max=${rmem_bytes}"
  run sysctl -w "net.core.rmem_default=${rmem_bytes}"
  run sysctl -w "net.core.netdev_max_backlog=${backlog}"

  if [[ "$persist" == true ]]; then
    sysctl_body="$(cat <<CONF
# Written by scripts/setup_camera_nic.bash for the FLIR GigE camera rig.
# The kernel default rmem of 212 KB drops stream packets once several cameras
# stream at once, and a reboot restores that default unless this file exists.
net.core.rmem_max = ${rmem_bytes}
net.core.rmem_default = ${rmem_bytes}
net.core.netdev_max_backlog = ${backlog}
CONF
)"
    if [[ "$dry_run" == true ]]; then
      echo "+ write ${sysctl_conf}:"
      echo "$sysctl_body" | sed 's/^/    /'
    else
      printf '%s\n' "$sysctl_body" | run tee "$sysctl_conf" >/dev/null
      echo "[camera-nic] persisted socket-buffer sysctls to ${sysctl_conf}"
    fi
  fi

  if command -v ethtool >/dev/null 2>&1; then
    # Not every driver allows resizing the ring, so do not fail the whole setup.
    run ethtool -G "$interface" rx "$rx_ring" || \
      echo "[camera-nic] could not set RX ring to ${rx_ring} on ${interface}; continuing." >&2
  else
    echo "[camera-nic] ethtool not found; skipping RX ring resize." >&2
  fi
fi

if [[ "$configure_ptp4l" == true ]]; then
  if command -v ptp4l >/dev/null 2>&1; then
    run setcap cap_net_raw,cap_net_admin,cap_net_bind_service,cap_sys_time+ep "$(command -v ptp4l)"
  else
    echo "[camera-nic] ptp4l not found; install linuxptp to use PTP master launch." >&2
  fi
fi

ip -br addr show "$interface"
ip -o link show "$interface" | grep -o "mtu [0-9]*"
