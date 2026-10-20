"""GigE Vision discovery / temporary ForceIP without any vendor SDK.

  python3 gvcp_ip.py list  [--iface enp5s0]
  python3 gvcp_ip.py force <serial> <ip> [--mask 255.255.255.0] [--gw 0.0.0.0] [--iface enp5s0]

ForceIP is not persistent: the camera returns to its stored IP config after a power cycle.
"""
import argparse
import socket
import struct
import time

PORT = 3956


def _sock(iface):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface.encode())
    s.bind(("", 0))
    s.settimeout(0.3)
    return s


def _str(b):
    return b.split(b"\0", 1)[0].decode(errors="replace")


def discover(iface, wait=1.5):
    s = _sock(iface)
    s.sendto(struct.pack(">BBHHH", 0x42, 0x11, 0x0002, 0, 1), ("255.255.255.255", PORT))
    found, end = {}, time.time() + wait
    while time.time() < end:
        try:
            data, _ = s.recvfrom(2048)
        except socket.timeout:
            continue
        if len(data) < 8 + 0xF8 or struct.unpack(">H", data[2:4])[0] != 0x0003:
            continue
        p = data[8:]
        cur = struct.unpack(">I", p[0x14:0x18])[0]
        dev = {"mac": p[0x0A:0x10], "ip": socket.inet_ntoa(p[0x24:0x28]), "mask": socket.inet_ntoa(p[0x34:0x38]),
               "vendor": _str(p[0x48:0x68]), "model": _str(p[0x68:0x88]), "version": _str(p[0x88:0xA8]),
               "serial": _str(p[0xD8:0xE8]), "name": _str(p[0xE8:0xF8]),
               "mode": "persistent" if cur & 1 else "dhcp" if cur & 2 else "lla"}
        found[dev["serial"]] = dev
    return list(found.values())


def force_ip(iface, mac, ip, mask, gw):
    s = _sock(iface)
    body = (b"\0\0" + mac[:2] + mac[2:6] + b"\0" * 12 + socket.inet_aton(ip) + b"\0" * 12
            + socket.inet_aton(mask) + b"\0" * 12 + socket.inet_aton(gw))
    s.sendto(struct.pack(">BBHHH", 0x42, 0x11, 0x0004, len(body), 2) + body, ("255.255.255.255", PORT))
    end = time.time() + 3
    while time.time() < end:
        try:
            data, _ = s.recvfrom(64)
        except socket.timeout:
            continue
        status, ack = struct.unpack(">HH", data[:4])
        if ack == 0x0005:
            return status
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["list", "force"])
    ap.add_argument("serial", nargs="?")
    ap.add_argument("ip", nargs="?")
    ap.add_argument("--mask", default="255.255.255.0")
    ap.add_argument("--gw", default="0.0.0.0")
    ap.add_argument("--iface", default="enp5s0")
    a = ap.parse_args()
    devices = discover(a.iface)
    if a.cmd == "list":
        for d in devices:
            print(f"{d['serial']:>12}  {d['ip']:>15}/{d['mask']:<15} {d['mode']:<10} {d['vendor']} {d['model']} fw {d['version']}")
        if not devices:
            print("no GigE Vision device answered on", a.iface)
        return
    dev = next((d for d in devices if d["serial"] == a.serial), None)
    if dev is None:
        raise SystemExit(f"serial {a.serial} not found on {a.iface}")
    status = force_ip(a.iface, dev["mac"], a.ip, a.mask, a.gw)
    if status is None:
        print("no FORCEIP ack (some cameras do not ack); checking discovery")
    elif status != 0:
        raise SystemExit(f"FORCEIP rejected, status 0x{status:04x}")
    time.sleep(2)
    after = next((d for d in discover(a.iface) if d["serial"] == a.serial), None)
    print(f"{a.serial}: {dev['ip']} -> {after['ip'] if after else 'not seen'}")


if __name__ == "__main__":
    main()
