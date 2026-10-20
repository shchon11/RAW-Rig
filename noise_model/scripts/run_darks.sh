#!/bin/bash
# All dark captures for the 2131 sessions, lens caps on both cameras. Two parallel tracks:
#   BFS  : matched ROI darks for every gain -> full-frame darks (6 gains x 6 exposures x 300)
#   LUCID: matched ROI darks (0 dB x 5 outputs, HDR x 5 gains) -> full-frame darks (same grid)
# Both cameras reboot now and then (ForceIP lost): every step re-forces the IP and retries once.
# Each track prints "TRACK <name> DONE" when it releases its camera.
cd "$(dirname "$0")"
D=../data
L=$D/ptc_20261019_2131_lucid
B=$D/ptc_20261019_2131_bfs
FF=$D/fullframe_dark_20261020
GAINS="0 6 10 12 18 24"
EXPS="1000 10000 30000 100000 300000 1000000"

step() {  # step <serial> <ip> <command...>
  local serial=$1 ip=$2; shift 2
  for attempt in 1 2; do
    python3 gvcp_ip.py force "$serial" "$ip" > /dev/null 2>&1
    echo "STEP $(date +%H:%M:%S) try $attempt: $*"
    "$@" && return 0
    echo "STEP FAILED $(date +%H:%M:%S): $*"
    sleep 5
  done
  return 1
}

(
  BPY="env -u LD_LIBRARY_PATH ../.venv/bin/python -u"
  for g in $GAINS; do
    step 26315393 192.168.100.202 $BPY bfs_ptc_capture.py dark --gain-db $g --session $B
  done
  for g in $GAINS; do
    step 26315393 192.168.100.202 $BPY fullframe_dark.py bfs --out $FF --gains $g --exposures-us $EXPS --frames 300
  done
  echo "TRACK BFS DONE $(date +%H:%M:%S)"
) > $D/darks_bfs.log 2>&1 &

(
  source ../env.sh
  step 254600231 192.168.100.201 python -u ptc_capture.py dark --session $L
  for g in 6 10 12 18 24; do
    step 254600231 192.168.100.201 python -u ptc_capture.py dark --gain-db $g --outputs HDR --session $L
  done
  for g in $GAINS; do
    step 254600231 192.168.100.201 python -u fullframe_dark.py lucid --out $FF --gains $g --exposures-us $EXPS --frames 300
  done
  echo "TRACK LUCID DONE $(date +%H:%M:%S)"
) > $D/darks_lucid.log 2>&1 &

wait
echo "ALL DARKS DONE $(date +%H:%M:%S)"
