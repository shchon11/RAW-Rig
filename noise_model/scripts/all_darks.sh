#!/bin/bash
# Lens caps on both cameras: darks for every bright stack taken on 2026-10-20 (same exposures/outputs).
#   LUCID (one camera, sequential): low-light session, 0 dB transition session, 12 dB transition session
#   BFS (parallel): low-light session
cd "$(dirname "$0")"
python3 gvcp_ip.py force 254600231 192.168.100.201 > /dev/null 2>&1
python3 gvcp_ip.py force 26315393 192.168.100.202 > /dev/null 2>&1
(
  env -u LD_LIBRARY_PATH ../.venv/bin/python -u fullframe_dark.py bfs --out ../data/lowlight_20261020/dark --gains 0 24 \
      --exposures-us 24 50 100 250 500 1000 2500 5000 10000 20000 --frames 16 --keep 4
  echo "BFS DARK DONE"
) > ../data/darks_bfs_20261020.log 2>&1 &
(
  source ../env.sh
  for s in lowlight_20261020/lucid hdr_transition_20261020 hdr_transition_20261020_g12; do
    python3 gvcp_ip.py force 254600231 192.168.100.201 > /dev/null 2>&1
    echo "== $s"
    python -u hdr_transition_capture.py dark --session ../data/$s 2>&1 | grep --line-buffered -v Warning
  done
  echo "LUCID DARK DONE"
) > ../data/darks_lucid_20261020.log 2>&1 &
wait
echo "ALL DARKS DONE"
