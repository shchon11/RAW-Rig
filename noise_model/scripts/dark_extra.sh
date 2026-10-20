#!/bin/bash
# Extra darks while the lens caps are on (2026-10-20):
#   BFS  : long exposures 50 ms - 5 s at 0/24 dB (dark-current regime), 300-frame stack at 1 ms (RTS pixels)
#   LUCID: 150 - 800 ms in 8 steps (dark-electron Poisson regime), 120-frame stack at 87 us (RTS pixels)
# ./dark_extra.sh bfs | lucid
cd "$(dirname "$0")"
OUT=../data/dark_extra_20261020
case $1 in
  bfs)
    python3 gvcp_ip.py force 26315393 192.168.100.202 > /dev/null 2>&1
    env -u LD_LIBRARY_PATH ../.venv/bin/python -u fullframe_dark.py bfs --out $OUT/long --gains 0 24 \
        --exposures-us 50000 200000 500000 2000000 5000000 --frames 16 --keep 4
    env -u LD_LIBRARY_PATH ../.venv/bin/python -u fullframe_dark.py bfs --out $OUT/rts --gains 0 \
        --exposures-us 1000 --frames 300 --keep 300
    echo "BFS EXTRA DONE" ;;
  lucid)
    python3 gvcp_ip.py force 254600231 192.168.100.201 > /dev/null 2>&1
    source ../env.sh
    python -u fullframe_dark.py lucid --out $OUT/long --gains 0 \
        --exposures-us 150000 200000 300000 400000 500000 600000 700000 800000 --frames 16 --keep 4
    python -u fullframe_dark.py lucid --out $OUT/rts --gains 0 --exposures-us 87 --frames 120 --keep 120
    echo "LUCID EXTRA DONE" ;;
esac
