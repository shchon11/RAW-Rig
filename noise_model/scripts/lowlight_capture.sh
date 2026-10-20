#!/bin/bash
# Low-signal PTC supplement (apertures stopped down ~5 stops, lens caps off), then darks.
#   ./lowlight_capture.sh bright     # ~12 min, both cameras in parallel, all frames kept
#   ./lowlight_capture.sh dark       # lens caps on, same exposures
# Fills the 0.1-50 e- range the main PTC did not reach (LUCID ch0/HDR dark range, BFS < 13 e-).
# LUCID: HDR + HDRChannel0..3 stacks per exposure (hdr_transition_capture.py) so ch0 (SP1 HCG,
# saturated under the transition light) gets its own PTC; BFS: full-frame stacks (fullframe_dark.py).
cd "$(dirname "$0")"
PHASE=${1:?bright|dark}
OUT=../data/lowlight_20261020/$PHASE
L_EXP_MS="0.087 0.15 0.3 0.6 1.2 2.5 5 10"
B_EXP="24 50 100 250 500 1000 2500 5000 10000 20000"
if [ "$PHASE" = bright ]; then N=48; KEEP=48; else N=16; KEEP=4; fi
python3 gvcp_ip.py force 254600231 192.168.100.201 > /dev/null 2>&1
python3 gvcp_ip.py force 26315393 192.168.100.202 > /dev/null 2>&1
(
  env -u LD_LIBRARY_PATH ../.venv/bin/python -u fullframe_dark.py bfs --out $OUT --gains 0 24 \
      --exposures-us $B_EXP --frames $N --keep $KEEP
  echo "BFS LOWLIGHT $PHASE DONE"
) > ../data/lowlight_bfs_$PHASE.log 2>&1 &
(
  source ../env.sh
  if [ "$PHASE" = bright ]; then
    python -u hdr_transition_capture.py bright --session ../data/lowlight_20261020/lucid --exposures-ms $L_EXP_MS
  else
    python -u hdr_transition_capture.py dark --session ../data/lowlight_20261020/lucid
  fi
  echo "LUCID LOWLIGHT $PHASE DONE"
) > ../data/lowlight_lucid_$PHASE.log 2>&1 &
wait
echo "LOWLIGHT $PHASE DONE"
