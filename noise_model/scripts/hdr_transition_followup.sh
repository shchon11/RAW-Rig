#!/bin/bash
# Follow-up to the 0 dB transition set (same light, lens caps off):
#  1) 0 dB, full channel stacks, exposures straddling the 2nd switch (ch2 clips at HDR ~575-640k DN)
#  2) 12 dB, HDR only: switch points under digital gain (sensor-domain vs post-gain threshold)
cd "$(dirname "$0")"
source ../env.sh
while pgrep -f "[h]dr_transition_capture.py bright --session ../data/hdr_transition_20261020 " > /dev/null; do sleep 5; done
python3 gvcp_ip.py force 254600231 192.168.100.201 > /dev/null 2>&1
python -u hdr_transition_capture.py bright --session ../data/hdr_transition_20261020 \
    --exposures-ms 42 50 60 2>&1 | grep -v Warning
python3 gvcp_ip.py force 254600231 192.168.100.201 > /dev/null 2>&1
python -u hdr_transition_capture.py bright --session ../data/hdr_transition_20261020_g12 --gain-db 12 --hdr-only \
    --exposures-ms 0.5 0.75 1 2.5 3 3.5 30 36 2>&1 | grep -v Warning
echo "FOLLOWUP DONE"
