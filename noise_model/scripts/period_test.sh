#!/bin/bash
# LUCID dark: does the >1 s anomaly follow the frame period rather than the exposure?
# Same exposure, two frame periods. Lens cap on, LUCID driver stopped. ~5 min.
cd "$(dirname "$0")"
source ../env.sh
OUT=/mnt/data/jsh_raw/trigger_dark_check/$(date +%Y%m%d_%H%M%S)_lucid_period_test
python3 gvcp_ip.py force 254600231 192.168.100.201 > /dev/null 2>&1
run() {  # run <name> <exposure_us> <rate_cap>
  python -u fullframe_dark.py lucid --out "$OUT/$1" --gains 0 --exposures-us "$2" --frames 50 --keep 5 --rate-cap "$3"
}
run p500_t300ms    300000  2.0     # period 500 ms  (as calibration)
run p1050_t300ms   300000  0.952   # period 1050 ms
run p800_t800ms    800000  2.0     # period 800.5 ms
run p1000_t1000ms 1000000  2.0     # period 1000.5 ms (as calibration)
run p1050_t1000ms 1000000  0.952   # period 1050 ms
echo "$OUT"
