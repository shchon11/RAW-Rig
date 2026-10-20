#!/usr/bin/env bash
#
# Record a calibration bag from all eight FLIR cameras.
#
# Usage:
#   scripts/camera_bagging.sh                 # <repo>/bags/calib_<timestamp>
#   scripts/camera_bagging.sh /path/my_bag    # explicit output directory
#
# Stop with Ctrl-C once the target has been seen from enough views.

set -euo pipefail

# Resolve against the repo, not the caller's cwd, so the bag always lands in
# <repo>/bags (a symlink to the data disk) no matter where this is invoked from.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bags_dir="${repo_root}/bags"

output="${1:-${bags_dir}/calib_$(date +%Y%m%d_%H%M%S)}"

# ros2 bag record takes topic names, not globs. A pattern like
# /camera_*/image_rgb/compressed is a *shell* glob: it matches no files, so bash
# passes it through verbatim and the recorder waits for a topic literally named
# "/camera_*/..." that never appears — the bag comes out empty. --regex is how a
# set of topics gets selected.
#
# What each topic is for:
#   image_rgb/compressed  the calibration input. Both calibration nodes subscribe
#                         to this (config/calibration.yaml line 3,
#                         config/extrinsic_calibration.yaml line 20), so the raw
#                         Bayer topic is not needed — and publish_raw is off.
#   camera_info           intrinsics travelling with the frames.
#   image_raw/metadata    carries camera_timestamp_ns, the PTP device timestamp.
#                         header.stamp on the image topics is host *arrival* time
#                         and scatters 4-8 ms across the rig, while the cameras
#                         actually expose within ~7 us of each other. A static
#                         calibration target barely cares, but this is the only
#                         place the true exposure time survives and it costs a few
#                         hundred bytes per frame, so record it.
topic_regex='/camera_[a-z0-9_]+/(image_rgb/compressed|image_raw/metadata|camera_info)'
# RAW research capture (docs/raw_capture_noise_calibration.md §3 #11): FLIR_RECORD_RAW=1 also records
# image_raw (Bayer 16-bit, ~4.6 MB per frame per camera). Run with the RAW params profile.
if [[ "${FLIR_RECORD_RAW:-0}" == "1" ]]; then
  topic_regex='/camera_[a-z0-9_]+/(image_raw|image_rgb/compressed|image_raw/metadata|camera_info)'
fi

echo "[camera_bagging] recording to: ${output}"
echo "[camera_bagging] stop with Ctrl-C"

exec ros2 bag record --output "${output}" --regex "${topic_regex}"
