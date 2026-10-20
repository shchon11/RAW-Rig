#!/usr/bin/env bash
# source scripts/setup_lucid_env.bash — ROS 2 Humble + Arena SDK + this workspace.
#
# The node finds the Arena libraries through its RPATH (CMakeLists.txt), so LD_LIBRARY_PATH is only
# set for Arena's own tools and the Python arena_api.

if [[ -n "${BASH_VERSION:-}" && "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "source this file instead of executing it: source scripts/setup_lucid_env.bash" >&2
  exit 1
fi

_lucid_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
export LUCID_ROS_WS="${_lucid_env_dir}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [[ -z "${ARENA_ROOT:-}" ]]; then
  export ARENA_ROOT="${HOME}/ArenaSDK_Linux_x64"
fi
if [[ ! -d "${ARENA_ROOT}" ]]; then
  echo "[lucid_env] Arena SDK not found at ${ARENA_ROOT}; export ARENA_ROOT." >&2
fi
case ":${LD_LIBRARY_PATH:-}:" in
  *":${ARENA_ROOT}/lib64:"*) ;;
  *) export LD_LIBRARY_PATH="${ARENA_ROOT}/lib64:${ARENA_ROOT}/GenICam/library/lib/Linux64_x64:${ARENA_ROOT}/ffmpeg:${ARENA_ROOT}/Metavision/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
esac

if [[ -f "${LUCID_ROS_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${LUCID_ROS_WS}/install/setup.bash"
else
  echo "[lucid_env] ${LUCID_ROS_WS}/install/setup.bash not found yet. Build the workspace first." >&2
fi
unset _lucid_env_dir
