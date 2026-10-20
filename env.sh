# source ~/jsh/env.sh — 터미널에서 랩 RAW 리그를 다룰 때 (ros2 topic/bag/param, 진단 스크립트).
# GUI(~/jsh/rawgui)와 같은 ROS 도메인 · DDS 프로파일 · 메시지 타입(FlirMetadata, LucidMetadata).
_jsh="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export ROS_DOMAIN_ID="${RAW_LAB_ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${_jsh}/config/fastdds_raw.xml"
[ -z "${ROS_DISTRO:-}" ] && source /opt/ros/humble/setup.bash
for _ws in FLIR_control LUCID_control DM_clipGUI; do
  [ -f "${_jsh}/${_ws}/install/local_setup.bash" ] && source "${_jsh}/${_ws}/install/local_setup.bash"
done
export ARENA_ROOT="${ARENA_ROOT:-$HOME/ArenaSDK_Linux_x64}"
echo "[jsh] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RAW 데이터: /mnt/data/jsh_raw  GUI: ~/jsh/rawgui"
unset _jsh _ws
