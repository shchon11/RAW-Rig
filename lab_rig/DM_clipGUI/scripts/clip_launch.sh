#!/usr/bin/env bash
# rawgui — 랩 RAW 취득 GUI 런처 (~/jsh/DM_clipGUI, ~/DM/DM_clipGUI 의 복제본).
#
# 원본(차량 DataMachine)과 같은 PC 에서 따로 돈다:
#   - ROS_DOMAIN_ID 기본 42 — 원본의 레코더(/clip_recorder) · 트리거와 섞이지 않게
#   - FASTRTPS_DEFAULT_PROFILES_FILE=~/jsh/config/fastdds_raw.xml — RAW 프레임(4.6~21 MB)용 SHM 크기
#   - 설정은 ~/.config/dm_clip_gui_raw (원본은 ~/.config/dm_clip_gui)
#
#   1) ROS + 워크스페이스 소싱
#   2) net_probe(IP별 트래픽 카운터)에 cap_net_raw가 없으면 sudo setcap 한 번
#      (colcon build로 바이너리가 새로 만들어졌을 때만 비밀번호를 묻는다)
#   3) GUI는 일반 사용자 권한으로 실행
#
# GUI 자체를 sudo로 띄우지 않는 이유: 클립/설정 파일이 root 소유가 되고, DDS 공유메모리
# 세그먼트도 root 소유가 되어 일반 사용자로 도는 카메라 드라이버와 충돌한다.
#
# 실행: ~/jsh/rawgui   (또는 ~/.bashrc 에 alias rawgui='~/jsh/rawgui')

# (set -u 사용 금지: ROS setup.bash가 nounset에 안전하지 않음)
WS="${DM_CLIP_WS:-$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)}"   # 이 스크립트가 있는 리포

JSH="$(dirname "$WS")"
export ROS_DOMAIN_ID="${RAW_LAB_ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [ -f "$JSH/config/fastdds_raw.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$JSH/config/fastdds_raw.xml"
fi
echo "[rawgui] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
else
  echo "워크스페이스가 빌드되지 않음: $WS/install 없음 (colcon build 먼저)" >&2
  exit 1
fi
# 카메라 메시지 타입(flir_spinnaker_camera/msg/FlirMetadata 등)을 GUI 도 읽게 — 녹화 진단 · 미리보기가 GUI 안에서
# bag 을 읽는다. 없으면 진단에 카메라 메타데이터 16토픽이 '메시지 타입 미설치' 로 나왔다 (2026-10-05).
# 센서 기동은 센서군마다 환경을 비우고 sensors.yaml 의 워크스페이스를 따로 소싱하므로 여기 영향을 안 받는다.
FLIR_WS="${DM_FLIR_WS:-$(dirname "$WS")/FLIR_control}"
if [ -f "$FLIR_WS/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$FLIR_WS/install/local_setup.bash"
else
  echo "카메라 워크스페이스 없음: $FLIR_WS/install — 녹화 진단에서 카메라 메타데이터 타입을 못 읽습니다" >&2
fi
# LUCID (lucid_raw_camera/msg/LucidMetadata) — 레코더도 이 환경을 물려받아야 그 토픽을 구독한다.
LUCID_WS="${DM_LUCID_WS:-$(dirname "$WS")/LUCID_control}"
if [ -f "$LUCID_WS/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$LUCID_WS/install/local_setup.bash"
else
  echo "LUCID 워크스페이스 없음: $LUCID_WS/install — LUCID 메타데이터를 녹화 · 진단할 수 없습니다" >&2
fi

NP="$(readlink -f "$WS/install/clip_recorder/lib/clip_recorder/net_probe" 2>/dev/null || true)"
if [ -n "$NP" ] && [ -x "$NP" ]; then
  if ! getcap "$NP" 2>/dev/null | grep -q cap_net_raw; then
    echo "net_probe 패킷 캡처 권한 부여 (빌드 후 한 번): sudo setcap cap_net_raw+ep $NP"
    if ! sudo setcap cap_net_raw+ep "$NP"; then
      echo "권한 부여 실패 — IP별 트래픽 없이 NIC 합계만 표시됩니다" >&2
    fi
  fi
else
  echo "net_probe 없음 ($NP) — IP별 트래픽은 표시되지 않습니다" >&2
fi

# (랩 리그는 PTP 를 쓰지 않는다 — 원본의 PTP 점검 블록은 뺐다)

# ros2 run 을 거치지 않고 GUI 를 바로 exec 한다. ros2 run 은 받은 SIGTERM 을 자식에게 넘기지 않아서
# 이 PID 로 보낸 종료 신호가 GUI 에 닿지 않았다. GUI 는 Ctrl+C · SIGTERM · SIGHUP 을 받으면
# 녹화 정리 → 센서 → 레코더 순으로 내리고 끈다 (한 번 더 누르면 기다리지 않고 강제 종료).
#
# GUI 본체는 소스(scripts/clip_gui.py)를 바로 띄운다. install/.../clip_gui 는 CMake 가 이름을 바꿔 설치하느라(RENAME)
# 심볼릭 링크가 아니라 복사본이라 colcon build 를 다시 하기 전까지 예전 코드다 — 2026-09-23 GUI 를 다시 켜도 수동 녹화
# 자동 진단 · 새 진단 창이 안 나왔다 (설치 사본이 소스보다 365줄 뒤처짐). 함께 import 하는 모듈(sensor_stage 등)은 소스로
# 링크돼 있어 최신이라, 일부만 옛 코드로 도는 걸 알아채기 어려웠다.
SRC_GUI="$WS/scripts/clip_gui.py"
if [ -f "$SRC_GUI" ]; then
  exec python3 "$SRC_GUI" "$@"
fi
GUI="$WS/install/clip_recorder/lib/clip_recorder/clip_gui"
if [ -x "$GUI" ]; then
  exec "$GUI" "$@"
fi
exec ros2 run clip_recorder clip_gui "$@"
