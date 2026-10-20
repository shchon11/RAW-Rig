# LUCID_control — LUCID Arena 카메라 RAW 취득 (ROS 2 Humble)

LUCID TRI054S-C (Sony IMX490) 를 RAW 연구용으로 취득하는 colcon 워크스페이스.
`~/jsh/FLIR_control` (Blackfly S) 의 `flir_spinnaker_camera` 와 같은 방식으로 동작해서, 랩 GUI
(`~/jsh/DM_clipGUI`) 가 두 카메라를 같은 방법으로 띄우고 조작한다.

## 빌드 · 실행

```bash
source scripts/setup_lucid_env.bash          # ROS 2 Humble + ARENA_ROOT(~/ArenaSDK_Linux_x64) + install/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch lucid_raw_camera lucid_cameras.launch.py      # config/lab_lucid_cameras.yaml 의 카메라 전부
```

Arena SDK 는 사용자 설치(`/etc/ld.so.conf.d` 미등록)라 실행 파일에 RPATH 로 SDK 라이브러리 경로를 넣었다.
`LD_LIBRARY_PATH` 없이도 뜬다 (GUI 는 센서를 띄울 때 `LD_LIBRARY_PATH` 를 지운다).
카메라 IP 는 기동 전에 맞춰야 한다 (GUI 가 자동으로, 손으로는 `~/jsh/imx490_ptc/scripts/gvcp_ip.py force`).

## 토픽 (카메라 네임스페이스마다)

| 토픽 | 타입 | 내용 |
|------|------|------|
| `image_raw` | `sensor_msgs/Image` | 카메라가 보낸 RAW. BayerRG24 → `32SC1`(픽셀당 32-bit, 값 < 2^24) 또는 `raw24_container: packed` 면 `bayer_rggb24`(3바이트 그대로). BayerRG16 → `bayer_rggb16`. BayerRG12p → `bayer_rggb16` 으로 풀어서 (`raw12p_shift`) |
| `image_raw/metadata` | `lucid_raw_camera/LucidMetadata` | 그 프레임의 chunk 값(노출 · 게인 · 블랙 · 타임스탬프 · CRC) + 세션 설정 + 온도. 앞부분 필드는 `FlirMetadata` 와 이름 · 의미가 같다 |
| `camera_info` | `sensor_msgs/CameraInfo` | 매 프레임 (GUI 가 Hz 를 여기서 센다) |
| `image_rgb/compressed` | `sensor_msgs/CompressedImage` | 미리보기 JPEG — RAW 를 디모자이크 · 블랙 제거 · 자동 밝기 · 감마. 연구용 RGB 가 아니다 |

모든 토픽의 `header.stamp` 는 같다 (`timestamp.mode: camera_latched` 면 카메라 시계를 host 시계로 옮긴 프레임 시각).

## 파라미터

- 카메라의 쓰기 가능한 GenICam 노드 전부가 `camera.<노드>` / `stream.<노드>` / `tl_device.<노드>` 파라미터다.
  기동 때 enum → bool → int → float → string 순서로 적용하고, 적용으로 풀린 노드는 다시 등록해 적용하고,
  스트리밍 직전에 다시 읽어 다르면 한 번 더 쓰고 ERROR(`not in effect after startup`)를 남긴다.
- 실행 중 `ros2 param set` (GUI 의 ISP 튜닝)으로 바로 카메라에 들어간다. 스트리밍 중 잠기는 노드
  (`HDROutput`, `AcquisitionFrameRate`, `PixelFormat` …, 기동 로그에 목록) 는 스트림을 잠깐 멈췄다 적용한다
  (`live.restart_stream_for_locked`). 스트림이 다시 시작되면 frame id 가 1 부터 다시 센다.
- `exposure.auto_frame_rate: true` 면 노출 시간에 맞춰 프레임레이트를 내리고 올린다 (TRI054S 는 노출이
  프레임 주기를 못 넘는다). 이때 `camera.AcquisitionFrameRate` 는 상한이다.
- selector 에 묶인 노드: `raw.black_level_raw`(BlackLevelRaw, DN), `raw.balance_ratio_{red,green,blue}`.
- `raw.white_level_dn`: -1 = 자동(아래 실측 클립점), 0 = 모름.
- `nodemap_dump_dir`: 기동 직후와 설정을 바꾼 2초 뒤 노드맵 전체를 `<serial>_<시각>_nodemap.json` 으로 남긴다
  (selector 펼침 — WB 비율, 색변환 계수, BlackLevel, HDR 채널별 게인 · 노출).

## 실측 (2026-10-19, TRI054S-C 254600231, 랩 리그)

| 항목 | 결과 |
|------|------|
| chunk | CRC · Timestamp · ExposureTime · Gain · BlackLevel · PixelFormat 전부 지원. FrameID chunk 는 없어서 GVSP block id 를 쓴다 (`GevGVSPExtendedIDMode: On` 으로 64-bit) |
| R3 (chunk 노출값이 그 프레임 것인가) | 20 → 40 ms 에서 chunk 값과 영상 밝기가 같은 프레임에서 바뀜 (평균 비 2.008). 설정 직후 두 프레임은 `settled=false` |
| 렌즈캡 dark (HDR, BayerRG24, BlackLevelRaw 256) | 평균 257.3, 코드 간격 1 |
| HDR 채널 analog gain | 19.8 / 4.2 / 29.4 / 4.2 dB (`hdr_channel_analog_gain_db`) |
| 채널 출력의 픽셀 포맷 (HDRChannel0) | BayerRG24 = 12-bit 코드 + 256 (최대 4351). BayerRG16 = min(그 값, 4095) << 4 (최대 65520). BayerRG12p = min(그 값, 4095). 16 · 12p 는 포화 근처 256 코드가 잘린다 |
| 스트리밍 중 잠기는 노드 | 40개 (ADCBitDepth, AcquisitionFrameRate(Enable), Binning*, DeviceLinkThroughputLimit(Mode), GevSCPSPacketSize, HDROutput, Height, LUTToneMapping, PixelFormat …) |
| 링크 | BFS 와 같은 스위치 → PC 1 GbE (enp5s0). 합이 ~72 MB/s 를 넘으면 스위치에서 패킷이 버려져 LUCID 프레임(패킷 ~1.1만 개)이 전부 불완전했다 (PC 쪽 드롭 0). 그래서 BFS 20 + LUCID 45 MB/s. 45 MB/s 에서 카메라는 10% 를 재전송용으로 남겨 BayerRG24 는 2.0 Hz 가 상한이다 |

## 구조

```
src/lucid_raw_camera/
  msg/LucidMetadata.msg
  src/lucid_raw_camera_node.cpp     노드 (Arena C++ API)
  src/raw_preview.hpp               미리보기 렌더러 (FLIR_control 과 같은 파일)
  config/lucid_camera_raw.yaml      RAW 프로파일 (GUI 의 '원본')
  config/lab_lucid_cameras.yaml     인벤토리 (serial → namespace)
  launch/lucid_cameras.launch.py    인벤토리의 카메라마다 노드 하나 (params_file, cameras_file)
scripts/setup_lucid_env.bash
```
