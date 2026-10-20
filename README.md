# ~/jsh — 저조도 RAW 취득 리그 (FLIR BFS + LUCID TRI054S)

ISP 셋업별 RGB → RAW 재구성 연구용 RAW 데이터 취득 환경. 두 카메라를 GUI 하나에서 띄우고, 설정을 바꾸고,
녹화한다. 녹화에는 **PTC 노이즈 모델을 지키는 데 필요한 메타데이터**가 프레임마다 같이 들어간다.

| 카메라 | 시리얼 | IP (ForceIP) | 드라이버 |
|--------|--------|--------------|----------|
| FLIR Blackfly S BFS-PGE-23S3C (Sony IMX392) — 노이즈 기준 캘리브레이션 카메라 | 26315393 | 192.168.100.202 | `FLIR_control` (`flir_spinnaker_camera`, 브랜치 `feat/raw-capture`) |
| LUCID TRI054S-C (Sony IMX490, HDR) | 254600231 | 192.168.100.201 | `LUCID_control` (`lucid_raw_camera`, 새로 작성) |

둘 다 `enp5s0` (192.168.100.100/24, MTU 1500) 의 같은 스위치에 물려 있다.

## 구성

```
~/jsh/
  rawgui                GUI 실행 (DM_clipGUI/scripts/clip_launch.sh)
  env.sh                터미널용: source ~/jsh/env.sh (ROS 도메인 42 · DDS 프로파일 · 메시지 타입)
  config/fastdds_raw.xml  큰 RAW 프레임(4.6~21 MB)용 Fast DDS SHM 설정
  FLIR_control/         ~/DM/FLIR_control 클론, 브랜치 feat/raw-capture (RAW 명세 §3–§4 구현)
  LUCID_control/        LUCID RAW 드라이버 워크스페이스 (git, 로컬)
  DM_clipGUI/           ~/DM/DM_clipGUI 클론, 브랜치 jsh/raw-lab (랩 모드)
  tools/rawbag.py       bag → numpy RAW + 프레임별 메타데이터
  bracket               브라케팅 촬영 창 (tools/bracket_gui.py · bracket.py 엔진)
  tools/rawdng.py       무손실 DNG 쓰기 · 읽기 (uint16 / 24-bit packed)
  tools/ptc_model.py    노이즈 모델(imx490_ptc/calibration)의 녹화 조건 · 유효 범위 · 프레임별 값 · NoiseProfile
  tools/gvcp_action.py  GigE Vision 액션 커맨드 (트리거)
  tools/aim_view.py     두 카메라 미리보기를 한 창에 (위치 맞출 때)
  tools/trigger_dark_check.py · trigger_dark_analyze.py   트리거 / free-run dark 를 찍어 노이즈 모델과 대조
  imx490_ptc/           PTC 측정과 노이즈 모델 (jsh-16 세션). 모델 패키지는 imx490_ptc/calibration/
  lucid_vision_driver/, lucid_ws/   autoware LUCID 드라이버 (BGR8 미리보기 전용, 이제 안 씀)
/mnt/data/jsh_raw/
  recordings/scene_NNN_<장면>/rec_<시각>/   녹화 (bag + capture_info.json + nodemap/ + params/)
  nodemap/              카메라 노드맵 덤프 (기동 때 · 설정 변경 2초 뒤)
```

이 폴더의 도구(`tools/` · `config/` · 실행 스크립트 · 이 README)는 git 저장소(`~/jsh`, main)이고,
GitHub `shchon11/RAW-Rig` 의 `lab-rig` 브랜치 `lab_rig/` 로 올린다 (git subtree, 작업 폴더 `~/jsh/RAW-Rig-lab`):
`cd ~/jsh/RAW-Rig-lab && git subtree pull --prefix=lab_rig ~/jsh main && git push origin lab-rig`.

원본 `~/DM` 은 건드리지 않았다. 랩 GUI 는 원본 DataMachine GUI 와 같은 PC 에서 따로 돈다 — 설정
`~/.config/dm_clip_gui_raw`, ROS_DOMAIN_ID 42 (원본 레코더 · 트리거와 안 섞임), 지도 서버 · 휴대폰 연동은 안 띄운다.

## 쓰는 법

```bash
~/jsh/rawgui
```

1. **[센서 기동]** — 감지 → (IP 자동 맞춤) → ▶ 기동. 카드 두 개: FLIR 카메라 · LUCID 카메라.
   다른 프로그램(예: PTC 스크립트)이 카메라를 쥐고 있으면 IP 를 바꾸지 않고 알린다.
2. 설정 탭 / **ISP 튜닝** — 노출 · 게인 · HDR 출력 · 픽셀 포맷 · 블랙 레벨 · 미리보기 밝기 등을 실행 중에 바꾼다.
   LUCID 의 HDR 출력 · 픽셀 포맷 · 프레임레이트는 스트리밍 중 잠기는 노드라 노드가 스트림을 ~0.5초 멈췄다 적용한다.
   LUCID 는 노출이 프레임 주기를 못 넘어서, 긴 노출을 주면 노드가 프레임레이트를 맞춰 내린다 (1 s → 1 Hz).
3. **[녹화]** — **장면**(폴더 이름) · **조도 lux** · **광원** · 조명 칩 · 라벨을 적고 [⏺ 수동 녹화].
   끝나면 그 bag 폴더에 `capture_info.json`, 녹화 시작 시점의 `nodemap/`, 기동 파라미터 `params/` 가 생기고
   자동 진단이 돈다 (frame id 로 유실을 센다).

터미널에서:

```bash
source ~/jsh/env.sh
ros2 topic hz /lucid_tri054/image_raw/metadata
ros2 param set /lucid_tri054/lucid_camera camera.ExposureTime 500000.0
python3 ~/jsh/tools/rawbag.py /mnt/data/jsh_raw/recordings/scene_001_xxx/rec_...
```

## 브라케팅 촬영 (정지 장면 · 노출 단계별 · 두 카메라 → 무손실 DNG)

```bash
~/jsh/bracket
```

화면 순서대로 쓴다.
1. **[카메라 켜기]**: rawgui 의 [센서 기동]과 같은 경로다. 이미 떠 있으면 그대로 쓴다.
   다른 프로그램(예: PTC 촬영)이 카메라를 쥐고 있으면 IP 도 안 바꾸고 띄우지 않는다.
2. **PTC 조건**: 켜지면 노이즈 모델의 녹화 조건과 실제 설정을 대조한다 (아래 목록).
   - 다르면 빨갛게 띄우고 촬영을 막는다. [조건대로 맞추기]가 고칠 수 있는 것은 고친다.
   - 펌웨어 · 해상도처럼 도구가 못 고치는 것은 그렇게 표시한다.
   - 온도가 측정 범위 밖이거나 켠 지 30분이 안 됐으면 주의로 띄운다.
3. 장면 이름(폴더 이름이 된다) · 조도 · 광원 · 메모를 적는다. 저조도면 **BFS 게인**을 올린다.
   BFS 게인은 아날로그라 read noise 가 줄어든다. LUCID 는 0 dB 고정이다 (HDR 합성 뒤 디지털이라 SNR 이득이 없다).
4. **[이 장면 찍기]**를 누르면 다음 순서로 돈다.
   1. 측광: 두 카메라를 같은 노출로 찍는다. 먼저 포화되는 쪽의 가장 밝은 곳(상위 0.5 %)이 포화의 80 %가 되는
      노출을 맨 위 단계로 잡는다.
   2. 거기서 절반씩 9단계, 단계당 3장을 찍는다. 노출은 카메라마다 모델 유효 범위 안에서 자른다
      (BFS 1 s, LUCID 800 ms — 맨 위 단계는 두 카메라 노출이 다를 수 있다. [고급]에서 둘 다 800 ms 로 맞출 수 있다).
   3. 렌즈캡을 씌우라고 안내하고 같은 노출 단계로 dark 를 찍는다.
      BFS 게인이 18 dB 이상이거나 BFS 를 켠 지 30분이 안 됐으면 dark 를 bright 앞에도 찍는다 (dark → bright → dark).
      게인이 높거나 데워지는 중이면 bright 와 dark 사이에 offset 이 흐를 수 있어서다.
   4. 노출 기준 · 단계 · 장수 · dark · 깜빡임 맞춤은 [고급]에서 바꾼다.

저장: `/mnt/data/jsh_raw/stills/scene_NNN_<장면>/`

| 파일 | 내용 |
|------|------|
| `scene.json` | 장면 · 계획(노출 단계) · 카메라(드라이버 파라미터 전부) · 노이즈 모델 참조 · 쌍마다 트리거 시각 · 노출 시작 차 · `dark_shift` · `warnings` |
| `bright/NN_ev±X.X_KK_<bfs\|lucid>.dng` (+ `.json`) | RAW. 같은 json 이 DNG 안 `DNGPrivateData` 에도 들어 있다 |
| `dark_pre/` · `dark/` | 렌즈캡, 같은 노출 단계 (`dark_pre` 는 BFS 게인 ≥ 18 dB 이거나 워밍업 30분 전일 때만) |
| `nodemap/` | 촬영 시작 때 두 카메라 노드맵 |

### 노이즈 모델과 맞추는 것 (2026-10-20 jsh-16 세션과 합의)

노이즈 모델은 `imx490_ptc/calibration/<카메라>_<시리얼>/` 에 있다 (`noise_model.json`, 픽셀 지도, 코드 `imx490_ptc/scripts/noise_model.py`).
도구는 `tools/ptc_model.py` 로 이 패키지를 읽는다.

- **녹화 조건 (측정 때 실제로 쓴 값)**
  - BFS: BayerRG16 · AdcBitDepth Bit12 · BlackLevel 1 % · clamp off · WB auto off, 비율 1.0 ·
    감마 · LUT · 결함보정 · ISP off · 노출 · 게인 auto off · 플립 없음 · 전체 화면 1920×1200 · 펌웨어 2103.0.335.0
  - LUCID: BayerRG24 · HDR 합성 · BlackLevelRaw 256 · 게인 0 dB · WB · CCM · LUT · HDR enhancement · digital clamping ·
    HDR tuning off (채널 게인 19.8/4.2/29.4/4.2 dB) · ADC 12-bit · Rolling · 플립 없음 · 2880×1860 · 펌웨어 1.138.0.0
- **유효 범위**
  - 노출: BFS 24 µs ~ 1 s, LUCID 86 µs ~ 800 ms (모델 파일 `valid_exposure_us` 에서 읽는다)
  - LUCID 1 s 는 쓰지 않는다: 전역 검은 레벨이 두 상태(정상 / −9.7 DN, CFA 모두 같은 양)를 오간다.
    스트림 시작마다 거의 반반으로 정해지고 찍는 도중 정상 → 낮음으로 넘어가기도 해서, 장면 dark 로도 보정이 안 된다.
    300 / 500 / 800 ms 는 반복해도 한 레벨이다 (2026-10-20 실측, jsh-16 합의).
  - BFS 게인: 0/6/10/12/18/24 dB
  - 온도: BFS 53~60 °C, LUCID 44~54 °C
- **DNG 에 넣는 것**
  - 표준 태그: 블랙 = 그 게인 · 노출의 모델 dark offset (CFA 평균), 화이트, 노출, 시리얼
  - `NoiseProfile` (BFS 만): S = α/(white−black), O = 그 노출의 dark 분산/(white−black)²
  - LUCID HDR 은 상수 α 하나로 표현이 안 돼서 `NoiseProfile` 을 비운다
  - private json: 모델 참조(경로 · `noise_model.json` sha256 · built), 그 게인 · 노출의 CFA 별 α · read σ · dark offset · dark 분산 · row σ ·
    코드 간격, LUCID HDR 채널 전환 구간(대략값, 2×2 쿼드 단위)
- **장면 dark 로 하는 전역 보정** (`scene.json` 의 `dark_shift`)
  - 노출 단계 · CFA 마다 offset 이동량(실측 평균 − 모델)과 `k_var`(표본분산 / 모델 분산, hot · noisy 마스크 밖 평균)를 남긴다
  - 이동량이 0.1 · read σ 를 넘으면 `warnings` 에 적는다
  - 픽셀 단위 dark 는 모델 지도에 이 전역 보정을 더해서 쓴다. 몇 장 평균을 픽셀마다 빼면 σ/√n 노이즈가 더해진다.
- **재부팅 감지**: 프레임 메타데이터 `device_uptime_s`(DeviceUptime)가 줄면 장면을 멈춘다. 재부팅하면 설정이 기본값으로 돌아간다.
- **60 Hz 조명 깜빡임**: 8.33 ms 이상 노출을 1/120 s 배수로 맞춘다 (기본으로 켜져 있다).
  그래서 단계 비가 정확히 2배가 아니다. 단계 비는 chunk 노출과 측정 신호로 구한다.
- LUCID 1 ms 미만은 실제 적분시간이 계단식이라, 단계 비를 측정 신호로 구한다.

### 트리거 (2026-10-20 실측)

- **LUCID TRI054S 는 액션 커맨드를 받고 `2 / 프레임레이트 − 노출 + 0.8 ms` 뒤에 노출을 시작한다.**
  - 2 Hz 에서 약 1 s 다. 노드 `TriggerLatencyTime` 값과 같다.
  - 저지연 트리거(`TriggerLatency`)는 이 모델에 구현돼 있지 않다.
  - BFS 는 바로 시작하고 `TriggerDelay` 는 최대 65.5 ms 다.
  - → 엔진이 LUCID 를 먼저 쏘고 지연 차이만큼 뒤에 BFS 를 쏜다. 노출 시작 차(header.stamp, 호스트 시계)는 16쌍 평균 +0.1 ms, 최대 0.6 ms 였다.
- **LUCID 는 노출을 바꾼 뒤 첫 트리거 프레임의 영상이 이전 노출이다** (chunk 노출값은 새 값). 그래서 노출을 바꾸면 한 쌍을 버린다.
- **트리거 모드의 LUCID dark 는 노출 150 ms 까지만 모델과 맞는다** (분산비 60/100/150 ms 1.03/1.04/1.05, 300 ms 부터 1.14).
  그래서 LUCID 노출이 150 ms 를 넘는 단계는 LUCID 를 free-run 으로 받고 BFS 만 트리거한다. 이때 두 카메라는 동시가 아니고,
  쌍의 `mode` 에 그렇게 적힌다.
- 짧은 노출은 쌍마다 약 1.5 s, 노출을 바꾼 단계는 버림 쌍 때문에 약 3 s 걸린다.

주의:
- LUCID 드라이버를 `source ~/jsh/env.sh` 한 셸에서 직접 실행하면 Arena 가 카메라를 못 연다
  ("Invalid security code"). FLIR 쪽 라이브러리 경로가 섞이기 때문이다. GUI · `~/jsh/bracket` 의 기동 경로로 띄운다.
- 두 카메라는 가끔 스스로 재부팅한다. 그러면 ForceIP 가 풀린다 (BFS 10.42.0.124, LUCID 169.254.x).
  [카메라 켜기]가 다시 맞춘다. 촬영 도중에 재부팅되면 장면이 멈추고 `scene.json` 이 "error" 로 끝난다.

## 녹화 데이터

카메라 네임스페이스 `camera_bfs`, `lucid_tri054` 마다:

| 토픽 | 내용 |
|------|------|
| `image_raw` | BFS: `bayer_rggb16` (12-bit 코드 << 4). LUCID: BayerRG24 → `32SC1` (픽셀당 32-bit, 값 < 2^24) |
| `image_raw/metadata` | `FlirMetadata` / `LucidMetadata` — 같은 `header.stamp` |
| `camera_info` | 매 프레임 |
| `image_rgb/compressed` | 미리보기 JPEG (RAW 를 자동 밝기로 그린 것 — **연구용 RGB 아님**) |

메타데이터 (두 타입 공통 이름): `camera_frame_id`, `camera_timestamp_ns`, `exposure_time_us` · `gain_db` ·
`black_level` (**그 프레임의 chunk 값**, 못 읽으면 NaN), `chunk_valid`, `crc_ok`, `pixel_format`, `serial`, `model`,
`firmware_version`, `adc_bit_depth`, `balance_ratio_*`, `lut_enable`, `white_level_dn`, `device_temperature_c`.
FLIR 만: `black_level_clamping_enable`, `defect_correct_static_enable`, `gamma_enable` …
LUCID 만: `hdr_output`, `hdr_channel_analog_gain_db[4]`, `hdr_channel_exposure_time_us[4]`, `black_level_raw`(DN),
`cfa_pattern`, `bits_per_pixel`, `settled`(설정 직후 두 프레임은 false) …

노이즈 모델(`Var = α·S + σ²_dark(T)`)에 쓰는 값:
- α · read noise: BFS 는 `FLIR_control/docs/raw_capture_noise_calibration.md` §2 기준값 (0 dB, BayerRG16 스케일),
  LUCID 는 `imx490_ptc/README.md` 의 채널별 결과 (BayerRG24 스케일, 출력 HDR / HDRChannelN 마다 다름).
- offset · dark 분산은 같은 설정 · 노출로 찍은 dark 에서 (렌즈캡). 블랙 설정이 바뀌면 offset 이 바뀐다.
- 포화: `white_level_dn` (BFS 65516 실측, LUCID 채널 출력 자동, HDR 합성은 아직 0 = 모름).

## 실측해서 정한 것 (2026-10-19)

- **링크**: 두 카메라 링크 제한 합이 ~72 MB/s 를 넘으면 스위치에서 패킷이 버려져 LUCID 프레임(패킷 ~1.1만 개)이
  전부 불완전해졌다 (PC NIC · 소켓 드롭 0). → BFS 20 MB/s (4.3 fps) + LUCID 45 MB/s. LUCID 는 그중 10% 를
  재전송용으로 남겨서 BayerRG24 는 2.0 Hz 가 상한. **더 빠르게 찍으려면 카메라를 PC 의 다른 NIC 포트에 따로**
  꽂으면 된다 (각자 1 GbE). GUI 설정 점검이 합을 경고한다.
- **BFS**: 코드 간격 16, 위상은 BlackLevel 의 디지털 부분 (1 % → 12). 포화 65516. chunk 노출값이 그 프레임 값 (R3).
  BayerRG12p 는 카메라가 16-bit 를 반올림해 보내 블랙 offset 이 +4 DN 달라진다 → RAW 는 BayerRG16.
- **LUCID**: 채널 출력(HDRChannelN)을 BayerRG16 · 12p 로 받으면 min(값, 4095) 로 잘린다 (24-bit 에서는 4095 + 페데스탈
  256 = 4351 까지). chunk 노출값과 영상은 같은 프레임에서 바뀐다.
- **ForceIP 는 임시** — 카메라 전원을 다시 넣으면(가끔은 그냥도) 원래 IP(BFS 10.42.0.124 · LUCID 169.254.x)로 돌아간다.
  GUI 의 IP 자동 맞춤이 기동 전에 다시 맞춘다. 영구로 하려면 카메라 persistent IP 를 바꿔야 한다 (BFS 는 기준 카메라라
  다른 셋업에서도 쓰므로 바꾸지 않았다).

## 아직 남은 것

- LUCID HDR 합성 출력의 포화값 (밝은 광원으로 재야 함).
- 측광(ETTR)은 렌즈캡을 씌운 상태에서만 돌려 봤다. 빛이 있는 장면에서 한 번 확인해야 한다.
- 렌즈 정보(`lens.model` · `lens.f_number`)가 비어 있다. DNG 의 FNumber 태그와 메타데이터에 들어가니 설정 파일에 적는다.
- `net_probe` (GUI 네트워크 표의 IP 별 트래픽) 권한: `sudo setcap cap_net_raw+ep ~/jsh/DM_clipGUI/build/clip_recorder/net_probe`
  (rawgui 가 처음 실행할 때 비밀번호를 묻는다).
- 원격 저장소 푸시: 로컬 브랜치만 있다 (`FLIR_control: feat/raw-capture`, `DM_clipGUI: jsh/raw-lab`, `LUCID_control: main`).
