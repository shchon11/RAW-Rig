# RAW 연구 취득 및 노이즈 캘리브레이션 명세

> RGB→RAW reconstruction 연구용으로 Blackfly S(IMX392)에서 **RAW만** 취득하고,
> 새 카메라의 노이즈 파라미터를 짧은 측정으로 다시 구하기 위한 작업 명세 + 검증 절차.
>
> 기준 캘리브레이션: 최현석, 2026-10-14~15, 카메라 26315393(BFS-PGE-23S3C), Li et al.
> (JIOHS 2016) PTC 방법 + EMVA 1288 Release 4.0 Linear의 노출시간 sweep 방식.
> 원본은 Slack 최현석 DM 스레드 "카메라 저조도 RAW 노이즈 실험"(2026-10-14 21:51).
>
> 작성: 2026-10-19. 아직 실측으로 검증되지 않았다. §5·§6을 진행하면서 실측값을
> 채워 넣고 갱신할 것.

## 1. 결정 사항

| 항목 | 결정 | 이유 |
|------|------|------|
| 저장 대상 | `image_raw`(Bayer 16-bit)만 | RAW가 정답 데이터. 카메라는 한 프레임을 Bayer와 ISP RGB 중 하나로만 내보내므로 내장 ISP를 켜면 RAW를 잃는다 |
| ISP | 나중에 호스트에서 오프라인 처리 (§7) | 같은 RAW에서 여러 ISP(기본 ISP, neural ISP)로 픽셀 정렬된 RGB를 만들 수 있다 |
| 취득 설정 | 기준 캘리브레이션과 같은 설정 (§3) | 그래야 노이즈 파라미터를 그대로 쓸 수 있다 |
| 기본 주행 프로파일 | 건드리지 않는다 | `flir_camera.yaml`은 8대 × 30 Hz JPEG 경로에 맞춰져 있다 (CLAUDE.md). RAW는 별도 프로파일 `config/flir_camera_raw.yaml` |
| 새 카메라 | 짧은 캘리브레이션만 (§6) | 모델 검증은 기준 측정에서 끝났고, 개체마다 다른 값만 다시 잰다 |

## 2. 기준 캘리브레이션 — 재사용하는 것과 다시 재는 것

### 노이즈 모델

```
RAW_i = offset_i(T) + S_i + ε_i
Var(ε_i) = α_c · S_i + σ²_dark,i(T)

S_i      : dark를 뺀 signal [stored DN]
α_c      : CFA 채널 c의 conversion gain [stored DN / e⁻]   (전자수 = S / α)
σ²_dark  : 같은 노출시간 T의 dark 시간 분산 [DN²] (read noise + dark current + 양자화)
```

- 양자화 분산은 σ²_dark에 이미 들어 있다. 따로 더하지 않는다.
- stored DN = BayerRG16에 담긴 값. 12-bit ADC 코드 간격은 16 DN이다.

### 기준값 (26315393, 0 dB, Bit12, BlackLevel 1%, clamp OFF, WB 1.0)

dark 41,800장 + bright 44,800장, 노출시간 24 µs ~ 3.8 s 40단계, 중앙 ROI 256×256.

| CFA | α (DN/e⁻) | read noise (e⁻) | read noise (DN) | offset @24 µs (DN) | dark var @24 µs (DN²) | dark var @19.9 ms (DN²) |
|-----|-----------|-----------------|-----------------|--------------------|-----------------------|-------------------------|
| R   | 5.774 | 2.395 | 13.83 | 633.0 | 191.3 | 205.0 |
| G1  | 5.808 | 2.357 | 13.69 | 632.9 | 187.4 | 200.9 |
| G2  | 5.808 | 2.369 | 13.76 | 633.0 | 189.2 | 201.7 |
| B   | 5.806 | 2.373 | 13.78 | 633.0 | 189.8 | 202.3 |

- shot noise log-log 기울기 0.4998~0.5014 (이론 0.5). 모델 대비 분산 오차 최대 2.8%.
- 노이즈가 거의 white: CFA 사이 상관 약 3%, 같은 CFA 가로 이웃 약 3%, 세로 이웃 0.3% 이하.
- 픽셀별 gain의 실제 편차 약 1% → 채널마다 상수 α 하나로 충분.
- 주행 노출(약 20 ms)에서 dark 분산은 최소 노출보다 약 7% 크다. 그래서 dark는 노출시간을 맞춰 쓴다.
- QE와 입사 광자수는 측정하지 않았다. 공간 고정패턴에는 조명과 렌즈 영향이 섞여 있어서 순수 PRNU로 쓰면 안 된다.

### 새 카메라에서

| 그대로 쓰는 것 (센서 설계 특성) | 다시 재는 것 (개체 특성) |
|------|------|
| 모델 형태, shot 기울기 0.5 | CFA별 α |
| 노이즈 white 특성, 픽셀 gain 균일성 | read noise, dark 분산 vs 노출시간 |
| | offset, hot pixel 위치 (전체 화면) |

짧은 측정으로 충분하다는 근거: 같은 카메라에서 먼저 한 짧은 측정(밝기 16단계 × 100장)이
최종 측정과 이미 1% 이내로 일치했다.

| CFA | 짧은 측정 α | 최종 α | 차이 | 짧은 측정 read noise | 최종 read noise |
|-----|-------------|--------|------|----------------------|-----------------|
| R   | 5.790 | 5.774 | 0.3% | 2.387 | 2.395 |
| G1  | 5.849 | 5.808 | 0.7% | 2.339 | 2.357 |
| G2  | 5.851 | 5.808 | 0.7% | 2.350 | 2.369 |
| B   | 5.786 | 5.806 | 0.3% | 2.378 | 2.373 |

## 3. 현재 설정이 RAW 연구 취득과 어긋나는 지점

`config/flir_camera.yaml`을 복사해 `config/flir_camera_raw.yaml`을 만들고 아래만 바꾼다.
`install(DIRECTORY config ...)`로 설치되므로 CMake 수정은 필요 없다.

| # | 위치 | 현재 값 | RAW 프로파일 | 이유 |
|---|------|---------|--------------|------|
| 1 | `flir_camera.yaml:52` | `pixel_format: "BayerRG8"` | `"BayerRG12p"` (R1 통과 시) 또는 `"BayerRG16"` | 8-bit에서는 RAW 정보가 잘린다 |
| 2 | `flir_camera.yaml:84` | `publish_raw: false` | `true` | |
| 3 | `flir_camera.yaml:87` | `publish_rgb_compressed: true` | `false` | 호스트 debayer CPU 예산이 풀린다. 미리보기가 필요할 때만 켠다 |
| 4 | `flir_camera.yaml:184` | `camera.GainDB_Val: 10.0` | `0.0` | 기준 캘리브레이션 조건 |
| 5 | (미설정) | 카메라 기본값 | `camera.AdcBitDepth: "Bit12"` | 기존 3DRAW 데이터는 G 코드 간격이 64 DN이었다(10-bit로 추정). 0 dB 10-bit면 코드 한 칸이 약 11 e⁻로 read noise보다 크다 |
| 6 | (미설정) | 카메라 기본값 | `camera.BlackLevelClampingEnable: false` | 3DRAW는 black 근처에서 잘려 야간 G 픽셀 최대 45%가 최저 코드에 몰렸다 |
| 7 | (미설정) | 카메라 기본값 | BlackLevel 1.0 % | selector 노드라 §4.1 코드로 설정 |
| 8 | (미설정) | 카메라 기본값 | `camera.BalanceWhiteAuto: "Off"`, BalanceRatio R/B = 1.0 | 3DRAW는 프레임마다 다른 WB가 RAW에 곱해져 있었다. BalanceRatio는 selector 노드 → §4.1 |
| 9 | (미설정) | 카메라 기본값 | `camera.GammaEnable: false`, `camera.LUTEnable: false`, `camera.DefectCorrectStaticEnable: false` | RAW에 아무 처리도 구워지지 않게 한다. hot pixel은 §6에서 직접 잰다 |
| 10 | `flir_camera.yaml:315` | `ptp_action.rate_hz: 30.0` | 12p는 `20.0`, 16-bit는 `15.0` | 아래 대역폭 표 |
| 11 | `scripts/all_sensors_bagging.sh:34`, `scripts/camera_bagging.sh` | `camera_regex`에 `image_raw` 없음 | RAW 모드일 때 `image_raw` 추가 (예: `FLIR_RECORD_RAW=1`). `image_raw/metadata`는 이미 기록된다 | 지금은 RGB JPEG와 metadata만 기록된다 |
| 12 | `flir_spinnaker_camera_node.cpp:3192` | 노출시간과 gain을 발행 시점의 노드 값으로 읽음 | chunk data (§4.2) | auto 모드나 설정 변경 직후에는 그 프레임의 실제 값이 아닐 수 있다 |

실행:

```bash
ros2 launch flir_spinnaker_camera multicam.launch.py \
  params_file:=$(ros2 pkg prefix flir_spinnaker_camera)/share/flir_spinnaker_camera/config/flir_camera_raw.yaml
```

### 대역폭과 저장 용량

카메라당 1 GbE 링크, `camera.DeviceLinkThroughputLimit: 100000000`(100 MB/s, 실측 상한 약 110 MB/s, `flir_camera.yaml:338-343`).

| 포맷 | 프레임 (전송) | 30 Hz | 20 Hz | 15 Hz |
|------|---------------|-------|-------|-------|
| BayerRG8 (현재) | 2.30 MB | 69 MB/s | 46 MB/s | 35 MB/s |
| BayerRG12p | 3.46 MB | **104 MB/s ✗** | 69 MB/s | 52 MB/s |
| BayerRG16 | 4.61 MB | **138 MB/s ✗** | 92 MB/s (여유 부족) | 69 MB/s |

- 권장: **BayerRG12p @ 20 Hz** (R1 통과 시), 아니면 **BayerRG16 @ 15 Hz**.
- 노출시간은 트리거 주기보다 짧아야 한다(`flir_camera.yaml:166`): 20 Hz면 50 ms, 15 Hz면 66 ms.
- 드라이버는 12p를 받아 호스트에서 BayerRG16으로 풀어 발행한다(`PrepareRawImage`, L2715). 그래서 bag에는 포맷과 관계없이 메시지당 4.61 MB가 쌓인다.

| 기록 | 20 Hz | 15 Hz |
|------|-------|-------|
| 카메라 1대 | 92 MB/s, 332 GB/h | 69 MB/s, 249 GB/h |
| 8대 | 737 MB/s, 2.65 TB/h | 553 MB/s, 1.99 TB/h |

db3(sqlite) 기록이 이 속도를 버티는지 R2에서 확인한다. 못 버티면 카메라 수나 rate를 줄인다.

새 카메라가 USB3 모델(BFS-U3-23S3C)이면 이 절의 GigE·PTP 항목은 해당되지 않는다. 대역폭 한계도 USB3 기준으로 다시 잡아야 한다. §6 노이즈 절차는 같다.

## 4. 코드 변경 명세

### 4.1 selector에 묶인 노드 설정

드라이버는 쓰기 가능한 노드를 전부 `camera.<노드명>`으로 노출한다(`RegisterWritableControlParameters`, L1186).
하지만 selector에 묶인 노드는 등록 시점에 선택돼 있던 항목 하나만 잡힌다. 그래서 YAML 한 줄로는 설정이 안 된다.
다음 파라미터를 추가하고, 일반 `camera.*` 적용이 끝난 뒤 `BeginAcquisition()` 전에 적용한다.
기존 `RequireEnumeration`/`TryEnumeration` 헬퍼를 쓴다.

| 파라미터 | 동작 | 비고 |
|----------|------|------|
| `raw.balance_ratio_red`, `raw.balance_ratio_blue` (기본 NaN = 그대로 둠) | `BalanceRatioSelector=Red/Blue` → `BalanceRatio=값` | `BalanceWhiteAuto=Off`가 먼저 적용돼 있어야 쓰기 가능 |
| `raw.black_level_percent` (기본 NaN) | `BlackLevelSelector`가 있으면 `All` → `BlackLevel=값` | 단위는 %. 기준 측정은 0.99999 |

적용 후 다시 읽어서 로그로 남기고, 같은 값을 `image_raw/metadata`(§4.3)에 담는다.

### 4.2 chunk data 활성화

| 파라미터 | 기본값 |
|----------|--------|
| `chunk.enable` | `false` (RAW 프로파일은 `true`) |
| `chunk.selectors` | `["ExposureTime", "Gain", "BlackLevel", "CRC"]` |

```cpp
// BeginAcquisition() 전에. ChunkModeActive는 acquisition 중에는 쓰기 불가.
SetBooleanByName(node_map, "ChunkModeActive", true);   // 기존 헬퍼 (L249)
for (const auto & name : chunk_selectors_) {
  if (!TryEnumeration(node_map, "ChunkSelector", name, context)) {
    RCLCPP_WARN(get_logger(), "chunk '%s' is not supported by this camera", name.c_str());
    continue;
  }
  SetBooleanByName(node_map, "ChunkEnable", true);
}
```

프레임마다 (Spinnaker 4.x `ChunkData`, 빌드할 때 헤더로 메서드 이름 확인):

```cpp
const Spinnaker::ChunkData & chunk = image->GetChunkData();
msg.exposure_time_us = chunk.GetExposureTime();
msg.gain_db          = chunk.GetGain();
msg.black_level      = chunk.GetBlackLevel();
msg.crc_ok = image->HasCRC() ? image->CheckCRC() : false;
```

`camera_frame_id`와 `camera_timestamp_ns`는 지금처럼 `image->GetFrameID()`와 `image->GetTimeStamp()`로 읽는다.
이 둘은 원래 프레임 버퍼에서 나오는 값이라 이미 그 프레임의 값이다.

`chunk.enable: true`인데 chunk를 못 읽은 프레임은 `chunk_valid=false`로 표시하고 값을 NaN으로 둔다.
노드 값으로 대신 채우지 않는다. 대신 채우면 프레임과 어긋난 값이 섞여도 구분할 수 없다.
`chunk.enable: false`(기본 주행 프로파일)면 지금처럼 노드 값을 쓰고 `chunk_valid=false`다.

### 4.3 기존 `FlirMetadata` 확장 (`image_raw/metadata` 그대로)

새 메시지나 토픽은 만들지 않는다. `msg/FlirMetadata.msg`에 필드를 추가하고 다시 빌드한다.
기존 필드(`exposure_time_us`, `gain_db`, `black_level`, `pixel_format`, `encoding`, `gamma_enable`,
`balance_white_auto` 등)는 이름과 의미를 유지한다. 바뀌는 건 chunk가 켜져 있을 때 값의 출처뿐이다(§4.2).

추가할 필드:

```
# 프레임별
bool    chunk_valid               # exposure_time_us / gain_db / black_level이 chunk에서 왔는지
bool    crc_ok

# 세션 고정 (설정 직후 1회 읽어 캐시, 매 메시지에 복사)
string  serial                    # DeviceSerialNumber
string  model                     # DeviceModelName
string  firmware_version          # DeviceFirmwareVersion
string  adc_bit_depth             # AdcBitDepth
bool    black_level_clamping_enable
float64 balance_ratio_red
float64 balance_ratio_blue
bool    lut_enable
bool    defect_correct_static_enable
bool    isp_enable
uint32  white_level_dn            # R1에서 실측한 포화 코드값 (파라미터로 입력)

# 천천히 변함
float64 device_temperature_c      # 1 Hz 타이머로 읽은 마지막 값, 실패 시 NaN
```

- `DeviceTemperature`는 grab 루프가 아니라 1 Hz 타이머에서 읽는다.
- 렌즈(모델, 수동 조리개 f-number)는 카메라에서 읽을 수 없다. `multicam_cameras.yaml`의 카메라 항목에
  `lens_model`, `f_number`를 추가하고 노드맵 덤프(§4.4)에 같이 넣는다.
- 메시지 정의가 바뀌므로 `scripts/check_multicam_sync.py`처럼 이 타입을 쓰는 쪽도 같은 빌드로 실행한다.

### 4.4 노드맵 덤프 → JSON 파일

토픽이 아니라 파일로 남긴다. 파라미터 `nodemap_dump_dir`(빈 문자열이면 끔, RAW 프로파일은 `bags/nodemap`)를 둔다.
설정이 끝난 직후 한 번 `<dir>/<serial>_<YYYYmmdd_HHMMSS>_nodemap.json`을 쓴다.
파일 이름의 시각으로 bag과 짝을 맞춘다.

- 읽을 수 있는 노드 전부 (이름 → 값 문자열).
- selector 노드는 항목별로 펼친다: `BalanceRatioSelector` {Red, Blue},
  `ColorTransformationValueSelector` {Gain00 … Gain22}, `BlackLevelSelector`, `GainSelector`.
  ColorTransformation 계수는 카메라 내장 CCM이다. 나중에 호스트 ISP에서 카메라 색을 흉내 낼 때 쓴다(§7).
- 렌즈 정보(§4.3), 드라이버 git commit, 프로파일 YAML 경로.

## 5. 검증 절차 — RAW 취득 (R0~R4)

### R0 — 설정 반영

노드맵 JSON(§4.4)과 `image_raw/metadata`의 세션 필드가 프로파일과 같은지 확인한다: `PixelFormat`, `AdcBitDepth=Bit12`,
`BlackLevelClampingEnable=false`, `BlackLevel≈1.0`, `BalanceWhiteAuto=Off`, `BalanceRatio` R/B = 1.0,
`Gain=0`, `GammaEnable/LUTEnable/DefectCorrectStaticEnable=false`.

### R1 — 값의 스케일, 코드 간격, 포화값

정지 장면에서 다음을 확인한다.

1. **12p 언팩 스케일:** 같은 장면을 BayerRG16으로 50장, BayerRG12p로 50장 찍는다.
   CFA별 평균 비율이 1.00 ± 0.5%면 통과다. 약 1/16이면 언팩 결과가 12-bit 스케일이라는 뜻이다.
   그때는 BayerRG16 @ 15 Hz를 쓴다. α가 BayerRG16 스케일 기준이기 때문이다.
   PySpin 오프라인 테스트로 확인하려 했지만 `Image.Create(BayerRG12p)`에서 프로세스가 죽어서 확인하지 못했다(2026-10-19).
2. **코드 간격:** CFA별 고유값의 간격이 16 DN이어야 한다. 64면 `AdcBitDepth`가 반영되지 않은 것이다.
   기준 카메라의 코드 위상은 mod 16 = 12였다.
3. **clamp OFF:** 렌즈캡 프레임의 최솟값이 0보다 크고, 최저 코드에 몰린 픽셀이 0.1% 미만이어야 한다.
   offset은 약 633 DN.
4. **포화값:** 일부를 포화시켜 최대 코드값을 읽고 `white_level_dn`에 넣는다.
   기준 카메라는 선언값 65520이었다. 3DRAW에서는 65532가 관측됐다.

### R2 — 프레임 드롭과 기록 처리량

선택한 rate로 5분 기록한다. 다음을 모두 만족해야 한다.
- 카메라마다 `camera_frame_id`가 끊김 없이 이어진다.
- `crc_ok`가 전부 true다.
- 메시지 rate가 `ptp_action.rate_hz`와 같다.
- bag 크기가 §3 표와 맞는다.

`buffer_handling_mode: NewestOnly`는 밀리는 프레임을 조용히 버린다. 그래서 frame id 연속성으로만 드롭을 알 수 있다.

### R3 — chunk 값이 그 프레임의 값인지

`ExposureAuto=Off`로 스트리밍하면서 `ros2 param set`으로 `camera.ExposureTime`을 5000 → 10000으로 바꾼다.
화면 평균이 처음 두 배가 되는 프레임과 `exposure_time_us=10000`이 처음 나오는 프레임이 같아야 한다.

### R4 — bag 재생

다음을 확인한다.
- `image_raw/metadata`와 `image_raw`의 `header.stamp`가 1:1로 대응한다.
- 추가한 필드가 채워져 있다 (`chunk_valid=true`, serial, 온도 등).
- 녹화 시작 시각에 맞는 노드맵 JSON이 `bags/nodemap`에 있다.
- 다시 빌드한 환경에서 `scripts/check_multicam_sync.py`가 동작한다.

## 6. 새 카메라 노이즈 캘리브레이션 (짧은 버전)

### 6.1 준비

- **설정:** §3 RAW 프로파일과 같게(0 dB, Bit12, BlackLevel 1%, clamp OFF, WB 1.0, gamma/LUT/defect OFF).
- **카메라 점유:** 카메라 노드는 내려둔다. 측정 도구가 카메라를 독점해야 한다.
- **워밍업:** 측정 전에 30분 스트리밍한다. offset이 온도에 따라 흐른다.
  `DeviceTemperature`를 시작과 끝에 기록한다.
- **장면:** 흰 종이나 diffuser를 카메라·렌즈·조리개를 고정한 채 비춘다. 조명은 DC LED가 좋다.
  PWM 조명이면 §6.4 flicker 검사가 중요하다.
- **ROI:** 전체 1920×1200을 받아 중앙 256×256 (x=832, y=472)을 그대로 잘라 저장한다.
  기준과 같은 위치이고, 짝수 offset이라 RGGB 위상이 유지된다. CFA당 128×128 = 16,384 픽셀.

### 6.2 노출시간 단계

1. G의 ROI 평균이 white의 약 80%(≈52,000 DN)가 되는 노출시간 T_sat를 찾는다.
2. T_sat이 20 ms보다 짧으면 조리개나 ND로 빛을 줄여 T_sat을 20~100 ms에 맞춘다.
3. T_min = max(카메라 최소 노출, T_sat / 3000)부터 T_sat까지 로그 간격 16단계로 잡는다.
   약 3 e⁻ ~ 9,000 e⁻ 범위가 된다.

### 6.3 촬영

| 세트 | 조건 | 장수 |
|------|------|------|
| bright | 16단계 | 단계당 100 (겹치지 않는 50쌍) |
| matched dark | 렌즈캡, 같은 16개 노출시간, 같은 gain | 단계당 100 |
| 전체 화면 dark | 렌즈캡, 주행 노출시간(예: 18 ms), 전체 1920×1200 | 300 |
| drift 확인 (선택) | 마지막에 첫 bright 단계를 한 번 더 | 100 |

- 총 약 3,500장이다. 워밍업을 빼면 촬영은 10분 안팎이다.
- 온도가 같도록 bright sweep 직후 바로 dark sweep을 찍는다.
- 단계당 정밀도: 채널 분산 추정의 상대 표준오차는 √(2 / (16,384 × 50)) ≈ 0.16%다.
- 저장 위치는 `calibration/captures/noise_<serial>_<YYYYMMDD>/` (gitignored).
  - 단계별 uint16 ROI 스택
  - 프레임별 chunk 값 (노출시간, gain, black level, frame id, timestamp, CRC)
  - 노드맵 JSON
  - 시작과 끝의 온도

### 6.4 분석 전 QA

- **무결성:** 모든 프레임의 CRC가 통과하고 frame id가 연속인지 본다.
  chunk 노출시간이 요청값과 같고 gain이 0인지 확인한다.
- **조명 안정성:** 겹치지 않는 쌍 (A, B)마다 |mean(A) − mean(B)| / mean ≤ 0.1%를 요구하고, 넘는 쌍은 버린다.
  기준 측정과 같은 기준이다. 절반 넘게 버려지면 DC 조명으로 다시 찍는다.
- **포화:** ROI에서 `white_level_dn − 64` 이상인 값이 1% 이상인 단계는 피팅에서 뺀다.

### 6.5 분석 (CFA 채널 c마다, ROI 안의 해당 채널 픽셀 i)

```
1. offset      o_i(T)  = mean_t dark_i,t(T)
2. dark 분산   v_d(T)  = mean_i  mean_pairs (A_i − B_i)² / 2        # 겹치지 않는 쌍: (0,1), (2,3), …
3. signal      S(T)    = mean_i [ mean_t bright_i,t(T) − o_i(T) ]
4. bright 분산 v_b(T)  = 2와 같은 방식, bright 프레임으로
5. shot 분산   v_s(T)  = v_b(T) − v_d(T)
6. α_c         원점을 지나는 가중 최소제곱  v_s = α · S
               사용 단계: 포화 아님, 10 e⁻ ≤ S ≤ 0.7 · (white − offset)   (전자수 환산은 기준 α 5.8로)
               가중치 w = N_pix · N_pairs / (2 (v_b² + v_d²))
               α = Σ w S v_s / Σ w S²,   SE(α) = 1 / √(Σ w S²)
7. read noise  σ_r,c = √v_d(T_min) / α_c   [e⁻]
8. 검사        shot 기울기: 10~3000 e⁻에서 log √v_s vs log S의 기울기
               모델 오차:   |v_b − (α S + v_d)| / v_b, 사용한 모든 단계에서
9. 경험적 SE   쌍을 짝수/홀수 번째로 나눠 α를 각각 구한다 → SE ≈ |α₀ − α₁| / 2
```

v_s는 S에 선형이다. 그래서 ROI 안에 조명 불균일이 있어도 채널 평균끼리 맞추면 그대로 성립한다.

전체 화면 dark에서 구하는 것:
- 픽셀별 평균 → offset 맵
- 픽셀별 표준편차 → noise 맵
- hot pixel = (평균 − 중앙값) > 5 × 중앙 표준편차
- noisy pixel = 표준편차 > 2 × 중앙 표준편차

### 6.6 판정 — 기준 카메라와 비교하는 sanity check

새 카메라의 값은 측정한 값 그대로 쓴다. 아래는 설정이나 측정이 틀렸는지 알아보는 기준이다.

| 항목 | 기대 범위 | 벗어나면 의심할 것 |
|------|-----------|-------------------|
| α_c | 5.5 ~ 6.1 DN/e⁻ (기준 ±5%) | gain ≠ 0 dB, AdcBitDepth, 12p 언팩 스케일 (R1) |
| G1 vs G2 α 차이 | ≤ 1% | ROI 위상, 장면 |
| read noise | 2.1 ~ 2.6 e⁻ (기준 ±10%) | AdcBitDepth, clamp, 온도 |
| shot 기울기 | 0.50 ± 0.01 | 조명 flicker, 포화 단계 포함 |
| 모델 오차 | ≤ 3% | flicker, 포화, dark 노출시간 불일치 |
| offset @T_min | 약 633 DN | BlackLevel 설정 |

### 6.7 산출물

`calibration/noise/<serial>.json` (커밋 대상):

```json
{
  "serial": "", "model": "", "sensor": "IMX392", "date": "YYYY-MM-DD",
  "settings": {"gain_db": 0.0, "adc_bit_depth": "Bit12", "pixel_format": "BayerRG16",
               "black_level_percent": 1.0, "black_level_clamping": false,
               "balance_ratio_red": 1.0, "balance_ratio_blue": 1.0, "white_level_dn": 0},
  "roi": [832, 472, 256, 256],
  "temperature_c": {"start": 0.0, "end": 0.0},
  "units": {"alpha": "stored DN per e-", "variance": "DN^2", "exposure": "us"},
  "cfa": {
    "R":  {"alpha": 0.0, "alpha_se": 0.0, "read_sigma_e": 0.0, "offset_dn_t_min": 0.0},
    "G1": {}, "G2": {}, "B": {}
  },
  "dark_by_exposure": [
    {"exposure_us": 0.0, "offset_dn": {"R": 0.0, "G1": 0.0, "G2": 0.0, "B": 0.0},
     "variance_dn2": {"R": 0.0, "G1": 0.0, "G2": 0.0, "B": 0.0}}
  ],
  "checks": {"shot_slope": {}, "max_model_residual": 0.0, "g1_g2_alpha_diff": 0.0},
  "hot_pixels": [[0, 0]],
  "reference": "26315393 (최현석, 2026-10-15)"
}
```

전체 화면 offset/noise 맵은 크기 때문에 캡처 폴더에 npz로만 둔다. 커밋하는 것은 hot pixel 좌표 목록뿐이다.

### 6.8 도구 (구현 예정)

`scripts/`의 다른 독립 스크립트처럼 ROS 노드가 아니라 PySpin으로 카메라를 직접 연다.
단계별 장수, 노출 설정과 chunk 확인, 렌즈캡 안내를 정확하게 제어하려면 이쪽이 낫다.
PySpin(spinnaker_python 4.3)은 `~/projects/DM/FLIR-Multicamera-Calibration/.venv`에 있다.

- `scripts/noise_calib_capture.py --serial <S> --out calibration/captures/noise_<S>_<date>`
  - §6.1 설정 적용, 워밍업, T_sat 탐색, §6.3 촬영
  - 렌즈캡 단계에서는 사용자 입력을 기다린다
- `scripts/noise_calib_analyze.py <capture_dir>`
  - §6.4 QA, §6.5 분석, §6.6 판정표 출력
  - `calibration/noise/<serial>.json` 생성

## 7. 오프라인 ISP (나중에)

2026-10-19 PySpin 4.3 오프라인 테스트로 확인했다. 카메라 없이, 저장된 BayerRG16 프레임
(3DRAW scene_020)으로 `PySpin.Image.Create(...)` → `ImageProcessor.Convert(...)`가 동작했다.

| 호스트 Spinnaker로 되는 것 | 안 되는 것 (카메라 FPGA ISP 기능) |
|---------------------------|-----------------------------------|
| 디모자이크 10종: NEAREST_NEIGHBOR(_AVG), BILINEAR, EDGE_SENSING, HQ_LINEAR, RIGOROUS, IPP, DIRECTIONAL_FILTER, WEIGHTED_DIRECTIONAL_FILTER | WB, CCM, tone, denoise, sharpening |
| RGB8 / RGB16 변환, `ApplyGamma` | |

- **RGB8 변환:** 16-bit 값의 상위 8비트만 남는다(선형). 테스트 프레임은 G 중앙값 1472 → 5였다.
  그래서 RGB16으로 받아 WB, 밝기, gamma를 직접 적용한다.
- **실시간 `image_rgb/compressed` 재현:** 드라이버와 같게 `color_processing: ipp` → RGB8 → JPEG 품질 80으로 처리하면 된다.
- **WB, CCM, tone:** 자체 파이프라인(예: isp_study `rawisp` Ladder)이나 rawpy로 한다.
  카메라 색을 맞추려면 노드맵의 ColorTransformation 계수(§4.4)를 쓰거나 컬러체커를 한 번 찍는다.
- **오프라인 입력 형식:** 패킹된 12p 버퍼로 `Image.Create`를 부르면 프로세스가 죽었다.
  오프라인 처리는 16-bit로 풀린 RAW로 한다. ROS bag은 원래 16-bit다.

## 8. 미결 사항

- **새 카메라 모델과 인터페이스:** BFS-PGE-23S3C인가 BFS-U3-23S3C인가. 이에 따라 §3 대역폭 절이 바뀐다.
  주행 rig `multicam_cameras.yaml`의 8대 중에는 기준 카메라 26315393이 없다.
- **노출 정책:** 현재처럼 18 ms 고정으로 갈지 auto로 갈지. auto면 chunk가 필수이고
  `AutoExposureExposureTimeUpperLimit`를 트리거 주기 아래로 둔다. 노이즈 모델은 노출시간과 무관하게 쓸 수 있다.
  dark 측정이 24 µs~3.8 s를 덮는다.
- **0 dB의 결과:**
  - 같은 광량에서 DN 값이 현재 10 dB보다 3.16배 작아 화면이 어둡다.
  - dynamic range는 가장 넓다(헤드라이트 포화가 줄어든다).
  - 코드 한 칸 ≈ 2.8 e⁻로 극저조도 계조가 거칠다. 이는 측정된 read noise에 이미 포함돼 있다.
  - 나중에 gain을 올리면 그 gain마다 §6을 다시 한다. α가 선형 gain에 비례할 것으로 예상하지만 확인이 필요하다.
- **온도:** 기준 데이터에서 긴 노출 dark의 drift는 시간·온도와 분리되지 않았다.
- **기존 3DRAW(/data/datasets/3DRAW):** 위 노이즈 모델을 그대로 적용할 수 없다.
  10-bit, clamp ON, WB가 곱해진 상태이고 프레임별 gain과 노출 기록이 없다.
  - R/B 코드 간격 ÷ 64로 프레임별 WB 배율은 역산할 수 있다(예: scene_030 R 1.44, B 2.44).
  - RGB는 카메라 ISP가 아니라 rawpy(AHD, auto WB, auto-bright, black=64)로 만든 것이다.

## 9. 구현 메모 (브랜치 `feat/raw-capture`, 2026-10-19)

§3–§4 를 구현했다. 명세와 다르게 한 것과 덧붙인 것만 적는다.

| 명세 | 구현 |
|------|------|
| `chunk.enable`, `chunk.selectors` (§4.2) | master 에 이미 있던 `chunk_data.enable`(Timestamp · FrameID · ExposureTime)을 넓혔다. 새 파라미터 `chunk_data.selectors` (기본값은 예전과 같은 셋, RAW 프로파일은 `+ Gain, BlackLevel, CRC`). 주행 프로파일 동작은 그대로다 |
| chunk 를 못 읽은 프레임은 NaN | 그대로. 단 chunk 를 켠 항목만이다. 카메라에 그 chunk 가 없으면 기동 때 WARN 을 남기고 노드 값을 쓴다 (그 카메라는 `chunk_valid` 가 늘 false) |
| `raw.*` 는 `BeginAcquisition()` 전에 (§4.1) | `VerifyControlOverridesTookEffect()` **뒤에** 적용한다. 같은 레지스터의 `camera.*` 키(`camera.BlackLevel`, `camera.BalanceRatioRed_Val` …)가 있으면 그 read-back 이 `raw.*` 값을 덮어쓰기 때문이다. 같이 있으면 WARN, 결과가 다르면 GUI 가 주황으로 잡는 ERROR (`not in effect after startup`) |
| 세션 고정 필드 (§4.3) | 실행 중 파라미터를 바꾸면(GUI ISP 튜닝) 다시 읽는다. `BalanceWhiteAuto` 가 Off 가 아니면 WB 비율은 1 Hz 로 다시 읽는다. 예전에 프레임마다 레지스터를 읽던 `exposure_auto` · `gain_auto` · `gamma*` · `balance_white_auto` · `acquisition_frame_rate*` 도 이 캐시에서 채운다 (획득 스레드의 GVCP 왕복이 줄어든다) |
| 노드맵 덤프 (§4.4) | `<serial>_latest_nodemap.json` 도 같이 쓴다. 실행 중 설정을 바꾸면 2초 뒤 다시 덤프한다(`reason: "parameter change"`). `driver_git` 은 빌드 때 `git describe`, `profile_path` 는 런치가 넘기는 params 파일, 렌즈는 인벤토리의 `lens_model` · `f_number` |
| 12p 언팩 | 드라이버가 직접 푼다 (`value << 4`). `ImageProcessor::Convert` 의 12p 스케일은 오프라인으로 확인할 수 없었다 (`Image::Create(BayerRG12p)` 가 0 바이트 이미지를 만들고 죽음, C++ 도 같음). 결과는 아래 R1 |
| (추가) `rgb_compressed_mode: preview` | 16-bit RAW 를 ImageProcessor 로 RGB8 로 만들면 상위 8비트만 남아 저조도가 검게 나온다. GUI 미리보기용으로 RAW 를 절반 해상도로 디모자이크 · 블랙 제거 · 자동 밝기(상한 `preview.max_gain`) · 감마로 그린다 (`src/raw_preview.hpp`). 연구용 RGB 가 아니다 |
| (추가) `FLIR_RECORD_RAW=1` | `camera_bagging.sh`, `all_sensors_bagging.sh` 가 `image_raw` 도 기록한다 (§3 #11) |

랩 리그(`~/jsh`, BFS 26315393 + LUCID TRI054S, enp5s0 1 GbE 공유, MTU 1500)에서는 `GevSCPSPacketSize` 1400,
`DeviceLinkThroughputLimit` 45 MB/s, 자유 실행(`AcquisitionFrameRateEnable: false`)으로 돌린다 (`config/flir_camera_raw.yaml` 머리말).

### R0 · R1 실측 (2026-10-19, 26315393, 렌즈 가림, 20 ms, 0 dB, BayerRG16)

| 항목 | 결과 |
|------|------|
| R0 설정 반영 | 노드맵 덤프 · metadata 모두 BayerRG16, Bit12, clamp OFF, BlackLevel 0.999999, WB Off · 1.0/1.0, gamma · LUT · defect OFF, Gain 0 |
| chunk · CRC | 56/56 프레임 `chunk_valid` · `crc_ok`, frame id 연속, 45 MB/s 에서 9.3 Hz |
| R1.2 코드 간격 | 네 CFA 모두 16 DN. 위상 mod 16 = 12 (기준 카메라와 같음) |
| 위상과 BlackLevel | 위상은 BlackLevel 의 디지털 부분(소수 코드)에서 온다: 0 % → 0, 0.5 % → 4, 1 % → 12, 1.5 % → 4, 2 % → 12, 3 % → 12. BlackLevel 은 Analog(0.976562 %) + Digital(0.0234365 %)로 나뉘어 들어간다 (덤프의 `selected.BlackLevelSelector`) |
| R1.3 clamp OFF | 최솟값 492 > 0, 최저 코드 비율 < 0.01 %. offset 634.7 DN (기준 633.0 @24 µs) |
| BlackLevel 0 % | 평균 16 DN — dark 분포가 0 에서 잘린다 (1 % 를 쓰는 이유) |
| R1.1 12p | 위상이 0 이 되고 평균이 +3.9 DN. 카메라가 16-bit 값(16k+12)을 12-bit 로 **반올림**해 보낸다 (k+1). 정보 손실은 없지만 블랙 offset 이 BlackLevel 에 따라 달라진다(1 % 에서 +4 DN ≈ 0.7 e⁻). 그래서 RAW 프로파일 기본값은 BayerRG16. 12p 를 쓰면 dark 도 12p 로 찍는다 |
| R1.4 포화값 | **65516** = 16·4094 + 12, 네 CFA 모두 (200 ms, 장면 일부 포화, 15~36만 픽셀이 그 값). 선언값 65520 이 아니다. RAW 프로파일 `raw.white_level_dn: 65516` |
| R3 chunk 노출값 | 5 → 10 ms: metadata 의 `exposure_time_us` 가 9999 로 처음 바뀐 프레임에서 영상 평균도 바로 뛰었다 (조명 깜빡임으로 프레임마다 ±15% 출렁여 비율은 1.74). 같은 프레임이다 — 통과 |
| 링크 (랩) | 같은 스위치의 LUCID 와 합이 ~72 MB/s 를 넘으면 LUCID 프레임이 전부 불완전 (스위치에서 버려짐, PC NIC · 소켓 드롭 0). BFS 25 + LUCID 50 실패, 22 + 50 정상 → BFS 20 MB/s (4.3 fps). `camera.DeviceLinkThroughputLimit` 는 실행 중에 바꿀 수 있다 |
