# IMX490 (LUCID TRI054S-C) PTC 측정

LUCID TRI054S-C(IMX490, SN 254600231)의 채널별 conversion gain·read noise·선형성을 재는 도구.
절차는 FLIR_control `docs/raw_capture_noise_calibration.md` §6(브랜치 `docs/raw-capture-noise-calib`)를
IMX490의 HDR 채널 구조에 맞게 바꾼 것이다.

## 환경

- Arena SDK x64 1.0.10.11: `~/ArenaSDK_Linux_x64` (사용자 설치, `/etc/ld.so.conf.d` 미등록)
- venv: `.venv` (`arena_api` 2.10.0, numpy, matplotlib). venv 안의 `arena_api/arena_api_config.py`가 SDK 경로를 직접 가리킨다
- 매번 `source env.sh`

ROS 드라이버(`../lucid_vision_driver`)는 빌드할 때 `/etc/ld.so.conf.d/Arena_SDK.conf`가 필요하다.
`cd ~/ArenaSDK_Linux_x64 && sudo sh Arena_SDK_Linux_x64_AVMP.conf`로 만든다(apt 설치도 같이 한다).
이 드라이버는 모든 프레임을 BGR8로 변환하므로 RAW 측정에는 쓰지 않는다.

## 네트워크

`enp5s0`(192.168.100.100/24)에 카메라 두 대가 물려 있다. 둘 다 원래 IP가 다른 대역이라 ForceIP로 옮겨서 쓴다.
ForceIP는 임시라 **카메라 전원이 꺼졌다 켜지면 다시 해야 한다.**

| 카메라 | 원래 IP | ForceIP |
|--------|---------|---------|
| LUCID TRI054S-C 254600231 | 169.254.180.201 (LLA) | 192.168.100.201 |
| FLIR BFS-PGE-23S3C 26315393 (기준 카메라) | 10.42.0.124 (persistent) | 192.168.100.202 |

```bash
python3 scripts/gvcp_ip.py list
python3 scripts/gvcp_ip.py force 254600231 192.168.100.201
python3 scripts/gvcp_ip.py force 26315393 192.168.100.202
```

## 스크립트

| 스크립트 | 하는 일 |
|----------|---------|
| `scripts/gvcp_ip.py` | SDK 없이 GigE Vision 검색 / 임시 ForceIP |
| `scripts/lucid_dump.py` | 노드맵 전체를 JSON으로 덤프. 읽기만 한다 |
| `scripts/dark_probe.py` | 렌즈캡 상태에서 출력별 offset·read noise·코드 간격·클리핑 측정 |
| `scripts/ptc_capture.py` | PTC 촬영 (`bright` → `dark` 순서) |
| `scripts/ptc_analyze.py` | CFA별 α, read noise, 선형성, 모델 검사, 그래프 |

모든 촬영 스크립트는 바꾼 노드를 끝날 때 원래 값으로 되돌린다. 실패하면 `restore_errors`에 기록한다.
UserSet은 저장하지 않는다.

## PTC 절차

1. 워밍업: 측정 전 30분 스트리밍 (offset이 온도에 따라 변한다)
2. 장면: 흰 종이나 diffuser를 DC 조명으로 고르게 비춘다. 화면 중앙 256×256만 쓴다.
   초점은 맞지 않아도 된다(흐리면 오히려 균일해진다). 촬영 중에는 카메라·렌즈·조리개·조명을 건드리지 않는다
3. 밝은 촬영
   ```bash
   source env.sh && cd scripts
   python ptc_capture.py bright --session ../data/ptc_$(date +%Y%m%d_%H%M)
   ```
   출력마다 G 평균이 포화 범위의 80%가 되는 노출시간 T_top을 찾는다.
   - `not saturated at T_max`: 빛이 약하다. 그 채널은 최대 노출(40 ms)까지만 잰다
   - `saturated near T_min`: 빛이 너무 세다
   - 한 광량으로 네 채널을 모두 덮기는 어렵다. 결과를 보고 광량을 바꿔 `--outputs`로 해당 채널만 새 세션에서 다시 찍는다
4. 렌즈캡을 씌우고 같은 세션으로 `python ptc_capture.py dark --session <같은 경로>`
5. `python ptc_analyze.py <세션>` → `<세션>/analysis/`에 출력별 JSON, PNG, `summary.json`

기본값: BayerRG24, Gain 0 dB, BlackLevelRaw 256, 16단계 × 100장, ISP(WB/CCM/LUT/HDR enhancement/clamping) 끔, chunk(ExposureTime/Gain/BlackLevel/CRC) 켬.
chunk의 노출시간이 요청값과 다른 프레임은 버린다(설정 변경 직후 남아 있던 프레임).

## 노이즈 모델 패키지 (RGB→RAW 학습·평가용)

`calibration/` 에 카메라별 최종 산출물(파라미터 JSON, 픽셀 지도, dark 분포)과 사용법이 있다 → `calibration/README.md`.
코드: `scripts/noise_model.py` (sample / variance / zscore / nll), 검증: `scripts/validate_noise_model.py`.

## 최종 결과: 2131 세션 (2026-10-19 21:31 ~ 10-20 03:12, BFS + LUCID 동시)

데이터 위치:
- PTC: `data/ptc_20261019_2131_lucid`, `data/ptc_20261019_2131_bfs` (ROI 256², 16단계 × 100장, 단계별 dark 포함)
- 전체 화면 dark: `data/fullframe_dark_20261020/{bfs,lucid}` (gain 0/6/10/12/18/24 × 노출 1/10/30/100/300/1000 ms × 300장)
- 분석 결과: 각 폴더의 `analysis/`

스크립트: `bfs_ptc_capture.py`(Spinnaker ctypes, `spin_common.py`), `fullframe_dark.py`, `fullframe_analyze.py`, `noise_model_test.py`, `run_darks.sh`

### BFS-PGE-23S3C (IMX392, SN 26315393)

기준 캘리브레이션(최현석, α 5.774~5.808, read 2.357~2.395 e⁻)과 0.6% 안에서 일치한다.

| gain | α (DN/e⁻, G) | 실제/공칭 gain | read noise | 검은 레벨 @1 ms | dark current |
|------|--------------|----------------|------------|-----------------|--------------|
| 0 dB | 5.811 | 1.000 | 2.36 e⁻ (13.7 DN) | 633 | 18 e⁻/s |
| 6 dB | 11.52 | 0.993 | 1.89 e⁻ | 609 | 17 |
| 10 dB | 18.23 | 0.992 | 1.69 e⁻ | 581 | 16.5 |
| 12 dB | 22.87 | 0.989 | 1.62 e⁻ | 561 | 16 |
| 18 dB | 45.58 | 0.987 | 1.43 e⁻ | 466 | 16 |
| 24 dB | 91.21 | 0.990 | 1.24 e⁻ | 277 | 15.5 |

- 아날로그 gain이 섞여 있어서 read noise(e⁻)가 gain에 따라 줄어든다. 그래서 gain마다 따로 재야 한다
- 검은 레벨도 gain에 따라 바뀐다
- dark current는 검은 레벨 상승률과 분산 증가율 두 방법이 1% 안에서 같다. α에 대한 교차 검증이기도 하다
- 행 노이즈는 0 dB에서 2.4 DN으로, 픽셀 분산의 3%다. 가로 이웃 상관 3%가 이것으로 설명된다. 열 노이즈는 무시할 수준이다
- hot pixel은 짧은 노출에서 10~30개, 1 s에서 CFA 채널당 약 2,000개(0.35%)다. noisy pixel은 0.3~0.7%다
- 상수 α 모델은 χ²로 기각된다(0 dB에서 χ²/dof 16).
  국소 α가 저신호에서 −1.2%, 1만~2.5만 DN에서 +1.5%로 매끄럽게 변한다.
  gain을 올릴수록(전자 수가 적을수록) 이 차이가 사라져서(24 dB에서 χ²/dof 2.4) 전하량에 따른 센서 비선형으로 본다. 저gain에서는 α(S) 표를 쓴다
- dark 노이즈 분포는 거의 정규분포다. 측정 분포가 NLL 0.005 nats 더 좋다

### LUCID TRI054S-C (IMX490, SN 254600231)

| 출력 | α (DN/e⁻) | read noise | 12-bit 끝 | χ²/dof |
|------|-----------|------------|-----------|--------|
| ch0 (SP1 HCG) | 9.627 | 0.53 e⁻ | 425 e⁻ | 1.0 (모델과 일치) |
| ch1 (SP1 LCG) | 0.4524 | 2.46 e⁻ | 9,050 e⁻ | 57.8 (국소 α ±3%) |
| ch2 (SP2) | 0.964 | 13.5 e⁻ | 4,250 e⁻ | 4.7 |
| ch3 (SP2) | 0.0261 | 102 e⁻ | 157,000 e⁻ | 63.1 (국소 α ±4%) |
| HDR | 구간별 | 0.49~0.52 e⁻ | 2²⁴−1 | — |

- 세 세션에 걸쳐 α가 0.1~1% 안에서 재현된다
- **Gain은 디지털이다(HDR 합성 뒤에 곱함).**
  - α가 gain에 0.5% 안에서 비례한다
  - read noise는 모든 gain에서 0.52 e⁻다
  - dark current는 모든 gain에서 1.4 e⁻/s다
  - 그래서 gain g의 모델은 0 dB 모델의 DN에 10^(g/20)을 곱한 것이다. gain을 올려도 SNR은 좋아지지 않고 동적 범위만 줄어든다
- 검은 레벨 ≈ 256 + 1.3·g, 행 노이즈는 픽셀 분산의 0.6%(무시 가능), 고정 패턴 0.46 DN
- **dark 노이즈가 밝은 쪽으로 치우친다.** 4σ 넘게 밝게 튀는 값이 정규분포보다 0.7 ms에서 5배, 320 ms에서 22배 많다. 어두운 쪽은 오히려 적다
  - 긴 노출: read noise가 전자 1개 미만이라 dark 전자의 푸아송 개수가 그대로 드러난다.
    평균으로 계산한 dark 전자 수와 분산으로 계산한 수가 635 ms까지 일치한다(1.129 vs 1.153 e⁻)
  - 짧은 노출의 초과분: 회로 RTS 계열로 본다
  - 같은 노출에서 HDR dark 분포는 ch0 단독과 같다. 어두운 구간은 ch0 하나로 정해진다
- 노이즈 모델 검증(짝수 프레임으로 모델을 만들고 홀수 프레임으로 채점):
  - 측정 분포 모델이 모든 노출에서 가장 좋다(NLL 최대 −0.038 nats, KS 0.006)
  - 정규분포는 긴 노출에서 보정이 무너진다(KS 0.054)
  - 그래서 LUCID 저조도 노이즈 합성과 메트릭에는 측정 분포(또는 푸아송 + read + 튀는 성분)를 쓴다
- **노출 1 s를 넘으면 모드가 바뀌는 것으로 보인다.**
  - 1.26 s와 2.5 s dark에서 검은 레벨이 268 → 259로 떨어지고, 분산이 비례보다 빨리 커진다(2.5 s 예상 450 → 실측 771)
  - 전체 화면 1.0 s는 정상이었다. 경계(1.0~1.26 s)는 아직 재지 않았다
  - 데이터 촬영에서는 1 s 넘는 노출을 피하거나 따로 취급한다
- HDR 국소 α: SP1 구간 약 10, SP2 구간 약 160(×16 = SP1/SP2 감도비)이다. ch2→ch3 전환 근처에서는 read noise가 더해져 일시적으로 443이다

### 남은 것 / 주의
- 조명: 짧은 노출에서 3% 깜빡임이 있었고, 첫 램프는 한 시간에 1/5로 어두워졌다. α는 영향이 없지만 노출 대비 신호 비교는 쓸 수 없다. DC 조명이 있으면 ch1·ch3 국소 α 변화의 원인을 확정할 수 있다
- 두 카메라가 밤사이 여러 번 재부팅했다(ForceIP 풀림). 특히 BFS는 혼자 재부팅했다. 전원(PoE)을 확인해야 한다
- HDR 합성 규칙(채널 전환 지점, 2×2 단위 여부)과 전환점 근처의 혼합 노이즈 모델은 아직 만들지 않았다
- 1 s 경계 확인: 0.9~1.3 s dark 몇 단계

## 지금까지 측정한 것 (2026-10-19, 렌즈캡, `data/dark_probe_254600231_20261019_172001`)

- `HDROutput`: `HDR`, `HDRChannel0`~`3`. 채널별 analog gain은 19.8 / 4.2 / 29.4 / 4.2 dB이다(`HDRTuningChannelSelector` 0~3)
- 모든 출력이 BayerRG24, 코드 간격 1 DN이다. 출력 offset은 `BlackLevelRaw` 값과 거의 같다(±4 DN)
- **BlackLevelRaw 0에서는 dark의 47~97%가 0으로 잘린다.** 64면 ch2(σ 13 DN)에 0이 아직 0.02% 남는다. 그래서 256을 쓴다
- dark 시간 노이즈 σ (DN, 86 µs): HDR 5.1, ch0 5.0, ch1 1.1, ch2 13.1, ch3 2.5~2.7
  - HDR 출력의 dark 구간 노이즈는 ch0과 같다. 어두운 구간은 ch0 스케일 그대로라는 뜻이다
- 20 ms dark에서 R/B의 dsnu가 커 보이는 것은 hot pixel 때문이다(ROI 512²에서 CFA당 약 50개, 최대 약 1,800 DN). robust dsnu는 네 채널 모두 1.48로 같다
- 채널이 SP1/SP2 × HCG/LCG 중 무엇인지는 아래 PTC 결과로 판단했다

## 최종 PTC 결과 (2026-10-19, `data/ptc_20261019_all`, `--long`, 5개 출력 한 세션)

조명을 다시 세팅해서 깜빡임이 절반 이하로 줄었다(106 µs에서 프레임 간 0.86%).
`--long` 모드에서는 노출마다 프레임레이트를 맞춰 노출을 2.5 s까지 늘렸다.
대신 램프 밝기가 촬영 중에 흘렀다(drift 검사 ch2 +10.9%, 세션 전체로 약 30%).
α는 단계마다 측정한 신호 기준이라 거의 영향이 없지만, "노출 대비 신호" 선형성 검사는 이 때문에 흔들린다.

| 출력 | 정체 | α (DN/e⁻) | read noise | 4095 DN | 비고 |
|------|------|-----------|------------|---------|------|
| ch0 | SP1 HCG | 9.586–9.619 | 0.525–0.540 e⁻ (5.0–5.2 DN) | 427 e⁻ | 모델 오차 ≤0.7%, 기울기 0.497 |
| ch1 | SP1 LCG | 0.4509–0.4542 | 2.47–2.49 e⁻ (1.11–1.13 DN) | 9,080 e⁻ | 국소 α ±3% |
| ch2 | SP2 | 0.965–0.970 | 13.4–13.9 e⁻ (13.0–13.4 DN) | 4,230 e⁻(SP2) | 모델 오차 ≤2.1% |
| ch3 | SP2 | 0.0259–0.0270 | 92–103 e⁻ (2.5–2.7 DN) | 157,000 e⁻(SP2) | G의 국소 α ±8%, 확인 필요 |
| HDR | 합성 | 구간별 (아래) | 0.50–0.51 e⁻ (dark=ch0) | 2²⁴−1 = 16,777,215 | |

- 재현성: 조명이 다른 첫 세션과 비교해 α는 ch0 0.1%, ch1 0.2%, ch2 0.5~1% 안에서 같다
- HDR 국소 α (G)
  - 4×10⁴ DN 이하: 9.7~10.1 (SP1)
  - 7.8×10⁴~2.8×10⁵ DN: 150~190 (SP2, ch2)
  - 5.5×10⁵ DN: 443 (ch3 read noise가 더해짐)
  - 10⁶ DN 이상: 327 → 161로 수렴 (SP2 shot)
  - SP2 구간 α가 SP1의 약 16배 = SP1/SP2 감도비
- HDR 범위: 0.5 e⁻ ~ 16.7M DN ÷ 9.8 ≈ 1.7M e⁻(SP1 기준), 약 130 dB
- ch3의 µs당 전자(0.073)가 ch2(0.048)보다 높다. 램프가 밝아진 영향이 일부 있지만, α를 DC 조명에서 다시 확인해야 한다
- 그림: `data/ptc_20261019_all/analysis/ptc_overview.png`, 출력별 `<output>.png`

## 첫 PTC 결과 (2026-10-19, `data/ptc_20261019_1835`, 형광등 계열 조명 + 외부광)

`ptc_analyze.py --detrend 2` 기본값으로 분석했다.
조명이 짧은 노출에서 프레임마다 1~2% 흔들리고, 그 흔들림이 화면에 고르지 않았다.
그래서 쌍 차이에서 2차 곡면을 뺐다. 빼기 전에는 ch0 α가 신호에 따라 8% 올라갔고, 빼고 나니 ±0.5%로 평평해졌다.
3차로 올려도 α는 0.15%만 바뀐다.

| 출력 | 정체(추정) | α (DN/e⁻) | read noise | 4095 DN | e⁻/µs |
|------|-----------|-----------|------------|---------|-------|
| ch0 | SP1 HCG | 9.59–9.62 | 0.53–0.55 e⁻ (5.1–5.2 DN) | 427 e⁻ | 0.67 |
| ch1 | SP1 LCG | 0.451–0.454 | 2.48–2.51 e⁻ (1.12–1.14 DN) | 9,070 e⁻ | 0.69 |
| ch2 | SP2 (작은 픽셀) | 0.972–0.982 | 13.5–13.7 e⁻ (13.1–13.4 DN) | 4,210 e⁻(SP2) | 0.047 |
| ch3 | SP2 (다른 gain) | 측정 못 함 | dark 2.5–2.7 DN | — | — |

- ch0과 ch1은 µs당 전자 수가 같다. 그래서 같은 포토다이오드(SP1)를 HCG/LCG로 읽은 것으로 본다.
  ch2는 그 1/14.5이라 작은 픽셀(SP2)로 본다
- 검사 결과: shot 기울기 0.488–0.506, 모델 오차 ch0 ≤0.7%, ch1 ≤4.2%, ch2 ≤2.0%, CFA 사이 α 차이 <1%, G1/G2 차이 ≤0.14%
- ch1·ch2의 국소 α가 ±3% 오르내린다. 조명 깜빡임이 남은 것인지 실제 비선형인지는 DC 조명으로 다시 재야 구분된다
- ch3: 40 ms에서도 신호가 약 50 DN이다. 빛이 20~40배 더 필요하다
- 노출시간은 약 20 µs 단위로 설정되지만, 신호는 약 40 µs 단위로만 바뀌었다.
  그래서 1 ms 아래에서는 "신호 vs 노출" 선형성이 10~15% 어긋나 보인다. α는 측정한 신호 기준이라 영향이 없다.
  ch2의 8~40 ms 선형성 잔차는 0.35%다
- HDR 합성 출력
  - dark 구간은 ch0과 같다
  - 약 5×10⁴ DN(SP1 약 5,000 e⁻)까지는 국소 α ≈ 10으로, SP1 HCG→LCG가 이어져도 shot noise가 연속이다
  - 그 위에서 SP2로 넘어가며 국소 α가 15~19배 뛴다(SNR dip)
  - R은 G보다 낮은 신호에서 전환된다. 2×2 단위로 판단하는 것으로 보인다
- 그림: `analysis/ptc_overview.png`, 출력별 `analysis/<output>.png`

## 2026-10-20 오후: LUCID HDR 합성 규칙, 저조도, PRNU, 포화 근처

데이터는 외장 SSD `noise_calibration_20261020/data/session_20261020_pm/`에 있다(약 120 GB).
그림과 요약 수치는 `results_20261020/`에 있다. 아직 `noise_model.py`와 보정 패키지에는 반영하지 않았다.

### 세션
| 폴더 | 내용 | 스크립트 |
|------|------|----------|
| `hdr_transition_0dB` | LUCID 0 dB, 흰 종이, 1–120 ms 22단계. HDR 48장 + HDRChannel0–3 각 12장 + 같은 노출 dark | `hdr_transition_capture.py` |
| `hdr_transition_12dB` | 12 dB, HDR만 8단계 + dark(12 dB로 재촬영) | 같음, `--hdr-only --gain-db 12` |
| `lowlight` | 조리개를 조여 0.2 e⁻까지. LUCID HDR+채널 8단계, BFS 0/24 dB 10단계, 각각 dark | `lowlight_capture.sh`, `all_darks.sh` |
| `dark_extra` | 긴 노출 dark(BFS 50 ms–5 s 0/24 dB, LUCID 150–800 ms), RTS 확인용 긴 스택(BFS 1 ms 300장, LUCID 87 µs 120장) | `dark_extra.sh` |
| `prnu_flats` | 렌즈 앞 복사지 확산판. BFS 5–37 ms 6단계, LUCID 1–800 ms 10단계, 각 32장 | `fullframe_dark.py`, `prnu_analyze.py` |
| `bfs_saturation` | BFS 복사지, 42–75 ms(포화의 약 80–150%) 32장 | `flat_figures.py` |
| `color_transition` | LUCID 색종이 7색(핑크·흰·노랑·보라·빨강·초록·파랑), HDR만 0.1–100 ms 28단계 | `color_transition_analysis.py`, `ch0_ch1_handover.py` |

### LUCID HDR 합성 규칙 (0 dB DN, gain과 무관; 12 dB에서 같은 센서 신호 위치)
| 전환 | 위치 | 방식 | 결정 단위 |
|------|------|------|-----------|
| ch0 → ch1 (SP1, HCG→LCG) | 약 3,900 DN (≈400 SP1 e⁻, ch0 포화) | 프레임마다 하드 전환 | 픽셀 |
| SP1 → SP2 | 묶음 판정값 반반 지점 48.5–49.8k DN, 폭 약 15k | **가중 평균으로 섞음** | 2×2 묶음 |
| ch2 → ch3 (SP2, HCG→LCG) | 약 55–60만 DN (ch2 신호 ≈3,700–3,800 DN) | 프레임마다 하드 전환 | 픽셀 |

- SP1→SP2 판정값: **각 픽셀의 SP1 값을 SP1 LCG 상한(≈21.7×4095 ≈ 8.9만 DN)에서 자른 뒤 네 픽셀을 같은 무게로 평균**.
  7색에서 반반 지점의 흩어짐(log 표준편차) 0.007. G 평균 0.226, 최댓값 0.255, 자르지 않은 평균 0.032
- 섞는 비율 w(판정값)은 분산으로 풀었다: Var = w²·V1(S) + (1−w)²·V2(S) (`hdr_transition_blend.py`, `results_20261020/sp1_sp2_blend_weight.json`)
- 섞는 구간에서 이미 SP1이 포화된 픽셀(진한 색의 밝은 채널)은 잘린 값이 섞여 평균이 최대 약 10% 처진다(파랑·보라의 B 9–10%, 빨강의 R 3%). SP2로 다 넘어가면 회복된다
- HDR 값은 채널마다 격자 위에 있다: K1(ch1) R 21.714 / Gr 21.706 / Gb 21.754 / B 21.718, K2(ch2) R 156.51 / Gr 144.95 / Gb 146.48 / B 140.37 DN/코드, K3(ch3) ≈ 37·K2(색마다 36.98–37.02로 조금씩 다름)
- `hdr_transition_sources.py`, `hdr_transition_ch3.py`는 격자로 프레임별 출처를 가르려던 시도다. 섞는 구간과 K3 오차 때문에 최종 결론에는 쓰지 않았다

### 저조도, dark
- 두 카메라 모두 약 0.2 e⁻부터 PTC 모델(BFS: 분산 = S + read², LUCID 채널별)과 맞는다. 그림은 픽셀 평균으로 묶을 때 생기는 치우침을 피하려고 짝수 프레임 평균으로 묶고 홀수 프레임으로 평균·분산을 낸다
- LUCID ch3 저신호 바닥은 G가 R·B보다 높다(분산 약 7 대 5.5–6 DN²). ch3 read noise는 색별로 둬야 한다
- LUCID dark 150–800 ms: 오프셋이 노출에 비례(약 14 DN/s = 1.4 e⁻/s), 800 ms까지 오프셋 튐 없음
- BFS 5 s dark: dark 신호 약 78 e⁻(15.6 e⁻/s), 시간 노이즈 8.5 e⁻ ≈ √78 + read

### PRNU (복사지 확산판, 조리개 개방, 8 px 가우시안 고역통과)
- BFS 0 dB: 0.91–0.95% (포화 10–50% 구간, 신호에 무관)
- LUCID: ch0 약 0.6%, ch1 0.7–1.0%, ch2 1.2–1.4%(R 1.3–2.2%)
- LUCID ch3 구간(800 ms, SP1 약 12배 과노출): 대부분 ±3%이지만 상위 1%가 1.4배, 0.1%가 2배 이상 밝다. SP1 과노출 시 SP2로 전하가 새는 것으로 추정되는 고정 패턴
- 종이 장면의 "spatial" 성분 3–4.5%는 대부분 조명 얼룩·종이 질감이다(`electron_ptc.py`)

### BFS 포화 근처
- 시간 분산은 약 8,500 e⁻까지 모델을 따르고, 그 위에서 눌려 1.1만 e⁻ 근처에서 모델보다 약 20% 작다. 65516 DN(약 1.12만 e⁻)에서 하드 클립
- 같은 픽셀의 노출당 응답은 1만 e⁻에서 약 0.5%, 1.1만 e⁻에서 약 1.5% 처진다

### 기타
- LUCID가 하루에 여러 번 저절로 재부팅(IP가 169.254로 돌아감). 촬영 전 `gvcp_ip.py force` 필요
- `hdr_transition_capture.py dark`가 처음에는 세션 gain을 무시하고 0 dB로 찍었다. 고쳤고 12 dB dark는 다시 찍었다
- 노이즈 성분 비율 대략치: `noise_budget.py`. BFS read noise는 gain을 올리는 내내 줄어 "앞단+뒷단" 2분할 모델이 맞지 않았다
