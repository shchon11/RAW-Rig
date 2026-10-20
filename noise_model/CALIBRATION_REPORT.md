# 카메라 노이즈 캘리브레이션 보고서 — BFS-PGE-23S3C (IMX392) · TRI054S-C (IMX490)

- 측정: 2026-10-19 17:00 ~ 2026-10-20 05:10, ircv PC (`~/jsh`)
- 작성: 2026-10-20
- 작업: shchon11 + Claude (PTC 세션). 브라케팅 도구 연동과 트리거 시험은 jsh-3e 세션

## 1. 목적

RGB에서 특정 카메라의 RAW를 복원하는 모델을 만든다. 그 모델을 **학습하고 평가할 때 쓸 카메라별 노이즈 모델**을 실측으로 만드는 것이 이 작업의 목적이다.

1. 학습 데이터 합성: 깨끗한 RAW 신호에 그 카메라와 통계적으로 같은 노이즈를 입힌다(저조도 쌍 생성)
2. 학습 손실: 정답 RAW에 섞인 노이즈를 감안한다(신호별 분산으로 나눈 오차, NLL)
3. 평가 메트릭: 예측과 실측의 차이가 그 카메라 노이즈로 설명되는지 통계로 판정한다(표준화 잔차, NLL, χ², KS). 절대오차나 PSNR로 판정하지 않는다

산출물은 `calibration/<model>_<serial>/` 패키지와 `scripts/noise_model.py`(sample / variance / zscore / nll)다.

## 2. 대상과 측정 조건

| | FLIR Blackfly S BFS-PGE-23S3C | LUCID Triton TRI054S-C |
|--|--|--|
| 센서 | Sony IMX392, 3.45 µm, global shutter | Sony IMX490, 3.0 µm, sub-pixel HDR (SP1/SP2 × HCG/LCG), rolling |
| 시리얼 / 펌웨어 | 26315393 / 2103.0.335.0 (FLIR_control 기준 캘리브레이션과 같은 카메라) | 254600231 / 1.138.0.0 |
| SDK | Spinnaker 4.3 C API를 ctypes로 사용(`spin_common.py`) | Arena SDK x64 1.0.10.11, arena_api 2.10.0 |
| 픽셀 포맷 | BayerRG16 (ADC 12-bit, 코드 << 4, 간격 16, 나머지 12) | BayerRG24, HDROutput=HDR (합성 출력, 간격 1) |
| 블랙 | BlackLevel 1.0 % (clamp off) → 약 633 DN @0 dB | BlackLevelRaw 256 (pedestal) |
| ISP | WB auto off·ratio 1.0, gamma/LUT/defect correction/IspEnable off | WB/CCM/LUT/HDR image enhancement/HDR digital clamping off |
| 기타 | ExposureAuto/GainAuto off, Reverse off, 비닝 1 | HDRTuningEnable false (채널 analog 19.8/4.2/29.4/4.2 dB), ADC Bits12, 비닝 1 |
| 포화 | 65516 | 2²⁴−1 (채널 출력은 4095+256) |

- 연결: 두 카메라 모두 `enp5s0`(192.168.100.100/24, MTU 1500)의 같은 스위치에 연결했다.
  원래 IP가 다른 대역이라 임시 ForceIP로 BFS .202, LUCID .201을 썼다(`gvcp_ip.py`)
- 이 설정은 `~/jsh` RAW 리그의 녹화 설정과 같다. **설정이 다르면 이 모델을 쓸 수 없다.**

## 3. 측정 절차

### 3.1 사전 확인
1. **노드맵 덤프**: `lucid_dump.py`로 503개 노드를 읽었다. 이때 HDROutput에 단일 채널 출력(HDRChannel0~3)이 있음을 확인했다
2. **렌즈캡 dark probe**(`dark_probe.py`)
   - BlackLevelRaw 0이면 dark의 47~97%가 0으로 잘린다
   - 64면 ch2(σ 13 DN)에 아직 0.02%가 남는다 → **256 채택**
   - 출력 offset ≈ BlackLevelRaw, 코드 간격 1
3. **gain 동작**(`lucid_gain_check.py`)
   - LUCID Gain을 바꿔도 채널 analog gain 노드는 그대로다
   - 출력은 모든 채널과 HDR에서 정확히 10^(dB/20)배다
   - → 합성 뒤에 곱하는 디지털 gain이다. read noise(e⁻)가 gain과 무관한 것으로 다시 확인했다(5장)

### 3.2 PTC (bright + matched dark)
EMVA 1288 Release 4의 노출시간 sweep 방식이다. 광량은 고정하고 노출시간만 바꾸며, 노출마다 같은 노출의 dark를 찍는다.

- **ROI**: 중앙 256×256(LUCID x=1312, y=802 / BFS x=832, y=472). 짝수 오프셋이라 RGGB가 유지된다. CFA당 16,384 픽셀이다
- **단계**: 출력마다 G 평균이 포화의 80%가 되는 노출 T_top을 자동으로 찾는다. 최소 노출~T_top을 로그 간격 16단계로 나누고, 단계당 100장(50쌍)을 찍는다. 끝에 맨 위 단계를 20장 더 찍어 drift를 본다
- **긴 노출(`--long`)**: LUCID는 프레임레이트가 노출을 제한한다(1 Hz → 1 s).
  그래서 단계마다 스트림 정지 → 레이트 최소 → 노출 → 레이트 = 1/(노출+0.5 ms) → 시작 순서로 설정한다. 노출은 2.5 s까지 썼다
- **프레임 검증**: chunk의 ExposureTime·CRC·frame id를 확인한다.
  LUCID는 노출을 바꾸면 chunk 값이 영상보다 1프레임 먼저 바뀐다(실측). 그래서 chunk 일치 뒤 2장을 더 버린다
- **dark**: bright가 끝나면 렌즈캡을 씌우고 같은 노출 목록으로 같은 장수를 찍는다
- **gain 시리즈**: BFS 0/6/10/12/18/24 dB 전부, LUCID는 HDR 출력으로 같은 6개다.
  LUCID는 0 dB에서 채널 4개(HDRChannel0~3)와 HDR을 모두 찍었다
- 세션

  | 세션 | 시각 | 내용 | 비고 |
  |------|------|------|------|
  | `ptc_20261019_1835` | 18:35 | LUCID 출력 5개, ≤40 ms | 형광등 + 외부광, 깜빡임 1.4~1.9 % |
  | `ptc_20261019_all` | 19:5x | LUCID 출력 5개, ≤2.5 s | 조명 재설정, 램프가 서서히 밝아짐 |
  | **`ptc_20261019_2131_{lucid,bfs}`** | 21:31~00:58 | **최종**: 두 카메라 동시, BFS gain 6개, LUCID 출력 5개 + HDR gain 5개 | 첫 램프가 1시간에 1/5로 어두워짐. 23:05에 조명을 바꾼 뒤 BFS 전체와 LUCID 6/10/12 dB를 다시 찍음. dark는 00:03~01:02 |

### 3.3 전체 화면 dark (노이즈 구조)
- `fullframe_dark.py`(`run_darks.sh`): 렌즈캡, 두 카메라 병렬, gain 0/6/10/12/18/24 × 노출 1/10/30/100/300/1000 ms × 300장. 00:12~03:12에 찍었다
- 링크 예산은 BFS 20 MB/s(≤4 Hz), LUCID 45 MB/s(≤2 Hz)다
- 저장은 프레임을 받으면서 바로 줄였다
  - 픽셀별 평균, 분산, 연속차 분산
  - 프레임별 행·열 평균
  - 원본 프레임 20장
- 단계마다 IP를 다시 맞추고 실패하면 한 번 재시도했다. 두 카메라가 밤사이 여러 번 스스로 재부팅했다

### 3.4 트리거·주기 검증 (2026-10-20 04:25~05:10)
- jsh-3e: 브라케팅 도구(GigE 액션 커맨드 트리거)로 dark를 찍었다
  - 1/30/300/1000 ms (BFS 0·24 dB, LUCID 0 dB)
  - free-run 대조군
  - LUCID 트리거 60/100/150 ms
  - 1 s 쌍안정 시험
- PTC 세션: `period_test.sh`(같은 노출에서 프레임 주기만 500/800/1000.5/1050 ms로 바꿈), `trigger_dark_compare.py`, `dark_npz_compare.py`

## 4. 분석 방법

### 4.1 PTC (`ptc_analyze.py`)
- 픽셀 i, CFA c 모델: `RAW_i = o_i(T) + S_i + e_i`, `Var(e_i) = α_c(S)·S_i + v_dark(T)`
- **신호**: S = mean_t(bright) − mean_t(dark)를 픽셀별로 구해 평균한다(matched dark)
- **분산**: 겹치지 않는 프레임 쌍의 차이 (A−B)에서 **2차 곡면을 빼고** 분산/2로 구한다
  - 조명이 프레임마다 1~3% 깜빡였는데, 그 흔들림이 화면에 고르지 않았다
  - 평균만 빼면(EMVA 방식) ch0 α가 신호에 따라 8% 올라갔다. 2차 곡면을 빼면 ±0.5%로 평평해졌다. 3차로 올려도 α는 0.15%만 바뀐다
  - 정답을 아는 합성 데이터에서는 이 처리가 α를 치우치게 하지 않음을 확인했다
- **hot pixel**: 가장 긴 dark의 평균이 중앙값 + 10·robust σ를 넘는 픽셀은 뺀다
- **α**: 원점을 지나는 가중 최소제곱으로 구한다. 가중치는 N_pix·N_pairs / 2(v_b²+v_d²)다
  - 사용 단계: 포화 아님, S ≤ 0.7·(white − offset), S ≥ max(10 e⁻, 20·σ_dark)
  - 표준오차는 WLS 이론값과 짝/홀 쌍 분할값을 같이 쓴다
- **국소 α** = (v_b − v_d)/S를 단계별로 구한다(α(S) 표)
- **read noise**: 최소 노출 dark의 √v_d / α
- **통계 검정**: 단계별 z = (v_b − (αS + v_d)) / √(SE_b² + SE_d² + (S·SE_α)²)이고, SE는 쌍 사이 산포로 구한다. 이것으로 χ²/dof와 p값을 낸다.
  **상대오차(%)로 판정하지 않는다.** 단계당 샘플이 약 80만 개라 1%만 달라도 수 σ다
- **선형성**: 노출 대비 신호 직선 적합. 단 LUCID는 노출이 ~40 µs 단위로만 실제 반영되고, 조명이 흘렀기 때문에 참고만 한다

### 4.2 전체 화면 dark (`fullframe_analyze.py`)
CFA별로 구한 것:
- offset 중앙값
- DSNU: robust 공간 σ에서 시간 성분을 뺀 값
- 시간 σ: 연속차로 구해서 느린 drift에 둔감하다
- σ 분위수
- hot pixel(mean − median > 5·median σ), noisy pixel(σ > 2·median σ)
- 행·열 시간 노이즈: 행·열 평균의 분산에서 픽셀 노이즈 몫을 뺀다
- 행·열 FPN
- 이웃 상관
- 표준화 잔차 분포: 첨도, 3/4/5σ 초과 비율, KS, Tukey-λ(PPCC)

dark current는 두 가지로 독립적으로 구해서 교차 검증한다.
- offset 기울기 / α
- 분산 기울기 / α²

### 4.3 노이즈 분포 모델 비교 (`noise_model_test.py`)
PTC dark 단계에서 짝수 프레임으로 모델을 맞추고, 홀수 프레임으로 채점했다.

비교한 모델:
1. 픽셀별 정규분포
2. 푸아송(dark 전자) + read noise
3. 실측 표준화 잔차 분포

채점: 코드 간격 q를 반영한 확률질량 F(x+q/2)−F(x−q/2)의 음의 로그우도(nats/픽셀), 정규 대비 차이와 그 표준오차, 무작위 PIT의 KS 거리.

### 4.4 패키지 생성과 검증 (`build_calibration.py`, `validate_noise_model.py`)
- **gain별 지도**: 1 ms offset, dark rate(1~1000 ms 선형 적합), read σ(1 ms), dark 분산 증가율, hot/noisy 마스크
- **gain별 표**: PTC의 CFA별 α, 국소 α 표, read noise, 전체 화면 dark 요약
- **분포**: gain × 노출별 표준화 잔차 분위수(1,005개)
- **검증**: 캘리브레이션에 쓰지 않은 세션(`ptc_20261019_all`)에 모델을 적용해 단계별 분산을 비교했다. `sample()`이 같은 분산을 재현하는지도 확인했다

## 5. 결과

### 5.1 BFS (IMX392)
FLIR_control 기준 캘리브레이션(최현석, 2026-10-15: α 5.774~5.808, read 2.357~2.395 e⁻)과 **0.6% 안에서 일치**한다.

| gain | α G (DN/e⁻) | 실제/공칭 | read noise | offset @1 ms | dark current | χ²/dof |
|------|------|------|------|------|------|------|
| 0 dB | 5.811 | 1.000 | 2.36 e⁻ (13.7 DN) | 633 | 18 e⁻/s | 16.5 |
| 6 dB | 11.52 | 0.993 | 1.89 e⁻ | 609 | 17 | 16.8 |
| 10 dB | 18.23 | 0.992 | 1.69 e⁻ | 581 | 16.5 | 5.2 |
| 12 dB | 22.87 | 0.989 | 1.62 e⁻ | 561 | 16 | 5.6 |
| 18 dB | 45.58 | 0.987 | 1.43 e⁻ | 466 | 16 | 3.5 |
| 24 dB | 91.21 | 0.990 | 1.24 e⁻ | 277 | 15.5 | 2.4 |

- 아날로그 gain 성분이 있어서 read noise(e⁻)가 gain에 따라 준다. 실제 gain은 공칭보다 약 1% 작다
- 상수 α는 저gain에서 χ²로 기각된다. 국소 α가 저신호에서 −1.2%, 1만~2.5만 DN에서 +1.5%다.
  고gain(전자 수 적음)일수록 맞으므로 전하량에 따른 비선형으로 본다 → α(S) 표를 쓴다
- 행 노이즈는 픽셀 분산의 3%(0 dB 2.4 DN)로 가로 이웃 상관 3%를 설명한다. 열 노이즈는 무시 가능하다
- dark 분포는 거의 정규다(측정 분포가 NLL −0.005)
- hot pixel은 짧은 노출 10~30개, 1 s에서 CFA당 약 2,000개(0.35%)다. noisy pixel은 0.3~0.7%다

### 5.2 LUCID (IMX490)

| 출력 | 정체 | α (DN/e⁻) | read noise | 12-bit 끝 | χ²/dof |
|------|------|------|------|------|------|
| ch0 | SP1 HCG | 9.627 | 0.53 e⁻ | 425 e⁻ | 1.0 |
| ch1 | SP1 LCG | 0.4524 | 2.46 e⁻ | 9,050 e⁻ | 57.8 |
| ch2 | SP2 | 0.964 | 13.5 e⁻ | 4,250 e⁻ | 4.7 |
| ch3 | SP2 | 0.0261 | ~100 e⁻ | 157,000 e⁻ | 63.1 |
| HDR | 합성 | 구간별(어두운 구간 9.83, SP2 구간 ~160) | 0.52 e⁻ | 2²⁴−1 | — |

- 세 세션에서 α가 0.1~1% 안에서 재현된다
- ch0과 ch1은 µs당 전자가 같다(같은 포토다이오드). ch2와 ch3는 SP2로, 감도가 SP1의 1/14.5~1/16이다
- HDR 출력 구조
  - 어두운 구간은 ch0 하나로 정해진다(dark 분포까지 ch0 단독과 같다)
  - 약 5×10⁴ DN에서 SP2로 넘어가며 국소 α가 ×16이 된다
  - 약 5×10⁵ DN에서 ch2→ch3 전환이 있다
  - 범위는 약 130 dB다
- **Gain은 디지털이다.** α는 0.5% 안에서 gain에 비례하고, read noise 0.52 e⁻와 dark current 1.4 e⁻/s는 모든 gain에서 같다.
  검은 레벨 ≈ 256 + 1.3·g다
- dark current 1.4 e⁻/s: offset 방법과 분산 방법이 일치한다
- 행 노이즈는 분산의 0.6%로 무시 가능하다. DSNU는 0.46 DN이다. hot pixel은 1 s에서 CFA당 약 8,000개(0.6%)다
- **dark 노이즈가 밝은 쪽으로 치우친다.** +4σ 초과가 0.7 ms에서 5배, 320 ms에서 22배고, 어두운 쪽은 오히려 적다
  - read noise가 전자 1개 미만이라 dark 전자의 푸아송 개수가 드러나는 것이다. 평균 기반 dark 전자 수 = 분산 기반 수(635 ms에서 1.129 vs 1.153 e⁻)
  - 짧은 노출의 초과분은 회로 RTS 계열로 본다
  - → 측정 분포 모델이 정규보다 NLL 최대 −0.038 nats, KS 0.006(정규는 0.054)

### 5.3 검증
- **LUCID, 독립 세션**
  - 검은 레벨 257.07 vs 모델 257.05
  - 분산 실측/모델: 15단계 중 13단계에서 0.95~1.00(중앙값 0.984)
  - 나머지 두 단계는 HDR 채널 전환점이다(7.7 ms 1.17, 53 ms 1.45) → 6장
- **BFS 0·24 dB**: 모든 단계에서 0.993~1.002이고, |z|는 대부분 2 이하다
- **`sample()`**: 모델 분산을 재현한다(양자화 이중 계산은 2026-10-20에 고쳤다)

## 6. 한계와 사용 조건

1. **LUCID HDR 채널 전환 구간**
   - 0 dB에서 SP1→SP2 약 4~8×10⁴ DN, ch2→ch3 약 2.8~5.5×10⁵ DN이다. gain g면 ×10^(g/20)
   - 이 구간에서 실측 노이즈가 모델보다 17~45% 크다. 프레임마다 다른 채널로 합성되어 두 분포가 섞이기 때문이다
   - 혼합 모델은 미완성이다. 그 구간 픽셀은 평가에서 따로 본다. 판정이 2×2 단위로 보이므로 쿼드 단위로 표시한다
2. **LUCID 노출 상한 800 ms**
   - 1 s에서는 전역 검은 레벨이 정상/−9.7 DN 두 상태를 무작위로 오가고, 스트리밍 중에도 바뀐다(jsh-3e 실측). 그래서 장면 dark로 보정되지 않는다
   - 300/500/800 ms는 단일 상태다. 1.26 s 이상은 분산 ×1.5인 다른 체제다
   - 캘리브레이션 1 s 전체 화면 dark는 전부 정상 상태였으므로 지도는 유효하다
3. **트리거 모드**: LUCID는 ≤150 ms(분산비 ≤1.05)만 쓴다. 300 ms 이상은 free-run보다 분산이 +7% 더 크다. BFS는 영향이 없다
4. **장면 dark 필수**: 녹화마다 같은 모드·노출로 dark를 찍는다
   - CFA별 offset 이동량을 더한다
   - 분산 배율 k_var는 마스크 밖 표본분산/모델 분산의 평균으로 구한다
   - 온도·재부팅 상태에 따라 dark current가 ±13% 변하는 것을 실측했다(BFS 24 dB 1 s offset 12 DN = 0.1σ)
5. **노출·gain 범위**
   - BFS [24 µs, 1 s], gain {0,6,10,12,18,24} dB만
   - LUCID [86 µs, 800 ms], gain은 아무 값이나 되지만 디지털이라 0 dB를 권장한다
6. **온도**: 측정 시 LUCID 47~51 °C, BFS 56~57 °C였다. ±3 °C 밖이면 경고한다. 워밍업은 스트리밍 30분이다
7. PRNU(픽셀별 감도)는 넣지 않았다(BFS 기준 약 1%)

## 7. 장비에서 실측으로 알아낸 것 (문서에 없던 것)

- LUCID는 ExposureTime을 바꾸면 chunk 값이 영상보다 1프레임 먼저 바뀐다(free-run·트리거 둘 다)
- LUCID 노출은 ~20 µs 단위로 설정되지만 영상은 ~40 µs 단위로 바뀌고, 약 32 µs 오프셋이 있다.
  1 ms 미만에서는 노출비 대신 측정 신호로 단계 간 비를 구한다
- LUCID 노출 상한은 프레임레이트에 묶인다(1 Hz → 1 s, 0.5 Hz → 2 s, 제한 없음 → 40~50 ms)
- LUCID 단일 채널 출력은 4095 + pedestal에서 잘린다. Gain을 올려도 채널 포화는 4095다
- BFS는 프레임레이트 하한이 약 1 Hz다. 그보다 긴 노출은 프레임레이트 제한을 꺼야 한다
- 두 카메라 모두 스스로 재부팅해 ForceIP가 풀리는 일이 반복됐다(BFS가 특히). 재부팅하면 설정이 기본값으로 돌아간다
- BFS 30 Hz 프레임레이트와 120 Hz 조명 리플이 맞물리면(4:1) 프레임 평균으로 깜빡임이 상쇄되지 않는다

## 8. 파일

```
noise_model/                (git: shchon11/RAW-Rig 브랜치 imx490-noise-model)
  CALIBRATION_REPORT.md     이 문서
  README.md                 설치·사용법
  LAB_LOG.md                측정 당시 작업 기록(시간순 결과)
  env.example.sh            Arena SDK 경로 설정 예
  scripts/                  측정·분석·모델 코드 (아래 표)
  calibration/<cam>/        noise_model.json, validation_*.json, residual_quantiles.npz
                            (maps_gNN.npz는 용량 때문에 git 제외 → 외장 번들)
외장 번들 noise_calibration_20261020/
  calibration_package/      패키지 전체 (지도 포함)
  data/ptc_final/           최종 PTC 원본 (ROI 스택 전체, manifest, analysis)
  data/fullframe_dark/      전체 화면 dark (지도용 통계 + 원본 20장/조건)
  data/ptc_earlier/         앞선 두 세션 (재현성)
  data/misc/                dark probe, 노드맵, 장면 확인 그림, 로그
  validation/trigger_dark_check/  트리거·free-run·주기 시험 (jsh-3e DNG + period test npz)
  code/                     코드 스냅샷 (+ git 커밋 해시)
```

| 스크립트 | 역할 |
|------|------|
| `gvcp_ip.py` | SDK 없이 GigE Vision 검색 / 임시 ForceIP |
| `lucid_common.py`, `spin_common.py` | LUCID(arena_api) / BFS(Spinnaker C ctypes) 설정·복원·grab |
| `lucid_dump.py`, `dark_probe.py`, `lucid_gain_check.py`, `scene_check.py` | 사전 확인 |
| `ptc_capture.py`, `bfs_ptc_capture.py` | PTC 촬영 (bright/dark, --long, --gain-db) |
| `fullframe_dark.py`, `run_darks.sh`, `period_test.sh` | 전체 화면 dark, 주기 시험 |
| `ptc_analyze.py`, `ptc_overview.py` | PTC 분석·그림 |
| `fullframe_analyze.py`, `noise_model_test.py` | dark 구조, 분포 모델 비교 |
| `build_calibration.py`, `noise_model.py`, `validate_noise_model.py` | 패키지 생성, 모델, 검증 |
| `trigger_dark_compare.py`, `dark_npz_compare.py` | 트리거/free-run dark 대조 |

## 9. 재현

```bash
source env.sh                                   # Arena SDK + venv (env.example.sh 참고)
cd scripts
python ptc_capture.py bright --long --session ../data/ptc_X_lucid          # 조명 켬, 평탄 타깃
env -u LD_LIBRARY_PATH python bfs_ptc_capture.py bright --gain-db 0 --session ../data/ptc_X_bfs
#   (렌즈캡)
python ptc_capture.py dark --session ../data/ptc_X_lucid
env -u LD_LIBRARY_PATH python bfs_ptc_capture.py dark --gain-db 0 --session ../data/ptc_X_bfs
./run_darks.sh                                   # 전체 화면 dark (세션 경로는 스크립트 안)
python ptc_analyze.py ../data/ptc_X_lucid ; python fullframe_analyze.py ../data/fullframe_dark_X/lucid --alpha-json ../data/ptc_X_lucid/analysis
python build_calibration.py ; python validate_noise_model.py ../calibration/<cam> <다른 세션> HDR --gain-db 0 --roi 802 1312
```
