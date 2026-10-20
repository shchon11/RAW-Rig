# 카메라 노이즈 모델 (RGB → RAW 복원 학습·평가용)

RGB에서 특정 카메라의 RAW를 복원하는 모델을 학습하거나 평가할 때 쓰는, 그 카메라의 실측 노이즈 모델이다.

| 폴더 | 카메라 | 센서 | 녹화 설정 |
|------|--------|------|-----------|
| `BFS-PGE-23S3C_26315393/` | FLIR Blackfly S | IMX392 | BayerRG16, ADC 12-bit, BlackLevel 1%, clamp off, ISP off |
| `TRI054S-C_254600231/` | LUCID Triton | IMX490 | BayerRG24, HDR 합성 출력, BlackLevelRaw 256, ISP off |

두 설정은 `~/jsh` RAW 리그의 녹화 설정과 같다(`~/.config/dm_clip_gui_raw/sensors/*.yaml`).
**설정이 다르면 이 모델을 쓸 수 없다.** 블랙 레벨, 픽셀 포맷, HDR 출력, ISP 중 하나라도 바뀌면 다시 재야 한다.

## 들어 있는 것

| 파일 | 내용 |
|------|------|
| `noise_model.json` | 설정, DN 단위, gain별 파라미터, 검증 결과 |
| `maps_gNN.npz` | gain별 전체 화면 픽셀 지도(아래 표) |
| `residual_quantiles.npz` | gain × 노출별 dark 노이즈 분포의 실측 분위수(분포 모양) |
| `validation_*.json` | 실측 프레임과의 비교 결과 |

`noise_model.json`의 gain별 파라미터:
- CFA별 α(DN/e⁻)
- 신호별 α 표
- read noise
- 노출별 dark offset과 분산
- dark current
- 행 노이즈
- hot/noisy pixel 수

`maps_gNN.npz`의 픽셀 지도(gain 0/6/10/12/18/24 dB):
- 검은 레벨(1 ms)
- dark current(DN/s)
- read noise σ
- dark 분산 증가율
- hot / noisy 마스크

코드: `../scripts/noise_model.py`. 만든 과정은 `../scripts/build_calibration.py`이고, 원본 측정 데이터는 `../data/`에 있다.

## 모델

```
raw = dark_offset(x, T) + shot + dark_noise + row_noise   → 코드 간격으로 반올림, [0, white]로 자름
shot       = α(S)·Poisson(S/α(S))       S = 노이즈 없는 신호(DN, 검은 레벨 위)
dark_noise ~ 실측 분포 모양 × √(read_σ(x)² + dark_var_rate(x)·T − σ_row²)
row_noise  ~ N(0, σ_row²), 행마다·프레임마다 하나
분산 = α(S)·S + read_σ(x)² + dark_var_rate(x)·T
```

## 학습·평가에 쓰는 법

```python
import sys; sys.path.insert(0, "/home/ircv/jsh/imx490_ptc/scripts")
from noise_model import NoiseModel
nm = NoiseModel("/home/ircv/jsh/imx490_ptc/calibration/TRI054S-C_254600231")
rng = np.random.default_rng()

# 1) 학습 데이터 합성: 깨끗한(또는 밝은 장면을 줄인) 신호 S에 이 카메라 노이즈를 입힌다
#    S는 전체 화면 RGGB Bayer, 검은 레벨을 뺀 DN. 저조도를 만들려면 밝은 RAW의 S에 k(<1)를 곱한다
raw_noisy = nm.sample(S, gain_db=0, exposure_us=30000, rng=rng)

# 2) 손실: 정답 RAW도 노이즈를 품고 있으므로 신호별 분산으로 나눈 오차 또는 NLL을 쓴다
nll = nm.nll(raw_measured, S_pred, gain_db=0, exposure_us=30000)        # 픽셀별 nats
# PyTorch 손실로 옮길 때: 0.5*log(2π·var) + 0.5*(raw-offset-S_pred)²/var, var = nm.variance(S_pred, ...)

# 3) 평가: 예측이 맞으면 표준화 잔차가 N(0,1)이어야 한다 → 평균, 분산, χ², KS로 판정
z = nm.zscore(raw_measured, S_pred, gain_db=0, exposure_us=30000)
```

평가는 절대오차나 PSNR이 아니라 통계로 한다. 예측이 완벽해도 정답 RAW에 노이즈가 있다.
- 잔차를 그 픽셀의 예측 노이즈 크기로 나눈 z의 평균이 0이면 밝기가 맞다
- 분산이 1이면 노이즈 수준까지 맞다
- 1보다 크면 그만큼 노이즈로 설명되지 않는 오차가 있다

## 핵심 수치

| | BFS (IMX392) | LUCID (IMX490, HDR) |
|--|--------------|---------------------|
| α (0 dB) | 5.81 DN/e⁻ (기준 캘리브레이션과 0.6% 일치) | 9.83 DN/e⁻(어두운 구간, SP1) → 밝은 구간 약 160 |
| read noise | 2.36 e⁻ → 24 dB에서 1.24 e⁻ | 0.52 e⁻ (모든 gain) |
| gain | 아날로그 성분 있음, 공칭보다 약 1% 작음 → gain별 값 사용 | 디지털 → DN만 10^(g/20)배, SNR은 그대로 |
| dark current | 약 17 e⁻/s | 1.4 e⁻/s |
| 행 노이즈 | 픽셀 분산의 3% (가로 줄무늬) | 0.6% (무시 가능) |
| dark 노이즈 모양 | 거의 정규분포 | 밝은 쪽으로 치우침(긴 노출에서 4σ 초과가 22배) → 실측 분포 사용 |
| 포화 | 65516 | 2²⁴−1 |
| 코드 간격 | 16 DN (나머지 12) | 1 DN |

## 검증 결과 (`validation_*.json`)

- **LUCID, 독립 세션**(캘리브레이션에 안 쓴 세션, 조명·시간 다름)
  - 검은 레벨: 257.07 vs 모델 257.05
  - 노이즈 분산: 15단계 중 13단계에서 실측/모델 0.95~1.00(중앙값 0.984)
  - 합성 노이즈 분산은 모델 분산과 같다
- **BFS 0·24 dB**: 모든 단계에서 실측/모델 0.993~1.002, |z| 대부분 2 이하
- **dark 노이즈 모양**(짝·홀 프레임 교차): 실측 분포 모델이 정규분포보다 NLL이 LUCID에서 최대 0.038, BFS에서 0.005 nats/픽셀 좋다

## 한계와 주의 (쓰기 전에 확인)

1. **LUCID HDR 채널 전환 지점에서는 노이즈가 틀린다.**
   - 큰 픽셀 → 작은 픽셀(0 dB에서 약 5×10⁴~8×10⁴ DN), ch2 → ch3(약 5×10⁵ DN) 근처에서 실측 분산이 모델보다 17~45% 크다
   - 픽셀이 프레임마다 다른 채널로 합성되면서 두 분포가 섞이기 때문이다. 이 혼합 모델은 아직 만들지 않았다
   - 그 구간 픽셀은 평가에서 따로 보거나 가중을 낮춘다
2. **LUCID 노출은 800 ms까지만 쓴다.**
   - 1 s에서는 전역 검은 레벨이 두 상태(정상 / −9.7 DN)를 무작위로 오가고, 스트리밍 중에 바뀌기도 한다. 그래서 장면 dark로도 보정이 안 된다 (jsh-3e 측정, 2026-10-20)
   - 300/500/800 ms는 항상 한 상태였다
   - 1.26 s 이상은 아예 다른 체제다(분산 ×1.5)
   - 캘리브레이션 1 s 전체 화면 dark는 전부 정상 상태였으니, 지도는 800 ms까지 그대로 유효하다
3. **검은 레벨은 녹화마다 dark 몇 장으로 다시 맞춘다.** BFS 24 dB에서 시점에 따라 12 DN(0.1σ) 흔들렸다. LUCID는 안정적이다.
4. **α(S) 표는 측정 단계 사이를 로그 보간한다.**
   - 상수 α는 통계적으로 기각된다. 저gain BFS ±1.5%, LUCID ch1/ch3 ±3~4%로 전하량이 많을 때 변한다
   - 표 범위 밖은 끝값을 쓴다
5. **gain**: BFS는 0/6/10/12/18/24 dB만 쓸 수 있고, LUCID는 아무 gain이나 된다(디지털 스케일).
6. **온도**: 측정 시 LUCID 47~51°C, BFS 56~57°C였다. 많이 다르면 dark current가 달라진다.
7. **트리거 모드(GigE 액션 커맨드)** 확인 결과 (2026-10-20, jsh-3e가 촬영, `/mnt/data/jsh_raw/trigger_dark_check/`)
   - 30 ms 이하는 두 카메라 모두 모델과 맞는다
   - BFS 긴 노출: 측정 때보다 1~2 °C 높아 dark가 늘었다. offset +3.5 DN(300 ms), +12 DN(1 s) @0 dB.
     → 장면 dark로 노출별 offset 이동량을 더하고, 분산 배율은 장면 dark에서 따로 잰다. offset 비율로 분산을 키우면 과대평가된다
   - LUCID 트리거 300 ms 이상: 분산이 +14%다(free-run보다 +7% 더 크다). → 트리거는 ≤150 ms만 쓰고(분산비 ≤1.05), 그 위는 free-run
   - LUCID 1 s에서 검은 레벨이 −8~−9 DN 내려가는 현상이 jsh-3e 드라이버(free-run·트리거 모두)에서 나왔다.
     같은 시각 내 스크립트(스트림 정지 → 프레임레이트 → 노출 → 시작)로 찍은 1 s는 주기 1000.5/1050 ms 모두 정상이었다.
     분산과 픽셀 패턴은 같고 전역 offset만 다르다 → 절차 차이로 보고, 장면 dark로 보정한다
     (`/mnt/data/jsh_raw/trigger_dark_check/20261020_044954_lucid_period_test`)
   - 재부팅 뒤(48 °C) LUCID dark current가 캘리브레이션보다 ~13% 높아, 긴 노출의 dark 분산이 +6~8%다.
     장면 dark의 분산 배율(k_var)로 보정한다
8. 코드 수정 (2026-10-20): 지도의 read σ에는 양자화(step²/12)가 이미 들어 있다. `zscore`/`nll`의 이중 가산과 `sample`의 이중 양자화를 고쳤다
9. **PRNU**(픽셀별 감도 차이)는 넣지 않았다. BFS 기준 약 1%다. 깨끗한 신호 S에 이미 들어 있다고 보는 게 일반적이다.
