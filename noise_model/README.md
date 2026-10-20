# noise_model — 카메라별 RAW 노이즈 모델 (BFS-PGE-23S3C · TRI054S-C)

RGB → 카메라 RAW 복원 모델의 학습·평가에 쓰는 실측 노이즈 모델과, 그것을 만든 측정·분석 코드다.

- 측정 방법, 결과, 한계: **[CALIBRATION_REPORT.md](CALIBRATION_REPORT.md)**
- 측정 당시 작업 기록: [LAB_LOG.md](LAB_LOG.md)
- 패키지 사용법: [calibration/README.md](calibration/README.md)

## 들어 있는 것

| 경로 | 내용 |
|------|------|
| `calibration/BFS-PGE-23S3C_26315393/`, `calibration/TRI054S-C_254600231/` | `noise_model.json`(gain별 파라미터), `residual_quantiles.npz`(dark 분포), `validation_*.json` |
| `scripts/` | 측정(LUCID arena_api, BFS Spinnaker C ctypes), 분석, 패키지 생성, 노이즈 모델, 검증 |
| `ros_preview/` | autoware `lucid_vision_driver` 미리보기용 파라미터와 실행 스크립트(BGR8, RAW 측정용 아님) |
| `env.example.sh` | Arena SDK 경로·venv 설정 예 |

픽셀 지도 `maps_gNN.npz`(카메라당 145~380 MB)와 원본 측정 데이터(약 22 GB)는 git에 넣지 않았다.
외장 번들 `noise_calibration_20261020/`(`calibration_package/`, `data/`, `validation/`)에 있다.
지도를 `calibration/<cam>/`에 복사해 넣으면 `noise_model.py`가 바로 쓴다.

## 쓰는 법

```python
import sys; sys.path.insert(0, "noise_model/scripts")
import numpy as np
from noise_model import NoiseModel

nm = NoiseModel("noise_model/calibration/TRI054S-C_254600231")       # maps_gNN.npz 필요
rng = np.random.default_rng()
raw = nm.sample(S, gain_db=0, exposure_us=30000, rng=rng)             # 학습 데이터 합성
z   = nm.zscore(raw_measured, S_pred, gain_db=0, exposure_us=30000)   # 평가: ~N(0,1)이어야 함
nll = nm.nll(raw_measured, S_pred, gain_db=0, exposure_us=30000)      # 손실/메트릭 (nats/픽셀)
```

`S`는 노이즈 없는 신호(DN, 검은 레벨 위)이고, 전체 화면 RGGB Bayer다.
녹화 설정이 `noise_model.json`의 `settings`와 같아야 한다.
- 노출: LUCID ≤ 800 ms(트리거 ≤ 150 ms), BFS 24 µs ~ 1 s
- 녹화마다 장면 dark로 offset 이동량과 분산 배율을 보정한다(보고서 6장)

## 의존성

- Python 3.10, numpy, scipy, matplotlib
- 측정용: LUCID Arena SDK x64 + `arena_api` wheel, FLIR Spinnaker 4.x(C 라이브러리, PySpin 불필요)
