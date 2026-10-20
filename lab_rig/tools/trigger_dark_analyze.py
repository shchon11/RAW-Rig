"""트리거 모드 dark 를 jsh-16 노이즈 모델과 대조한다 (2026-10-20 합의한 판정: 이동량 < 0.1σ, 분산비 0.95~1.05).

    ~/jsh/.venv-dng/bin/python ~/jsh/tools/trigger_dark_analyze.py /mnt/data/jsh_raw/trigger_dark_check/<run> [...]

노출 · 카메라마다 (hot/noisy 마스크 밖 픽셀, CFA 별):
  shift      CFA 평균(시간 평균 − 모델 offset) [DN], 그리고 /σ (σ = 모델 dark σ 의 중앙값)
  var_ratio  픽셀별 시간 분산 / 모델 분산 의 중앙값
  zmean      (픽셀 시간 평균 − 모델 offset) / (σ_i/√N) 의 평균 · 표준편차 (모델 지도가 픽셀 offset 을 맞히면 std ≈ 1)
  z0         zscore(S=0) 전체의 평균 · 분산 (모든 프레임)
  hot        실측 hot(평균이 CFA 중앙값보다 6σ 이상 높음) 과 모델 hot 마스크의 겹침 (Jaccard)
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path.home() / "jsh/imx490_ptc/scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import rawdng  # noqa: E402
from noise_model import NoiseModel  # noqa: E402

CAL = Path.home() / "jsh/imx490_ptc/calibration"
MODELS = {"bfs": CAL / "BFS-PGE-23S3C_26315393", "lucid": CAL / "TRI054S-C_254600231"}
CFA = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}


def analyze_step(files, nm, gain, exposure_us):
    offset, dvar = nm.dark(gain, exposure_us)
    var_model = dvar          # read_sigma 지도가 실측이라 양자화가 이미 들어 있다 (jsh-16 확인, 2026-10-20)
    g0, _ = nm._gain(gain)
    maps = nm.maps(g0)
    bad = maps["hot"] | maps["noisy"]
    good = ~bad
    s = np.zeros(offset.shape)
    ss = np.zeros(offset.shape)
    z0_sum = z0_sq = 0.0
    z0_n = 0
    inv_sigma = 1.0 / np.sqrt(var_model)
    for f in files:
        raw, _info = rawdng.read(f)
        x = raw.astype(np.float64)
        s += x
        ss += x * x
        z = (x - offset) * inv_sigma
        zg = z[good]
        z0_sum += float(zg.sum())
        z0_sq += float((zg * zg).sum())
        z0_n += zg.size
    n = len(files)
    mean = s / n
    var_t = (ss - n * mean * mean) / (n - 1)
    out = {"frames": n}
    for c, (i, j) in CFA.items():
        g = good[i::2, j::2]
        d = (mean - offset)[i::2, j::2][g]
        vm = var_model[i::2, j::2][g]
        sigma = float(np.sqrt(np.median(vm)))
        zm = d / np.sqrt(vm / n)
        mean_c = mean[i::2, j::2]
        hot_meas = (mean_c - np.median(mean_c)) > 6.0 * sigma
        hot_model = maps["hot"][i::2, j::2]
        union = int((hot_meas | hot_model).sum())
        out[c] = {"shift_dn": float(d.mean()), "shift_over_sigma": float(d.mean()) / sigma, "sigma_dn": sigma,
                  "var_ratio": float(np.median(var_t[i::2, j::2][g] / vm)),
                  "zmean_mean": float(zm.mean()), "zmean_std": float(zm.std()),
                  "hot_measured": int(hot_meas.sum()), "hot_model": int(hot_model.sum()),
                  "hot_jaccard": float((hot_meas & hot_model).sum()) / union if union else 1.0}
    z0_mean = z0_sum / z0_n
    out["z0_mean"] = z0_mean
    out["z0_var"] = z0_sq / z0_n - z0_mean ** 2
    return out


def main(argv):
    report = {}
    for run in map(Path, argv[1:]):
        log = json.loads((run / "log.json").read_text())
        for step_dir in sorted(run.glob("*us"), key=lambda p: int(p.name[:-2])):
            exposure_us = int(step_dir.name[:-2])
            for cam in log["cameras"]:
                files = sorted(step_dir.glob(f"*_{cam}.dng"))
                if not files:
                    continue
                gain = float(log["bfs_gain_db"]) if cam == "bfs" else float(log.get("lucid_gain_db", 0.0))
                r = analyze_step(files, NoiseModel(MODELS[cam]), gain, exposure_us)
                key = f"{cam} {gain:g}dB {exposure_us / 1000:g}ms"
                report[key] = r
                worst_shift = max(abs(r[c]["shift_over_sigma"]) for c in CFA)
                ratios = [r[c]["var_ratio"] for c in CFA]
                ok = worst_shift < 0.1 and all(0.95 <= v <= 1.05 for v in ratios)
                print(f"{key:22s} shift/σ max {worst_shift:.3f} (DN " +
                      " ".join(f"{r[c]['shift_dn']:+.2f}" for c in CFA) + ") | var ratio " +
                      " ".join(f"{v:.3f}" for v in ratios) + f" | zmean std {np.mean([r[c]['zmean_std'] for c in CFA]):.2f}"
                      f" | z0 {r['z0_mean']:+.3f}/{r['z0_var']:.3f} | hot J {np.mean([r[c]['hot_jaccard'] for c in CFA]):.2f}"
                      f" | {'OK' if ok else 'CHECK'}", flush=True)
        (run / "analysis.json").write_text(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
