"""Does the free-run noise model hold in trigger mode? Compare jsh-3e's trigger darks to the model.

  python trigger_dark_compare.py /mnt/data/jsh_raw/trigger_dark_check/<time> [--bfs-gain 0]

Per camera and exposure (lens cap on, full frame, N frames):
  shift      : CFA mean of the frames - model dark offset (DN, and in units of read sigma)
  mean z     : per-pixel (mean_i - offset_i - shift) / sqrt(var_model_i/N + var_map_i/300); std ~1 if the
               per-pixel offset map holds (map from 300 frames)
  var ratio  : median(temporal var, successive differences) / median(model var)
  z (S=0)    : std of (x - mean_i)/sqrt(var_model_i) and the >4 sigma fraction vs the model's residual table
  hot overlap: share of pixels hot in this data that the model's hot mask also flags
Pass: |shift| < 0.1 read sigma and var ratio 0.95-1.05.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path.home() / "jsh" / "tools"))
from rawdng import read  # noqa: E402

from noise_model import NoiseModel, CFA  # noqa: E402

CALIB = Path(__file__).resolve().parent.parent / "calibration"


def stream(files):
    s = s2 = d2 = prev = None
    n = 0
    for f in files:
        x = read(f)[0].astype(np.float64)
        if s is None:
            s, s2, d2 = np.zeros_like(x), np.zeros_like(x), np.zeros_like(x)
            first = x.copy()
        s += x
        s2 += x * x
        if prev is not None:
            d2 += (x - prev) ** 2
        prev = x
        n += 1
    mean = s / n
    return mean, d2 / (2 * (n - 1)), first, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", type=Path)
    ap.add_argument("--bfs-gain", type=float)
    a = ap.parse_args()
    log = json.loads((a.folder / "log.json").read_text()) if (a.folder / "log.json").exists() else {}
    gains = {"bfs": a.bfs_gain if a.bfs_gain is not None else log.get("bfs_gain_db", 0.0), "lucid": 0.0}
    models = {"bfs": NoiseModel(CALIB / "BFS-PGE-23S3C_26315393"), "lucid": NoiseModel(CALIB / "TRI054S-C_254600231")}
    rows = []
    print(f"{'cam':5s} {'T ms':>7s} {'N':>3s} | {'shift G1 DN':>11s} {'/read σ':>7s} | {'mean z std':>10s} | "
          f"{'var ratio':>9s} | {'z std':>6s} {'>4σ':>9s} {'model >4σ':>9s} | {'hot ovl':>7s}")
    for exp_dir in sorted((p for p in a.folder.iterdir() if p.is_dir()), key=lambda p: int(p.name[:-2])):
        for cam in ("bfs", "lucid"):
            files = sorted(exp_dir.glob(f"*_{cam}.dng"))
            if len(files) < 4:
                continue
            nm, g = models[cam], gains[cam]
            T = float(read(files[0])[1]["meta"].get("exposure_time_us") or int(exp_dir.name[:-2]))
            mean, var_t, first, n = stream(files)
            off, var_m = nm.dark(g, T)
            g0, _ = nm._gain(g)
            m = nm.maps(g0)
            read_sd = np.sqrt(np.median(m["read_sigma_dn"] ** 2))
            out = {"camera": cam, "exposure_us": T, "gain_db": g, "n": n, "cfa": {}}
            for c, (i, j) in CFA.items():
                sl = (slice(i, None, 2), slice(j, None, 2))
                shift = float(np.mean(mean[sl] - off[sl]))
                var_map = (m["read_sigma_dn"][sl].astype(np.float64)) ** 2
                zmean = (mean[sl] - off[sl] - shift) / np.sqrt(var_m[sl] / n + var_map / 300)
                ratio = float(np.median(var_t[sl]) / np.median(var_m[sl]))
                zf = (first[sl] - mean[sl]) / np.sqrt(var_m[sl])
                hot_data = (mean[sl] - np.median(mean[sl])) > 5 * np.sqrt(np.median(var_m[sl]))
                hot_model = m["hot"][sl]
                ovl = float((hot_data & hot_model).sum() / max(hot_data.sum(), 1))
                out["cfa"][c] = {"shift_dn": shift, "shift_over_read_sigma": shift / read_sd,
                                 "mean_z_std": float(1.4826 * np.median(np.abs(zmean - np.median(zmean)))),
                                 "var_ratio": ratio, "z_std": float(np.std(zf)), "z_tail4": float(np.mean(np.abs(zf) > 4)),
                                 "hot_data": int(hot_data.sum()), "hot_overlap": ovl}
            q = nm._shape(g, T)
            model_tail = float(np.interp(-4, q, nm.probs) + 1 - np.interp(4, q, nm.probs))
            out["model_tail4"] = model_tail
            r = out["cfa"]["G1"]
            ok = abs(r["shift_over_read_sigma"]) < 0.1 and 0.95 <= r["var_ratio"] <= 1.05
            out["pass"] = bool(ok)
            rows.append(out)
            print(f"{cam:5s} {T/1000:7.1f} {n:3d} | {r['shift_dn']:11.2f} {r['shift_over_read_sigma']:7.3f} | "
                  f"{r['mean_z_std']:10.2f} | {r['var_ratio']:9.3f} | {r['z_std']:6.3f} {r['z_tail4']:9.1e} {model_tail:9.1e} | "
                  f"{r['hot_overlap']:7.2f}  {'PASS' if ok else 'CHECK'}", flush=True)
    (a.folder / "model_compare.json").write_text(json.dumps(rows, indent=1, default=float))
    print("written:", a.folder / "model_compare.json")


if __name__ == "__main__":
    main()
