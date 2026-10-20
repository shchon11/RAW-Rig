"""Compare fullframe_dark.py outputs (npz) of LUCID/BFS with the calibrated dark model.

  python dark_npz_compare.py <folder with */lucid/g00_t*us.npz or lucid/*.npz> [--camera lucid]

Per set: frame period, temperature, shift (CFA mean - model offset, DN and / read sigma),
variance ratio mean(var_diff)/mean(model var) over pixels outside the calibration hot/noisy masks
(unbiased, no distribution assumption), robust std of per-pixel mean z.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from noise_model import CFA, NoiseModel

CALIB = Path(__file__).resolve().parent.parent / "calibration"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", type=Path)
    ap.add_argument("--camera", default="lucid", choices=["lucid", "bfs"])
    a = ap.parse_args()
    nm = NoiseModel(CALIB / ("TRI054S-C_254600231" if a.camera == "lucid" else "BFS-PGE-23S3C_26315393"))
    print(f"{'set':34s} {'period ms':>9s} {'temp':>6s} | {'shift G1':>8s} {'/σ':>6s} | {'var ratio':>9s} | {'mean z std':>10s}")
    rows = []
    for js in sorted(a.folder.rglob(f"{a.camera}/g*_t*us.json")):
        info = json.loads(js.read_text())
        z = np.load(js.with_suffix(".npz"))
        g, T, n = info["gain_db"], info["exposure_us"], info["n"]
        off, var_m = nm.dark(g, T)
        g0, _ = nm._gain(g)
        m = nm.maps(g0)
        good = ~(m["hot"] | m["noisy"])
        mean, vd = z["mean"].astype(np.float64), z["var_diff"].astype(np.float64)
        read_sd = float(np.sqrt(np.median(m["read_sigma_dn"] ** 2)))
        res = {"set": str(js.relative_to(a.folder)), "period_ms": 1000 / info["frame_rate_hz"], "exposure_us": T,
               "temperature_c": info["temperature_c"], "cfa": {}}
        for c, (i, j) in CFA.items():
            sl = (slice(i, None, 2), slice(j, None, 2))
            gd = good[sl]
            shift = float(np.mean((mean[sl] - off[sl])[gd]))
            ratio = float(np.mean(vd[sl][gd]) / np.mean(var_m[sl][gd]))
            zm = ((mean[sl] - off[sl] - shift) / np.sqrt(var_m[sl] / n + m["read_sigma_dn"][sl].astype(np.float64) ** 2 / 300))[gd]
            res["cfa"][c] = {"shift_dn": shift, "shift_over_read_sigma": shift / read_sd, "var_ratio": ratio,
                             "mean_z_std": float(1.4826 * np.median(np.abs(zm - np.median(zm))))}
        r = res["cfa"]["G1"]
        rows.append(res)
        print(f"{res['set']:34s} {res['period_ms']:9.1f} {np.mean(info['temperature_c']):6.2f} | {r['shift_dn']:8.2f} "
              f"{r['shift_over_read_sigma']:6.2f} | {r['var_ratio']:9.3f} | {r['mean_z_std']:10.2f}")
    (a.folder / "model_compare.json").write_text(json.dumps(rows, indent=1, default=float))


if __name__ == "__main__":
    main()
