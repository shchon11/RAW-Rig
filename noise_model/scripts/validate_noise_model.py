"""Check noise_model.py against measured PTC frames (statistically, per level).

  python validate_noise_model.py <calib_dir> <ptc_session> <output> --gain-db G --roi Y0 X0 [--tag name]

Per bright level and CFA (G1 shown):
  real  : per-pair variance of (A-B) after removing a quadratic surface (light ripple), /2
  model : nm.variance(S_i) averaged over pixels, S_i = per-pixel mean - modelled dark offset
  sim   : the same statistic on frames drawn with nm.sample(S_i)
  z     : (real - model) / SE(real), SE from the pair-to-pair scatter
  zscore: std of nm.zscore on detrended pair differences (should be 1), KS vs N(0,1)
and the dark offset of the session vs the model (black-level stability across time/temperature).
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy import stats

from noise_model import NoiseModel
from ptc_analyze import design


def detrended_pairs(x, A, Ap):
    """x [n, npix] -> residual pair differences [pairs, npix] with a fitted surface removed."""
    n = x.shape[0] - x.shape[0] % 2
    d = x[0:n:2] - x[1:n:2]
    return d - (d @ Ap.T) @ A.T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("calib", type=Path)
    ap.add_argument("session", type=Path)
    ap.add_argument("output")
    ap.add_argument("--gain-db", type=float, default=0.0)
    ap.add_argument("--roi", type=int, nargs=2, required=True, help="Y0 X0 of the 256x256 PTC ROI")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    rec = json.loads((a.session / "manifest.json").read_text())["outputs"][a.output]
    folder = a.session / a.output
    nm = NoiseModel(a.calib, roi=(a.roi[0], a.roi[1], 256, 256))
    rng = np.random.default_rng(1)
    i, j = 0, 1                                   # G1
    yy, xx = np.mgrid[0:128, 0:128]
    A, Ap = design(yy.ravel().astype(float), xx.ravel().astype(float), 2)
    rows = []
    d0 = np.load(folder / rec["dark"][0]["file"])["stack"].astype(float)
    off_model, _ = nm.dark(a.gain_db, rec["dark"][0]["exposure_us"])
    print(f"dark offset (shortest exposure, G1): session {d0[:, i::2, j::2].mean():.2f}  model {off_model[i::2, j::2].mean():.2f}")
    print(f"{'T ms':>9s} {'S G1':>10s} | {'real var':>11s} {'model var':>11s} {'sim var':>11s} | {'z':>6s} {'real/model':>10s} | {'std(z)':>6s} {'KS':>6s}")
    for lv in rec["bright"]:
        T = lv["exposure_us"]
        x = np.load(folder / lv["file"])["stack"].astype(float)
        S = nm.signal(x.mean(0), a.gain_db, T)
        sat = x.max() >= nm.white - nm.step
        if sat:
            continue
        g = x[:, i::2, j::2].reshape(len(x), -1)
        d = detrended_pairs(g, A, Ap)
        vp = (d ** 2).sum(1) / (d.shape[1] - A.shape[1]) / 2
        real, se = vp.mean(), vp.std(ddof=1) / np.sqrt(vp.size)
        var_m = nm.variance(S, a.gain_db, T)[i::2, j::2].ravel()
        sim = nm.sample(S, a.gain_db, T, rng, n_frames=len(x))[:, i::2, j::2].reshape(len(x), -1)
        ds = detrended_pairs(sim, A, Ap)
        sim_v = ((ds ** 2).sum(1) / (ds.shape[1] - A.shape[1]) / 2).mean()
        model = float(var_m.mean())
        zz = (d / np.sqrt(2 * var_m)[None]).ravel()
        row = {"exposure_us": T, "S": float(S[i::2, j::2].mean()), "real_var": float(real), "real_se": float(se),
               "model_var": model, "sim_var": float(sim_v), "z": float((real - model) / se),
               "ratio": float(real / model), "std_z": float(zz.std()),
               "ks": float(stats.kstest(zz[:500_000], "norm").statistic)}
        rows.append(row)
        print(f"{T/1000:9.3f} {row['S']:10.1f} | {real:11.2f} {model:11.2f} {sim_v:11.2f} | {row['z']:+6.1f} {row['ratio']:10.3f} | "
              f"{row['std_z']:6.3f} {row['ks']:6.3f}", flush=True)
    r = np.array([x["ratio"] for x in rows])
    print(f"real/model variance over {len(r)} levels: median {np.median(r):.3f}, min {r.min():.3f}, max {r.max():.3f}")
    out = a.calib / f"validation_{a.tag or a.session.name + '_' + a.output}.json"
    out.write_text(json.dumps({"session": str(a.session), "output": a.output, "gain_db": a.gain_db, "levels": rows}, indent=1))
    print("written:", out)


if __name__ == "__main__":
    main()
