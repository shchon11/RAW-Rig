"""PRNU from diffuser flats (sheet of paper on the lens, aperture open), per CFA plane.

  python prnu_analyze.py bfs   ../data/prnu_20261020/bfs     # fullframe_dark.py output (frames kept)
  python prnu_analyze.py lucid ../data/prnu_20261020/lucid

Per level: mean image of the stack minus the calibrated dark (offset_1ms + dark_rate*T, which also
removes the DSNU); each CFA plane is divided by a Gaussian-smoothed copy of itself (sigma 8 px in the
plane), which takes out vignetting and the slow illumination shape and leaves the pixel-scale pattern.
PRNU = sqrt(spatial variance of that ratio - what temporal noise still leaves in an n-frame mean).
The white-noise loss of the high-pass (1/(4 pi sigma^2)) is corrected. Hot/noisy pixels are excluded.
Writes <folder>/prnu.json and prnu_map.npz (per-pixel relative gain from all levels combined).
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter

ROOT = Path(__file__).resolve().parent.parent
CAL = {"bfs": ROOT / "calibration/BFS-PGE-23S3C_26315393", "lucid": ROOT / "calibration/TRI054S-C_254600231"}
POS = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}
SIGMA = 8.0
CLIP = {"bfs": 65500, "lucid": 2 ** 24 - 1}


def main():
    cam, folder = sys.argv[1], Path(sys.argv[2])
    out = {"sigma_px": SIGMA, "levels": []}
    acc, wsum = None, None
    for f in sorted(folder.glob("g*_t*us.npz"), key=lambda p: float(p.stem.split("_t")[1][:-2])):
        info = json.loads(f.with_suffix(".json").read_text())
        g, T = int(round(info["gain_db"])), info["exposure_us"]
        maps = np.load(CAL[cam] / f"maps_g{g:02d}.npz")
        model = json.loads((CAL[cam] / "noise_model.json").read_text())["gains"][str(g)]
        z = np.load(f)
        st = z["frames"]
        n = st.shape[0]
        dark = maps["offset_1ms_dn"] + maps["dark_rate_dn_per_s"] * (T - 1000) / 1e6
        x = st.astype(np.float64) - dark
        g_fr = x.reshape(n, -1).mean(1)
        x /= (g_fr / g_fr.mean())[:, None, None]                    # light ripple
        mean = x.mean(0)
        vt = x.var(0, ddof=1)
        bad = maps["hot"] | maps["noisy"] | (st >= CLIP[cam]).any(0)
        rel = np.full(mean.shape, np.nan)
        lev = {"exposure_us": T, "gain_db": g, "n": n}
        H, W = mean.shape
        for c, (dy, dx) in POS.items():
            p = mean[dy::2, dx::2]
            b = bad[dy::2, dx::2]
            pf = np.where(b, np.nan, p)
            # NaN-aware smoothing of the plane
            w = gaussian_filter((~b).astype(float), SIGMA)
            sm = gaussian_filter(np.nan_to_num(pf), SIGMA) / np.maximum(w, 1e-6)
            r = p / sm
            m = ~b & (w > 0.5)
            m[:3 * int(SIGMA)] = m[-3 * int(SIGMA):] = False          # drop the borders of the filter
            m[:, :3 * int(SIGMA)] = m[:, -3 * int(SIGMA):] = False
            hp_loss = 1 - 1 / (4 * np.pi * SIGMA ** 2)
            v_spatial = np.var(r[m]) / hp_loss
            v_temporal = np.mean(vt[dy::2, dx::2][m] / sm[m] ** 2) / n
            prnu = np.sqrt(max(v_spatial - v_temporal, 0))
            lev[c] = {"signal_dn": float(np.nanmean(p[m])), "signal_frac_of_white": float(np.nanmean(p[m]) / CLIP[cam]),
                      "prnu_pct": 100 * prnu, "temporal_part_pct": 100 * np.sqrt(v_temporal),
                      "alpha": model["alpha_dn_per_e"][c]}
            rel[dy::2, dx::2] = np.where(m, r, np.nan)
        out["levels"].append(lev)
        wgt = np.mean([lev[c]["signal_dn"] for c in POS])
        acc = rel * wgt if acc is None else np.nansum([acc, rel * wgt], axis=0)
        wsum = wgt if wsum is None else wsum + wgt
        print(f"{f.stem}: " + "  ".join(f"{c} {lev[c]['signal_frac_of_white']:.0%} of white, PRNU {lev[c]['prnu_pct']:.3f}% "
                                         f"(temporal residue {lev[c]['temporal_part_pct']:.3f}%)" for c in POS), flush=True)
    (folder / "prnu.json").write_text(json.dumps(out, indent=1))
    np.savez_compressed(folder / "prnu_map.npz", relative_gain=(acc / wsum).astype(np.float32))
    print("->", folder / "prnu.json")


if __name__ == "__main__":
    main()
