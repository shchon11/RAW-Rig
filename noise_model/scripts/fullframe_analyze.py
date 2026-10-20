"""Dark-noise structure from fullframe_dark.py output (per camera folder).

  python fullframe_analyze.py ../data/fullframe_dark_20261020/bfs [--alpha-json PTC_SESSION/analysis]

Per condition (gain x exposure), per CFA:
  offset (median of the per-pixel mean), DSNU (robust spatial std of the mean map, minus the
  temporal share), temporal sigma (from successive differences), percentiles of per-pixel sigma,
  hot pixels (mean - median > 5 median sigma), noisy pixels (sigma > 2 median sigma),
  row / column temporal noise (variance of row/col means beyond the pixel-noise share) and
  row / column FPN, neighbour correlation of the temporal residual, and the residual
  distribution against N(0,1): kurtosis, tail fractions, KS statistic, Tukey-lambda (PPCC).
Across exposures: offset and variance slopes (dark current, DN/s and e-/s with --alpha-json).
Writes <folder>/analysis/summary.json and plots.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy import stats

CFA = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}


def robust_std(x):
    return 1.4826 * np.median(np.abs(x - np.median(x)))


def analyze(npz, n_frames):
    z = np.load(npz)
    mean, var_d, frames = z["mean"].astype(np.float64), z["var_diff"].astype(np.float64), z["frames"].astype(np.float64)
    rows, cols = z["row_means"].astype(np.float64), z["col_means"].astype(np.float64)
    H, W = mean.shape
    out = {"cfa": {}}
    resid = frames - mean[None]                       # temporal residual of the kept frames
    for c, (i, j) in CFA.items():
        m, v = mean[i::2, j::2], var_d[i::2, j::2]
        sig = np.sqrt(v)
        med_sig = float(np.median(sig))
        med_m = float(np.median(m))
        hot = (m - med_m) > 5 * med_sig
        noisy = sig > 2 * med_sig
        good = ~hot & ~noisy
        dsnu = float(np.sqrt(max(robust_std(m[good]) ** 2 - med_sig ** 2 / n_frames, 0)))
        col_fpn = float(np.std(np.median(m, axis=0)))
        row_fpn = float(np.std(np.median(m, axis=1)))
        r = resid[:, i::2, j::2]
        rg = (r / np.where(sig > 0, sig, 1))[:, good]   # standardized by each pixel's own sigma
        sample = rg.ravel()
        rng = np.random.default_rng(0)
        sub = rng.choice(sample, size=min(200_000, sample.size), replace=False)
        ks = stats.kstest(sub, "norm")
        try:
            lam = float(stats.ppcc_max(sub[:50_000], brack=(-0.5, 0.5), dist="tukeylambda"))
        except Exception:
            lam = float("nan")
        h_res = r[:, :, 1:] - 0  # same-colour neighbours are 2 pixels apart in the full frame
        corr_h = float(np.corrcoef(r[:, :, :-1].ravel()[:2_000_000], r[:, :, 1:].ravel()[:2_000_000])[0, 1])
        corr_v = float(np.corrcoef(r[:, :-1, :].ravel()[:2_000_000], r[:, 1:, :].ravel()[:2_000_000])[0, 1])
        out["cfa"][c] = {
            "offset_dn": med_m, "temporal_sigma_dn": med_sig, "dsnu_dn": dsnu,
            "sigma_percentiles_dn": {p: float(np.percentile(sig, p)) for p in (1, 50, 90, 99, 99.9)},
            "hot_pixels": int(hot.sum()), "noisy_pixels": int(noisy.sum()), "pixels": int(m.size),
            "row_fpn_dn": row_fpn, "col_fpn_dn": col_fpn,
            "kurtosis_excess": float(stats.kurtosis(sub)),
            "tail_frac": {k: float(np.mean(np.abs(sample) > k)) for k in (3, 4, 5)},
            "tail_frac_gauss": {k: float(2 * stats.norm.sf(k)) for k in (3, 4, 5)},
            "ks_stat": float(ks.statistic), "ks_p": float(ks.pvalue), "tukey_lambda": lam,
            "corr_same_colour_h": corr_h, "corr_same_colour_v": corr_v,
        }
    # row / column temporal noise over all pixels (beyond what pixel noise alone gives)
    sig_all2 = float(np.median(var_d))
    rows_t = rows - rows.mean(0, keepdims=True)
    cols_t = cols - cols.mean(0, keepdims=True)
    out["row_temporal_sigma_dn"] = float(np.sqrt(max(np.mean(np.var(rows_t, axis=0, ddof=1)) - sig_all2 / W, 0)))
    out["col_temporal_sigma_dn"] = float(np.sqrt(max(np.mean(np.var(cols_t, axis=0, ddof=1)) - sig_all2 / H, 0)))
    out["pixel_sigma_dn"] = float(np.sqrt(sig_all2))
    full = resid[:, :, :]
    out["corr_adjacent_h"] = float(np.corrcoef(full[:, :, :-1].ravel()[:2_000_000], full[:, :, 1:].ravel()[:2_000_000])[0, 1])
    out["corr_adjacent_v"] = float(np.corrcoef(full[:, :-1, :].ravel()[:2_000_000], full[:, 1:, :].ravel()[:2_000_000])[0, 1])
    return out


def alpha_for(alpha_dir, camera, gain):
    if not alpha_dir:
        return None
    name = ("BFS" if camera == "bfs" else "HDR") + ("" if gain == 0 else f"_g{gain:02.0f}")
    p = Path(alpha_dir) / f"{name}.json"
    if not p.exists():
        return None
    r = json.loads(p.read_text())
    a = [r["cfa"][c].get("alpha_dn_per_e") for c in ("G1", "G2")]
    if camera == "lucid":  # HDR: use the SP1-range local alpha (dark-range scale)
        a = [np.nanmedian([lv["alpha_local"] or np.nan for lv in r["cfa"][c]["levels"]
                           if lv["S"] and lv["S"] < 2e4 * 10 ** (gain / 20)]) for c in ("G1", "G2")]
    return float(np.mean(a)) if all(a) else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", type=Path)
    ap.add_argument("--alpha-json", help="PTC analysis folder for e- conversion")
    a = ap.parse_args()
    camera = a.folder.name
    out_dir = a.folder / "analysis"
    out_dir.mkdir(exist_ok=True)
    conds = []
    for js in sorted(a.folder.glob("g*_t*us.json")):
        info = json.loads(js.read_text())
        res = analyze(js.with_suffix(".npz"), info["n"])
        conds.append({"tag": js.stem, "gain_db": round(info["gain_db"]), "exposure_us": info["exposure_us"],
                      "temperature_c": info["temperature_c"], "frame_id_gaps": info["frame_id_gaps"],
                      "crc_ok": info["crc_ok"], "n": info["n"], **res})
        g = res["cfa"]["G1"]
        print(f"{js.stem:20s} off {g['offset_dn']:8.1f}  σ {g['temporal_sigma_dn']:7.2f}  dsnu {g['dsnu_dn']:6.2f}  "
              f"hot {g['hot_pixels']:5d}  noisy {g['noisy_pixels']:5d}  row σ {res['row_temporal_sigma_dn']:5.2f}  "
              f"col σ {res['col_temporal_sigma_dn']:5.2f}  kurt {g['kurtosis_excess']:+6.3f}  "
              f">4σ {g['tail_frac'][4]:.1e} (gauss {g['tail_frac_gauss'][4]:.1e})  λ {g['tukey_lambda']:+.3f}  "
              f"ρh {g['corr_same_colour_h']:+.3f}", flush=True)
    summary = {"camera": camera, "conditions": conds, "dark_vs_exposure": {}}
    for gain in sorted({c["gain_db"] for c in conds}):
        cs = sorted([c for c in conds if c["gain_db"] == gain], key=lambda c: c["exposure_us"])
        t = np.array([c["exposure_us"] for c in cs]) * 1e-6
        alpha = alpha_for(a.alpha_json, camera, gain)
        per = {}
        for ch in CFA:
            off = np.array([c["cfa"][ch]["offset_dn"] for c in cs])
            var = np.array([c["cfa"][ch]["temporal_sigma_dn"] ** 2 for c in cs])
            so, sv = np.polyfit(t, off, 1)[0], np.polyfit(t, var, 1)[0]
            per[ch] = {"offset_slope_dn_per_s": float(so), "var_slope_dn2_per_s": float(sv),
                       "read_sigma_dn_t0": float(np.sqrt(np.polyfit(t, var, 1)[1])),
                       "dark_e_per_s_from_offset": float(so / alpha) if alpha else None,
                       "dark_e_per_s_from_var": float(sv / alpha ** 2) if alpha else None}
        summary["dark_vs_exposure"][str(gain)] = {"alpha_dn_per_e": alpha, "cfa": per}
        g = per["G1"]
        print(f"gain {gain:3d} dB  α {alpha}  offset +{g['offset_slope_dn_per_s']:.1f} DN/s  var +{g['var_slope_dn2_per_s']:.1f} DN²/s"
              + (f"  dark current {g['dark_e_per_s_from_offset']:.1f} e-/s (offset) / {g['dark_e_per_s_from_var']:.1f} e-/s (var)" if alpha else ""))
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=1))
    print("written:", out_dir / "summary.json")


if __name__ == "__main__":
    main()
