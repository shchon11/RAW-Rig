"""Analyze a ptc_capture.py session: per output and CFA channel, conversion gain, read noise,
linearity and model checks (FLIR_control docs/raw_capture_noise_calibration.md §6.4-6.6).

  python ptc_analyze.py ../data/ptc_<name>          # writes <session>/analysis/

Model per pixel i:  RAW_i = o_i(T) + S_i + e_i,  Var(e_i) = alpha_c * S_i + v_dark(T)
alpha in stored DN per e-. Pair variance follows EMVA 1288: mean((A-B)^2)/2 minus the squared
pair-mean difference / 2, so a uniform illumination step between the two frames cancels.
For the combined HDR output a single alpha is not expected; see alpha_local per level.
"""
import argparse
import json
from pathlib import Path

import numpy as np

CFA = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}
PAIR_REL_TOL = 1e-3       # |mean(A)-mean(B)| / S, FLIR_control §6.4
HOT_SIGMA = 10.0


def load(folder, name):
    return np.load(folder / name)["stack"].astype(np.float64)


def pair_var(x, keep=None):
    """x [n, h, w] for one CFA -> per-pair EMVA variance array [n_pairs]."""
    n = x.shape[0] - x.shape[0] % 2
    d = x[0:n:2] - x[1:n:2]
    v = (np.mean(d ** 2, axis=(1, 2)) - np.mean(d, axis=(1, 2)) ** 2) / 2
    return v if keep is None else v[keep]


def design(yy, xx, order):
    """Polynomial surface basis (up to `order`) at pixel coordinates -> (A, pinv(A))."""
    y = yy / max(yy.max(), 1) - 0.5
    x = xx / max(xx.max(), 1) - 0.5
    cols = [np.ones_like(x)]
    for p in range(1, order + 1):
        cols += [x ** (p - q) * y ** q for q in range(p + 1)]
    A = np.stack(cols, 1)
    return A, np.linalg.pinv(A)


def pair_var_detrended(x, A, Ap):
    """x [n, npix] -> per-pair variance of (A-B) after removing a fitted surface.
    A spatially non-uniform illumination ripple between the two frames is smooth, so it is
    removed with the surface; white temporal noise loses only k of npix degrees of freedom."""
    n = x.shape[0] - x.shape[0] % 2
    d = x[0:n:2] - x[1:n:2]
    r = d - (d @ Ap.T) @ A.T
    npix, k = A.shape
    return np.sum(r ** 2, axis=1) / (npix - k) / 2


def wls_alpha(S, vs, vb, vd, npix, npairs):
    w = npix * npairs / (2 * (vb ** 2 + vd ** 2))
    a = np.sum(w * S * vs) / np.sum(w * S ** 2)
    return float(a), float(1 / np.sqrt(np.sum(w * S ** 2)))


def analyze_output(session, output, rec, white_override=None, detrend=2):
    folder = session / output
    bright, dark = rec.get("bright", []), rec.get("dark", [])
    if not bright or len(dark) != len(bright):
        return {"error": f"bright {len(bright)} / dark {len(dark)} levels; run both phases"}
    T = np.array([lv["exposure_us"] for lv in bright])
    white = white_override or rec.get("white_level")
    nlev = len(T)

    # hot-pixel mask from the longest dark level
    d_long = load(folder, dark[-1]["file"]).mean(0)
    mask = {}
    for c, (i, j) in CFA.items():
        x = d_long[i::2, j::2]
        med = np.median(x)
        rstd = max(1.4826 * np.median(np.abs(x - med)), 0.5)
        mask[c] = (x - med) <= HOT_SIGMA * rstd
    basis = {}
    for c, m in mask.items():
        yy, xx = np.nonzero(m)
        basis[c] = design(yy.astype(float), xx.astype(float), detrend)

    rows = {c: [] for c in CFA}
    qa = {"levels": []}
    for k in range(nlev):
        B = load(folder, bright[k]["file"])
        D = load(folder, dark[k]["file"])
        npairs = B.shape[0] // 2
        o_all = D.mean(0)
        # pair QA on the whole ROI (illumination stability)
        mB = B.reshape(B.shape[0], -1).mean(1)
        S_all = float(mB.mean() - o_all.mean())
        vb_all = float(np.median(pair_var(B)))
        dm = np.abs(mB[0:2 * npairs:2] - mB[1:2 * npairs:2])
        sigma_dm = np.sqrt(2 * vb_all / B[0].size)
        keep = dm <= np.maximum(PAIR_REL_TOL * abs(S_all), 4 * sigma_dm)
        flicker = bool(keep.sum() < npairs / 2)
        if flicker:  # light ripple: the EMVA pair-mean subtraction removes the uniform part, keep all pairs
            keep = np.ones_like(keep)
        top = B.max()
        sat_frac = float(np.mean(B >= white - 4)) if white else float(np.mean(B == top))
        saturated = sat_frac > 1e-3 and (white is not None or top > o_all.mean() + 50 * np.sqrt(vb_all))
        ids = bright[k].get("frame_ids", [])
        gaps = int(np.sum(np.diff(ids) != 1)) if len(ids) > 1 else 0
        qa["levels"].append({"k": k, "exposure_us": float(T[k]), "pairs_kept": int(keep.sum()), "pairs": int(npairs),
                             "flicker": flicker, "pair_mean_rel_diff_median": float(np.median(dm) / max(abs(S_all), 1e-9)),
                             "saturated": bool(saturated), "sat_frac": sat_frac, "frame_id_gaps": gaps,
                             "crc_ok": bright[k].get("crc_ok"), "n": bright[k].get("n"),
                             "dark_crc_ok": dark[k].get("crc_ok")})
        for c, (i, j) in CFA.items():
            m = mask[c]
            b = B[:, i::2, j::2][:, m]
            d = D[:, i::2, j::2][:, m]
            o = d.mean(0)
            vb_p = pair_var_detrended(b, *basis[c])
            vd_p = pair_var_detrended(d, *basis[c])
            S = float(np.mean(b.mean(0) - o))
            vb = float(vb_p[keep].mean()) if keep.any() else np.nan
            vd = float(vd_p.mean())
            ev, od = keep.copy(), keep.copy()
            ev[1::2] = False
            od[0::2] = False
            kb = vb_p[keep]
            rows[c].append({"S": S, "vb": vb, "vd": vd, "vs": vb - vd, "offset": float(o.mean()),
                            # empirical SE from the pair-to-pair scatter (includes light ripple etc.)
                            "vb_se": float(kb.std(ddof=1) / np.sqrt(kb.size)) if kb.size > 1 else np.nan,
                            "vd_se": float(vd_p.std(ddof=1) / np.sqrt(vd_p.size)),
                            "npix": int(m.sum()), "npairs": int(keep.sum()),
                            "vb_even": float(vb_p[ev].mean()) if ev.any() else np.nan,
                            "vb_odd": float(vb_p[od].mean()) if od.any() else np.nan,
                            "vd_even": float(vd_p[0::2].mean()), "vd_odd": float(vd_p[1::2].mean()),
                            "saturated": bool(saturated), "dsnu_dn": float(d.mean(0).std()),
                            "spatial_nonuniformity": float((b.mean(0) - o).std() / max(S, 1e-9))})

    result = {"output": output, "white_level": white, "exposure_us": T.tolist(), "qa": qa, "cfa": {},
              "pair_detrend_order": detrend}
    result["qa"]["hot_pixels"] = {c: int((~mask[c]).sum()) for c in CFA}
    result["qa"]["pairs_rejected_frac"] = 1 - sum(l["pairs_kept"] for l in qa["levels"]) / max(1, sum(l["pairs"] for l in qa["levels"]))
    if "drift" in rec:
        k = int(np.argmin(np.abs(T - rec["drift"].get("exposure_us", T[-1]))))
        drift = load(folder, rec["drift"]["file"]).mean()
        ref = load(folder, bright[k]["file"]).mean()
        o0 = load(folder, dark[k]["file"]).mean()
        result["qa"]["drift_rel"] = float((drift - ref) / max(ref - o0, 1e-9))

    for c in CFA:
        r = rows[c]
        S = np.array([x["S"] for x in r])
        vb = np.array([x["vb"] for x in r])
        vd = np.array([x["vd"] for x in r])
        vs = vb - vd
        npix = r[0]["npix"]
        npairs = np.array([max(x["npairs"], 1) for x in r])
        sat = np.array([x["saturated"] for x in r])
        offset = r[0]["offset"]
        sd0 = np.sqrt(vd[0])
        hi = 0.7 * (white - offset) if white else np.inf
        ok = (~sat) & np.isfinite(vb) & (vs > 0) & (S <= hi)
        sel = ok & (S >= 20 * sd0)
        out = {"offset_dn_tmin": offset, "dark_var_dn2_tmin": float(vd[0]), "read_noise_dn": float(sd0),
               "levels": [{k2: x[k2] for k2 in ("S", "vb", "vd", "vs", "offset", "npairs", "saturated",
                                                  "dsnu_dn", "spatial_nonuniformity")} for x in r]}
        for x, lv in zip(out["levels"], r):
            x["alpha_local"] = lv["vs"] / lv["S"] if lv["S"] > 0 else None
        if sel.sum() >= 3:
            a1, _ = wls_alpha(S[sel], vs[sel], vb[sel], vd[sel], npix, npairs[sel])
            # >= 10 e- and >= 20 sigma_dark: for a low-gain channel (ch3: 10 e- = 0.26 DN) the e- floor
            # alone would admit levels far below the read noise
            sel = ok & (S >= max(10 * a1, 20 * sd0))
            if sel.sum() < 3:
                sel = ok & (S >= 20 * sd0)
            a, se = wls_alpha(S[sel], vs[sel], vb[sel], vd[sel], npix, npairs[sel])
            ve = np.array([x["vb_even"] - x["vd_even"] for x in r])[sel]
            vo = np.array([x["vb_odd"] - x["vd_odd"] for x in r])[sel]
            a_e, _ = wls_alpha(S[sel], ve, vb[sel], vd[sel], npix, npairs[sel] / 2)
            a_o, _ = wls_alpha(S[sel], vo, vb[sel], vd[sel], npix, npairs[sel] / 2)
            sl = np.polyfit(np.log(S[sel]), np.log(np.sqrt(vs[sel])), 1)[0] if sel.sum() >= 3 else np.nan
            resid = np.abs(vb[sel] - (a * S[sel] + vd[sel])) / vb[sel]
            # statistical test of Var = alpha*S + v_d: z per level from the empirical SEs, chi2 over levels
            from scipy.stats import chi2 as chi2_dist
            se_b = np.array([x["vb_se"] for x in r])[sel]
            se_d = np.array([x["vd_se"] for x in r])[sel]
            z = (vb[sel] - (a * S[sel] + vd[sel])) / np.sqrt(se_b ** 2 + se_d ** 2 + (S[sel] * se) ** 2)
            dof = max(int(sel.sum()) - 1, 1)
            out["model_test"] = {"z": [float(v) for v in z], "S": [float(v) for v in S[sel]],
                                 "chi2": float(np.sum(z ** 2)), "dof": dof,
                                 "p_value": float(chi2_dist.sf(np.sum(z ** 2), dof)),
                                 "max_abs_z": float(np.abs(z).max())}
            # local alpha only where v_s is known to 2% (else the spread is just sampling noise)
            rel_se = np.sqrt(2 * (vb ** 2 + vd ** 2) / (npix * npairs)) / np.where(vs > 0, vs, np.inf)
            precise = sel & (rel_se < 0.02)
            loc = vs[precise] / S[precise]
            out.update({"alpha_dn_per_e": a, "alpha_se_wls": se, "alpha_se_empirical": abs(a_e - a_o) / 2,
                        "read_noise_e": float(sd0 / a), "n_fit_levels": int(sel.sum()),
                        "fit_S_range_dn": [float(S[sel].min()), float(S[sel].max())],
                        "shot_slope": float(sl), "max_model_residual": float(resid.max()),
                        "alpha_local_spread": float(loc.max() / loc.min() - 1) if loc.size >= 2 else None,
                        "full_well_e": float((white - offset) / a) if white else None})
        else:
            out["fit_error"] = f"only {int(sel.sum())} usable levels (signal >= 20 sigma_dark, unsaturated)"
        # linearity: S vs T on unsaturated levels above the dark floor
        lin = (~sat) & (S >= 20 * sd0)
        if lin.sum() >= 4:
            k_, b_ = np.polyfit(T[lin], S[lin], 1, w=1 / S[lin])
            rel = (S[lin] - (k_ * T[lin] + b_)) / S[lin]
            out["linearity"] = {"dn_per_us": float(k_), "intercept_dn": float(b_),
                                "max_rel_residual": float(np.abs(rel).max()),
                                "loglog_slope": float(np.polyfit(np.log(T[lin]), np.log(S[lin]), 1)[0]),
                                "levels": int(lin.sum())}
        # dark vs exposure
        off = np.array([x["offset"] for x in r])
        if nlev >= 3 and "alpha_dn_per_e" in out:
            ts = T * 1e-6
            out["dark_vs_exposure"] = {"offset_slope_dn_per_s": float(np.polyfit(ts, off, 1)[0]),
                                       "var_slope_dn2_per_s": float(np.polyfit(ts, vd, 1)[0]),
                                       "dark_e_per_s_from_var": float(np.polyfit(ts, vd, 1)[0] / out["alpha_dn_per_e"] ** 2)}
        result["cfa"][c] = out
    a = {c: result["cfa"][c].get("alpha_dn_per_e") for c in CFA}
    if a["G1"] and a["G2"]:
        result["g1_g2_alpha_diff"] = abs(a["G1"] - a["G2"]) / ((a["G1"] + a["G2"]) / 2)
    return result


def plot_output(res, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    series = [("R", ("R",), "#2a78d6"), ("G", ("G1", "G2"), "#eb6834"), ("B", ("B",), "#1baf7a")]
    ink, muted, grid = "#0b0b0b", "#52514e", "#e4e3df"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": muted, "axes.labelcolor": ink, "xtick.color": muted,
                         "ytick.color": muted, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4), facecolor="#fcfcfb")
    T = np.array(res["exposure_us"])

    def avg(keys, field):
        cols = [np.array([lv[field] if lv[field] is not None else np.nan for lv in res["cfa"][k]["levels"]]) for k in keys]
        return np.mean(cols, 0)

    # 1) PTC components for G
    S, vb, vd = avg(("G1", "G2"), "S"), avg(("G1", "G2"), "vb"), avg(("G1", "G2"), "vd")
    vs = vb - vd
    pos = (S > 0) & (vs > 0)
    floor = S >= 20 * np.sqrt(vd[0])   # same pool as the fits
    for y, lab, col in ((np.sqrt(vb), "total", "#2a78d6"), (np.sqrt(np.clip(vs, 1e-9, None)), "shot", "#eb6834"),
                        (np.sqrt(vd), "dark", "#1baf7a")):
        ax[0].plot(S[pos], y[pos], "-o", color=col, lw=2, ms=5, label=lab)
    g_alpha = [res["cfa"][k].get("alpha_dn_per_e") for k in ("G1", "G2")]
    if all(g_alpha):
        a = np.mean(g_alpha)
        xs = np.geomspace(S[pos].min(), S[pos].max(), 50)
        ax[0].plot(xs, np.sqrt(a * xs), "--", color=muted, lw=1, label=f"√(αS), α={a:.4g}")
    ax[0].set(xscale="log", yscale="log", xlabel="signal S (DN)", ylabel="temporal σ (DN)", title="PTC · G")
    ax[0].legend(frameon=False)

    # 2) alpha_local vs S
    for lab, keys, col in series:
        S_c, a_c = avg(keys, "S"), avg(keys, "alpha_local")
        ok = floor & np.isfinite(a_c)
        ax[1].plot(S_c[ok], a_c[ok], "-o", color=col, lw=2, ms=5, label=lab)
    ax[1].set(xscale="log", xlabel="signal S (DN)", ylabel="(v_b − v_d) / S  (DN/e⁻)",
              title="local conversion gain · S ≥ 20σ_dark")
    ax[1].legend(frameon=False)

    # 3) linearity residual
    for lab, keys, col in series:
        S_c = avg(keys, "S")
        lin = [res["cfa"][k].get("linearity") for k in keys]
        if not all(lin):
            continue
        k_ = np.mean([l["dn_per_us"] for l in lin])
        b_ = np.mean([l["intercept_dn"] for l in lin])
        ok = floor & (S_c > 0)
        ax[2].plot(T[ok], 100 * (S_c[ok] - (k_ * T[ok] + b_)) / S_c[ok], "-o", color=col, lw=2, ms=5, label=lab)
    ax[2].axhline(0, color=muted, lw=1)
    ax[2].set(xscale="log", xlabel="exposure (µs)", ylabel="S vs linear fit (%)", title="linearity · S ≥ 20σ_dark")
    ax[2].legend(frameon=False)
    for a_ in ax:
        a_.grid(True, color=grid, lw=0.6)
        a_.set_facecolor("#fcfcfb")
    fig.suptitle(res["output"], color=ink)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--outputs", nargs="*")
    ap.add_argument("--white", type=int, help="override white level (all outputs)")
    ap.add_argument("--detrend", type=int, default=2,
                    help="polynomial order removed from each pair difference (0 = EMVA mean only)")
    a = ap.parse_args()
    manifest = json.loads((a.session / "manifest.json").read_text())
    out_dir = a.session / "analysis"
    out_dir.mkdir(exist_ok=True)
    summary = {"serial": manifest.get("serial"), "black_level_raw": manifest.get("black_level_raw"),
               "roi": manifest.get("roi"), "outputs": {}}
    for output, rec in manifest["outputs"].items():
        if a.outputs and output not in a.outputs:
            continue
        res = analyze_output(a.session, output, rec, a.white, a.detrend)
        (out_dir / f"{output}.json").write_text(json.dumps(res, indent=1, default=float))
        if "error" in res:
            print(f"[{output}] {res['error']}")
            continue
        plot_output(res, out_dir / f"{output}.png")
        summary["outputs"][output] = {c: {k: res["cfa"][c].get(k) for k in (
            "alpha_dn_per_e", "alpha_se_wls", "alpha_se_empirical", "read_noise_e", "read_noise_dn", "offset_dn_tmin",
            "full_well_e", "shot_slope", "max_model_residual", "alpha_local_spread", "n_fit_levels")} for c in CFA}
        q = res["qa"]
        print(f"\n[{output}] white={res['white_level']}  pairs rejected {q['pairs_rejected_frac']:.1%}  "
              f"hot {q['hot_pixels']}  drift {q.get('drift_rel', float('nan')):+.3%}  "
              f"saturated levels {sum(l['saturated'] for l in q['levels'])}/{len(q['levels'])}  "
              f"flicker levels {sum(l['flicker'] for l in q['levels'])} "
              f"(max pair Δ {max(l['pair_mean_rel_diff_median'] for l in q['levels']):.2%})  "
              f"id gaps {sum(l['frame_id_gaps'] for l in q['levels'])}")
        print(f"  {'CFA':3s} {'alpha DN/e':>11s} {'SE wls':>8s} {'SE emp':>8s} {'read e':>7s} {'read DN':>8s} "
              f"{'shot sl':>7s} {'model':>6s} {'α spread':>8s} {'lin sl':>7s} {'lin res':>7s} {'n':>3s}")
        for c in CFA:
            r = res["cfa"][c]
            lin = r.get("linearity", {})
            f = lambda v, fmt: format(v, fmt) if isinstance(v, (int, float)) and v is not None else "-"
            print(f"  {c:3s} {f(r.get('alpha_dn_per_e'), '11.4f')} {f(r.get('alpha_se_wls'), '8.4f')} "
                  f"{f(r.get('alpha_se_empirical'), '8.4f')} {f(r.get('read_noise_e'), '7.3f')} {f(r.get('read_noise_dn'), '8.3f')} "
                  f"{f(r.get('shot_slope'), '7.4f')} {f(r.get('max_model_residual'), '6.1%')} {f(r.get('alpha_local_spread'), '8.1%')} "
                  f"{f(lin.get('loglog_slope'), '7.4f')} {f(lin.get('max_rel_residual'), '7.2%')} {r.get('n_fit_levels', 0):3d}"
                  + (f"  {r['fit_error']}" if "fit_error" in r else ""))
        if "g1_g2_alpha_diff" in res:
            print(f"  G1/G2 alpha diff {res['g1_g2_alpha_diff']:.2%}")
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=1, default=float))
    print("\nwritten:", out_dir.resolve())


if __name__ == "__main__":
    main()
