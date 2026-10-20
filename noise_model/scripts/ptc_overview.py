"""One-page PTC overview from ptc_analyze.py results (G = mean of G1, G2).

  python ptc_overview.py ../data/ptc_<name>      # -> <session>/analysis/ptc_overview.png

Left: per-channel PTC in electrons (each channel's own alpha). Right: combined HDR output in DN.
"""
import argparse
import json
from pathlib import Path

import numpy as np

COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # categorical slots 1-4 (lines: adjacent-validated)
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def g_levels(res, field):
    return np.mean([[lv[field] for lv in res["cfa"][k]["levels"]] for k in ("G1", "G2")], 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", type=Path)
    a = ap.parse_args()
    d = a.session / "analysis"
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(1, 2, figsize=(13, 5), facecolor=SURFACE)

    k = 0
    for ch in ("HDRChannel0", "HDRChannel1", "HDRChannel2", "HDRChannel3"):
        p = d / f"{ch}.json"
        if not p.exists():
            continue
        res = json.loads(p.read_text())
        alpha = np.mean([res["cfa"][c].get("alpha_dn_per_e") or np.nan for c in ("G1", "G2")])
        if not np.isfinite(alpha):
            continue
        S, vb = g_levels(res, "S"), g_levels(res, "vb")
        ok = S > 0
        read_e = np.mean([res["cfa"][c]["read_noise_e"] for c in ("G1", "G2")])
        lab = f"ch{ch[-1]}  α={alpha:.3g} DN/e⁻, read {read_e:.2f} e⁻"
        ax[0].plot(S[ok] / alpha, np.sqrt(vb[ok]) / alpha, "-o", color=COLORS[k], lw=2, ms=4, label=lab)
        ax[0].annotate(f"ch{ch[-1]}", (S[ok][-1] / alpha, np.sqrt(vb[ok][-1]) / alpha), xytext=(5, 0),
                       textcoords="offset points", color=INK, va="center")
        k += 1
    xs = np.geomspace(0.5, 3e5, 50)
    ax[0].plot(xs, np.sqrt(xs), "--", color=MUTED, lw=1, label="shot noise only (√N)")
    ax[0].set(xscale="log", yscale="log", xlabel="signal (e⁻)", ylabel="temporal noise (e⁻)",
              title="Per-channel PTC · electrons (G)")
    ax[0].legend(frameon=False, loc="upper left")

    p = d / "HDR.json"
    if p.exists():
        res = json.loads(p.read_text())
        S, vb, vd = g_levels(res, "S"), g_levels(res, "vb"), g_levels(res, "vd")
        ok = S > 0
        ax[1].plot(S[ok], np.sqrt(vb[ok]), "-o", color=COLORS[0], lw=2, ms=4, label="HDR total σ")
        a0 = np.mean([res["cfa"][c]["levels"][0]["alpha_local"] for c in ("G1", "G2")])
        low = [lv for lv in res["cfa"]["G1"]["levels"] if lv["alpha_local"] and lv["S"] < 2e4]
        a_low = np.median([lv["alpha_local"] for lv in low]) if low else a0
        xs = np.geomspace(S[ok].min(), S[ok].max(), 50)
        ax[1].plot(xs, np.sqrt(a_low * xs + vd[0]), "--", color=MUTED, lw=1,
                   label=f"single-gain model, α={a_low:.3g} DN/e⁻")
        ax[1].set(xscale="log", yscale="log", xlabel="signal S (DN, 24-bit)", ylabel="temporal σ (DN)",
                  title="Combined HDR output · G")
        ax[1].legend(frameon=False, loc="upper left")
    for x in ax:
        x.grid(True, color=GRID, lw=0.6)
        x.set_facecolor(SURFACE)
    fig.suptitle(f"IMX490 PTC · {a.session.name}", color=INK)
    fig.tight_layout()
    out = d / "ptc_overview.png"
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == "__main__":
    main()
