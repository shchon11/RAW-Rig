"""Rough temporal-noise budget of both cameras: which source makes up the variance at each signal.

  python noise_budget.py [--exposure-us 10000]

BFS (per gain, G1): photon shot = S; dark-current shot = rate*T; quantization = 16^2/12 DN^2;
row / column temporal noise from the full-frame darks; the rest of the read noise split into a
part before the analog gain (same e- at every gain) and after it (scales as 1/gain), from a fit of
read^2(g) = a^2 + b^2 / (alpha_g/alpha_0)^2 over the six gains.
LUCID (0 dB HDR output, G): SP1 shot, SP2 shot, read noise of the channel in use (ch0..ch3) times
its HDR scale, lattice quantization K^2/12, row/column, dark current; SP1->SP2 blend with the
measured w(quad G) and the ch2->ch3 per-frame switch (logistic, centre 556k DN).
Writes plots/noise_budget.png and prints tables. Rough: dark offsets of today's sessions pending.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter

ROOT = Path(__file__).resolve().parent.parent
BFS = json.loads((ROOT / "calibration/BFS-PGE-23S3C_26315393/noise_model.json").read_text())
LUC = json.loads((ROOT / "calibration/TRI054S-C_254600231/noise_model.json").read_text())
BLEND = ROOT / "data/hdr_transition_20261020/blend_weight.json"
plt.rcParams.update({"font.family": "NanumSquare", "axes.unicode_minus": False, "font.size": 9})


def bfs_budget(T):
    gains = sorted(BFS["gains"], key=float)
    a0 = BFS["gains"]["0"]["alpha_dn_per_e"]["G1"]
    g_lin = np.array([BFS["gains"][g]["alpha_dn_per_e"]["G1"] / a0 for g in gains])
    r2 = np.array([BFS["gains"][g]["read_noise_e"]["G1"] ** 2 for g in gains])
    # read noise keeps falling up to 24 dB, faster than a^2 + b^2/g^2 allows: no clean two-stage split.
    # Use the 24 dB value as the upper bound of the part that does not shrink with gain.
    pre2 = float(r2.min())
    out = {"fit": {"floor_e": float(np.sqrt(pre2)), "gains": gains, "read_e": np.sqrt(r2).tolist(), "g_lin": g_lin.tolist()}}
    for g in gains:
        d = BFS["gains"][g]
        al = d["alpha_dn_per_e"]["G1"]
        df = d["dark_fullframe"]
        row = (np.median(df["row_temporal_sigma_dn"][:3]) / al) ** 2
        col = (np.median(df["col_temporal_sigma_dn"][:3]) / al) ** 2
        quant = 16 ** 2 / 12 / al ** 2
        total = d["read_noise_e"]["G1"] ** 2
        pre = min(pre2, total)
        white_post = max(total - pre - quant - row - col, 0.0)
        out[g] = {"alpha": al, "read_total": total, "pre": pre, "post_white": white_post, "quant": quant,
                  "row": row, "col": col, "dark": d["dark_current_e_per_s"]["G1"] * T / 1e6}
    return out


def lucid_budget(S, T, blend):
    """Variance components (HDR DN^2) at HDR signal S (DN, 0 dB, G)."""
    g0 = LUC["gains"]["0"]
    a1 = g0["alpha_dark_range_dn_per_e"]          # HDR DN per SP1 electron
    df = g0["dark_fullframe"]
    K1, K2, K3 = 21.7056, 144.9510, 5363.0
    r_ch = {0: 0.53 * 9.627, 1: 1.113, 2: 13.01, 3: 2.61}   # channel read noise, channel DN (PTC 2026-10-19)
    k_ch = {0: a1 / 9.627, 1: K1, 2: K2, 3: K3}
    slope2 = blend["G"]["V2_fit"][0]               # SP2 shot, HDR DN^2 per DN of signal
    comp = {k: np.zeros_like(S) for k in
            ("SP1 shot", "SP2 shot", "ch0 read", "ch1 read", "ch2 read", "ch3 read", "quantization", "row/col", "dark current")}
    rowcol = df["row_temporal_sigma_dn"][1] ** 2 + df["col_temporal_sigma_dn"][1] ** 2
    dark = a1 ** 2 * g0["dark_current_e_per_s"]["G1"] * T / 1e6
    # SP1 side: ch0 below its clip (~425 e-, hard handover assumed at 4.0k DN), ch1 above
    tab = np.array(blend["G"]["table"])
    w = np.interp(S, tab[:, 0], np.clip(tab[:, 6], 0, 1), left=1.0, right=0.0)
    p3 = 1 / (1 + np.exp(-(S - 556e3) / 9e3))
    on0 = S < 4.0e3
    v1 = {"SP1 shot": a1 * S,
          "ch0 read": np.where(on0, (k_ch[0] * r_ch[0]) ** 2 - rowcol, 0),
          "ch1 read": np.where(on0, 0, (K1 * r_ch[1]) ** 2),
          "quantization": np.where(on0, k_ch[0] ** 2 / 12, K1 ** 2 / 12)}
    v2 = {"SP2 shot": slope2 * S,
          "ch2 read": (1 - p3) * (K2 * r_ch[2]) ** 2,
          "ch3 read": p3 * (K3 * r_ch[3]) ** 2,
          "quantization": (1 - p3) * K2 ** 2 / 12 + p3 * K3 ** 2 / 12}
    for k, v in v1.items():
        comp[k] += w ** 2 * v
    for k, v in v2.items():
        comp[k] += (1 - w) ** 2 * v
    comp["row/col"] += rowcol
    comp["dark current"] += dark
    return comp, a1


def fmt_axis(ax):
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}" if 1e-3 <= v < 1e5 else f"1e{int(round(np.log10(v)))}"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exposure-us", type=float, default=10000)
    a = ap.parse_args()
    T = a.exposure_us
    blend = json.loads(BLEND.read_text())
    bb = bfs_budget(T)
    f = bb["fit"]
    print(f"BFS read noise by gain (floor kept at every gain <= {f['floor_e']:.2f} e-):")
    for g, r, gl in zip(f["gains"], f["read_e"], f["g_lin"]):
        print(f"   {g:>2s} dB (x{gl:5.2f})  read {r:.3f} e-")
    cols = ["SP1 shot", "SP2 shot", "ch0 read", "ch1 read", "ch2 read", "ch3 read", "quantization", "row/col", "dark current"]
    hue = {"photon shot": "#4c78a8", "read, stays at high gain": "#f58518", "read, shrinks with gain": "#e45756", "quantization": "#72b7b2",
           "row noise": "#b279a2", "column noise": "#9d755d", "dark current": "#54a24b",
           "SP1 shot": "#4c78a8", "SP2 shot": "#9ecae9", "ch0 read": "#f58518", "ch1 read": "#e45756",
           "ch2 read": "#b279a2", "ch3 read": "#9d755d", "row/col": "#bab0ac"}
    fig, axes = plt.subplots(1, 3, figsize=(20, 6), constrained_layout=True)
    S_e = np.geomspace(0.1, 2e4, 400)
    for ax, g in zip(axes[:2], ["0", "24"]):
        b = bb[g]
        parts = {"photon shot": S_e, "read, stays at high gain": np.full_like(S_e, b["pre"]),
                 "read, shrinks with gain": np.full_like(S_e, b["post_white"]), "quantization": np.full_like(S_e, b["quant"]),
                 "row noise": np.full_like(S_e, b["row"]), "column noise": np.full_like(S_e, b["col"]),
                 "dark current": np.full_like(S_e, b["dark"])}
        tot = sum(parts.values())
        ax.stackplot(S_e, *[v / tot * 100 for v in parts.values()], labels=list(parts), colors=[hue[k] for k in parts])
        ax.set_xscale("log"); ax.set_xlim(S_e[0], S_e[-1]); ax.set_ylim(0, 100); fmt_axis(ax)
        ax.set_xlabel("signal (e-)"); ax.set_ylabel("share of temporal variance (%)")
        ax.set_title(f"BFS {g} dB, exposure {T / 1000:g} ms  (read {np.sqrt(b['read_total']):.2f} e-)")
        print(f"\nBFS {g} dB (T {T / 1000:g} ms), variance e-^2 at signal ... | " + " | ".join(parts))
        for s in (1, 10, 100, 1000, 10000):
            vals = [s if k == "photon shot" else float(v[0]) for k, v in parts.items()]
            t = sum(vals)
            print(f"   S {s:6d} e-: total {t:9.2f}  " + "  ".join(f"{k} {v / t * 100:5.1f}%" for k, v in zip(parts, vals)))
    axes[0].legend(loc="center right", fontsize=8)
    S = np.geomspace(3, 1.5e7, 800)
    comp, a1 = lucid_budget(S, T, blend)
    tot = sum(comp.values())
    ax = axes[2]
    ax.stackplot(S, *[comp[k] / tot * 100 for k in cols], labels=cols, colors=[hue[k] for k in cols])
    ax.set_xscale("log"); ax.set_xlim(S[0], S[-1]); ax.set_ylim(0, 100); fmt_axis(ax)
    ax.set_xlabel("HDR signal (DN, 0 dB; 1 SP1 e- = %.1f DN)" % a1); ax.set_ylabel("share of temporal variance (%)")
    ax.set_title(f"LUCID HDR 0 dB (G), exposure {T / 1000:g} ms")
    ax.legend(loc="center left", fontsize=8)
    for x0, x1, lab in ((50.5e3, 66e3, "SP1/SP2\nblend"), (540e3, 580e3, "ch2->ch3")):
        ax.axvspan(x0, x1, color="k", alpha=0.08)
        ax.text(np.sqrt(x0 * x1), 50, lab, ha="center", va="center", fontsize=8, bbox=dict(fc="white", ec="none", alpha=0.8))
    print(f"\nLUCID HDR 0 dB G (T {T / 1000:g} ms): share of variance at signal (DN / SP1 e-)")
    for s in (10, 100, 1e3, 1e4, 4e4, 5.8e4, 1e5, 3e5, 5.6e5, 1e6, 5e6):
        c, _ = lucid_budget(np.array([s], float), T, blend)
        t = sum(float(v[0]) for v in c.values())
        print(f"   S {s:9.0f} DN ({s / a1:8.0f} e-): sigma {np.sqrt(t):8.1f} DN  " +
              "  ".join(f"{k} {float(c[k][0]) / t * 100:4.1f}%" for k in cols if float(c[k][0]) / t > 0.005))
    fig.suptitle(f"시간 노이즈 성분 비율 (대략, 2026-10-20)  —  노출 {T / 1000:g} ms 기준", fontsize=12)
    out = ROOT / "plots/noise_budget.png"
    fig.savefig(out, dpi=110)
    print("->", out)


if __name__ == "__main__":
    main()
