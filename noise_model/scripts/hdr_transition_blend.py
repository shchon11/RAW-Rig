"""SP1->SP2 blend weight of the LUCID HDR combination, from the temporal variance.

  python hdr_transition_blend.py ../data/hdr_transition_<date> [levels ...]

In the first transition the combined value is a weighted mean of the two photodiodes,
  v = w * X1 + (1 - w) * X2,   X1 = SP1 LCG (scaled), X2 = SP2 HCG (scaled), independent noise,
so Var[v] = w^2 V1(S) + (1-w)^2 V2(S). V1, V2 are fitted (a*S + c) per CFA colour on the pure
SP1 / pure SP2 sides, and w is solved bin by bin of the quad green mean (the transition is decided
per 2x2 quad: R moves with its quad's green). Per-pixel stats use the per-frame light factor;
means use the session darks when present, else the calibrated dark map.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CAL = ROOT / "calibration/TRI054S-C_254600231"
BLOCK = 116
POS = {"R": (0, 0), "Gr": (0, 1), "Gb": (1, 0), "B": (1, 1)}


def pixel_stats(sess, lev, maps, T):
    st = np.load(sess / lev / "bright_hdr.npy", mmap_mode="r")
    n, H, W = st.shape
    r0, r1, c0, c1 = H // 5 // 2 * 2, 4 * H // 5 // 2 * 2, W // 5 // 2 * 2, 4 * W // 5 // 2 * 2
    dpath = sess / lev / "dark_hdr.npy"
    if dpath.exists():
        dark = np.load(dpath, mmap_mode="r")[:, r0:r1, c0:c1].astype(np.float64).mean(0)
    else:
        dark = (maps["offset_1ms_dn"] + maps["dark_rate_dn_per_s"] * (T - 1000) / 1e6)[r0:r1, c0:c1].astype(np.float64)
    sub = st[:, r0:r1:3, c0:c1:3].astype(np.float64) - dark[::3, ::3]
    m = sub.mean(0)
    sel = (m > 2e3) & (m < 1.2e7)
    g = sub[:, sel].mean(1)
    g /= g.mean()
    mu, var = np.empty((r1 - r0, c1 - c0)), np.empty((r1 - r0, c1 - c0))
    for r in range(0, r1 - r0, BLOCK):
        x = (st[:, r0 + r:r0 + min(r + BLOCK, r1 - r0), c0:c1].astype(np.float64) - dark[r:r + BLOCK]) / g[:, None, None]
        mu[r:r + BLOCK], var[r:r + BLOCK] = x.mean(0), x.var(0, ddof=1)
    bad = (maps["hot"] | maps["noisy"])[r0:r1, c0:c1]
    return mu, var, bad, dpath.exists()


def main():
    sess = Path(sys.argv[1])
    levels = sys.argv[2:] or ["1.5ms", "2ms", "2.5ms", "3ms", "4ms", "5ms", "6ms", "12ms"]
    man = json.loads((sess / "manifest.json").read_text())
    maps = np.load(CAL / "maps_g00.npz")
    rows = {c: [] for c in ("R", "G", "B")}
    measured = []
    for lev in levels:
        mu, var, bad, md = pixel_stats(sess, lev, maps, man["levels"][lev]["exposure_us"])
        measured.append(md)
        q = lambda a, nm: a[POS[nm][0]::2, POS[nm][1]::2]
        gq = (q(mu, "Gr") + q(mu, "Gb")) / 2                       # quad green mean
        okq = ~(q(bad, "R") | q(bad, "Gr") | q(bad, "Gb") | q(bad, "B"))
        for c, names in (("R", ["R"]), ("G", ["Gr", "Gb"]), ("B", ["B"])):
            for nm in names:
                rows[c].append(np.stack([gq[okq], q(mu, nm)[okq], q(var, nm)[okq]], 1))
        print(f"{lev}: quad G {np.percentile(gq[okq], 1):.0f}-{np.percentile(gq[okq], 99):.0f}", flush=True)
    out = {"levels": levels, "dark": "session" if all(measured) else "calibrated map (provisional)"}
    edges = np.arange(2e4, 1.5e5, 500.0)
    for c, lst in rows.items():
        a = np.concatenate(lst)
        g, s, v = a[:, 0], a[:, 1], a[:, 2]
        # side fits on the quad-green axis: pure SP1 below 45k, pure SP2 above 72k (checked with the lattices)
        lo, hi = g < 45e3, (g > 72e3) & (g < 1.5e5)
        ok = lambda sel: sel & (v < 6 * np.median(v[sel]))
        A1 = np.polyfit(s[ok(lo)], v[ok(lo)], 1)
        A2 = np.polyfit(s[ok(hi)], v[ok(hi)], 1)
        tab = []
        for e in edges:
            sel = (g >= e) & (g < e + 500)
            if sel.sum() < 100:
                continue
            vv = v[sel]
            vv = vv[vv < 6 * np.median(vv)]
            S = s[sel].mean()
            V, se = vv.mean(), vv.std() / np.sqrt(len(vv))
            V1, V2 = np.polyval(A1, S), np.polyval(A2, S)
            # Var = w^2 V1 + (1-w)^2 V2 -> branch going from w=1 (SP1) through the minimum to w=0 (SP2)
            disc = V2 ** 2 - (V1 + V2) * (V2 - V)
            w = (V2 + np.sqrt(max(disc, 0))) / (V1 + V2) if V <= V1 * 1.02 and e < 5e4 else \
                (V2 - np.sqrt(max(disc, 0))) / (V1 + V2)
            tab.append((e + 250, S, V, se, V1, V2, w, int(sel.sum())))
        tab = np.array(tab)
        out[c] = {"V1_fit": A1.tolist(), "V2_fit": A2.tolist(), "table_cols": "quadG S var se V1 V2 w n",
                  "table": tab.tolist()}
        print(f"\n{c}: V1 = {A1[0]:.2f} S + {A1[1]:.0f}   V2 = {A2[0]:.2f} S + {A2[1]:.0f}")
        for r in tab[::2]:
            if 4.4e4 < r[0] < 7.6e4:
                print(f"  quad G {r[0]:7.0f}  S {r[1]:7.0f}  var {r[2]:10.0f} +- {r[3]:7.0f}  V1 {r[4]:9.0f} V2 {r[5]:10.0f}  w {r[6]:.3f}  n {int(r[7])}")
    (sess / "blend_weight.json").write_text(json.dumps(out, indent=1))
    print("->", sess / "blend_weight.json")


if __name__ == "__main__":
    main()
