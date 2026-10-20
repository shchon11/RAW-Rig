"""Which 2x2 quantity decides the LUCID SP1 -> SP2 blend? Colour-patch sweep analysis.

  python color_transition_analysis.py ../data/color_transition_<date> [--reduce-only]

1. reduce: per level, per quad (R, Gr, Gb, B) temporal mean (calibrated dark removed, light ripple
   removed) and variance -> <level>/quads.npz (cached).
2. label: patches by R/G and B/G of a mid exposure (all on SP1, unsaturated).
3. for each candidate decision variable D (G mean, G max, max of 4, sum of 4 / 4, R, B):
   per patch and colour, bin quads on D, solve the blend weight w from the mean variance
   (Var = w^2 V1(S) + (1-w)^2 V2(S), V1/V2 per colour from blend_weight.json), and read the D where
   w = 0.5. The variable the sensor uses puts that point at the same D for every patch and colour.
4. SP1 clipping check: in patches where R or B is much brighter than G, does the HDR value of that
   pixel stop rising (SP1 clipped while the quad stays on SP1) or keep rising (quad switched)?
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.family": "NanumSquare", "axes.unicode_minus": False, "font.size": 10})
SHOW = {"white": "#888888", "yellow": "#d4a500", "pink": "#ff69b4", "purple": "#7b2d9b", "red": "#d62728",
        "green": "#2ca02c", "blue": "#1f5fbf"}
ROOT = Path(__file__).resolve().parent.parent
MAPS = np.load(ROOT / "calibration/TRI054S-C_254600231/maps_g00.npz")
BLEND = json.loads((ROOT / "data/hdr_transition_20261020/blend_weight.json").read_text())
BLOCK = 116
SP1_CLIP = 21.71 * (4351 - 256)      # SP1 LCG (ch1) full scale in HDR DN
PATCHES = {   # nominal raw R/G, B/G (no white balance) from the 3 ms preview
    "white": (0.42, 1.00), "yellow": (0.47, 0.25), "pink": (0.90, 0.90), "purple": (0.61, 2.9),
    "red": (2.2, 0.56), "green": (0.18, 0.50), "blue": (0.18, 2.45)}


def reduce_level(folder, T):
    out = folder / "quads.npz"
    if out.exists():
        return dict(np.load(out))
    st = np.load(folder / "bright_hdr.npy", mmap_mode="r")
    n, H, W = st.shape
    dark = (MAPS["offset_1ms_dn"] + MAPS["dark_rate_dn_per_s"] * (T - 1000) / 1e6).astype(np.float64)
    sub = st[:, ::3, ::3].astype(np.float64) - dark[::3, ::3]
    m = sub.mean(0)
    sel = (m > 2e3) & (m < 1.2e7)
    g = sub[:, sel].mean(1) if sel.sum() > 1000 else np.ones(n)
    g /= g.mean()
    mu, var = np.empty((H, W), np.float32), np.empty((H, W), np.float32)
    sat = np.zeros((H, W), bool)
    for r in range(0, H, BLOCK):
        raw = st[:, r:r + BLOCK]
        x = (raw.astype(np.float64) - dark[r:r + BLOCK]) / g[:, None, None]
        mu[r:r + BLOCK], var[r:r + BLOCK] = x.mean(0), x.var(0, ddof=1)
        sat[r:r + BLOCK] = (raw >= 2 ** 24 - 1).any(0)
    bad = MAPS["hot"] | MAPS["noisy"] | sat
    q = {}
    for name, (dy, dx) in {"R": (0, 0), "Gr": (0, 1), "Gb": (1, 0), "B": (1, 1)}.items():
        q[f"m_{name}"] = mu[dy::2, dx::2]
        q[f"v_{name}"] = var[dy::2, dx::2]
        q[f"bad_{name}"] = bad[dy::2, dx::2]
    np.savez_compressed(out, **q)
    return q


def labels(q):
    G = (q["m_Gr"] + q["m_Gb"]) / 2
    r, b = q["m_R"] / G, q["m_B"] / G
    lab = np.full(G.shape, -1, int)
    names = list(PATCHES)
    d = np.stack([np.hypot(np.log(r / pr), np.log(b / pb)) for pr, pb in PATCHES.values()])
    best = d.argmin(0)
    ok = (d.min(0) < 0.18) & (G > 500)
    lab[ok] = best[ok]
    # keep quads whose 5x5 neighbourhood has the same label (drops patch borders)
    from scipy.ndimage import minimum_filter, maximum_filter
    same = (minimum_filter(lab, 5) == maximum_filter(lab, 5))
    lab[~same] = -1
    return lab, names


def w_from_var(V, V1, V2):
    disc = np.maximum(V2 ** 2 - (V1 + V2) * (V2 - V), 0)
    return np.where(V <= V1, 1.0, (V2 - np.sqrt(disc)) / (V1 + V2))


def main():
    sess = Path(sys.argv[1])
    man = json.loads((sess / "manifest.json").read_text())
    levels = sorted([(lv["exposure_us"], k) for k, lv in man["levels"].items()
                     if "hdr" in lv.get("bright", {}) and (sess / k / "bright_hdr.npy").exists()])
    qs = []
    for T, k in levels:
        qs.append((T, k, reduce_level(sess / k, T)))
        print(f"reduced {k}", flush=True)
    if "--reduce-only" in sys.argv:
        return
    ref = min(qs, key=lambda t: abs(t[0] - 2000))[2]
    lab, names = labels(ref)
    print("patch quads:", {n: int((lab == i).sum()) for i, n in enumerate(names)})
    fits = {"R": BLEND["R"], "Gr": BLEND["G"], "Gb": BLEND["G"], "B": BLEND["B"]}
    cands = {
        "G mean": lambda q: (q["m_Gr"] + q["m_Gb"]) / 2,
        "G max": lambda q: np.maximum(q["m_Gr"], q["m_Gb"]),
        "max of 4": lambda q: np.max([q["m_R"], q["m_Gr"], q["m_Gb"], q["m_B"]], axis=0),
        "mean of 4": lambda q: (q["m_R"] + q["m_Gr"] + q["m_Gb"] + q["m_B"]) / 4,
        "R": lambda q: q["m_R"], "B": lambda q: q["m_B"],
        "R+G+G+0.75B": lambda q: (q["m_R"] + q["m_Gr"] + q["m_Gb"] + 0.75 * q["m_B"]) / 3.75,
        "R+G+G+0.85B": lambda q: (q["m_R"] + q["m_Gr"] + q["m_Gb"] + 0.85 * q["m_B"]) / 3.85,
        "mean of 4, each capped at SP1 clip": lambda q: np.mean([np.minimum(q[k], SP1_CLIP) for k in ("m_R", "m_Gr", "m_Gb", "m_B")], axis=0)}
    result = {}
    fig, axes = plt.subplots(3, 3, figsize=(20, 15), constrained_layout=True)
    for ax, (cn, f) in zip(axes.ravel(), cands.items()):
        rows = {}
        for i, pn in enumerate(names):
            for col in ("R", "Gr", "Gb", "B"):
                D, V, S = [], [], []
                for T, k, q in qs:
                    sel = (lab == i) & ~q[f"bad_{col}"] & ~q["bad_R"] & ~q["bad_Gr"] & ~q["bad_Gb"] & ~q["bad_B"]
                    D.append(f(q)[sel]); V.append(q[f"v_{col}"][sel]); S.append(q[f"m_{col}"][sel])
                D, V, S = np.concatenate(D), np.concatenate(V), np.concatenate(S)
                edges = np.geomspace(2e4, 3e5, 120)
                idx = np.digitize(D, edges)
                ws, ds = [], []
                for b in range(1, len(edges)):
                    s = idx == b
                    if s.sum() < 300:
                        continue
                    v = V[s]; v = v[v < 6 * np.median(v)]
                    Sm = S[s].mean()
                    V1 = np.polyval(fits[col]["V1_fit"], Sm); V2 = np.polyval(fits[col]["V2_fit"], Sm)
                    ws.append(float(w_from_var(v.mean(), V1, V2))); ds.append(float(np.sqrt(edges[b - 1] * edges[b])))
                ws, ds = np.array(ws), np.array(ds)
                d50 = np.nan
                if len(ws) > 3 and ws.max() > 0.6 and ws.min() < 0.4:
                    j = np.where((ws[:-1] >= 0.5) & (ws[1:] < 0.5))[0]
                    if len(j):
                        j = j[0]
                        d50 = float(np.exp(np.interp(0.5, [ws[j + 1], ws[j]], [np.log(ds[j + 1]), np.log(ds[j])])))
                rows[f"{pn}/{col}"] = d50
                if col in ("Gr", "R") and len(ws):
                    ax.plot(ds, ws, "-" if col == "Gr" else ":", color=SHOW[pn], lw=1.8 if col == "Gr" else 1.2,
                            label=pn if col == "Gr" else None)
        vals = np.array([v for v in rows.values() if np.isfinite(v)])
        spread = float(np.std(np.log(vals))) if len(vals) > 2 else np.nan
        result[cn] = {"d50": rows, "log_spread": spread, "n": int(len(vals))}
        ax.set_xscale("log"); ax.set_ylim(-0.05, 1.05); ax.grid(True, which="both", alpha=0.3)
        ax.axhline(0.5, color="k", lw=0.6, ls="--")
        ax.set_title(f"기준 후보: {cn}   (w=0.5 지점의 흩어짐 {spread:.3f})")
        ax.set_xlabel(f"{cn} (DN, 0 dB)"); ax.set_ylabel("섞는 비율 w (1 = SP1만, 0 = SP2만)")
        print(f"\n== {cn}: D at w=0.5 per patch/colour  (spread of log D: {spread:.3f}, n={len(vals)})")
        for pn in names:
            print(f"   {pn:7s} " + "  ".join(f"{c}:{rows[pn + '/' + c] / 1e3:6.1f}k" if np.isfinite(rows[pn + '/' + c]) else f"{c}:   --  "
                                            for c in ("R", "Gr", "Gb", "B")))
    # SP1 clipping check: HDR mean / exposure for the brightest colour in red and purple/blue patches
    print("\n== response per ms (DN/ms) of the bright colour vs exposure: a drop = clipped on SP1")
    for pn, col in (("red", "R"), ("purple", "B"), ("blue", "B"), ("purple", "R")):
        i = names.index(pn)
        line = []
        for T, k, q in qs:
            sel = (lab == i) & ~q[f"bad_{col}"]
            if sel.sum() < 100:
                continue
            line.append(f"{k}:{np.median(q[f'm_{col}'][sel]) / (T / 1000):.0f}")
        print(f"   {pn}/{col}: " + " ".join(line))
    axes[0, 0].legend(title="색종이 (실선 G, 점선 R)", fontsize=8)
    fig.suptitle("LUCID SP1→SP2 섞는 기준 찾기: 맞는 기준이면 모든 색종이의 곡선이 한 줄로 겹친다 "
                 f"({len(qs)}단계, {qs[0][1]}~{qs[-1][1]})", fontsize=13)
    fig.savefig(ROOT / "plots/color_decision_variable.png", dpi=100)
    plt.close(fig)
    # SP1 clipping check plot
    fig, ax = plt.subplots(figsize=(10, 5.5), constrained_layout=True)
    for pn, col in (("red", "R"), ("purple", "B"), ("blue", "B"), ("pink", "R"), ("white", "Gr")):
        i = names.index(pn)
        Ts, rs = [], []
        for T, k, q in qs:
            sel = (lab == i) & ~q[f"bad_{col}"]
            if sel.sum() < 100 or T < 500:
                continue
            Ts.append(T / 1000); rs.append(np.median(q[f"m_{col}"][sel]))
        Ts, rs = np.array(Ts), np.array(rs)
        ax.plot(rs, rs / Ts / np.median((rs / Ts)[:4]), "o-", color=SHOW[pn], ms=4, label=f"{pn} 종이의 {col}")
    ax.axvline(88.9e3, color="k", ls=":", lw=1); ax.text(90e3, 0.62, "SP1 LCG 포화\n(ch1 4095 코드)", fontsize=8)
    ax.set_xscale("log"); ax.set_ylim(0.6, 1.1); ax.grid(True, which="both", alpha=0.3)
    ax.set_xlabel("그 픽셀의 HDR 값 (DN, 0 dB)"); ax.set_ylabel("노출당 응답 (짧은 노출 기준 = 1)")
    ax.set_title("밝은 색 픽셀이 SP1 포화에 먼저 닿으면 응답이 꺾인다 (꺾이지 않으면 그 전에 SP2로 넘어간 것)")
    ax.legend(fontsize=9)
    fig.savefig(ROOT / "plots/color_sp1_clip_check.png", dpi=100)
    (sess / "decision_variable.json").write_text(json.dumps(result, indent=1))
    print("->", sess / "decision_variable.json")


if __name__ == "__main__":
    main()
