"""Figures for the diffuser-flat sessions of 2026-10-20.

  python flat_figures.py

plots/prnu_lucid.png : LUCID PRNU per signal region (from prnu.json) + distribution of the pixel/
                       neighbourhood ratio at 300 ms (ch2) and 800 ms (ch3, SP1 heavily overexposed)
plots/bfs_saturation.png : BFS near full scale: temporal variance vs signal (e-) with the PTC model,
                       per-pixel response per exposure relative to the 42 ms level, clipped fraction
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter
from scipy.ndimage import gaussian_filter

ROOT = Path(__file__).resolve().parent.parent
plt.rcParams.update({"font.family": "NanumSquare", "axes.unicode_minus": False, "font.size": 10})
FMT = FuncFormatter(lambda v, _: f"{v:g}" if 1e-3 <= v < 1e5 else f"1e{int(round(np.log10(v)))}")
HUE = {"R": "#d62728", "G1": "#2ca02c", "G2": "#66a182", "B": "#1f5fbf"}


def lucid_prnu():
    folder = ROOT / "data/prnu_20261020/lucid"
    pj = json.loads((folder / "prnu.json").read_text())
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(16, 6), constrained_layout=True)
    for c in ("R", "G1", "G2", "B"):
        S = [lv[c]["signal_dn"] for lv in pj["levels"]]
        P = [lv[c]["prnu_pct"] for lv in pj["levels"]]
        a1.plot(S, P, "o-", color=HUE[c], label=c)
    for x0, x1, lab in ((0, 3.9e3, "ch0"), (3.9e3, 5.0e4, "ch1 (SP1)"), (6.6e4, 5.5e5, "ch2 (SP2)"), (6.0e5, 3e6, "ch3")):
        a1.axvspan(max(x0, 800), x1, alpha=0.06, color="k")
        a1.text(np.sqrt(max(x0, 800) * x1), 12.5, lab, ha="center", fontsize=9)
    a1.set_xscale("log"); a1.set_yscale("log"); a1.xaxis.set_major_formatter(FMT); a1.yaxis.set_major_formatter(FMT)
    a1.set_ylim(0.3, 20); a1.grid(True, which="both", alpha=0.3)
    a1.set_xlabel("HDR signal (DN, 0 dB)"); a1.set_ylabel("PRNU (픽셀 대 이웃 평균의 표준편차, %)")
    a1.set_title("LUCID PRNU: 구간별 (복사지 확산판, 조리개 개방)"); a1.legend()
    for f, lab, col in (("g00_t299991us.npz", "300 ms (ch2 구간)", "#1f77b4"), ("g00_t800008us.npz", "800 ms (ch3 구간, SP1 12배 과노출)", "#d62728")):
        st = np.load(folder / f)["frames"]
        G = st[:, 0::2, 1::2].astype(np.float64).mean(0) - 257.3
        r = (G / gaussian_filter(G, 8))[50:-50, 50:-50].ravel()
        a2.hist(r, bins=np.linspace(0.8, 2.2, 281), histtype="step", lw=1.6, color=col,
                label=f"{lab}: 중앙 {np.median(r):.3f}, 99% {np.percentile(r, 99):.2f}, 99.9% {np.percentile(r, 99.9):.2f}")
    a2.set_yscale("log"); a2.grid(True, alpha=0.3)
    a2.set_xlabel("픽셀 값 / 주변 평균 (G)"); a2.set_ylabel("픽셀 수")
    a2.set_title("ch3 구간에서만 오른쪽 꼬리: 일부 픽셀이 1.4~2배 밝다 (SP1 과노출 시 전하가 새는 것으로 추정)")
    a2.legend(fontsize=9)
    fig.savefig(ROOT / "plots/prnu_lucid.png", dpi=100)
    plt.close(fig)


def bfs_saturation():
    folder = ROOT / "data/bfs_saturation_20261020/bfs"
    cal = json.loads((ROOT / "calibration/BFS-PGE-23S3C_26315393/noise_model.json").read_text())["gains"]["0"]
    maps = np.load(ROOT / "calibration/BFS-PGE-23S3C_26315393/maps_g00.npz")
    a = cal["alpha_dn_per_e"]["G1"]
    rd = cal["read_noise_e"]["G1"]
    files = sorted(folder.glob("g00_t*us.npz"), key=lambda p: float(p.stem.split("_t")[1][:-2]))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(16, 6), constrained_layout=True)
    ref = None
    white_e = (65516 - np.median(maps["offset_1ms_dn"])) / a
    cmap = plt.get_cmap("viridis")
    for k, f in enumerate(files):
        info = json.loads(f.with_suffix(".json").read_text())
        T = info["exposure_us"]
        st = np.load(f)["frames"]
        dark = maps["offset_1ms_dn"] + maps["dark_rate_dn_per_s"] * (T - 1000) / 1e6
        x = st[:, 0::2, 1::2].astype(np.float64) - dark[0::2, 1::2]
        msel = x[0::2].mean(0) / a
        mu, var = x[1::2].mean(0) / a, x[1::2].var(0, ddof=1) / a ** 2
        clipped = (st[:, 0::2, 1::2] >= 65516).any(0)
        edges = np.linspace(white_e * 0.4, white_e * 1.02, 60)
        idx = np.digitize(msel, edges)
        S, V = [], []
        for b in range(1, len(edges)):
            s = idx == b
            if s.sum() > 300:
                S.append(mu[s].mean()); V.append(var[s].mean())
        col = cmap(k / max(len(files) - 1, 1))
        a1.plot(S, V, "o", ms=3, color=col, label=f"{T / 1000:.0f} ms (포화 픽셀 {clipped.mean():.0%})")
        full = x.mean(0) / a
        if ref is None:
            ref, Tref = full, T
        else:
            ratio = (full / T) / (ref / Tref)
            ok = ~clipped
            e2 = np.linspace(white_e * 0.5, white_e * 1.0, 40)
            id2 = np.digitize(full[ok], e2)
            Rs, Ss = [], []
            for b in range(1, len(e2)):
                s = id2 == b
                if s.sum() > 300:
                    Rs.append(np.median(ratio[ok][s])); Ss.append(full[ok][s].mean())
            a2.plot(Ss, Rs, "o-", ms=3, color=col, label=f"{T / 1000:.0f} ms / {Tref / 1000:.0f} ms")
    Sm = np.linspace(white_e * 0.4, white_e * 1.02, 200)
    a1.plot(Sm, Sm + rd ** 2, "k--", lw=1, label=f"PTC 모델 (분산 = S + {rd:.2f}^2)")
    a1.axvline(white_e, color="r", ls=":", lw=1); a1.text(white_e * 0.995, a1.get_ylim()[0] if False else 200, "white level\n65516 DN", ha="right", fontsize=8, color="r")
    a1.set_xlabel("signal (e-, G)"); a1.set_ylabel("temporal variance (e-^2)")
    a1.set_title("BFS 0 dB 포화 근처: 시간 분산"); a1.grid(True, alpha=0.3); a1.legend(fontsize=8)
    a2.axhline(1, color="k", lw=0.8)
    a2.axvline(white_e, color="r", ls=":", lw=1)
    a2.set_xlabel("signal (e-, G)"); a2.set_ylabel("노출당 응답 / 42 ms 기준 (같은 픽셀)")
    a2.set_title("BFS 0 dB 포화 근처: 선형성 (1보다 작으면 응답이 휨)"); a2.grid(True, alpha=0.3); a2.legend(fontsize=8)
    fig.savefig(ROOT / "plots/bfs_saturation.png", dpi=100)
    plt.close(fig)


if __name__ == "__main__":
    lucid_prnu()
    bfs_saturation()
    print("->", ROOT / "plots/prnu_lucid.png", ROOT / "plots/bfs_saturation.png")
