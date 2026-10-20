"""Where the LUCID HDR output hands over from ch0 (SP1 HCG) to ch1 (SP1 LCG), and how.

  python ch0_ch1_handover.py ../data/color_transition_<date> 0.2ms 0.25ms 0.3ms 0.4ms 0.5ms 1ms

ch1 values sit on a lattice of step K1 (~21.7 DN per ch1 code, per CFA position); ch0 values
(step ~1 DN) do not. For each pixel, the concentration |mean exp(2 pi i v / K1)| over its frames is
~0.99 when every frame came from ch1, ~1/sqrt(n) when none did, and ~ the ch1 fraction in between
(the ch1 frames share one phase). Binned against the pixel mean, this gives the handover curve; a
blend (weighted mean of ch0 and ch1, as in the SP1->SP2 transition) would instead put values on
neither lattice, which shows as a dip with no ch1-only plateau. Writes plots/ch0_ch1_handover.png.
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
K1 = {(0, 0): 21.7136, (0, 1): 21.7056, (1, 0): 21.7542, (1, 1): 21.7180}
NAME = {(0, 0): "R", (0, 1): "Gr", (1, 0): "Gb", (1, 1): "B"}
MAPS = np.load(ROOT / "calibration/TRI054S-C_254600231/maps_g00.npz")
plt.rcParams.update({"font.family": "NanumSquare", "axes.unicode_minus": False, "font.size": 10})


def main():
    sess, levels = Path(sys.argv[1]), sys.argv[2:]
    edges = np.geomspace(300, 3e4, 70)
    acc = {p: [[] for _ in range(len(edges) + 1)] for p in K1}
    for lev in levels:
        st = np.load(sess / lev / "bright_hdr.npy", mmap_mode="r")
        n, H, W = st.shape
        T = float(lev[:-2]) * 1000
        dark = MAPS["offset_1ms_dn"] + MAPS["dark_rate_dn_per_s"] * (T - 1000) / 1e6
        bad = MAPS["hot"] | MAPS["noisy"]
        for r in range(0, H, 124):
            x = st[:, r:r + 124].astype(np.float64)
            mu = (x - dark[r:r + 124]).mean(0)
            for (dy, dx), k in K1.items():
                xc = x[:, dy::2, dx::2]
                conc = np.abs(np.exp(2j * np.pi * xc / k).mean(0))
                m = mu[dy::2, dx::2]
                ok = ~bad[r:r + 124][dy::2, dx::2]
                idx = np.digitize(m[ok], edges)
                cc = conc[ok]
                for b in np.unique(idx):
                    acc[(dy, dx)][b].append(cc[idx == b])
        print("done", lev, flush=True)
    fig, ax = plt.subplots(figsize=(10, 5.5), constrained_layout=True)
    hue = {"R": "#d1495b", "Gr": "#2e8b57", "Gb": "#66a182", "B": "#3a6ea5"}
    for p, bins in acc.items():
        xs, ys = [], []
        for b in range(1, len(edges)):
            if bins[b]:
                v = np.concatenate(bins[b])
                if len(v) > 300:
                    xs.append(np.sqrt(edges[b - 1] * edges[b])); ys.append(v.mean())
        ax.plot(xs, ys, "o-", ms=3, color=hue[NAME[p]], label=NAME[p])
        xs, ys = np.array(xs), np.array(ys)
        if len(ys) and ys.max() > 0.6:
            half = 0.5 * (ys.min() + ys.max())
            j = np.where(ys >= half)[0][0]
            print(f"{NAME[p]:2s}: ch1 share reaches half at {xs[j]:.0f} DN (~{xs[j] / 9.83:.0f} SP1 e-); "
                  f"low plateau {ys[:3].mean():.2f}, high plateau {ys[-3:].mean():.2f}")
    ax.axhline(1 / np.sqrt(48), color="gray", ls=":", lw=1)
    ax.text(330, 1 / np.sqrt(48) + 0.02, "ch1 값 없음 (무작위 수준)", fontsize=8, color="gray")
    ax.set_xscale("log")
    ax.set_xlabel("HDR signal (DN, dark subtracted; 1 SP1 e- = 9.8 DN)")
    ax.set_ylabel("ch1 격자 집중도 (= ch1에서 온 프레임 비율)")
    ax.set_title("LUCID ch0 → ch1 전환: 픽셀 평균 신호별로 ch1 값의 비율 (색종이 세션 " + ", ".join(levels) + ")")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    out = ROOT / "plots/ch0_ch1_handover.png"
    fig.savefig(out, dpi=110)
    print("->", out)


if __name__ == "__main__":
    main()
