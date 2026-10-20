"""Second LUCID HDR transition (SP2 HCG ch2 -> SP2 LCG ch3): per-frame source of every value.

  python hdr_transition_ch3.py ../data/hdr_transition_<date> <ref level> <levels ...>

ch2 values sit on a lattice of step K2 (per CFA position) and ch3 values on K3 = 37*K2 exactly,
so every value is on the K2 lattice and its lattice index modulo 37 tells the source: ch3 frames
share one residue per pixel (taken from a reference level where the pixel is all ch3), ch2 frames
fall on any residue (1/37 chance coincidence, corrected for in the rates). Output per level:
<level>/ch3_sources.npz (bool is_ch3 for the paper region) and a table of the ch3 fraction and the
largest ch2 value vs the pixel mean, per CFA colour.
"""
import sys
from pathlib import Path

import numpy as np

POS = {"R": (0, 0), "Gr": (0, 1), "Gb": (1, 0), "B": (1, 1)}
K2 = {"R": 156.5116, "Gr": 144.9510, "Gb": 146.4790, "B": 140.3690}
M = 37


def load(sess, lev):
    st = np.load(sess / lev / "bright_hdr.npy", mmap_mode="r")
    n, H, W = st.shape
    r0, r1, c0, c1 = H // 5 // 2 * 2, 4 * H // 5 // 2 * 2, W // 5 // 2 * 2, 4 * W // 5 // 2 * 2
    return np.asarray(st[:, r0:r1, c0:c1]), (r0, c0)


def main():
    sess, ref, levels = Path(sys.argv[1]), sys.argv[2], sys.argv[3:]
    R, _ = load(sess, ref)
    for lev in levels:
        X, (r0, c0) = load(sess, lev)
        is3 = np.zeros(X.shape, bool)
        print(f"== {lev}")
        for nm, (dy, dx) in POS.items():
            k = K2[nm]
            a = R[:, dy::2, dx::2].astype(np.float64)
            x = X[:, dy::2, dx::2].astype(np.float64)
            pa = np.angle(np.exp(2j * np.pi * a / k).mean(0)) * k / (2 * np.pi)       # K2 phase (DN)
            px = np.angle(np.exp(2j * np.pi * x / k).mean(0)) * k / (2 * np.pi)
            # carry the reference phase to this level: common shift, then per-pixel unwrap
            sh = np.angle(np.exp(2j * np.pi * (px - pa) / k).mean()) * k / (2 * np.pi)
            px = pa + sh + np.angle(np.exp(2j * np.pi * (px - pa - sh) / k)) * k / (2 * np.pi)
            ia = np.rint((a - pa) / k).astype(np.int64) % M
            j0 = np.array([np.bincount(col, minlength=M).argmax() for col in ia.reshape(ia.shape[0], -1).T])
            j0 = j0.reshape(ia.shape[1:])
            ix = np.rint((x - px) / k).astype(np.int64) % M
            # common residue shift of this level (peak over all frames/pixels)
            hs = np.bincount(((ix - j0) % M).ravel(), minlength=M)
            s = hs.argmax()
            on = ((ix - j0) % M) == s
            is3[:, dy::2, dx::2] = on
            mu = x.mean(0)
            f = on.mean(0)
            edges = np.arange(4.0e5, 9.0e5, 2.0e4)
            line = []
            for e in edges:
                sel = (mu >= e) & (mu < e + 2e4)
                if sel.sum() < 30:
                    continue
                p = (f[sel].mean() - 1 / M) / (1 - 1 / M)
                ch2vals = x[:, sel][~on[:, sel]]
                line.append(f"{e / 1e3:.0f}k:{max(p, 0):.2f}" + (f"/{ch2vals.max() / 1e3:.0f}k" if ch2vals.size else ""))
            print(f"  {nm:2s} ref-coherence {np.abs(np.exp(2j*np.pi*a/k).mean(0)).mean():.3f} residue peak "
                  f"{hs[s] / hs.sum():.3f}  ch3 fraction [/max ch2-classified value] by pixel mean: " + " ".join(line))
        np.savez_compressed(sess / lev / "ch3_sources.npz", is_ch3=is3, r0=r0, c0=c0)


if __name__ == "__main__":
    main()
