"""Reduce hdr_transition_capture.py stacks to per-pixel statistics (one npz per exposure).

  python hdr_transition_reduce.py ../data/hdr_transition_<date>

Per exposure folder -> stats.npz (float32 maps, full frame):
  mu_hdr, var_hdr            HDR signal above the dark mean, and its temporal variance (unbiased)
  mu_ch{c}, var_ch{c}, sat_ch{c}   same for HDRChannel0..3 (sat = any frame at the 12-bit clip)
  gain_hdr, gain_ch{c}       per-frame light factor removed (static scene: frame mean ratio)
  drift                      mean(hdr_check) / mean(hdr) - 1
Frames are normalized by the per-frame light factor (uniform ripple), measured on pixels with a
mid-range signal. Rows are processed in blocks from memory-mapped stacks.
"""
import json
import sys
from pathlib import Path

import numpy as np

CH_CLIP = 4351          # 4095 + pedestal 256: channel outputs clip here
BLOCK = 116             # rows per block (even -> keeps the Bayer phase)


def frame_gains(st, dark_mean, lo, hi):
    """Per-frame light factor from pixels whose mean signal is within [lo, hi]."""
    n, H = st.shape[0], st.shape[1]
    sums, cnt = np.zeros(n), 0
    for r in range(0, H, BLOCK):
        x = st[:, r:r + BLOCK].astype(np.float64) - dark_mean[r:r + BLOCK]
        m = x.mean(0)
        sel = (m > lo) & (m < hi)
        if sel.any():
            sums += x[:, sel].sum(1)
            cnt += sel.sum()
    if cnt == 0:
        return np.ones(n)
    g = sums / cnt
    return g / g.mean()


def stats(st, dark_mean, gains, clip=None):
    n, H, W = st.shape
    mu, var = np.empty((H, W), np.float32), np.empty((H, W), np.float32)
    sat = np.zeros((H, W), bool)
    for r in range(0, H, BLOCK):
        raw = st[:, r:r + BLOCK]
        x = (raw.astype(np.float64) - dark_mean[r:r + BLOCK]) / gains[:, None, None]
        mu[r:r + BLOCK] = x.mean(0)
        var[r:r + BLOCK] = x.var(0, ddof=1)
        if clip is not None:
            sat[r:r + BLOCK] = (raw >= clip).any(0)
    return mu, var, sat


def dark_mean_of(path):
    d = np.load(path, mmap_mode="r")
    out = np.empty(d.shape[1:], np.float64)
    for r in range(0, d.shape[1], BLOCK):
        out[r:r + BLOCK] = d[:, r:r + BLOCK].astype(np.float64).mean(0)
    return out


def main():
    sess = Path(sys.argv[1])
    man = json.loads((sess / "manifest.json").read_text())
    for key, lv in sorted(man["levels"].items(), key=lambda kv: kv[1]["exposure_us"]):
        folder = sess / key
        if (folder / "stats.npz").exists():
            print(key, "done");
            continue
        out = {}
        dh = dark_mean_of(folder / "dark_hdr.npy")
        bh = np.load(folder / "bright_hdr.npy", mmap_mode="r")
        g = frame_gains(bh, dh, 2e3, 1.2e7)
        out["mu_hdr"], out["var_hdr"], _ = stats(bh, dh, g)
        out["gain_hdr"] = g.astype(np.float32)
        if (folder / "bright_hdr_check.npy").exists():
            chk = np.load(folder / "bright_hdr_check.npy", mmap_mode="r")
            out["drift"] = np.float32(np.mean([(f.astype(np.float64) - dh).mean() for f in chk]) / out["mu_hdr"].mean() - 1)
        for c in range(4):
            if not (folder / f"bright_hdrchannel{c}.npy").exists():     # --hdr-only sessions
                continue
            dc = dark_mean_of(folder / f"dark_hdrchannel{c}.npy")
            bc = np.load(folder / f"bright_hdrchannel{c}.npy", mmap_mode="r")
            gc = frame_gains(bc, dc, 200, 3500)
            out[f"mu_ch{c}"], out[f"var_ch{c}"], out[f"sat_ch{c}"] = stats(bc, dc, gc, clip=CH_CLIP)
            out[f"gain_ch{c}"] = gc.astype(np.float32)
            out[f"dark_ch{c}"] = dc.astype(np.float32)
        out["dark_hdr"] = dh.astype(np.float32)
        np.savez_compressed(folder / "stats.npz", **out)
        print(f"{key:>7s}  HDR mean {out['mu_hdr'].mean():12.0f}  light ripple {g.std():.3%}  drift {float(out.get('drift', np.nan)):+.3%}  "
              f"sat ch0..3 {[round(float(out[f'sat_ch{c}'].mean()), 3) for c in range(4) if f'sat_ch{c}' in out]}", flush=True)


if __name__ == "__main__":
    main()
