"""RGB preview of a fullframe_dark.py npz (2x2 binned, dark subtracted, log or gamma scaled) with a grid.

  python rgb_preview.py <npz> <out.png> [--cal TRI054S-C_254600231] [--log]

Used to check what a scene puts on the sensor (colour patches, saturation) before a capture.
Prints quad R/G/B means on a coarse grid (cells of 1/8 of the frame) so patches can be located.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--cal", default="TRI054S-C_254600231")
    ap.add_argument("--white", type=float, default=2 ** 24 - 1)
    ap.add_argument("--log", action="store_true")
    a = ap.parse_args()
    info = json.loads(a.npz.with_suffix(".json").read_text())
    T, g = info["exposure_us"], int(round(info["gain_db"]))
    maps = np.load(ROOT / "calibration" / a.cal / f"maps_g{g:02d}.npz")
    raw = np.load(a.npz)["mean"].astype(np.float64)
    x = raw - (maps["offset_1ms_dn"] + maps["dark_rate_dn_per_s"] * (T - 1000) / 1e6)
    R, G, B = x[0::2, 0::2], (x[0::2, 1::2] + x[1::2, 0::2]) / 2, x[1::2, 1::2]
    rgb = np.stack([R, G, B], -1).clip(1, None)
    sat = (raw >= a.white * 0.999)
    sat_q = sat[0::2, 0::2] | sat[0::2, 1::2] | sat[1::2, 0::2] | sat[1::2, 1::2]
    if a.log:
        img = np.log(rgb) / np.log(rgb.max())
    else:
        img = (rgb / np.percentile(rgb, 99.5)).clip(0, 1) ** (1 / 2.2)
    img[sat_q] = [1, 0, 1]                                 # saturated quads in magenta
    h, w = G.shape
    fig, ax = plt.subplots(figsize=(12, 12 * h / w))
    ax.imshow(img.clip(0, 1))
    for k in range(1, 8):
        ax.axhline(k * h / 8, color="w", lw=0.5, alpha=0.5)
        ax.axvline(k * w / 8, color="w", lw=0.5, alpha=0.5)
    for i in range(8):
        for j in range(8):
            cell = (slice(i * h // 8, (i + 1) * h // 8), slice(j * w // 8, (j + 1) * w // 8))
            gm = np.median(G[cell])
            ax.text((j + 0.5) * w / 8, (i + 0.5) * h / 8, f"G {gm / 1e3:.0f}k\nR/G {np.median(R[cell]) / gm:.2f}\nB/G {np.median(B[cell]) / gm:.2f}",
                    ha="center", va="center", fontsize=7, color="w", bbox=dict(fc="k", alpha=0.4, ec="none"))
    ax.set_title(f"{a.npz.stem}  exposure {T / 1000:g} ms  (magenta = saturated)")
    ax.axis("off")
    fig.savefig(a.out, dpi=90, bbox_inches="tight")
    print("saturated quads:", f"{sat_q.mean():.2%}", "->", a.out)


if __name__ == "__main__":
    main()
