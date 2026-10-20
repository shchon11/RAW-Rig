"""Is the scene ready for PTC? Full-frame preview + center-ROI level, uniformity and flicker.

  python scene_check.py [--output HDRChannel0] [--exposures-us 100 1000 10000 40000]

Writes data/scene_check_<time>.png (log-scaled full frame, PTC ROI boxed). Restores settings.
"""
import argparse
import time
from pathlib import Path

import numpy as np

from lucid_common import RAW_ISP_OFF, Settings, enable_chunks, grab, open_lucid, setup_stream


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial")
    ap.add_argument("--output", default="HDRChannel0")
    ap.add_argument("--exposures-us", type=float, nargs="+", default=[100, 1000, 10000, 40000])
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--roi", type=int, default=256)
    ap.add_argument("--black-level-raw", type=int, default=256)
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent.parent / "data")
    a = ap.parse_args()

    system, device, info = open_lucid(a.serial)
    st = Settings(device.nodemap)
    try:
        setup_stream(device)
        for name, value in RAW_ISP_OFF:
            st.write(name, value)
        st.write("Gain", 0.0, required=True)
        st.write("BlackLevelRaw", a.black_level_raw, required=True)
        st.write("HDROutput", a.output, required=True)
        st.write("PixelFormat", "BayerRG24", required=True)
        enable_chunks(st)
        h, w = int(st.read("HeightMax")), int(st.read("WidthMax"))
        s = a.roi
        y0, x0 = (h - s) // 2 // 2 * 2, (w - s) // 2 // 4 * 4
        t_max = float(st.node("ExposureTime").max)
        rows, preview = [], None
        print(f"{a.output}  BL {a.black_level_raw}  ROI {s}x{s} at x={x0} y={y0}  temp {st.read('DeviceTemperature')} C")
        print(f"{'T us':>9s} {'G mean':>10s} {'G-off':>10s} {'max':>9s} {'ROI CV':>7s} {'frame CV':>8s} {'pair Δ':>7s}")
        with device.start_stream(8):
            for t in a.exposures_us:
                st.write("ExposureTime", float(min(t, t_max)), required=True)
                actual = float(st.read("ExposureTime"))
                stack, _, _ = grab(device, a.frames, expect_exposure_us=actual)
                roi = stack[:, y0:y0 + s, x0:x0 + s].astype(np.float64)
                g = np.concatenate([roi[:, 0::2, 1::2].reshape(len(roi), -1), roi[:, 1::2, 0::2].reshape(len(roi), -1)], 1)
                fm = g.mean(1) - a.black_level_raw
                sig = fm.mean()
                gm = (roi[:, 0::2, 1::2] + roi[:, 1::2, 0::2]).mean(0) / 2 - a.black_level_raw
                pair = np.abs(fm[0::2] - fm[1::2]) / max(sig, 1e-9)
                rows.append((actual, sig))
                print(f"{actual:9.1f} {g.mean():10.1f} {sig:10.1f} {int(stack.max()):9d} {gm.std() / max(sig, 1e-9):7.2%} "
                      f"{fm.std() / max(sig, 1e-9):8.3%} {np.median(pair):7.3%}", flush=True)
                if preview is None or sig < 0.7 * np.iinfo(np.int32).max:
                    preview = (actual, stack.mean(0))
    finally:
        try:
            device.stop_stream()
        except Exception:
            pass
        errors = st.restore()
        system.destroy_device()
        print("restore errors:", errors or "none")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    t, img = preview
    g = (img[0::2, 1::2] + img[1::2, 0::2]) / 2 - a.black_level_raw
    fig, ax = plt.subplots(figsize=(10, 6.6), facecolor="#fcfcfb")
    ax.imshow(np.log10(np.clip(g, 1, None)), cmap="gray")
    ax.add_patch(Rectangle((x0 / 2, y0 / 2), s / 2, s / 2, fill=False, ec="#eb6834", lw=2))
    ax.set_title(f"{a.output} G (log10 DN above pedestal), T={t:.0f} µs · box = PTC ROI", fontsize=10)
    ax.axis("off")
    path = a.out / f"scene_check_{time.strftime('%Y%m%d_%H%M%S')}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    print("preview:", path)


if __name__ == "__main__":
    main()
