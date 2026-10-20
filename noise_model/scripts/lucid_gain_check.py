"""How does the LUCID `Gain` node act? Analog per HDR channel, or digital after the ADC?

  source ../env.sh && python lucid_gain_check.py [--gains 0 6 12 24] [--exposure-us 300]

Per gain: the four HDRChannelAnalogGain values, and per output (HDRChannel0..3, HDR) the G mean
above the pedestal and the code step (gcd of distinct values) at a fixed exposure, centre ROI.
Digital gain shows as a code step > 1 and an exact 10^(dB/20) mean ratio. Restores everything.
"""
import argparse
import json

import numpy as np

from lucid_common import RAW_ISP_OFF, Settings, enable_chunks, grab, open_lucid, setup_stream


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gains", type=float, nargs="+", default=[0, 6, 12, 24])
    ap.add_argument("--exposure-us", type=float, default=300.0)
    ap.add_argument("--outputs", nargs="+", default=["HDRChannel0", "HDRChannel1", "HDRChannel2", "HDRChannel3", "HDR"])
    a = ap.parse_args()
    system, dev, _ = open_lucid()
    st = Settings(dev.nodemap)
    rows = []
    try:
        setup_stream(dev)
        for n, v in RAW_ISP_OFF:
            st.write(n, v)
        st.write("BlackLevelRaw", 256)
        st.write("PixelFormat", "BayerRG24")
        st.write("Width", 256)
        st.write("Height", 256)
        st.write("OffsetX", 1312)
        st.write("OffsetY", 802)
        enable_chunks(st)
        st.write("ExposureTime", a.exposure_us)
        for out in a.outputs:
            st.write("HDROutput", out, required=True)
            st.write("PixelFormat", "BayerRG24")
            for g in a.gains:
                st.write("Gain", float(g), required=True)
                analog = []
                for c in range(4):
                    st.write("HDRTuningChannelSelector", c)
                    analog.append(st.read("HDRChannelAnalogGain"))
                with dev.start_stream(8):
                    s, m, _ = grab(dev, 8, expect_exposure_us=float(st.read("ExposureTime")))
                gvals = np.concatenate([s[:, 0::2, 1::2].ravel(), s[:, 1::2, 0::2].ravel()])
                u = np.unique(gvals)
                step = int(np.gcd.reduce(np.diff(u))) if u.size > 1 else 0
                row = {"output": out, "gain_db": g, "chunk_gain": m[0].get("ChunkGain"), "analog_db": analog,
                       "g_minus_pedestal": float(gvals.mean() - 256), "code_step": step, "max": int(gvals.max())}
                rows.append(row)
                print(f"{out:12s} gain {g:4.0f} dB  chunk {row['chunk_gain']}  ch analog {analog}  "
                      f"G-256 {row['g_minus_pedestal']:12.1f}  step {step:4d}  max {row['max']}", flush=True)
    finally:
        try:
            dev.stop_stream()
        except Exception:
            pass
        print("restore", st.restore() or "ok")
        system.destroy_device()
    base = {r["output"]: r["g_minus_pedestal"] for r in rows if r["gain_db"] == a.gains[0]}
    for r in rows:
        r["ratio_vs_first"] = r["g_minus_pedestal"] / base[r["output"]] if base[r["output"]] else None
        r["expected_ratio"] = 10 ** ((r["gain_db"] - a.gains[0]) / 20)
    print(json.dumps([{k: r[k] for k in ("output", "gain_db", "ratio_vs_first", "expected_ratio", "code_step")}
                      for r in rows], indent=0))


if __name__ == "__main__":
    main()
