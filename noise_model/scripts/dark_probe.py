"""Lens-cap dark probe for the TRI054S: per-output offset, read noise, code step and clipping.

  python dark_probe.py [--serial S] [--frames 16] [--black-levels 0 64] [--exposures-us min 20000]

Needs the lens cap on. Restores every touched setting on exit.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from lucid_common import (RAW_ISP_OFF, Settings, center_roi, cfa_stats, enable_chunks, grab, open_lucid,
                          setup_stream)

OUTPUTS = ("HDR", "HDRChannel0", "HDRChannel1", "HDRChannel2", "HDRChannel3")


def channel_table(st):
    rows = []
    for c in range(4):
        if not st.write("HDRTuningChannelSelector", c):
            break
        rows.append({"channel": c, **{k: st.read(k) for k in (
            "HDRChannelAnalogGain", "HDRChannelAnalogGainRaw", "HDRChannelDigitalGain",
            "HDRChannelDigitalGainRaw", "HDRChannelExposureTime")}})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial")
    ap.add_argument("--frames", type=int, default=16)
    ap.add_argument("--roi", type=int, default=512)
    ap.add_argument("--black-levels", type=int, nargs="+", default=[0, 64], help="BlackLevelRaw values")
    ap.add_argument("--exposures-us", nargs="+", default=["min", "20000"])
    ap.add_argument("--outputs", nargs="+", default=list(OUTPUTS))
    ap.add_argument("--pixel-format", default="BayerRG24")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent.parent / "data")
    a = ap.parse_args()

    system, device, info = open_lucid(a.serial)
    out = a.out / f"dark_probe_{info['serial']}_{time.strftime('%Y%m%d_%H%M%S')}"
    out.mkdir(parents=True)
    nm = device.nodemap
    st = Settings(nm)
    report = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "serial": info["serial"], "model": info["model"],
              "temperature_start_c": st.read("DeviceTemperature"), "conditions": []}
    try:
        setup_stream(device)
        for name, value in RAW_ISP_OFF:
            st.write(name, value)
        st.write("Gain", 0.0, required=True)
        st.write("BlackLevelSelector", "All")
        enable_chunks(st)
        report["hdr_channels"] = channel_table(st)
        print("HDR channels:", json.dumps(report["hdr_channels"]))
        h, w = int(st.read("HeightMax")), int(st.read("WidthMax"))
        roi = center_roi(h, w, a.roi)
        report["roi_y_x_size"] = roi

        for output in a.outputs:
            if not st.write("HDROutput", output):
                print(f"[{output}] HDROutput not settable, skipped")
                continue
            fmts = list(st.node("PixelFormat").enumentry_names)
            if not st.write("PixelFormat", a.pixel_format):
                print(f"[{output}] PixelFormat {a.pixel_format} not settable; entries={fmts}")
                continue
            t_min = float(st.node("ExposureTime").min)
            exposures = [t_min if e == "min" else float(e) for e in a.exposures_us]
            with device.start_stream(8):
                for bl in a.black_levels:
                    st.write("BlackLevelRaw", bl)
                    for t in exposures:
                        st.write("ExposureTime", t)
                        actual_t = float(st.read("ExposureTime"))
                        stack, metas, skipped = grab(device, a.frames, roi=roi, expect_exposure_us=actual_t)
                        stats = cfa_stats(stack)
                        cond = {"output": output, "pixel_format": a.pixel_format, "black_level_raw": bl,
                                "black_level_pct": st.read("BlackLevel"), "exposure_us": actual_t,
                                "skipped": skipped, "frame_ids": [m["frame_id"] for m in metas],
                                "chunk_first": metas[0], "stats": stats, "pixel_formats_available": fmts,
                                "temperature_c": st.read("DeviceTemperature")}
                        report["conditions"].append(cond)
                        tag = f"{output}_bl{bl}_t{actual_t:.0f}us"
                        np.savez_compressed(out / f"{tag}.npz", stack=stack)
                        g = stats["G1"]
                        print(f"[{tag:30s}] G1 mean={g['mean']:10.1f} tstd={g['temporal_std']:8.2f} "
                              f"dsnu={g['dsnu_std']:7.2f} min={g['min']:7d} max={g['max']:9d} step={g['code_step']:5d} "
                              f"@min={g['frac_at_min']:6.2%} zero={g['frac_zero']:6.2%} skip={skipped}", flush=True)
    finally:
        try:
            device.stop_stream()
        except Exception:
            pass
        report["temperature_end_c"] = st.read("DeviceTemperature")
        report["settings_log"] = st.log
        report["restore_errors"] = st.restore()
        system.destroy_device()
        (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str))
        print("restore errors:", report["restore_errors"] or "none")
        print("saved:", out)


if __name__ == "__main__":
    main()
