"""Full-frame dark statistics (lens cap on) for either camera, at gains x exposures.

  source ../env.sh && python fullframe_dark.py lucid --gains 0 6 --exposures-us 1000 30000 300000 --out DIR
  env -u LD_LIBRARY_PATH ../.venv/bin/python fullframe_dark.py bfs --gains 0 12 24 ... --out DIR

Same RAW settings as the PTC captures, but no ROI. Frames are reduced as they arrive (a LUCID
stack of 200 would be 4 GB). Per condition <out>/<camera>/g<gain>_t<exposure>us.npz holds
  mean, var (per pixel, unbiased), var_diff (from successive differences: insensitive to slow
  offset drift), row_means [n, H], col_means [n, W], frames (first --keep raw frames)
and <same>.json the settings, chunk checks, frame-id gaps and temperatures.
Link budget on the shared switch (README): LUCID <= 45 MB/s (2 Hz BayerRG24), BFS <= 20 MB/s.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np


def reduce_frames(grab_chunk, n, keep):
    s = s2 = d2 = prev = None
    rows, cols, kept, metas = [], [], [], []
    while len(metas) < n:
        stack, m, _ = grab_chunk(min(8, n - len(metas)), first=not metas)
        for img in stack:
            x = img.astype(np.float64)
            if s is None:
                s, s2, d2 = np.zeros_like(x), np.zeros_like(x), np.zeros_like(x)
            s += x
            s2 += x * x
            if prev is not None:
                d2 += (x - prev) ** 2
            prev = x
            rows.append(x.mean(1).astype(np.float32))
            cols.append(x.mean(0).astype(np.float32))
            if len(kept) < keep:
                kept.append(img.copy())
        metas += m
    mean = s / n
    return {"mean": mean.astype(np.float32), "var": ((s2 - n * mean * mean) / (n - 1)).astype(np.float32),
            "var_diff": (d2 / (2 * (n - 1))).astype(np.float32), "row_means": np.stack(rows),
            "col_means": np.stack(cols), "frames": np.stack(kept)}, metas


def summary(metas):
    ids = [m["frame_id"] for m in metas]
    return {"n": len(metas), "frame_id_gaps": int(np.sum(np.diff(ids) != 1)),
            "crc_ok": sum(bool(m.get("crc_ok")) for m in metas),
            "chunk_exposure_us": [metas[0].get("ChunkExposureTime"), metas[-1].get("ChunkExposureTime")],
            "chunk_gain_db": [metas[0].get("ChunkGain"), metas[-1].get("ChunkGain")],
            "chunk_black_level": [metas[0].get("ChunkBlackLevel"), metas[-1].get("ChunkBlackLevel")],
            "timestamps_ns": [metas[0]["timestamp_ns"], metas[-1]["timestamp_ns"]]}


def run_lucid(a, out):
    from lucid_common import RAW_ISP_OFF, Settings, enable_chunks, grab, open_lucid, setup_stream
    from ptc_capture import Cam
    system, device, info = open_lucid(a.serial)
    st = Settings(device.nodemap)
    cam = Cam(device, st, True, rate_cap=a.rate_cap or 2.0)
    try:
        setup_stream(device)
        for name, value in RAW_ISP_OFF:
            st.write(name, value)
        st.write("BlackLevelSelector", "All")
        st.write("BlackLevelRaw", a.black_level_raw, required=True)
        st.write("HDROutput", a.hdr_output, required=True)
        st.write("PixelFormat", "BayerRG24", required=True)
        st.write("OffsetX", 0, required=True)
        st.write("OffsetY", 0, required=True)
        st.write("Width", int(st.read("WidthMax")), required=True)
        st.write("Height", int(st.read("HeightMax")), required=True)
        st.write("DeviceLinkThroughputLimitMode", "On")
        st.write("DeviceLinkThroughputLimit", int(a.link_bytes or 45_000_000))
        enable_chunks(st)
        cam.t_range(a.t_max_us)
        for g in a.gains:
            st.write("Gain", float(g), required=True)
            for t in a.exposures_us:
                actual = cam.expose(t)

                def chunk(k, first):
                    return grab(device, k, timeout_ms=int(3 * actual / 1000) + 5000, expect_exposure_us=actual,
                                settle=2 if first else 0)
                tag = f"g{g:02.0f}_t{actual:.0f}us"
                t0 = time.time()
                temp0 = st.read("DeviceTemperature")
                data, metas = reduce_frames(chunk, a.frames, a.keep)
                info_ = {"camera": "TRI054S-C", "serial": info["serial"], "gain_db": st.read("Gain"),
                         "exposure_us": actual, "hdr_output": a.hdr_output, "pixel_format": "BayerRG24",
                         "black_level_raw": a.black_level_raw, "frame_rate_hz": st.read("AcquisitionFrameRate"),
                         "temperature_c": [temp0, st.read("DeviceTemperature")], "seconds": time.time() - t0,
                         **summary(metas)}
                save(out / "lucid", tag, data, info_)
            cam.stop()
    finally:
        try:
            cam.stop()
        except Exception:
            pass
        errors = st.restore()
        system.destroy_device()
        print("restore errors:", errors or "none", flush=True)


def run_bfs(a, out):
    from bfs_ptc_capture import RAW, BfsCam
    from spin_common import BFS, SpinSettings
    dev = BFS(a.serial or "26315393")
    st = SpinSettings(dev.nm)
    sn = SpinSettings(dev.stream)
    cam = BfsCam(dev, st, rate_cap=a.rate_cap or 4.0)
    try:
        for name, kind, value in RAW:
            st.write(name, kind, value)
        for ch in ("Red", "Blue"):
            st.write("BalanceRatio", "float", 1.0, selector=("BalanceRatioSelector", ch))
        st.write("BlackLevel", "float", a.black_level_percent, required=True, selector=("BlackLevelSelector", "All"))
        st.write("OffsetX", "int", 0, required=True)
        st.write("OffsetY", "int", 0, required=True)
        st.write("Width", "int", int(dev.nm.bounds("Width", "int")[1]), required=True)
        st.write("Height", "int", int(dev.nm.bounds("Height", "int")[1]), required=True)
        st.write("DeviceLinkThroughputLimit", "int", int(a.link_bytes or 20_000_000))
        st.write("ChunkModeActive", "bool", True)
        for c in ("ExposureTime", "Gain", "BlackLevel", "CRC", "FrameID", "Timestamp"):
            st.write("ChunkEnable", "bool", True, selector=("ChunkSelector", c))
        sn.write("StreamBufferHandlingMode", "enum", "OldestFirst")
        sn.write("StreamPacketResendEnable", "bool", True)
        for g in a.gains:
            st.write("Gain", "float", float(g), required=True)
            for t in a.exposures_us:
                actual = cam.expose(t)

                def chunk(k, first):
                    return dev.grab(k, timeout_ms=int(3 * actual / 1000) + 5000, expect_exposure_us=actual,
                                    settle=2 if first else 0)
                tag = f"g{g:02.0f}_t{actual:.0f}us"
                t0 = time.time()
                temp0 = st.read("DeviceTemperature")
                data, metas = reduce_frames(chunk, a.frames, a.keep)
                data["frames"] = data["frames"].astype(np.uint16)
                info_ = {"camera": "BFS-PGE-23S3C", "serial": a.serial or "26315393", "gain_db": st.read("Gain"),
                         "exposure_us": actual, "pixel_format": "BayerRG16", "adc_bit_depth": "Bit12",
                         "black_level_percent": st.read("BlackLevel"), "frame_rate_hz": st.read("AcquisitionFrameRate"),
                         "temperature_c": [temp0, st.read("DeviceTemperature")], "seconds": time.time() - t0,
                         **summary(metas)}
                save(out / "bfs", tag, data, info_)
            dev.stop()
    finally:
        try:
            dev.stop()
        except Exception:
            pass
        errors = st.restore() + sn.restore()
        dev.close()
        print("restore errors:", errors or "none", flush=True)


def save(folder, tag, data, info_):
    folder.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(folder / f"{tag}.npz", **data)
    (folder / f"{tag}.json").write_text(json.dumps(info_, indent=1, default=str))
    m = data["mean"]
    print(f"  {tag:22s} n={info_['n']} gaps={info_['frame_id_gaps']} crc={info_['crc_ok']}  "
          f"offset med {np.median(m):9.1f}  temporal σ med {np.sqrt(np.median(data['var_diff'])):7.2f}  "
          f"{info_['seconds']:.0f}s", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera", choices=["lucid", "bfs"])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--serial")
    ap.add_argument("--gains", type=float, nargs="+", default=[0.0])
    ap.add_argument("--exposures-us", type=float, nargs="+", default=[1000, 30000, 300000])
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--keep", type=int, default=20, help="raw frames saved per condition")
    ap.add_argument("--hdr-output", default="HDR")
    ap.add_argument("--black-level-raw", type=int, default=256)
    ap.add_argument("--black-level-percent", type=float, default=1.0)
    ap.add_argument("--rate-cap", type=float)
    ap.add_argument("--link-bytes", type=int)
    ap.add_argument("--t-max-us", type=float, default=2.5e6)
    a = ap.parse_args()
    (run_lucid if a.camera == "lucid" else run_bfs)(a, a.out)


if __name__ == "__main__":
    main()
