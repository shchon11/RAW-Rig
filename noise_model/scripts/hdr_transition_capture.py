"""LUCID HDR channel-transition measurement: full-frame HDR + per-channel stacks at the same exposures.

  source ../env.sh
  python hdr_transition_capture.py bright --session ../data/hdr_transition_<date> [--exposures-ms ...]
  python hdr_transition_capture.py dark   --session ../data/hdr_transition_<date>      # lens cap on

Per exposure (free-run, frame rate <= 2 Hz, 0 dB, BayerRG24, BlackLevelRaw 256, ISP off):
  HDR x --hdr-frames, then HDRChannel0..3 x --ch-frames, then HDR x --check-frames (light drift check).
Static scene: the same pixels seen through each output give the combination rule (which channel,
scale, blending, 2x2 decision) and the noise across the switch points. Full stacks are kept
(uint32 .npy) so frame-level light ripple can be removed in analysis. Dark repeats the same plan.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from lucid_common import RAW_ISP_OFF, Settings, enable_chunks, grab, open_lucid, setup_stream
from ptc_capture import Cam

OUTPUTS = ["HDRChannel0", "HDRChannel1", "HDRChannel2", "HDRChannel3"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("phase", choices=["bright", "dark"])
    ap.add_argument("--session", type=Path, required=True)
    ap.add_argument("--exposures-ms", type=float, nargs="+",
                    default=[4, 6, 8, 10, 12.5, 15, 20, 30, 50, 70, 90, 120, 160, 250])
    ap.add_argument("--hdr-frames", type=int, help="default: bright 48, dark 16")
    ap.add_argument("--ch-frames", type=int, help="default: bright 12, dark 8")
    ap.add_argument("--check-frames", type=int, help="default: bright 6, dark 0")
    ap.add_argument("--rate-cap", type=float, default=2.0)
    ap.add_argument("--gain-db", type=float, help="default: 0 for bright; dark repeats the bright session's gain")
    ap.add_argument("--hdr-only", action="store_true", help="skip the per-channel stacks")
    a = ap.parse_args()
    # dark only has to pin the offsets (dark sigma ~5 DN vs hundreds of DN of signal noise at the switch points)
    dflt = {"bright": (48, 12, 6), "dark": (16, 8, 0)}[a.phase]
    a.hdr_frames = a.hdr_frames or dflt[0]
    a.ch_frames = a.ch_frames or dflt[1]
    a.check_frames = a.check_frames if a.check_frames is not None else dflt[2]
    a.session.mkdir(parents=True, exist_ok=True)
    mpath = a.session / "manifest.json"
    man = json.loads(mpath.read_text()) if mpath.exists() else {"levels": {}}
    if a.phase == "dark" and not man["levels"]:
        raise SystemExit("run bright first")
    if a.phase == "dark":
        a.hdr_only = man.get("hdr_only", a.hdr_only)
        if a.gain_db is None:
            a.gain_db = float(man.get("gain_db", 0.0))
    if a.gain_db is None:
        a.gain_db = 0.0
    man["hdr_only"] = a.hdr_only
    exposures = [float(e) * 1000 for e in a.exposures_ms] if a.phase == "bright" else \
        sorted({lv["exposure_us"] for lv in man["levels"].values()})

    system, device, info = open_lucid()
    st = Settings(device.nodemap)
    cam = Cam(device, st, True, rate_cap=a.rate_cap)
    man.update({"serial": info["serial"], "model": info["model"], "gain_db": a.gain_db, "black_level_raw": 256,
                "pixel_format": "BayerRG24", "frame_rate_cap_hz": a.rate_cap})
    try:
        setup_stream(device)
        for name, value in RAW_ISP_OFF:
            st.write(name, value)
        st.write("Gain", float(a.gain_db), required=True)
        st.write("BlackLevelSelector", "All")
        st.write("BlackLevelRaw", 256, required=True)
        st.write("PixelFormat", "BayerRG24", required=True)
        st.write("OffsetX", 0, required=True)
        st.write("OffsetY", 0, required=True)
        st.write("Width", int(st.read("WidthMax")), required=True)
        st.write("Height", int(st.read("HeightMax")), required=True)
        st.write("DeviceLinkThroughputLimitMode", "On")
        st.write("DeviceLinkThroughputLimit", 45_000_000)
        enable_chunks(st)
        cam.t_range(800_000)
        plan = [("HDR", a.hdr_frames, "hdr")] + ([] if a.hdr_only else [(o, a.ch_frames, o.lower()) for o in OUTPUTS]) + \
               ([("HDR", a.check_frames, "hdr_check")] if a.check_frames else [])
        t_start = time.time()
        for k, T in enumerate(exposures):
            key = f"{T / 1000:g}ms"
            lv = man["levels"].setdefault(key, {"exposure_us": T})
            folder = a.session / key
            folder.mkdir(exist_ok=True)
            for output, n, tag in plan:
                cam.stop()
                st.write("HDROutput", output, required=True)
                st.write("PixelFormat", "BayerRG24", required=True)
                actual = cam.expose(T)
                stack, metas, skipped = grab(device, n, timeout_ms=int(3 * actual / 1000) + 5000,
                                             expect_exposure_us=actual)
                np.save(folder / f"{a.phase}_{tag}.npy",          # channel outputs <= 4351 -> uint16
                        stack.astype(np.uint16 if output != "HDR" else np.uint32))
                lv.setdefault(a.phase, {})[tag] = {
                    "output": output, "n": n, "exposure_us": actual, "skipped": skipped,
                    "frame_ids": [m["frame_id"] for m in metas], "crc_ok": sum(bool(m.get("crc_ok")) for m in metas),
                    "frame_rate_hz": st.read("AcquisitionFrameRate"), "temperature_c": st.read("DeviceTemperature"),
                    "uptime_s": st.read("DeviceUpTime"), "frame_mean_g": [float(f[0::2, 1::2].mean()) for f in stack]}
                g = np.mean(lv[a.phase][tag]["frame_mean_g"])
                print(f"[{k + 1:2d}/{len(exposures)}] {a.phase} {key:>7s} {tag:12s} n={n:3d} G mean {g:12.1f} "
                      f"max {int(stack.max()):9d} crc {lv[a.phase][tag]['crc_ok']}/{n} "
                      f"({(time.time() - t_start) / 60:.1f} min)", flush=True)
            mpath.write_text(json.dumps(man, indent=1, default=float))
    finally:
        try:
            cam.stop()
        except Exception:
            pass
        man.setdefault("restore_errors", {})[a.phase] = st.restore()
        system.destroy_device()
        mpath.write_text(json.dumps(man, indent=1, default=float))
        print("restore errors:", man["restore_errors"][a.phase] or "none")
        print("session:", a.session.resolve())


if __name__ == "__main__":
    main()
