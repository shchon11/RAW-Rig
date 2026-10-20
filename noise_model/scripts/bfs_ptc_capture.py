"""PTC capture for the BFS-PGE-23S3C (IMX392), same session format as ptc_capture.py.

  python bfs_ptc_capture.py bright --session ../data/ptc_<name>
  python bfs_ptc_capture.py dark   --session ../data/ptc_<name>     # lens cap on

Settings = FLIR_control docs/raw_capture_noise_calibration.md §6.1 (0 dB, Bit12, BayerRG16,
BlackLevel 1 %, clamp off, WB 1.0, gamma/LUT/defect off, centre ROI 256x256 at x=832 y=472).
Output name in the session: "BFS". Run with plain python (no Arena LD_LIBRARY_PATH).
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from ptc_capture import find_top, g_mean, save_level
from spin_common import BFS, SpinSettings

OUTPUT = "BFS"
RAW = (  # (node, kind, value); order matters (auto before value, size before offset)
    ("TriggerMode", "enum", "Off"), ("ExposureMode", "enum", "Timed"), ("AcquisitionMode", "enum", "Continuous"),
    ("ExposureAuto", "enum", "Off"), ("GainAuto", "enum", "Off"), ("Gain", "float", 0.0),
    ("BalanceWhiteAuto", "enum", "Off"),
    ("GammaEnable", "bool", False), ("LUTEnable", "bool", False),
    ("DefectCorrectStaticEnable", "bool", False), ("IspEnable", "bool", False),
    ("ReverseX", "bool", False), ("ReverseY", "bool", False),
    ("AdcBitDepth", "enum", "Bit12"), ("PixelFormat", "enum", "BayerRG16"),
    ("BlackLevelClampingEnable", "bool", False),
    ("GevSCPSPacketSize", "int", 1400),  # enp5s0 MTU 1500
)


class BfsCam:
    """expose()/grab() interface of ptc_capture.Cam; frame rate follows the exposure (cap 30 Hz)."""

    def __init__(self, dev, st, rate_cap=30.0):
        self.dev, self.st, self.rate_cap = dev, st, rate_cap

    def expose(self, t):
        """Free-run (rate follows exposure) below the ~1 Hz rate floor, else cap at 30 Hz to keep
        the shared link light."""
        st, nm = self.st, self.dev.nm
        self.dev.stop()
        st.write("AcquisitionFrameRateEnable", "bool", False, required=True)
        st.write("ExposureTime", "float", float(t), required=True)
        actual = float(st.read("ExposureTime"))
        rate = min(self.rate_cap, 1e6 / (actual + 1000.0))
        st.write("AcquisitionFrameRateEnable", "bool", True, required=True)
        lo, hi = nm.bounds("AcquisitionFrameRate")
        if rate >= lo:
            st.write("AcquisitionFrameRate", "float", min(rate, hi), required=True)
        else:
            st.write("AcquisitionFrameRateEnable", "bool", False, required=True)
        self.dev.start()
        return float(st.read("ExposureTime"))

    def grab(self, n, t):
        return self.dev.grab(n, timeout_ms=int(3 * t / 1000) + 3000, expect_exposure_us=t)


def configure(dev, st, a):
    for name, kind, value in RAW:
        st.write(name, kind, value)
    for ch in ("Red", "Blue"):
        st.write("BalanceRatio", "float", 1.0, selector=("BalanceRatioSelector", ch))
    st.write("BlackLevel", "float", a.black_level_percent, required=True, selector=("BlackLevelSelector", "All"))
    st.write("Width", "int", a.roi, required=True)
    st.write("Height", "int", a.roi, required=True)
    st.write("OffsetX", "int", a.offset_x, required=True)
    st.write("OffsetY", "int", a.offset_y, required=True)
    st.write("ChunkModeActive", "bool", True)
    for c in ("ExposureTime", "Gain", "BlackLevel", "CRC", "FrameID", "Timestamp"):
        st.write("ChunkEnable", "bool", True, selector=("ChunkSelector", c))
    sn = SpinSettings(dev.stream)
    sn.write("StreamBufferHandlingMode", "enum", "OldestFirst")
    sn.write("StreamPacketResendEnable", "bool", True)
    return sn


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("phase", choices=["bright", "dark"])
    ap.add_argument("--session", type=Path, required=True)
    ap.add_argument("--serial", default="26315393")
    ap.add_argument("--levels", type=int, default=16)
    ap.add_argument("--frames", type=int, default=100)
    ap.add_argument("--drift-frames", type=int, default=20)
    ap.add_argument("--roi", type=int, default=256)
    ap.add_argument("--offset-x", type=int, default=832)
    ap.add_argument("--offset-y", type=int, default=472)
    ap.add_argument("--black-level-percent", type=float, default=1.0)
    ap.add_argument("--offset-dn", type=float, default=633.0, help="approx. offset, only for the T_top target")
    ap.add_argument("--target", type=float, default=0.8)
    ap.add_argument("--t-max-us", type=float, default=2.5e6)
    ap.add_argument("--gain-db", type=float, default=0.0, help="output name BFS (0 dB) or BFS_gNN")
    a = ap.parse_args()
    global OUTPUT
    if a.gain_db:
        OUTPUT = f"BFS_g{a.gain_db:02.0f}"

    a.session.mkdir(parents=True, exist_ok=True)
    mpath = a.session / "manifest.json"
    manifest = json.loads(mpath.read_text()) if mpath.exists() else {"outputs": {}}
    if a.phase == "dark" and OUTPUT not in manifest["outputs"]:
        raise SystemExit("run the BFS bright phase first")

    dev = BFS(a.serial)
    st = SpinSettings(dev.nm)
    sn = None
    cam = BfsCam(dev, st)
    rec = manifest["outputs"].setdefault(OUTPUT, {})
    try:
        sn = configure(dev, st, a)
        st.write("Gain", "float", a.gain_db, required=True)
        rec.update({"camera": "BFS-PGE-23S3C", "serial": a.serial, "pixel_format": "BayerRG16", "roi": a.roi,
                    "gain_db": st.read("Gain"),
                    "roi_offset": [a.offset_x, a.offset_y], "black_level_percent": st.read("BlackLevel")})
        folder = a.session / OUTPUT
        folder.mkdir(exist_ok=True)
        st.write("AcquisitionFrameRateEnable", "bool", False, required=True)
        t_min = float(dev.nm.bounds("ExposureTime")[0])
        t_max = min(float(dev.nm.bounds("ExposureTime")[1]), a.t_max_us)
        rec[f"{a.phase}_temperature_start_c"] = st.read("DeviceTemperature")
        rec[f"{a.phase}_time"] = time.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{OUTPUT}] {a.phase}  T range {t_min:.1f}-{t_max:.1f} us  temp {rec[f'{a.phase}_temperature_start_c']} C",
              flush=True)
        if a.phase == "bright":
            t_top, white, note = find_top(cam, a, t_min, t_max, a.offset_dn)
            print(f"  T_top={t_top:.1f} us  white={white}  {note}", flush=True)
            rec.update({"t_top_us": t_top, "white_level": white, "note": note, "t_min_us": t_min, "t_max_us": t_max})
            targets = list(np.geomspace(t_min, t_top, a.levels)) if t_top > t_min else [t_min]
        else:
            targets = [lv["exposure_us"] for lv in rec["bright"]]
        levels = []
        for k, t in enumerate(targets):
            actual = cam.expose(t)
            if a.phase == "bright" and levels and abs(actual - levels[-1]["exposure_us"]) < 1.0:
                continue
            stack, metas, skipped = cam.grab(a.frames, actual)
            lv = {"exposure_us": actual, "skipped": skipped, "temperature_c": st.read("DeviceTemperature"),
                  "frame_rate_hz": st.read("AcquisitionFrameRate"), **save_level(folder, f"{a.phase}_{k:02d}", stack, metas)}
            levels.append(lv)
            print(f"  {a.phase} {k:2d}  T={actual:10.1f} us  G mean={g_mean(stack):10.1f}  max={int(stack.max()):6d}  "
                  f"crc {lv['crc_ok']}/{lv['n']}  skip {skipped}  {lv['frame_rate_hz']:.2f} Hz", flush=True)
        if a.phase == "bright" and a.drift_frames:
            actual = cam.expose(targets[-1])
            stack, metas, _ = cam.grab(a.drift_frames, actual)
            rec["drift"] = {"exposure_us": actual, **save_level(folder, "bright_drift", stack, metas)}
        rec[a.phase] = levels
        dev.stop()
        rec[f"{a.phase}_temperature_end_c"] = st.read("DeviceTemperature")
    finally:
        try:
            dev.stop()
        except Exception:
            pass
        errors = st.restore() + (sn.restore() if sn else [])
        rec.setdefault("settings_logs", {})[a.phase] = st.log
        rec.setdefault("restore_errors", {})[a.phase] = errors
        dev.close()
        # merge: the LUCID capture may write the same manifest concurrently
        latest = json.loads(mpath.read_text()) if mpath.exists() else {"outputs": {}}
        latest.setdefault("outputs", {})[OUTPUT] = rec
        mpath.write_text(json.dumps(latest, ensure_ascii=False, indent=1, default=str))
        print("restore errors:", errors or "none")
        print("session:", a.session.resolve())


if __name__ == "__main__":
    main()
