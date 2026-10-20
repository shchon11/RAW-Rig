"""PTC capture for the TRI054S (IMX490): exposure sweep at fixed light, then matched dark.

  # 1) light on, flat target (white paper / diffuser), lens + aperture fixed
  python ptc_capture.py bright --session ../data/ptc_<name>
  # 2) lens cap on, nothing else touched
  python ptc_capture.py dark   --session ../data/ptc_<name>

Per output (HDROutput = HDRChannel0..3, HDR), the bright phase finds T_top where the G mean reaches
--target of the saturation range (or the longest exposure if the light is too weak), then captures
--levels log-spaced exposures from the minimum exposure to T_top, --frames each, plus a drift repeat
of the first level. The dark phase repeats the same exposure list. Every frame is checked against
chunk ExposureTime / CRC; frames left over from the previous setting are dropped.
Follows FLIR_control docs/raw_capture_noise_calibration.md §6 with IMX490 per-channel outputs.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from lucid_common import RAW_ISP_OFF, Settings, enable_chunks, grab, open_lucid, setup_stream

OUTPUTS = ("HDRChannel0", "HDRChannel1", "HDRChannel2", "HDRChannel3", "HDR")


def configure(st, output, a):
    for name, value in RAW_ISP_OFF:
        st.write(name, value)
    st.write("Gain", float(a.gain_db), required=True)
    st.write("BlackLevelSelector", "All")
    st.write("BlackLevelRaw", a.black_level_raw, required=True)
    st.write("HDROutput", output, required=True)
    st.write("PixelFormat", "BayerRG24", required=True)
    # ROI: size first, then offset (restore runs in reverse: offset back to 0 before size grows).
    s = a.roi
    st.write("Width", s, required=True)
    st.write("Height", s, required=True)
    st.write("OffsetX", (int(st.read("WidthMax")) - s) // 2 // 4 * 4, required=True)
    st.write("OffsetY", (int(st.read("HeightMax")) - s) // 2 // 2 * 2, required=True)
    enable_chunks(st)


class Cam:
    """Exposure control and streaming. With --long the frame rate follows each exposure
    (AcquisitionFrameRate caps ExposureTime: 1 Hz -> 1 s, 0.5 Hz -> 2 s), and both are
    written only while the stream is stopped."""

    def __init__(self, device, st, long_mode, rate_cap=None):
        self.device, self.st, self.long, self.on = device, st, long_mode, False
        self.rate_cap = rate_cap  # Hz; full-frame BayerRG24 needs <= 2 Hz on the shared link

    def start(self):
        if not self.on:
            self.device.start_stream(8)
            self.on = True

    def stop(self):
        if self.on:
            self.device.stop_stream()
            self.on = False

    def t_range(self, t_max_long):
        st = self.st
        if not self.long:
            return float(st.node("ExposureTime").min), float(st.node("ExposureTime").max)
        st.write("AcquisitionFrameRateEnable", True, required=True)
        st.write("AcquisitionFrameRate", float(st.node("AcquisitionFrameRate").min), required=True)
        return float(st.node("ExposureTime").min), min(float(st.node("ExposureTime").max), t_max_long)

    def expose(self, t):
        st = self.st
        if self.long:
            self.stop()
            st.write("AcquisitionFrameRate", float(st.node("AcquisitionFrameRate").min), required=True)
            st.write("ExposureTime", float(t), required=True)
            rate = min(float(st.node("AcquisitionFrameRate").max), 1e6 / (float(st.read("ExposureTime")) + 500.0))
            if self.rate_cap:
                rate = min(rate, self.rate_cap)
            st.write("AcquisitionFrameRate", rate, required=True)
        else:
            st.write("ExposureTime", float(t), required=True)
        self.start()
        return float(st.read("ExposureTime"))

    def grab(self, n, t):
        return grab(self.device, n, timeout_ms=int(3 * t / 1000) + 3000, expect_exposure_us=t)


def g_mean(stack):
    return float(np.mean([stack[:, 0::2, 1::2].mean(), stack[:, 1::2, 0::2].mean()]))


def find_top(cam, a, t_min, t_max, offset):
    """Return (t_top, white_level or None, note)."""
    t = cam.expose(t_max)
    stack, _, _ = cam.grab(4, t)
    top = int(stack.max())
    at_top = float(np.mean(stack == top))
    if at_top < 0.01:
        return t, None, f"not saturated at T_max ({at_top:.2%} at max code {top}); light too weak for full range"
    white = top
    span = white - offset
    t = cam.expose(t_min)
    for _ in range(10):
        stack, _, _ = cam.grab(4, t)
        sig = g_mean(stack) - offset
        sat = float(np.mean(stack >= white))
        print(f"    search T={t:10.1f} us  G-off={sig:12.1f} ({sig / span:6.1%})  sat={sat:.2%}", flush=True)
        if sat < 0.001 and abs(sig / span - a.target) < 0.05:
            return t, white, "ok"
        if t <= t_min and (sat > 0.01 or sig / span > a.target):
            return t, white, "saturated near T_min; reduce light"
        ratio = a.target * span / max(sig, 1.0) if sat < 0.01 else 0.5
        t = cam.expose(float(np.clip(t * min(ratio, 50.0), t_min, t_max)))
    return t, white, "search did not converge; using last"


def save_level(folder, tag, stack, metas):
    dtype = np.uint16 if stack.max() < 65536 else np.uint32
    np.savez_compressed(folder / f"{tag}.npz", stack=stack.astype(dtype))
    return {"file": f"{tag}.npz", "frame_ids": [m["frame_id"] for m in metas],
            "timestamps_ns": [m["timestamp_ns"] for m in metas],
            "crc_ok": sum(bool(m.get("crc_ok")) for m in metas), "n": len(metas),
            "chunk_exposure_us": metas[0].get("ChunkExposureTime"), "chunk_gain_db": metas[0].get("ChunkGain"),
            "chunk_black_level": metas[0].get("ChunkBlackLevel")}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("phase", choices=["bright", "dark"])
    ap.add_argument("--session", type=Path, required=True)
    ap.add_argument("--serial")
    ap.add_argument("--outputs", nargs="+", default=list(OUTPUTS))
    ap.add_argument("--levels", type=int, default=16)
    ap.add_argument("--frames", type=int, default=100, help="per level, even (non-overlapping pairs)")
    ap.add_argument("--drift-frames", type=int, default=20)
    ap.add_argument("--roi", type=int, default=256)
    ap.add_argument("--black-level-raw", type=int, default=256, help="pedestal; 0 clips dark noise (dark_probe)")
    ap.add_argument("--target", type=float, default=0.8, help="T_top: G signal / (white - offset)")
    ap.add_argument("--t-top-us", type=float, help="skip the search and use this T_top for every output")
    ap.add_argument("--long", action="store_true",
                    help="exposures beyond 40 ms: frame rate follows each exposure (dark reuses the bright setting)")
    ap.add_argument("--t-max-us", type=float, default=2.5e6, help="longest exposure with --long")
    ap.add_argument("--gain-db", type=float, default=0.0, help="Gain node; outputs are stored as <output>_gNN")
    a = ap.parse_args()

    a.session.mkdir(parents=True, exist_ok=True)
    manifest_path = a.session / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {"outputs": {}}
    if a.phase == "dark" and not manifest["outputs"]:
        raise SystemExit("run the bright phase first")
    if a.phase == "dark":
        a.long = manifest.get("long_exposure", False)

    system, device, info = open_lucid(a.serial)
    st = Settings(device.nodemap)
    cam = Cam(device, st, a.long)
    manifest.update({"serial": info["serial"], "model": info["model"], "roi": a.roi, "long_exposure": a.long,
                     "black_level_raw": a.black_level_raw, "pixel_format": "BayerRG24"})
    try:
        setup_stream(device)
        for output in a.outputs:
            name = output if not a.gain_db else f"{output}_g{a.gain_db:02.0f}"
            if a.phase == "dark" and name not in manifest["outputs"]:
                print(f"[{name}] no bright data, skipped")
                continue
            folder = a.session / name
            folder.mkdir(exist_ok=True)
            cam.stop()
            configure(st, output, a)
            t_min, t_max = cam.t_range(a.t_max_us)
            rec = manifest["outputs"].setdefault(name, {})
            rec["gain_db"] = st.read("Gain")
            rec[f"{a.phase}_temperature_start_c"] = st.read("DeviceTemperature")
            rec[f"{a.phase}_time"] = time.strftime("%Y-%m-%d %H:%M:%S")
            print(f"[{name}] {a.phase}  T range {t_min:.1f}-{t_max:.1f} us  temp {st.read('DeviceTemperature')} C", flush=True)
            if a.phase == "bright":
                if a.t_top_us:
                    t_top, white, note = cam.expose(a.t_top_us), None, "manual T_top"
                else:
                    t_top, white, note = find_top(cam, a, t_min, t_max, a.black_level_raw)
                print(f"  T_top={t_top:.1f} us  white={white}  {note}", flush=True)
                rec.update({"t_top_us": t_top, "white_level": white, "note": note, "t_min_us": t_min, "t_max_us": t_max})
                targets = list(np.geomspace(t_min, t_top, a.levels)) if t_top > t_min else [t_min]
            else:
                targets = [lv["exposure_us"] for lv in rec["bright"]]
            levels = []
            for k, t in enumerate(targets):
                actual = cam.expose(t)
                if a.phase == "bright" and levels and abs(actual - levels[-1]["exposure_us"]) < 1.0:
                    continue  # ExposureTime is quantized (~20 us); a repeat adds no new point
                stack, metas, skipped = cam.grab(a.frames, actual)
                lv = {"exposure_us": actual, "skipped": skipped, "temperature_c": st.read("DeviceTemperature"),
                      "frame_rate_hz": st.read("AcquisitionFrameRate"),
                      **save_level(folder, f"{a.phase}_{k:02d}", stack, metas)}
                levels.append(lv)
                print(f"  {a.phase} {k:2d}  T={actual:10.1f} us  G mean={g_mean(stack):12.1f}  max={int(stack.max()):9d}  "
                      f"crc {lv['crc_ok']}/{lv['n']}  skip {skipped}  {lv['frame_rate_hz']:.2f} Hz", flush=True)
            if a.phase == "bright" and a.drift_frames:  # repeat the top level: best relative precision
                actual = cam.expose(targets[-1])
                stack, metas, _ = cam.grab(a.drift_frames, actual)
                rec["drift"] = {"exposure_us": actual, **save_level(folder, "bright_drift", stack, metas)}
            rec[a.phase] = levels
            cam.stop()
            rec[f"{a.phase}_temperature_end_c"] = st.read("DeviceTemperature")
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=1, default=str))
    finally:
        try:
            cam.stop()
        except Exception:
            pass
        manifest.setdefault("settings_logs", {})[a.phase] = st.log
        manifest.setdefault("restore_errors", {})[a.phase] = st.restore()
        system.destroy_device()
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=1, default=str))
        print("restore errors:", manifest["restore_errors"][a.phase] or "none")
        print("session:", a.session.resolve())


if __name__ == "__main__":
    main()
