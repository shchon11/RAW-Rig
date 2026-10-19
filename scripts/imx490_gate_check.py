"""IMX490 (LUCID TDR054S) gate check: is BayerRG24 linear RAW?

Standalone; needs only numpy and LUCID's arena_api (shipped with the Arena SDK).
Points the camera at a fixed, evenly lit scene, sweeps exposure, and checks that the
per-CFA mean grows in proportion to exposure. A tone curve (AltaView) shows up as a
log-log slope below 1 or as frame-to-frame drift at a fixed exposure.
Every setting it touches is restored on exit.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

FULL24 = (1 << 24) - 1
DUMP_PATTERNS = ("HDR", "Tone", "Balance", "Gamma", "LUT", "BlackLevel", "Gain", "Exposure",
                 "PixelFormat", "AcquisitionFrameRate", "Device")
CFA = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}


def node(nodemap, name):
    try:
        return nodemap.get_node(name)
    except Exception:
        return None


def read(nodemap, name):
    n = node(nodemap, name)
    if n is None:
        return None
    try:
        return n.value if n.is_readable else None
    except Exception:
        return None


def write(nodemap, name, value, saved):
    n = node(nodemap, name)
    if n is None or not n.is_writable:
        return False
    if name not in saved:
        saved[name] = n.value
    n.value = value
    return True


def entries(nodemap, name):
    n = node(nodemap, name)
    try:
        return list(n.enumentry_names) if n is not None else None
    except Exception:
        return None


def dump(nodemap):
    names = getattr(nodemap, "feature_names", None) or []
    out = {}
    for name in names:
        if any(p.lower() in name.lower() for p in DUMP_PATTERNS):
            value = read(nodemap, name)
            if value is not None:
                out[name] = value if isinstance(value, (int, float, bool)) else str(value)
    return out


def grab(device, count, timeout_ms):
    frames = []
    for _ in range(count):
        buf = device.get_buffer(timeout=timeout_ms)
        try:
            h, w = buf.height, buf.width
            bpp = buf.bits_per_pixel // 8
            raw = np.ctypeslib.as_array(buf.pdata, (h * w * bpp,)).copy()
        finally:
            device.requeue_buffer(buf)
        frames.append((raw, h, w, bpp))
    return frames


def decode24(raw, h, w, little=True):
    b = raw.reshape(h, w, 3).astype(np.uint32)
    if little:
        return b[..., 0] | (b[..., 1] << 8) | (b[..., 2] << 16)
    return (b[..., 0] << 16) | (b[..., 1] << 8) | b[..., 2]


def cfa_means(img, roi):
    y0, x0, s = roi
    crop = img[y0:y0 + s, x0:x0 + s].astype(np.float64)
    return {k: float(crop[i::2, j::2].mean()) for k, (i, j) in CFA.items()}


def capture_means(device, count, timeout_ms, roi):
    le, be, frame_g = [], [], []
    last = None
    for raw, h, w, bpp in grab(device, count, timeout_ms):
        if bpp != 3:
            raise RuntimeError(f"BayerRG24가 아님: {bpp * 8} bit/pixel")
        img = decode24(raw, h, w, True)
        le.append(cfa_means(img, roi))
        be.append(cfa_means(decode24(raw, h, w, False), roi))
        frame_g.append(le[-1]["G1"])
        last = img
    avg = lambda rows: {k: float(np.mean([r[k] for r in rows])) for k in CFA}
    return avg(le), avg(be), frame_g, last


def linearity(exposures, means):
    t = np.asarray(exposures, float)
    m = np.asarray(means, float)
    ok = m < 0.9 * FULL24
    if ok.sum() < 4:
        return {"usable_levels": int(ok.sum())}
    a, b = np.polyfit(t[ok], m[ok], 1, w=1.0 / np.maximum(m[ok], 1.0))  # relative-residual fit across decades
    resid = (m[ok] - (a * t[ok] + b)) / np.maximum(m[ok], 1.0)
    sig = m[ok] - b
    good = sig > 0
    slope = float(np.polyfit(np.log(t[ok][good]), np.log(sig[good]), 1)[0]) if good.sum() >= 4 else float("nan")
    return {"usable_levels": int(ok.sum()), "dn_per_us": float(a), "offset_dn": float(b),
            "max_rel_residual": float(np.abs(resid).max()), "loglog_slope": slope}


def find_top_exposure(nodemap, device, args, roi, saved):
    t = args.start_us
    for _ in range(20):
        write(nodemap, "ExposureTime", float(t), saved)
        time.sleep(0.05)
        m, _, _, _ = capture_means(device, 2, args.timeout_ms, roi)
        g = m["G1"]
        print(f"  T={t:10.1f} us  G1 mean={g:12.0f} ({g / FULL24:6.1%})", flush=True)
        if 0.3 * FULL24 <= g <= 0.7 * FULL24:
            return t
        t = t / 4 if g > 0.7 * FULL24 else t * min(4.0, 0.5 * FULL24 / max(g, 1.0))
        t = float(np.clip(t, args.t_min, args.t_max))
    return t


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--serial", help="여러 대 연결 시 대상 serial")
    p.add_argument("--out", type=Path, default=Path("imx490_gate_check"))
    p.add_argument("--levels", type=int, default=12)
    p.add_argument("--decades", type=float, default=3.0, help="최상단 노출 아래로 몇 decade 내려갈지")
    p.add_argument("--frames", type=int, default=8)
    p.add_argument("--start-us", type=float, default=1000.0)
    p.add_argument("--t-min", type=float, default=20.0)
    p.add_argument("--t-max", type=float, default=500000.0)
    p.add_argument("--roi", type=int, default=512, help="중앙 정사각 ROI 한 변 (짝수)")
    p.add_argument("--hdr-output", help="HDROutput 엔트리를 바꿔서 시험할 때")
    p.add_argument("--keep-enhancement", action="store_true", help="HDRImageEnhancement/Clamping을 끄지 않음")
    p.add_argument("--timeout-ms", type=int, default=5000)
    args = p.parse_args()

    from arena_api.system import system

    infos = system.device_infos
    if args.serial:
        infos = [i for i in infos if str(i.get("serial")) == args.serial]
    if not infos:
        raise SystemExit("카메라를 찾지 못함")
    device = system.create_device(device_infos=[infos[0]])[0]
    nm, stream = device.nodemap, device.tl_stream_nodemap
    for name, value in (("StreamAutoNegotiatePacketSize", True), ("StreamPacketResendEnable", True),
                        ("StreamBufferHandlingMode", "OldestFirst")):
        write(stream, name, value, {})

    args.out.mkdir(parents=True, exist_ok=True)
    report = {"time": time.strftime("%Y-%m-%d %H:%M:%S"),
              "device": {k: read(nm, k) for k in ("DeviceModelName", "DeviceSerialNumber", "DeviceFirmwareVersion")},
              "pixel_formats": entries(nm, "PixelFormat"), "hdr_output_entries": entries(nm, "HDROutput"),
              "nodes_before": dump(nm)}
    print(json.dumps(report["device"], ensure_ascii=False))
    print("PixelFormat:", report["pixel_formats"])
    print("HDROutput:", report["hdr_output_entries"], "current:", read(nm, "HDROutput"))

    saved = {}
    try:
        if not write(nm, "PixelFormat", "BayerRG24", saved):
            raise SystemExit("PixelFormat=BayerRG24 설정 불가")
        for name, value in (("ExposureAuto", "Off"), ("GainAuto", "Off"), ("BalanceWhiteAuto", "Off"),
                            ("AcquisitionFrameRateEnable", False), ("GammaEnable", False), ("LUTEnable", False)):
            write(nm, name, value, saved)
        if not args.keep_enhancement:
            for name in ("HDRImageEnhancementEnable", "HDRDigitalClampingEnable"):
                write(nm, name, False, saved)
        if args.hdr_output:
            write(nm, "HDROutput", args.hdr_output, saved)
        gain = node(nm, "Gain")
        if gain is not None and gain.is_writable:
            write(nm, "Gain", float(gain.min), saved)
        report["nodes_during"] = dump(nm)

        h, w = int(read(nm, "Height")), int(read(nm, "Width"))
        s = args.roi - args.roi % 2
        roi = ((h - s) // 2 & ~1, (w - s) // 2 & ~1, s)

        with device.start_stream(4):
            print("최상단 노출 탐색 (G1 평균 30~70%)", flush=True)
            top = find_top_exposure(nm, device, args, roi, saved)
            exposures = np.geomspace(max(top / 10 ** args.decades, args.t_min), top, args.levels)
            rows = []
            for t in exposures:
                write(nm, "ExposureTime", float(t), saved)
                time.sleep(0.05)
                le, be, frame_g, img = capture_means(device, args.frames, args.timeout_ms, roi)
                actual = float(read(nm, "ExposureTime") or t)
                cv = float(np.std(frame_g) / max(np.mean(frame_g), 1.0))
                rows.append({"exposure_us": actual, "mean_le": le, "mean_be": be, "frame_cv_g1": cv})
                np.save(args.out / f"frame_{actual:.0f}us.npy", img)
                print(f"  T={actual:10.1f} us  G1={le['G1']:12.0f}  frame CV={cv:.2%}", flush=True)

            write(nm, "ExposureTime", float(exposures[len(exposures) // 2]), saved)
            time.sleep(0.05)
            _, _, drift_g, _ = capture_means(device, 30, args.timeout_ms, roi)

        report["levels"] = rows
        report["fit"] = {}
        for endian in ("le", "be"):
            report["fit"][endian] = {k: linearity([r["exposure_us"] for r in rows], [r[f"mean_{endian}"][k] for r in rows])
                                     for k in CFA}
        report["fixed_exposure_cv_g1_30frames"] = float(np.std(drift_g) / max(np.mean(drift_g), 1.0))
    finally:
        try:
            device.stop_stream()
        except Exception:
            pass
        for name in reversed(list(saved)):
            try:
                node(nm, name).value = saved[name]
            except Exception as error:
                print(f"복원 실패 {name}: {error}")
        system.destroy_device()

    fit = report["fit"]["le"]
    slope = [fit[k].get("loglog_slope", float("nan")) for k in CFA]
    resid = [fit[k].get("max_rel_residual", float("nan")) for k in CFA]
    linear = all(abs(x - 1) <= 0.02 for x in slope) and all(r <= 0.02 for r in resid)
    stable = report["fixed_exposure_cv_g1_30frames"] <= 0.005
    report["verdict"] = {"linear": linear, "stable": stable, "loglog_slope": dict(zip(CFA, slope)),
                         "max_rel_residual": dict(zip(CFA, resid))}
    (args.out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str))

    print("\n채널   log-log 기울기   최대 상대잔차")
    for k, sl, r in zip(CFA, slope, resid):
        print(f"{k:4s}   {sl:10.4f}     {r:10.2%}")
    print(f"고정 노출 30장 G1 변동: {report['fixed_exposure_cv_g1_30frames']:.2%}")
    print("판정:", "선형 RAW로 보임" if linear and stable else
          "선형 아님 또는 불안정 — report.json의 be 해석·노드값·조명 flicker 확인")
    print("저장:", args.out.resolve())


if __name__ == "__main__":
    main()
