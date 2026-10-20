"""트리거 모드 dark 확인 — 렌즈캡 상태로, 액션 커맨드 트리거로 노출별 dark 를 찍어 DNG 로 저장한다.

jsh-16(PTC 세션)과 합의한 시험 (2026-10-20): free-run 으로 잰 노이즈 모델이 트리거 모드에도 맞는지.
LUCID 는 트리거 모드에서 노출 시작이 ~1 s 늦어 프레임 주기가 길어지는데, 1 s 넘는 노출에서 dark 동작이
바뀐 적이 있어서 확인한다.

    source ~/jsh/env.sh && ~/jsh/.venv-dng/bin/python ~/jsh/tools/trigger_dark_check.py [--frames 50] [--bfs-gain 0]
저장: /mnt/data/jsh_raw/trigger_dark_check/<시각>/<노출 us>us/<KK>_<cam>.dng (+ .json)
"""
import argparse
import json
import sys
import time
from pathlib import Path

import bracket as br

ROOT = Path("/mnt/data/jsh_raw/trigger_dark_check")


def free_run_frames(link, keys, exposure, first):
    """free-run: 노출을 바꾼 뒤 settled(드라이버가 설정 직후 두 장은 false) 이고 chunk 노출이 맞는 다음 프레임."""
    if first:
        link.clear(keys)
    got = {}
    for k in keys:
        f = link.take(k, 0.0, exposure[k] / 1e6 * 3 + 10.0,
                      lambda f, k=k: br.exposure_ok(f.meta, exposure[k]) and f.meta.get("settled", True))
        if f is None:
            raise RuntimeError(f"{k}: free-run 프레임이 안 온다")
        got[k] = f
    return got, {"fire_time_ns": {k: f.stamp_ns for k, f in got.items()}, "mode": "free-run",
                 "camera_frame_id": {k: f.meta.get("camera_frame_id") for k, f in got.items()}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=50)
    ap.add_argument("--exposures-ms", default="1 30 300 1000")
    ap.add_argument("--bfs-gain", type=float, default=0.0)
    ap.add_argument("--cams", default="bfs,lucid")
    ap.add_argument("--free-run", action="store_true",
                    help="트리거 없이 free-run 으로 (대조군). 노출을 바꾼 뒤 settled 프레임부터 연속 N장")
    args = ap.parse_args()
    out = ROOT / (time.strftime("%Y%m%d_%H%M%S") + f"_bfs{int(args.bfs_gain)}dB_" + args.cams.replace(",", "+")
                  + ("_freerun" if args.free_run else ""))
    out.mkdir(parents=True)
    link = br.RosLink()
    try:
        t0 = time.time()
        keys = args.cams.split(",")
        while not all(link.alive(k) for k in keys) and time.time() - t0 < 30:
            time.sleep(0.2)
        rig = br.Rig(link, keys=keys)
        link.set_params("bfs", {"camera.Gain": float(args.bfs_gain)})
        if not args.free_run:
            rig.arm(True)
        log = {"frames": args.frames, "cameras": keys, "bfs_gain_db": args.bfs_gain, "lucid_gain_db": 0.0,
               "mode": "free-run" if args.free_run else "trigger (action command)", "steps": []}
        try:
            for ms in (float(v) for v in args.exposures_ms.split()):
                exposure = {k: ms * 1000.0 for k in keys}
                rig.set_exposure(exposure)
                folder = out / f"{int(round(ms * 1000))}us"
                folder.mkdir()
                step = {"exposure_us": exposure, "offsets_ms": [], "temperature_c": {}}
                t_step = time.time()
                for j in range(args.frames):
                    if args.free_run:
                        frames, info = free_run_frames(link, keys, exposure, first=(j == 0))
                    else:
                        frames, info = rig.shoot(exposure)
                    step["offsets_ms"].append(info.get("start_offset_ms"))
                    for k, f in frames.items():
                        rig.save(f, folder / f"{j:02d}_{k}.dng", {"test": "trigger_dark_check", "shot": info,
                                                                  "fire_time_ns": min(info["fire_time_ns"].values())})
                        step["temperature_c"].setdefault(k, []).append(f.meta.get("device_temperature_c"))
                step["seconds"] = time.time() - t_step
                log["steps"].append(step)
                print(f"{ms:g} ms: {args.frames} pairs in {step['seconds']:.0f}s", flush=True)
        finally:
            if not args.free_run:
                rig.arm(False)
            link.set_params("bfs", {"camera.Gain": 0.0})
        (out / "log.json").write_text(json.dumps(log, indent=1, default=str))
        print(out)
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
