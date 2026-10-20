"""랩 RAW 취득 모드 (~/jsh) — DM_clipGUI 복제본에서 차량 전용 기능 대신 쓰는 것들.

clip_gui.RAW_LAB 이 True 면:
  - 녹화 폴더: <저장 위치>/scene_NNN_<장면>/rec_… · clip_…  (원본의 route_NNN_<지역> 대신)
  - 녹화가 끝나면 bag 폴더에 capture_info.json · nodemap/ · params/ 를 남긴다 (write_capture_info)
  - 온라인 캘리브레이션 · 지도 · 내비 · GNSS · 취득 현황표는 끈다 (NoCalib)

capture_info.json 은 RAW → RGB 재구성 연구에서 그 녹화를 다시 쓸 때 필요한 것을 한곳에 모은다:
장면 · 조도 · 광원(사람이 적는 것), 녹화에 들어간 카메라(시리얼 ↔ 토픽 네임스페이스), 녹화 시작 시점의
카메라 상태(노드맵 덤프 — 프레임별 값은 bag 의 image_raw/metadata 에 있다), 기동에 쓴 파라미터 사본,
소프트웨어 버전(git describe).
"""
import json
import re
import shutil
import subprocess
import time
from datetime import datetime
from pathlib import Path

import yaml

# 녹화 중에 켜 둔 조명 조건 — capture_info.json "light_tags" (원본의 도로 형상 칩 자리)
LIGHT_TAGS = ["암실", "실내", "실외", "야간", "LED", "형광등", "백열/할로겐", "자연광", "가로등", "혼합광"]

ROUTE_PREFIX = "scene"
_DUMP_RE = re.compile(r"^(?P<serial>.+)_(?P<stamp>\d{8}_\d{6})_nodemap\.json$")


class NoCalib:
    """온라인 캘리브레이션 탭 자리 — 랩 리그에는 없다 (녹화 · 종료 경로가 부르는 것만)."""

    def running_job(self):
        return None

    def cancel_job(self, *_args, **_kwargs):
        return None

    def shutdown(self):
        return None

    def history_dialog(self):
        return None

    def current_title(self):
        return ""


def _bag_window(bag):
    """(시작 ns, 끝 ns, {토픽: 메시지 수}) — rosbag2 metadata.yaml 에서. 없으면 (None, None, {})."""
    try:
        meta = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
        info = meta["rosbag2_bagfile_information"]
        start = int(info["starting_time"]["nanoseconds_since_epoch"])
        dur = int(info["duration"]["nanoseconds"])
        counts = {t["topic_metadata"]["name"]: int(t.get("message_count", 0))
                  for t in info.get("topics_with_message_count", [])}
        return start, start + dur, counts
    except Exception:                                              # noqa: BLE001
        return None, None, {}


def _dump_time_ns(stamp):
    return int(datetime.strptime(stamp, "%Y%m%d_%H%M%S").timestamp() * 1e9)


def _launched_cameras(generated_dir):
    """GUI 가 이번 기동에 넘긴 인벤토리 사본들 → [{group, serial, namespace, name}]."""
    out = []
    for inv in sorted(Path(generated_dir).glob("*__inventory.yaml")):
        try:
            data = yaml.safe_load(inv.read_text(encoding="utf-8")) or {}
        except Exception:                                          # noqa: BLE001
            continue
        for value in data.values():
            cams = ((value or {}).get("ros__parameters") or {}).get("cameras") or []
            for cam in cams:
                out.append({"inventory": inv.name, "serial": str(cam.get("serial", "")),
                            "namespace": str(cam.get("namespace", cam.get("name", ""))),
                            "name": str(cam.get("name", ""))})
    return out


def nodemap_dirs(registry):
    """sensors.yaml subset 의 forced_params.nodemap_dump_dir 들."""
    dirs = []
    for group in (registry or {}).get("groups", []):
        for subset in group.get("subsets") or [group]:
            d = ((subset.get("forced_params") or {}).get("nodemap_dump_dir")
                 or (group.get("forced_params") or {}).get("nodemap_dump_dir"))
            if d:
                p = Path(str(d)).expanduser()
                if p not in dirs:
                    dirs.append(p)
    return dirs


def _git(repo):
    try:
        describe = subprocess.run(["git", "-C", str(repo), "describe", "--always", "--dirty", "--tags"],
                                  capture_output=True, text=True, timeout=5).stdout.strip()
        branch = subprocess.run(["git", "-C", str(repo), "rev-parse", "--abbrev-ref", "HEAD"],
                                capture_output=True, text=True, timeout=5).stdout.strip()
        return f"{branch}@{describe}" if describe else None
    except Exception:                                              # noqa: BLE001
        return None


def write_capture_info(bag, *, kind, scene, label, lux, light, tags, registry, generated_dir, repos):
    """녹화(clip_ · rec_ 폴더)가 닫힌 뒤 부른다. 쓴 파일 수를 담은 한 줄 요약을 돌려준다."""
    bag = Path(bag)
    start, end, counts = _bag_window(bag)
    cameras = _launched_cameras(generated_dir)
    serials = {c["serial"] for c in cameras if c["serial"]}

    # 노드맵 덤프: 녹화 시작 시점의 상태(시작 전 마지막 것) + 녹화 중에 설정을 바꿔 새로 생긴 것
    copied = []
    nm_out = bag / "nodemap"
    for d in nodemap_dirs(registry):
        if not d.is_dir():
            continue
        by_serial = {}
        for f in d.iterdir():
            m = _DUMP_RE.match(f.name)
            if m and m["serial"] in serials:
                by_serial.setdefault(m["serial"], []).append((_dump_time_ns(m["stamp"]), f))
        for serial, items in by_serial.items():
            items.sort()
            pick = []
            if start is not None:
                before = [it for it in items if it[0] <= start]
                if before:
                    pick.append(before[-1])
                pick += [it for it in items if start < it[0] <= (end or start) + 5_000_000_000]
            elif items:
                pick.append(items[-1])
            for _, f in pick:
                nm_out.mkdir(exist_ok=True)
                shutil.copy2(f, nm_out / f.name)
                copied.append(f.name)

    # 기동에 쓴 파라미터 · 인벤토리 사본 (GUI 가 만든 그대로)
    params_out = bag / "params"
    n_params = 0
    for f in sorted(Path(generated_dir).glob("*.yaml")):
        params_out.mkdir(exist_ok=True)
        shutil.copy2(f, params_out / f.name)
        n_params += 1

    def num(text):
        try:
            return float(str(text).replace(",", "."))
        except ValueError:
            return None

    info = {
        "format": "raw_lab capture_info v1",
        "written": datetime.now().astimezone().isoformat(timespec="seconds"),
        "bag": bag.name,
        "kind": kind,
        "scene": scene,
        "scene_folder": bag.parent.name,
        "label": label,
        "illuminance_lux": num(lux) if str(lux).strip() else None,
        "illuminance_text": str(lux).strip(),
        "light_source": light,
        "light_tags": list(tags),
        "start_ns": start,
        "end_ns": end,
        "message_counts": counts,
        "cameras": cameras,
        "nodemap_files": sorted(copied),
        "software": {Path(r).name: _git(r) for r in repos},
        "notes": ("프레임별 노출 · 게인 · 블랙 · 온도는 각 카메라의 image_raw/metadata, 녹화 시작 시점의 카메라 "
                  "상태 전체는 nodemap/, 기동 파라미터는 params/ 에 있다."),
    }
    (bag / "capture_info.json").write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")
    return (f"capture_info.json · 노드맵 {len(copied)}개 · 파라미터 {n_params}개 → {bag.name}"
            + ("" if copied else " (노드맵 덤프를 못 찾음 — nodemap_dump_dir 확인)"))


def next_scene_number(base):
    """저장 위치 안의 scene_NNN_* 다음 번호."""
    base = Path(base)
    nums = []
    if base.is_dir():
        for p in base.iterdir():
            m = re.match(rf"{ROUTE_PREFIX}_(\d+)_", p.name)
            if p.is_dir() and m:
                nums.append(int(m.group(1)))
    return max(nums, default=0) + 1


def stamp():
    return time.strftime("%Y%m%d_%H%M%S")
