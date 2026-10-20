"""두 카메라 동시 노출 브라케팅 — 장면 하나를 노출 단계별로 쌍 촬영해서 무손실 DNG + json 으로 저장한다.

    source ~/jsh/env.sh
    ~/jsh/bracket                                               # GUI (tools/bracket_gui.py)
    ~/jsh/.venv-dng/bin/python ~/jsh/tools/bracket.py --scene 시험 --ev "0 -2 -4" --frames 2 --meter

전제: 두 드라이버(FLIR_control · LUCID_control)가 떠 있다 — GUI 의 [카메라 기동], 또는 rawgui [센서 기동].
트리거: 촬영하는 동안만 두 드라이버를 trigger.mode=action 으로 바꾸고, 쌍마다 브로드캐스트 액션 커맨드를
       한 번 쏜다 (gvcp_action.fire). 두 카메라가 같은 순간 노출을 시작한다. 끝나면 free run 으로 되돌린다.
노출:  기준 노출(EV 0)에 2^EV 를 곱한다. 기준은 측광(카메라별로 밝은 쪽 p99.5 가 포화의 target 이 되게 —
       ETTR) 또는 직접 입력. "같은 노출시간" 규칙이면 두 카메라 모두 둘 중 짧은 쪽을 쓴다.
확인:  쌍마다 두 카메라의 ACK, chunk 노출값(요청값과 같은지), CRC 를 보고, 안 맞으면 한 번 다시 찍는다.
       DNG 는 쓰고 나서 다시 읽어 픽셀을 비교한다 (rawdng.write).
저장:  /mnt/data/jsh_raw/stills/scene_NNN_<이름>/
         scene.json                       장면 · 계획 · 카메라 · 촬영 목록 (찍는 동안 계속 갱신)
         bright/NN_ev±X.X_KK_<cam>.dng    (+ 같은 이름 .json)
         dark/                            렌즈캡 — 같은 노출 단계
         nodemap/                         촬영 시작 때의 카메라 노드맵
"""
import argparse
import datetime as dt
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Image

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import gvcp_action   # noqa: E402
import ptc_model     # noqa: E402
import rawdng        # noqa: E402
from rawbag import to_array   # noqa: E402

STILLS = Path("/mnt/data/jsh_raw/stills")
NODEMAP_DIR = Path("/mnt/data/jsh_raw/nodemap")
IFACE = "enp5s0"
GUI_SCRIPTS = Path.home() / "jsh/DM_clipGUI/scripts"


class Cam:
    def __init__(self, key, ns, node, meta_type, label, make):
        self.key, self.ns, self.node, self.meta_type, self.label, self.make = key, ns, node, meta_type, label, make


CAMS = [Cam("bfs", "camera_bfs", "/camera_bfs/flir_camera", "flir_spinnaker_camera/msg/FlirMetadata", "FLIR BFS",
            "FLIR"),
        Cam("lucid", "lucid_tri054", "/lucid_tri054/lucid_camera", "lucid_raw_camera/msg/LucidMetadata",
            "LUCID TRI054S", "LUCID")]
CAM = {c.key: c for c in CAMS}
# 노출 범위 = 노이즈 모델의 유효 범위 (ptc_model.EXPOSURE_US: BFS 24 µs~1 s, LUCID 86 µs~1.0 s)
DEFAULT_LIMITS_US = dict(ptc_model.EXPOSURE_US)


class Stopped(Exception):
    pass


def _plain(value):
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)) or type(value).__name__ == "array":
        return [_plain(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, float)):
        return None if value != value else float(value)    # NaN → null
    if isinstance(value, np.integer):
        return int(value)
    return value


class Frame:
    """image_raw + image_raw/metadata 한 쌍 (같은 header.stamp). numpy 변환은 필요할 때 한 번."""

    def __init__(self, key, raw_msg, meta_msg, stamp_ns):
        self.key, self.raw_msg, self.meta_msg, self.stamp_ns = key, raw_msg, meta_msg, stamp_ns
        self.arrived = time.monotonic()
        self._raw = self._meta = None

    @property
    def raw(self):
        if self._raw is None:
            self._raw = to_array(self.raw_msg)
        return self._raw

    @property
    def meta(self):
        if self._meta is None:
            self._meta = _plain(dict(message_to_ordereddict(self.meta_msg)))
            self._meta["encoding"] = self.raw_msg.encoding
        return self._meta


class RosLink:
    """드라이버 두 개와 ROS 로 연결: 프레임 · 미리보기 받기, 파라미터 읽고 쓰기."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("raw_bracket")
        self.cond = threading.Condition()
        self.frames = {c.key: deque(maxlen=6) for c in CAMS}
        self.pending = {c.key: ({}, {}) for c in CAMS}
        self.preview = {c.key: None for c in CAMS}          # (jpeg bytes, monotonic)
        self.last_meta = {c.key: None for c in CAMS}        # 마지막 metadata 메시지
        self.last_seen = {c.key: 0.0 for c in CAMS}
        self.meta_times = {c.key: deque(maxlen=20) for c in CAMS}
        self.set_cli, self.get_cli, self.list_cli = {}, {}, {}
        qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        for c in CAMS:
            group = MutuallyExclusiveCallbackGroup()
            self.node.create_subscription(Image, f"/{c.ns}/image_raw", lambda m, k=c.key: self._on(k, 0, m), qos,
                                          callback_group=group)
            self.node.create_subscription(get_message(c.meta_type), f"/{c.ns}/image_raw/metadata",
                                          lambda m, k=c.key: self._on(k, 1, m), qos, callback_group=group)
            self.node.create_subscription(CompressedImage, f"/{c.ns}/image_rgb/compressed",
                                          lambda m, k=c.key: self._on_preview(k, m), qos_profile_sensor_data,
                                          callback_group=MutuallyExclusiveCallbackGroup())
            services = MutuallyExclusiveCallbackGroup()
            self.set_cli[c.key] = self.node.create_client(SetParameters, f"{c.node}/set_parameters",
                                                          callback_group=services)
            self.get_cli[c.key] = self.node.create_client(GetParameters, f"{c.node}/get_parameters",
                                                          callback_group=services)
            self.list_cli[c.key] = self.node.create_client(ListParameters, f"{c.node}/list_parameters",
                                                           callback_group=services)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown()

    # ---- 받기 ----
    def _on(self, key, kind, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self.cond:
            if kind == 1:
                self.last_meta[key] = msg
                self.meta_times[key].append(time.monotonic())
            self.last_seen[key] = time.monotonic()
            raws, metas = self.pending[key]
            (raws if kind == 0 else metas)[stamp] = msg
            if stamp in raws and stamp in metas:
                frame = Frame(key, raws.pop(stamp), metas.pop(stamp), stamp)
                for d in (raws, metas):
                    for s in [s for s in d if s < stamp]:
                        del d[s]
                self.frames[key].append(frame)
                self.cond.notify_all()

    def _on_preview(self, key, msg):
        with self.cond:
            self.preview[key] = (bytes(msg.data), time.monotonic())
            self.last_seen[key] = time.monotonic()

    def alive(self, key, within=5.0):
        return time.monotonic() - self.last_seen[key] < within

    def fps(self, key, window=5.0):
        with self.cond:
            times = [t for t in self.meta_times[key] if time.monotonic() - t < window]
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.0

    def latest(self, key):
        """가장 최근에 들어온 프레임 (꺼내지 않는다) — 화면 표시용."""
        with self.cond:
            return self.frames[key][-1] if self.frames[key] else None

    def drivers_present(self):
        names = {f"{ns.rstrip('/')}/{n}" for n, ns in self.node.get_node_names_and_namespaces()}
        return {c.key: c.node in names for c in CAMS}

    def clear(self, keys):
        with self.cond:
            for k in keys:
                self.frames[k].clear()

    def take(self, key, since, timeout, accept):
        """since(monotonic) 뒤에 들어온 프레임 중 accept 를 통과한 첫 프레임. 없으면 None."""
        deadline = time.monotonic() + timeout
        with self.cond:
            while True:
                for f in list(self.frames[key]):
                    if f.arrived >= since and accept(f):
                        self.frames[key].remove(f)
                        return f
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self.cond.wait(left)

    # ---- 파라미터 ----
    def _call(self, cli, request, timeout):
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{cli.srv_name}: 드라이버가 응답하지 않는다")
        future = cli.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise TimeoutError(f"{cli.srv_name}: {timeout:.0f}초 안에 답이 없다")
        return future.result()

    def set_params(self, key, params, timeout=20.0):
        request = SetParameters.Request(parameters=[Parameter(n, value=v).to_parameter_msg()
                                                    for n, v in params.items()])
        result = self._call(self.set_cli[key], request, timeout)
        bad = [f"{n}: {r.reason}" for n, r in zip(params, result.results) if not r.successful]
        if bad:
            raise RuntimeError(f"{CAM[key].label}: " + "; ".join(bad))

    def get_params(self, key, names, timeout=10.0):
        result = self._call(self.get_cli[key], GetParameters.Request(names=list(names)), timeout)
        return {n: parameter_value_to_python(v) for n, v in zip(names, result.values)}

    def all_params(self, key):
        result = self._call(self.list_cli[key], ListParameters.Request(depth=0), 10.0)
        names = list(result.result.names)
        values = {}
        for i in range(0, len(names), 200):
            values.update(self.get_params(key, names[i:i + 200]))
        return {k: _plain(v) for k, v in sorted(values.items())}


# ---------- 프레임 해석 ----------

def cfa_of(frame):
    meta = frame.meta
    text = (meta.get("cfa_pattern") or meta.get("encoding") or "").upper()
    for pattern in rawdng.CFA_CODES:
        if pattern in text:
            return pattern
    for prefix, pattern in (("BAYERRG", "RGGB"), ("BAYERBG", "BGGR"), ("BAYERGR", "GRBG"), ("BAYERGB", "GBRG")):
        if prefix in text.replace("_", ""):
            return pattern
    return "RGGB"


def frame_gain(frame):
    """노이즈 모델에 넣을 게인: BFS 는 chunk 게인을 모델에 있는 값으로 맞춘 것, LUCID 는 0 dB 고정."""
    gain = float(frame.meta.get("gain_db") or 0.0)
    allowed = ptc_model.GAINS_DB[frame.key]
    return min(allowed, key=lambda g: abs(g - gain))


def frame_model(frame):
    """그 프레임의 게인 · 노출에서 노이즈 모델 값 (ptc_model.frame_params)."""
    exposure = float(frame.meta.get("exposure_time_us") or 0.0)
    return ptc_model.frame_params(frame.key, frame_gain(frame), round(exposure, 1))


def black_and_white(frame):
    """(CFA 별 black, white) — 노이즈 모델의 dark offset (그 게인 · 노출에서 픽셀 지도의 CFA 평균)."""
    return ptc_model.dng_black_white(frame.key, frame_model(frame))


def brightness(frame, percentile=99.5):
    """CFA 채널마다 percentile 을 재서 가장 밝은 채널의 (값 − black)/(white − black)."""
    raw = frame.raw
    black, white = black_and_white(frame)
    b = float(np.mean(list(black.values())))
    level = max(float(np.percentile(raw[dy::2, dx::2][::4, ::4], percentile)) for dy in (0, 1) for dx in (0, 1))
    return (level - b) / max(1.0, white - b)


def cfa_means(frame):
    raw = frame.raw
    return {c: float(raw[i::2, j::2].mean()) for c, (i, j) in ptc_model.CFA.items()}


FLICKER_PERIOD_60HZ_US = 1e6 / 120.0      # 60 Hz 전원 → 밝기 리플 120 Hz
FLICKER_NOTE = ("exposures >= 8.33 ms rounded to multiples of 1/120 s (60 Hz mains flicker): the step ratio is not "
                "exactly the nominal factor — use the recorded chunk exposure and derive step ratios from the measured "
                "signal")
LUCID_SUB_MS_NOTE = ("LUCID exposure < 1 ms: set in ~20 us steps but the image changes in ~40 us steps with a ~32 us "
                     "offset — derive the ratio between steps from the measured signal, not from the exposure values")
ANALYSIS_NOTES = [
    "per-pixel dark: use the package maps (offset_1ms + dark_rate*(T-1 ms)) plus the session shift in dark_shift; "
    "do not subtract a few-frame dark average per pixel (adds sigma/sqrt(n) noise)",
    "hot / noisy pixels grow with exposure (1 s: BFS ~2k, LUCID ~8k per channel): mask them with maps_gNN.npz",
    "a clean reference made by averaging N frames keeps variance/N of noise — include it in evaluation",
    "LUCID HDR: noise near the channel transitions is 17-45 % above the model (see noise_model.at_this_frame."
    "hdr_transitions, quad-based)",
]


def chi2_median_over_k(k):
    """median(χ²_k)/k — 픽셀별 분산 추정(자유도 k)의 중앙값이 참값보다 작게 나오는 비율."""
    if k == 2:
        return float(np.log(2.0))
    z = np.random.default_rng(0).standard_normal((200_000, k))
    return float(np.median((z * z).sum(axis=1)) / k)


def dark_variance_scale(key, entry):
    """장면 dark 의 분산 / 모델 dark 분산 (CFA 별, hot · noisy 마스크 밖). jsh-16 합의: 노출별 · CFA 별 전역 보정.

    픽셀마다 n 장의 표본분산(자유도 n−1)을 모델 분산으로 나눈 비 r 의 평균(mean, 주 값: 분포 가정 없이 불편 —
    BFS 는 16 DN 양자화 때문에 중앙값이 크게 흔들린다)과, 중앙값을 median(χ²_k/k) 로 나눈 값(대조용)을 남긴다.
    """
    n = len(entry["means"])
    if n < 2 or "sum" not in entry:
        return None
    p = entry["params"]
    nm = ptc_model.model(key)
    g0, _ = nm._gain(p["gain_db"])
    _offset, dvar = nm.dark(p["gain_db"], p["exposure_us"])
    maps = nm.maps(g0)
    good = ~(maps["hot"] | maps["noisy"])
    s2 = (entry["sumsq"] - entry["sum"] ** 2 / n) / (n - 1)
    m_k = chi2_median_over_k(n - 1)
    out = {"frames": n, "dof": n - 1, "chi2_median_over_k": m_k,
           "method": "k_var = mean over pixels outside the calibration hot/noisy masks of (sample variance / model "
                     "dark variance); median_corrected = median / median(chi2_k/k), for comparison"}
    for c, (i, j) in ptc_model.CFA.items():
        r = (s2[i::2, j::2] / dvar[i::2, j::2])[good[i::2, j::2]]
        out[c] = {"k_var": float(np.mean(r)), "median_corrected": float(np.median(r) / m_k)}
    return out


def step_exposures(base_us, evs, keys, max_us=None, flicker_period_us=None, clip_common=False):
    """노출 단계 [{key: us}] — 기준 × 2^EV 를 카메라마다 모델 유효 범위 · 최대 노출로 자르고 깜빡임 주기 배수로 맞춘다.

    LUCID 는 800 ms, BFS 는 1 s 까지라 맨 위 단계는 두 카메라 노출이 다를 수 있다 (jsh-16 합의: 정지 장면 ·
    카메라별 RAW 모델이라 괜찮다). clip_common=True 면 두 카메라 모두 좁은 쪽 범위로 자른다 (같은 노출 비교용).
    """
    ranges = {k: ptc_model.EXPOSURE_US[k] for k in keys}
    if clip_common:
        common = (max(r[0] for r in ranges.values()), min(r[1] for r in ranges.values()))
        ranges = {k: common for k in keys}
    if max_us:
        ranges = {k: (lo, min(hi, max_us)) for k, (lo, hi) in ranges.items()}
    out = []
    for ev in evs:
        step = {}
        for k in keys:
            lo, hi = ranges[k]
            v = float(np.clip(base_us[k] * 2.0 ** ev, lo, hi))
            if flicker_period_us and v >= flicker_period_us:
                n = max(1, int(v / flicker_period_us + 0.5))
                while n > 1 and n * flicker_period_us > hi + 1e-6:
                    n -= 1
                v = n * flicker_period_us
            step[k] = v
        out.append(step)
    return out


class CameraRebooted(RuntimeError):
    pass


def exposure_ok(meta, want_us):
    have = meta.get("exposure_time_us")
    if have is None or not meta.get("chunk_valid", True) or meta.get("crc_ok") is False:
        return False
    return abs(have - want_us) <= max(0.015 * want_us, 60.0)


# ---------- 드라이버 기동 (rawgui 와 같은 코드 경로) ----------

def launch_drivers(log_dir, log=print):
    sys.path.insert(0, str(GUI_SCRIPTS))
    import net_tools
    import sensor_discovery
    import sensor_launcher
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    registry = sensor_discovery.load_registry()
    groups = {g["key"]: g for g in registry["groups"]}
    found = {k: sensor_discovery.discover_group(g) for k, g in groups.items()}
    fixed = False
    for key, r in found.items():
        for d in r.get("devices", []):
            if not d.get("subset"):
                continue
            # 다른 프로그램(예: 다른 세션의 PTC 촬영)이 쥐고 있으면 IP 도 바꾸지 않고 드라이버도 띄우지 않는다 —
            # 드라이버는 거부당해도 다시 시도하다가 그 프로그램이 잠깐 놓는 틈에 카메라를 가로챈다.
            held = net_tools.gvcp_control_holder(d["nic"], d.get("ip"))
            if held and held[0]:
                raise RuntimeError(f"{d['identity']} 를 다른 프로그램({held[1]})이 쓰고 있다 — 끝난 뒤에 기동")
            if d["state"] == sensor_discovery.SUBNET:
                sub = next(s for s in groups[key]["subsets"] if s["key"] == d["subset"])
                log(f"IP 맞춤: {d['identity']} {d.get('ip')} → {sub['force_ip_base']}")
                net_tools.gvcp_force_ip(d["nic"], d["mac"], sub["force_ip_base"], "255.255.255.0")
                fixed = True
    if fixed:
        time.sleep(3)
        found = {k: sensor_discovery.discover_group(g) for k, g in groups.items()}
    env = dict(os.environ)
    for var in sensor_launcher.LEAKY_ENV:
        env.pop(var, None)
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    procs = []
    for key, g in groups.items():
        notes = []
        program, args, _ns = sensor_launcher.build_command(g, {}, found[key].get("devices", []), notes)
        for n in notes:
            log(f"[{key}] {n}")
        out = open(log_dir / f"{key}.log", "w")
        procs.append(subprocess.Popen([program] + args, cwd=os.path.expanduser(g.get("workdir", "~")), env=env,
                                      stdout=out, stderr=subprocess.STDOUT, start_new_session=True))
    return procs


def stop_drivers(procs):
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    for p in procs:
        try:
            p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


# ---------- 촬영 ----------

def slug(text):
    text = re.sub(r"\s+", "_", text.strip())
    return re.sub(r"[^0-9A-Za-z가-힣_\-]", "", text)[:40] or "scene"


def new_scene_dir(name, root=STILLS):
    root.mkdir(parents=True, exist_ok=True)
    numbers = [int(m.group(1)) for p in root.iterdir() if (m := re.match(r"scene_(\d+)", p.name))]
    path = root / f"scene_{max(numbers, default=0) + 1:03d}_{slug(name)}"
    path.mkdir()
    return path


def write_json(path, data):
    tmp = Path(str(path) + ".part")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1, default=str))
    tmp.replace(path)


GROUP_MASK = {"lucid": 0x1, "bfs": 0x2}     # 카메라마다 따로 쏠 수 있게 액션 그룹 마스크를 나눈다


class Rig:
    """두 카메라 쌍 촬영. 2026-10-20 실측으로 정한 방식:

    - LUCID TRI054S 는 트리거를 받고 '2 × 프레임 주기 − 노출'(2 Hz 에서 약 1 s, TriggerLatencyTime 노드 값)
      뒤에 노출을 시작한다 (저지연 트리거 TriggerLatency 는 이 모델에 구현돼 있지 않다). BFS 는 바로 시작한다
      (TriggerDelay 최대 65.5 ms 라 BFS 쪽에서 기다릴 수도 없다). 그래서 LUCID 를 먼저 쏘고, 두 카메라의
      시작 지연 차이만큼 뒤에 BFS 를 쏴서 노출 시작을 맞춘다. 지연은 쌍마다 header.stamp(노출 시작, 호스트
      시계)로 다시 재서 고친다 — info["start_offset_ms"] 가 그 쌍의 LUCID − BFS 시작 차이.
    - LUCID 는 노출을 바꾼 뒤 첫 트리거 프레임의 영상이 이전 노출이다 (chunk 값은 새 값). 그래서 노출을
      바꾼 뒤에는 한 쌍을 버리고 찍는다 (그 쌍으로 지연도 잰다).
    - 프레임은 노출 시작 시각이 그 트리거로 예상되는 창 안에 든 것만 받는다 (늦게 온 이전 트리거 프레임 거름).
    """

    def __init__(self, link, iface=IFACE, log=print, unicast=None, keys=None):
        self.link, self.iface, self.log = link, iface, log
        self.unicast = unicast          # None = 브로드캐스트, 또는 {key: 카메라 IP} (그 카메라에만 보낸다)
        self._keys = list(keys) if keys else [c.key for c in CAMS]    # 한 카메라만 쓸 때 ["bfs"] 등
        self.last_fire = 0.0
        self.latency_s = {}             # 카메라별 트리거 → 노출 시작 지연 (실측)
        self.dirty = True               # 노출을 바꾼 뒤 아직 버림 쌍을 안 찍었다
        self.uptime = {}                # 카메라별 마지막 DeviceUptime (재부팅 감지)
        self.lucid_triggered = False    # LUCID 가 지금 trigger.mode=action 인가
        self.free_run_stamp = {}        # free-run 으로 마지막에 받은 프레임 stamp
        self.changed_ns = 0             # 마지막으로 노출 · 트리거 모드를 바꾼 시각

    def keys(self):
        return list(self._keys)

    def arm(self, on):
        for key in self.keys():
            self.link.set_params(key, {"trigger.action.group_mask": GROUP_MASK[key],
                                       "trigger.mode": "action" if on else "off"})
        self.log("트리거: 액션 커맨드" if on else "트리거: 끔 (free run)")
        self.dirty = True
        self.lucid_triggered = on
        if on:
            time.sleep(1.0)       # 날아오던 free-run 프레임이 다 들어오게

    def set_exposure(self, exposure_us):
        for key, value in exposure_us.items():
            self.link.set_params(key, {"camera.ExposureTime": float(value)})
        self.dirty = True
        self.changed_ns = time.time_ns()     # free-run 프레임은 이 뒤에 노출이 시작된 것만 받는다

    def _fire(self, key):
        target = (self.unicast or {}).get(key)
        return gvcp_action.fire(self.iface, group_mask=GROUP_MASK[key], expect=1,
                                unicast=[target] if target else None)

    def _pair(self, exposure_us, settle):
        """한 쌍: LUCID 먼저, 시작 지연 차이만큼 뒤 BFS. ({key: Frame}, info) — 못 받은 카메라는 빠진다."""
        keys = sorted(exposure_us, key=lambda k: k != "lucid")
        known = all(k in self.latency_s for k in keys)
        lead = max(0.0, self.latency_s.get("lucid", 0.0) - self.latency_s.get("bfs", 0.0)) \
            if known and len(keys) > 1 else 0.0
        wait = 0.6 - (time.monotonic() - self.last_fire)
        if wait > 0:
            time.sleep(wait)
        fire_ns, acks, t_first = {}, {}, None
        for k in keys:
            if t_first is not None and lead > 0:
                target = t_first + lead
                while (left := target - time.perf_counter()) > 0:
                    time.sleep(left - 0.0015 if left > 0.002 else 0)
            t_send = time.perf_counter()
            fire_ns[k], acks[k] = self._fire(k)
            t_first = t_first if t_first is not None else t_send
        self.last_fire = time.monotonic()
        longest = max(exposure_us.values()) / 1e6
        got = {}
        for k in keys:
            if known and not settle:
                lo, hi = self.latency_s[k] - 0.15, self.latency_s[k] + 0.15
            else:      # 지연을 아직 모른다 (첫 쌍 · 노출을 바꾼 뒤): 넓게
                lo, hi = -0.05, 2.0 * (longest + 1.0) + 1.0
            deadline = max(hi, 0.0) + longest + 3.0
            frame = self.link.take(
                k, 0.0, max(0.0, fire_ns[k] / 1e9 + deadline - time.time()),
                lambda f, k=k, lo=lo, hi=hi: lo <= (f.stamp_ns - fire_ns[k]) / 1e9 <= hi and
                exposure_ok(f.meta, exposure_us[k]))
            if frame is not None:
                got[k] = frame
                self.latency_s[k] = (frame.stamp_ns - fire_ns[k]) / 1e9
        info = {"fire_time_ns": fire_ns, "acks": {k: [list(a) for a in v] for k, v in acks.items()},
                "trigger_lead_ms": lead * 1e3,
                "start_latency_ms": {k: (got[k].stamp_ns - fire_ns[k]) / 1e6 for k in got},
                "camera_timestamp_ns": {k: got[k].meta.get("camera_timestamp_ns") for k in got},
                "camera_frame_id": {k: got[k].meta.get("camera_frame_id") for k in got},
                "header_stamp_ns": {k: got[k].stamp_ns for k in got}}
        if "lucid" in got and "bfs" in got:
            info["start_offset_ms"] = (got["lucid"].stamp_ns - got["bfs"].stamp_ns) / 1e6
        self.check_reboot(got)
        return got, info

    def _lucid_trigger(self, on):
        if "lucid" in self.keys() and self.lucid_triggered != on:
            self.link.set_params("lucid", {"trigger.mode": "action" if on else "off"})
            self.lucid_triggered = on
            self.dirty = True
            self.changed_ns = time.time_ns()
            self.log("  LUCID " + ("트리거" if on else "free-run (긴 노출)"))

    def _free_run_frame(self, key, exposure_us):
        """free-run 프레임: chunk 노출이 맞고 settled(설정 직후 두 장 제외)이며 앞서 받은 것보다 새 프레임."""
        # 노출 · 모드를 바꾼 뒤에 노출이 시작된 프레임만 (그 전에 버퍼에 남은 프레임 — 예를 들어 렌즈캡을 벗기기 전
        # dark 단계의 free-run 프레임 — 을 다음 단계로 받지 않게). header.stamp 가 노출 시작(호스트 시계)이다.
        last = max(self.free_run_stamp.get(key, 0), self.changed_ns)
        frame = self.link.take(key, 0.0, exposure_us / 1e6 * 3 + 10.0,
                               lambda f: f.stamp_ns > last and exposure_ok(f.meta, exposure_us)
                               and f.meta.get("settled", True))
        if frame is None:
            raise RuntimeError(f"{CAM[key].label} free-run 프레임이 안 온다")
        self.free_run_stamp[key] = frame.stamp_ns
        self.check_reboot({key: frame})
        return frame

    def capture(self, exposure_us, lucid_free_run=False):
        """한 쌍. lucid_free_run: LUCID 는 free-run 으로 받고 나머지만 트리거한다 (동시가 아니다).

        2026-10-20 jsh-16 과 합의: 트리거 모드의 LUCID 는 노출 ≥ 300 ms 에서 dark 가 모델과 다르다
        (분산 +14 %, 1 s 에서 offset −7 DN). 프레임 주기가 길어지는 탓으로 보인다. free-run 은 모델과 맞는다.
        """
        if not lucid_free_run or "lucid" not in exposure_us:
            self._lucid_trigger(True)
            frames, info = self.shoot(exposure_us)
            return frames, dict(info, mode="pair (simultaneous start)")
        self._lucid_trigger(False)
        others = {k: v for k, v in exposure_us.items() if k != "lucid"}
        frames, info = self.shoot(others) if others else ({}, {"fire_time_ns": {}})
        lucid = self._free_run_frame("lucid", exposure_us["lucid"])
        frames["lucid"] = lucid
        info = dict(info, mode="split (LUCID free-run, not simultaneous)")
        info["fire_time_ns"] = dict(info.get("fire_time_ns", {}), lucid=lucid.stamp_ns)
        info.setdefault("header_stamp_ns", {})["lucid"] = lucid.stamp_ns
        info.setdefault("camera_frame_id", {})["lucid"] = lucid.meta.get("camera_frame_id")
        return frames, info

    def shoot(self, exposure_us, retries=1):
        """두 카메라 한 쌍. ({key: Frame}, 정보 dict). 노출을 바꾼 뒤면 버림 쌍을 먼저 찍는다."""
        keys = list(exposure_us)
        if self.dirty:
            got, _info = self._pair(exposure_us, settle=True)
            self.dirty = False
            if "lucid" in got:
                # The flush frame may still carry the old exposure (and so the old start latency), so the
                # next window comes from the model, measured 2026-10-20 within 0.2 ms at 2 Hz and 0.67 Hz:
                # LUCID exposure ends 2 frame periods (+0.8 ms) after the trigger.
                fps = got["lucid"].meta.get("acquisition_frame_rate_hz") or 0.0
                if fps > 0:
                    self.latency_s["lucid"] = 2.0 / fps - exposure_us["lucid"] / 1e6 + 0.0008
                self.log(f"  (버림 쌍: 노출 반영 — LUCID 시작 지연 {self.latency_s['lucid'] * 1e3:.0f} ms 로 맞춘다)")
        for attempt in range(retries + 1):
            got, info = self._pair(exposure_us, settle=False)
            info["attempt"] = attempt
            if len(got) == len(keys):
                return got, info
            missing = [CAM[k].label for k in keys if k not in got]
            self.log(f"  {', '.join(missing)} 프레임 없음 — 다시 찍는다" if attempt < retries
                     else f"  {', '.join(missing)} 프레임 없음")
            time.sleep(max(self.latency_s.values(), default=1.0) + max(exposure_us.values()) / 1e6 + 1.0)
            self.dirty = True
            self._pair(exposure_us, settle=True)      # 지연을 다시 잰다
            self.dirty = False
        raise RuntimeError("쌍을 못 찍었다: " + ", ".join(CAM[k].label for k in keys if k not in got))

    def meter(self, start_us, target=0.8, percentile=99.5, limits=None, rule="same", iterations=6,
              stop=None, progress=None):
        """측광 → 기준 노출 {key: us}, 마지막 밝기 {key: 0..1}.

        rule "same": 두 카메라를 같은 노출로 찍으면서, 더 먼저 포화되는 쪽의 밝은 채널 p99.5 가 포화의
        target 이 되는 노출 하나를 찾는다. "per_camera": 카메라마다 따로 (ETTR).
        """
        limits = {k: (max(v[0], ptc_model.EXPOSURE_US[k][0]), min(v[1], ptc_model.EXPOSURE_US[k][1]))
                  for k, v in (limits or DEFAULT_LIMITS_US).items()}
        keys = list(start_us)

        def want(exposure, level):
            if level >= 0.97:
                return exposure / 8.0
            if level <= 1e-4:
                return exposure * 32.0
            return exposure * target / level

        if rule != "same":
            exposure = {k: float(np.clip(v, *limits[k])) for k, v in start_us.items()}
            levels = {}
            for it in range(iterations):
                if stop is not None and stop.is_set():
                    raise Stopped()
                if progress is not None:
                    progress(it + 1, iterations, exposure)
                self.set_exposure(exposure)
                frames, _info = self.shoot(exposure)
                levels = {k: brightness(f, percentile) for k, f in frames.items()}
                self.log("  측광 %d: " % (it + 1) + ", ".join(
                    f"{CAM[k].label} {exposure[k] / 1000:.3g} ms → 밝기 {levels[k] * 100:.1f}%" for k in levels))
                new = {k: float(np.clip(want(exposure[k], levels[k]), *limits[k])) for k in keys}
                done = all(abs(levels[k] / target - 1.0) < 0.12 or abs(new[k] - exposure[k]) < 1e-6 for k in keys)
                exposure = new
                if done:
                    break
            return exposure, levels

        # "same": 공통 노출 c 하나. 카메라마다 자기 범위로 잘라 찍고(LUCID 800 ms, BFS 1 s), 자기 상한 안에서
        # target 에 닿는 카메라만 c 를 제한한다 (상한에 걸리는 카메라는 거기서도 target 아래라 포화 걱정이 없다).
        lo = max(limits[k][0] for k in keys)
        hi = max(limits[k][1] for k in keys)
        common = float(np.clip(min(start_us.values()), lo, hi))
        levels = {}
        for it in range(iterations):
            if stop is not None and stop.is_set():
                raise Stopped()
            exposure = {k: float(np.clip(common, *limits[k])) for k in keys}
            if progress is not None:
                progress(it + 1, iterations, exposure)
            self.set_exposure(exposure)
            frames, _info = self.shoot(exposure)
            levels = {k: brightness(f, percentile) for k, f in frames.items()}
            self.log("  측광 %d: " % (it + 1) + ", ".join(
                f"{CAM[k].label} {exposure[k] / 1000:.3g} ms → 밝기 {levels[k] * 100:.1f}%" for k in levels))
            limits_c = [want(exposure[k], levels[k]) for k in keys if want(exposure[k], levels[k]) < limits[k][1]]
            new_common = float(np.clip(min(limits_c) if limits_c else hi, lo, hi))
            binding = [k for k in keys if want(exposure[k], levels[k]) < limits[k][1]]
            done = abs(new_common - common) < 1e-6 or (
                binding and min(abs(levels[k] / target - 1.0) for k in binding) < 0.12)
            common = new_common
            if done:
                break
        return {k: common for k in keys}, levels

    def save(self, frame, path, context):
        """DNG + json. black = 모델 dark offset (CFA 평균), NoiseProfile = BFS 만 (ptc_model.noise_profile)."""
        params = frame_model(frame)
        black, white = ptc_model.dng_black_white(frame.key, params)
        meta = frame.meta
        noise = ptc_model.noise_profile(frame.key, params)
        notes = list(context.get("notes", []))
        if frame.key == "lucid" and (meta.get("exposure_time_us") or 1e9) < 1000:
            notes.append(LUCID_SUB_MS_NOTE)
        record = {k: v for k, v in context.items() if k not in ("lenses", "notes")}
        record.update({
            "camera_key": frame.key, "camera": meta, "header_stamp_ns": frame.stamp_ns,
            "noise_model": {"reference": ptc_model.reference(frame.key), "gain_db": frame_gain(frame),
                            "at_this_frame": params},
            "black_level_source": "noise model dark offset at this gain and exposure (CFA mean of the per-pixel "
                                  "map); the session shift measured from dark frames is in scene.json",
            "noise_profile_basis": "DNG NoiseProfile: S = alpha/(white-black), O = dark variance at this exposure "
                                   "(read sigma^2 + dark_var_rate*T)/(white-black)^2" if noise else
                                   "none (LUCID HDR: use the package alpha(S) table)",
            "notes": notes})
        lens = context.get("lenses", {}).get(frame.key, {})
        taken = dt.datetime.fromtimestamp(context.get("fire_time_ns", time.time_ns()) / 1e9)
        return rawdng.write(path, frame.raw, cfa=cfa_of(frame), black=black, white=white,
                            exposure_s=meta["exposure_time_us"] / 1e6, make=CAM[frame.key].make,
                            model=meta.get("model") or CAM[frame.key].label, serial=meta.get("serial") or "?",
                            meta=record, noise=noise, bps=24 if frame.key == "lucid" else 16,
                            f_number=lens.get("f_number"), taken=taken)

    def _dark_shift(self, doc, darks, stage, keys, n_steps):
        """dark 단계의 세션 offset 이동량 (CFA 평균 − 모델 offset) → scene.json. 0.1·read σ 넘으면 경고."""
        out = doc["dark_shift"].setdefault(stage, {})
        for k in keys:
            per = out.setdefault(k, {})
            for i in range(n_steps):
                d = darks.get((stage, i, k))
                if not d:
                    continue
                p = d["params"]
                entry = {"exposure_us": p["exposure_us"], "gain_db": p["gain_db"], "frames": len(d["means"]),
                         "files": d["files"], "first_stamp_ns": min(d["times_ns"]), "last_stamp_ns": max(d["times_ns"]),
                         "k_var": d.get("k_var"), "cfa": {}}
                worst = 0.0
                for c in ptc_model.CFA:
                    measured = float(np.mean([m[c] for m in d["means"]]))
                    model = p["cfa"][c]["dark_offset_dn"]
                    rel = (measured - model) / p["cfa"][c]["read_sigma_dn"]
                    entry["cfa"][c] = {"measured_mean_dn": measured, "model_offset_dn": model,
                                       "shift_dn": measured - model, "shift_over_read_sigma": rel}
                    worst = max(worst, abs(rel))
                per[str(i)] = entry
                if worst > ptc_model.SHIFT_WARN_SIGMA:
                    msg = (f"{stage} {CAM[k].label} {p['exposure_us'] / 1000:.4g} ms: dark offset 이동 {worst:.2f}·read σ "
                           f"(> {ptc_model.SHIFT_WARN_SIGMA}σ)")
                    doc["warnings"].append(msg)
                    self.log("  경고: " + msg)
        # dark current 는 offset 을 올리기만 한다. 노출이 길어지는데 이동량이 내려가면 카메라 안의 전역 검은 레벨
        # 보정이 다르게 걸린 것 (2026-10-20, LUCID 1 s 에서 −9 DN 이 그렇게 나왔다 — jsh-16 합의로 경고).
        for k in keys:
            entries = sorted(out.get(k, {}).values(), key=lambda e: e["exposure_us"])
            for short, long_ in zip(entries, entries[1:]):
                drops = [long_["cfa"][c]["shift_over_read_sigma"] - short["cfa"][c]["shift_over_read_sigma"]
                         for c in ptc_model.CFA]
                if max(drops) < -ptc_model.SHIFT_WARN_SIGMA:
                    msg = (f"{stage} {CAM[k].label}: 노출 {short['exposure_us'] / 1000:.4g} → {long_['exposure_us'] / 1000:.4g} ms "
                           f"에서 dark 이동량이 {max(drops):+.2f}σ 로 내려간다 — dark current 로 설명 안 되는 방향 "
                           "(카메라 내부 검은 레벨 보정이 다르게 걸렸을 수 있다)")
                    doc["warnings"].append(msg)
                    self.log("  경고: " + msg)
        if stage == "dark" and "dark_pre" in doc["dark_shift"]:
            drift = {}
            for k in keys:
                for i, post in doc["dark_shift"]["dark"].get(k, {}).items():
                    pre = doc["dark_shift"]["dark_pre"].get(k, {}).get(i)
                    if pre:
                        drift.setdefault(k, {})[i] = {c: post["cfa"][c]["shift_dn"] - pre["cfa"][c]["shift_dn"]
                                                      for c in ptc_model.CFA}
            doc["dark_shift"]["drift_post_minus_pre_dn"] = drift

    def preflight(self, bfs_gain_db=None):
        """녹화 조건 대조 (ptc_model.REQUIRED) → {key: (틀림, 모름, 경고)}. 최근 프레임 메타데이터 + 드라이버 파라미터."""
        results = {}
        for key in self.keys():
            meta_msg = self.link.last_meta[key]
            meta = _plain(dict(message_to_ordereddict(meta_msg))) if meta_msg is not None else {}
            names = sorted({r.param for r in ptc_model.REQUIRED[key] if r.param})
            params = self.link.get_params(key, names)
            results[key] = ptc_model.check(key, meta, params, gain_db=bfs_gain_db if key == "bfs" else None)
        return results

    def fix(self, results):
        """틀린 항목 중 고칠 수 있는 것을 조건대로 (REQUIRED 순서: auto 를 값보다 먼저)."""
        failed = []
        for key, (wrong, _unknown, _warnings) in results.items():
            for label, _have, _want, fix in wrong:
                if not fix:
                    failed.append(f"{CAM[key].label} {label}: 도구가 못 고친다")
                    continue
                for name, value in fix.items():
                    try:
                        self.link.set_params(key, {name: value})
                    except Exception as e:  # noqa: BLE001
                        failed.append(f"{CAM[key].label} {label}: {e}")
        return failed

    def check_reboot(self, frames):
        """DeviceUptime 이 줄었으면 재부팅 (드라이버가 메타데이터 device_uptime_s 를 줄 때)."""
        for key, frame in frames.items():
            uptime = frame.meta.get("device_uptime_s")
            if uptime is None or uptime != uptime:
                continue
            last = self.uptime.get(key)
            if last is not None and uptime + 5.0 < last:
                raise CameraRebooted(f"{CAM[key].label} 가 재부팅됐다 (DeviceUptime {last:.0f} s → {uptime:.0f} s) — "
                                     "설정이 기본값으로 돌아갔을 수 있어 장면을 멈춘다")
            self.uptime[key] = uptime

    def snapshot(self, folder):
        """노드맵 사본과 드라이버 파라미터 — 장면 폴더에."""
        (folder / "nodemap").mkdir(exist_ok=True)
        cams = {}
        for c in (CAM[k] for k in self.keys()):
            params = self.link.all_params(c.key)
            meta_msg = self.link.last_meta[c.key]
            meta = _plain(dict(message_to_ordereddict(meta_msg))) if meta_msg is not None else {}
            serial = meta.get("serial") or ""
            dump = NODEMAP_DIR / f"{serial}_latest_nodemap.json"
            if serial and dump.is_file():
                shutil.copy2(dump, folder / "nodemap" / f"{c.key}_{serial}_nodemap.json")
            cams[c.key] = {"label": c.label, "namespace": c.ns, "node": c.node, "serial": serial,
                           "model": meta.get("model"), "firmware_version": meta.get("firmware_version"),
                           "lens": {"model": params.get("lens.model"), "f_number": params.get("lens.f_number")},
                           "driver_params": params}
        return cams

    def run(self, scene, plan, stop=None, progress=None, ask=None, root=STILLS, on_step=None):
        """장면 하나: (dark_pre) → bright → dark, 각 단계 × 노출 단계 × 장수. 장면 폴더 Path 를 돌려준다.

        progress(done, total, shot, frames): 쌍마다. on_step(stage, i, n, exposure_us): 노출 단계를 시작할 때.
        ask(질문, kind) → bool: 렌즈캡을 씌우거나 벗길 때 (kind "cap_on" / "cap_off"). False 면 그 dark 를 건너뛴다.
        plan["dark_before"]: bright 앞에도 dark (BFS 게인 ≥ 18 dB 에서 — offset 흔들림, jsh-16 합의).
        """
        stop = stop or threading.Event()
        keys = self.keys()
        evs = [float(e) for e in plan["ev"]]
        steps = step_exposures({k: float(plan["base_us"][k]) for k in keys}, evs, keys,
                               plan.get("max_us"), plan.get("flicker_period_us"), plan.get("clip_common", False))
        frames_for = {"dark_pre": int(plan.get("dark_pre_frames", 3)), "bright": int(plan["frames"]),
                      "dark": int(plan["frames"])}
        stages = (["dark_pre"] if plan.get("dark") and plan.get("dark_before") else []) + ["bright"] + \
            (["dark"] if plan.get("dark") else [])
        total = sum(frames_for[s] for s in stages) * len(evs)
        folder = new_scene_dir(scene.get("name") or "scene", root)
        self.log(f"장면 폴더: {folder}")
        before = {k: self.link.get_params(k, ["camera.ExposureTime"])["camera.ExposureTime"] for k in keys}
        notes = []
        if plan.get("flicker_period_us"):
            notes.append(FLICKER_NOTE)
        doc = {"scene": dict(scene, created=dt.datetime.now().isoformat(timespec="seconds")),
               "plan": dict(plan, ev=evs, stages=stages, exposures_us=steps), "cameras": self.snapshot(folder),
               "noise_model": {k: ptc_model.reference(k) for k in keys},
               "trigger": {"kind": "GigE Vision action command (broadcast, no PTP); LUCID fired first, BFS after the "
                                   "measured start-latency difference", "iface": self.iface,
                           "device_key": gvcp_action.DEVICE_KEY, "group_key": gvcp_action.GROUP_KEY,
                           "group_mask": GROUP_MASK},
               "format": {"dng": "tools/rawdng.py — big-endian, uncompressed; BFS uint16, LUCID HDR 24-bit packed",
                          "file_name": "<stage>/<NN>_ev<±X.X>_<KK>_<cam>.dng"},
               "analysis_notes": ANALYSIS_NOTES + notes, "shots": [], "dark_shift": {}, "warnings": [],
               "status": "running"}
        lenses = {k: doc["cameras"][k]["lens"] for k in keys}
        write_json(folder / "scene.json", doc)
        done = 0
        darks = {}            # (stage, step, key) → [CFA 평균], 파일, 시각
        self.uptime = {}
        self.arm(True)
        try:
            for stage in stages:
                prompt = {"dark_pre": ("렌즈캡을 씌워 줘 — bright 앞 dark 를 먼저 찍는다", "cap_on"),
                          "bright": ("렌즈캡을 벗겨 줘 — bright 를 찍는다", "cap_off"),
                          "dark": ("bright 끝. 렌즈캡을 씌워 줘 — 같은 노출로 dark 를 찍는다", "cap_on")}[stage]
                if ask is not None and (stage != "bright" or "dark_pre" in stages):
                    if not ask(*prompt):
                        if stage == "bright":
                            raise Stopped()
                        self.log(f"{stage} 건너뜀")
                        doc["plan"][f"{stage}_skipped"] = True
                        continue
                (folder / stage).mkdir(exist_ok=True)
                for i, (ev, exposure) in enumerate(zip(evs, steps)):
                    if on_step is not None:
                        on_step(stage, i, len(evs), exposure)
                    self.set_exposure(exposure)
                    self.log(f"[{stage}] EV {ev:+.1f}: " + ", ".join(
                        f"{CAM[k].label} {exposure[k] / 1000:.4g} ms" for k in keys))
                    for j in range(frames_for[stage]):
                        if stop.is_set():
                            raise Stopped()
                        lucid_free = plan.get("lucid_trigger_max_us") is not None and "lucid" in exposure and \
                            exposure["lucid"] > plan["lucid_trigger_max_us"]
                        frames, info = self.capture(exposure, lucid_free_run=lucid_free)
                        shot = {"stage": stage, "step": i, "ev": ev, "index": j, "requested_exposure_us": exposure,
                                **info, "files": {}}
                        context = {"scene": doc["scene"], "shot": {k: v for k, v in shot.items() if k != "files"},
                                   "lenses": lenses, "fire_time_ns": min(info["fire_time_ns"].values()),
                                   "notes": notes}
                        for k, frame in frames.items():
                            name = f"{stage}/{i:02d}_ev{ev:+.1f}_{j:02d}_{k}.dng"
                            self.save(frame, folder / name, context)
                            shot["files"][k] = name
                            if stage != "bright":
                                d = darks.setdefault((stage, i, k), {"means": [], "files": [], "times_ns": [],
                                                                     "params": frame_model(frame)})
                                d["means"].append(cfa_means(frame))
                                d["files"].append(name)
                                d["times_ns"].append(frame.stamp_ns)
                                x = frame.raw.astype(np.float64)
                                if "sum" not in d:
                                    d["sum"], d["sumsq"] = np.zeros_like(x), np.zeros_like(x)
                                d["sum"] += x
                                d["sumsq"] += x * x
                        doc["shots"].append(shot)
                        write_json(folder / "scene.json", doc)
                        done += 1
                        if progress is not None:
                            progress(done, total, shot, frames)
                    if stage != "bright":
                        for k in keys:
                            d = darks.get((stage, i, k))
                            if d is not None:
                                d["k_var"] = dark_variance_scale(k, d)
                                d.pop("sum", None)
                                d.pop("sumsq", None)
                if stage != "bright":
                    self._dark_shift(doc, darks, stage, keys, len(evs))
            doc["status"] = "complete"
        except Stopped:
            doc["status"] = "stopped"
            self.log("중지했다")
        except Exception as e:  # noqa: BLE001
            doc["status"] = f"error: {e}"
            raise
        finally:
            write_json(folder / "scene.json", doc)
            try:
                self.arm(False)
                self.set_exposure(before)
            except Exception as e:  # noqa: BLE001
                self.log(f"되돌리기 실패: {e}")
        return folder


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--scene", default="test")
    ap.add_argument("--lux", default="")
    ap.add_argument("--light", default="")
    ap.add_argument("--notes", default="")
    ap.add_argument("--ev", default="0 -1 -2 -3 -4 -5 -6")
    ap.add_argument("--frames", type=int, default=2)
    ap.add_argument("--meter", action="store_true", help="측광으로 기준 노출을 정한다")
    ap.add_argument("--rule", choices=["per_camera", "same"], default="same")
    ap.add_argument("--base-bfs-ms", type=float, default=20.0)
    ap.add_argument("--base-lucid-ms", type=float, default=20.0)
    ap.add_argument("--max-ms", type=float, default=2000.0)
    ap.add_argument("--lucid-output", default="")
    ap.add_argument("--dark", action="store_true")
    ap.add_argument("--root", default=str(STILLS))
    ap.add_argument("--iface", default=IFACE)
    args = ap.parse_args(argv)

    link = RosLink()
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10 and not all(link.alive(k) for k in CAM):
            time.sleep(0.2)
        missing = [CAM[k].label for k in CAM if not link.alive(k)]
        if missing:
            print("드라이버가 안 보인다:", ", ".join(missing))
            return 1
        rig = Rig(link, args.iface)
        limits = {k: (30.0, args.max_ms * 1000.0) for k in CAM}
        base = {"bfs": args.base_bfs_ms * 1000.0, "lucid": args.base_lucid_ms * 1000.0}
        if args.meter:
            rig.arm(True)
            try:
                base, levels = rig.meter(base, limits=limits, rule=args.rule)
            finally:
                rig.arm(False)
            print("기준 노출:", {k: f"{v / 1000:.4g} ms" for k, v in base.items()})
        plan = {"ev": args.ev.split(), "frames": args.frames, "base_us": base, "limits_us": limits,
                "dark": args.dark, "lucid_output": args.lucid_output or None, "rule": args.rule}
        scene = {"name": args.scene, "lux": args.lux, "light": args.light, "notes": args.notes}
        folder = rig.run(scene, plan, ask=lambda q: input(q + " [Enter=확인, n=건너뜀] ").strip().lower() != "n",
                         progress=lambda d, t, shot, frames: print(
                             f"  {d}/{t}  " + "  ".join(f"{k} {f.meta['exposure_time_us']:.0f}us" for k, f in
                                                       frames.items())),
                         root=Path(args.root))
        print("저장:", folder)
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
