"""랩 RAW bag 읽기 — image_raw 를 numpy 배열로, 같은 프레임의 image_raw/metadata 와 짝지어 준다.

    source ~/jsh/env.sh            # FlirMetadata · LucidMetadata 타입이 필요하다
    python3 ~/jsh/tools/rawbag.py <bag 폴더>                    # 토픽 · 프레임 수 · 첫 프레임 요약
    python3 ~/jsh/tools/rawbag.py <bag 폴더> lucid_tri054 3     # 그 카메라의 처음 3장

    from rawbag import frames
    for raw, meta, stamp_ns in frames(bag, "camera_bfs"):
        ...   # raw: (H, W) uint16 (BFS BayerRG16) / uint32 (LUCID BayerRG24) …, meta: dict (메타데이터 필드 전부)

image_raw 인코딩:
  bayer_rggb16 / mono16  → uint16 (BFS BayerRG16 = 12-bit 코드 << 4, LUCID BayerRG16 = min(값, 4095) << 4)
  32SC1                  → uint32 (LUCID BayerRG24, 값 < 2^24: 채널 출력은 12-bit 코드 + 페데스탈)
  bayer_rggb24 / mono24  → uint32 (LUCID raw24_container: packed — 3바이트를 풀어서)
  bayer_rggb8 / mono8    → uint8
CFA 는 meta["cfa_pattern"] (LUCID) 또는 인코딩 이름(RGGB). 블랙은 dark 프레임에서 재고(노이즈 모델의 offset),
포화는 meta["white_level_dn"] (0 = 모름).
"""
import sys
from pathlib import Path

import numpy as np

_META_SUFFIX = "/image_raw/metadata"


def _open(bag):
    import rosbag2_py
    bag = Path(bag)
    storage = "mcap" if any(bag.glob("*.mcap")) else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    return reader


def to_array(msg):
    """sensor_msgs/Image → numpy (H, W)."""
    enc = msg.encoding
    buf = bytes(msg.data)
    big = bool(msg.is_bigendian)
    if enc in ("bayer_rggb16", "bayer_bggr16", "bayer_gbrg16", "bayer_grbg16", "mono16", "16UC1"):
        a = np.frombuffer(buf, dtype=">u2" if big else "<u2").reshape(msg.height, msg.step // 2)
    elif enc == "32SC1":
        a = np.frombuffer(buf, dtype=">i4" if big else "<i4").reshape(msg.height, msg.step // 4).view(np.uint32)
    elif enc.endswith("24"):
        b = np.frombuffer(buf, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        b = b.reshape(msg.height, msg.width, 3).astype(np.uint32)
        a = (b[..., 0] << 16 | b[..., 1] << 8 | b[..., 2]) if big else (b[..., 0] | b[..., 1] << 8 | b[..., 2] << 16)
    elif enc.endswith("8"):
        a = np.frombuffer(buf, dtype=np.uint8).reshape(msg.height, msg.step)
    else:
        raise ValueError(f"unsupported encoding {enc}")
    return a[:, :msg.width]


def _as_dict(msg):
    from rosidl_runtime_py import message_to_ordereddict
    return dict(message_to_ordereddict(msg))


def frames(bag, namespace, limit=None):
    """(raw ndarray, metadata dict, header.stamp ns) — image_raw 와 metadata 를 header.stamp 로 짝짓는다."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    ns = "/" + namespace.strip("/")
    reader = _open(bag)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    raw_topic, meta_topic = ns + "/image_raw", ns + _META_SUFFIX
    if raw_topic not in types:
        raise KeyError(f"{raw_topic} 가 bag 에 없다 (있는 것: {sorted(types)})")
    raws, metas, n = {}, {}, 0
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic not in (raw_topic, meta_topic):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        (raws if topic == raw_topic else metas)[stamp] = msg
        if stamp in raws and (stamp in metas or meta_topic not in types):
            meta = metas.pop(stamp, None)
            yield to_array(raws.pop(stamp)), (_as_dict(meta) if meta is not None else {}), stamp
            n += 1
            if limit and n >= limit:
                return


def summary(bag):
    reader = _open(bag)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    counts = {}
    while reader.has_next():
        topic, _d, _t = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
    for topic in sorted(counts):
        print(f"  {topic:44s} {counts[topic]:6d}  {types[topic]}")
    return [t[:-len(_META_SUFFIX)].strip("/") for t in types if t.endswith(_META_SUFFIX)]


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    bag = argv[1]
    if len(argv) == 2:
        print(f"{bag}")
        for ns in summary(bag):
            for raw, meta, stamp in frames(bag, ns, limit=1):
                print(f"\n  [{ns}] {raw.shape} {raw.dtype}  min {raw.min()} max {raw.max()} mean {raw.mean():.1f}")
                for key in ("pixel_format", "exposure_time_us", "gain_db", "black_level", "white_level_dn",
                            "device_temperature_c", "chunk_valid", "crc_ok", "serial", "hdr_output"):
                    if key in meta:
                        print(f"    {key}: {meta[key]}")
        return 0
    for raw, meta, stamp in frames(bag, argv[2], limit=int(argv[3]) if len(argv) > 3 else 3):
        print(stamp, raw.shape, raw.dtype, f"mean {raw.mean():.1f}", "exp", meta.get("exposure_time_us"),
              "frame", meta.get("camera_frame_id"), "settled", meta.get("settled", "-"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
