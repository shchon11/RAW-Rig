"""RAW 프레임 → 무손실 DNG, 그리고 되읽기 — 랩 RAW 리그의 저장 형식.

    BFS (BayerRG16, 12-bit 코드 << 4)       → uint16 DNG
    LUCID 채널 출력 HDRChannelN (값 ≤ 4351) → uint16 DNG
    LUCID HDR 합성 (BayerRG24, 값 < 2^24)    → 24-bit packed DNG (BitsPerSample 24)

규칙
- 파일은 빅엔디안(MM)이다. DNG 규격은 8/16/32 가 아닌 비트수를 리틀엔디안 파일에서도 빅엔디안으로 채우라고 하는데,
  일반 TIFF 리더(tifffile 등)는 리틀엔디안 파일이면 그 바이트를 뒤집어 읽는다. 빅엔디안 파일이면 둘이 같다.
- 압축하지 않는다 (24-bit 정수는 규격상 무압축만 된다. 16-bit 도 같은 방식으로 맞췄다).
- 노이즈: 2-파라미터 모델(분산 = α·S + σ²)이 맞는 출력은 표준 NoiseProfile 태그에 넣는다.
  프레임 메타데이터 · 캘리브레이션 원값 · HDR 합성의 구간별 노이즈 표는 DNGPrivateData 에 json 으로 넣고,
  같은 json 을 옆 파일(<이름>.json)로도 쓴다.
- 쓰고 나서 다시 읽어 픽셀이 같은지 확인한다. 픽셀 sha256 을 메타데이터에 넣어 나중에도 확인할 수 있다.

    from rawdng import write, read
    write("x.dng", raw, cfa="RGGB", black={"R": 633.0, ...}, white=65516, exposure_s=0.02,
          make="FLIR", model="BFS-PGE-23S3C", serial="26315393", meta={...})
    raw, info = read("x.dng")       # raw: uint16 / uint32 ndarray, info["meta"]: 같이 넣은 json

읽기는 tifffile 로 된다. rawpy(LibRaw) · darktable(rawspeed) 는 16-bit 를 넘는 정수 DNG 를 열지 못한다.
"""
import datetime as _dt
import hashlib
import json
import struct
from pathlib import Path

import numpy as np

PRIVATE_TAG = b"jsh-raw-lab\0"
CFA_CODES = {"RGGB": (0, 1, 1, 2), "BGGR": (2, 1, 1, 0), "GRBG": (1, 0, 2, 1), "GBRG": (1, 2, 0, 1)}

_BYTE, _ASCII, _SHORT, _LONG, _RATIONAL, _SRATIONAL, _DOUBLE = 1, 2, 3, 4, 5, 10, 12
_FMT = {_BYTE: "B", _SHORT: "H", _LONG: "I", _RATIONAL: "I", _SRATIONAL: "i", _DOUBLE: "d"}


def cfa_sites(cfa):
    """2×2 반복 패턴의 자리 이름 (행 우선): RGGB → ['R', 'G1', 'G2', 'B']. G 는 먼저 나오는 쪽이 G1."""
    names, g = [], 0
    for code in CFA_CODES[cfa.upper()]:
        if code == 1:
            g += 1
            names.append(f"G{g}")
        else:
            names.append("RGB"[code])
    return names


def pixel_sha256(raw, bps):
    """픽셀 값의 sha256 — 파일의 비트수 기준: 16-bit 이하는 리틀엔디안 uint16, 그 위는 uint32 바이트."""
    dtype = "<u2" if bps <= 16 else "<u4"
    return hashlib.sha256(np.ascontiguousarray(raw, dtype=dtype).tobytes()).hexdigest()


def noise_profile(alpha_dn, var_dn2, black, white):
    """CFA 자리별 α (DN/e⁻) · 분산 offset (DN²) → DNG NoiseProfile [S_R, O_R, S_G, O_G, S_B, O_B].

    DNG: N(x) = sqrt(S·x + O), x = (DN − black)/(white − black) ∈ [0, 1]
    분산 = α·(DN − black) + var  →  S = α/(white − black), O = var/(white − black)²
    G 는 G1 · G2 평균 (DNG NoiseProfile 은 색 평면 R/G/B 단위).
    """
    def plane(key, values):
        keys = [k for k in values if k.rstrip("12") == key]
        return float(np.mean([values[k] for k in keys]))
    span = float(white) - float(np.mean(list(black.values())))
    out = []
    for c in "RGB":
        out += [plane(c, alpha_dn) / span, plane(c, var_dn2) / span**2]
    return out


def _tag(code, typ, values):
    if typ == _ASCII:
        b = str(values).encode("utf-8") + b"\0"
        return code, typ, len(b), b
    if isinstance(values, (bytes, bytearray)):
        return code, typ, len(values), bytes(values)
    vals = list(values) if isinstance(values, (list, tuple)) else [values]
    b = struct.pack(">" + _FMT[typ] * len(vals), *vals)
    return code, typ, len(vals) // (2 if typ in (_RATIONAL, _SRATIONAL) else 1), b


def _rational(x, den=10000):
    return [int(round(float(x) * den)), den]


def _write_tiff(path, entries, strip):
    entries = sorted([e for e in entries if e[0] not in (273, 279)]
                     + [_tag(273, _LONG, 0), _tag(279, _LONG, len(strip))])
    ifd_end = 8 + 2 + 12 * len(entries) + 4
    blob, fields = bytearray(), []
    for code, typ, count, b in entries:
        if len(b) <= 4:
            fields.append((code, typ, count, b.ljust(4, b"\0")))
        else:
            blob += b"\0" * ((ifd_end + len(blob)) % 2)
            fields.append((code, typ, count, struct.pack(">I", ifd_end + len(blob))))
            blob += b
    data_off = ifd_end + len(blob)
    data_off += -data_off % 16
    head = bytearray(b"MM\0*" + struct.pack(">IH", 8, len(fields)))
    for code, typ, count, v in fields:
        head += struct.pack(">HHI", code, typ, count) + (struct.pack(">I", data_off) if code == 273 else v)
    head += struct.pack(">I", 0) + blob
    head += b"\0" * (data_off - len(head))
    tmp = Path(str(path) + ".part")
    with open(tmp, "wb") as f:
        f.write(head)
        f.write(strip)
    tmp.replace(path)


def _pack(raw, bps):
    if bps == 16:
        return np.ascontiguousarray(raw, dtype=">u2").tobytes()
    a = raw.astype(np.uint32, copy=False)
    out = np.empty(a.shape + (3,), np.uint8)
    out[..., 0] = a >> 16
    out[..., 1] = (a >> 8) & 255
    out[..., 2] = a & 255
    return out.tobytes()


def write(path, raw, *, cfa, black, white, exposure_s, make, model, serial, meta=None, noise=None,
          bps=None, f_number=None, focal_length_mm=None, taken=None, software="jsh raw_lab", verify=True,
          sidecar=True):
    """raw (H, W) uint16/uint32 → DNG. black: {"R": .., "G1": .., "G2": .., "B": ..} (DN, 소수 가능).

    noise: noise_profile() 결과 (6개) 또는 None. meta: DNGPrivateData · 옆 json 에 넣을 dict.
    bps: 16 / 24 (기본: 값 범위로 고른다). 돌려주는 dict 는 옆 json 과 같다.
    """
    path = Path(path)
    raw = np.asarray(raw)
    if raw.ndim != 2:
        raise ValueError(f"raw 는 (H, W) 여야 한다: {raw.shape}")
    vmax = int(raw.max()) if raw.size else 0
    if bps is None:
        bps = 16 if vmax < 65536 else 24
    if bps not in (16, 24) or vmax >= (1 << bps):
        raise ValueError(f"값 {vmax} 이 {bps}-bit 에 안 들어간다")
    cfa = cfa.upper()
    sites = cfa_sites(cfa)
    if set(black) != set(sites):
        raise ValueError(f"black 은 {sites} 자리마다 있어야 한다: {sorted(black)}")
    h, w = raw.shape
    taken = taken or _dt.datetime.now()
    stamp = taken.strftime("%Y:%m:%d %H:%M:%S")
    meta = dict(meta or {})
    meta.update({"pixel_sha256": pixel_sha256(raw, bps), "bits_per_sample": bps, "cfa": cfa,
                 "black_level_dn": {k: float(black[k]) for k in sites}, "white_level_dn": int(white),
                 "noise_profile": noise, "dng_writer": "jsh/tools/rawdng.py"})
    private = PRIVATE_TAG + json.dumps(meta, ensure_ascii=False, default=_json_default).encode("utf-8")
    entries = [
        _tag(254, _LONG, 0), _tag(256, _LONG, w), _tag(257, _LONG, h), _tag(258, _SHORT, bps),
        _tag(259, _SHORT, 1), _tag(262, _SHORT, 32803), _tag(271, _ASCII, make), _tag(272, _ASCII, model),
        _tag(274, _SHORT, 1), _tag(277, _SHORT, 1), _tag(278, _LONG, h), _tag(284, _SHORT, 1),
        _tag(305, _ASCII, software), _tag(306, _ASCII, stamp), _tag(339, _SHORT, 1),
        _tag(33421, _SHORT, [2, 2]), _tag(33422, _BYTE, CFA_CODES[cfa]),
        _tag(33434, _RATIONAL, _rational(exposure_s, 1_000_000)), _tag(36867, _ASCII, stamp),
        _tag(50706, _BYTE, [1, 4, 0, 0]), _tag(50707, _BYTE, [1, 1, 0, 0]),
        _tag(50708, _ASCII, f"{make} {model}"), _tag(50710, _BYTE, [0, 1, 2]), _tag(50711, _SHORT, 1),
        _tag(50713, _SHORT, [2, 2]), _tag(50714, _RATIONAL, sum((_rational(black[k]) for k in sites), [])),
        _tag(50717, _LONG, int(white)),
        # 색 행렬은 아직 캘리브레이션 전 — 단위 행렬 자리표시 (DNG 필수 태그)
        _tag(50721, _SRATIONAL, [1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1]),
        _tag(50728, _RATIONAL, [1, 1, 1, 1, 1, 1]), _tag(50735, _ASCII, serial),
        _tag(50740, _BYTE, private), _tag(50778, _SHORT, 21), _tag(50935, _RATIONAL, [0, 1]),
    ]
    if noise is not None:
        entries.append(_tag(51041, _DOUBLE, [float(v) for v in noise]))
    if f_number:
        entries.append(_tag(33437, _RATIONAL, _rational(f_number, 100)))
    if focal_length_mm:
        entries.append(_tag(37386, _RATIONAL, _rational(focal_length_mm, 100)))
    _write_tiff(path, entries, _pack(raw, bps))
    if verify:
        back, info = read(path)
        if back.shape != raw.shape or not np.array_equal(back, raw):
            raise IOError(f"{path}: 다시 읽은 픽셀이 다르다")
        if info["meta"].get("pixel_sha256") != meta["pixel_sha256"] or not info.get("pixel_ok"):
            raise IOError(f"{path}: 메타데이터 · 픽셀 해시를 다시 읽지 못했다")
        try:                # 다른 구현으로도 한 번 (있으면)
            import tifffile
        except ImportError:
            tifffile = None
        if tifffile is not None and not np.array_equal(tifffile.imread(path), raw):
            raise IOError(f"{path}: tifffile 로 다시 읽은 픽셀이 다르다")
    meta["file"] = path.name
    if sidecar:
        path.with_suffix(".json").write_text(json.dumps(meta, ensure_ascii=False, indent=1, default=_json_default))
    return meta


_SIZES = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 1, 7: 1, 8: 2, 9: 4, 10: 8, 11: 4, 12: 8}
_UNPACK = {3: "H", 4: "I", 5: "I", 6: "b", 8: "h", 9: "i", 10: "i", 11: "f", 12: "d"}


def _parse(path):
    """무압축 단일 IFD DNG (이 모듈이 쓴 파일, 그리고 같은 꼴의 파일) → (tags, raw). 의존성 없이 numpy 만 쓴다."""
    data = Path(path).read_bytes()
    e = {b"MM": ">", b"II": "<"}.get(data[:2])
    if e is None or struct.unpack(e + "H", data[2:4])[0] != 42:
        raise ValueError(f"{path}: TIFF 가 아니다")
    ifd = struct.unpack(e + "I", data[4:8])[0]
    tags = {}
    for i in range(struct.unpack(e + "H", data[ifd:ifd + 2])[0]):
        off = ifd + 2 + 12 * i
        code, typ, count = struct.unpack(e + "HHI", data[off:off + 8])
        size = _SIZES.get(typ, 1) * count
        at = off + 8 if size <= 4 else struct.unpack(e + "I", data[off + 8:off + 12])[0]
        chunk = data[at:at + size]
        if typ == 2:
            value = chunk.rstrip(b"\0").decode("utf-8", "replace")
        elif typ in (1, 7):
            value = chunk
        else:
            vals = struct.unpack(e + _UNPACK[typ] * (count * (2 if typ in (5, 10) else 1)), chunk)
            if typ in (5, 10):
                vals = tuple(vals[k] / vals[k + 1] if vals[k + 1] else float("nan") for k in range(0, len(vals), 2))
            value = vals[0] if len(vals) == 1 else vals
        tags[code] = value
    if tags.get(259, 1) != 1:
        raise ValueError(f"{path}: 압축된 DNG — tifffile 로 읽어야 한다")
    w, h, bps = tags[256], tags[257], tags[258]
    offsets, counts = tags[273], tags[279]
    offsets = offsets if isinstance(offsets, tuple) else (offsets,)
    counts = counts if isinstance(counts, tuple) else (counts,)
    pix = b"".join(data[o:o + c] for o, c in zip(offsets, counts))
    if bps == 16:
        raw = np.frombuffer(pix, e + "u2", count=w * h).reshape(h, w).astype(np.uint16)
    elif bps == 24:      # DNG: 8/16/32 가 아닌 비트수는 파일 바이트 순서와 상관없이 빅엔디안으로 채운다
        b = np.frombuffer(pix, np.uint8, count=w * h * 3).reshape(h, w, 3).astype(np.uint32)
        raw = (b[..., 0] << 16) | (b[..., 1] << 8) | b[..., 2]
    elif bps == 8:
        raw = np.frombuffer(pix, np.uint8, count=w * h).reshape(h, w).copy()
    elif bps == 32:
        raw = np.frombuffer(pix, e + "u4", count=w * h).reshape(h, w).astype(np.uint32)
    else:
        raise ValueError(f"{path}: BitsPerSample {bps} 은 못 읽는다")
    return tags, raw


def read(path):
    """DNG → (raw ndarray, info). info: 주요 표준 태그 · meta(DNGPrivateData json) · pixel_ok."""
    tags, raw = _parse(path)
    meta = {}
    private = tags.get(50740)
    if private is not None:
        private = bytes(private)
        if private.startswith(PRIVATE_TAG):
            meta = json.loads(private[len(PRIVATE_TAG):].decode("utf-8"))
    info = {
        "bits_per_sample": tags.get(258), "cfa_pattern": tags.get(33422), "black_level": tags.get(50714),
        "white_level": tags.get(50717), "exposure_time": tags.get(33434), "noise_profile": tags.get(51041),
        "serial": tags.get(50735), "model": tags.get(272), "meta": meta,
    }
    if "pixel_sha256" in meta:
        info["pixel_ok"] = pixel_sha256(raw, tags[258]) == meta["pixel_sha256"]
    return raw, info


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (Path, _dt.datetime)):
        return str(o)
    raise TypeError(f"json 으로 못 바꾼다: {type(o)}")


if __name__ == "__main__":
    import sys
    for p in sys.argv[1:]:
        r, i = read(p)
        print(p, r.shape, r.dtype, "min", int(r.min()), "max", int(r.max()), "pixel_ok", i.get("pixel_ok"),
              "bps", i["bits_per_sample"], "black", i["meta"].get("black_level_dn"),
              "white", i["white_level"], "noise", i["noise_profile"])
