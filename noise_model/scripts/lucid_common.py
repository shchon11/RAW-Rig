"""Shared helpers for LUCID TRI054S (IMX490) RAW measurements via arena_api.

Every node written through `Settings` is restored (reverse order) by `Settings.restore()`.
"""
import time

import numpy as np

CFA = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}  # IspBayerPattern = RGGB

# ISP stages that must not touch RAW. Order matters (auto before enable).
RAW_ISP_OFF = (
    ("ExposureAuto", "Off"),
    ("GainAuto", "Off"),
    ("BalanceWhiteAuto", "Off"),
    ("BalanceWhiteEnable", False),
    ("ColorTransformationEnable", False),
    ("LUTEnable", False),
    ("HDRImageEnhancementEnable", False),
    ("HDRDigitalClampingEnable", False),
    ("AcquisitionFrameRateEnable", False),
    ("ReverseX", False),
    ("ReverseY", False),
)
CHUNKS = ("CRC", "Timestamp", "ExposureTime", "Gain", "BlackLevel", "PixelFormat")


def open_lucid(serial=None):
    from arena_api.system import system
    infos = [i for i in system.device_infos if str(i.get("vendor", "")).startswith("Lucid")]
    if serial:
        infos = [i for i in infos if str(i.get("serial")) == str(serial)]
    if not infos:
        raise SystemExit("LUCID camera not found (check IP: scripts/gvcp_ip.py list)")
    device = system.create_device(device_infos=[infos[0]])[0]
    return system, device, infos[0]


class Settings:
    """Write nodes while remembering the first-seen value of each, for restore."""

    def __init__(self, nodemap):
        self.nm = nodemap
        self.saved = []          # [(name, original, selector_context)]
        self.log = []

    def node(self, name):
        try:
            return self.nm.get_node(name)
        except Exception:
            return None

    def read(self, name):
        n = self.node(name)
        try:
            return n.value if n is not None and n.is_readable else None
        except Exception:
            return None

    def write(self, name, value, required=False):
        n = self.node(name)
        ok, err = False, None
        if n is None or not n.is_writable:
            err = "missing" if n is None else "not writable"
        else:
            try:
                if not any(s[0] == name and s[2] is None for s in self.saved):
                    self.saved.append((name, n.value, None))
                n.value = value
                ok = True
            except Exception as e:
                err = str(e)
        self.log.append({"node": name, "value": value, "ok": ok, "error": err, "readback": self.read(name)})
        if required and not ok:
            raise RuntimeError(f"{name}={value} failed: {err}")
        return ok

    def write_selected(self, selector, entry, name, value):
        """Write `name` under selector=entry; restore both later."""
        sel = self.node(selector)
        if sel is None or not sel.is_writable:
            return False
        if not any(s[0] == selector and s[2] is None for s in self.saved):
            self.saved.append((selector, sel.value, None))
        sel.value = entry
        n = self.node(name)
        if n is None or not n.is_writable:
            return False
        self.saved.append((name, n.value, (selector, entry)))
        n.value = value
        self.log.append({"node": f"{name}[{selector}={entry}]", "value": value, "ok": True, "readback": n.value})
        return True

    def restore(self):
        errors = []
        for name, value, ctx in reversed(self.saved):
            try:
                if ctx is not None:
                    self.nm.get_node(ctx[0]).value = ctx[1]
                self.nm.get_node(name).value = value
            except Exception as e:
                errors.append(f"{name}: {e}")
        self.saved.clear()
        return errors


def setup_stream(device):
    s = device.tl_stream_nodemap
    s["StreamAutoNegotiatePacketSize"].value = True
    s["StreamPacketResendEnable"].value = True
    s["StreamBufferHandlingMode"].value = "OldestFirst"


def enable_chunks(st, names=CHUNKS):
    st.write("ChunkModeActive", True)
    for c in names:
        try:
            st.write_selected("ChunkSelector", c, "ChunkEnable", True)
        except Exception as e:
            st.log.append({"node": f"ChunkEnable[{c}]", "ok": False, "error": str(e)})


def decode(buf):
    """Image part of a buffer -> uint32 (h, w). Chunk data appended after the image is ignored."""
    h, w, bits = buf.height, buf.width, buf.bits_per_pixel
    nbytes = h * w * bits // 8
    raw = np.ctypeslib.as_array(buf.pdata, (nbytes,)).copy()
    big = "BIG" in str(buf.pixel_endianness).upper()
    if bits == 8:
        return raw.reshape(h, w).astype(np.uint32)
    if bits == 16:
        return raw.view(">u2" if big else "<u2").reshape(h, w).astype(np.uint32)
    if bits == 24:
        b = raw.reshape(h, w, 3).astype(np.uint32)
        return (b[..., 0] << 16 | b[..., 1] << 8 | b[..., 2]) if big else (b[..., 0] | b[..., 1] << 8 | b[..., 2] << 16)
    if bits == 12:
        b = raw.reshape(-1, 3).astype(np.uint32)
        p0 = b[:, 0] | (b[:, 1] & 0x0F) << 8
        p1 = b[:, 1] >> 4 | b[:, 2] << 4
        return np.stack([p0, p1], 1).reshape(h, w)
    raise ValueError(f"unsupported bits_per_pixel {bits}")


def chunk_meta(buf, names=CHUNKS):
    meta = {"frame_id": buf.frame_id, "timestamp_ns": buf.timestamp_ns, "incomplete": buf.is_incomplete,
            "pixel_format": str(buf.pixel_format).split(".")[-1], "bits": buf.bits_per_pixel}
    try:
        if buf.has_chunkdata:
            got = buf.get_chunk([f"Chunk{c}" for c in names if c != "CRC"])
            for k, n in got.items():
                meta[k] = n.value if isinstance(n.value, (int, float, bool)) else str(n.value)
            meta["crc_ok"] = bool(buf.is_valid_crc) if "CRC" in names else None
    except Exception as e:
        meta["chunk_error"] = str(e)
    return meta


def grab(device, count, timeout_ms=3000, roi=None, expect_exposure_us=None, max_skip=10, settle=2):
    """Grab `count` complete frames. If expect_exposure_us is set, skip frames whose chunk
    exposure differs (settings pipeline after a change), then `settle` more: the chunk value
    switches one frame before the image does (measured 2026-10-19). Returns (stack, metas, skipped)."""
    frames, metas, skipped = [], [], 0
    settle_left = settle if expect_exposure_us is not None else 0
    while len(frames) < count:
        buf = device.get_buffer(timeout=timeout_ms)
        try:
            meta = chunk_meta(buf)
            bad = meta["incomplete"] or meta.get("crc_ok") is False
            if expect_exposure_us is not None and "ChunkExposureTime" in meta:
                bad |= abs(meta["ChunkExposureTime"] - expect_exposure_us) > max(1.0, 1e-3 * expect_exposure_us)
            if not bad and settle_left > 0:
                settle_left -= 1
                bad = True
            if bad:
                skipped += 1
                if skipped > max_skip + count:
                    raise RuntimeError(f"too many rejected frames: last {meta}")
                continue
            img = decode(buf)
        finally:
            device.requeue_buffer(buf)
        if roi is not None:
            y0, x0, s = roi
            img = img[y0:y0 + s, x0:x0 + s]
        frames.append(img)
        metas.append(meta)
    return np.stack(frames), metas, skipped


def center_roi(h, w, size):
    s = size - size % 2
    return ((h - s) // 2 & ~1, (w - s) // 2 & ~1, s)


def cfa_stats(stack):
    """Per-CFA temporal/spatial stats of a dark or flat stack [n, h, w] (n even)."""
    n = stack.shape[0] - stack.shape[0] % 2
    out = {}
    for k, (i, j) in CFA.items():
        x = stack[:n, i::2, j::2].astype(np.float64)
        a, b = x[0::2], x[1::2]
        tvar = float(np.mean((a - b) ** 2) / 2)
        mean_img = x.mean(0)
        vals = stack[:n, i::2, j::2]
        u = np.unique(vals)
        step = int(np.gcd.reduce(np.diff(u))) if u.size > 1 else 0
        vmin = int(u[0])
        out[k] = {"mean": float(x.mean()), "temporal_var": tvar, "temporal_std": tvar ** 0.5,
                  "dsnu_std": float(np.sqrt(max(mean_img.var() - tvar / n, 0.0))),
                  "min": vmin, "max": int(u[-1]), "n_unique": int(u.size), "code_step": step,
                  "frac_at_min": float(np.mean(vals == vmin)), "frac_zero": float(np.mean(vals == 0)),
                  "code_phase": int(u[0] % step) if step else None}
    return out
