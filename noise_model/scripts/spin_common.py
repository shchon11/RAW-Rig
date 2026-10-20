"""Minimal ctypes bridge to the Spinnaker C API for the BFS (IMX392) PTC capture.

Signatures follow RAW-Rig rawrig/spinnaker.py (SpinnakerC.h, SDK 4.x); no PySpin needed.
Every node written through SpinSettings is restored (reverse order) by restore().
Run without the Arena LD_LIBRARY_PATH (env.sh) so the two GenICam stacks never mix.
"""
import ctypes as C
import time

import numpy as np

H, SZ, I64, U64, BOOL, DOUBLE = C.c_void_p, C.c_size_t, C.c_int64, C.c_uint64, C.c_uint8, C.c_double
P = C.POINTER
TIMEOUT = -1011

SIGNATURES = {
    "spinErrorGetLastMessage": [C.c_char_p, P(SZ)],
    "spinSystemGetInstance": [P(H)], "spinSystemReleaseInstance": [H],
    "spinSystemGetCameras": [H, H], "spinCameraListCreateEmpty": [P(H)],
    "spinCameraListDestroy": [H], "spinCameraListClear": [H],
    "spinCameraListGetBySerial": [H, C.c_char_p, P(H)], "spinCameraRelease": [H],
    "spinCameraInit": [H], "spinCameraDeInit": [H],
    "spinCameraBeginAcquisition": [H], "spinCameraEndAcquisition": [H],
    "spinCameraGetNodeMap": [H, P(H)], "spinCameraGetTLStreamNodeMap": [H, P(H)],
    "spinCameraGetNextImageEx": [H, U64, P(H)],
    "spinNodeMapGetNode": [H, C.c_char_p, P(H)],
    "spinNodeIsReadable": [H, P(BOOL)], "spinNodeIsWritable": [H, P(BOOL)],
    "spinNodeIsAvailable": [H, P(BOOL)], "spinNodeToString": [H, C.c_char_p, P(SZ)],
    "spinEnumerationGetEntryByName": [H, C.c_char_p, P(H)],
    "spinEnumerationEntryGetIntValue": [H, P(I64)],
    "spinEnumerationReleaseNode": [H, H], "spinEnumerationSetIntValue": [H, I64],
    "spinFloatSetValue": [H, DOUBLE], "spinFloatGetValue": [H, P(DOUBLE)],
    "spinFloatGetMin": [H, P(DOUBLE)], "spinFloatGetMax": [H, P(DOUBLE)],
    "spinIntegerSetValue": [H, I64], "spinIntegerGetValue": [H, P(I64)],
    "spinIntegerGetMin": [H, P(I64)], "spinIntegerGetMax": [H, P(I64)],
    "spinBooleanSetValue": [H, BOOL], "spinBooleanGetValue": [H, P(BOOL)],
    "spinImageRelease": [H], "spinImageGetData": [H, P(H)],
    "spinImageGetWidth": [H, P(SZ)], "spinImageGetHeight": [H, P(SZ)], "spinImageGetStride": [H, P(SZ)],
    "spinImageGetFrameID": [H, P(U64)], "spinImageGetTimeStamp": [H, P(U64)],
    "spinImageGetPixelFormatName": [H, C.c_char_p, P(SZ)],
    "spinImageIsIncomplete": [H, P(BOOL)], "spinImageHasCRC": [H, P(BOOL)], "spinImageCheckCRC": [H, P(BOOL)],
    "spinImageChunkDataGetFloatValue": [H, C.c_char_p, P(DOUBLE)],
}


class SpinError(RuntimeError):
    def __init__(self, name, code, msg):
        self.code = code
        super().__init__(f"{name}: {msg} (SDK {code})")


class Spin:
    def __init__(self, path="/opt/spinnaker/lib/libSpinnaker_C.so"):
        self.lib = C.CDLL(path)
        for name, args in SIGNATURES.items():
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = args, C.c_int

    def call(self, name, *args):
        code = getattr(self.lib, name)(*args)
        if code:
            buf, size = C.create_string_buffer(4096), SZ(4096)
            self.lib.spinErrorGetLastMessage(buf, C.byref(size))
            raise SpinError(name, code, buf.value.decode("utf-8", "replace"))

    def value(self, name, handle, dtype, *args):
        out = dtype()
        self.call(name, handle, *args, C.byref(out))
        return out.value

    def handle(self, name, *args):
        h = H()
        self.call(name, *args, C.byref(h))
        return h

    def string(self, name, handle):
        buf, size = C.create_string_buffer(16384), SZ(16384)
        self.call(name, handle, buf, C.byref(size))
        return buf.value.decode("utf-8", "replace")


class NodeMap:
    """Typed access. kind: 'enum' | 'float' | 'int' | 'bool'."""

    def __init__(self, spin, handle):
        self.s, self.h, self.cache = spin, handle, {}

    def node(self, name):
        if name not in self.cache:
            self.cache[name] = self.s.handle("spinNodeMapGetNode", self.h, name.encode())
        return self.cache[name]

    def has(self, name):
        try:
            n = self.node(name)
            return bool(n.value and self.s.value("spinNodeIsAvailable", n, BOOL))
        except SpinError:
            return False

    def readable(self, name):
        return self.has(name) and bool(self.s.value("spinNodeIsReadable", self.node(name), BOOL))

    def writable(self, name):
        return self.has(name) and bool(self.s.value("spinNodeIsWritable", self.node(name), BOOL))

    def get(self, name, kind):
        n = self.node(name)
        if kind == "enum":
            return self.s.string("spinNodeToString", n)
        if kind == "float":
            return self.s.value("spinFloatGetValue", n, DOUBLE)
        if kind == "int":
            return self.s.value("spinIntegerGetValue", n, I64)
        return bool(self.s.value("spinBooleanGetValue", n, BOOL))

    def set(self, name, kind, value):
        n = self.node(name)
        if kind == "enum":
            entry = self.s.handle("spinEnumerationGetEntryByName", n, str(value).encode())
            try:
                self.s.call("spinEnumerationSetIntValue", n, self.s.value("spinEnumerationEntryGetIntValue", entry, I64))
            finally:
                self.s.call("spinEnumerationReleaseNode", n, entry)
        elif kind == "float":
            self.s.call("spinFloatSetValue", n, float(value))
        elif kind == "int":
            self.s.call("spinIntegerSetValue", n, int(value))
        else:
            self.s.call("spinBooleanSetValue", n, bool(value))

    def bounds(self, name, kind="float"):
        k, dt = ("Integer", I64) if kind == "int" else ("Float", DOUBLE)
        n = self.node(name)
        return self.s.value(f"spin{k}GetMin", n, dt), self.s.value(f"spin{k}GetMax", n, dt)


class SpinSettings:
    def __init__(self, nm):
        self.nm, self.saved, self.log = nm, [], []

    def read(self, name, kind="float"):
        try:
            return self.nm.get(name, kind) if self.nm.readable(name) else None
        except SpinError:
            return None

    def write(self, name, kind, value, required=False, selector=None):
        """selector=(selector_node, entry): write `name` under that selector entry."""
        ok, err = False, None
        try:
            if selector:
                sel, entry = selector
                if not any(s[0] == sel and s[3] is None for s in self.saved):
                    self.saved.append((sel, "enum", self.nm.get(sel, "enum"), None))
                self.nm.set(sel, "enum", entry)
            if not self.nm.writable(name):
                err = "missing" if not self.nm.has(name) else "not writable"
            else:
                if selector or not any(s[0] == name and s[3] is None for s in self.saved):
                    self.saved.append((name, kind, self.nm.get(name, kind), selector))
                self.nm.set(name, kind, value)
                ok = True
        except SpinError as e:
            err = str(e)
        tag = f"{name}[{selector[0]}={selector[1]}]" if selector else name
        self.log.append({"node": tag, "value": value, "ok": ok, "error": err, "readback": self.read(name, kind)})
        if required and not ok:
            raise RuntimeError(f"{tag}={value} failed: {err}")
        return ok

    def restore(self):
        errors = []
        for name, kind, value, selector in reversed(self.saved):
            try:
                if selector:
                    self.nm.set(selector[0], "enum", selector[1])
                if self.nm.writable(name):
                    self.nm.set(name, kind, value)
            except SpinError as e:
                errors.append(f"{name}: {e}")
        self.saved.clear()
        return errors


class BFS:
    """One Spinnaker GigE camera by serial, with the grab interface ptc_capture expects."""

    def __init__(self, serial):
        self.s = Spin()
        self.system = self.s.handle("spinSystemGetInstance")
        self.cams = self.s.handle("spinCameraListCreateEmpty")
        self.s.call("spinSystemGetCameras", self.system, self.cams)
        self.cam = self.s.handle("spinCameraListGetBySerial", self.cams, str(serial).encode())
        self.s.call("spinCameraInit", self.cam)
        self.nm = NodeMap(self.s, self.s.handle("spinCameraGetNodeMap", self.cam))
        self.stream = NodeMap(self.s, self.s.handle("spinCameraGetTLStreamNodeMap", self.cam))
        self.on = False

    def start(self):
        if not self.on:
            self.s.call("spinCameraBeginAcquisition", self.cam)
            self.on = True

    def stop(self):
        if self.on:
            self.s.call("spinCameraEndAcquisition", self.cam)
            self.on = False

    def _next(self, timeout_ms):
        deadline = time.monotonic() + timeout_ms / 1000
        while True:
            try:
                return self.s.handle("spinCameraGetNextImageEx", self.cam, 200)
            except SpinError as e:
                if e.code != TIMEOUT or time.monotonic() > deadline:
                    raise

    def grab(self, count, timeout_ms=3000, expect_exposure_us=None, settle=2, max_skip=10):
        frames, metas, skipped = [], [], 0
        settle_left = settle if expect_exposure_us is not None else 0
        while len(frames) < count:
            img = self._next(timeout_ms)
            try:
                s = self.s
                meta = {"frame_id": s.value("spinImageGetFrameID", img, U64),
                        "timestamp_ns": s.value("spinImageGetTimeStamp", img, U64),
                        "incomplete": bool(s.value("spinImageIsIncomplete", img, BOOL)),
                        "pixel_format": s.string("spinImageGetPixelFormatName", img)}
                for chunk in ("ExposureTime", "Gain", "BlackLevel"):
                    v = DOUBLE()
                    try:
                        s.call("spinImageChunkDataGetFloatValue", img, f"Chunk{chunk}".encode(), C.byref(v))
                        meta[f"Chunk{chunk}"] = v.value
                    except SpinError:
                        pass
                has_crc = bool(s.value("spinImageHasCRC", img, BOOL))
                meta["crc_ok"] = bool(s.value("spinImageCheckCRC", img, BOOL)) if has_crc else None
                bad = meta["incomplete"] or meta["crc_ok"] is False
                if expect_exposure_us is not None and meta.get("ChunkExposureTime") is not None:
                    bad |= abs(meta["ChunkExposureTime"] - expect_exposure_us) > max(1.0, 1e-3 * expect_exposure_us)
                if not bad and settle_left > 0:
                    settle_left -= 1
                    bad = True
                if bad:
                    skipped += 1
                    if skipped > max_skip + count:
                        raise RuntimeError(f"too many rejected frames: last {meta}")
                    continue
                w, h = s.value("spinImageGetWidth", img, SZ), s.value("spinImageGetHeight", img, SZ)
                stride = s.value("spinImageGetStride", img, SZ)
                data = s.handle("spinImageGetData", img)
                view = (C.c_uint8 * (stride * h)).from_address(data.value)
                px = np.ndarray((h, w), dtype="<u2", buffer=view, strides=(stride, 2)).astype(np.uint32)
            finally:
                self.s.call("spinImageRelease", img)
            frames.append(px)
            metas.append(meta)
        return np.stack(frames), metas, skipped

    def close(self):
        try:
            self.stop()
        finally:
            self.s.call("spinCameraDeInit", self.cam)
            self.s.call("spinCameraRelease", self.cam)
            self.s.call("spinCameraListClear", self.cams)
            self.s.call("spinCameraListDestroy", self.cams)
            self.s.call("spinSystemReleaseInstance", self.system)
