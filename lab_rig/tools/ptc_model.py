"""PTC 노이즈 모델 (jsh-16, ~/jsh/imx490_ptc/calibration) — 촬영 도구가 쓰는 부분.

2026-10-20 jsh-16 과 합의한 대로:
- 녹화 조건 = 측정 때 실제로 쓴 설정 (REQUIRED). 촬영 전에 대조하고, 다르면 막는다.
  dark offset · 픽셀 지도는 이 조건에서만 맞는다.
- 유효 범위: 노출 BFS 24 µs~1 s, LUCID 86 µs~1.0 s / 게인 BFS {0,6,10,12,18,24} dB, LUCID 0 dB 고정
  (LUCID 게인은 HDR 합성 뒤 디지털이라 SNR 이득이 없다) / 온도 측정 범위 ±3 °C / 워밍업 30분.
- 프레임별 값: 그 게인 · 노출의 CFA 별 α, read σ, dark offset · 분산(모델 지도의 CFA 평균), row σ, white, 코드 간격.
- DNG NoiseProfile (BFS 만): S = α/(white−black), O = dark 분산(T)/(white−black)², black = 모델 dark offset.
  LUCID HDR 은 비운다 (상수 α 하나로는 SP1 구간 ~4×10⁴ DN 까지만 맞는다).
"""
import hashlib
import json
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np

CAL = Path.home() / "jsh/imx490_ptc/calibration"
sys.path.insert(0, str(Path.home() / "jsh/imx490_ptc/scripts"))
from noise_model import NoiseModel  # noqa: E402

PACKAGES = {"bfs": CAL / "BFS-PGE-23S3C_26315393", "lucid": CAL / "TRI054S-C_254600231"}
CFA = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}
# 노출 유효 범위: 모델 파일의 valid_exposure_us (LUCID [86, 800000] — 1 s 는 검은 레벨이 두 상태를 오가서
# 2026-10-20 에 800 ms 로 내림), 없으면 jsh-16 합의값 (BFS 24 µs ~ 1 s, 전체 화면 dark 최대 1 s).
_FALLBACK_EXPOSURE_US = {"bfs": (24.0, 1.0e6), "lucid": (86.0, 8.0e5)}


def _valid_exposure(key):
    try:
        doc = json.loads((PACKAGES[key] / "noise_model.json").read_text())
        lo, hi = doc.get("valid_exposure_us") or _FALLBACK_EXPOSURE_US[key]
        return float(lo), float(hi)
    except (OSError, ValueError, TypeError):
        return _FALLBACK_EXPOSURE_US[key]


EXPOSURE_US = {k: _valid_exposure(k) for k in PACKAGES}
GAINS_DB = {"bfs": (0.0, 6.0, 10.0, 12.0, 18.0, 24.0), "lucid": (0.0,)}
TEMPERATURE_C = {"bfs": (53.0, 60.0), "lucid": (44.0, 54.0)}
WARMUP_S = 30 * 60
DARK_BEFORE_FROM_GAIN_DB = 18.0      # BFS 게인이 이 이상이면 dark 를 bright 앞뒤로 (offset 흔들림 0.1σ 기준)
SHIFT_WARN_SIGMA = 0.1               # 세션 offset 이동량 경고 (read σ 대비)
# LUCID 를 트리거로 찍는 노출 상한. 그 위는 free-run (동시 아님): 트리거 모드의 LUCID dark 분산이 300 ms 부터
# 모델보다 +14 % (60/100/150 ms 는 1.03/1.04/1.05). 2026-10-20 jsh-16 과 합의.
LUCID_TRIGGER_MAX_US = 150_000.0
LUCID_HDR_TRANSITIONS_0DB = {         # G 기준 대략값, 2×2 쿼드 단위로 판정되는 것으로 보임 (jsh-16)
    "sp1_to_sp2_dn": (4.0e4, 7.8e4), "ch2_to_ch3_dn": (2.8e5, 5.5e5),
    "excess_noise": {"sp1_to_sp2": 0.17, "ch2_to_ch3": 0.45},
    "note": "approximate; quad-based (R switches at lower R than G); scale DN by 10^(gain_dB/20)"}
NOTES = {
    "bfs": "constant alpha = per-gain WLS fit; local alpha -1.2 % at low signal, +1.5 % at 1e4-2.5e4 DN "
           "(alpha(S) table in the package)",
    "lucid": "HDR combined output: no standard NoiseProfile; use the package alpha(S) table. "
             "Noise near the channel transitions is 17-45 % above the model (mixture not modelled).",
}


class Req:
    """녹화 조건 하나. meta: 프레임 메타데이터 필드 / param: 드라이버 파라미터 (읽기, 고치기)."""

    def __init__(self, label, want, meta=None, param=None, fix=None, tol=None):
        self.label, self.want, self.meta, self.param, self.tol = label, want, meta, param, tol
        self.fix = fix if fix is not None else ({param: want} if param else None)

    def matches(self, have):
        if have is None:
            return None
        if isinstance(self.want, (list, tuple)):
            return len(have) == len(self.want) and all(abs(float(h) - float(w)) <= (self.tol or 0.05)
                                                       for h, w in zip(have, self.want))
        if isinstance(self.want, bool):
            return bool(have) == self.want
        if isinstance(self.want, (int, float)):
            try:
                return abs(float(have) - float(self.want)) <= (self.tol if self.tol is not None else 1e-3)
            except (TypeError, ValueError):
                return False
        return str(have) == str(self.want)


# jsh-16 답 1번 (측정 때 실제로 쓴 값). fix=False: 도구가 못 고친다 (드라이버 재시작 · 하드웨어).
REQUIRED = {
    "bfs": [
        Req("PixelFormat", "BayerRG16", meta="pixel_format", fix=False),
        Req("AdcBitDepth", "Bit12", meta="adc_bit_depth", param="camera.AdcBitDepth"),
        Req("BlackLevel (All) %", 1.0, meta="black_level", fix={"raw.black_level_percent": 1.0}, tol=0.01),
        Req("BlackLevelClampingEnable", False, meta="black_level_clamping_enable",
            param="camera.BlackLevelClampingEnable"),
        Req("BalanceWhiteAuto", "Off", meta="balance_white_auto", param="camera.BalanceWhiteAuto"),
        Req("BalanceRatio Red", 1.0, meta="balance_ratio_red", fix={"raw.balance_ratio_red": 1.0}, tol=0.01),
        Req("BalanceRatio Blue", 1.0, meta="balance_ratio_blue", fix={"raw.balance_ratio_blue": 1.0}, tol=0.01),
        Req("GammaEnable", False, meta="gamma_enable", param="camera.GammaEnable"),
        Req("LUTEnable", False, meta="lut_enable", param="camera.LUTEnable"),
        Req("DefectCorrectStaticEnable", False, meta="defect_correct_static_enable",
            param="camera.DefectCorrectStaticEnable"),
        Req("IspEnable", False, meta="isp_enable", param="camera.IspEnable"),
        Req("ExposureAuto", "Off", meta="exposure_auto", param="camera.ExposureAuto"),
        Req("GainAuto", "Off", meta="gain_auto", param="camera.GainAuto"),
        Req("ReverseX", False, param="camera.ReverseX"),
        Req("ReverseY", False, param="camera.ReverseY"),
        Req("Width", 1920, meta="width", fix=False),
        Req("Height", 1200, meta="height", fix=False),
        Req("OffsetX", 0, param="camera.OffsetX"),
        Req("OffsetY", 0, param="camera.OffsetY"),
        Req("BinningHorizontal", 1, param="camera.BinningHorizontal"),
        Req("BinningVertical", 1, param="camera.BinningVertical"),
        Req("DecimationHorizontal", 1, param="camera.DecimationHorizontal"),
        Req("DecimationVertical", 1, param="camera.DecimationVertical"),
        Req("Firmware", "2103.0.335.0", meta="firmware_version", fix=False),
    ],
    "lucid": [
        Req("PixelFormat", "BayerRG24", meta="pixel_format", fix={"pixel_format": "BayerRG24"}),
        Req("HDROutput", "HDR", meta="hdr_output", param="camera.HDROutput"),
        Req("BlackLevelRaw (All)", 256, meta="black_level_raw", fix={"raw.black_level_raw": 256}),
        Req("Gain", 0.0, meta="gain_db", param="camera.Gain", fix={"camera.Gain": 0.0}, tol=0.05),
        Req("BalanceWhiteAuto", "Off", meta="balance_white_auto", param="camera.BalanceWhiteAuto"),
        Req("BalanceWhiteEnable", False, meta="balance_white_enable", param="camera.BalanceWhiteEnable"),
        Req("ColorTransformationEnable", False, meta="color_transformation_enable",
            param="camera.ColorTransformationEnable"),
        Req("LUTEnable", False, meta="lut_enable", param="camera.LUTEnable"),
        Req("HDRImageEnhancementEnable", False, meta="hdr_image_enhancement_enable",
            param="camera.HDRImageEnhancementEnable"),
        Req("HDRDigitalClampingEnable", False, meta="hdr_digital_clamping_enable",
            param="camera.HDRDigitalClampingEnable"),
        Req("HDRTuningEnable", False, meta="hdr_tuning_enable", param="camera.HDRTuningEnable"),
        Req("HDR channel analog gain dB", (19.8, 4.2, 29.4, 4.2), meta="hdr_channel_analog_gain_db", fix=False,
            tol=0.05),
        Req("ADCBitDepth", "Bits12", meta="adc_bit_depth", param="camera.ADCBitDepth"),
        Req("SensorShutterMode", "Rolling", meta="sensor_shutter_mode", param="camera.SensorShutterMode"),
        Req("ExposureAuto", "Off", meta="exposure_auto", param="camera.ExposureAuto"),
        Req("GainAuto", "Off", meta="gain_auto", param="camera.GainAuto"),
        Req("ReverseX", False, meta="reverse_x", param="camera.ReverseX"),
        Req("ReverseY", False, meta="reverse_y", param="camera.ReverseY"),
        Req("Width", 2880, meta="width", fix=False),
        Req("Height", 1860, meta="height", fix=False),
        Req("BinningHorizontal", 1, param="camera.BinningHorizontal"),
        Req("BinningVertical", 1, param="camera.BinningVertical"),
        Req("Firmware", "1.138.0.0", meta="firmware_version", fix=False),
    ],
}


def check(key, meta, params, gain_db=None):
    """녹화 조건 대조 → (틀림, 모름, 경고) 목록. 틀림/모름 항목: (label, have, want, fix dict 또는 None)."""
    wrong, unknown, warnings = [], [], []
    for req in REQUIRED[key]:
        have = meta.get(req.meta) if req.meta else None
        if have is None and req.param:
            have = params.get(req.param)
        ok = req.matches(have)
        fix = req.fix if req.fix not in (False, None) else None
        if ok is None:
            unknown.append((req.label, None, req.want, fix))
        elif not ok:
            wrong.append((req.label, have, req.want, fix))
    gain = meta.get("gain_db")
    allowed = GAINS_DB[key]
    if key == "bfs" and gain is not None and not any(abs(gain - g) <= 0.05 for g in allowed):
        target = gain_db if gain_db is not None else 0.0
        wrong.append(("Gain (모델에 있는 값)", gain, list(allowed), {"camera.Gain": float(target)}))
    elif key == "bfs" and gain_db is not None and gain is not None and abs(gain - gain_db) > 0.05:
        wrong.append(("Gain (고른 값)", gain, gain_db, {"camera.Gain": float(gain_db)}))
    temp = meta.get("device_temperature_c")
    lo, hi = TEMPERATURE_C[key]
    if temp is not None and temp == temp and not lo <= temp <= hi:
        warnings.append(f"온도 {temp:.1f} °C — 모델은 {lo:.0f}~{hi:.0f} °C 에서 쟀다 (dark current 가 다르다, 노출 ≥100 ms 에서 의미)")
    uptime = meta.get("device_uptime_s")
    if uptime is not None and uptime == uptime and uptime < WARMUP_S:
        warnings.append(f"켠 지 {uptime / 60:.0f}분 — 워밍업 30분 권장")
    return wrong, unknown, warnings


@lru_cache(maxsize=None)
def model(key):
    return NoiseModel(PACKAGES[key])


@lru_cache(maxsize=None)
def reference(key):
    path = PACKAGES[key] / "noise_model.json"
    doc = json.loads(path.read_text())
    return {"package": str(PACKAGES[key]), "noise_model_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "built": doc.get("built"), "settings": doc.get("settings"), "code": str(CAL.parent / "scripts/noise_model.py")}


@lru_cache(maxsize=64)
def frame_params(key, gain_db, exposure_us):
    """그 게인 · 노출의 스칼라 값 (CFA 별). exposure_us 는 반올림해서 캐시 키로 쓴다."""
    nm = model(key)
    g0, k = nm._gain(gain_db)
    doc = nm.doc["gains"][str(g0)]
    offset, dvar = nm.dark(gain_db, exposure_us)
    out = {"gain_db": gain_db, "exposure_us": exposure_us, "white_dn": nm.white, "code_step_dn": nm.step,
           "code_phase_dn": nm.phase, "row_sigma_dn": nm.row_sigma(gain_db, exposure_us), "cfa": {}}
    for c, (i, j) in CFA.items():
        out["cfa"][c] = {"alpha_dn_per_e": float(doc["alpha_dn_per_e"][c]) * k,
                         "read_sigma_dn": float(doc["read_noise_dn"][c]) * k,
                         "dark_offset_dn": float(np.mean(offset[i::2, j::2])),
                         "dark_var_dn2": float(np.mean(dvar[i::2, j::2]))}
    if key == "lucid":
        scale = 10 ** (gain_db / 20)
        t = LUCID_HDR_TRANSITIONS_0DB
        out["hdr_transitions"] = {"sp1_to_sp2_dn": [v * scale for v in t["sp1_to_sp2_dn"]],
                                  "ch2_to_ch3_dn": [v * scale for v in t["ch2_to_ch3_dn"]],
                                  "excess_noise": t["excess_noise"], "note": t["note"]}
    out["note"] = NOTES[key]
    return out


def dng_black_white(key, params):
    return {c: v["dark_offset_dn"] for c, v in params["cfa"].items()}, int(params["white_dn"])


def noise_profile(key, params):
    """DNG NoiseProfile [S_R, O_R, S_G, O_G, S_B, O_B] — BFS 만, LUCID HDR 은 None."""
    if key != "bfs":
        return None
    white = params["white_dn"]
    out = []
    for plane in ("R", "G", "B"):
        sites = [c for c in params["cfa"] if c.rstrip("12") == plane]
        alpha = np.mean([params["cfa"][c]["alpha_dn_per_e"] for c in sites])
        var = np.mean([params["cfa"][c]["dark_var_dn2"] for c in sites])
        black = np.mean([params["cfa"][c]["dark_offset_dn"] for c in sites])
        span = white - black
        out += [float(alpha / span), float(var / span ** 2)]
    return out


def clip_exposure(exposure_us, key):
    return float(np.clip(exposure_us, *EXPOSURE_US[key]))
