"""브라케팅 촬영 창 — 장면 하나를 노출 단계별로 두 카메라 동시에 찍어서 무손실 DNG 로 저장한다.

    ~/jsh/bracket

화면 순서대로: [카메라 켜기] → PTC 조건 확인 (다르면 [조건대로 맞추기]) → 장면 이름 · 조도 · 광원 → [이 장면 찍기].
노출은 촬영 시작 때 자동으로 정한다: 두 카메라를 같은 노출로 찍고, 먼저 포화되는 쪽의 가장 밝은 곳이 포화 직전이
되는 노출부터 절반씩 어둡게 (노이즈 모델 유효 범위 ≤ 1 s 안에서). 나머지는 [고급]. 엔진은 bracket.py,
노이즈 모델 조건은 ptc_model.py (jsh-16 과 2026-10-20 합의).
"""
import json
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

import bracket as br
import ptc_model

LIGHTS = ["", "형광등", "LED (백색)", "LED (전구색)", "할로겐 · 백열", "나트륨등", "수은등", "자연광 (황혼 · 새벽)",
          "야간 외광 (가로등 · 하늘)", "혼합", "기타"]
RULES = [("same", "두 카메라 같은 노출 (먼저 포화되는 쪽 기준)"), ("per_camera", "카메라별 따로 (각자 포화 직전)"),
         ("manual", "직접 입력")]
SPACINGS = [(2.0, "2배씩 (1 EV)"), (4.0, "4배씩 (2 EV)"), (2.0 ** 0.5, "1.4배씩 (½ EV)")]
LOG_DIR = Path("/mnt/data/jsh_raw/logs/bracket")
PENDING, ACTIVE, DONE = "#e4e4e4", "#7fb2ff", "#7ccf86"
STAGE_LABEL = {"dark_pre": "dark (앞, 렌즈캡)", "bright": "bright", "dark": "dark (렌즈캡)"}
MAX_MS = max(v[1] for v in ptc_model.EXPOSURE_US.values()) / 1000.0      # BFS 1 s (LUCID 는 자기 상한에서 따로 잘린다)


def fmt_ms(us):
    ms = us / 1000.0
    if ms >= 100:
        return f"{ms:.0f}"
    if ms >= 10:
        return f"{ms:.1f}".rstrip("0").rstrip(".")
    return f"{ms:.2f}".rstrip("0").rstrip(".")


class Emitter(QtCore.QObject):
    log = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)
    step = QtCore.pyqtSignal(str, int, dict)          # stage, i, exposure
    shot = QtCore.pyqtSignal(str, int, int)           # stage, i, frames done in this step
    base = QtCore.pyqtSignal(dict, dict)              # base exposure, levels
    finished = QtCore.pyqtSignal(str)
    ask = QtCore.pyqtSignal(str, str)                 # question, kind
    drivers = QtCore.pyqtSignal(object)
    preflight = QtCore.pyqtSignal(object)


class Preview(QtWidgets.QLabel):
    """미리보기 JPEG + 포화 픽셀(빨강) 겹쳐 그리기."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(420, 260)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet("background: #111; color: #888;")
        self.setText("카메라 꺼짐")
        self._pix = self._overlay = self._stamp = None

    def show_jpeg(self, data, stamp):
        if stamp == self._stamp:
            return
        pix = QtGui.QPixmap()
        if pix.loadFromData(data, "JPG"):
            self._pix, self._stamp = pix, stamp
            self._rescale()

    def set_overlay(self, image):
        self._overlay = image
        self._rescale()

    def clear(self, text):
        if self._pix is None and self.text() == text:
            return
        self._pix = self._overlay = self._stamp = None
        self.setPixmap(QtGui.QPixmap())
        self.setText(text)

    def _rescale(self):
        if self._pix is None:
            return
        scaled = self._pix.scaled(self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        if self._overlay is not None:
            painter = QtGui.QPainter(scaled)
            painter.drawImage(scaled.rect(), self._overlay)
            painter.end()
        self.setPixmap(scaled)

    def resizeEvent(self, event):
        self._rescale()
        super().resizeEvent(event)


class Window(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RAW 브라케팅 — FLIR BFS + LUCID TRI054S")
        self.settings = QtCore.QSettings("jsh", "raw_bracket")
        self.emitter = Emitter()
        self.emitter.log.connect(self._append_log)
        self.emitter.status.connect(self._set_status)
        self.emitter.step.connect(self._on_step)
        self.emitter.shot.connect(self._on_shot)
        self.emitter.base.connect(self._on_base)
        self.emitter.finished.connect(self._on_finished)
        self.emitter.ask.connect(self._on_ask)
        self.emitter.drivers.connect(self._on_drivers)
        self.emitter.preflight.connect(self._on_preflight)
        self.link = br.RosLink()
        self.rig = br.Rig(self.link, log=self.emitter.log.emit)
        self.procs = []
        self.launching = False
        self.stop_event = threading.Event()
        self.answer = (threading.Event(), [False])
        self.worker = None
        self.armed = False
        self.base_us = None             # 마지막 측광 결과 {key: us}
        self.metered_at = None
        self.overlay_stamp = {}
        self.boxes = {}
        self.frames_for = {}
        self.preflight = None           # 마지막 대조 결과 {key: (wrong, unknown, warnings)} 또는 오류 문자열
        self.checking = False
        self.were_alive = False
        self._build()
        self._load_settings()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(250)

    # ---------- 화면 ----------
    def _build(self):
        root = QtWidgets.QVBoxLayout(self)
        big = QtGui.QFont()
        big.setPointSize(13)

        top = QtWidgets.QHBoxLayout()
        self.lbl_cam = QtWidgets.QLabel()
        self.lbl_cam.setTextFormat(QtCore.Qt.RichText)
        self.btn_cam = QtWidgets.QPushButton("카메라 켜기")
        self.btn_cam.clicked.connect(self._toggle_cameras)
        top.addWidget(self.lbl_cam)
        top.addWidget(self.btn_cam)
        top.addSpacing(24)
        self.ed_scene = QtWidgets.QLineEdit()
        self.ed_scene.setPlaceholderText("예: 주차장 입구")
        self.ed_lux = QtWidgets.QLineEdit()
        self.ed_lux.setPlaceholderText("lux")
        self.ed_lux.setMaximumWidth(80)
        self.cb_light = QtWidgets.QComboBox()
        self.cb_light.setEditable(True)
        self.cb_light.addItems(LIGHTS)
        self.cb_light.setMinimumWidth(170)
        self.ed_notes = QtWidgets.QLineEdit()
        self.ed_notes.setPlaceholderText("메모 (선택)")
        for label, w, stretch in (("장면", self.ed_scene, 2), ("조도", self.ed_lux, 0), ("광원", self.cb_light, 0),
                                  (None, self.ed_notes, 2)):
            if label:
                top.addWidget(QtWidgets.QLabel(label))
            top.addWidget(w, stretch)
        root.addLayout(top)

        ptc = QtWidgets.QHBoxLayout()
        self.lbl_ptc = QtWidgets.QLabel()
        self.lbl_ptc.setTextFormat(QtCore.Qt.RichText)
        self.btn_fix = QtWidgets.QPushButton("조건대로 맞추기")
        self.btn_fix.clicked.connect(self._fix)
        self.btn_recheck = QtWidgets.QPushButton("다시 확인")
        self.btn_recheck.clicked.connect(self._check)
        ptc.addWidget(self.lbl_ptc, 1)
        ptc.addWidget(self.btn_fix)
        ptc.addWidget(self.btn_recheck)
        root.addLayout(ptc)
        self.lbl_ptc_detail = QtWidgets.QLabel()
        self.lbl_ptc_detail.setTextFormat(QtCore.Qt.RichText)
        self.lbl_ptc_detail.setWordWrap(True)
        self.lbl_ptc_detail.hide()
        root.addWidget(self.lbl_ptc_detail)

        views = QtWidgets.QHBoxLayout()
        self.previews, self.meters = {}, {}
        for cam in br.CAMS:
            col = QtWidgets.QVBoxLayout()
            title = QtWidgets.QLabel(f"<b>{cam.label}</b>")
            self.previews[cam.key] = Preview()
            self.meters[cam.key] = QtWidgets.QLabel("")
            self.meters[cam.key].setTextFormat(QtCore.Qt.RichText)
            col.addWidget(title)
            col.addWidget(self.previews[cam.key], 1)
            col.addWidget(self.meters[cam.key])
            views.addLayout(col, 1)
        root.addLayout(views, 1)
        hint = QtWidgets.QLabel("미리보기는 보기 좋게 밝힌 화면이다. 빨간 곳은 RAW 가 포화된 픽셀.")
        hint.setStyleSheet("color: #777;")
        root.addWidget(hint)

        ladder_box = QtWidgets.QGroupBox("노출 단계")
        ladder = QtWidgets.QVBoxLayout(ladder_box)
        self.lbl_ladder = QtWidgets.QLabel()
        self.lbl_ladder.setTextFormat(QtCore.Qt.RichText)
        self.lbl_ladder.setFont(big)
        self.lbl_ladder.setWordWrap(True)
        row = QtWidgets.QHBoxLayout()
        self.cb_gain = QtWidgets.QComboBox()
        for g in ptc_model.GAINS_DB["bfs"]:
            self.cb_gain.addItem(f"{g:g} dB", g)
        self.cb_gain.setToolTip("BFS 는 아날로그 게인이라 올리면 read noise 가 줄어 저조도에서 이득 (모델에 있는 값만).\n"
                                "LUCID 게인은 HDR 합성 뒤 디지털이라 SNR 이득이 없어 0 dB 로 고정.")
        self.lbl_ladder_hint = QtWidgets.QLabel()
        self.lbl_ladder_hint.setStyleSheet("color: #555;")
        self.lbl_ladder_hint.setWordWrap(True)
        self.btn_meter = QtWidgets.QPushButton("미리 측광")
        self.btn_meter.setToolTip("노출 단계를 지금 장면으로 미리 정해 본다 (촬영 시작 때도 다시 잰다)")
        self.btn_meter.clicked.connect(self._meter)
        row.addWidget(QtWidgets.QLabel("BFS 게인"))
        row.addWidget(self.cb_gain)
        row.addWidget(QtWidgets.QLabel("· LUCID 0 dB 고정   "))
        row.addWidget(self.lbl_ladder_hint, 1)
        row.addWidget(self.btn_meter)
        ladder.addWidget(self.lbl_ladder)
        ladder.addLayout(row)
        root.addWidget(ladder_box)

        run = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("▶  이 장면 찍기")
        self.btn_start.setFont(big)
        self.btn_start.setMinimumHeight(44)
        self.btn_start.setStyleSheet("QPushButton { background: #2f7d32; color: white; border-radius: 6px; padding: 0 22px; }"
                                     "QPushButton:disabled { background: #9aa59b; }")
        self.btn_start.clicked.connect(self._start)
        self.btn_stop = QtWidgets.QPushButton("■ 멈춤")
        self.btn_stop.setMinimumHeight(44)
        self.btn_stop.clicked.connect(self._stop)
        self.lbl_estimate = QtWidgets.QLabel()
        run.addWidget(self.btn_start)
        run.addWidget(self.btn_stop)
        run.addWidget(self.lbl_estimate, 1)
        root.addLayout(run)

        self.lbl_status = QtWidgets.QLabel("카메라를 켜고 장면 이름을 적은 뒤 [이 장면 찍기]")
        self.lbl_status.setFont(big)
        root.addWidget(self.lbl_status)
        self.banner = QtWidgets.QFrame()
        self.banner.setStyleSheet("QFrame { background: #fff3c4; border: 1px solid #e0b000; border-radius: 6px; }")
        banner = QtWidgets.QHBoxLayout(self.banner)
        self.lbl_banner = QtWidgets.QLabel()
        self.lbl_banner.setFont(big)
        self.btn_go = QtWidgets.QPushButton("계속")
        self.btn_skip = QtWidgets.QPushButton("이 dark 건너뛰기")
        self.btn_go.clicked.connect(lambda: self._answer(True))
        self.btn_skip.clicked.connect(lambda: self._answer(False))
        banner.addWidget(self.lbl_banner, 1)
        banner.addWidget(self.btn_go)
        banner.addWidget(self.btn_skip)
        self.banner.hide()
        root.addWidget(self.banner)
        self.box_rows = QtWidgets.QVBoxLayout()
        root.addLayout(self.box_rows)

        self.btn_adv = QtWidgets.QToolButton()
        self.btn_adv.setText("고급")
        self.btn_adv.setCheckable(True)
        self.btn_adv.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.btn_adv.setArrowType(QtCore.Qt.RightArrow)
        self.btn_adv.toggled.connect(self._toggle_advanced)
        root.addWidget(self.btn_adv)
        self.adv = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(self.adv)
        self.cb_rule = QtWidgets.QComboBox()
        for key, label in RULES:
            self.cb_rule.addItem(label, key)
        self.sp_manual = {k: self._spin(0.086, MAX_MS, 20, 3, " ms") for k in ("bfs", "lucid")}
        manual = QtWidgets.QHBoxLayout()
        for k in ("bfs", "lucid"):
            manual.addWidget(QtWidgets.QLabel(br.CAM[k].label))
            manual.addWidget(self.sp_manual[k])
        manual.addStretch(1)
        self.sp_target = self._spin(10, 98, 80, 0, " %")
        self.sp_max = self._spin(1, MAX_MS, MAX_MS, 0, " ms")
        self.sp_steps = QtWidgets.QSpinBox()
        self.sp_steps.setRange(1, 20)
        self.cb_spacing = QtWidgets.QComboBox()
        for factor, label in SPACINGS:
            self.cb_spacing.addItem(label, factor)
        self.sp_frames = QtWidgets.QSpinBox()
        self.sp_frames.setRange(1, 200)
        self.chk_dark = QtWidgets.QCheckBox("렌즈캡 씌우고 같은 노출로 dark 도 찍기 (BFS 게인 ≥ 18 dB 면 bright 앞뒤로)")
        self.chk_flicker = QtWidgets.QCheckBox("60 Hz 조명 깜빡임: 8.33 ms 이상 노출을 1/120 s 배수로 맞춤 (단계 비는 측정 신호로)")
        self.chk_common = QtWidgets.QCheckBox("두 카메라를 같은 노출로 자르기 (LUCID 상한에 BFS 도 맞춤 — 같은 노출 비교용)")
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        self.log.setFixedHeight(130)
        form.addRow("노출 기준", self.cb_rule)
        form.addRow("직접 입력 (맨 위 단계)", manual)
        form.addRow("맨 위 단계의 밝기", self._row(self.sp_target, "가장 밝은 곳(상위 0.5 %)이 포화의 몇 %가 되게"))
        form.addRow("최대 노출", self._row(self.sp_max, "노이즈 모델 유효 범위: " + " · ".join(
            f"{br.CAM[k].label} ≤ {v[1] / 1000:.0f} ms" for k, v in ptc_model.EXPOSURE_US.items())))
        form.addRow("단계 수", self._row(self.sp_steps, "", self.cb_spacing))
        form.addRow("단계당 장수", self._row(self.sp_frames))
        form.addRow("dark", self.chk_dark)
        form.addRow("깜빡임", self.chk_flicker)
        form.addRow("노출 범위", self.chk_common)
        form.addRow("기록", self.log)
        self.adv.hide()
        root.addWidget(self.adv)

        for w in (self.cb_rule, self.cb_spacing):
            w.currentIndexChanged.connect(self._plan_changed)
        self.cb_gain.currentIndexChanged.connect(self._gain_changed)
        for w in (self.sp_steps, self.sp_frames, self.sp_target, self.sp_max, *self.sp_manual.values()):
            w.valueChanged.connect(self._plan_changed)
        self.chk_dark.toggled.connect(self._plan_changed)
        self.chk_flicker.toggled.connect(self._plan_changed)
        self.chk_common.toggled.connect(self._plan_changed)
        self._set_busy(False)

    @staticmethod
    def _spin(lo, hi, value, decimals, suffix):
        s = QtWidgets.QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(decimals)
        s.setValue(value)
        s.setSuffix(suffix)
        s.setKeyboardTracking(False)
        return s

    @staticmethod
    def _row(*widgets):
        box = QtWidgets.QHBoxLayout()
        for w in widgets:
            box.addWidget(QtWidgets.QLabel(w) if isinstance(w, str) else w)
        box.addStretch(1)
        return box

    def _toggle_advanced(self, on):
        self.adv.setVisible(on)
        self.btn_adv.setArrowType(QtCore.Qt.DownArrow if on else QtCore.Qt.RightArrow)

    # ---------- 계획 ----------
    def _rule(self):
        return self.cb_rule.currentData()

    def _gain(self):
        return float(self.cb_gain.currentData())

    def _factor(self):
        return float(self.cb_spacing.currentData())

    def _evs(self):
        return [-i * float(np.log2(self._factor())) for i in range(self.sp_steps.value())]

    def _flicker(self):
        return br.FLICKER_PERIOD_60HZ_US if self.chk_flicker.isChecked() else None

    def _bfs_warming(self):
        """BFS 가 켠 지 30분이 안 됐나 (DeviceUptime) — 데워지는 중이라 bright · dark 사이 offset 이 흐를 수 있다."""
        meta = self.link.last_meta.get("bfs")
        uptime = getattr(meta, "device_uptime_s", float("nan")) if meta is not None else float("nan")
        return uptime == uptime and uptime < ptc_model.WARMUP_S

    def _dark_before(self):
        """dark 를 bright 앞뒤로: BFS 게인 ≥ 18 dB, 또는 BFS 워밍업 30분 전 (jsh-16 합의)."""
        return self.chk_dark.isChecked() and (self._gain() >= ptc_model.DARK_BEFORE_FROM_GAIN_DB or self._bfs_warming())

    def _stages(self):
        return (["dark_pre"] if self._dark_before() else []) + ["bright"] + (["dark"] if self.chk_dark.isChecked() else [])

    def _top_us(self):
        """맨 위 단계 노출 {key: us} — 직접 입력, 또는 마지막 측광값 (없으면 None)."""
        if self._rule() == "manual":
            return {k: self.sp_manual[k].value() * 1000.0 for k in self.sp_manual}
        return dict(self.base_us) if self.base_us else None

    def _ladder(self, top):
        return br.step_exposures(top, self._evs(), list(top), self.sp_max.value() * 1000.0, self._flicker(),
                                 self.chk_common.isChecked())

    def _plan_changed(self, *_):
        self.sp_manual["bfs"].setEnabled(self._rule() == "manual")
        self.sp_manual["lucid"].setEnabled(self._rule() == "manual")
        self._update_buttons()
        self._draw_ladder()
        self._draw_estimate()

    def _draw_ladder(self, current=None):
        top = self._top_us()
        n = self.sp_steps.value()
        notes = []
        if self._dark_before():
            notes.append("BFS 게인 ≥ 18 dB → dark 를 bright 앞뒤로 찍는다" if self._gain() >= ptc_model.DARK_BEFORE_FROM_GAIN_DB
                         else "BFS 워밍업 30분 전 → dark 를 bright 앞뒤로 찍는다")
        if top is None:
            self.lbl_ladder.setText(
                f"{n}단계 · {self.cb_spacing.currentText().split(' ')[0]} 어둡게 · 최대 {fmt_ms(self.sp_max.value() * 1000)} ms"
                " — 맨 위 노출은 촬영 시작 때 측광으로 정한다")
            notes.insert(0, f"측광: 같은 노출로 찍어서 먼저 포화되는 쪽의 가장 밝은 곳이 포화의 {self.sp_target.value():.0f}%"
                         if self._rule() == "same" else "측광: 카메라마다 따로 포화 직전")
            self.lbl_ladder_hint.setText(" · ".join(notes))
            return
        steps = self._ladder(top)
        same = all(abs(s["bfs"] - s["lucid"]) < 1e-6 for s in steps)
        rows = [("두 카메라", "bfs")] if same else [("BFS", "bfs"), ("LUCID", "lucid")]
        html = []
        for label, k in rows:
            cells = []
            for i, s in enumerate(steps):
                text = fmt_ms(s[k])
                if s["lucid"] > ptc_model.LUCID_TRIGGER_MAX_US:
                    text = f"<i>{text}</i>"           # LUCID free-run 단계
                if current is not None and i == current:
                    text = f"<span style='background:{ACTIVE}; padding:2px;'><b>{text}</b></span>"
                cells.append(text)
            html.append(f"<b>{label}</b>&nbsp; " + " → ".join(cells) + " ms")
        self.lbl_ladder.setText("<br>".join(html))
        free = sum(s["lucid"] > ptc_model.LUCID_TRIGGER_MAX_US for s in steps)
        if free:
            notes.append(f"LUCID {fmt_ms(ptc_model.LUCID_TRIGGER_MAX_US)} ms 초과 {free}단계는 free-run (BFS 와 동시 아님 — "
                         "트리거 모드 dark 가 모델과 달라서)")
        if any(s["lucid"] < 1000 for s in steps):
            notes.append("LUCID 1 ms 미만 단계: 실제 적분시간이 계단식이라 단계 비는 측정 신호로")
        if self._rule() == "manual":
            notes.insert(0, "직접 입력한 노출부터")
        elif self.metered_at:
            notes.insert(0, f"{self.metered_at} 측광값 — 촬영 시작 때 다시 잰다")
        self.lbl_ladder_hint.setText(" · ".join(notes))

    def _draw_estimate(self):
        top = self._top_us() or {"bfs": 0.2e6, "lucid": 0.2e6}
        steps = self._ladder(top)
        frames = self.sp_frames.value()
        per_frames = {"dark_pre": 3, "bright": frames, "dark": frames}
        # 쌍 하나: LUCID 트리거 지연(2/fps − 노출, 2 Hz 에서 ~1 s) + 노출 + 전송 ~0.5 s. 단계마다 버림 쌍 하나.
        per_step = [max(1.0, 2 * max(s.values()) / 1e6) + 0.5 for s in steps]
        seconds = sum((per_frames[st] + 1) * t for st in self._stages() for t in per_step)
        seconds += 20 if self._rule() != "manual" else 0
        pairs = sum(per_frames[st] for st in self._stages()) * len(steps)
        self.lbl_estimate.setText(f"   {pairs}쌍 · 약 {max(1, round(seconds / 60))}분 · {pairs * 20.7 / 1000:.1f} GB")

    # ---------- 설정 저장 ----------
    def _load_settings(self):
        s = self.settings
        self.ed_scene.setText(s.value("scene", ""))
        self.ed_lux.setText(s.value("lux", ""))
        self.cb_light.setCurrentText(s.value("light", ""))
        self.ed_notes.setText(s.value("notes", ""))
        self.cb_rule.setCurrentIndex(max(0, self.cb_rule.findData(s.value("rule2", "same"))))
        for k in self.sp_manual:
            self.sp_manual[k].setValue(float(s.value(f"manual_{k}", 20)))
        self.sp_target.setValue(float(s.value("target", 80)))
        self.sp_max.setValue(min(MAX_MS, float(s.value("max_ms", MAX_MS))))
        self.sp_steps.setValue(int(s.value("steps", 9)))
        self.cb_spacing.setCurrentIndex(int(s.value("spacing", 0)))
        self.sp_frames.setValue(int(s.value("frames", 3)))
        # 장면 dark 는 기본으로 안 찍는다 (2026-10-20 사용자: 렌즈캡이 번거롭고 노이즈 모델로 충분하다).
        # 예전 "dark"=true 저장값을 따르지 않도록 키를 바꿨다.
        self.chk_dark.setChecked(s.value("dark_scene", "false") == "true")
        self.chk_flicker.setChecked(s.value("flicker", "true") == "true")
        self.chk_common.setChecked(s.value("clip_common", "false") == "true")
        self.cb_gain.blockSignals(True)
        self.cb_gain.setCurrentIndex(max(0, self.cb_gain.findData(float(s.value("bfs_gain", 0.0)))))
        self.cb_gain.blockSignals(False)
        self._plan_changed()

    def _save_settings(self):
        s = self.settings
        for key, value in (("scene", self.ed_scene.text()), ("lux", self.ed_lux.text()),
                           ("light", self.cb_light.currentText()), ("notes", self.ed_notes.text()),
                           ("rule2", self._rule()), ("target", self.sp_target.value()),
                           ("max_ms", self.sp_max.value()), ("steps", self.sp_steps.value()),
                           ("spacing", self.cb_spacing.currentIndex()), ("frames", self.sp_frames.value()),
                           ("dark_scene", "true" if self.chk_dark.isChecked() else "false"),
                           ("flicker", "true" if self.chk_flicker.isChecked() else "false"),
                           ("clip_common", "true" if self.chk_common.isChecked() else "false"),
                           ("bfs_gain", self._gain())):
            s.setValue(key, value)
        for k in self.sp_manual:
            s.setValue(f"manual_{k}", self.sp_manual[k].value())

    # ---------- PTC 조건 ----------
    def _check(self):
        if self.checking or not all(self.link.alive(c.key) for c in br.CAMS):
            return
        self.checking = True
        self.lbl_ptc.setText("PTC 조건 확인 중 …")
        gain = self._gain()

        def body():
            try:
                result = self.rig.preflight(bfs_gain_db=gain)
            except Exception as e:  # noqa: BLE001
                result = f"확인 실패: {e}"
            self.emitter.preflight.emit(result)
        threading.Thread(target=body, daemon=True).start()

    def _on_preflight(self, result):
        self.checking = False
        self.preflight = result
        if isinstance(result, str):
            self.lbl_ptc.setText(f"<span style='color:#d33;'>●</span> {result}")
            self.lbl_ptc_detail.hide()
            self._update_buttons()
            return
        wrong = [(k, w) for k, (ws, _u, _x) in result.items() for w in ws]
        unknown = [(k, u) for k, (_w, us, _x) in result.items() for u in us]
        warnings = [(k, x) for k, (_w, _u, xs) in result.items() for x in xs]
        if wrong:
            head = f"<span style='color:#d33; font-size:14pt;'>●</span> <b>PTC 조건과 다른 설정 {len(wrong)}개</b> — 맞출 때까지 촬영 막음"
        else:
            head = "<span style='color:#2e9d3a; font-size:14pt;'>●</span> PTC 조건 맞음 (노이즈 모델을 그대로 쓸 수 있다)"
        if warnings:
            head += f" · <span style='color:#c77700;'>주의 {len(warnings)}</span>"
        self.lbl_ptc.setText(head)
        lines = [f"<span style='color:#d33;'>✗ {br.CAM[k].label} {label}: {have} (필요: {want})"
                 f"{'' if fix else ' — 도구가 못 고침'}</span>" for k, (label, have, want, fix) in wrong]
        lines += [f"<span style='color:#c77700;'>! {br.CAM[k].label} {x}</span>" for k, x in warnings]
        lines += [f"<span style='color:#888;'>? {br.CAM[k].label} {label}: 읽을 수 없어 확인 못 함</span>"
                  for k, (label, _h, _w, _f) in unknown]
        self.lbl_ptc_detail.setText("<br>".join(lines))
        self.lbl_ptc_detail.setVisible(bool(lines))
        self._update_buttons()

    def _ptc_ok(self):
        return isinstance(self.preflight, dict) and not any(w for w, _u, _x in self.preflight.values())

    def _fix(self):
        if not isinstance(self.preflight, dict):
            return
        result = self.preflight
        self.btn_fix.setEnabled(False)
        self.lbl_ptc.setText("조건대로 맞추는 중 …")

        def body():
            failed = self.rig.fix(result)
            for msg in failed:
                self.emitter.log.emit("맞추기 실패: " + msg)
            time.sleep(2.5)          # 새 설정으로 프레임 · 메타데이터가 들어오게
            try:
                out = self.rig.preflight(bfs_gain_db=self._gain())
            except Exception as e:  # noqa: BLE001
                out = f"확인 실패: {e}"
            self.emitter.preflight.emit(out)
        threading.Thread(target=body, daemon=True).start()

    def _gain_changed(self, *_):
        self._plan_changed()
        if not self.link.alive("bfs") or self.worker:
            return
        gain = self._gain()

        def body():
            try:
                self.link.set_params("bfs", {"camera.Gain": gain})
                self.emitter.log.emit(f"BFS 게인 {gain:g} dB")
                time.sleep(1.5)
                out = self.rig.preflight(bfs_gain_db=gain)
            except Exception as e:  # noqa: BLE001
                out = f"게인 설정 실패: {e}"
            self.emitter.preflight.emit(out)
        threading.Thread(target=body, daemon=True).start()

    # ---------- 주기적으로: 카메라 상태 · 미리보기 · 포화 표시 ----------
    def _refresh(self):
        present = self.link.drivers_present()
        alive = {c.key: self.link.alive(c.key) for c in br.CAMS}
        ours = [p for p in self.procs if p.poll() is None]
        both = all(alive.values())
        if both or (self.armed and all(present.values())):
            dot, text = "#2e9d3a", "두 카메라 연결됨" + (" (다른 곳에서 켬)" if not ours else "")
        elif self.launching or (ours and not all(present.values())):
            dot, text = "#e0a000", "켜는 중 … (LUCID 는 15초쯤)"
        elif all(present.values()):        # 드라이버는 떠 있고 프레임만 잠깐 없다 (스트림 재시작 · 설정 변경)
            dot, text = "#e0a000", "프레임 기다리는 중: " + ", ".join(br.CAM[k].label for k, v in alive.items() if not v)
        elif any(present.values()):
            dot, text = "#d33", "한쪽만 연결됨: " + ", ".join(br.CAM[k].label for k, v in alive.items() if v)
        else:
            dot, text = "#999", "카메라 꺼짐"
        self.lbl_cam.setText(f"<span style='color:{dot}; font-size:16pt;'>●</span> {text}")
        if both and not self.were_alive and not self.worker:
            QtCore.QTimer.singleShot(1500, self._check)
        if not both and not self.armed:
            if self.preflight is not None:
                self.preflight = None
                self.lbl_ptc_detail.hide()
            self.lbl_ptc.setText("PTC 조건: 카메라를 켜면 확인한다")
        self.were_alive = both
        warming = self._bfs_warming()
        if warming != getattr(self, "_was_warming", None) and not self.worker:
            self._was_warming = warming
            self._plan_changed()
        if not self.worker:
            self.btn_cam.setText("카메라 끄기" if ours else "카메라 켜기")
            self.btn_cam.setEnabled(not self.launching and (bool(ours) or not any(present.values())))
            self._update_buttons()
        for cam in br.CAMS:
            k = cam.key
            preview = self.link.preview[k]
            if preview is not None and alive[k]:
                self.previews[k].show_jpeg(*preview)
            elif not alive[k] and not self.armed:
                self.previews[k].clear("카메라 꺼짐" if not present.get(k) else "프레임 없음")
                self.meters[k].setText("")
            self._update_meter(k)

    def _update_buttons(self):
        busy = bool(self.worker)
        ok = self._ptc_ok()
        self.btn_start.setEnabled(not busy and ok)
        self.btn_meter.setEnabled(not busy and ok and self._rule() != "manual")
        fixable = isinstance(self.preflight, dict) and any(f for ws, _u, _x in self.preflight.values() for *_r, f in ws)
        self.btn_fix.setEnabled(not busy and fixable and not self.checking)
        self.btn_recheck.setEnabled(not busy and not self.checking)
        self.btn_start.setToolTip("" if ok else "PTC 조건이 맞아야 찍을 수 있다 (위 빨간 줄)")

    def _update_meter(self, k):
        frame = self.link.latest(k)
        if frame is None or self.overlay_stamp.get(k) == frame.stamp_ns:
            return
        self.overlay_stamp[k] = frame.stamp_ns
        try:
            raw = frame.raw
            black, white = br.black_and_white(frame)
            b = float(np.mean(list(black.values())))
            h, w = (raw.shape[0] // 8) * 8, (raw.shape[1] // 8) * 8
            sites = np.stack([raw[dy:h:8, dx:w:8] for dy in (0, 1) for dx in (0, 1)]).astype(np.float32)
            level = float(max(np.percentile(s, 99.5) for s in sites) - b) / max(1.0, white - b)
            clipped = sites.max(axis=0) >= b + 0.98 * (white - b)
        except Exception:  # noqa: BLE001 — 표시용이라 한 장 놓쳐도 된다
            return
        frac = float(clipped.mean())
        rgba = np.zeros(clipped.shape + (4,), np.uint8)
        rgba[clipped] = (255, 30, 30, 200)
        image = QtGui.QImage(rgba.data, rgba.shape[1], rgba.shape[0], rgba.shape[1] * 4,
                             QtGui.QImage.Format_RGBA8888).copy()
        self.previews[k].set_overlay(image if frac > 0 else None)
        meta = frame.meta
        color = "#d33" if frac > 0.001 else "#e08a00" if level >= 0.9 else "#2e7d32"
        text = (f"노출 {fmt_ms(meta['exposure_time_us'])} ms · 게인 {meta.get('gain_db', 0):g} dB · 가장 밝은 곳 "
                f"<b style='color:{color}'>{max(0.0, level) * 100:.0f}%</b> · 포화 {frac * 100:.2f}%")
        temp = meta.get("device_temperature_c")
        if temp is not None and temp == temp:
            lo, hi = ptc_model.TEMPERATURE_C[k]
            tcolor = "#2e7d32" if lo <= temp <= hi else "#c77700"
            text += f" · <span style='color:{tcolor}'>{temp:.0f}°C</span>"
        self.meters[k].setText(text)

    # ---------- 일 ----------
    def _set_busy(self, busy):
        for w in (self.btn_cam, self.ed_scene, self.ed_lux, self.cb_light, self.ed_notes, self.cb_gain,
                  self.cb_rule, *self.sp_manual.values(), self.sp_target, self.sp_max, self.sp_steps,
                  self.cb_spacing, self.sp_frames, self.chk_dark, self.chk_flicker, self.chk_common):
            w.setEnabled(not busy)
        self.btn_stop.setEnabled(busy)
        if not busy:
            self._plan_changed()
        self._update_buttons()

    def _set_status(self, text):
        self.lbl_status.setText(text)

    def _append_log(self, text):
        self.log.appendPlainText(time.strftime("%H:%M:%S ") + text)

    def _run_in_thread(self, fn, label):
        if self.worker:
            return
        self.stop_event.clear()
        self.worker = True                  # 버튼을 바로 잠근다 (스레드 객체는 아래에서)
        self._set_busy(True)

        def body():
            result = ""
            try:
                result = fn() or ""
            except br.Stopped:
                result = "멈춤"
            except Exception as e:  # noqa: BLE001
                self.emitter.log.emit(f"{label} 실패: {e}")
                self.emitter.log.emit(traceback.format_exc().strip().splitlines()[-1])
                result = f"{label} 실패: {e}"
            finally:
                self.armed = False
                self.emitter.finished.emit(result)
        self.worker = threading.Thread(target=body, daemon=True)
        self.worker.start()

    def _cameras_ready(self):
        missing = [c.label for c in br.CAMS if not self.link.alive(c.key)]
        if missing:
            QtWidgets.QMessageBox.warning(self, "카메라", "카메라가 안 켜져 있다: " + ", ".join(missing))
            return False
        if not self._ptc_ok():
            QtWidgets.QMessageBox.warning(self, "PTC 조건", "PTC 조건이 맞지 않는다. 위 빨간 항목을 맞춘 뒤에 찍을 수 있다.")
            return False
        return True

    def _verify(self):
        """촬영 직전 다시 대조 — 그 사이 설정이 바뀌었으면 멈춘다."""
        result = self.rig.preflight(bfs_gain_db=self._gain())
        self.emitter.preflight.emit(result)
        wrong = [f"{br.CAM[k].label} {w[0]}" for k, (ws, _u, _x) in result.items() for w in ws]
        if wrong:
            raise RuntimeError("PTC 조건과 다른 설정: " + ", ".join(wrong))
        return result

    def _do_meter(self):
        rule = "same" if self._rule() == "same" else "per_camera"
        self.emitter.status.emit("측광 중 …")
        self.rig.arm(True)
        self.armed = True
        try:
            start = self.base_us or {"bfs": 20000.0, "lucid": 20000.0}
            limit = self.sp_max.value() * 1000.0
            base, levels = self.rig.meter(
                start, target=self.sp_target.value() / 100.0,
                limits={k: (v[0], min(v[1], limit)) for k, v in ptc_model.EXPOSURE_US.items()}, rule=rule,
                stop=self.stop_event,
                progress=lambda i, n, e: self.emitter.status.emit(
                    f"측광 중 … {i}/{n}  (" + ", ".join(f"{br.CAM[k].label} {fmt_ms(v)} ms" for k, v in e.items()) + ")"))
        finally:
            self.armed = False
            self.rig.arm(False)
        self.emitter.base.emit(base, levels)
        top = max(base.values())
        if top >= limit * 0.999 and max(levels.values()) < 0.5 * self.sp_target.value() / 100.0:
            self.emitter.log.emit("측광: 최대 노출(1 s)에서도 어둡다 — BFS 게인을 올리면 BFS 신호가 커진다 "
                                  "(LUCID 는 게인 이득이 없다)")
        return base

    def _meter(self):
        if self._cameras_ready():
            self._save_settings()
            self._run_in_thread(lambda: (self._verify(), self._do_meter()) and "측광 끝", "측광")

    def _start(self):
        if not self.ed_scene.text().strip():
            QtWidgets.QMessageBox.warning(self, "장면", "장면 이름을 적어 줘 (폴더 이름이 된다).")
            self.ed_scene.setFocus()
            return
        if not self._cameras_ready():
            return
        self._save_settings()
        scene = {"name": self.ed_scene.text().strip(), "lux": self.ed_lux.text().strip(),
                 "light": self.cb_light.currentText().strip(), "notes": self.ed_notes.text().strip()}
        rule = self._rule()
        self._make_boxes(self._stages(), self.sp_steps.value())

        def job():
            check = self._verify()
            top = self._do_meter() if rule != "manual" else self._top_us()
            plan = {"ev": self._evs(), "frames": self.sp_frames.value(), "base_us": top,
                    "max_us": self.sp_max.value() * 1000.0, "flicker_period_us": self._flicker(),
                    "dark": self.chk_dark.isChecked(), "dark_before": self._dark_before(), "dark_pre_frames": 3,
                    "dark_before_reason": ("bfs_gain>=18dB" if self._gain() >= ptc_model.DARK_BEFORE_FROM_GAIN_DB
                                           else "bfs_warmup<30min" if self._bfs_warming() else None),
                    "lucid_trigger_max_us": ptc_model.LUCID_TRIGGER_MAX_US, "clip_common": self.chk_common.isChecked(),
                    "rule": rule, "step_factor": self._factor(), "bfs_gain_db": self._gain(), "lucid_gain_db": 0.0,
                    "meter_target": self.sp_target.value() / 100.0 if rule != "manual" else None,
                    "preflight_warnings": {k: x for k, (_w, _u, x) in check.items() if x},
                    "preflight_unknown": {k: [u[0] for u in us] for k, (_w, us, _x) in check.items() if us}}
            self.armed = True
            folder = self.rig.run(
                scene, plan, stop=self.stop_event, ask=self._ask_cb,
                on_step=lambda stage, i, n, e: self.emitter.step.emit(stage, i, e),
                progress=lambda done, total, shot, frames: self.emitter.shot.emit(
                    shot["stage"], shot["step"], shot["index"] + 1))
            return str(folder)
        self._run_in_thread(job, "촬영")

    def _make_boxes(self, stages, steps):
        while self.box_rows.count():
            item = self.box_rows.takeAt(0)
            layout = item.layout()
            if layout is not None:
                while layout.count():
                    w = layout.takeAt(0).widget()
                    if w is not None:
                        w.deleteLater()
        self.boxes = {}
        self.frames_for = {"dark_pre": 3, "bright": self.sp_frames.value(), "dark": self.sp_frames.value()}
        for stage in stages:
            row = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(STAGE_LABEL[stage])
            label.setMinimumWidth(110)
            row.addWidget(label)
            for i in range(steps):
                box = QtWidgets.QLabel(f"0/{self.frames_for[stage]}")
                box.setAlignment(QtCore.Qt.AlignCenter)
                box.setFixedSize(52, 26)
                box.setStyleSheet(f"background:{PENDING}; border-radius:4px;")
                self.boxes[(stage, i)] = box
                row.addWidget(box)
            row.addStretch(1)
            self.box_rows.addLayout(row)

    def _on_step(self, stage, i, exposure):
        box = self.boxes.get((stage, i))
        if box is not None:
            box.setStyleSheet(f"background:{ACTIVE}; border-radius:4px;")
        self._draw_ladder(current=i)
        n = self.sp_steps.value()
        self._set_status(f"{STAGE_LABEL[stage]} {i + 1}/{n} 단계 · " +
                         ", ".join(f"{br.CAM[k].label} {fmt_ms(v)} ms" for k, v in exposure.items()))

    def _on_shot(self, stage, i, count):
        box = self.boxes.get((stage, i))
        if box is not None:
            box.setText(f"{count}/{self.frames_for.get(stage, count)}")
            if count >= self.frames_for.get(stage, count):
                box.setStyleSheet(f"background:{DONE}; border-radius:4px;")

    def _on_base(self, base, levels):
        self.base_us = dict(base)
        self.metered_at = time.strftime("%H:%M")
        if levels:
            k = max(levels, key=levels.get)
            self._append_log(f"측광: 맨 위 {fmt_ms(max(base.values()))} ms — {br.CAM[k].label} 가장 밝은 곳 "
                             f"{levels[k] * 100:.0f}%")
        self._plan_changed()

    def _ask_cb(self, question, kind=None):
        event, holder = self.answer
        event.clear()
        self.emitter.ask.emit(question, kind or "")
        while not event.wait(0.2):
            if self.stop_event.is_set():
                return False
        return holder[0]

    def _on_ask(self, question, kind):
        self.lbl_banner.setText(question)
        self.btn_go.setText("벗겼다 — 계속" if kind == "cap_off" else "씌웠다 — 계속")
        self.btn_skip.setVisible(kind != "cap_off")
        self.banner.show()
        self._set_status("렌즈캡 기다리는 중")
        QtWidgets.QApplication.beep()

    def _answer(self, yes):
        self.banner.hide()
        event, holder = self.answer
        holder[0] = yes
        event.set()

    def _stop(self):
        self.stop_event.set()
        self.banner.hide()
        self._set_status("멈추는 중 — 지금 쌍까지 찍고 멈춘다")

    def _on_finished(self, result):
        self.worker = None
        self.banner.hide()
        self._set_busy(False)
        self._draw_ladder()
        if result.startswith("/"):
            tail = "  (렌즈캡을 벗기면 다음 장면)" if self.chk_dark.isChecked() else ""
            try:
                warnings = json.loads((Path(result) / "scene.json").read_text()).get("warnings", [])
            except (OSError, ValueError):
                warnings = []
            note = (f"  · <span style='color:#c77700;'>경고 {len(warnings)} — scene.json warnings (dark 이동량은 "
                    "dark_shift 로 보정된다)</span>") if warnings else ""
            self._set_status(f"끝 — {Path(result).name}{tail}{note}")
            self._append_log(f"폴더: {result}")
        elif result:
            self._set_status(result)
        QtCore.QTimer.singleShot(2000, self._check)

    # ---------- 카메라 ----------
    def _toggle_cameras(self):
        ours = [p for p in self.procs if p.poll() is None]
        if ours:
            procs, self.procs = self.procs, []
            self._append_log("카메라 끄는 중 …")
            threading.Thread(target=lambda: (br.stop_drivers(procs), self.emitter.log.emit("카메라 꺼짐")),
                             daemon=True).start()
            return
        self.launching = True
        self._append_log("카메라 켜는 중 …")

        def body():
            try:
                self.emitter.drivers.emit(br.launch_drivers(LOG_DIR, log=self.emitter.log.emit))
            except Exception as e:  # noqa: BLE001
                self.emitter.log.emit(f"켜기 실패: {e}")
                self.emitter.status.emit(f"카메라를 못 켰다: {e}")
                self.emitter.drivers.emit([])
        threading.Thread(target=body, daemon=True).start()

    def _on_drivers(self, procs):
        self.launching = False
        self.procs.extend(procs)

    def closeEvent(self, event):
        if self.worker:
            if QtWidgets.QMessageBox.question(self, "촬영 중", "촬영 중이다. 멈추고 닫을까?") != \
                    QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
            self.stop_event.set()
            if isinstance(self.worker, threading.Thread):
                self.worker.join(timeout=30)
        self._save_settings()
        if self.procs:
            br.stop_drivers(self.procs)
        self.link.close()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = Window()
    w.resize(1500, 1080)
    w.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
