#!/usr/bin/env python3
# ui_theme.py — DataMachine 공통 테마 (라이트 · 다크).
#
# Fusion 스타일 + 명시적 팔레트 + 앱 전역 QSS. 팔레트를 명시하는 이유: 데스크톱 테마와 Qt 기본 팔레트가
# 섞이면 QSS로 칠한 곳과 안 칠한 곳이 따로 논다.
#
# 색은 전부 토큰(LIGHT / DARK)에서 온다. apply(app, mode) 가 고른 토큰으로 팔레트 · QSS 를 만들고, 다른 모듈이 쓰는
# ui_theme.OK / ERR / WARN / ACCENT / MUTED / TEXT / BORDER / BG 도 그 테마 값으로 바꾼다 — 그래서 창을 만들기 전에
# 한 번 불러야 하고, 테마를 바꾸면 GUI 를 다시 켜야 전부 바뀐다 (위젯마다 만들 때 박은 색이 있다).
# 위젯에 색을 직접 줄 때는 hex 를 쓰지 말고 c("muted") 처럼 토큰을 쓴다 — 그래야 다크에서도 맞는다.
#
# 버튼 변형은 동적 프로퍼티로 고른다 (set_variant):
#   "primary"  파란 채움 — 화면의 주 동작 하나
#   "success"  초록 채움 — 끝났으니 다음 단계로
#   "danger"   빨간 테두리 — 중지/삭제

import subprocess
from pathlib import Path

from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QColor, QGuiApplication, QPalette, QWheelEvent
from PyQt5.QtWidgets import QAbstractScrollArea, QAbstractSpinBox, QApplication, QComboBox, QDial, QSlider

APP_NAME = "RAW Lab"     # ~/jsh 복제본 (원본 DataMachine 과 창 · 작업 표시줄에서 구별)

LIGHT = {
    "bg": "#f5f6f8", "surface": "#ffffff", "surface2": "#f9fafb", "hover": "#f3f4f6", "pressed": "#e5e7eb",
    "border": "#e3e5e8", "border_strong": "#d1d5db", "text": "#111827", "text2": "#374151",
    "muted": "#6b7280", "faint": "#9ca3af", "disabled_bg": "#f3f4f6",
    "accent": "#2563eb", "accent_hover": "#1d4ed8", "accent_soft": "#dbeafe", "accent_softer": "#f5f9ff",
    "accent_disabled": "#93c5fd", "accent_disabled_text": "#eff6ff", "accent_border": "#93c5fd",
    "ok": "#16a34a", "ok_hover": "#15803d", "warn": "#d97706", "err": "#dc2626", "caution": "#b45309",
    "danger_text": "#b91c1c", "danger_border": "#fca5a5", "danger_bg": "#fef2f2",
    "gridline": "#f0f1f3", "altbase": "#fafbfc", "scroll": "#cfd4da", "scroll_hover": "#9ca3af",
    "split": "#d6dade", "split_hover": "#60a5fa", "tool_hover": "#eef0f3", "zero": "#b6bbc3",
    "tooltip_bg": "#111827", "tooltip_fg": "#f9fafb",
    "log_bg": "#0f172a", "log_fg": "#e2e8f0", "log_sel": "#334155", "image_bg": "#0f172a", "image_fg": "#64748b",
    "ok_box": "#ecfdf5", "ok_box_border": "#a7f3d0",
    "warn_box": "#fffbeb", "warn_box_border": "#fde68a",
    "info_box": "#eff6ff", "info_box_border": "#bfdbfe", "calm_box": "#eef2ff",
    "dot_off": "#d1d5db", "dot_dim": "#fde68a", "map_bg": "#f4f5f7", "map_btn": "#ffffff", "map_btn_border": "#999999",
    "map_label_bg": "rgba(255,255,255,190)", "map_label_fg": "#333333",
    "strong_btn": "#1f2937", "strong_btn_hover": "#111827",
    "diag_tint": {"OK": "#f0fdf4", "WARN": "#fffbeb", "FAIL": "#fef2f2"},
    # 녹화 탭 상태 줄 (배경, 글자, 테두리)
    "state": ("#f1f5f9", "#334155", "#cbd5e1"), "state_clip": ("#fef3c7", "#92400e", "#f59e0b"),
    "state_write": ("#dbeafe", "#1e3a8a", "#60a5fa"), "state_diag": ("#ede9fe", "#4c1d95", "#a78bfa"),
    "state_rec": ("#fee2e2", "#991b1b", "#f87171"),
    # 설정 행의 경로 태그 (배경, 글자)
    "tag_genicam": ("#dbeafe", "#1e40af"), "tag_node": ("#ede9fe", "#5b21b6"), "tag_pc": ("#f1f5f9", "#475569"),
    # 실행 상태 알약 (글자, 배경)
    "pill": {"stopped": ("#4b5563", "#eef0f3"), "starting": ("#92400e", "#fef3c7"),
             "running": ("#166534", "#dcfce7"), "failed": ("#991b1b", "#fee2e2")},
}

DARK = {
    "bg": "#16181d", "surface": "#1f2228", "surface2": "#252931", "hover": "#2a2f38", "pressed": "#323843",
    "border": "#2f343d", "border_strong": "#3b414c", "text": "#e5e7eb", "text2": "#cbd5e1",
    "muted": "#9aa3b2", "faint": "#6b7280", "disabled_bg": "#22262d",
    "accent": "#3b82f6", "accent_hover": "#2563eb", "accent_soft": "#1e3a5f", "accent_softer": "#1a2433",
    "accent_disabled": "#1e3a8a", "accent_disabled_text": "#93c5fd", "accent_border": "#1d4ed8",
    "ok": "#22c55e", "ok_hover": "#16a34a", "warn": "#f59e0b", "err": "#ef4444", "caution": "#f59e0b",
    "danger_text": "#f87171", "danger_border": "#7f1d1d", "danger_bg": "#2a1517",
    "gridline": "#2a2e36", "altbase": "#1c1f25", "scroll": "#3b414c", "scroll_hover": "#4b5563",
    "split": "#3b414c", "split_hover": "#3b82f6", "tool_hover": "#2a2f38", "zero": "#4b5563",
    "tooltip_bg": "#e5e7eb", "tooltip_fg": "#111827",
    "log_bg": "#0b0e14", "log_fg": "#e2e8f0", "log_sel": "#334155", "image_bg": "#0b0e14", "image_fg": "#64748b",
    "ok_box": "#0f2a1e", "ok_box_border": "#14532d",
    "warn_box": "#2a2111", "warn_box_border": "#78350f",
    "info_box": "#13233a", "info_box_border": "#1e3a8a", "calm_box": "#1a2033",
    "dot_off": "#4b5563", "dot_dim": "#78350f", "map_bg": "#1f2228", "map_btn": "#252931", "map_btn_border": "#3b414c",
    "map_label_bg": "rgba(31,34,40,200)", "map_label_fg": "#e5e7eb",
    "strong_btn": "#4b5563", "strong_btn_hover": "#6b7280",
    "diag_tint": {"OK": "#0f2a1e", "WARN": "#2a2111", "FAIL": "#2a1517"},
    "state": ("#252931", "#cbd5e1", "#3b414c"), "state_clip": ("#3a2a0c", "#fcd34d", "#b45309"),
    "state_write": ("#13233a", "#93c5fd", "#1d4ed8"), "state_diag": ("#251a3f", "#c4b5fd", "#6d28d9"),
    "state_rec": ("#3a1416", "#fca5a5", "#b91c1c"),
    "tag_genicam": ("#13233a", "#93c5fd"), "tag_node": ("#251a3f", "#c4b5fd"), "tag_pc": ("#252931", "#cbd5e1"),
    "pill": {"stopped": ("#cbd5e1", "#2a2f38"), "starting": ("#fcd34d", "#3a2a0c"),
             "running": ("#86efac", "#0f2a1e"), "failed": ("#fca5a5", "#3a1416")},
}

MODES = {"system": "시스템 따라감", "light": "라이트", "dark": "다크"}

T = dict(LIGHT)        # 지금 테마의 토큰 — apply() 가 바꾼다
DARK_MODE = False
# 다른 모듈이 ui_theme.OK 처럼 바로 쓰는 값 (apply() 가 테마 값으로 다시 넣는다)
ACCENT, OK, WARN, ERR = T["accent"], T["ok"], T["warn"], T["err"]
MUTED, TEXT, BORDER, BG = T["muted"], T["text"], T["border"], T["bg"]
PILL = T["pill"]


def c(name):
    """지금 테마의 색 토큰 (위젯에 색을 직접 줄 때)."""
    return T[name]


def solid_button(bg, hover, fg="#ffffff", radius=8):
    """채운 큰 버튼 QSS (클립 · 녹화 버튼). 비활성 색은 테마에서."""
    return (f"QPushButton {{background:{bg}; color:{fg}; border:none; border-radius:{radius}px;}}"
            f"QPushButton:hover {{background:{hover};}}"
            f"QPushButton:disabled {{background:{T['disabled_bg']}; color:{T['faint']};}}")


def outline_button(color, radius=8):
    """흰(바탕색) 바탕 + 색 테두리 큰 버튼 — 녹화 중인 [녹화 중지]."""
    return (f"QPushButton {{background:{T['surface']}; color:{T['danger_text']}; border:2px solid {color};"
            f" border-radius:{radius}px;}} QPushButton:hover {{background:{T['danger_bg']};}}"
            f"QPushButton:disabled {{color:{T['faint']}; border-color:{T['border_strong']};}}")


def chip_button():
    """켜고 끄는 알약 버튼 (도로 형상)."""
    return (f"QPushButton {{border:1px solid {T['border_strong']}; border-radius:12px; padding:3px 12px;"
            f" background:{T['surface2']};}}"
            f"QPushButton:checked {{background:{T['accent']}; color:#ffffff; border-color:{T['accent']};"
            f" font-weight:bold;}}")


def fill_bar(bar, frac, warn=0.7, err=0.85):
    """용량 막대(링 버퍼 · 메모리 · 디스크) — 찰수록 눈에 띄게: 보통은 파랑, warn 부터 주황, err 부터 빨강 + 굵은 글씨.
    단계가 바뀔 때만 스타일을 다시 건다 (매 틱 setStyleSheet 은 비싸다)."""
    level = 2 if frac >= err else 1 if frac >= warn else 0
    if bar.property("fillLevel") == level:
        return level
    bar.setProperty("fillLevel", level)
    color = (T["accent"], T["warn"], T["err"])[level]
    weight = "font-weight:bold;" if level else ""
    bar.setStyleSheet(f"QProgressBar {{ {weight} }} QProgressBar::chunk {{ background: {color}; border-radius: 4px; }}")
    return level


def system_prefers_dark():
    """GNOME 이 다크면 True (color-scheme prefer-dark, 또는 GTK 테마 이름에 dark). 못 읽으면 False."""
    for key in ("color-scheme", "gtk-theme"):
        try:
            out = subprocess.run(["gsettings", "get", "org.gnome.desktop.interface", key],
                                 capture_output=True, text=True, timeout=1.5).stdout.lower()
        except (OSError, subprocess.SubprocessError):
            continue
        if "dark" in out:
            return True
    return False


def resolve_mode(mode):
    """'system' | 'light' | 'dark' → 실제로 쓸 'light' | 'dark'."""
    if mode == "dark":
        return "dark"
    if mode == "light":
        return "light"
    return "dark" if system_prefers_dark() else "light"


# 콤보/스핀 화살표. QSS 로 테두리를 바꾸면 Fusion 기본 화살표가 깨져 그리므로 직접 준다.
_ARROW_SVG = ('<svg xmlns="http://www.w3.org/2000/svg" width="10" height="10" viewBox="0 0 10 10">'
              '<path d="{d}" fill="none" stroke="{color}" stroke-width="1.6" '
              'stroke-linecap="round" stroke-linejoin="round"/></svg>')
_ARROW_DIR = Path.home() / ".cache" / "dm_clip_gui_raw" / "theme"


def _arrow_files(color, tag):
    _ARROW_DIR.mkdir(parents=True, exist_ok=True)
    out = {}
    for name, d in (("down", "M2 3.5 L5 6.5 L8 3.5"), ("up", "M2 6.5 L5 3.5 L8 6.5")):
        path = _ARROW_DIR / f"{name}_{tag}.svg"
        path.write_text(_ARROW_SVG.format(d=d, color=color), encoding="utf-8")
        out[name] = path.as_posix()
    return out


# 칸 경계: 가운데 2px 선만 보이게 (그라디언트 트릭). 올리면 파랗게 — 끌 수 있다는 게 보여야 한다.
def _handle(axis, color):
    x2, y2 = ("1", "0") if axis == "h" else ("0", "1")
    return (f"qlineargradient(x1:0, y1:0, x2:{x2}, y2:{y2}, stop:0 transparent, "
            f"stop:0.36 transparent, stop:0.37 {color}, stop:0.63 {color}, "
            f"stop:0.64 transparent, stop:1 transparent)")


def _state(kind):
    bg, fg, border = T[kind]
    return f"background: {bg}; color: {fg}; border-color: {border};"


def _tag(kind):
    bg, fg = T[kind]
    return f"background: {bg}; color: {fg};"


def _qss(arrow_down, arrow_up):
    t = T
    return f"""
QMainWindow, QDialog {{ background: {t['bg']}; }}
/* 글자색은 팔레트(apply)가 정한다. 여기서 QWidget {{ color }} 를 주면 Qt 5.15 스타일시트가
   입력칸의 안내문(placeholder) 색까지 본문색으로 덮어서, 비어 있는 칸이 값이 든 것처럼 보인다. */

QTabWidget::pane {{ border: none; background: {t['bg']}; }}
QTabBar::tab {{
    background: transparent; border: none; border-bottom: 2px solid transparent;
    padding: 8px 18px; margin-right: 2px; color: {t['muted']}; font-weight: 600;
}}
QTabBar::tab:selected {{ color: {t['text']}; border-bottom: 2px solid {t['accent']}; }}
QTabBar::tab:hover {{ color: {t['text']}; }}

QGroupBox {{
    background: {t['surface']}; border: 1px solid {t['border']}; border-radius: 8px;
    margin-top: 16px; padding: 12px 8px 8px 8px; font-weight: 600;
}}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {t['text2']}; }}

QPushButton {{
    background: {t['surface']}; border: 1px solid {t['border_strong']}; border-radius: 6px;
    padding: 5px 12px; min-height: 18px;
}}
QPushButton:hover {{ background: {t['hover']}; }}
QPushButton:pressed {{ background: {t['pressed']}; }}
QPushButton:checked {{ background: {t['accent_soft']}; border-color: {t['accent_border']}; }}
QPushButton:disabled {{ color: {t['faint']}; background: {t['surface2']}; border-color: {t['border']}; }}
QPushButton[variant="primary"] {{
    background: {t['accent']}; border-color: {t['accent']}; color: #ffffff; font-weight: 600;
}}
QPushButton[variant="primary"]:hover {{ background: {t['accent_hover']}; }}
QPushButton[variant="primary"]:disabled {{
    background: {t['accent_disabled']}; border-color: {t['accent_disabled']}; color: {t['accent_disabled_text']};
}}
QPushButton[variant="success"] {{
    background: {t['ok']}; border-color: {t['ok']}; color: #ffffff; font-weight: 600;
}}
QPushButton[variant="success"]:hover {{ background: {t['ok_hover']}; }}
QPushButton[variant="danger"] {{ color: {t['danger_text']}; border-color: {t['danger_border']}; }}
QPushButton[variant="danger"]:hover {{ background: {t['danger_bg']}; }}
QPushButton[variant="danger"]:disabled {{ color: {t['faint']}; border-color: {t['border']}; }}
QToolButton {{ border: none; border-radius: 4px; padding: 2px 4px; color: {t['muted']}; }}
QToolButton:hover {{ background: {t['tool_hover']}; color: {t['text']}; }}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QKeySequenceEdit {{
    background: {t['surface']}; border: 1px solid {t['border_strong']}; border-radius: 5px;
    padding: 3px 6px; min-height: 20px;
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color: {t['accent']}; }}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {{
    background: {t['disabled_bg']}; color: {t['faint']};
}}

QTableWidget, QTableView, QListWidget, QTreeWidget, QTreeView {{
    background: {t['surface']}; border: 1px solid {t['border']}; border-radius: 6px;
    gridline-color: {t['gridline']}; alternate-background-color: {t['altbase']};
    selection-background-color: {t['accent_soft']}; selection-color: {t['text']};
}}
QHeaderView::section {{
    background: {t['surface2']}; border: none; border-bottom: 1px solid {t['border']};
    padding: 5px 6px; color: {t['muted']}; font-weight: 600;
}}
QTableCornerButton::section {{ background: {t['surface2']}; border: none; }}
QTextEdit, QPlainTextEdit, QTextBrowser {{
    background: {t['surface']}; border: 1px solid {t['border']}; border-radius: 6px;
}}

QProgressBar {{
    border: 1px solid {t['border']}; border-radius: 5px; background: {t['disabled_bg']};
    text-align: center; min-height: 16px;
}}
QProgressBar::chunk {{ background: {t['accent']}; border-radius: 4px; }}

QComboBox {{ padding-right: 24px; }}
QComboBox::drop-down {{
    subcontrol-origin: padding; subcontrol-position: center right; width: 22px; border: none;
}}
QComboBox::down-arrow {{ image: url({arrow_down}); width: 10px; height: 10px; }}
QComboBox QAbstractItemView {{
    background: {t['surface']}; border: 1px solid {t['border']}; outline: 0;
    selection-background-color: {t['accent_soft']}; selection-color: {t['text']};
}}
QAbstractSpinBox {{ padding-right: 20px; }}
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{
    subcontrol-origin: border; width: 18px; border: none; background: transparent;
}}
QAbstractSpinBox::up-button {{ subcontrol-position: top right; }}
QAbstractSpinBox::down-button {{ subcontrol-position: bottom right; }}
QAbstractSpinBox::up-button:hover, QAbstractSpinBox::down-button:hover {{ background: {t['tool_hover']}; }}
QAbstractSpinBox::up-arrow {{ image: url({arrow_up}); width: 8px; height: 8px; }}
QAbstractSpinBox::down-arrow {{ image: url({arrow_down}); width: 8px; height: 8px; }}

QMenuBar {{ background: {t['bg']}; }}
QMenuBar::item:selected {{ background: {t['hover']}; }}
QMenu {{ background: {t['surface']}; border: 1px solid {t['border']}; }}
QMenu::item:selected {{ background: {t['accent_soft']}; color: {t['text']}; }}
QMenu::separator {{ height: 1px; background: {t['border']}; margin: 4px 6px; }}

QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle {{ background: {t['scroll']}; border-radius: 3px; }}
QScrollBar::handle:vertical {{ min-height: 28px; }}
QScrollBar::handle:horizontal {{ min-width: 28px; }}
QScrollBar::handle:hover {{ background: {t['scroll_hover']}; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; border: none; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

QSplitter::handle:horizontal {{ width: 9px; background: {_handle("h", t['split'])}; }}
QSplitter::handle:vertical {{ height: 9px; background: {_handle("v", t['split'])}; }}
QSplitter::handle:horizontal:hover {{ background: {_handle("h", t['split_hover'])}; }}
QSplitter::handle:vertical:hover {{ background: {_handle("v", t['split_hover'])}; }}

QScrollArea {{ border: none; background: transparent; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}
QToolTip {{ background: {t['tooltip_bg']}; color: {t['tooltip_fg']}; border: none; padding: 5px 7px; }}

/* ---- 센서 기동 탭 ---- */
QFrame#Toolbar, QFrame#Panel {{
    background: {t['surface']}; border: 1px solid {t['border']}; border-radius: 10px;
}}
QFrame#SensorCard {{
    background: {t['surface']}; border: 1px solid {t['border']}; border-radius: 10px;
}}
QFrame#SensorCard:hover {{ border-color: {t['accent_border']}; }}
QFrame#SensorCard[selected="true"] {{ border: 2px solid {t['accent']}; background: {t['accent_softer']}; }}
QFrame#SensorCard[disabled="true"] {{ background: {t['surface2']}; }}
/* 실행 상태 띠 — [selected] 뒤에 와야 선택된 카드에서도 왼쪽 띠가 보인다 */
QFrame#SensorCard[run="running"] {{ border-left: 6px solid {t['ok']}; }}
QFrame#SensorCard[run="starting"] {{ border-left: 6px solid {t['warn']}; }}
QFrame#SensorCard[run="failed"] {{ border-left: 6px solid {t['err']}; }}
QLabel#RunDot {{ color: {t['dot_off']}; font-size: 12pt; }}
QLabel#RunDot[run="running"] {{ color: {t['ok']}; }}
QLabel#RunDot[run="starting"] {{ color: {t['warn']}; }}
QLabel#RunDot[run="starting"][dim="true"] {{ color: {t['dot_dim']}; }}
QLabel#RunDot[run="failed"] {{ color: {t['err']}; }}
QLabel#CardTitle {{ font-size: 11pt; font-weight: 700; }}
QLabel#CardCount {{ font-size: 22pt; font-weight: 700; }}
QLabel#CardCount[zero="true"] {{ color: {t['zero']}; }}
QLabel#CardSub, QLabel#PageSub, QLabel#Hint {{ color: {t['muted']}; }}
QLabel#CardSub[warn="true"] {{ color: {t['warn']}; }}
QLabel#PageTitle {{ font-size: 15pt; font-weight: 700; }}
QLabel#Section {{ font-weight: 700; color: {t['text2']}; }}
QLabel#Pill {{ border-radius: 9px; padding: 2px 10px; font-weight: 600; }}
QLabel#Changed {{ color: {t['accent']}; font-weight: 700; }}
QTabWidget#Inner::pane {{
    background: {t['surface']}; border: 1px solid {t['border']}; border-radius: 10px; top: -1px;
}}
QTabWidget#Inner > QTabBar::tab {{ padding: 7px 14px; }}
/* ---- 센서 미리보기 ---- */
QFrame#Tile {{ background: {t['surface']}; border: 1px solid {t['border']}; border-radius: 8px; }}
QFrame#Tile[dropTarget="true"] {{ border: 2px dashed {t['accent']}; background: {t['accent_softer']}; }}
QLabel#TileName {{ font-weight: 700; }}
QLabel#TileImage {{ background: {t['image_bg']}; border-radius: 5px; color: {t['image_fg']}; }}
QLabel#PillSmall {{ border-radius: 7px; padding: 0px 6px; font-size: 8pt; font-weight: 600; }}
QFrame#Banner {{ background: {t['ok_box']}; border: 1px solid {t['ok_box_border']}; border-radius: 10px; }}
QFrame#Warn {{ background: {t['warn_box']}; border: 1px solid {t['warn_box_border']}; border-radius: 8px; }}
QFrame#Info {{ background: {t['info_box']}; border: 1px solid {t['info_box_border']}; border-radius: 8px; }}
QPlainTextEdit#Log, QTextEdit#Log {{
    background: {t['log_bg']}; color: {t['log_fg']}; border: none; border-radius: 6px;
    selection-background-color: {t['log_sel']};
}}

/* ---- 녹화 탭: 현재 상태 줄 (kind 마다 바탕색) ---- */
QLabel#StateBar {{
    border-radius: 8px; padding: 6px 12px; font-size: 10.5pt; font-weight: 700; border: 1px solid;
    {_state("state")}
}}
QLabel#StateBar[kind="clip"] {{ {_state("state_clip")} }}
QLabel#StateBar[kind="write"] {{ {_state("state_write")} }}
QLabel#StateBar[kind="diag"] {{ {_state("state_diag")} }}
QLabel#StateBar[kind="rec"] {{ {_state("state_rec")} }}
/* 녹화 탭: GNSS 상태 점 (기동 탭 카메라 목록의 점과 같은 모양) — ok 초록 · bad 빨강(깜빡) · off 회색 */
QLabel#GnssDot {{ color: {t['dot_off']}; font-size: 12pt; }}
QLabel#GnssDot[state="ok"] {{ color: {t['ok']}; }}
QLabel#GnssDot[state="bad"] {{ color: {t['err']}; }}
QLabel#GnssDot[state="bad"][dim="true"] {{ color: {t['danger_border']}; }}
QLabel#GnssDotName {{ font-weight: 600; }}
QLabel#GnssDotName[state="off"] {{ color: {t['faint']}; }}

/* ---- 설정 탭 ---- */
QFrame#SettingsBox {{ background: {t['surface']}; border: 1px solid {t['border']}; border-radius: 8px; }}
QFrame#SettingsBox[primary="true"] {{ border-color: {t['info_box_border']}; }}
QPushButton#SectionHead {{
    font-weight: 700; color: {t['text2']}; padding: 4px 4px; text-align: left; border: none;
    border-radius: 5px; background: transparent;
}}
QPushButton#SectionHead:hover {{ background: {t['hover']}; color: {t['text']}; }}
QPushButton#SectionHead:pressed {{ background: {t['pressed']}; }}
QLabel#SubHead {{
    color: {t['muted']}; font-weight: 600; font-size: 8pt; padding-top: 6px;
    border-bottom: 1px solid {t['gridline']};
}}
QLabel#KeyLabel {{ font-family: "DejaVu Sans Mono", "Monospace"; font-size: 8.5pt; }}
QLabel#KeyLabel[unset="true"], QLabel#FieldLabel[unset="true"] {{ color: {t['faint']}; }}
QLabel#FieldLabel[caution="true"] {{ color: {t['caution']}; }}
QLabel#FieldLabel[idle="true"] {{ color: {t['faint']}; }}
/* 주요 설정 행의 경로 태그 — 값이 카메라에 어떻게 닿는가 (sensor_config.field_route) */
QLabel#RouteTag {{ border-radius: 6px; padding: 0px 5px; font-size: 7.5pt; font-weight: 600; }}
QLabel#RouteTag[route="genicam"] {{ {_tag("tag_genicam")} }}
QLabel#RouteTag[route="node"] {{ {_tag("tag_node")} }}
QLabel#RouteTag[route="pc"] {{ {_tag("tag_pc")} }}
QLineEdit[invalid="true"], QComboBox[invalid="true"] {{ border-color: {t['err']}; background: {t['danger_bg']}; }}
QLabel#BigTitle {{ font-size: 10.5pt; font-weight: 700; color: {t['text']}; }}
"""


def apply(app, mode="system"):
    """QApplication 에 테마를 건다. main() 에서 창을 만들기 전에 한 번. mode: system | light | dark."""
    global T, DARK_MODE, ACCENT, OK, WARN, ERR, MUTED, TEXT, BORDER, BG, PILL
    DARK_MODE = resolve_mode(mode) == "dark"
    T = dict(DARK if DARK_MODE else LIGHT)
    ACCENT, OK, WARN, ERR = T["accent"], T["ok"], T["warn"], T["err"]
    MUTED, TEXT, BORDER, BG = T["muted"], T["text"], T["border"], T["bg"]
    PILL = T["pill"]

    app.setStyle("Fusion")
    palette = QPalette()
    for role, color in (
            (QPalette.Window, T["bg"]), (QPalette.WindowText, T["text"]),
            (QPalette.Base, T["surface"]), (QPalette.AlternateBase, T["altbase"]),
            (QPalette.Text, T["text"]), (QPalette.Button, T["surface"]),
            (QPalette.ButtonText, T["text"]), (QPalette.Highlight, T["accent_soft"]),
            (QPalette.HighlightedText, T["text"]), (QPalette.ToolTipBase, T["tooltip_bg"]),
            (QPalette.ToolTipText, T["tooltip_fg"]), (QPalette.PlaceholderText, T["faint"]),
            (QPalette.Link, T["accent"]), (QPalette.LinkVisited, T["accent"]),
            (QPalette.BrightText, T["err"]), (QPalette.Light, T["surface2"]),
            (QPalette.Midlight, T["border"]), (QPalette.Mid, T["border_strong"]),
            (QPalette.Dark, T["border_strong"]), (QPalette.Shadow, "#000000")):
        palette.setColor(role, QColor(color))
    for role in (QPalette.Text, QPalette.WindowText, QPalette.ButtonText):
        palette.setColor(QPalette.Disabled, role, QColor(T["faint"]))
    app.setPalette(palette)
    arrows = _arrow_files(T["muted"], "dark" if DARK_MODE else "light")
    app.setStyleSheet(_qss(arrows["down"], arrows["up"]))
    lock_wheel()


# ---- 마우스 휠로 값이 바뀌지 않게 ----
# 2026-09-22 요청: 표 · 설정 목록을 휠로 내리다가 지나가는 콤보 · 스핀 · 슬라이더의 값이 바뀌는 일이 없게 (장비 표의
# 동기 방식, 설정 대화상자 숫자 칸 …). 값을 바로 보며 조정하는 곳(ISP 튜닝 탭)만 allow_wheel(위젯) 으로 연다 —
# 그 위젯 · 조상 중 가장 가까운 allow_wheel 속성이 정한다 (False 로 다시 막을 수도 있다).
# 앱 전체 이벤트 필터 대신 이 위젯 종류의 wheelEvent 만 바꾼다 — 전체 필터는 모든 이벤트(그리기 · 마우스 이동)가
# 파이썬을 거치게 해, 진단 같은 파이썬 스레드가 돌 때 GUI 가 GIL 을 기다리며 버벅인다.
# 막은 휠은 가장 가까운 스크롤 영역(표 · 목록 · 스크롤 창)에 넘긴다 — 무시(ignore)만 하면 Qt 가 프로그램이 보낸
# 휠은 부모로 안 올려 보내서, 막힌 칸 위에서는 스크롤도 안 되는 경우가 생긴다.

_WHEEL_ORIGINAL = {}


def allow_wheel(widget, allowed=True):
    """이 위젯과 그 안의 값 칸은 휠로 값을 바꿀 수 있게 (allowed=False 면 그 안에서 다시 막는다)."""
    widget.setProperty("allow_wheel", bool(allowed))


def wheel_allowed(widget):
    while widget is not None:
        allowed = widget.property("allow_wheel")
        if allowed is not None:
            return bool(allowed)
        widget = widget.parentWidget()
    return False


def forward_wheel(widget, event):
    """휠을 widget 을 감싼 가장 가까운 스크롤 영역에 넘긴다 (그 칸 위에서도 목록이 스크롤되게)."""
    area = widget.parentWidget()
    while area is not None and not isinstance(area, QAbstractScrollArea):
        area = area.parentWidget()
    if area is None:
        event.ignore()
        return
    viewport = area.viewport()
    moved = QWheelEvent(QPointF(viewport.mapFromGlobal(event.globalPos())), QPointF(event.globalPos()),
                        event.pixelDelta(), event.angleDelta(), event.buttons(), event.modifiers(),
                        event.phase(), event.inverted())
    QApplication.sendEvent(viewport, moved)
    event.accept()


def lock_wheel():
    """콤보 · 스핀 · 슬라이더 · 다이얼의 휠을 막는다 (allow_wheel 로 연 곳 제외). 위젯을 만들기 전에 한 번."""
    for cls in (QAbstractSpinBox, QComboBox, QSlider, QDial):
        if cls in _WHEEL_ORIGINAL:
            continue
        _WHEEL_ORIGINAL[cls] = cls.wheelEvent

        def wheelEvent(self, event, _cls=cls):
            if wheel_allowed(self):
                _WHEEL_ORIGINAL[_cls](self, event)
            else:
                forward_wheel(self, event)
        cls.wheelEvent = wheelEvent


def repolish(widget):
    """동적 프로퍼티를 바꾼 뒤 QSS를 다시 먹인다 (안 하면 [selected="true"] 가 안 바뀐다)."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


def set_variant(button, variant):
    button.setProperty("variant", variant)
    repolish(button)


def pill_css(state):
    fg, bg = PILL.get(state, PILL["stopped"])
    return f"color:{fg}; background:{bg};"


def fit_to_screen(window, width, height, margin=40):
    """원하는 크기로 열되 화면보다 크면 줄인다.

    GPU 드라이버가 빠져 800x600 프레임버퍼로 떠 있던 적이 있다 — 그때 1000x900 창은
    아래쪽 버튼이 화면 밖으로 나간다.
    """
    screen = QGuiApplication.primaryScreen()
    if screen is None:
        window.resize(width, height)
        return
    avail = screen.availableGeometry()
    window.resize(min(width, avail.width() - margin), min(height, avail.height() - margin))
