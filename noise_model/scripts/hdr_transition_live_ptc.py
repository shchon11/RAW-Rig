"""Live PTC of the HDR transition sessions: combined HDR output and HDRChannel0..3, filled level by level.

  python hdr_transition_live_ptc.py            # process new complete levels, redraw the figure
  python hdr_transition_live_ptc.py --watch    # repeat every 30 s, print one line per new level

Per level and output: per-pixel temporal mean and variance (per-frame light factor removed),
binned by mean per CFA colour on the paper region (central 60 % of the frame), hot/noisy pixels
excluded, outliers (> 6x bin median variance) dropped. Means use a provisional dark until the
lens-cap darks of the session exist: HDR = calibrated dark map at that exposure (maps_gNN.npz),
channels = pedestal 256. Variance does not depend on the dark.
Binned results are cached per level in <session>/live_ptc/<level>.npz.
"""
import argparse
import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter, LogLocator, NullFormatter

from noise_budget import BLEND, bfs_budget, lucid_budget

ROOT = Path(__file__).resolve().parent.parent
SESSIONS = [ROOT / "data/hdr_transition_20261020", ROOT / "data/hdr_transition_20261020_g12",
            ROOT / "data/lowlight_20261020/lucid"]
LABELS = {"hdr_transition_20261020": "전환 0 dB", "hdr_transition_20261020_g12": "전환 12 dB", "lucid": "저조도 0 dB"}
MARKERS = {"hdr_transition_20261020": dict(marker="o"), "hdr_transition_20261020_g12": dict(marker="^", mfc="none", ms=4),
           "lucid": dict(marker="s", ms=2.5)}
CAL = ROOT / "calibration/TRI054S-C_254600231"
OUT_PNG = ROOT / "plots/hdr_transition_live_ptc.png"
BFS_DIR = ROOT / "data/lowlight_20261020/bright/bfs"
BFS_DARK = ROOT / "data/lowlight_20261020/dark/bfs"
BFS_CAL = ROOT / "calibration/BFS-PGE-23S3C_26315393"
OUT_BFS = ROOT / "plots/bfs_lowlight_live_ptc.png"
CH_CLIP = 4351
HDR_CLIP = 2 ** 24 - 1
BLOCK = 116
BINS = np.geomspace(1, 3e7, 7 * 16 + 1)          # 16 bins per decade
COLORS = {"R": (0, 0), "G": None, "B": (1, 1)}
# channel models from the PTC of 2026-10-19 (DN of the channel output): alpha DN/e-, read noise DN
COMP_HUE = {"photon shot": "#4c78a8", "read, stays at high gain": "#f58518", "read, shrinks with gain": "#e45756",
            "quantization": "#72b7b2", "row noise": "#b279a2", "column noise": "#9d755d", "dark current": "#54a24b",
            "SP1 shot": "#4c78a8", "SP2 shot": "#6baed6", "ch0 read": "#f58518", "ch1 read": "#e45756",
            "ch2 read": "#b279a2", "ch3 read": "#9d755d", "row/col": "#bab0ac"}
CH_MODEL = {0: (9.627, 0.53 * 9.627), 1: (0.4524, 2.46 * 0.4524), 2: (0.964, 13.5 * 0.964), 3: (0.0261, 100 * 0.0261)}

plt.rcParams.update({"font.family": "NanumSquare", "axes.unicode_minus": False, "font.size": 9,
                     "mathtext.fontset": "dejavusans"})


def plain_log_ticks(ax):
    """Log-axis labels as plain text (NanumSquare has no U+2212 for the mathtext minus)."""
    fmt = FuncFormatter(lambda v, _: f"{v:g}" if 1e-3 <= v < 1e5 else f"1e{int(round(np.log10(v)))}")
    for axis in (ax.xaxis, ax.yaxis):
        if axis.get_scale() == "log":
            axis.set_major_locator(LogLocator(base=10))
            axis.set_major_formatter(fmt)
            axis.set_minor_formatter(NullFormatter())


def cfa_masks(H, W):
    yy, xx = np.mgrid[0:H, 0:W]
    return {"R": (yy % 2 == 0) & (xx % 2 == 0), "G": (yy % 2) != (xx % 2), "B": (yy % 2 == 1) & (xx % 2 == 1)}


def frame_gains(st, dark, rows, cols, lo, hi):
    n = st.shape[0]
    x = st[:, rows, cols].astype(np.float64) - dark[rows, cols]
    m = x.mean(0)
    sel = (m > lo) & (m < hi)
    if sel.sum() < 1000:
        return np.ones(n)
    g = x[:, sel].mean(1)
    return g / g.mean()


def level_stats(stack_path, dark, gains_rc, lo, hi, clip, bad):
    """Binned (mean, var) per colour for one stack; returns dict colour -> arrays."""
    st = np.load(stack_path, mmap_mode="r")
    n, H, W = st.shape
    r0, r1, c0, c1 = H // 5 // 2 * 2, 4 * H // 5 // 2 * 2, W // 5 // 2 * 2, 4 * W // 5 // 2 * 2
    g = frame_gains(st, dark, slice(r0, r1, 3), slice(c0, c1, 3), lo, hi)
    # split half: bin on the mean of the even frames, report mean and variance of the odd frames
    # (binning on the same noisy mean pushes low-signal points to the left: errors-in-variables)
    sels, mus, vrs, oks = [], [], [], []
    for r in range(r0, r1, BLOCK):
        rr = slice(r, min(r + BLOCK, r1))
        raw = st[:, rr, c0:c1]
        x = (raw.astype(np.float64) - dark[rr, c0:c1]) / g[:, None, None]
        sels.append(x[0::2].mean(0))
        mus.append(x[1::2].mean(0))
        vrs.append(x[1::2].var(0, ddof=1))
        oks.append(~(raw >= clip).any(0) & ~bad[rr, c0:c1])
    msel, mu, var, ok = np.concatenate(sels), np.concatenate(mus), np.concatenate(vrs), np.concatenate(oks)
    masks = cfa_masks(*mu.shape)
    out = {"n_frames": n, "ripple": float(g.std())}
    for cname, m in masks.items():
        sel = m & ok & (msel > 0)
        mv, vv = mu[sel], var[sel]
        idx = np.digitize(msel[sel], BINS)
        rows = []
        for k in np.unique(idx):
            s = idx == k
            if s.sum() < 200:
                continue
            v = vv[s]
            keep = v < 6 * np.median(v)
            v, mk = v[keep], mv[s][keep]
            rows.append((mk.mean(), v.mean(), v.std() / np.sqrt(len(v)), len(v)))
        out[cname] = np.array(rows) if rows else np.zeros((0, 4))
    return out


def complete_levels(sess):
    mpath = sess / "manifest.json"
    if not mpath.exists():
        return {}
    man = json.loads(mpath.read_text())
    want = ["hdr"] + ([] if man.get("hdr_only") else [f"hdrchannel{c}" for c in range(4)])
    return {k: lv for k, lv in man["levels"].items()
            if all(t in lv.get("bright", {}) for t in want) and (sess / k / "bright_hdr.npy").exists()}, man


def process(sess):
    res = complete_levels(sess)
    if not res:
        return [], None
    levels, man = res
    gain = float(man.get("gain_db", 0.0))
    maps = np.load(CAL / f"maps_g{int(round(gain)):02d}.npz")
    bad = maps["hot"] | maps["noisy"]
    cache = sess / "live_ptc"
    cache.mkdir(exist_ok=True)
    new = []
    for key, lv in sorted(levels.items(), key=lambda kv: kv[1]["exposure_us"]):
        cfile = cache / f"{key}.npz"
        has_dark = (sess / key / "dark_hdr.npy").exists()
        if cfile.exists() and (not has_dark or bool(np.load(cfile)["measured_dark"])):
            continue
        T = lv["exposure_us"]
        out = {"exposure_us": T, "measured_dark": has_dark}
        if has_dark and not all(t in lv.get("dark", {}) for t in ("hdr",)):
            continue                                      # dark stack still being written
        if has_dark:
            d = np.load(sess / key / "dark_hdr.npy", mmap_mode="r")
            dh = d.astype(np.float64).mean(0)
        else:
            dh = maps["offset_1ms_dn"].astype(np.float64) + maps["dark_rate_dn_per_s"] * (T - 1000) / 1e6
        s = level_stats(sess / key / "bright_hdr.npy", dh, None, 2e3, 1.2e7, HDR_CLIP, bad)
        out["ripple_hdr"] = s.pop("ripple"); s.pop("n_frames")
        for cname, arr in s.items():
            out[f"hdr_{cname}"] = arr
        for c in range(4):
            p = sess / key / f"bright_hdrchannel{c}.npy"
            if not p.exists():
                continue
            dp = sess / key / f"dark_hdrchannel{c}.npy"
            dc = np.load(dp, mmap_mode="r").astype(np.float64).mean(0) if dp.exists() else np.full(dh.shape, 256.0)
            s = level_stats(p, dc, None, 200, 3500, CH_CLIP, bad)
            s.pop("ripple"); s.pop("n_frames")
            for cname, arr in s.items():
                out[f"ch{c}_{cname}"] = arr
        np.savez(cfile, **out)
        new.append(key)
        break                                     # one level per call: redraw after every level
    return new, gain


def hdr_model(gain):
    m = json.loads((CAL / "noise_model.json").read_text())["gains"][str(int(round(gain)))]
    S = np.asarray(m["alpha_table"]["S_by_cfa"]["G1"], float)
    a = np.asarray(m["alpha_table"]["alpha"]["G1"], float)
    return S, a * S + m["read_noise_dn"]["G1"] ** 2


def draw():
    fig, axes = plt.subplots(2, 6, figsize=(26.5, 8.5), constrained_layout=True)
    hdr_pts = []
    hue = {"R": "#d1495b", "G": "#2e8b57", "B": "#3a6ea5"}
    done = {}
    for si, sess in enumerate(SESSIONS):
        cache = sess / "live_ptc"
        if not cache.exists():
            continue
        man = json.loads((sess / "manifest.json").read_text())
        gain = float(man.get("gain_db", 0.0))
        k = 10 ** (gain / 20)
        files = sorted(cache.glob("*.npz"), key=lambda p: float(np.load(p)["exposure_us"]))
        done[sess.name] = [(p.stem, bool(np.load(p)["measured_dark"])) for p in files]
        for p in files:
            d = np.load(p)
            for col, out in enumerate(["hdr", "ch0", "ch1", "ch2", "ch3"]):
                for cname in "RGB":
                    key = f"{out}_{cname}"
                    if key not in d.files or not len(d[key]):
                        continue
                    a = d[key]
                    S, V = a[:, 0] / (k if out == "hdr" else 1), a[:, 1] / (k * k if out == "hdr" else 1)
                    mk = dict(marker="o", ms=3, ls="none", alpha=0.75)
                    mk.update(MARKERS[sess.name])
                    axes[0, col].plot(S, V, color=hue[cname], **mk)
                    if out == "hdr" and cname == "G":
                        hdr_pts.append(np.stack([S, V], 1))
                    axes[1, col].plot(S, V / S, color=hue[cname], **mk)
    # reference models
    S, V = hdr_model(0)
    axes[0, 0].plot(S, V, "k--", lw=1, label="기존 모델 (G, 0 dB)")
    axes[1, 0].plot(S, V / S, "k--", lw=1)
    for c, (al, rd) in CH_MODEL.items():
        S = np.geomspace(1, 4095, 200)
        axes[0, c + 1].plot(S, al * S + rd ** 2, "k--", lw=1, label=f"PTC 모델 a={al:g}, read={rd:.2f} DN")
        axes[1, c + 1].plot(S, (al * S + rd ** 2) / S, "k--", lw=1)
    # noise components of the combined output (model, G, 0 dB) with the measured G points
    Sc = np.geomspace(3, 1.5e7, 700)
    comp, _ = lucid_budget(Sc, 10000, json.loads(BLEND.read_text()))
    tot = sum(comp.values())
    for k, v in comp.items():
        if v.max() <= 0:
            continue
        v = np.where(v > 0, v, np.nan)
        axes[0, 5].plot(Sc, v, color=COMP_HUE[k], lw=1.4, label=k)
        axes[1, 5].plot(Sc, v / Sc, color=COMP_HUE[k], lw=1.4)
    if hdr_pts:
        P = np.concatenate(hdr_pts)
    axes[0, 5].plot(Sc, tot, "k--", lw=1.2, label="성분 합계 (모델)")
    axes[1, 5].plot(Sc, tot / Sc, "k--", lw=1.2)
    if hdr_pts:
        P = np.concatenate(hdr_pts)
        mkp = dict(ls="none", marker="o", ms=3, mfc="none", mec="#d62728", mew=0.6, zorder=5)
        axes[0, 5].plot(P[:, 0], P[:, 1], label="측정 (G, 전 세션)", **mkp)
        axes[1, 5].plot(P[:, 0], P[:, 1] / P[:, 0], **mkp)
    axes[0, 5].set_ylim(0.05, 3e9)
    axes[1, 5].set_ylim(1e-3, 3e3)
    titles = ["합성 HDR (24-bit)", "ch0 SP1 HCG", "ch1 SP1 LCG", "ch2 SP2 HCG", "ch3 SP2 LCG",
              "합성 HDR 성분별 (모델, G, 0 dB, 노출 10 ms)"]
    for col in range(1, 5):
        if not axes[1, col].lines[1:]:
            axes[1, col].text(0.5, 0.5, "이 조명에서는 전 픽셀 포화\n(저조도 촬영에서 채움)", transform=axes[1, col].transAxes,
                              ha="center", va="center", color="#777777")
    for col in range(6):
        for row in range(2):
            ax = axes[row, col]
            ax.set_xscale("log"); ax.set_yscale("log"); ax.grid(True, which="both", alpha=0.25)
            ax.set_xlabel("signal (DN, dark subtracted)")
            plain_log_ticks(ax)
        axes[0, col].set_title(titles[col])
        axes[0, col].set_ylabel("temporal variance (DN^2)")
        axes[1, col].set_ylabel("variance / signal (DN)")
        axes[0, col].legend(fontsize=7 if col == 5 else 8, loc="upper left")
    for cname in "RGB":
        axes[1, 0].plot([], [], "o", color=hue[cname], label=f"{cname}")
    for name in done:
        extra = " (DN / 3.98, DN^2 / 3.98^2)" if name.endswith("g12") else ""
        axes[1, 0].plot([], [], ls="none", color="gray", **{**dict(marker="o", ms=3), **MARKERS[name]},
                        label=LABELS[name] + extra)
    axes[1, 0].legend(fontsize=8, loc="upper left")
    allv = [m for v in done.values() for _, m in v]
    prov = not all(allv)
    head = "  |  ".join(f"{LABELS[n]} {len(v)}단계" + (f" ({v[0][0]}~{v[-1][0]})" if v else "") for n, v in done.items())
    fig.suptitle(f"LUCID IMX490 HDR 전환 구간 PTC (2026-10-20)  |  {head}" +
                 ("  |  평균은 임시 dark로 보정 (분산은 확정)" if prov else "  |  렌즈캡 dark로 보정 완료"), fontsize=11)
    OUT_PNG.parent.mkdir(exist_ok=True)
    fig.savefig(OUT_PNG, dpi=110)
    plt.close(fig)


def process_bfs():
    """BFS low-light stacks (fullframe_dark.py output): per-pixel stats in electrons, binned per colour."""
    if not BFS_DIR.exists():
        return []
    cache = BFS_DIR.parent / "bfs_live"
    cache.mkdir(exist_ok=True)
    model = json.loads((BFS_CAL / "noise_model.json").read_text())["gains"]
    new = []
    for f in sorted(BFS_DIR.glob("g*_t*us.npz")):
        if not f.with_suffix(".json").exists():          # json is written after the npz: level not finished
            continue
        info = json.loads(f.with_suffix(".json").read_text())
        cfile = cache / f.name
        dpath = BFS_DARK / f.name
        dark_ready = dpath.exists() and dpath.with_suffix(".json").exists()      # json is written last
        if cfile.exists() and (not dark_ready or bool(np.load(cfile)["measured_dark"])):
            continue
        g, T = int(round(info["gain_db"])), info["exposure_us"]
        maps = np.load(BFS_CAL / f"maps_g{g:02d}.npz")
        alpha = model[str(g)]["alpha_dn_per_e"]["G1"]
        if dark_ready:
            dark = np.load(dpath)["mean"].astype(np.float64)
        else:
            dark = maps["offset_1ms_dn"] + maps["dark_rate_dn_per_s"] * (T - 1000) / 1e6
        st = np.load(f)["frames"]
        H, W = st.shape[1:]
        r0, r1, c0, c1 = H // 5 // 2 * 2, 4 * H // 5 // 2 * 2, W // 5 // 2 * 2, 4 * W // 5 // 2 * 2
        x = st[:, r0:r1, c0:c1].astype(np.float64) - dark[r0:r1, c0:c1]
        m = x.mean(0)
        sel = m > 50
        if sel.sum() > 1000:                              # light factor only where there is light to measure
            gf = x[:, sel].mean(1)
            x /= (gf / gf.mean())[:, None, None]
        msel = x[0::2].mean(0) / alpha                     # split half (see level_stats)
        mu, var = x[1::2].mean(0) / alpha, x[1::2].var(0, ddof=1) / alpha ** 2
        bad = (maps["hot"] | maps["noisy"])[r0:r1, c0:c1] | (st[:, r0:r1, c0:c1] >= 65500).any(0)   # + saturated
        out = {"exposure_us": T, "gain_db": g, "measured_dark": dark_ready, "alpha": alpha}
        for cname, msk in cfa_masks(*mu.shape).items():
            mv, vv = mu[msk & ~bad], var[msk & ~bad]
            edges = np.concatenate([np.linspace(-2, 2, 17), np.geomspace(2.2, 2e4, 80)])
            idx = np.digitize(msel[msk & ~bad], edges)
            rows = []
            for k in np.unique(idx):
                sk = idx == k
                if sk.sum() < 200:
                    continue
                v = vv[sk]
                keep = v < 6 * np.median(v)
                rows.append((mv[sk][keep].mean(), v[keep].mean(), v[keep].std() / np.sqrt(keep.sum()), keep.sum()))
            out[cname] = np.array(rows) if rows else np.zeros((0, 4))
        np.savez(cfile, **out)
        new.append(f.stem)
        break
    return new


def draw_bfs():
    cache = BFS_DIR.parent / "bfs_live"
    files = sorted(cache.glob("*.npz"), key=lambda p: (int(np.load(p)["gain_db"]), float(np.load(p)["exposure_us"])))
    if not files:
        return
    model = json.loads((BFS_CAL / "noise_model.json").read_text())["gains"]
    fig, axes = plt.subplots(1, 3, figsize=(21, 5.5), constrained_layout=True)
    hue = {"R": "#d1495b", "G": "#2e8b57", "B": "#3a6ea5"}
    mk = {0: dict(marker="o", ms=3), 24: dict(marker="^", mfc="none", ms=4)}
    bb = bfs_budget(10000)
    Sc = np.geomspace(0.05, 3e4, 300)
    for g, ls in ((0, "-"), (24, "--")):
        b = bb[str(g)]
        parts = {"photon shot": Sc, "read, stays at high gain": b["pre"], "read, shrinks with gain": b["post_white"],
                 "quantization": b["quant"], "row noise": b["row"], "column noise": b["col"], "dark current": b["dark"]}
        tot = np.zeros_like(Sc)
        for k, v in parts.items():
            v = np.broadcast_to(np.asarray(v, float), Sc.shape)
            tot = tot + v
            if v.max() < 1e-3:
                continue
            axes[2].plot(Sc, v, color=COMP_HUE[k], ls=ls, lw=1.3, label=k if g == 0 else None)
        axes[2].plot(Sc, tot, color="k", ls=ls, lw=1.8, label=f"성분 합계 {g} dB")
    done = {}
    for p in files:
        d = np.load(p)
        g = int(d["gain_db"])
        done.setdefault(g, []).append(f"{float(d['exposure_us']):.0f}")
        for cname in "RGB":
            a = d[cname]
            if not len(a):
                continue
            pos = a[:, 0] > 0.05
            axes[0].plot(a[pos, 0], a[pos, 1], ls="none", color=hue[cname], alpha=0.75, **mk[g])
            if cname == "G":
                axes[2].plot(a[pos, 0], a[pos, 1], ls="none", color="#555555", alpha=0.6, **mk[g])
            axes[1].plot(a[pos, 0], a[pos, 1] / a[pos, 0], ls="none", color=hue[cname], alpha=0.75, **mk[g])
    S = np.geomspace(0.05, 3e4, 300)
    for g, ls in ((0, "--"), (24, ":")):
        if g not in done:
            continue
        r = model[str(g)]["read_noise_e"]["G1"]
        axes[0].plot(S, S + r ** 2, "k", ls=ls, lw=1, label=f"모델 {g} dB: 분산 = S + {r:.2f}^2 (e-)")
        axes[1].plot(S, (S + r ** 2) / S, "k", ls=ls, lw=1)
    for ax in axes:
        ax.set_xscale("log"); ax.set_yscale("log"); ax.grid(True, which="both", alpha=0.25)
        ax.set_xlabel("signal (e-, dark subtracted)")
        plain_log_ticks(ax)
    axes[0].set_ylabel("temporal variance (e-^2)")
    axes[1].set_ylabel("variance / signal")
    for cname in "RGB":
        axes[1].plot([], [], "o", color=hue[cname], label=cname)
    for g in done:
        axes[1].plot([], [], ls="none", color="gray", label=f"{g} dB", **mk[g])
    axes[0].legend(fontsize=8, loc="upper left"); axes[1].legend(fontsize=8, loc="upper right")
    axes[2].set_ylim(1e-3, 5e4)
    axes[2].set_ylabel("temporal variance (e-^2)")
    axes[2].set_title("성분별 PTC (모델; 실선 0 dB, 점선 24 dB, 노출 10 ms; 회색 = 측정 G)")
    axes[2].legend(fontsize=7, loc="upper left")
    prov = any(not bool(np.load(p)["measured_dark"]) for p in files)
    fig.suptitle("BFS IMX392 저조도 PTC (2026-10-20)  |  " +
                 "  |  ".join(f"{g} dB {len(v)}단계 ({v[0]}~{v[-1]} us)" for g, v in done.items()) +
                 ("  |  평균은 임시 dark로 보정 (분산은 확정)" if prov else "  |  렌즈캡 dark로 보정 완료"), fontsize=11)
    fig.savefig(OUT_BFS, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", action="store_true")
    a = ap.parse_args()
    for fn in (draw, draw_bfs):
        try:
            fn()
        except Exception as e:                            # nothing cached yet
            print("initial draw skipped:", fn.__name__, e, flush=True)
    while True:
        new = []
        for sess in SESSIONS:
            got, gain = process(sess)
            new += [f"{LABELS[sess.name]}:{k}" for k in got]
        if new:
            draw()
            print(f"{time.strftime('%H:%M:%S')} PTC updated: {' '.join(new)} -> {OUT_PNG}", flush=True)
        got = process_bfs()
        if got:
            draw_bfs()
            print(f"{time.strftime('%H:%M:%S')} PTC updated: BFS:{' '.join(got)} -> {OUT_BFS}", flush=True)
        new += got
        if not a.watch:
            if not new:
                draw()
                break
            continue
        if not new:
            time.sleep(30)


if __name__ == "__main__":
    main()
