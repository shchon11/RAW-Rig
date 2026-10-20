"""Measured electron PTC (Janesick style): every noise term measured on its own, in RMS electrons.

  python electron_ptc.py bfs            # all gains found (main PTC session + low-light session)
  python electron_ptc.py lucid          # HDR and HDRChannel0..3 (low-light + transition sessions)

Per level, on a fixed ROI (the PTC ROI, 256 x 256) and per CFA position:
  S       mean(bright) - mean(matched dark), / alpha                       [e-]
  v_t     temporal variance of the bright stack (per pixel over frames, ROI mean; light ripple removed)
  v_d     temporal variance of the dark stack at the same exposure ("matched dark")
  shot    v_t - v_d  ("measured positive variance difference"; compare with sqrt(S))
  v_s     spatial variance of the per-pixel mean (bright - dark) minus the noise left in those means
          (v_t/n_b + v_d/n_d): pixel response non-uniformity + optics/illumination (+ scene texture)
  total   v_t + v_s
  read    sqrt(v_d) at the shortest exposure
Levels without a matched dark stack are skipped. Writes plots/electron_ptc/<name>.png and a json table.
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "plots/electron_ptc"
POS = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}
ROI = 256
plt.rcParams.update({"font.size": 10, "axes.unicode_minus": False})


def level_terms(bright, dark_mean, dark_var, n_dark, alpha):
    """bright: (n, h, w) ROI stack; dark_mean/var: (h, w). Returns per-CFA dict of terms in e- (variances e-^2)."""
    x = bright.astype(np.float64) - dark_mean
    n = x.shape[0]
    out = {}
    for c, (dy, dx) in POS.items():
        xc = x[:, dy::2, dx::2]
        m = xc.mean()
        if m > 50:                                            # light ripple: per-frame factor, only where measurable
            g = xc.reshape(n, -1).mean(1)
            xc = xc / (g / g.mean())[:, None, None]
        mu = xc.mean(0)
        vt = xc.var(0, ddof=1).mean()
        vd = dark_var[dy::2, dx::2].mean()
        vs = mu.var() - vt / n - vd / n_dark
        a = alpha[c]
        out[c] = {"S": mu.mean() / a, "vt": vt / a ** 2, "vd": vd / a ** 2, "vs": vs / a ** 2}
    return out


def bfs_sources():
    cal = json.loads((ROOT / "calibration/BFS-PGE-23S3C_26315393/noise_model.json").read_text())["gains"]
    rows = {}
    sess = ROOT / "data/ptc_20261019_2131_bfs"
    man = json.loads((sess / "manifest.json").read_text())["outputs"]
    for name, o in man.items():
        g = int(round(o["gain_db"]))
        alpha = cal[str(g)]["alpha_dn_per_e"]
        for b, d in zip(o["bright"], o["dark"]):
            st = np.load(sess / name / b["file"])["stack"]
            ds = np.load(sess / name / d["file"])["stack"].astype(np.float64)
            t = level_terms(st, ds.mean(0), ds.var(0, ddof=1), ds.shape[0], alpha)
            rows.setdefault(g, []).append((b["exposure_us"], "PTC 10-19", t))
    x0, y0 = 832, 472                                          # same ROI cut from the full-frame low-light stacks
    for f in sorted((ROOT / "data/lowlight_20261020/bright/bfs").glob("g*_t*us.npz")):
        dpath = ROOT / "data/lowlight_20261020/dark/bfs" / f.name
        if not dpath.with_suffix(".json").exists():
            continue
        info = json.loads(f.with_suffix(".json").read_text())
        g = int(round(info["gain_db"]))
        st = np.load(f)["frames"][:, y0:y0 + ROI, x0:x0 + ROI]
        if st.max() >= 65500:
            continue
        dz = np.load(dpath)
        dinfo = json.loads(dpath.with_suffix(".json").read_text())
        t = level_terms(st, dz["mean"][y0:y0 + ROI, x0:x0 + ROI].astype(np.float64),
                        dz["var"][y0:y0 + ROI, x0:x0 + ROI].astype(np.float64), dinfo["n"], cal[str(g)]["alpha_dn_per_e"])
        rows.setdefault(g, []).append((info["exposure_us"], "low light 10-20", t))
    return {f"BFS_g{g:02d}": (f"Measured electron PTC | 26315393 | Gain {g}dB", r) for g, r in rows.items()}


def lucid_sources():
    cal = json.loads((ROOT / "calibration/TRI054S-C_254600231/noise_model.json").read_text())["gains"]
    ch_alpha = {0: 9.627, 1: 0.4524, 2: 0.964, 3: 0.0261}       # channel PTC 2026-10-19 (DN per own electron)
    rows = {}
    for sess, label in ((ROOT / "data/lowlight_20261020/lucid", "low light"),
                        (ROOT / "data/hdr_transition_20261020", "transition"),
                        (ROOT / "data/hdr_transition_20261020_g12", "transition 12 dB")):
        mp = sess / "manifest.json"
        if not mp.exists():
            continue
        man = json.loads(mp.read_text())
        g = int(round(man.get("gain_db", 0)))
        for key, lv in man["levels"].items():
            for tag in ["hdr"] + [f"hdrchannel{c}" for c in range(4)]:
                if tag not in lv.get("dark", {}) or not (sess / key / f"bright_{tag}.npy").exists():
                    continue
                bs = np.load(sess / key / f"bright_{tag}.npy", mmap_mode="r")
                H, W = bs.shape[1:]
                y0, x0 = (H - ROI) // 2 // 2 * 2, (W - ROI) // 2 // 4 * 4
                st = np.asarray(bs[:, y0:y0 + ROI, x0:x0 + ROI])
                clip = 2 ** 24 - 1 if tag == "hdr" else 4351
                if st.max() >= clip:
                    continue
                ds = np.load(sess / key / f"dark_{tag}.npy", mmap_mode="r")[:, y0:y0 + ROI, x0:x0 + ROI].astype(np.float64)
                if tag == "hdr":
                    alpha, name = cal[str(g)]["alpha_dn_per_e"], f"LUCID_HDR_g{g:02d}"
                    title = f"Measured electron PTC | 254600231 | HDR output (SP1 e-) | Gain {g}dB"
                else:
                    c = int(tag[-1])
                    alpha, name = {k: ch_alpha[c] for k in POS}, f"LUCID_ch{c}"
                    title = f"Measured electron PTC | 254600231 | HDRChannel{c} | Gain 0dB"
                t = level_terms(st, ds.mean(0), ds.var(0, ddof=1), ds.shape[0], alpha)
                rows.setdefault(name, (title, []))[1].append((lv["dark"][tag]["exposure_us"], label, t))
    return rows


def plot(name, title, rows):
    rows = sorted(rows, key=lambda r: r[2]["G1"]["S"])
    tmin = min(r[0] for r in rows)
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
    table = {}
    for ax, c in zip(axes.ravel(), POS):
        S = np.array([r[2][c]["S"] for r in rows])
        vt = np.array([r[2][c]["vt"] for r in rows])
        vd = np.array([r[2][c]["vd"] for r in rows])
        vs = np.array([r[2][c]["vs"] for r in rows])
        T = np.array([r[0] for r in rows])
        read = np.sqrt(np.mean(vd[T <= tmin * 1.01]))
        ok = S > 0
        shot = vt - vd
        tot = vt + np.maximum(vs, 0)
        src = np.array([r[1] for r in rows])
        # spatial term depends on each session's illumination / scene: one line per session (solid, dashed, ...)
        for k, sname in enumerate(dict.fromkeys(src)):
            m = ok & (src == sname)
            ls = ["-", "--", ":", "-."][k % 4]
            ax.plot(S[m], np.sqrt(tot[m]), "o", ls=ls, color="#1f6fc5", ms=4,
                    label="Total: temporal + spatial" + (f" [{sname}]" if len(set(src)) > 1 else ""))
            ss = m & (vs > 0)
            ax.plot(S[ss], np.sqrt(vs[ss]), "x", ls=ls, color="#d62728", ms=5,
                    label="Spatial FPN + optics/illumination" + (f" [{sname}]" if len(set(src)) > 1 else ""))
        ax.plot(S[ok], np.full(ok.sum(), read), "d-", color="#17becf", ms=4, label="Read (shortest dark)")
        ax.plot(S[ok], np.sqrt(vd[ok]), ".-", color="#888888", ms=4, label="Matched dark incl. exposure")
        sp = ok & (shot > 0)
        ax.plot(S[sp], np.sqrt(shot[sp]), "+-", color="#d62fc0", ms=6, label="Measured positive variance difference")
        Sr = np.geomspace(S[ok].min(), S[ok].max(), 100)
        ax.plot(Sr, np.sqrt(Sr), "--", color="#d62fc0", lw=1, label="Poisson reference sqrt(signal e-)")
        ax.set_xscale("log"); ax.set_yscale("log"); ax.grid(True, which="both", alpha=0.3)
        ax.set_xlabel("Measured signal / PTC gain [e-]"); ax.set_ylabel("Noise RMS [e-]"); ax.set_title(c)
        ax.legend(fontsize=7, loc="upper left")
        table[c] = {"read_e": float(read), "rows": [{"exposure_us": float(r[0]), "source": r[1], **{k: float(v) for k, v in r[2][c].items()}}
                                                    for r in rows]}
    fig.suptitle(title, fontsize=14)
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / f"{name}.png", dpi=110)
    plt.close(fig)
    (OUT / f"{name}.json").write_text(json.dumps(table, indent=1))
    print(f"{name}: {len(rows)} levels, read {table['G1']['read_e']:.2f} e- (G1) -> {OUT / (name + '.png')}")


def main():
    srcs = bfs_sources() if sys.argv[1] == "bfs" else lucid_sources()
    for name, (title, rows) in sorted(srcs.items()):
        if rows:
            plot(name, title, rows)


if __name__ == "__main__":
    main()
