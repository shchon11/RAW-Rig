"""Figures for the calibration report page -> data/plots/report/*.png"""
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
D = ROOT / "data"
OUT = D / "plots" / "report"
OUT.mkdir(parents=True, exist_ok=True)
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, MUTED, GRID, BG = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({"font.family": "NanumSquare", "axes.unicode_minus": False, "font.size": 10,
                     "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.spines.top": False, "axes.spines.right": False, "figure.facecolor": BG, "axes.facecolor": BG,
                     "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "legend.frameon": False})
LUC, BFS = D / "ptc_20261019_2131_lucid" / "analysis", D / "ptc_20261019_2131_bfs" / "analysis"
GAINS = [0, 6, 10, 12, 18, 24]


def load(p):
    return json.loads(Path(p).read_text())


def g_avg(res, field):
    return np.mean([[lv[field] if lv[field] is not None else np.nan for lv in res["cfa"][c]["levels"]] for c in ("G1", "G2")], 0)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / f"{name}.png", dpi=110)
    plt.close(fig)


# 1. BFS PTC by gain
fig, ax = plt.subplots(figsize=(8, 5))
for k, g in enumerate(GAINS):
    r = load(BFS / f"{'BFS' if g == 0 else f'BFS_g{g:02d}'}.json")
    S, vb = g_avg(r, "S"), g_avg(r, "vb")
    ok = S > 0
    ax.plot(S[ok], np.sqrt(vb[ok]), "-o", color=C[k], lw=2, ms=4, label=f"{g} dB")
    a = np.mean([r["cfa"][c]["alpha_dn_per_e"] for c in ("G1", "G2")])
    vd = np.mean([r["cfa"][c]["dark_var_dn2_tmin"] for c in ("G1", "G2")])
    xs = np.geomspace(S[ok].min(), S[ok].max(), 60)
    ax.plot(xs, np.sqrt(a * xs + vd), "--", color=C[k], lw=1, alpha=0.7)
ax.set(xscale="log", yscale="log", xlabel="신호 S (DN, 검은 레벨 위)", ylabel="시간 노이즈 σ (DN)",
       title="BFS (IMX392) PTC — gain별 실측(점·선)과 모델(점선)")
ax.legend(title="gain", ncol=2)
save(fig, "01_bfs_ptc")

# 2. LUCID channels in electrons
fig, ax = plt.subplots(figsize=(8, 5))
names = {"HDRChannel0": "ch0 (SP1 HCG)", "HDRChannel1": "ch1 (SP1 LCG)", "HDRChannel2": "ch2 (SP2)", "HDRChannel3": "ch3 (SP2)"}
for k, n in enumerate(names):
    r = load(LUC / f"{n}.json")
    a = np.mean([r["cfa"][c]["alpha_dn_per_e"] for c in ("G1", "G2")])
    S, vb = g_avg(r, "S"), g_avg(r, "vb")
    ok = S > 0
    re = np.mean([r["cfa"][c]["read_noise_e"] for c in ("G1", "G2")])
    ax.plot(S[ok] / a, np.sqrt(vb[ok]) / a, "-o", color=C[k], lw=2, ms=4, label=f"{names[n]}: α {a:.3g} DN/e-, read {re:.3g} e-")
xs = np.geomspace(0.5, 3e5, 60)
ax.plot(xs, np.sqrt(xs), "--", color=MUTED, lw=1, label="빛 노이즈만 있을 때 (√N)")
ax.set(xscale="log", yscale="log", xlabel="신호 (전자 수)", ylabel="노이즈 (전자 수)", title="LUCID (IMX490) 채널별 PTC — 전자 단위, 0 dB")
ax.legend(fontsize=9, loc="upper left")
save(fig, "02_lucid_channels")

# 3. LUCID HDR by gain
fig, ax = plt.subplots(figsize=(8, 5))
for k, g in enumerate(GAINS):
    r = load(LUC / f"{'HDR' if g == 0 else f'HDR_g{g:02d}'}.json")
    S, vb = g_avg(r, "S"), g_avg(r, "vb")
    ok = S > 0
    ax.plot(S[ok], np.sqrt(vb[ok]), "-o", color=C[k], lw=2, ms=4, label=f"{g} dB")
for lo, hi in ((4e4, 7.8e4), (2.8e5, 5.5e5)):
    ax.axvspan(lo, hi, color=GRID, alpha=0.8)
ax.text(5.6e4, 30, "채널 전환\n(0 dB)", ha="center", fontsize=9)
ax.text(3.9e5, 30, "채널 전환\n(0 dB)", ha="center", fontsize=9)
ax.set(xscale="log", yscale="log", xlabel="신호 S (DN, 24-bit)", ylabel="시간 노이즈 σ (DN)",
       title="LUCID HDR 합성 출력 PTC — gain별 (계단 = 큰 픽셀→작은 픽셀 전환)")
ax.legend(title="gain", ncol=2, loc="lower right")
save(fig, "03_lucid_hdr")

# 4/5. alpha and read noise vs gain
bfs = [load(BFS / f"{'BFS' if g == 0 else f'BFS_g{g:02d}'}.json") for g in GAINS]
luc = [load(LUC / f"{'HDR' if g == 0 else f'HDR_g{g:02d}'}.json") for g in GAINS]
a_b = np.array([np.mean([r["cfa"][c]["alpha_dn_per_e"] for c in ("G1", "G2")]) for r in bfs])
re_b = np.array([np.mean([r["cfa"][c]["read_noise_e"] for c in ("G1", "G2")]) for r in bfs])
def lucid_alpha(r, g):
    lv = r["cfa"]["G1"]["levels"]
    S = np.array([x["S"] for x in lv]); al = np.array([x["alpha_local"] or np.nan for x in lv])
    vd0 = r["cfa"]["G1"]["dark_var_dn2_tmin"]
    m = (S > 20 * np.sqrt(vd0)) & (S < 2e4 * 10 ** (g / 20))
    return float(np.nanmedian(al[m])), float(np.sqrt(vd0))
lu = np.array([lucid_alpha(r, g) for r, g in zip(luc, GAINS)])
a_l, rd_l = lu[:, 0], lu[:, 1]
theory = 10 ** (np.array(GAINS) / 20)
fig, ax = plt.subplots(figsize=(7, 4.6))
ax.plot(GAINS, 100 * (a_b / a_b[0] / theory - 1), "-o", color=C[0], lw=2, ms=6, label="BFS: 실측 α 배율 / 공칭 gain")
ax.plot(GAINS, 100 * (a_l / a_l[0] / theory - 1), "-s", color=C[1], lw=2, ms=6, label="LUCID: 실측 α 배율 / 공칭 gain")
ax.axhline(0, color=MUTED, lw=1)
ax.set(xlabel="gain (dB)", ylabel="공칭 대비 차이 (%)", ylim=(-2, 1),
       title="gain 대비 변환 이득 α — 조명과 무관한 측정 (0 = 공칭과 같음)")
ax.legend()
save(fig, "04_alpha_vs_gain")
fig, ax = plt.subplots(figsize=(7, 4.6))
ax.plot(GAINS, re_b, "-o", color=C[0], lw=2, ms=6, label="BFS (아날로그 gain 성분 → 줄어듦)")
ax.plot(GAINS, rd_l / a_l, "-s", color=C[1], lw=2, ms=6, label="LUCID HDR (디지털 gain → 그대로)")
ax.plot([0], [2.376], "D", color=INK, ms=7, label="BFS 기준 캘리브레이션 (최현석, 0 dB)")
ax.set(xlabel="gain (dB)", ylabel="read noise (전자)", ylim=(0, 2.7), title="gain별 read noise")
ax.legend()
save(fig, "05_read_noise_vs_gain")

# 6. local alpha
fig, ax = plt.subplots(figsize=(8, 4.8))
items = [(BFS / "BFS.json", "BFS 0 dB"), (BFS / "BFS_g24.json", "BFS 24 dB"), (LUC / "HDRChannel0.json", "LUCID ch0"),
         (LUC / "HDRChannel1.json", "LUCID ch1"), (LUC / "HDRChannel3.json", "LUCID ch3")]
for k, (p, lab) in enumerate(items):
    r = load(p)
    a = np.mean([r["cfa"][c]["alpha_dn_per_e"] for c in ("G1", "G2")])
    S, al = g_avg(r, "S"), g_avg(r, "alpha_local")
    sd0 = np.sqrt(np.mean([r["cfa"][c]["dark_var_dn2_tmin"] for c in ("G1", "G2")]))
    ok = (S >= 20 * sd0) & np.isfinite(al)
    ax.plot(S[ok] / a, 100 * (al[ok] / a - 1), "-o", color=C[k], lw=2, ms=4, label=lab)
ax.axhline(0, color=MUTED, lw=1)
ax.set(xscale="log", xlabel="신호 (전자 수)", ylabel="국소 α와 평균 α의 차이 (%)", title="신호별 변환 이득 — 전자가 많이 쌓이면 α가 변함 (α(S) 표로 반영)")
ax.legend(ncol=2, fontsize=9)
save(fig, "06_local_alpha")

# 7. dark offset / noise vs exposure (0 dB, G1)
fig, axs = plt.subplots(1, 2, figsize=(11, 4.4))
for k, (cam, lab) in enumerate((("bfs", "BFS"), ("lucid", "LUCID HDR"))):
    s = load(D / "fullframe_dark_20261020" / cam / "analysis" / "summary.json")
    cs = sorted([c for c in s["conditions"] if c["gain_db"] == 0], key=lambda c: c["exposure_us"])
    T = np.array([c["exposure_us"] for c in cs]) / 1000
    off = np.array([c["cfa"]["G1"]["offset_dn"] for c in cs]); sig = np.array([c["cfa"]["G1"]["temporal_sigma_dn"] for c in cs])
    dc = s["dark_vs_exposure"]["0"]["cfa"]["G1"]
    axs[0].plot(T, off - off[0], "-o", color=C[k], lw=2, ms=5, label=f"{lab}: {dc['dark_e_per_s_from_offset']:.1f} e-/s")
    axs[1].plot(T, sig, "-o", color=C[k], lw=2, ms=5, label=f"{lab}: {dc['dark_e_per_s_from_var']:.1f} e-/s (분산으로)")
axs[0].set(xscale="log", xlabel="노출 (ms)", ylabel="검은 레벨 상승 (DN)", title="dark current — 검은 레벨 (0 dB)")
axs[1].set(xscale="log", xlabel="노출 (ms)", ylabel="dark 노이즈 σ (DN)", title="dark current — 노이즈 (두 방법이 일치)")
axs[0].legend(); axs[1].legend()
save(fig, "07_dark_current")

# 8. validation ratio
fig, ax = plt.subplots(figsize=(8.5, 4.8))
vals = [(ROOT / "calibration/TRI054S-C_254600231/validation_independent_all_HDR.json", "LUCID HDR 0 dB — 독립 세션", C[1]),
        (ROOT / "calibration/BFS-PGE-23S3C_26315393/validation_roundtrip_BFS.json", "BFS 0 dB", C[0]),
        (ROOT / "calibration/BFS-PGE-23S3C_26315393/validation_roundtrip_BFS_g24.json", "BFS 24 dB", C[2])]
for p, lab, col in vals:
    lv = load(p)["levels"]
    S = np.array([x["S"] for x in lv]); r = np.array([x["ratio"] for x in lv])
    ax.plot(S, r, "-o", color=col, lw=2, ms=5, label=lab)
ax.axhspan(0.95, 1.05, color=GRID, alpha=0.7, label="±5 %")
ax.axhline(1, color=MUTED, lw=1)
for lo, hi in ((4e4, 7.8e4), (2.8e5, 5.5e5)):
    ax.axvspan(lo, hi, color="#eb6834", alpha=0.08)
ax.text(1.1e5, 1.38, "LUCID HDR 채널 전환 구간\n(모델 미완성)", fontsize=9, color=INK)
ax.set(xscale="log", xlabel="신호 S (DN)", ylabel="실측 분산 / 모델 분산", title="모델 검증 — 1이면 완벽")
ax.legend(loc="upper left", fontsize=9)
save(fig, "08_validation")

# 9. noise model comparison
fig, ax = plt.subplots(figsize=(8, 4.6))
for k, (p, lab) in enumerate(((LUC / "noise_model_test_HDR_G1.json", "LUCID HDR"), (BFS / "noise_model_test_BFS_G1.json", "BFS"))):
    lv = [x for x in load(p)["levels"] if x["exposure_us"] <= 1_000_000]
    T = np.array([x["exposure_us"] for x in lv]) / 1000
    d = np.array([x["d_empirical"] for x in lv]); se = np.array([x["d_empirical_se"] for x in lv])
    ax.errorbar(T, d, yerr=2 * se, fmt="-o", color=C[k], lw=2, ms=4, capsize=2, label=f"{lab}: 측정 분포 − 정규분포")
ax.axhline(0, color=MUTED, lw=1)
ax.set(xscale="log", xlabel="노출 (ms)", ylabel="ΔNLL (nats/픽셀, 낮을수록 좋음)",
       title="dark 노이즈 모델 비교 (홀수 프레임 채점, 오차막대 ±2SE)")
ax.legend()
save(fig, "09_noise_model_test")

# 10. trigger / period tests (LUCID var ratio vs exposure)
fig, ax = plt.subplots(figsize=(8, 4.6))
trig = [(60, 1.027), (100, 1.036), (150, 1.046), (300, 1.140), (1000, 1.141)]
trig1 = [(1, 1.013), (30, 1.045)]
free_drv = [(300, 1.072), (1000, 1.086)]
mine = [(300, 1.070), (300, 1.058), (800, 1.077), (1000, 1.084), (1000, 1.074)]
ax.plot(*zip(*sorted(trig1 + trig)), "-o", color=C[1], lw=2, ms=5, label="트리거 모드 (jsh-3e 드라이버)")
ax.plot(*zip(*free_drv), "s", color=C[0], ms=7, label="free-run (드라이버)")
ax.plot(*zip(*mine), "D", color=C[2], ms=6, label="free-run (내 스크립트, 주기 500~1050 ms)")
ax.axhspan(0.95, 1.05, color=GRID, alpha=0.7)
ax.axvline(150, color=MUTED, lw=1, ls=":"); ax.text(160, 1.12, "트리거 상한 150 ms", fontsize=9)
ax.axvline(800, color=MUTED, lw=1, ls=":"); ax.text(560, 1.0, "노출 상한\n800 ms", fontsize=9)
ax.set(xscale="log", xlabel="노출 (ms)", ylabel="dark 분산 / 모델",
       title="LUCID 트리거·free-run dark (재부팅 뒤 dark current +13 % → free-run도 ~1.07)")
ax.legend(fontsize=9, loc="upper left")
save(fig, "10_trigger_tests")
print("written", sorted(p.name for p in OUT.glob("*.png")))
