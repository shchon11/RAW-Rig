"""Build the calibration results page (single HTML, figures embedded) for the Artifact viewer."""
import base64
import sys
from pathlib import Path

FIG = Path(__file__).resolve().parent.parent / "data" / "plots" / "report"
OUT = Path(sys.argv[1])


def img(name, alt):
    b = base64.b64encode((FIG / name).read_bytes()).decode()
    return f'<img src="data:image/png;base64,{b}" alt="{alt}" loading="lazy">'


def fig(name, alt, title, body):
    return f"""<figure class="fig"><div class="plate">{img(name, alt)}</div>
<figcaption><h3>{title}</h3>{body}</figcaption></figure>"""


HTML = f"""<title>IMX392·IMX490 노이즈 캘리브레이션</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans+KR:wght@400;600&family=IBM+Plex+Mono:wght@400;500&family=Noto+Sans+KR:wght@400;500&display=swap">
<style>
/* layout: one reading column; verdict strip first, then figure plates with a short reading each */
:root {{
  --bg: #f6f7f9; --fg: #16191f; --muted: #5b6270; --line: #dfe3ea; --accent: #1f5fbf;
  --ok: #1d7a4a; --warn: #a3620a; --plate: #fcfcfb;
  --display: "IBM Plex Sans KR", "Noto Sans KR", system-ui, sans-serif;
  --body: "Noto Sans KR", "IBM Plex Sans KR", system-ui, sans-serif;
  --mono: "IBM Plex Mono", ui-monospace, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg: #12151b; --fg: #e7eaf0; --muted: #a2a9b7; --line: #2a303b; --accent: #7fb0ff;
  --ok: #5fcf93; --warn: #f0b04a; --plate: #fcfcfb; color-scheme: dark }} }}
:root[data-theme="dark"] {{
  --bg: #12151b; --fg: #e7eaf0; --muted: #a2a9b7; --line: #2a303b; --accent: #7fb0ff;
  --ok: #5fcf93; --warn: #f0b04a; --plate: #fcfcfb; color-scheme: dark }}
body {{ background: var(--bg); color: var(--fg); font-family: var(--body); font-size: 15px; line-height: 1.65 }}
.wrap {{ max-width: 980px; margin: 0 auto; padding-block: 32px 64px;
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px));
  display: grid; gap: 40px }}
header {{ display: grid; gap: 8px }}
.eyebrow {{ font-family: var(--mono); font-size: 12px; letter-spacing: .06em; color: var(--muted); text-transform: uppercase }}
h1 {{ font-family: var(--display); font-weight: 600; font-size: clamp(26px, 4vw, 36px); line-height: 1.25; margin: 0; text-wrap: balance }}
h2 {{ font-family: var(--display); font-weight: 600; font-size: 21px; margin: 0; text-wrap: balance }}
h3 {{ font-family: var(--display); font-weight: 600; font-size: 16px; margin: 0 }}
p {{ margin: 0; max-width: 68ch }}
.lede {{ color: var(--muted) }}
.verdict {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 12px }}
.card {{ border: 1px solid var(--line); border-radius: 8px; padding: 14px 16px; display: grid; gap: 6px; min-width: 0 }}
.chip {{ font-family: var(--mono); font-size: 12px; font-weight: 500; justify-self: start; padding: 2px 8px; border-radius: 4px; border: 1px solid currentColor }}
.chip.ok {{ color: var(--ok) }} .chip.warn {{ color: var(--warn) }}
.card p {{ font-size: 14px }}
section {{ display: grid; gap: 18px }}
.fig {{ margin: 0; display: grid; gap: 10px }}
.plate {{ background: var(--plate); border: 1px solid var(--line); border-radius: 6px; padding: 6px; overflow-x: auto }}
.plate img {{ display: block; width: 100%; height: auto; max-width: 100% }}
figcaption {{ display: grid; gap: 4px }}
figcaption p {{ font-size: 14.5px }}
.num {{ font-family: var(--mono); font-variant-numeric: tabular-nums }}
table {{ border-collapse: collapse; font-size: 14px; min-width: 520px }}
.tbl {{ overflow-x: auto }}
th, td {{ text-align: left; padding: 6px 10px; border-bottom: 1px solid var(--line) }}
th {{ font-family: var(--display); font-weight: 600 }}
td.n {{ font-family: var(--mono); font-variant-numeric: tabular-nums }}
ul {{ margin: 0; padding-left: 20px; display: grid; gap: 4px; max-width: 72ch }}
code {{ font-family: var(--mono); font-size: 13px }}
</style>
<div class="wrap">
<header>
  <div class="eyebrow">ircv lab · 2026-10-19 ~ 20 · PTC + full-frame dark</div>
  <h1>IMX392·IMX490 노이즈 캘리브레이션</h1>
  <p class="lede">RGB → 카메라 RAW 복원 모델을 학습·평가할 때 쓰는 카메라별 노이즈 모델. BFS-PGE-23S3C (IMX392, SN 26315393)와 TRI054S-C (IMX490, SN 254600231).</p>
</header>

<section>
  <h2>판정: 캘리브레이션은 잘 됐다</h2>
  <div class="verdict">
    <div class="card"><span class="chip ok">BFS 통과</span>
      <p>기준 캘리브레이션(최현석)과 α·read noise가 <span class="num">0.6 %</span> 안에서 일치. 모델 분산이 실측과 모든 단계에서 <span class="num">±0.7 %</span>.</p></div>
    <div class="card"><span class="chip ok">LUCID 통과</span>
      <p>채널별 α가 세 세션에서 <span class="num">0.1~1 %</span> 안에서 재현. 캘리브레이션에 안 쓴 세션에서 15단계 중 13단계가 <span class="num">0.95~1.00</span>.</p></div>
    <div class="card"><span class="chip warn">LUCID 한 곳 미완성</span>
      <p>HDR이 채널을 바꾸는 두 밝기 구간에서 실측 노이즈가 모델보다 <span class="num">17~45 %</span> 크다. 이 구간만 다시 정밀 측정한다.</p></div>
    <div class="card"><span class="chip warn">사용 조건</span>
      <p>LUCID 노출 ≤ 800 ms (트리거 ≤ 150 ms), BFS ≤ 1 s, 녹화마다 장면 dark로 검은 레벨·분산 보정.</p></div>
  </div>
</section>

<section>
  <h2>노이즈 곡선 (PTC)</h2>
  {fig("01_bfs_ptc.png", "BFS gain별 PTC", "BFS — gain 6개",
   "<p>점은 실측, 점선은 모델이다. 점선이 실측에 덮여 거의 안 보일 만큼 맞는다. gain을 올리면 같은 신호에서 노이즈(DN)가 커진다.</p>")}
  {fig("02_lucid_channels.png", "LUCID 채널별 PTC", "LUCID — 채널 4개, 전자 단위",
   "<p>ch0과 ch1은 같은 포토다이오드라 빛 노이즈 선(점선) 위에서 겹친다. ch0은 read noise가 <span class=num>0.53</span> 전자로 전자 하나보다 작다. ch2·ch3는 작은 픽셀이라 바닥 노이즈가 크지만, 아주 밝은 곳에서만 쓰인다.</p>")}
  {fig("03_lucid_hdr.png", "LUCID HDR PTC", "LUCID HDR 합성 출력 — gain 6개",
   "<p>녹화하는 출력이다. 큰 픽셀 → 작은 픽셀로 넘어가는 지점에서 노이즈가 한 번에 뛴다(회색 띠, 0 dB 기준). gain이 디지털이라 곡선이 그대로 오른쪽 위로 밀린다.</p>")}
</section>

<section>
  <h2>gain 특성</h2>
  {fig("04_alpha_vs_gain.png", "gain 대비 α", "gain과 변환 이득",
   "<p>전자 하나가 몇 DN인지(α)를 gain별로 공칭값과 비교했다. 조명 밝기와 무관한 측정이다. LUCID는 공칭과 <span class=num>0.5 %</span> 안, BFS는 약 <span class=num>1 %</span> 작다. 노이즈 모델에는 실측값을 쓴다.</p>")}
  {fig("05_read_noise_vs_gain.png", "gain별 read noise", "gain과 read noise",
   "<p>BFS는 gain을 올리면 read noise가 <span class=num>2.36 → 1.24</span> 전자로 줄어 저조도에서 이득이다. LUCID는 gain이 디지털이라 <span class=num>0.52</span> 전자로 그대로다. 그래서 LUCID는 0 dB 고정을 권한다.</p>")}
</section>

<section>
  <h2>신호 크기별 변환 이득</h2>
  {fig("06_local_alpha.png", "신호별 국소 α", "전자가 많이 쌓이면 α가 조금 변한다",
   "<p>전자가 적은 출력(LUCID ch0, BFS 24 dB)은 평평하다. 많이 쌓이는 출력(BFS 0 dB, LUCID ch1·ch3)은 ±1.5~4 % 움직인다. 센서 자체의 비선형으로 보고, 모델은 상수 α 대신 이 곡선을 표로 쓴다.</p>")}
</section>

<section>
  <h2>빛이 없을 때 (dark)</h2>
  {fig("07_dark_current.png", "dark current", "dark current",
   "<p>노출이 길어지면 검은 레벨과 노이즈가 같이 오른다. 두 방법으로 구한 dark current가 BFS <span class=num>18</span>, LUCID <span class=num>1.4</span> 전자/초로 서로 일치해서 α까지 교차 검증된다.</p>")}
  {fig("dark_noise_tails.png", "dark 노이즈 분포", "LUCID dark 노이즈는 밝은 쪽으로 치우친다",
   "<p>같은 픽셀을 빛 없이 반복해서 찍은 값의 분포다. 종 모양(점선)보다 밝은 쪽으로 크게 튀는 값이 많다(320 ms에서 22배). read noise가 전자 하나보다 작아서, dark 전자가 0개·1개·2개로 딱 떨어지는 게 그대로 드러난다. 그래서 LUCID는 정규분포 대신 실측 분포를 쓴다. BFS는 거의 종 모양이다.</p>")}
</section>

<section>
  <h2>검증</h2>
  {fig("08_validation.png", "실측 대 모델", "실측 노이즈 / 모델 노이즈",
   "<p>1에 붙어 있으면 모델이 맞다. BFS는 모든 밝기에서 회색 띠(±5 %) 안이다. LUCID는 캘리브레이션에 안 쓴 세션인데도 맞고, 두 채널 전환 구간에서만 튄다. 이 두 구간이 지금 다시 재려는 곳이다.</p>")}
  {fig("09_noise_model_test.png", "분포 모델 비교", "dark 노이즈 모델: 실측 분포 vs 정규분포",
   "<p>짝수 프레임으로 모델을 만들고 홀수 프레임으로 채점했다(낮을수록 좋음). 모든 노출에서 실측 분포가 이기고, LUCID 긴 노출에서 차이가 가장 크다.</p>")}
</section>

<section>
  <h2>트리거 모드와 노출 상한</h2>
  {fig("10_trigger_tests.png", "트리거 시험", "LUCID 트리거·free-run dark",
   "<p>두 카메라 동시 촬영(트리거)은 150 ms까지 모델과 맞고, 300 ms부터 노이즈가 더 커진다. 1 s에서는 검은 레벨이 무작위로 두 상태를 오가는 현상이 있어서 LUCID 노출 상한을 800 ms로 정했다. free-run이 1.07 근처인 것은 재부팅 뒤 센서가 조금 따뜻해서 생긴 dark current 증가이고, 장면 dark로 보정된다.</p>")}
</section>

<section>
  <h2>핵심 수치</h2>
  <div class="tbl"><table>
    <tr><th>카메라 / 출력</th><th>α (DN/전자)</th><th>read noise (전자)</th><th>dark current (전자/초)</th><th>포화</th></tr>
    <tr><td>BFS 0 dB</td><td class="n">5.811</td><td class="n">2.36</td><td class="n">18</td><td class="n">65516</td></tr>
    <tr><td>BFS 24 dB</td><td class="n">91.21</td><td class="n">1.24</td><td class="n">15.5</td><td class="n">65516</td></tr>
    <tr><td>LUCID ch0 (SP1 HCG)</td><td class="n">9.627</td><td class="n">0.53</td><td class="n">—</td><td class="n">425 전자</td></tr>
    <tr><td>LUCID ch1 (SP1 LCG)</td><td class="n">0.4524</td><td class="n">2.46</td><td class="n">—</td><td class="n">9,050 전자</td></tr>
    <tr><td>LUCID ch2 (SP2)</td><td class="n">0.964</td><td class="n">13.5</td><td class="n">—</td><td class="n">4,250 전자</td></tr>
    <tr><td>LUCID ch3 (SP2)</td><td class="n">0.0261</td><td class="n">~100</td><td class="n">—</td><td class="n">157,000 전자</td></tr>
    <tr><td>LUCID HDR 0 dB</td><td class="n">9.83 → ~160</td><td class="n">0.52</td><td class="n">1.4</td><td class="n">2²⁴−1</td></tr>
  </table></div>
</section>

<section>
  <h2>자료 위치</h2>
  <ul>
    <li>코드·보고서: GitHub <code>shchon11/RAW-Rig</code> 브랜치 <code>imx490-noise-model</code>, 폴더 <code>noise_model/</code></li>
    <li>외장 SSD용 번들(22 GB): <code>/mnt/data/jsh_raw/noise_calibration_20261020/</code> — 패키지, 원본 측정, 검증 데이터, 체크섬</li>
    <li>방법 전체: 번들의 <code>CALIBRATION_REPORT.md</code></li>
  </ul>
</section>
</div>
"""
OUT.write_text(HTML)
print(OUT, f"{OUT.stat().st_size / 1e6:.2f} MB")
