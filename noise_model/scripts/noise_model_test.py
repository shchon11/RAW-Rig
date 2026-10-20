"""Which dark-noise model fits held-out data? Statistical comparison on PTC dark levels.

  python noise_model_test.py ../data/ptc_<session> --output HDR [--alpha 9.83]

Each dark level has 100 frames (ROI). Even frames fit, odd frames test (interleaved: slow drift
falls on both halves). Per pixel i, models for the test values x:
  gauss   : N(mu_i, sigma_i^2) from the fit frames
  poisson : o_i + alpha*K + N(0, s_i^2), K ~ Poisson(lambda_i), lambda_i = max(mu_i - o_i, 0)/alpha,
            o_i and s_i from the shortest-exposure dark (read noise only)
  empirical: pooled standardized residual distribution of the fit frames, scaled by sigma_i
Data are integers with code step q (1 DN LUCID, 16 DN BFS), so each model gives the probability
mass F(x+q/2) - F(x-q/2). Scores on the test frames: mean NLL (nats/sample), paired difference
to gauss with its standard error, and the KS distance of randomized PIT values from uniform.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy import special, stats

EPS = 1e-300


def code_step(x):
    u = np.unique(x[: min(len(x), 2_000_000)])   # all distinct codes in a large sample
    return float(np.gcd.reduce(np.diff(u).astype(np.int64))) if u.size > 1 else 1.0


def cdf_gauss(x, mu, sd):
    return special.ndtr((x - mu) / sd)


def cdf_poisson_gauss(x, o, lam, s, alpha, kmax):
    k = np.arange(kmax + 1)
    pk = stats.poisson.pmf(k[None, :], lam[:, None])                       # [pixels, k]
    out = np.zeros_like(x)
    for j in range(kmax + 1):
        out += pk[:, j][None, :] * special.ndtr((x - o[None, :] - alpha * j) / s[None, :])
    return out


def score(prob_mass, cdf_low, rng):
    p = np.clip(prob_mass, EPS, None)
    nll = -np.log(p)
    u = np.clip(cdf_low + rng.random(p.shape) * prob_mass, 0, 1)   # randomized PIT
    ks = stats.kstest(u.ravel()[:1_000_000], "uniform").statistic
    return nll, ks


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--output", default="HDR")
    ap.add_argument("--alpha", type=float, required=True, help="DN per e- in the dark range")
    ap.add_argument("--cfa", default="G1", choices=["R", "G1", "G2", "B", "all"])
    a = ap.parse_args()
    rec = json.loads((a.session / "manifest.json").read_text())["outputs"][a.output]
    folder = a.session / a.output
    sl = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}

    def load(f):
        s = np.load(folder / f)["stack"].astype(np.float64)
        if a.cfa != "all":
            i, j = sl[a.cfa]
            s = s[:, i::2, j::2]
        return s.reshape(s.shape[0], -1)

    base = load(rec["dark"][0]["file"])
    o, s_read = base[0::2].mean(0), base[0::2].std(0, ddof=1)
    rng = np.random.default_rng(0)
    rows = []
    print(f"{'T ms':>9s} {'λ mean':>7s} {'q':>3s} | {'NLL gauss':>9s} | {'Δ poisson':>16s} | {'Δ empirical':>16s} | "
          f"{'KS gauss':>8s} {'KS pois':>8s} {'KS emp':>8s}")
    for lv in rec["dark"]:
        x = load(lv["file"])
        fit, test = x[0::2], x[1::2]
        q = code_step(x.ravel())
        mu, sd = fit.mean(0), np.maximum(fit.std(0, ddof=1), q / np.sqrt(12))
        lo, hi = test - q / 2, test + q / 2
        # gauss
        cl, ch = cdf_gauss(lo, mu, sd), cdf_gauss(hi, mu, sd)
        nll_g, ks_g = score(ch - cl, cl, rng)
        # poisson + read noise
        lam = np.maximum(mu - o, 0) / a.alpha
        sr = np.maximum(s_read, q / np.sqrt(12))
        kmax = int(np.ceil(lam.max() + 8 * np.sqrt(lam.max() + 1) + 4))
        kmax = min(kmax, 60)
        cl_p = cdf_poisson_gauss(lo, o, lam, sr, a.alpha, kmax)
        ch_p = cdf_poisson_gauss(hi, o, lam, sr, a.alpha, kmax)
        nll_p, ks_p = score(ch_p - cl_p, cl_p, rng)
        # empirical shape
        z_fit = np.sort(((fit - mu) / sd).ravel())
        ecdf = lambda z: np.searchsorted(z_fit, z, side="right") / z_fit.size
        # light smoothing: mix 1% gaussian so unseen values keep non-zero mass
        cdf_e = lambda xx: 0.99 * ecdf((xx - mu) / sd) + 0.01 * cdf_gauss(xx, mu, sd)
        cl_e, ch_e = cdf_e(lo), cdf_e(hi)
        nll_e, ks_e = score(ch_e - cl_e, cl_e, rng)
        d_p, d_e = (nll_p - nll_g).ravel(), (nll_e - nll_g).ravel()
        row = {"exposure_us": lv["exposure_us"], "lambda_mean": float(lam.mean()), "q": q,
               "nll_gauss": float(nll_g.mean()), "nll_poisson": float(nll_p.mean()), "nll_empirical": float(nll_e.mean()),
               "d_poisson": float(d_p.mean()), "d_poisson_se": float(d_p.std() / np.sqrt(d_p.size)),
               "d_empirical": float(d_e.mean()), "d_empirical_se": float(d_e.std() / np.sqrt(d_e.size)),
               "ks_gauss": float(ks_g), "ks_poisson": float(ks_p), "ks_empirical": float(ks_e), "n": int(test.size)}
        rows.append(row)
        print(f"{lv['exposure_us']/1000:9.3f} {row['lambda_mean']:7.3f} {q:3.0f} | {row['nll_gauss']:9.4f} | "
              f"{row['d_poisson']:+.4f}±{row['d_poisson_se']:.4f} | {row['d_empirical']:+.4f}±{row['d_empirical_se']:.4f} | "
              f"{ks_g:8.4f} {ks_p:8.4f} {ks_e:8.4f}", flush=True)
    out = a.session / "analysis" / f"noise_model_test_{a.output}_{a.cfa}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"output": a.output, "alpha": a.alpha, "cfa": a.cfa, "levels": rows}, indent=1))
    print("negative Δ = better than gauss (nats/sample); KS: 0 = perfectly calibrated")
    print("written:", out)


if __name__ == "__main__":
    main()
