"""Camera noise model for training / evaluating RGB->RAW reconstruction (BFS IMX392, LUCID IMX490).

    from noise_model import NoiseModel
    nm = NoiseModel("calibration/TRI054S-C_254600231")
    raw = nm.sample(S, gain_db=0, exposure_us=30000, rng=rng)          # synthesize a noisy RAW frame
    z   = nm.zscore(raw_measured, S_pred, gain_db=0, exposure_us=30000) # evaluation: ~N(0,1) if S_pred is right
    nll = nm.nll(raw_measured, S_pred, gain_db=0, exposure_us=30000)    # loss / metric (nats per pixel)

S is the noise-free signal above the dark offset, in the camera's stored DN, full-frame Bayer (RGGB),
same settings as the calibration (see noise_model.json "settings"). Raw values include the per-pixel
dark offset, are quantized to the camera's code lattice and clipped to [0, white].

Noise pieces (all measured, see README):
  shot      : alpha(S) * Poisson(S / alpha(S)), alpha(S) per CFA from the PTC table (log-S interpolation)
  dark mean : offset_1ms map + dark_rate map * (T - 1 ms)
  dark noise: read_sigma map^2 + dark_var_rate map * (T - 1 ms), shape = empirical residual quantiles
              of the nearest measured exposure (LUCID dark noise is right-skewed), minus the row part
  row noise : one N(0, sigma_row^2) per row and frame (BFS ~3 % of the pixel variance; LUCID ~0.6 %)
Gains: measured 0/6/10/12/18/24 dB. LUCID gain is digital, so other gains are scaled from the nearest
measured one; BFS gains in between are not interpolated (raise).
"""
import json
from pathlib import Path

import numpy as np

CFA = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}


class NoiseModel:
    def __init__(self, calib_dir, roi=None):
        """roi=(y0, x0, h, w): use the maps cropped to a sensor ROI (offsets even -> RGGB kept)."""
        self.dir = Path(calib_dir)
        self.roi = roi
        self.doc = json.loads((self.dir / "noise_model.json").read_text())
        q = np.load(self.dir / "residual_quantiles.npz")
        self.probs = q["probs"]
        self.quant = {k: q[k] for k in q.files if k != "probs"}
        self.white = self.doc["white_level_dn"]
        self.step = self.doc["code_step_dn"]
        self.phase = self.doc["code_phase_dn"]
        self.digital_gain = "digital" in self.doc["gain_note"]
        self._maps = {}

    # ---------- parameters ----------
    def _gain(self, gain_db):
        measured = sorted(int(g) for g in self.doc["gains"])
        g0 = min(measured, key=lambda g: abs(g - gain_db))
        if abs(g0 - gain_db) > 1e-6 and not self.digital_gain:
            raise ValueError(f"gain {gain_db} dB not calibrated; measured {measured}")
        return g0, 10 ** ((gain_db - g0) / 20)          # digital: scale DN by k

    def maps(self, g0):
        if g0 not in self._maps:
            m = dict(np.load(self.dir / self.doc["gains"][str(g0)]["maps"]))
            if self.roi:
                y0, x0, h, w = self.roi
                m = {k: v[y0:y0 + h, x0:x0 + w] for k, v in m.items()}
            self._maps[g0] = m
        return self._maps[g0]

    def alpha(self, S, gain_db):
        """alpha(S) [DN/e-] per pixel for a full-frame Bayer signal array S (DN)."""
        g0, k = self._gain(gain_db)
        tab = self.doc["gains"][str(g0)]["alpha_table"]
        out = np.empty_like(S, dtype=np.float64)
        for c, (i, j) in CFA.items():
            s_tab = np.asarray(tab["S_by_cfa"][c]) * k
            a_tab = np.asarray(tab["alpha"][c]) * k
            s = np.maximum(S[i::2, j::2], 1e-9)
            out[i::2, j::2] = np.interp(np.log(s), np.log(s_tab), a_tab)
        return out

    def dark(self, gain_db, exposure_us):
        """Per-pixel dark offset (DN) and dark variance (DN^2) at this gain / exposure."""
        g0, k = self._gain(gain_db)
        m = self.maps(g0)
        dt = (exposure_us - 1000.0) * 1e-6
        ped = self.doc["settings"].get("black_level_raw", 0)
        offset = ped + (m["offset_1ms_dn"] - ped + m["dark_rate_dn_per_s"] * dt) * k
        var = (m["read_sigma_dn"].astype(np.float64) ** 2 + np.maximum(m["dark_var_rate_dn2_per_s"], 0) * dt) * k * k
        return offset, np.maximum(var, (self.step ** 2) / 12)

    def row_sigma(self, gain_db, exposure_us):
        g0, k = self._gain(gain_db)
        d = self.doc["gains"][str(g0)]["dark_fullframe"]
        return k * float(np.interp(np.log(exposure_us), np.log(d["exposure_us"]), d["row_temporal_sigma_dn"]))

    def variance(self, S, gain_db, exposure_us):
        """Total temporal variance (DN^2) of a raw pixel with noise-free signal S."""
        _, dvar = self.dark(gain_db, exposure_us)
        return self.alpha(S, gain_db) * np.maximum(S, 0) + dvar

    def _shape(self, gain_db, exposure_us):
        g0, _ = self._gain(gain_db)
        keys = [k for k in self.quant if k.startswith(f"g{g0:02d}_")]
        texp = {k: float(k.split("_t")[1][:-2]) for k in keys}
        best = min(keys, key=lambda k: abs(np.log(texp[k]) - np.log(exposure_us)))
        return self.quant[best]

    # ---------- use ----------
    def sample(self, S, gain_db, exposure_us, rng, dark_shape="empirical", n_frames=None):
        """Noisy raw frame(s) for noise-free signal S (DN above dark offset)."""
        S = np.asarray(S, dtype=np.float64)
        shape = S.shape if n_frames is None else (n_frames,) + S.shape
        a = self.alpha(S, gain_db)
        n_eff = np.maximum(S, 0) / a
        shot = a * rng.poisson(np.broadcast_to(n_eff, shape))
        offset, dvar = self.dark(gain_db, exposure_us)
        srow = self.row_sigma(gain_db, exposure_us)
        # the measured dark variance already contains the quantization (step^2/12) that the rounding
        # below adds again, so draw the pre-quantization part only
        indep = np.sqrt(np.maximum(dvar - srow ** 2 - (self.step ** 2) / 12, 0))
        if dark_shape == "empirical":
            u = rng.random(shape)
            z = np.interp(u, self.probs, self._shape(gain_db, exposure_us))
            z = (z - z.mean()) / z.std()          # quantile table is ~unit-variance; renormalize the draw
        else:
            z = rng.standard_normal(shape)
        rows = rng.standard_normal(shape[:-1] + (1,)) * srow
        x = offset + shot + indep * z + rows
        x = np.round((x - self.phase) / self.step) * self.step + self.phase
        return np.clip(x, 0, self.white)

    def signal(self, raw, gain_db, exposure_us):
        """Measured raw -> signal above the per-pixel dark offset (DN)."""
        offset, _ = self.dark(gain_db, exposure_us)
        return np.asarray(raw, dtype=np.float64) - offset

    def zscore(self, raw, S_pred, gain_db, exposure_us):
        """(raw - expected raw) / predicted sigma. ~N(0,1) per pixel if S_pred is the true signal."""
        offset, _ = self.dark(gain_db, exposure_us)
        var = self.variance(S_pred, gain_db, exposure_us)   # measured maps already include quantization
        return (np.asarray(raw, dtype=np.float64) - offset - S_pred) / np.sqrt(var)

    def nll(self, raw, S_pred, gain_db, exposure_us):
        """Gaussian NLL (nats/pixel) with the calibrated, signal-dependent variance; usable as a loss.
        Saturated pixels (raw >= white) contribute -log P(x >= white)."""
        from scipy.special import log_ndtr
        offset, _ = self.dark(gain_db, exposure_us)
        var = self.variance(S_pred, gain_db, exposure_us)   # measured maps already include quantization
        mu = offset + S_pred
        x = np.asarray(raw, dtype=np.float64)
        out = 0.5 * np.log(2 * np.pi * var) + 0.5 * (x - mu) ** 2 / var - np.log(self.step)
        sat = x >= self.white - self.step / 2
        if np.any(sat):
            out[sat] = -log_ndtr((mu[sat] - self.white) / np.sqrt(var[sat]))
        return out
