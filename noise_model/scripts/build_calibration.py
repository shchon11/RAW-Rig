"""Collect the PTC + full-frame dark results into one noise-model package per camera.

  python build_calibration.py            # -> ../calibration/<model>_<serial>/

Package (consumed by noise_model.py, for training / evaluating RGB->RAW reconstruction):
  noise_model.json        settings, DN domain, per gain: alpha per CFA, alpha(S) table, read noise,
                          dark offset/variance vs exposure, dark current, row/col noise, white
                          level, code step/phase, validity range; LUCID also per-channel results
  maps_gNN.npz            per pixel (full frame): offset at 1 ms, dark rate (DN/s), read sigma (DN),
                          dark variance rate (DN^2/s), hot / noisy masks
  residual_quantiles.npz  per (gain, exposure): quantiles of the standardized dark residual
                          (empirical shape; won the held-out NLL test)
"""
import json
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
OUT = ROOT / "calibration"
CFA = {"R": (0, 0), "G1": (0, 1), "G2": (1, 0), "B": (1, 1)}
GAINS = [0, 6, 10, 12, 18, 24]
PROBS = np.concatenate([[1e-6, 1e-5, 1e-4], np.linspace(1e-3, 1 - 1e-3, 999), [1 - 1e-4, 1 - 1e-5, 1 - 1e-6]])

CAMERAS = {
    "bfs": {"dir": "BFS-PGE-23S3C_26315393", "model": "BFS-PGE-23S3C", "sensor": "Sony IMX392", "serial": "26315393",
            "ptc": DATA / "ptc_20261019_2131_bfs", "ptc_name": lambda g: "BFS" if g == 0 else f"BFS_g{g:02d}",
            "settings": {"pixel_format": "BayerRG16", "adc_bit_depth": "Bit12", "black_level_percent": 1.0,
                         "black_level_clamping": False, "balance_ratio_rb": 1.0, "gamma_lut_defect": "off",
                         "cfa": "RGGB"},
            "dn_domain": "BayerRG16 stored DN = 12-bit ADC code << 4 (step 16, phase 12)",
            "white": 65516, "step": 16, "phase": 12,
            "gain_note": "analog (read noise in e- falls with gain); actual gain ~1% below nominal"},
    "lucid": {"dir": "TRI054S-C_254600231", "model": "TRI054S-C", "sensor": "Sony IMX490", "serial": "254600231",
              "ptc": DATA / "ptc_20261019_2131_lucid", "ptc_name": lambda g: "HDR" if g == 0 else f"HDR_g{g:02d}",
              "settings": {"pixel_format": "BayerRG24", "hdr_output": "HDR", "black_level_raw": 256,
                           "isp": "WB/CCM/LUT/HDR enhancement/digital clamping off", "cfa": "RGGB"},
              "dn_domain": "BayerRG24 DN of the combined HDR output, pedestal 256 (step 1)",
              "white": 2 ** 24 - 1, "step": 1, "phase": 0,
              "gain_note": "digital, applied after HDR combination: alpha and DN noise scale by 10^(g/20), "
                           "read noise in e- and dark current unchanged",
              "valid_exposure_us": [86, 800_000],
              "caveat": "exposure limit 800 ms: 1 s black level is bistable (-9.7 DN state, jsh-3e 2026-10-20); "
                        ">1 s is a different regime; trigger mode <= 150 ms"},
}


def alpha_table(res, white):
    tab = {"S_dn": None, "alpha": {}}
    for c in CFA:
        lv = res["cfa"][c]["levels"]
        sd0 = res["cfa"][c]["read_noise_dn"]
        keep = [x for x in lv if x["alpha_local"] and not x["saturated"] and x["S"] >= 20 * sd0
                and (not white or x["S"] < 0.9 * (white - x["offset"]))]
        tab["alpha"][c] = [float(x["alpha_local"]) for x in keep]
        tab.setdefault("S_by_cfa", {})[c] = [float(x["S"]) for x in keep]
    tab.pop("S_dn")
    return tab


def maps_for(ff, g):
    js = sorted(ff.glob(f"g{g:02d}_t*us.json"), key=lambda p: json.loads(p.read_text())["exposure_us"])
    infos = [json.loads(p.read_text()) for p in js]
    T = np.array([i["exposure_us"] for i in infos]) * 1e-6
    use = T <= 1.0 + 1e-6
    means, vds = [], []
    for p, u in zip(js, use):
        if u:
            z = np.load(p.with_suffix(".npz"))
            means.append(z["mean"].astype(np.float64))
            vds.append(z["var_diff"].astype(np.float64))
    t = T[use]
    tc = t - t.mean()
    den = np.sum(tc ** 2)
    M, V = np.stack(means), np.stack(vds)
    rate = np.tensordot(tc, M - M.mean(0), axes=1) / den
    vrate = np.tensordot(tc, V - V.mean(0), axes=1) / den
    off1 = M[np.argmin(t)]
    sig1 = np.sqrt(V[np.argmin(t)])
    long = M[np.argmax(t)]
    sig_long = np.sqrt(V[np.argmax(t)])
    hot = np.zeros(off1.shape, bool)
    noisy = np.zeros(off1.shape, bool)
    for c, (i, j) in CFA.items():
        sl = (slice(i, None, 2), slice(j, None, 2))
        hot[sl] = (long[sl] - np.median(long[sl])) > 5 * np.median(sig_long[sl])
        noisy[sl] = sig1[sl] > 2 * np.median(sig1[sl])
    return {"offset_1ms_dn": off1.astype(np.float32), "dark_rate_dn_per_s": rate.astype(np.float32),
            "read_sigma_dn": sig1.astype(np.float32), "dark_var_rate_dn2_per_s": vrate.astype(np.float32),
            "hot": hot, "noisy": noisy}, infos


def residual_quantiles(ff, g):
    out = {}
    for p in sorted(ff.glob(f"g{g:02d}_t*us.npz")):
        z = np.load(p)
        fr = z["frames"].astype(np.float64)
        sd = np.sqrt(np.maximum(z["var_diff"].astype(np.float64), 1e-6))
        r = ((fr - z["mean"][None]) / sd[None]).ravel()
        rng = np.random.default_rng(0)
        sub = rng.choice(r, size=min(r.size, 10_000_000), replace=False)
        out[p.stem] = np.quantile(sub, PROBS).astype(np.float32)
    return out


def build(key):
    cam = CAMERAS[key]
    dest = OUT / cam["dir"]
    dest.mkdir(parents=True, exist_ok=True)
    ff = DATA / "fullframe_dark_20261020" / key
    ffsum = json.loads((ff / "analysis" / "summary.json").read_text())
    doc = {"camera": cam["model"], "sensor": cam["sensor"], "serial": cam["serial"], "built": time.strftime("%Y-%m-%d %H:%M"),
           "purpose": "noise model for training / evaluating RGB->RAW reconstruction of this camera",
           "settings": cam["settings"], "dn_domain": cam["dn_domain"], "cfa_pattern": "RGGB",
           "white_level_dn": cam["white"], "code_step_dn": cam["step"], "code_phase_dn": cam["phase"],
           "gain_note": cam["gain_note"], "valid_exposure_us": cam.get("valid_exposure_us"), "caveat": cam.get("caveat"),
           "model": "Var[DN] = alpha(S)*S + v_dark(T);  S = signal above dark offset (DN);  "
                    "alpha(S) from the alpha table (constant alpha is rejected statistically at high charge); "
                    "v_dark(T) = read_sigma^2 + dark_var_rate*T;  dark shape: empirical residual quantiles",
           "sources": {"ptc": str(cam["ptc"].relative_to(ROOT)), "fullframe_dark": str(ff.relative_to(ROOT))},
           "gains": {}}
    q_all = {}
    for g in GAINS:
        res = json.loads((cam["ptc"] / "analysis" / f"{cam['ptc_name'](g)}.json").read_text())
        maps, infos = maps_for(ff, g)
        np.savez_compressed(dest / f"maps_g{g:02d}.npz", **maps)
        for k, v in residual_quantiles(ff, g).items():
            q_all[k] = v
        conds = sorted([c for c in ffsum["conditions"] if c["gain_db"] == g], key=lambda c: c["exposure_us"])
        dve = ffsum["dark_vs_exposure"][str(g)]
        entry = {
            "alpha_dn_per_e": {c: res["cfa"][c].get("alpha_dn_per_e") for c in CFA},
            "alpha_table": alpha_table(res, cam["white"] if key == "bfs" else None),
            "read_noise_dn": {c: res["cfa"][c]["read_noise_dn"] for c in CFA},
            "model_test": {c: {k: res["cfa"][c].get("model_test", {}).get(k) for k in ("chi2", "dof", "p_value", "max_abs_z")}
                           for c in CFA},
            "dark_fullframe": {
                "exposure_us": [c["exposure_us"] for c in conds],
                "offset_dn": {ch: [c["cfa"][ch]["offset_dn"] for c in conds] for ch in CFA},
                "temporal_sigma_dn": {ch: [c["cfa"][ch]["temporal_sigma_dn"] for c in conds] for ch in CFA},
                "row_temporal_sigma_dn": [c["row_temporal_sigma_dn"] for c in conds],
                "col_temporal_sigma_dn": [c["col_temporal_sigma_dn"] for c in conds],
                "hot_pixels": {ch: [c["cfa"][ch]["hot_pixels"] for c in conds] for ch in CFA},
                "noisy_pixels": {ch: [c["cfa"][ch]["noisy_pixels"] for c in conds] for ch in CFA},
                "tail_frac_4sigma": {ch: [c["cfa"][ch]["tail_frac"]["4"] for c in conds] for ch in CFA},
            },
            "dark_current_e_per_s": {ch: dve["cfa"][ch]["dark_e_per_s_from_offset"] for ch in CFA},
            "maps": f"maps_g{g:02d}.npz",
        }
        a = np.mean([entry["alpha_dn_per_e"][c] for c in ("G1", "G2") if entry["alpha_dn_per_e"][c]])
        entry["read_noise_e"] = {c: entry["read_noise_dn"][c] / a for c in CFA}
        if key == "lucid":   # HDR: dark-range alpha = SP1 local alpha (single-alpha fit is meaningless)
            entry["alpha_dark_range_dn_per_e"] = dve["alpha_dn_per_e"]
            entry["read_noise_e"] = {c: entry["read_noise_dn"][c] / dve["alpha_dn_per_e"] for c in CFA}
        doc["gains"][str(g)] = entry
        print(f"[{key}] gain {g:2d}: maps + {sum(1 for k in q_all if k.startswith(f'g{g:02d}'))} residual tables", flush=True)
    np.savez_compressed(dest / "residual_quantiles.npz", probs=PROBS.astype(np.float64), **q_all)
    if key == "lucid":
        doc["channels_0dB"] = {}
        for n in ("HDRChannel0", "HDRChannel1", "HDRChannel2", "HDRChannel3"):
            r = json.loads((cam["ptc"] / "analysis" / f"{n}.json").read_text())
            doc["channels_0dB"][n] = {k: {c: r["cfa"][c].get(k) for c in CFA}
                                      for k in ("alpha_dn_per_e", "read_noise_e", "read_noise_dn")}
            doc["channels_0dB"][n]["white_level_dn"] = 4351
        doc["channels_note"] = ("ch0=SP1 HCG, ch1=SP1 LCG (same photodiode: equal e-/us), ch2/ch3=SP2 (1/14.5-1/16 "
                                "sensitivity). Dark range of HDR = ch0 alone. SP1->SP2 switch near 4-8e4 DN (0 dB), "
                                "alpha_local jumps x16; R switches earlier than G (2x2 decision?)")
    (dest / "noise_model.json").write_text(json.dumps(doc, indent=1, default=float))
    print(f"[{key}] written {dest}")


if __name__ == "__main__":
    for k in ("bfs", "lucid"):
        build(k)
