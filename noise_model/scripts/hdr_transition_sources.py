"""Which photodiode/conversion gain produced each HDR value: per pixel, per frame.

  python hdr_transition_sources.py ../data/hdr_transition_<date> classify 2.5ms 3ms 4ms 5ms
  python hdr_transition_sources.py ../data/hdr_transition_<date> rule 2.5ms 3ms 4ms 5ms

The combined HDR output is an integer rounding of K_c * code + b, where code is the 12-bit code of
the channel the sensor picked and K_c is a per-CFA-position scale (measured from the stacks):
values coming from different channels sit on different lattices (SP1 LCG ~21.7 DN apart, SP2 HCG
~140-157 DN, SP2 LCG 37x that). Each pixel's lattice offset is taken from a reference level where
every pixel uses that channel (one global shift per level is re-fitted), and each value is assigned
to the lattice it lies on (|distance| < TOL DN). Chance coincidence: 2*TOL/K (e.g. 1.7 % for SP1
values on the SP2 lattice), corrected for in the rates below.

classify -> <level>/sources.npz : src (n, h, w) int8 for the paper region (0 other, 1 SP1 LCG,
            2 SP2 HCG, 3 SP2 LCG), region offsets, K table, global shifts.
rule     -> quad agreement, and the switching variable: for frames that stayed on SP1, the largest
            value of each candidate decision variable (a sharp cut = the variable the sensor uses).
"""
import json
import sys
from pathlib import Path

import numpy as np

POS = {"R": (0, 0), "Gr": (0, 1), "Gb": (1, 0), "B": (1, 1)}
# lattice steps (HDR DN per channel code), fitted 2026-10-20 on the 0 dB transition session
K = {1: {"R": 21.7136, "Gr": 21.7056, "Gb": 21.7542, "B": 21.7180},
     2: {"R": 156.5116, "Gr": 144.9510, "Gb": 146.4790, "B": 140.3690}}
REF = {1: "2ms", 2: "6ms"}       # every paper pixel on that channel
TOL = 1.2
BLOCK = 58


def region(H, W):
    return H // 5 // 2 * 2, 4 * H // 5 // 2 * 2, W // 5 // 2 * 2, 4 * W // 5 // 2 * 2


def ref_phase(sess, lev, lat, r0, r1, c0, c1):
    """Per-pixel lattice phase (radians) for one lattice, on the paper region."""
    st = np.load(sess / lev / "bright_hdr.npy", mmap_mode="r")
    out = np.empty((r1 - r0, c1 - c0))
    kmap = kgrid(lat, r1 - r0, c1 - c0)
    for r in range(r0, r1, BLOCK):
        x = st[:, r:min(r + BLOCK, r1), c0:c1].astype(np.float64)
        out[r - r0:r - r0 + x.shape[1]] = np.angle(np.exp(2j * np.pi * x / kmap[r - r0:r - r0 + x.shape[1]]).mean(0))
    return out


def kgrid(lat, h, w):
    k = np.empty((h, w))
    for name, (dy, dx) in POS.items():
        k[dy::2, dx::2] = K[lat][name]
    return k


def classify(sess, levels):
    st0 = np.load(sess / REF[1] / "bright_hdr.npy", mmap_mode="r")
    H, W = st0.shape[1:]
    r0, r1, c0, c1 = region(H, W)
    h, w = r1 - r0, c1 - c0
    phases = {lat: ref_phase(sess, REF[lat], lat, r0, r1, c0, c1) for lat in (1, 2)}
    kmaps = {lat: kgrid(lat, h, w) for lat in (1, 2)}
    for lev in levels:
        st = np.load(sess / lev / "bright_hdr.npy", mmap_mode="r")
        n = st.shape[0]
        # residual (DN) of every value against each pixel's reference lattice
        res = {lat: np.empty((n, h, w), np.float32) for lat in (1, 2)}
        for r in range(r0, r1, BLOCK):
            x = st[:, r:min(r + BLOCK, r1), c0:c1].astype(np.float64)
            sl = slice(r - r0, r - r0 + x.shape[1])
            for lat in (1, 2):
                k = kmaps[lat][sl]
                res[lat][:, sl] = np.mod(x - phases[lat][sl] * k / (2 * np.pi), k)
        src = np.zeros((n, h, w), np.int8)
        shifts, rates = {}, {}
        for name, (dy, dx) in POS.items():
            for lat in (1, 2):
                k = K[lat][name]
                rr = res[lat][:, dy::2, dx::2]
                hist, edges = np.histogram(rr, bins=int(k * 4), range=(0, k))
                s = (edges[np.argmax(hist)] + 0.125)
                shifts[f"{name}_{lat}"] = float(s)
                d = np.abs(np.mod(rr - s + k / 2, k) - k / 2)
                on = d < TOL
                view = src[:, dy::2, dx::2]
                if lat == 1:
                    view[on & (view == 0)] = 1
                else:
                    view[on] = 2          # SP2 wins: SP1 values hit the SP2 lattice by chance only 2*TOL/K2
            v = src[:, dy::2, dx::2]
            rates[name] = {c: float((v == c).mean()) for c in (0, 1, 2)}
        np.savez_compressed(sess / lev / "sources.npz", src=src, r0=r0, c0=c0,
                            K=json.dumps(K), shifts=json.dumps(shifts))
        print(f"{lev}: " + "  ".join(f"{nm} SP1 {r[1]:.3f} SP2 {r[2]:.3f} other {r[0]:.4f}" for nm, r in rates.items()),
              flush=True)


def rule(sess, levels):
    for lev in levels:
        z = np.load(sess / lev / "sources.npz")
        src, r0, c0 = z["src"], int(z["r0"]), int(z["c0"])
        n, h, w = src.shape
        st = np.load(sess / lev / "bright_hdr.npy", mmap_mode="r")
        v = np.asarray(st[:, r0:r0 + h, c0:c0 + w]).astype(np.float64)
        q = lambda a, nm: a[:, POS[nm][0]::2, POS[nm][1]::2]
        s = {nm: q(src, nm) for nm in POS}
        x = {nm: q(v, nm) for nm in POS}
        known = np.all([s[nm] > 0 for nm in POS], axis=0)
        same = known & (s["R"] == s["Gr"]) & (s["Gr"] == s["Gb"]) & (s["Gb"] == s["B"])
        sp2 = np.mean([s[nm] == 2 for nm in POS], axis=0)
        print(f"== {lev}: quad-frames with all 4 sources identified {known.mean():.4f}; of those, all on the same "
              f"photodiode {same[known].mean():.4f}; quads switched in some frames {((sp2 > 0.5).any(0)).mean():.4f}")
        stay = known & (s["R"] == 1) & (s["Gr"] == 1) & (s["Gb"] == 1) & (s["B"] == 1)
        sw = known & (s["R"] == 2) & (s["Gr"] == 2) & (s["Gb"] == 2) & (s["B"] == 2)
        cands = {"Gr": x["Gr"], "Gb": x["Gb"], "max(Gr,Gb)": np.maximum(x["Gr"], x["Gb"]),
                 "mean(Gr,Gb)": (x["Gr"] + x["Gb"]) / 2, "max(R,Gr,Gb,B)": np.max([x[k] for k in POS], axis=0),
                 "sum4": sum(x[k] for k in POS), "R": x["R"], "B": x["B"]}
        mu = {nm: x[nm].mean(0) for nm in POS}
        for cn, c in cands.items():
            a = c[stay]
            top = np.sort(a)[-5:]
            # how many SP1-stayed frames sit within 0.5 % below the max: a hard cut piles values up to it
            print(f"   {cn:15s} SP1 frames: max {top[-1]:10.0f}  next {top[-2]:10.0f} {top[-3]:10.0f}   "
                  f"99.9% {np.percentile(a, 99.9):10.0f}")


def main():
    sess = Path(sys.argv[1])
    {"classify": classify, "rule": rule}[sys.argv[2]](sess, sys.argv[3:])


if __name__ == "__main__":
    main()
