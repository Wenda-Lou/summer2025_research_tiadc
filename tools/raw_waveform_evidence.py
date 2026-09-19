"""Raw-waveform evidence: look at A, B and A-B directly, with the estimators out of the way.

The professor's question: are the impulses in the shared captures, and has the raw
two-channel output been examined as gain/skew/offset change, *without* algorithms?  This
does exactly that for the two actuator states we have captures for, using only:

  1. the documented unpacking (int16 -> codes, 8-word groups = 4 samples of A then 4 of B),
     and "pick the split phase with the largest |corr(A, B)|";
  2. a least-squares sinusoid + DC fit at the tone frequency, refined over a small grid
     (the record is 156.43 tone cycles, so a *fixed* frequency leaks into the amplitude --
     an unrefined fit reports a spurious ~2 % A/B amplitude difference);
  3. means, differences, and folding the record modulo the 130-sample impulse period.

No polarity anchor, no dither template projection, no loop, no correction, no estimator.

    python tools/raw_waveform_evidence.py
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

FRAME_BYTES = 4095
GROUP, HALF = 8, 4
FS_ADC = 1.300e9
F0 = 1276 / 8320            # tone, cycles per ADC sample
PERIOD = 130                # ADC samples between impulses
WIN = 40                    # half-width of the folded replica window

SETS = (
    ("neutral code 24 (skew ~-44 ps)", "frames_neutral"),
    ("converged code 31 (skew ~-9.7 ps)", "dither_vs_tone_frames"),
)


def decode(raw: bytes) -> tuple[np.ndarray, np.ndarray]:
    """Unpack one frame and split it into A and B, sign-normalising B."""
    words = np.frombuffer(raw[:FRAME_BYTES - (FRAME_BYTES % 2)], dtype="<i2")
    groups = words[:words.size // GROUP * GROUP].reshape(-1, GROUP)

    def split(g):
        a = (g[:, :HALF].astype(np.int32) >> 2).ravel().astype(float)
        b = (g[:, HALF:].astype(np.int32) >> 2).ravel().astype(float)
        return a, b

    best, best_c = 0, -1.0
    for r in range(GROUP):
        a, b = split(np.roll(groups, -r, axis=0))
        c = abs(float(np.corrcoef(a, b)[0, 1]))
        if c > best_c:
            best, best_c = r, c
    a, b = split(np.roll(groups, -best, axis=0))
    if float(np.corrcoef(a, b)[0, 1]) < 0:      # one branch of the splitter is inverted
        b = -b
    return a, b


def fit_tone(x: np.ndarray, grid: int = 41) -> dict:
    """Least-squares sinusoid + DC, with the frequency refined over a small grid."""
    n = np.arange(x.size)
    span = 0.5 / x.size
    best = None
    for f in np.linspace(F0 - span, F0 + span, grid):
        w = 2.0 * np.pi * f * n
        design = np.column_stack([np.cos(w), np.sin(w), np.ones_like(w)])
        coef, *_ = np.linalg.lstsq(design, x, rcond=None)
        model = design @ coef
        sse = float((x - model) @ (x - model))
        if best is None or sse < best[0]:
            best = (sse, coef, f, model)
    sse, coef, f, model = best
    return {"amp": float(math.hypot(coef[0], coef[1])), "freq": float(f), "model": model,
            "resid_rms": float(np.sqrt(sse / x.size))}


def replica(frames, chan: int, half: int = WIN):
    """The impulse replica as the ADC saw it: fold the tone-removed record on its impulses.

    The impulses repeat every PERIOD samples, so their phase within a frame is found once
    (peak of the tone-removed residual in the first period) and every impulse in the frame
    is then averaged about it, across all frames.
    """
    acc, n = None, 0
    for a, b in frames:
        x = a if chan == 0 else b
        r = x - fit_tone(x)["model"]
        k0 = int(np.argmax(np.abs(r[:PERIOD])))
        for k in range(k0, x.size - half, PERIOD):
            if k < half:
                continue
            w = r[k - half:k + half + 1]
            acc = w if acc is None else acc + w
            n += 1
    return acc / max(n, 1), n


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_out", "raw_evidence"))
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data, stats = [], []
    for label, set_dir in SETS:
        files = sorted(glob.glob(os.path.join(REPO, "calibration_out", set_dir,
                                              "frame_*.bin")))
        frames = [decode(open(f, "rb").read()) for f in files]
        fa = [fit_tone(a) for a, _ in frames]
        fb = [fit_tone(b) for _, b in frames]
        amp_a = np.array([f["amp"] for f in fa])
        amp_b = np.array([f["amp"] for f in fb])
        dc = np.array([(a - b).mean() for a, b in frames])
        ab = np.array([fit_tone(a - b)["amp"] for a, b in frames])
        rep_a, n_ev = replica(frames, 0)
        rep_b, _ = replica(frames, 1)
        ratio = min(ab.mean() / (2.0 * amp_a.mean()), 1.0)
        dt_ps = math.asin(ratio) / (math.pi * F0) / FS_ADC * 1e12
        data.append((label, frames, rep_a, rep_b))
        stats.append({"label": label, "n": len(frames), "amp_a": amp_a.mean(),
                      "amp_b": amp_b.mean(), "ratio": amp_b.mean() / amp_a.mean(),
                      "dc": dc.mean(), "dc_sd": dc.std(ddof=1), "ab": ab.mean(),
                      "dt": dt_ps, "resid": np.mean([f["resid_rms"] for f in fa]),
                      "rep": float(np.max(rep_a) - np.min(rep_a)), "ev": n_ev})

    print("raw two-channel numbers, no estimator (per-frame, then averaged)\n")
    print(f"{'state':>32}{'n':>4}{'|A|':>8}{'|B|':>8}{'B/A':>9}{'A-B dc':>8}"
          f"{'|A-B| f0':>10}{'dt from |A-B|':>14}{'pulse pk':>10}")
    for s in stats:
        print(f"{s['label']:>32}{s['n']:>4}{s['amp_a']:>8.1f}{s['amp_b']:>8.1f}"
              f"{s['ratio']:>9.5f}{s['dc']:>8.2f}{s['ab']:>10.2f}"
              f"{s['dt']:>12.1f} ps{s['rep']:>10.1f}")
    print(f"\n(pulse pk = peak-to-peak of the folded replica, {stats[0]['ev']} impulse "
          f"windows averaged; tone-fit residual rms {stats[0]['resid']:.1f} codes)")

    fig, ax = plt.subplots(2, 2, figsize=(13, 7.5))
    a0, b0 = data[0][1][0][0], data[0][1][0][1]
    n = np.arange(a0.size)
    ax[0, 0].plot(n, a0, lw=0.8, color="#3060a0")
    ax[0, 0].set_title("Channel A, one raw capture (1020 samples)\n"
                       "tone at 199.375 MHz, one impulse every 130 samples", fontsize=10)
    ax[0, 0].set_xlabel("sample")
    ax[0, 0].set_ylabel("ADC codes")

    z = slice(0, 300)
    ax[0, 1].plot(n[z], a0[z], lw=1.0, color="#3060a0", label="A")
    ax[0, 1].plot(n[z], b0[z], lw=1.0, color="#a05030", label="B (sign-normalised)")
    ax[0, 1].plot(n[z], (a0 - b0)[z], lw=0.9, color="k", label="A - B")
    ax[0, 1].set_title("Zoom on ~2 impulses: the dither is in the data\n"
                       "(A and B nearly overlap: same sampling instant, ~1.0004 gain ratio)",
                       fontsize=10)
    ax[0, 1].set_xlabel("sample")
    ax[0, 1].legend(fontsize=8)

    off = np.arange(-WIN, WIN + 1)
    for (label, _, rep_a, rep_b) in data:
        style = "-" if "24" in label else "--"
        ax[1, 0].plot(off, rep_a, style, lw=1.3, label=f"A — {label}")
        ax[1, 0].plot(off, rep_b, style, lw=1.0, alpha=0.55, label=f"B — {label}")
    ax[1, 0].set_title("Impulse replica the ADC actually sees\n"
                       "(tone removed by one sinusoid fit, then folded on the impulses)",
                       fontsize=10)
    ax[1, 0].set_xlabel("samples from the impulse")
    ax[1, 0].set_ylabel("codes")
    ax[1, 0].legend(fontsize=8)

    for (label, frames, _, _) in data:
        d = frames[0][0] - frames[0][1]
        ax[1, 1].plot(np.arange(d.size), d, lw=0.9,
                      label=f"{label.split(' (')[0]}: |A-B| = {fit_tone(d)['amp']:.1f} codes")
    ax[1, 1].set_title("A - B in one capture from each state, same scale\n"
                       "smaller amplitude = smaller timing skew", fontsize=10)
    ax[1, 1].set_xlabel("sample")
    ax[1, 1].set_ylabel("codes")
    ax[1, 1].legend(fontsize=8)

    for a in ax.ravel():
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    png = os.path.join(args.out, "raw_waveform_evidence.png")
    fig.savefig(png, dpi=140)
    plt.close(fig)
    print(f"figure: {png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
