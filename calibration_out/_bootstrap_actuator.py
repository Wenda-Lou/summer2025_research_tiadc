"""Bootstrap argument for the actuator experiment: how many frames per point?

Uses the 100 frames saved in calibration_out/frames100 (read off the bench once;
everything here is offline).

Two different questions, deliberately kept apart:

A. Single-point estimation.  "After averaging N frames, how well is the current
   skew known?"  -> bootstrap SE, 95 % CI half-width, and the probability that a
   deadband rule fires on noise alone.

B. Two-batch comparison.  "Can two independent N-frame batches tell a 13.8 ps
   actuator step apart?"  -> bootstrap detection power against a 5 % false-positive
   threshold, sign-correct probability, and the N needed for 80/90/95 % power.

The +13.8 ps injected into the stepped batch is an *ideal actuator* assumption:
one firmware control code moves A and B in opposite directions by 4 raw 1.725 ps
taps each, so the observable B-A change is 13.8 ps.  A real write may add noise or
a JESD state change that this model does not contain, so the powers below are
upper bounds on what the bench will actually deliver.

Frame acceptance (data-driven, not just skew_source): the centroid observable is
known to be unusable (+-600 ps), so frames the estimator rejected as
"observables-disagree" are *kept* when the phase route and the independent
difference-signal route agree, the tone and margin look normal, and the source is
not the profile fallback.  Population sizes for both filters are reported.
"""

from __future__ import annotations

import csv
import glob
import math
import os
import statistics as st
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                        # noqa: E402
from calibration_loop.estimator import (                                # noqa: E402
    estimate_block, fit_tone, prepare_capture,
)

DIR = os.path.join(REPO, "calibration_out", "frames100")
OUT = os.path.join(REPO, "calibration_out", "bootstrap")
STEP_PS = 13.8          # one firmware control step, differential
N_GRID = (5, 10, 15, 20, 25, 30, 40, 50, 75)
B_IID = 50_000
B_BLOCK = 20_000
SEED = 20260917


def trimmed_mean(a, axis=-1, frac=0.10):
    s = np.sort(a, axis=axis)
    k = int(s.shape[axis] * frac)
    if k == 0:
        return s.mean(axis=axis)
    sl = [slice(None)] * s.ndim
    sl[axis] = slice(k, s.shape[axis] - k)
    return s[tuple(sl)].mean(axis=axis)


def load_population():
    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    ts = 1e12 / cfg.fs_adc
    ps_per_rad = ts / (2 * math.pi * f0)
    files = sorted(glob.glob(os.path.join(DIR, "frame_*.bin")))
    if not files:
        raise SystemExit(f"no frames in {DIR}; run _capture_frames.py first")

    rec = []
    for path in files:
        with open(path, "rb") as fh:
            raw = fh.read()
        prep = prepare_capture(raw, cfg)
        a, b = prep["ch_a"], prep["ch_b"]
        est = estimate_block(a, b, cfg, n0=prep["n0"])
        f_c = 0.5 * (fit_tone(a, f0, refine=True)["f0"]
                     + fit_tone(b, f0, refine=True)["f0"])
        fa = fit_tone(a, f_c, refine=False)
        fb = fit_tone(b, f_c, refine=False)
        dphi = math.atan2(math.sin(fb["phase"] - fa["phase"]),
                          math.cos(fb["phase"] - fa["phase"]))
        d = a - b
        amp_ab = abs(fit_tone(d - d.mean(), f_c, refine=False)["amplitude"])
        ratio = min(1.0, amp_ab / (2.0 * fa["amplitude"])) if fa["amplitude"] > 0 else 0.0
        rec.append({
            "file": os.path.basename(path),
            "phase_ps": dphi * ps_per_rad,
            "diff_ps": -2.0 * math.asin(ratio) * ps_per_rad,
            "source": est.skew_source,
            "margin": prep["align_margin"],
            "events": est.ch_a.n_events_used,
            "tone": fa["amplitude"],
            "resid": float(np.std(fa["residual"])),
        })
    return rec


def population(rec, tol_ps):
    """Acceptance filter: phase usable, both routes agree, frame looks sane."""
    phase = np.array([r["phase_ps"] for r in rec])
    diff = np.array([r["diff_ps"] for r in rec])
    agree = np.abs(phase - diff)
    tone = np.array([r["tone"] for r in rec])
    resid = np.array([r["resid"] for r in rec])
    margin = np.array([r["margin"] for r in rec])
    events = np.array([r["events"] for r in rec])
    src = np.array([r["source"] for r in rec])
    tone_med = float(np.median(tone))
    resid_med = float(np.median(resid))
    keep = ((src != "profile-fallback") & np.isfinite(phase) & np.isfinite(diff)
            & (agree <= tol_ps) & (np.abs(tone - tone_med) <= 0.10 * tone_med)
            & (resid > 0.5 * resid_med) & (resid < 2.0 * resid_med)
            & (margin >= 6.0) & (events >= 7))
    return phase[keep], diff[keep], agree, keep


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    rng = np.random.default_rng(SEED)
    rec = load_population()
    all_phase = np.array([r["phase_ps"] for r in rec])
    agree_all = np.abs(all_phase - np.array([r["diff_ps"] for r in rec]))
    print(f"frames analysed: {len(rec)}")
    print(f"route agreement residual: median {np.median(agree_all):.3f} ps  "
          f"p95 {np.percentile(agree_all, 95):.3f} ps  max {agree_all.max():.3f} ps")

    strict, _, _, keep_strict = population(rec, 2.0)
    phase, diff, agree, keep = population(rec, 5.0)
    src_kept = {}
    for r, k in zip(rec, keep):
        src_kept[r["source"]] = src_kept.get(r["source"], 0) + (1 if k else 0)
    print(f"accepted with tol=2 ps: {keep_strict.sum()}/{len(rec)}   "
          f"tol=5 ps: {keep.sum()}/{len(rec)}")
    print(f"accepted by original skew_source: {src_kept}")

    # why are frames dropped?  each criterion on its own
    src = np.array([r["source"] for r in rec])
    ph = np.array([r["phase_ps"] for r in rec])
    df = np.array([r["diff_ps"] for r in rec])
    tone = np.array([r["tone"] for r in rec])
    resid = np.array([r["resid"] for r in rec])
    margin = np.array([r["margin"] for r in rec])
    events = np.array([r["events"] for r in rec])
    reasons = {
        "profile-fallback source": src == "profile-fallback",
        "phase not finite": ~np.isfinite(ph),
        "routes disagree": np.abs(ph - df) > 5.0,
        "tone off by >10%": np.abs(tone - np.median(tone)) > 0.10 * np.median(tone),
        "residual out of 0.5-2x": (resid <= 0.5 * np.median(resid)) | (resid >= 2.0 * np.median(resid)),
        "margin < 6": margin < 6.0,
        "events < 7": events < 7,
    }
    print("rejection reasons (a frame can fail more than one):")
    for name, mask in reasons.items():
        print(f"  {name:<24}{int(mask.sum()):>4} frames fail this alone")
    print(f"  frames failing nothing   {int((~keep).sum()):>4} rejected in total")
    print(f"accepted population: mean {phase.mean():+.2f} ps  std {phase.std():.2f} ps  "
          f"n={phase.size}")
    print(f"yield = {keep.sum() / len(rec):.3f}  -> acquisitions per accepted frame "
          f"= {1.0 / (keep.sum() / len(rec)):.2f}")

    sigma = float(phase.std())
    rows = []

    # ---------------- A: single-point estimation ----------------------------
    print("\nA. single-point estimate of the current skew")
    print(f"{'N':>4}{'mean SE':>10}{'med SE':>9}{'trim SE':>9}{'CI95 half':>11}"
          f"{'P(|e|>8ps)':>12}{'P(|e|>13.8)':>13}")
    for n in N_GRID:
        idx = rng.integers(0, phase.size, size=(B_IID, n))
        s = phase[idx]
        m = s.mean(axis=1)
        md = np.median(s, axis=1)
        tm = trimmed_mean(s, axis=1)
        ci = float(np.percentile(m, 97.5) - np.percentile(m, 2.5)) / 2.0
        dev = np.abs(m - phase.mean())
        p8 = float(np.mean(dev > 8.0))
        p14 = float(np.mean(dev > STEP_PS))
        print(f"{n:>4}{m.std(ddof=1):>10.2f}{md.std(ddof=1):>9.2f}{tm.std(ddof=1):>9.2f}"
              f"{ci:>11.2f}{p8:>12.3f}{p14:>13.3f}")
        rows.append({"part": "A", "N": n, "population": f"tol5 ({phase.size})",
                     "estimator": "mean", "SE_ps": round(float(m.std(ddof=1)), 3),
                     "CI95_half_ps": round(ci, 3),
                     "false_trigger_8ps": round(p8, 4),
                     "false_trigger_13p8ps": round(p14, 4),
                     "detection_power": "", "sign_correct": ""})
        rows.append({"part": "A", "N": n, "population": f"tol5 ({phase.size})",
                     "estimator": "median", "SE_ps": round(float(md.std(ddof=1)), 3),
                     "CI95_half_ps": "", "false_trigger_8ps": "", "false_trigger_13p8ps": "",
                     "detection_power": "", "sign_correct": ""})
        rows.append({"part": "A", "N": n, "population": f"tol5 ({phase.size})",
                     "estimator": "trim10", "SE_ps": round(float(tm.std(ddof=1)), 3),
                     "CI95_half_ps": "", "false_trigger_8ps": "", "false_trigger_13p8ps": "",
                     "detection_power": "", "sign_correct": ""})

    # ---------------- B: two-batch comparison -------------------------------
    print("\nB. two independent N-frame batches, +13.8 ps injected (ideal actuator)")
    print(f"{'N':>4}{'null p95':>10}{'diff SE':>9}{'CI95 half':>11}{'power':>8}"
          f"{'sign ok':>9}{'blk2 pw':>9}{'blk5 pw':>9}")
    power_curve = {}
    for n in N_GRID:
        # null: both batches from the same population -> the 5 % threshold
        a0 = phase[rng.integers(0, phase.size, size=(B_IID, n))].mean(axis=1)
        b0 = phase[rng.integers(0, phase.size, size=(B_IID, n))].mean(axis=1)
        thr = float(np.percentile(np.abs(b0 - a0), 95.0))
        # injected
        bi = phase[rng.integers(0, phase.size, size=(B_IID, n))].mean(axis=1) + STEP_PS
        d = bi - a0
        se = float(d.std(ddof=1))
        halves = (float(np.percentile(d, 97.5) - np.percentile(d, 2.5)) / 2.0)
        power = float(np.mean(np.abs(d) > thr))
        sign_ok = float(np.mean(d > 0))
        # moving-block sensitivity check: exactly the same two-batch test, but with
        # resampling in blocks of L consecutive accepted frames
        blk_pow, blk_se = {}, {}

        def block_batch(_rng=None, _L=None, _nb=None, _n=None):
            r = rng if _rng is None else _rng
            L, nb, nn = _L, _nb, _n
            starts = r.integers(0, phase.size - L + 1, size=(B_BLOCK, nb))
            idx = starts[:, :, None] + np.arange(L)[None, None, :]
            return phase[idx].reshape(B_BLOCK, -1)[:, :nn].mean(axis=1)

        for L in (2, 5):
            nb = max(1, n // L)
            aa = block_batch(_L=L, _nb=nb, _n=n)
            bb = block_batch(_L=L, _nb=nb, _n=n)
            blk_se[L] = float((bb - aa).std(ddof=1))
            t2 = float(np.percentile(np.abs(bb - aa), 95.0))
            blk_pow[L] = float(np.mean(np.abs((bb + STEP_PS) - aa) > t2))
        power_curve[n] = power
        print(f"{n:>4}{thr:>10.2f}{se:>9.2f}{halves:>11.2f}{power:>8.3f}"
              f"{sign_ok:>9.3f}{blk_pow[2]:>9.3f}{blk_pow[5]:>9.3f}"
              f"   (block SE {blk_se[2]:.2f}/{blk_se[5]:.2f} vs IID {se:.2f})")
        rows.append({"part": "B", "N": n, "population": "ideal +13.8 ps",
                     "estimator": "mean", "SE_ps": round(se, 3),
                     "CI95_half_ps": round(halves, 3), "false_trigger_8ps": "",
                     "false_trigger_13p8ps": "", "detection_power": round(power, 4),
                     "sign_correct": round(sign_ok, 4)})

    nn = np.array(N_GRID, dtype=float)
    pw = np.array([power_curve[n] for n in N_GRID])
    print("\nN needed for a given power (linear interpolation on the curve):")
    need = {}
    for target in (0.80, 0.90, 0.95):
        if pw.max() < target:
            need[target] = None
        else:
            need[target] = float(np.interp(target, pw, nn))
        got = need[target]
        txt = f"{got:.0f} valid frames/point" if got else "not reached in this N grid"
        extra = f"   ({got / (keep.sum() / len(rec)):.0f} acquisitions/point)" if got else ""
        print(f"  {target:.0%} power -> {txt}{extra}")

    # ---------------- artefacts --------------------------------------------
    with open(os.path.join(OUT, "bootstrap_summary.csv"), "w", newline="",
              encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(7, 4.5))
        ax.plot(N_GRID, [sigma / math.sqrt(n) for n in N_GRID], "o-",
                label="single point, SE = sigma/sqrt(N)")
        ax.plot(N_GRID, [sigma * math.sqrt(2 / n) for n in N_GRID], "s-",
                label="two-batch difference, SE")
        ax.plot(N_GRID, [1.96 * sigma / math.sqrt(n) for n in N_GRID], "^--",
                label="single point, 95% CI half-width")
        ax.axhline(STEP_PS, color="k", ls=":", label="one control step 13.8 ps")
        ax.axhline(sigma, color="r", ls=":", label="per-frame scatter 18.9 ps")
        ax.set_xlabel("valid frames per point (N)")
        ax.set_ylabel("picoseconds")
        ax.set_title("Bootstrap uncertainty vs N (100 bench frames)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "bootstrap_se_vs_n.png"), dpi=130)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 4.5))
        ax.plot(N_GRID, pw, "o-", label="detection power (5% false-positive)")
        ax.plot(N_GRID, [power_curve[n] for n in N_GRID], ":", alpha=0.0)
        for target, style in ((0.80, "--"), (0.90, "--"), (0.95, "--")):
            if need[target]:
                ax.axvline(need[target], color="grey", ls=style, alpha=0.7)
                ax.text(need[target], 0.05, f" {target:.0%}: N={need[target]:.0f}",
                        rotation=90, fontsize=8, va="bottom")
        ax.axhline(0.8, color="grey", ls=":", alpha=0.6)
        ax.set_xlabel("valid frames per point (N)")
        ax.set_ylabel("probability")
        ax.set_ylim(0, 1.02)
        ax.set_title("Two-batch detection of an ideal +13.8 ps step")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "bootstrap_power_vs_n.png"), dpi=130)
        plt.close(fig)
        print(f"\nwrote {OUT}/bootstrap_se_vs_n.png, bootstrap_power_vs_n.png, "
              f"bootstrap_summary.csv")
    except Exception as exc:  # noqa: BLE001
        print(f"(plots skipped: {exc})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
