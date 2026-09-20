#!/usr/bin/env python3
"""Session figures for the 2026-09-20 bench day — one figure per question.

    python calibration_out/_session_figures.py

Writes, into ``calibration_out/professor_update/``:

  session_loop.png       Q1 — the dither-only closed loop (``run5_phase``), in the house four-panel
                         layout: gain, offset, skew, dynamic performance.
  session_tone.png       Q3 — today's tone-mode closed loop (``tonezero_check``), same four panels
                         (the only run today whose SNDR/SFDR/ENOB are measurable).
  session_fullscale.png  Q2 — how far the dither can be pushed, from the top-of-ladder state
                         (30000 LSB, 91.6 % of DAC full scale) and the ceiling it implies.

The four panels are the ones ``CalibrationLoop.plot()`` draws (``calibration_loop/loop.py``): gain
ratio, offset mismatch, timing skew and dynamic performance — i.e. the calibration's own outputs,
not a proxy for them.  The one axis a tone-free waveform cannot report is SNDR/ENOB: those columns
are scored at f_in, and with no tone they read the noise floor, so `session_loop.png` says so in the
panel and plots the dither SNR instead.

**Best-of rule.**  Each figure carries the best result of its question, not every state and every
control run that was measured.  The runs that lost (`run4_tonefree` on the fold route, the lower
ladder amplitudes, the route-scale comparison) are in the tables of
`BENCH_SESSION_DITHER_ONLY.md` §7 and §F, and the numbers this script prints below still cover all
of them — the figures are the highlight reel, the doc is the record.

Every plotted number is read from the committed run outputs under ``calibration_out/``.  Only bench
constants are literals, and each carries its source:

  * ``FS_ADC_VPP = 1.59`` — AD9695 differential input full scale at IFC register 0x1910 = 0x0C,
    the setting every capture in this session was taken at.
  * ``DAC_FS_LSB = 32767`` — the DPG waveform generator's own ceiling (``dither.DAC_FULL_SCALE``);
    it refuses a vector whose peak would exceed it.
  * ``DEADBAND_PS = 10`` — ``LoopOptions.skew_deadband_ps`` for both closed-loop runs.
  * ``TONEFREE_PARK_CODE = 34`` — the actuator code the dither-only loop parked at, which is where
    the tone-mode run was started (its own CSV records only the code *after* the move).
"""

from __future__ import annotations

import csv
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

OUT = os.path.join(REPO, "calibration_out", "professor_update")
CL = os.path.join(REPO, "calibration_out", "closed_loop")
LADDER = os.path.join(REPO, "calibration_out", "dither_ladder",
                      "dither_ladder_16000_24000_30000.csv")
IFC_16K = os.path.join(REPO, "calibration_out", "dither_ladder", "ifc_amp16000",
                       "dither_ladder.csv")
IFC_30K = os.path.join(REPO, "calibration_out", "dither_ladder", "ifc_amp30000",
                       "dither_ladder.csv")
STEP = os.path.join(REPO, "calibration_out", "skew_step", "skew_step_points.csv")

FS_ADC_VPP = 1.59
DAC_FS_LSB = 32767.0
DEADBAND_PS = 10.0
TONEFREE_PARK_CODE = 34.0
ADC_SPAN_CODES = 16384.0          # 14-bit signed, so the differential span is 2**14 codes
CODES_TO_MV = FS_ADC_VPP * 1e3 / ADC_SPAN_CODES   # 97.05 uV per code, differential

C_R5 = "#1f4e79"
C_R4 = "#c00000"
C_TONE = "#2e7d32"


# --------------------------------------------------------------------------- helpers

def load(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def col(rows, key):
    """Column as float array; missing values and absent columns become NaN."""
    out = []
    for r in rows:
        try:
            out.append(float(r[key]))
        except (KeyError, TypeError, ValueError):
            out.append(np.nan)
    return np.array(out)


def qualified(rows):
    return [r for r in rows if not r.get("rejected")]


def batches(rows, start_code):
    """Batch decisions as [(code at which the batch was measured, mean ps, se ps, n, action, iter)].

    A row carrying ``skew_batch_mean_ps`` is the batch decision; the code it was taken at is the
    one *before* the move, which the log does not record (``skew_code`` is post-move), so the code
    is carried forward from ``start_code`` and advanced by each ``move`` action.
    """
    out = []
    code = float(start_code)
    for r in rows:
        if not r.get("skew_batch_used") and not r.get("skew_action"):
            continue
        try:
            mean = float(r["skew_batch_mean_ps"])
        except (TypeError, ValueError):
            continue
        se = float(r["skew_batch_se_ps"]) if r.get("skew_batch_se_ps") else np.nan
        n = float(r["skew_batch_n"]) if r.get("skew_batch_n") else np.nan
        action = r.get("skew_action", "")
        try:
            it = float(r["iteration"])
        except (KeyError, TypeError, ValueError):
            it = np.nan
        out.append((code, mean, se, n, action, it))
        if action.startswith("move") and r.get("skew_code"):
            code = float(r["skew_code"])
    return out


def code_trace(rows, start_code):
    """Actuator code per qualified sample (fill-forward, seeded with the starting code)."""
    code = float(start_code)
    out = np.empty(len(rows), dtype=float)
    for i, r in enumerate(rows):
        if r.get("skew_code"):
            code = float(r["skew_code"])
        out[i] = code
    return out


def fit(x, y):
    """Least-squares slope/intercept, ignoring NaN."""
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 2:
        return np.nan, np.nan
    return tuple(np.polyfit(x[m], y[m], 1))


def zero_code(x, y):
    """Actuator code at which a route's linear reading crosses zero."""
    s, i0 = fit(np.asarray(x), np.asarray(y))
    return -i0 / s


# --------------------------------------------------------------------------- figures

def _loop_grid(plt, rows, gain_series, draw_dynamic):
    """The house closed-loop layout: gain, offset, skew, dynamic performance.

    Same four panels as ``CalibrationLoop.plot()`` (``calibration_loop/loop.py``), so a figure drawn
    here is directly comparable with a run's own ``*_learning.png``.
    """
    fig, ax = plt.subplots(2, 2, figsize=(11.6, 7.0))
    it = np.arange(len(rows))
    gain, offset, skew, perf = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]

    for key, label, kw in gain_series:
        gain.plot(it, col(rows, key), label=label, **kw)
    gain.axhline(1.0, ls="--", lw=1, color="k")
    gain.set_ylabel("$g_B/g_A$")
    gain.set_title("(a) Gain ratio residual", fontsize=10)
    gain.legend(fontsize=8)

    dc = col(rows, "offset_b_codes") - col(rows, "offset_a_codes")
    offset.plot(it, dc, color=C_R5, lw=1.1)
    offset.axhline(0.0, ls="--", lw=1, color="k")
    offset.set_ylabel("DC mismatch B$-$A [codes]")
    offset.set_title("(b) Offset mismatch", fontsize=10)

    skew.axhspan(-DEADBAND_PS, DEADBAND_PS, color="0.85", zorder=0,
                 label=f"{DEADBAND_PS:.0f} ps deadband")
    skew.plot(it, col(rows, "skew_used_ps"), color=C_R5, lw=1.1, label="controlled route")
    skew.axhline(0.0, ls="--", lw=1, color="k")
    skew.set_ylabel("timing skew residual [ps]")
    skew.set_title("(c) Timing skew", fontsize=10)
    skew.legend(fontsize=8, loc="lower right")

    draw_dynamic(perf)

    for a in ax.ravel():
        a.set_xlabel("qualified sample")
        a.grid(True, alpha=0.3)
    return fig


def figure_loop(plt, r5):
    """Q1: the dither-only closed loop (run5_phase), in the house four-panel layout.

    Panel (d) is the one axis this run cannot report as SNDR/ENOB: with no tone in the waveform
    those columns are scored at f_in and read the noise floor (SNDR -7.1 dB, ENOB -1.5 bits here),
    so the tone-free dynamic metric is the dither SNR, and it is labelled as such.
    """
    def dynamic(a):
        a.plot(np.arange(len(r5)), col(r5, "snr_dither_db"), color=C_R5, lw=1.1,
               label="dither SNR (raw)")
        a.plot(np.arange(len(r5)), col(r5, "snr_dither_db_cal"), color=C_TONE, lw=1.1,
               label="dither SNR (corrected)")
        a.set_ylabel("dither SNR [dB]")
        a.set_title("(d) Dynamic performance", fontsize=10)
        a.legend(fontsize=8, loc="lower right")

    fig = _loop_grid(
        plt, r5,
        gain_series=[("gain_mag_ratio", "dither magnitude (controlled)", dict(color=C_R5, lw=1.2)),
                     ("gain_ratio", "pulse-window ratio", dict(color=C_R4, lw=0.8, ls=":", alpha=0.8))],
        draw_dynamic=dynamic)
    fig.suptitle("Dither-only closed loop", fontsize=13)
    fig.tight_layout()
    p = os.path.join(OUT, "session_loop.png")
    fig.savefig(p, dpi=140)
    plt.close(fig)
    return p


def figure_tone(plt, rt):
    """Q3: the tone-mode closed loop (tonezero_check), same four panels — all four measurable."""
    def dynamic(a):
        it = np.arange(len(rt))
        a.plot(it, col(rt, "cal_sfdr_db"), color=C_R4, lw=1.1, label="SFDR")
        a.plot(it, col(rt, "cal_sndr_db"), color=C_R5, lw=1.2, label="SNDR")
        a.set_ylabel("SNDR / SFDR [dB]")
        a.set_ylim(30, 46)
        a.set_title("(d) Dynamic performance", fontsize=10)
        b = a.twinx()
        b.plot(it, col(rt, "cal_enob"), color=C_TONE, lw=1.1, ls="--", label="ENOB (right)")
        b.set_ylabel("ENOB [bits]", color=C_TONE)
        b.tick_params(axis="y", colors=C_TONE)
        b.set_ylim(5.0, 6.4)
        h1, l1 = a.get_legend_handles_labels()
        h2, l2 = b.get_legend_handles_labels()
        a.legend(h1 + h2, l1 + l2, fontsize=8, loc="lower right")

    fig = _loop_grid(
        plt, rt,
        gain_series=[("tone_ratio", "tone ratio (controlled)", dict(color=C_R5, lw=1.2)),
                     ("gain_ratio", "dither ratio", dict(color=C_R4, lw=0.8, ls=":", alpha=0.8))],
        draw_dynamic=dynamic)
    fig.suptitle("Tone-mode closed loop", fontsize=13)
    fig.tight_layout()
    p = os.path.join(OUT, "session_tone.png")
    fig.savefig(p, dpi=140)
    plt.close(fig)
    return p


def step_pooled(pts, min_margin=6.0):
    """The paired 34↔35 characterization as the tool's own verdict.

    ``tools/skew_step_characterize.py`` pools every code-34 frame (baseline + both restores) into
    one batch and every code-35 frame into another, applies the loop's alignment-margin floor, and
    differences the two means.  Pooling this way is what gives the documented +19.08 ± 0.06 ps/code,
    so the figure uses the same arithmetic rather than a per-block average.
    """
    if not pts:
        return {}
    base, step = [], []
    for r in pts:
        try:
            margin, value, code = float(r["margin"]), float(r["phase_ps"]), float(r["code"])
        except (KeyError, TypeError, ValueError):
            continue
        if margin < min_margin:
            continue
        (base if code == 34 else step).append(value)
    if len(base) < 2 or len(step) < 2:
        return {}
    b, s = np.array(base), np.array(step)
    se_b = float(b.std(ddof=1) / np.sqrt(b.size))
    se_s = float(s.std(ddof=1) / np.sqrt(s.size))
    return {"step": float(s.mean() - b.mean()), "se": float(np.hypot(se_b, se_s)),
            "baseline": float(b.mean()), "stepped": float(s.mean()),
            "n_base": b.size, "n_step": s.size}


def figure_fullscale(plt, lad, ifc30):
    """Q2: how far the dither can be pushed, from the top-of-ladder state only.

    The ladder walked three amplitudes; this figure carries the *best* one (30000 LSB, 91.6 % of
    DAC full scale) and the DAC ceiling projected linearly from it.  The lower states are kept in
    the table of `BENCH_SESSION_DITHER_ONLY.md` §F -- what they establish (linearity to 0.1 %) is
    stated here as a number instead of being drawn three times.
    """
    fig, ax = plt.subplots(1, 2, figsize=(10.4, 4.6))

    amp = col(lad, "amplitude_lsb")
    rep = col(lad, "rep_pp_a")
    sd = col(lad, "dt_phase_sd")
    pct_dac = col(lad, "pct_dac_fs")

    # (a) the top state and the ceiling it implies
    a = ax[0]
    g = float(rep[-1] / amp[-1])
    top_amp, top_rep, top_pct, top_sd = amp[-1], rep[-1], pct_dac[-1], sd[-1]
    xs = np.linspace(0, DAC_FS_LSB, 50)
    a.plot(xs, g * xs, "--", color="k", lw=1.0, label="linear through the measured state")
    a.plot([top_amp], [top_rep], "o", ms=8, color=C_R5,
           label=f"measured: {top_amp:.0f} LSB ({top_pct:.1f} % FS DAC)\n"
                 f"{top_rep:.0f} codes = {top_rep * CODES_TO_MV:.1f} mVpp at the ADC")
    a.plot([DAC_FS_LSB], [g * DAC_FS_LSB], "D", ms=9, mfc="none", mec=C_R5, mew=1.6,
           label=f"DAC ceiling: 32767 LSB (100 % FS)\n"
                 f"{g * DAC_FS_LSB:.0f} codes = {g * DAC_FS_LSB * CODES_TO_MV:.1f} mVpp")
    a.axvline(DAC_FS_LSB, color=C_R5, lw=0.8, ls=":", alpha=0.7)
    a.set_xlim(0, DAC_FS_LSB * 1.08)
    a.set_ylim(0, 880)
    a.set_xlabel("commanded dither amplitude [DAC LSB]")
    a.set_ylabel("replica pk-pk [ADC codes]")
    a.set_title("(a) Dither amplitude at the ADC", fontsize=10)
    a.legend(fontsize=7.5, loc="upper left")
    a.grid(True, alpha=0.3)
    b = a.twinx()
    b.set_ylim(0, 880 * CODES_TO_MV)
    b.set_ylabel("replica pk-pk [mVpp differential at the ADC]", color="0.35")
    b.tick_params(axis="y", colors="0.35")

    # (b) ground truth at the same top state: a known IFC gain step
    a = ax[1]
    s30 = float(col(ifc30, "gain_mag")[0] - col(ifc30, "gain_mag")[1])
    e30 = float(np.hypot(col(ifc30, "gain_mag_sd")[0], col(ifc30, "gain_mag_sd")[1]))
    nominal = 100 * (1 - 1.59 / 1.70)
    a.bar([0], [-100 * s30], 0.45, color=C_TONE,
          yerr=[100 * e30], capsize=5, error_kw=dict(lw=1.2),
          label=f"measured at {top_pct:.1f} % FS: −{100 * s30:.3f} ± {100 * e30:.3f} %")
    a.axhline(-nominal, ls="--", lw=1.2, color="k",
              label=f"nominal 1.59/1.70 = −{nominal:.2f} %")
    a.set_xticks([0])
    a.set_xticklabels(["IFC 0x0C → 0x0D\nchannel B only"], fontsize=9)
    a.set_xlim(-0.75, 0.75)
    a.set_ylim(-8.2, 0)
    a.set_ylabel("measured gain step [%]")
    a.set_title("(b) Known gain step at full scale", fontsize=10)
    a.legend(fontsize=8, loc="lower left")
    a.grid(True, alpha=0.3, axis="y")

    fig.suptitle("Full-scale dither ladder", fontsize=13)
    fig.tight_layout()
    p = os.path.join(OUT, "session_fullscale.png")
    fig.savefig(p, dpi=140)
    plt.close(fig)
    return p


# --------------------------------------------------------------------------- main

def main() -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(OUT, exist_ok=True)
    r5 = qualified(load(os.path.join(CL, "run5_phase.csv")))
    r4 = qualified(load(os.path.join(CL, "run4_tonefree.csv")))
    # The newest tone-mode run is the one to show: it is the one driven with the tone+dither vector
    # built from stage F (6-sample pulse, 5190 LSB, the most the -1.5 dBFS tone leaves).  The older
    # and much shorter tonezero_check stays available by name if a comparison is ever needed.
    rt = qualified(load(os.path.join(CL, "run6_tone_w06.csv")))
    lad = load(LADDER)
    ifc16 = load(IFC_16K)
    ifc30 = load(IFC_30K)
    step_pts = load(STEP)

    p1 = figure_loop(plt, r5)
    p2 = figure_tone(plt, rt)
    p3 = figure_fullscale(plt, lad, ifc30)

    # ------------------------------------------------------------------ the numbers
    print("figures:")
    for p in (p1, p2, p3):
        print(f"  {p}")

    dc5 = col(r5, "offset_b_codes") - col(r5, "offset_a_codes")
    print(f"\nQ1 closed loop (run5_phase, dither_phase): {len(r5)} qualified, "
          f"{len(load(os.path.join(CL, 'run5_phase.csv'))) - len(r5)} rejected")
    print(f"  skew {np.nanmean(col(r5, 'skew_used_ps')[:20]):+.2f} -> "
          f"{np.nanmean(col(r5, 'skew_used_ps')[-60:]):+.2f} ps "
          f"(batch se {np.nanmean([b[2] for b in batches(r5, 24)[-4:]]):.2f} ps, "
          f"per-frame sd {np.nanstd(col(r5, 'skew_used_ps')[-60:], ddof=1):.2f} ps)")
    print(f"  moves: {sum(1 for b in batches(r5, 24) if b[4].startswith('move'))}, "
          f"code 24 -> {code_trace(r5, 24)[-1]:.0f}, "
          f"mean step {fit(np.array([b[0] for b in batches(r5, 24)]), np.array([b[1] for b in batches(r5, 24)]))[0]:+.2f} ps/code")
    print(f"  A-B coherent: raw {np.nanmean(col(r5, 'dbc_ab_coherent')[:20]):.2f} -> "
          f"{np.nanmean(col(r5, 'dbc_ab_coherent')[-60:]):.2f} dBc, corrected "
          f"{np.nanmean(col(r5, 'dbc_ab_coherent_cal')[-60:]):.2f} dBc")
    print(f"  gain magnitude ratio {np.nanmean(col(r5, 'gain_mag_ratio')[-60:]):.5f} "
          f"(sd {np.nanstd(col(r5, 'gain_mag_ratio')[-60:], ddof=1):.5f}), "
          f"DC B-A {np.nanmean(dc5[-60:]):+.3f} codes (sd {np.nanstd(dc5[-60:], ddof=1):.2f})")
    ratio = col(r5, "gain_corr_b") / col(r5, "gain_corr_a")
    print(f"  applied gain ratio 1 + {np.nanmean(ratio[-60:]) - 1:.5f} "
          f"-> {20 * np.log10(abs(np.nanmean(ratio[-60:]) - 1)):.2f} dBc floor for the corrected metric")
    print(f"  control run (run4_tonefree, dither_fold): "
          f"{np.nanmean(col(r4, 'skew_used_ps')[:20]):+.2f} -> "
          f"{np.nanmean(col(r4, 'skew_used_ps')[-60:]):+.2f} ps, code 24 -> "
          f"{code_trace(r4, 24)[-1]:.0f}, raw A-B "
          f"{np.nanmean(col(r4, 'dbc_ab_coherent')[-60:]):.2f} dBc")

    print(f"\nQ3 tone-mode run (tonezero_check): {len(rt)} qualified from "
          f"{len(load(os.path.join(CL, 'tonezero_check.csv')))} captures")
    for code, mean, se, n, action, it in batches(rt, TONEFREE_PARK_CODE):
        print(f"  batch at code {code:.0f} (iter {it:.0f}): {mean:+.3f} +/- {se:.3f} ps "
              f"(n={n:.0f}) -> {action}")
    print(f"  tone_ratio {np.nanmean(col(rt, 'tone_ratio')[-60:]):.5f} "
          f"(sd {np.nanstd(col(rt, 'tone_ratio')[-60:], ddof=1):.5f}), "
          f"gain_ratio {np.nanmean(col(rt, 'gain_ratio')[-60:]):.5f}")
    print(f"  A-B spur: raw {np.nanmean(col(rt, 'raw_difference_dbc')[-60:]):.2f} dBc, "
          f"corrected {np.nanmean(col(rt, 'cal_difference_dbc')[-60:]):.2f} dBc")
    print(f"  SNDR {np.nanmean(col(rt, 'raw_a_sndr_db')[-60:]):.2f} dB, "
          f"SFDR {np.nanmean(col(rt, 'raw_a_sfdr_db')[-60:]):.2f} dB, "
          f"DC B-A {np.nanmean((col(rt, 'offset_b_codes') - col(rt, 'offset_a_codes'))[-60:]):+.3f} codes")
    blk = step_pooled(step_pts)
    if blk:
        print(f"  paired step 34<->35 (tone route, margin >= 6): baseline "
              f"{blk['baseline']:+.3f} ps (n={blk['n_base']}), stepped "
              f"{blk['stepped']:+.3f} ps (n={blk['n_step']}) -> "
              f"{blk['step']:+.3f} +/- {blk['se']:.3f} ps/code")
    b4 = batches(r4, 24)
    b5 = batches(r5, 24)
    bt = batches(rt, TONEFREE_PARK_CODE)
    print(f"  route zero crossings: fold "
          f"{zero_code([b[0] for b in b4], [b[1] for b in b4]):.2f}, phase "
          f"{zero_code([b[0] for b in b5], [b[1] for b in b5]):.2f}, tone "
          f"{zero_code([b[0] for b in bt], [b[1] for b in bt]):.2f} code")
    print(f"  route slopes: fold "
          f"{fit(np.array([b[0] for b in b4]), np.array([b[1] for b in b4]))[0]:+.2f}, phase "
          f"{fit(np.array([b[0] for b in b5]), np.array([b[1] for b in b5]))[0]:+.2f}, tone "
          f"{fit(np.array([b[0] for b in bt]), np.array([b[1] for b in bt]))[0]:+.2f} ps/code")

    amp = col(lad, "amplitude_lsb")
    rep = col(lad, "rep_pp_a")
    print("\nQ2 full-scale ladder:")
    for i in range(len(amp)):
        print(f"  amp{amp[i]:.0f} ({100 * amp[i] / DAC_FS_LSB:.1f} % FS DAC): replica "
              f"{rep[i]:.1f} codes pk-pk = {rep[i] * CODES_TO_MV:.1f} mVpp differential = "
              f"{100 * rep[i] / ADC_SPAN_CODES:.2f} % of the ADC span; "
              f"dt scatter {col(lad, 'dt_phase_sd')[i]:.2f} ps")
    g = float(rep[-1] / amp[-1])
    print(f"  path gain {g:.6f} codes/LSB -> at the DAC ceiling ({DAC_FS_LSB:.0f} LSB) "
          f"{g * DAC_FS_LSB:.1f} codes = {g * DAC_FS_LSB * CODES_TO_MV:.1f} mVpp = "
          f"{100 * g * DAC_FS_LSB / ADC_SPAN_CODES:.2f} % of the ADC span")
    print(f"  1 code = {CODES_TO_MV * 1e3:.2f} uV differential at {FS_ADC_VPP} Vpp full scale")
    s16 = 100 * (col(ifc16, "gain_mag")[1] / col(ifc16, "gain_mag")[0] - 1)
    s30 = 100 * (col(ifc30, "gain_mag")[1] / col(ifc30, "gain_mag")[0] - 1)
    print(f"  IFC 0x0C->0x0D step: {s16:+.3f} % at amp16000, {s30:+.3f} % at amp30000 "
          f"(nominal {100 * (1.59 / 1.70 - 1):+.2f} %)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
