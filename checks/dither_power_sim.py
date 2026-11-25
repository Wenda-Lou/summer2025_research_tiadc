"""Simulated dither-power sweep: how small can the dither be and still calibrate?

For each dither level the joint three-axis matrix (8 corners + centre: offset
+/-32 codes, gain_b 0.78/1.17, skew +/-187 ps, tone -6 dBFS through the model
combiner) runs on the production calibration_loop_c code, then every corner is
graded with the bench rule: HOLD |mean| + 1.96 SE <= tolerance, >= 80 % HOLD
yield, per axis.

    python checks/dither_power_sim.py                      # default ladder, noise 1 code
    python checks/dither_power_sim.py --noise 2            # noisier ADC model
    python checks/dither_power_sim.py --levels 2400,1200,600,300

Levels are the model dither peak in DAC LSB; the ADC-domain peak is x0.25
(model analog scale), printed alongside. The model's DAC->ADC scale is an
assumption, so transfer the result to the bench as dither peak / noise, using
the ADC-side dither amplitude measured at each bench level, not as a DAC code.
Outputs: calibration_out/dither_power_sim_<timestamp>/ (per-level traces,
sweep_summary.csv, dither_power_sweep.png).
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_joint_sim import CSV_KEYS, ROOT, SIM, TRAIN, converge_frame, load  # noqa: E402

ADC_PER_DAC = 8191.0 / 32767.0            # lt_bench_t.adc_scale
TOL_CSV = {"offset": 0.5, "gain": 1000.0, "skew": 10.0}   # CSV units: codes, ppm, ps
UNITS = {"offset": "code", "gain": "ppm", "skew": "ps"}
DEFAULT_LEVELS = (9600, 4800, 2400, 1200, 600, 300, 150, 75)
COLORS = {"offset": "#257de0", "gain": "#f47230", "skew": "#00b894"}   # validated slots 1-3
MARKERS = {"offset": "o", "gain": "s", "skew": "^"}
INK, MUTED, GRID = "#1f1f1f", "#6b6b6b", "#e6e6e6"


def run_level(exe: Path, dac: float, noise: float, out: Path) -> Path:
    out.mkdir(parents=True, exist_ok=True)
    subprocess.run([str(exe), "--joint-dither", f"{dac:g}", "--joint-noise", f"{noise:g}",
                    "--output-dir", str(out)], check=True, stdout=subprocess.DEVNULL)
    return out / "joint_trace.csv"


def grade_corner(rows):
    """Bench rule per axis on the independent HOLD; truth reported alongside."""
    hold = [r for r in rows if r["phase"] == "hold"]
    ok = [r for r in hold if r["rejected"] == 0]
    out = {"hold_yield": len(ok) / len(hold) if hold else 0.0,
           "rejected_train": sum(r["rejected"] != 0 for r in rows if r["phase"] == "train")}
    for axis, (truth_key, est_key) in CSV_KEYS.items():
        values = [r[est_key] for r in ok if math.isfinite(r[est_key])]
        n = len(values)
        mean = sum(values) / n if n else math.nan
        sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else math.nan
        se = sd / math.sqrt(n) if n > 1 else math.nan
        bound = abs(mean) + 1.96 * se if n > 1 else math.inf
        out[axis] = {
            "mean": mean, "bound": bound,
            "pass": n >= 2 and out["hold_yield"] >= 0.8 and bound <= TOL_CSV[axis],
            "truth": hold[-1][truth_key] if hold else math.nan,
            "converge": converge_frame(rows, truth_key, TOL_CSV[axis]),
        }
    out["pass"] = all(out[a]["pass"] for a in CSV_KEYS)
    return out


def summarise(level, noise, corners):
    graded = [grade_corner(rows) for rows in corners.values()]
    n = len(graded)
    row = {"dither_dac": level, "dither_adc_peak": level * ADC_PER_DAC, "noise_rms": noise,
           "peak_over_noise": level * ADC_PER_DAC / noise if noise > 0 else math.inf,
           "corners": n, "all_pass": sum(g["pass"] for g in graded),
           "min_hold_yield": min(g["hold_yield"] for g in graded),
           "train_rejects": sum(g["rejected_train"] for g in graded)}
    for a in CSV_KEYS:
        row[f"{a}_pass"] = sum(g[a]["pass"] for g in graded)
        row[f"{a}_worst_converge"] = max(g[a]["converge"] for g in graded)
        row[f"{a}_worst_bound"] = max(g[a]["bound"] for g in graded)
        row[f"{a}_worst_truth"] = max(abs(g[a]["truth"]) for g in graded)
    return row


def threshold(summary):
    """Smallest level such that it and every larger level pass all corners."""
    best = None
    for row in sorted(summary, key=lambda r: -r["dither_dac"]):
        if row["all_pass"] != row["corners"]:
            break
        best = row
    return best


def plot(summary, noise, path, plt):
    import matplotlib.ticker as ticker
    xs = [r["dither_adc_peak"] for r in summary]
    fig, axs = plt.subplots(3, 1, figsize=(9, 9.5), sharex=True)
    fig.suptitle(f"Simulated dither-power sweep (joint 3-axis, 9 corners, ADC noise {noise:g} code RMS)",
                 fontsize=11, color=INK, x=0.07, ha="left")
    panels = (
        ("Corners passing HOLD (of 9)", lambda r, a: r[f"{a}_pass"], None),
        ("Worst convergence (TRAIN frame)", lambda r, a: r[f"{a}_worst_converge"], TRAIN),
        ("Worst HOLD bound / tolerance", lambda r, a: r[f"{a}_worst_bound"] / TOL_CSV[a], 1.0),
    )
    for ax, (title, value, ref) in zip(axs, panels):
        for a in CSV_KEYS:
            ys = [value(r, a) for r in summary]
            ys = [y if math.isfinite(y) else float("nan") for y in ys]
            ax.plot(xs, ys, color=COLORS[a], lw=2, marker=MARKERS[a], ms=7,
                    markeredgecolor="white", markeredgewidth=1.5, label=a)
        if ref is not None:
            ax.axhline(ref, color=INK, lw=0.8, ls="--")
        ax.set_ylabel(title, color=INK, fontsize=9)
        ax.grid(True, color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.tick_params(colors=MUTED, labelsize=8)
    axs[0].set_ylim(-0.5, 9.5)
    axs[0].legend(loc="lower right", fontsize=8, frameon=False)
    axs[1].text(0.01, TRAIN, "TRAIN budget (600) = did not settle", va="bottom", fontsize=8, color=INK,
                transform=axs[1].get_yaxis_transform())
    axs[2].set_yscale("log")
    axs[2].text(0.01, 1.0, "tolerance (below = PASS)", va="bottom", fontsize=8, color=INK,
                transform=axs[2].get_yaxis_transform())
    best = threshold(summary)
    for ax in axs:
        ax.set_xscale("log", base=2)
        if best:
            ax.axvline(best["dither_adc_peak"], color=MUTED, lw=1, ls=":")
    if best:
        axs[0].text(best["dither_adc_peak"], 9.4, f" smallest all-pass: {best['dither_adc_peak']:.0f} codes peak",
                    fontsize=8, color=INK, va="top")
    axs[-1].xaxis.set_major_formatter(ticker.FuncFormatter(lambda v, _: f"{v:g}"))
    axs[-1].set_xlabel("dither peak at ADC (codes, model)  —  each step is -6 dB", color=INK, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(path, dpi=150)
    return fig


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--levels", help="comma-separated dither peaks in model DAC LSB (default 9600..75, -6 dB steps)")
    ap.add_argument("--noise", type=float, default=1.0, help="ADC noise RMS in codes (default 1)")
    ap.add_argument("--out", type=Path, help="output folder (default calibration_out/dither_power_sim_<timestamp>)")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args(argv)
    levels = [float(v) for v in args.levels.split(",")] if args.levels else list(DEFAULT_LEVELS)
    if not levels or any(v <= 0 for v in levels) or not args.noise >= 0:
        raise SystemExit("levels must be positive and noise >= 0")

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = args.out or ROOT / "calibration_out" / time.strftime("dither_power_sim_%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    build = SIM / "build"
    for cmd in (["cmake", "-S", str(SIM), "-B", str(build)], ["cmake", "--build", str(build)]):
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    exe = next(p for p in (build / "adc_cal_sim.exe", build / "adc_cal_sim") if p.exists())

    summary = []
    for level in sorted(levels, reverse=True):
        print(f"level {level:g} DAC LSB ({level * ADC_PER_DAC:.0f} codes peak) ...", flush=True)
        trace = run_level(exe, level, args.noise, out / f"level_{level:g}")
        summary.append(summarise(level, args.noise, load(trace)))

    with (out / "sweep_summary.csv").open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)

    print(f"\n{'DAC':>6} {'ADC pk':>7} {'pk/noise':>8} | pass all  off gain skew | worst converge off/gain/skew "
          f"| worst HOLD bound off(code) gain(ppm) skew(ps)")
    for r in summary:
        conv = [r[f"{a}_worst_converge"] for a in CSV_KEYS]
        print(f"{r['dither_dac']:>6g} {r['dither_adc_peak']:>7.0f} {r['peak_over_noise']:>8.0f} | "
              f"{r['all_pass']:>4}/{r['corners']} {r['offset_pass']:>4} {r['gain_pass']:>4} {r['skew_pass']:>4} | "
              f"{conv[0]:>5} {conv[1]:>5} {conv[2]:>5}            | "
              f"{r['offset_worst_bound']:>8.3f} {r['gain_worst_bound']:>9.0f} {r['skew_worst_bound']:>8.2f}")
    best = threshold(summary)
    print("\nSmallest level where it and every larger level pass all 9 corners:",
          f"{best['dither_dac']:g} DAC LSB = {best['dither_adc_peak']:.0f} codes peak "
          f"({best['peak_over_noise']:.0f}x noise RMS)" if best else "none")
    figure = out / "dither_power_sweep.png"
    plot(summary, args.noise, figure, plt)
    print(f"figure: {figure}\nsummary: {out / 'sweep_summary.csv'}")
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
