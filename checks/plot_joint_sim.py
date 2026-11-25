"""Visualise the simulated joint three-axis calibration (offset + gain + skew at once).

One command builds calibration_sim, runs `adc_cal_sim --run-joint` (the production
calibration_loop_c code against the C bench model) and plots its per-frame trace:

    python checks/plot_joint_sim.py                 # build, run, plot corner 0
    python checks/plot_joint_sim.py --corner 5      # another injected corner (0..8)
    python checks/plot_joint_sim.py --no-run        # re-plot the last trace only
    python checks/plot_joint_sim.py --show          # also open the windows

Outputs go to calibration_out/joint_sim_<timestamp>/ (trace CSV, two PNGs, test_summary.txt).
`truth` is the sim-only residual of the calibrated pair; `estimate` is what the loop
itself measured on that frame. Grading uses the frozen HOLD, never TRAIN residuals.
Simulation only: uniform 19.1 ps/code, no JESD/DMA/analog effects.
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "calibration_sim"
TRAIN, FREEZE = 600, 100                      # must match test_joint_three_axis()
# (column, label, unit, tolerance, scale applied to the CSV value)
AXES = (("offset", "Offset B−A", "codes", 0.5, 1.0),
        ("gain", "Gain B/A − 1", "%", 0.1, 1e-4),     # CSV is ppm; 0.1 % = 1000 ppm
        ("skew", "Skew B−A", "ps", 10.0, 1.0))
CSV_KEYS = {"offset": ("true_offset", "est_offset"),
            "gain": ("true_gain_ppm", "est_gain_ppm"),
            "skew": ("true_skew_ps", "est_skew_ps")}
TRUTH, ESTIMATE, OTHER = "#257de0", "#f47230", "#b8b8b8"   # validated slots 1, 2; neutral
INK, MUTED, GRID = "#1f1f1f", "#6b6b6b", "#e6e6e6"


def build_and_run(out: Path) -> Path:
    build = SIM / "build"
    for cmd in (["cmake", "-S", str(SIM), "-B", str(build)], ["cmake", "--build", str(build)]):
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    exe = next(p for p in (build / "adc_cal_sim.exe", build / "adc_cal_sim") if p.exists())
    result = subprocess.run([str(exe), "--run-joint", "--output-dir", str(out)],
                            capture_output=True, text=True)
    print(result.stdout.splitlines()[1] if result.stdout else result.stderr)
    if result.returncode != 0:
        print("Simulation reported failures; see", out / "test_summary.txt")
    return out / "joint_trace.csv"


def load(path: Path):
    corners = {}
    with path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            corners.setdefault(int(row["corner"]), []).append(
                {k: (v if k == "phase" else float(v)) for k, v in row.items()})
    if not corners:
        raise SystemExit(f"{path} has no rows")
    return corners


def converge_frame(rows, column, tol_csv, window=20):
    """Last TRAIN frame where the 20-frame truth mean was out of tolerance, +1."""
    values = [r[column] for r in rows if r["phase"] == "train"]
    last_bad = window - 2
    for i in range(window - 1, len(values)):
        if abs(sum(values[i + 1 - window:i + 1]) / window) > tol_csv:
            last_bad = i
    return last_bad + 1


def hold_mean(rows, column):
    values = [r[column] for r in rows if r["phase"] == "hold" and math.isfinite(r[column])]
    return sum(values) / len(values) if values else math.nan


def title_of(rows):
    r = rows[0]
    return (f"offset {r['offset_inj']:+g} codes · gain_b {r['gain_b']:.3f} · "
            f"skew {r['skew_inj_ps']:+g} ps · B {'inverted' if r['inverted'] else 'direct'}")


def style(ax, plt):
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)


def phases(ax, last):
    ax.axvspan(TRAIN - FREEZE, TRAIN, color="#f3f3f3", lw=0, zorder=0)
    ax.axvline(TRAIN, color=MUTED, lw=0.8, ls="--")
    ax.set_xlim(0, last)


def plot_process(corner, rows, plt, path):
    import matplotlib.ticker  # noqa: F401  (integer code ticks)
    import matplotlib
    """One corner: truth vs loop estimate per axis, plus the clock-delay code."""
    frames = [r["frame"] for r in rows]
    fig, axs = plt.subplots(4, 1, figsize=(10, 9.5), sharex=True,
                            gridspec_kw={"height_ratios": [3, 3, 3, 1.6]})
    fig.suptitle(f"Joint calibration, corner {corner}: {title_of(rows)}", fontsize=11, color=INK, x=0.06, ha="left")
    for ax, (name, label, unit, tol, scale) in zip(axs, AXES):
        truth_key, est_key = CSV_KEYS[name]
        est = [(f, r[est_key] * scale) for f, r in zip(frames, rows) if math.isfinite(r[est_key])]
        ax.axhspan(-tol, tol, color="#e8f1fc", lw=0, zorder=0)
        ax.plot([f for f, _ in est], [v for _, v in est], ".", ms=2.5, color=ESTIMATE,
                alpha=0.55, label="loop estimate (per frame)")
        ax.plot(frames, [r[truth_key] * scale for r in rows], color=TRUTH, lw=2, label="truth (sim only)")
        ax.set_yscale("symlog", linthresh=tol, linscale=2.0)
        ax.set_ylabel(f"{label} ({unit})", color=INK, fontsize=9)
        c = converge_frame(rows, truth_key, tol / scale)
        ax.axvline(c, color=TRUTH, lw=0.8, ls=":")
        ax.text(c + 4, 0.93, f"settled @ frame {c}", transform=ax.get_xaxis_transform(),
                fontsize=8, color=INK, va="top")
        h = hold_mean(rows, est_key) * scale
        ax.text(TRAIN + 4, 0.04, f"HOLD mean {h:+.3g} {unit}\n(tol ±{tol:g})",
                transform=ax.get_xaxis_transform(), fontsize=8, color=INK, va="bottom")
        style(ax, plt)
        phases(ax, frames[-1])
    axs[0].legend(loc="upper right", fontsize=8, frameon=False)
    axs[0].text(TRAIN - FREEZE / 2, 1.02, "freeze tail", transform=axs[0].get_xaxis_transform(),
                ha="center", fontsize=8, color=MUTED)
    axs[0].text(TRAIN + 4, 1.02, "HOLD (frozen)", transform=axs[0].get_xaxis_transform(),
                fontsize=8, color=MUTED)
    code = axs[3]
    code.step(frames, [r["code"] for r in rows], where="post", color=INK, lw=1.5)
    code.set_ylabel("delay code", color=INK, fontsize=9)
    code.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True, nbins=4))
    style(code, plt)
    phases(code, frames[-1])
    rejected = [f for f, r in zip(frames, rows) if r["rejected"] != 0]
    if rejected:
        code.plot(rejected, [code.get_ylim()[1]] * len(rejected), "|", color=MUTED, ms=6,
                  label=f"rejected frames ({len(rejected)})")
        code.legend(loc="lower right", fontsize=8, frameon=False)
    code.set_xlabel("frame (TRAIN 0–599, HOLD 600–699)", color=INK, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=150)
    return fig


def plot_overview(corners, selected, plt, path):
    """All corners: |truth| per axis on a log scale, the selected one highlighted."""
    fig, axs = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    fig.suptitle(f"All {len(corners)} injected corners: |truth residual| vs frame "
                 f"(corner {selected} highlighted)", fontsize=11, color=INK, x=0.06, ha="left")
    for ax, (name, label, unit, tol, scale) in zip(axs, AXES):
        key = CSV_KEYS[name][0]
        floor = tol / 100.0
        for corner, rows in corners.items():
            if corner == selected:
                continue
            ax.plot([r["frame"] for r in rows], [max(abs(r[key] * scale), floor) for r in rows],
                    color=OTHER, lw=1, label="other corners" if ax.get_legend_handles_labels()[1] == [] else None)
        rows = corners[selected]
        ax.plot([r["frame"] for r in rows], [max(abs(r[key] * scale), floor) for r in rows],
                color=TRUTH, lw=2, label=f"corner {selected}")
        ax.axhline(tol, color=INK, lw=0.8, ls="--")
        ax.annotate(f"tolerance {tol:g} {unit}", (0, tol), xytext=(4, 3), textcoords="offset points",
                    fontsize=8, color=INK)
        ax.set_yscale("log")
        ax.set_ylim(bottom=floor * 0.8)
        ax.set_ylabel(f"|{label}| ({unit}, floor {floor:g})", color=INK, fontsize=9)
        style(ax, plt)
        phases(ax, rows[-1]["frame"])
    axs[0].legend(loc="upper right", fontsize=8, frameon=False)
    axs[-1].set_xlabel("frame (TRAIN 0–599, HOLD 600–699)", color=INK, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(path, dpi=150)
    return fig


def print_table(corners):
    print(f"\n{'corner':>6} {'offset':>7} {'gain_b':>7} {'skew_ps':>8} {'inv':>3} | settled frames "
          f"off/gain/skew | HOLD offset codes, gain %, skew ps")
    for corner, rows in corners.items():
        r = rows[0]
        settled = [converge_frame(rows, CSV_KEYS[n][0], tol / s) for n, _, _, tol, s in AXES]
        hold = [hold_mean(rows, CSV_KEYS[n][1]) * s for n, _, _, _, s in AXES]
        print(f"{corner:>6} {r['offset_inj']:>+7g} {r['gain_b']:>7.3f} {r['skew_inj_ps']:>+8g} "
              f"{int(r['inverted']):>3} | {settled[0]:>4} {settled[1]:>4} {settled[2]:>4}       "
              f"| {hold[0]:+8.3f} {hold[1]:+8.4f} {hold[2]:+7.2f}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--corner", type=int, default=0, help="corner to detail, 0..8 (8 = no mismatch)")
    ap.add_argument("--no-run", action="store_true", help="plot the newest existing trace; do not build/run")
    ap.add_argument("--trace", type=Path, help="explicit joint_trace.csv to plot (implies --no-run)")
    ap.add_argument("--out", type=Path, help="output folder (default calibration_out/joint_sim_<timestamp>)")
    ap.add_argument("--show", action="store_true", help="open the figures in windows")
    args = ap.parse_args(argv)

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if args.trace or args.no_run:
        trace = args.trace or max((ROOT / "calibration_out").glob("joint_sim_*/joint_trace.csv"),
                                  key=lambda p: p.stat().st_mtime, default=None)
        if trace is None or not trace.exists():
            raise SystemExit("no joint_trace.csv found; run without --no-run first")
        out = args.out or trace.parent
        out.mkdir(parents=True, exist_ok=True)
        if trace.parent != out:
            shutil.copy2(trace, out / "joint_trace.csv")
    else:
        out = args.out or ROOT / "calibration_out" / time.strftime("joint_sim_%Y%m%d_%H%M%S")
        out.mkdir(parents=True, exist_ok=True)
        trace = build_and_run(out)

    corners = load(trace)
    if args.corner not in corners:
        raise SystemExit(f"--corner must be one of {sorted(corners)}")
    print_table(corners)
    process = out / f"joint_process_corner{args.corner}.png"
    overview = out / "joint_overview.png"
    plot_process(args.corner, corners[args.corner], plt, process)
    plot_overview(corners, args.corner, plt, overview)
    print(f"\nfigures: {process}\n         {overview}")
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
