from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FormatStrFormatter, MultipleLocator


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "test_platform"
    / "thesis_v3_500mhz_appl"
    / "adc_data"
    / "calibration_exports"
    / "calibration_run_20260816_210453"
    / "calibration_offset_iterations.csv"
)
OUTPUT = Path(__file__).with_name("offset_calibration_convergence.png")
VERIFICATION_X = 8.65
VERIFICATION_RESIDUAL = 0.59


def load_iterations() -> tuple[list[int], list[float], list[float], list[int], float]:
    with SOURCE.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    iterations = [int(row["iteration"]) for row in rows]
    residuals = [float(row["filtered_batch_residual"]) for row in rows]
    corrections = [float(row["correction_after"]) for row in rows]
    pass_counts = [int(row["pass_count"]) for row in rows]
    tolerance = float(rows[-1]["tolerance"])
    return iterations, residuals, corrections, pass_counts, tolerance


def main() -> None:
    iterations, residuals, corrections, pass_counts, tolerance = load_iterations()
    pass_indices = [i for i, count in enumerate(pass_counts) if count > 0]

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.titleweight": "bold",
            "axes.labelcolor": "#243447",
            "xtick.color": "#44546A",
            "ytick.color": "#44546A",
        }
    )

    fig, ax = plt.subplots(figsize=(16, 9), dpi=240, facecolor="white")
    fig.subplots_adjust(left=0.135, right=0.885, bottom=0.15, top=0.78)
    ax.set_facecolor("white")

    ax.axhspan(-tolerance, tolerance, color="#EAF6F1", alpha=0.92, zorder=0)
    ax.axhline(
        tolerance,
        color="#D55E00",
        linewidth=2.5,
        linestyle=(0, (8, 6)),
        zorder=1,
    )
    ax.axhline(
        -tolerance,
        color="#D55E00",
        linewidth=2.5,
        linestyle=(0, (8, 6)),
        zorder=1,
    )
    ax.axhline(0.0, color="#667788", linewidth=2.0, zorder=1)
    ax.text(
        8.98,
        tolerance + 0.06,
        "+1 code",
        fontsize=14,
        fontweight="bold",
        color="#B54B00",
        ha="right",
        va="bottom",
    )
    ax.text(
        8.98,
        -tolerance - 0.06,
        "−1 code",
        fontsize=14,
        fontweight="bold",
        color="#B54B00",
        ha="right",
        va="top",
    )
    ax.text(
        0.86,
        0.05,
        "Zero target",
        fontsize=13,
        fontweight="bold",
        color="#667788",
        ha="left",
        va="bottom",
    )

    ax.plot(
        iterations,
        residuals,
        color="#174A7E",
        linewidth=3.4,
        marker="o",
        markersize=10,
        markerfacecolor="#174A7E",
        markeredgecolor="white",
        markeredgewidth=1.8,
        zorder=4,
    )

    pass_x = [iterations[i] for i in pass_indices]
    pass_y = [residuals[i] for i in pass_indices]
    ax.scatter(
        pass_x,
        pass_y,
        s=245,
        color="#1B9E77",
        edgecolor="white",
        linewidth=2.4,
        zorder=6,
    )

    ax.plot(
        [6.92, 6.92, 8.08, 8.08],
        [1.08, 1.15, 1.15, 1.08],
        color="#16845B",
        linewidth=2.2,
        clip_on=False,
        zorder=5,
    )
    ax.text(
        7.5,
        1.19,
        "2 / 2 convergence passes",
        fontsize=15,
        fontweight="bold",
        color="#16845B",
        ha="center",
        va="bottom",
    )

    ax.plot(
        [iterations[-1], VERIFICATION_X],
        [residuals[-1], VERIFICATION_RESIDUAL],
        color="#7A8795",
        linewidth=2.0,
        linestyle=(0, (3, 4)),
        zorder=3,
    )
    ax.scatter(
        [VERIFICATION_X],
        [VERIFICATION_RESIDUAL],
        s=260,
        marker="D",
        color="#16845B",
        edgecolor="white",
        linewidth=2.4,
        zorder=7,
    )
    ax.annotate(
        "Final verify  ≈ +0.59 code",
        xy=(VERIFICATION_X, VERIFICATION_RESIDUAL),
        xytext=(8.25, 0.30),
        fontsize=13,
        fontweight="bold",
        color="#145F47",
        ha="right",
        va="center",
        arrowprops={"arrowstyle": "-", "color": "#6B7C8F", "lw": 1.5},
        bbox={
            "boxstyle": "round,pad=0.42",
            "facecolor": "white",
            "edgecolor": "#B8C4CF",
        },
        zorder=8,
    )

    correction_ax = ax.twinx()
    correction_ax.plot(
        iterations,
        corrections,
        color="#6F5AA8",
        linewidth=2.7,
        linestyle=(0, (6, 4)),
        marker="s",
        markersize=7,
        markerfacecolor="white",
        markeredgewidth=1.7,
        alpha=0.92,
        zorder=2,
    )
    correction_ax.set_ylim(0.0, 5.5)
    correction_ax.yaxis.set_major_locator(MultipleLocator(1.0))
    correction_ax.yaxis.set_major_formatter(FormatStrFormatter("%.0f"))
    correction_ax.tick_params(axis="y", labelsize=14, length=0, pad=9, colors="#6F5AA8")
    correction_ax.set_ylabel(
        "Applied offset correction (codes)",
        fontsize=17,
        fontweight="bold",
        color="#6F5AA8",
        labelpad=17,
    )
    correction_ax.spines["top"].set_visible(False)
    correction_ax.spines["right"].set_visible(False)
    correction_ax.spines["left"].set_visible(False)

    ax.set_xlim(0.68, 9.02)
    ax.set_ylim(-3.45, 1.42)
    tick_positions = iterations + [VERIFICATION_X]
    tick_labels = [str(value) for value in iterations] + ["Verify"]
    ax.set_xticks(tick_positions, tick_labels)
    ax.yaxis.set_major_locator(MultipleLocator(1.0))
    ax.yaxis.set_major_formatter(FormatStrFormatter("%+.0f"))
    ax.tick_params(axis="both", labelsize=15, length=0, pad=8)
    ax.set_xlabel("Iteration", fontsize=20, fontweight="bold", labelpad=16)
    ax.set_ylabel(
        "Filtered offset residual (ADC codes)",
        fontsize=18,
        fontweight="bold",
        labelpad=16,
    )

    ax.grid(axis="y", color="#D9E0E7", linewidth=1.2, alpha=0.74)
    ax.grid(axis="x", visible=False)
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)
    ax.spines["bottom"].set_color("#9AA8B5")
    ax.spines["bottom"].set_linewidth(1.2)

    legend_handles = [
        Line2D(
            [0],
            [0],
            color="#174A7E",
            linewidth=3.2,
            marker="o",
            markersize=8,
            label="Filtered residual",
        ),
        Line2D(
            [0],
            [0],
            color="#6F5AA8",
            linewidth=2.7,
            linestyle=(0, (6, 4)),
            marker="s",
            markersize=7,
            label="Applied correction",
        ),
    ]
    ax.legend(
        handles=legend_handles,
        loc="lower left",
        bbox_to_anchor=(0.012, 1.012),
        ncol=2,
        frameon=False,
        fontsize=13,
        handlelength=2.8,
        columnspacing=2.0,
        borderpad=0.55,
    )

    fig.text(
        0.10,
        0.925,
        "Offset Calibration — Convergence",
        fontsize=29,
        fontweight="bold",
        color="#16324F",
        ha="left",
        va="center",
    )
    fig.text(
        0.10,
        0.875,
        "Filtered residual settles inside the ±1-code tolerance band",
        fontsize=17,
        color="#5B6777",
        ha="left",
        va="center",
    )
    fig.text(
        0.84,
        0.902,
        "CONVERGED\n2 / 2 PASSES",
        fontsize=17,
        fontweight="bold",
        color="white",
        ha="center",
        va="center",
        linespacing=1.35,
        bbox={
            "boxstyle": "round,pad=0.75,rounding_size=0.18",
            "facecolor": "#16845B",
            "edgecolor": "none",
        },
    )

    fig.savefig(
        OUTPUT,
        dpi=240,
        facecolor="white",
        edgecolor="none",
        bbox_inches=None,
    )
    print(
        f"Saved {OUTPUT} ({len(iterations)} iterations, "
        f"final residual {residuals[-1]:+.6f}, "
        f"final correction {corrections[-1]:+.6f}, "
        f"passes {pass_counts[-1]}/{max(pass_counts)})"
    )


if __name__ == "__main__":
    main()
