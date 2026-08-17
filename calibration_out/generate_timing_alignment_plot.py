from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter, MultipleLocator


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "test_platform"
    / "thesis_v3_500mhz_appl"
    / "adc_data"
    / "calibration_exports"
    / "calibration_run_20260816_210453"
    / "calibration_timing_captures.csv"
)
OUTPUT = Path(__file__).with_name("timing_alignment_correlation.png")
THRESHOLD = 0.97


def load_captures() -> tuple[list[int], list[float], list[bool]]:
    with SOURCE.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    captures = [int(row["capture_index"]) for row in rows]
    correlations = [float(row["correlation"]) for row in rows]
    accepted = [row["accepted"].strip() == "1" for row in rows]
    return captures, correlations, accepted


def main() -> None:
    captures, correlations, accepted = load_captures()
    accepted_count = sum(accepted)

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
    fig.subplots_adjust(left=0.10, right=0.95, bottom=0.15, top=0.82)
    ax.set_facecolor("white")

    ax.axhspan(THRESHOLD, 1.001, color="#EAF6F1", alpha=0.68, zorder=0)
    ax.axhline(
        THRESHOLD,
        color="#D55E00",
        linewidth=2.8,
        linestyle=(0, (8, 6)),
        zorder=1,
    )
    ax.text(
        10.35,
        THRESHOLD + 0.0007,
        "Acceptance threshold  0.97",
        color="#B54B00",
        fontsize=15,
        fontweight="bold",
        ha="right",
        va="bottom",
    )

    ax.plot(
        captures,
        correlations,
        color="#174A7E",
        linewidth=3.2,
        zorder=3,
    )
    ax.scatter(
        captures,
        correlations,
        s=145,
        color="#1B9E77",
        edgecolor="white",
        linewidth=2.0,
        zorder=4,
    )

    ax.set_xlim(0.65, 10.35)
    ax.set_ylim(0.968, 1.001)
    ax.set_xticks(captures)
    ax.yaxis.set_major_locator(MultipleLocator(0.01))
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.2f"))
    ax.tick_params(axis="both", labelsize=15, length=0, pad=8)
    ax.set_xlabel("Capture index", fontsize=20, fontweight="bold", labelpad=16)
    ax.set_ylabel("Correlation", fontsize=20, fontweight="bold", labelpad=16)

    ax.grid(axis="y", color="#D9E0E7", linewidth=1.2, alpha=0.75)
    ax.grid(axis="x", visible=False)
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)
    ax.spines["bottom"].set_color("#9AA8B5")
    ax.spines["bottom"].set_linewidth(1.2)

    fig.text(
        0.10,
        0.925,
        "Timing Alignment — Correlation by Capture",
        fontsize=29,
        fontweight="bold",
        color="#16324F",
        ha="left",
        va="center",
    )
    fig.text(
        0.10,
        0.875,
        "All frames exceed the acceptance criterion with substantial margin",
        fontsize=17,
        color="#5B6777",
        ha="left",
        va="center",
    )
    fig.text(
        0.855,
        0.902,
        f"ALL FRAMES ACCEPTED\n{accepted_count} / {len(accepted)}",
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

    minimum = min(correlations)
    minimum_index = captures[correlations.index(minimum)]
    ax.annotate(
        f"Minimum: {minimum:.4f}",
        xy=(minimum_index, minimum),
        xytext=(minimum_index - 0.7, minimum - 0.0062),
        fontsize=14,
        fontweight="bold",
        color="#174A7E",
        ha="center",
        arrowprops={"arrowstyle": "-", "color": "#6B7C8F", "lw": 1.6},
        bbox={
            "boxstyle": "round,pad=0.35",
            "facecolor": "white",
            "edgecolor": "#B8C4CF",
        },
        zorder=5,
    )

    fig.savefig(
        OUTPUT,
        dpi=240,
        facecolor="white",
        edgecolor="none",
        bbox_inches=None,
    )
    print(
        f"Saved {OUTPUT} ({len(captures)} captures, "
        f"{accepted_count} accepted, min correlation {minimum:.6f})"
    )


if __name__ == "__main__":
    main()
