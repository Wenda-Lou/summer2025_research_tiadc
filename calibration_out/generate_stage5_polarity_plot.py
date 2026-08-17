from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
RUN_DIR = (
    ROOT
    / "test_platform"
    / "thesis_v3_500mhz_appl"
    / "adc_data"
    / "calibration_exports"
    / "calibration_run_20260816_210453"
)
PERFORMANCE_SOURCE = RUN_DIR / "calibration_performance.csv"
OUTPUT = Path(__file__).with_name("stage5_polarity_correction_before_after.png")

NAVY = "#16324F"
TEXT = "#34495E"
MUTED = "#6B7C8F"
BEFORE = "#9AA8B5"
GREEN = "#16845B"
ARROW = "#5A9B83"
TRACK = "#D8E0E7"
CARD_FILL = "#FBFCFD"
CARD_EDGE = "#E0E6EB"
SUCCESS_FILL = "#EAF6F1"


@dataclass(frozen=True)
class Metric:
    title: str
    before: float
    after: float
    xlim: tuple[float, float]
    before_label: str
    after_label: str
    cue: str


def read_after_means() -> dict[str, float]:
    with PERFORMANCE_SOURCE.open(newline="", encoding="utf-8-sig") as handle:
        rows = [
            row
            for row in csv.DictReader(handle)
            if row["accepted"] == "1" and row["valid"] == "1"
        ]

    if len(rows) != 30:
        raise ValueError(f"Expected 30 valid Stage-5 captures, found {len(rows)}")

    columns = {
        "correlation": "cal_ab_correlation",
        "rmse": "cal_ab_rmse_codes",
        "sndr": "cal_parallel_avg_sndr_db",
        "enob": "cal_parallel_avg_enob",
    }
    return {
        name: fmean(float(row[column]) for row in rows)
        for name, column in columns.items()
    }


def add_card(ax: plt.Axes, metric: Metric) -> None:
    ax.add_patch(
        FancyBboxPatch(
            (-0.035, -0.08),
            1.07,
            1.27,
            transform=ax.transAxes,
            boxstyle="round,pad=0.018,rounding_size=0.035",
            facecolor=CARD_FILL,
            edgecolor=CARD_EDGE,
            linewidth=1.2,
            clip_on=False,
            zorder=-10,
        )
    )

    ax.set_xlim(*metric.xlim)
    ax.set_ylim(-0.75, 0.75)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    ax.text(
        0.0,
        1.14,
        metric.title,
        transform=ax.transAxes,
        fontsize=15,
        fontweight="bold",
        color=NAVY,
        ha="left",
        va="center",
    )
    ax.text(
        1.0,
        1.14,
        metric.cue,
        transform=ax.transAxes,
        fontsize=10.5,
        fontweight="bold",
        color=GREEN,
        ha="right",
        va="center",
    )

    scale_width = metric.xlim[1] - metric.xlim[0]
    track_start = metric.xlim[0] + 0.035 * scale_width
    track_end = metric.xlim[1] - 0.035 * scale_width
    ax.plot([track_start, track_end], [0, 0], color=TRACK, lw=3.0, zorder=0)
    ax.add_patch(
        FancyArrowPatch(
            (metric.before, 0),
            (metric.after, 0),
            arrowstyle="-|>",
            mutation_scale=22,
            linewidth=3.2,
            color=ARROW,
            shrinkA=9,
            shrinkB=11,
            zorder=2,
        )
    )
    ax.scatter(
        [metric.before],
        [0],
        s=205,
        color=BEFORE,
        edgecolor="white",
        linewidth=2.0,
        zorder=4,
    )
    ax.scatter(
        [metric.after],
        [0],
        s=255,
        color=GREEN,
        edgecolor="white",
        linewidth=2.2,
        zorder=5,
    )

    before_fraction = (metric.before - metric.xlim[0]) / scale_width
    after_fraction = (metric.after - metric.xlim[0]) / scale_width
    before_align = "right" if before_fraction > 0.72 else "left"
    after_align = "right" if after_fraction > 0.72 else "left"
    before_dx = -0.012 if before_align == "right" else 0.012
    after_dx = -0.012 if after_align == "right" else 0.012

    ax.text(
        before_fraction + before_dx,
        0.66,
        f"BEFORE\n{metric.before_label}",
        transform=ax.transAxes,
        fontsize=12.2,
        fontweight="bold",
        color="#718091",
        ha=before_align,
        va="bottom",
        linespacing=1.25,
    )
    ax.text(
        after_fraction + after_dx,
        0.34,
        f"AFTER\n{metric.after_label}",
        transform=ax.transAxes,
        fontsize=13.5,
        fontweight="bold",
        color=GREEN,
        ha=after_align,
        va="top",
        linespacing=1.25,
    )


def main() -> None:
    after = read_after_means()

    metrics = [
        Metric(
            title="A/B CORRELATION",
            before=-0.99995,
            after=after["correlation"],
            xlim=(-1.18, 1.18),
            before_label="\u22120.99995",
            after_label=f"+{after['correlation']:.5f}",
            cue="POLARITY SIGN RESTORED",
        ),
        Metric(
            title="A/B RMSE",
            before=768.0,
            after=after["rmse"],
            xlim=(-55.0, 840.0),
            before_label="768 codes",
            after_label=f"{after['rmse']:.1f} codes",
            cue="97.6% LOWER",
        ),
        Metric(
            title="PARALLEL-AVERAGE SNDR",
            before=3.02,
            after=after["sndr"],
            xlim=(-2.5, 43.0),
            before_label="3.02 dB",
            after_label=f"{after['sndr']:.1f} dB",
            cue=f"+{after['sndr'] - 3.02:.1f} dB",
        ),
        Metric(
            title="ENOB",
            before=0.21,
            after=after["enob"],
            xlim=(-0.45, 6.65),
            before_label="0.21 bits",
            after_label=f"\u2248{after['enob']:.1f} bits",
            cue=f"+{after['enob'] - 0.21:.1f} BITS",
        ),
    ]

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.labelcolor": TEXT,
            "xtick.color": TEXT,
            "ytick.color": TEXT,
        }
    )

    fig, axes = plt.subplots(2, 2, figsize=(16, 9), dpi=240, facecolor="white")
    fig.subplots_adjust(left=0.095, right=0.95, bottom=0.15, top=0.675, wspace=0.16, hspace=0.62)

    fig.text(
        0.095,
        0.925,
        "Stage 5 \u2014 Channel-Polarity Correction: Before vs After",
        fontsize=24,
        fontweight="bold",
        color=NAVY,
        ha="left",
        va="center",
    )
    fig.text(
        0.095,
        0.875,
        "Propagating the Stage-4 INVERTED relation prevents destructive channel cancellation",
        fontsize=15.5,
        color=MUTED,
        ha="left",
        va="center",
    )
    fig.text(
        0.865,
        0.902,
        "COMBINED OUTPUT\nPRESERVED",
        fontsize=14.5,
        fontweight="bold",
        color="white",
        ha="center",
        va="center",
        linespacing=1.30,
        bbox={
            "boxstyle": "round,pad=0.68,rounding_size=0.18",
            "facecolor": GREEN,
            "edgecolor": "none",
        },
    )
    fig.text(
        0.50,
        0.775,
        "Stage-4 polarity normalization applied before channel combining.",
        fontsize=13.5,
        fontweight="bold",
        color="#145F47",
        ha="center",
        va="center",
        bbox={
            "boxstyle": "round,pad=0.62,rounding_size=0.16",
            "facecolor": SUCCESS_FILL,
            "edgecolor": "#B9DCCF",
            "linewidth": 1.1,
        },
    )

    for ax, metric in zip(axes.flat, metrics):
        add_card(ax, metric)

    fig.text(
        0.095,
        0.065,
        "After values are means across 30 valid Stage-5 captures from the 2026-08-16 calibration run.",
        fontsize=11.2,
        color=MUTED,
        ha="left",
        va="center",
    )

    fig.savefig(OUTPUT, dpi=240, facecolor="white", edgecolor="none")
    print(
        f"Saved {OUTPUT} | after means: corr={after['correlation']:.6f}, "
        f"RMSE={after['rmse']:.3f}, SNDR={after['sndr']:.3f}, ENOB={after['enob']:.3f}"
    )


if __name__ == "__main__":
    main()
