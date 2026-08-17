from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import median, quantiles

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
RUN_DIR = (
    ROOT
    / "test_platform"
    / "thesis_v3_500mhz_appl"
    / "adc_data"
    / "calibration_exports"
    / "calibration_run_20260816_210453"
)
SOURCE = RUN_DIR / "calibration_performance.csv"
OUTPUT = Path(__file__).with_name("stage5_performance_across_valid_captures.png")

NAVY = "#16324F"
TEXT = "#34495E"
MUTED = "#6B7C8F"
GREEN = "#16845B"
GREEN_DARK = "#145F47"
GREEN_LIGHT = "#A9D8C8"
CARD_FILL = "#F8FAFC"
CARD_EDGE = "#DDE5EC"
RANGE = "#A8B4BF"


@dataclass(frozen=True)
class MetricSummary:
    count: int
    median: float
    q1: float
    q3: float
    minimum: float
    maximum: float


def find_column(headers: list[str], candidates: tuple[str, ...], role: str) -> str:
    by_lower = {header.lower(): header for header in headers}
    for candidate in candidates:
        if candidate.lower() in by_lower:
            return by_lower[candidate.lower()]
    raise KeyError(f"Could not infer {role} column from headers")


def is_truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "valid", "accepted", "pass"}


def read_valid_metrics() -> tuple[list[float], list[float]]:
    with SOURCE.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        headers = list(reader.fieldnames or [])

    if not rows or not headers:
        raise ValueError("Performance CSV is empty")

    stage_column = next((header for header in headers if header.lower() == "stage"), None)
    if stage_column:
        rows = [row for row in rows if "performance" in row[stage_column].strip().lower()]

    validity_columns = [
        header
        for header in headers
        if header.lower() in {"valid", "is_valid", "accepted", "is_accepted"}
    ]
    if validity_columns:
        rows = [row for row in rows if all(is_truthy(row[column]) for column in validity_columns)]

    sndr_column = find_column(
        headers,
        (
            "cal_parallel_avg_sndr_db",
            "parallel_avg_sndr_db",
            "calibrated_parallel_avg_sndr_db",
            "cal_sndr_db",
            "sndr_db",
        ),
        "parallel-average SNDR",
    )
    enob_column = find_column(
        headers,
        (
            "cal_parallel_avg_enob",
            "parallel_avg_enob",
            "calibrated_parallel_avg_enob",
            "cal_enob",
            "enob",
        ),
        "parallel-average ENOB",
    )

    sndr_values: list[float] = []
    enob_values: list[float] = []
    for row in rows:
        try:
            sndr = float(row[sndr_column])
            enob = float(row[enob_column])
        except (TypeError, ValueError):
            continue
        if math.isfinite(sndr) and math.isfinite(enob):
            sndr_values.append(sndr)
            enob_values.append(enob)

    if not sndr_values:
        raise ValueError("No valid finite Stage-5 SNDR/ENOB rows found")
    if len(sndr_values) != 30:
        raise ValueError(f"Expected 30 valid Stage-5 captures, found {len(sndr_values)}")
    return sndr_values, enob_values


def summarize(values: list[float]) -> MetricSummary:
    q1, _, q3 = quantiles(values, n=4, method="inclusive")
    return MetricSummary(
        count=len(values),
        median=median(values),
        q1=q1,
        q3=q3,
        minimum=min(values),
        maximum=max(values),
    )


def add_metric_card(
    ax: plt.Axes,
    heading: str,
    summary: MetricSummary,
    unit: str,
) -> None:
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    card = FancyBboxPatch(
        (0.01, 0.02),
        0.98,
        0.95,
        boxstyle="round,pad=0.012,rounding_size=0.025",
        facecolor=CARD_FILL,
        edgecolor=CARD_EDGE,
        linewidth=1.4,
        transform=ax.transAxes,
        clip_on=False,
    )
    ax.add_patch(card)

    ax.text(
        0.07,
        0.88,
        heading,
        transform=ax.transAxes,
        fontsize=16.5,
        fontweight="bold",
        color=NAVY,
        ha="left",
        va="center",
    )
    ax.text(
        0.07,
        0.79,
        "MEDIAN",
        transform=ax.transAxes,
        fontsize=11.5,
        fontweight="bold",
        color=MUTED,
        ha="left",
        va="center",
    )
    ax.text(
        0.50,
        0.61,
        f"{summary.median:.2f}",
        transform=ax.transAxes,
        fontsize=54,
        fontweight="bold",
        color=GREEN,
        ha="center",
        va="center",
    )
    ax.text(
        0.50,
        0.48,
        unit,
        transform=ax.transAxes,
        fontsize=17,
        fontweight="bold",
        color=TEXT,
        ha="center",
        va="center",
    )

    x_left, x_right = 0.13, 0.87
    range_span = summary.maximum - summary.minimum

    def x_position(value: float) -> float:
        if range_span == 0:
            return (x_left + x_right) / 2
        return x_left + (x_right - x_left) * (value - summary.minimum) / range_span

    y_range = 0.275
    ax.text(
        0.50,
        0.355,
        "CAPTURE-TO-CAPTURE STABILITY",
        transform=ax.transAxes,
        fontsize=10.7,
        fontweight="bold",
        color=MUTED,
        ha="center",
        va="center",
    )
    ax.plot(
        [x_left, x_right],
        [y_range, y_range],
        transform=ax.transAxes,
        color=RANGE,
        linewidth=3.0,
        solid_capstyle="round",
        zorder=2,
    )
    for x_cap in (x_left, x_right):
        ax.plot(
            [x_cap, x_cap],
            [y_range - 0.025, y_range + 0.025],
            transform=ax.transAxes,
            color=RANGE,
            linewidth=2.0,
            zorder=2,
        )
    ax.plot(
        [x_position(summary.q1), x_position(summary.q3)],
        [y_range, y_range],
        transform=ax.transAxes,
        color=GREEN_LIGHT,
        linewidth=13,
        solid_capstyle="round",
        zorder=3,
    )
    ax.scatter(
        [x_position(summary.median)],
        [y_range],
        transform=ax.transAxes,
        s=125,
        marker="D",
        color=GREEN_DARK,
        edgecolor="white",
        linewidth=1.5,
        zorder=4,
    )
    ax.text(
        x_left,
        0.205,
        f"{summary.minimum:.2f}",
        transform=ax.transAxes,
        fontsize=11,
        color=MUTED,
        ha="center",
        va="center",
    )
    ax.text(
        x_right,
        0.205,
        f"{summary.maximum:.2f}",
        transform=ax.transAxes,
        fontsize=11,
        color=MUTED,
        ha="center",
        va="center",
    )
    ax.text(
        0.50,
        0.115,
        f"IQR {summary.q1:.2f}–{summary.q3:.2f} {unit}",
        transform=ax.transAxes,
        fontsize=12.2,
        fontweight="bold",
        color=TEXT,
        ha="center",
        va="center",
    )


def main() -> None:
    sndr_values, enob_values = read_valid_metrics()
    sndr_summary = summarize(sndr_values)
    enob_summary = summarize(enob_values)

    plt.rcParams.update({"font.family": "DejaVu Sans"})
    fig, (sndr_ax, enob_ax) = plt.subplots(
        1,
        2,
        figsize=(16, 9),
        dpi=240,
        facecolor="white",
    )
    fig.subplots_adjust(left=0.095, right=0.95, bottom=0.15, top=0.73, wspace=0.075)

    fig.text(
        0.095,
        0.925,
        "Stage 5 — Final Performance Evaluation",
        fontsize=26,
        fontweight="bold",
        color=NAVY,
        ha="left",
        va="center",
    )
    fig.text(
        0.095,
        0.872,
        "Median performance across 30 valid post-calibration captures",
        fontsize=15.5,
        color=MUTED,
        ha="left",
        va="center",
    )
    fig.text(
        0.877,
        0.902,
        "30 / 30 VALID\nCAPTURES",
        fontsize=14,
        fontweight="bold",
        color="white",
        ha="center",
        va="center",
        linespacing=1.25,
        bbox={
            "boxstyle": "round,pad=0.72,rounding_size=0.18",
            "facecolor": GREEN,
            "edgecolor": "none",
        },
    )

    add_metric_card(sndr_ax, "PARALLEL-AVERAGE SNDR", sndr_summary, "dB")
    add_metric_card(enob_ax, "PARALLEL-AVERAGE ENOB", enob_summary, "bits")

    fig.text(
        0.50,
        0.072,
        "Thin line: min–max  •  light-green segment: interquartile range  •  diamond: median",
        fontsize=11.2,
        color=MUTED,
        ha="center",
        va="center",
    )

    fig.savefig(OUTPUT, dpi=240, facecolor="white", edgecolor="none")
    print(
        f"Saved {OUTPUT} | valid={sndr_summary.count}, "
        f"SNDR median={sndr_summary.median:.6f} dB "
        f"(IQR {sndr_summary.q1:.6f}–{sndr_summary.q3:.6f}), "
        f"ENOB median={enob_summary.median:.6f} bits "
        f"(IQR {enob_summary.q1:.6f}–{enob_summary.q3:.6f})"
    )


if __name__ == "__main__":
    main()
