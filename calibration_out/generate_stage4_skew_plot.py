from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path
from statistics import median

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator


ROOT = Path(__file__).resolve().parents[1]
RUN_DIR = (
    ROOT
    / "test_platform"
    / "thesis_v3_500mhz_appl"
    / "adc_data"
    / "calibration_exports"
    / "calibration_run_20260816_210453"
)
CAPTURE_SOURCE = RUN_DIR / "calibration_skew_captures.corrected.csv"
ITERATION_SOURCE = RUN_DIR / "calibration_skew_iterations.corrected.csv"
OUTPUT = Path(__file__).with_name("stage4_skew_characterization.png")

INITIAL_SKEW_PS = -60.04
FINAL_SKEW_PS = -5.96
INITIAL_REGISTER = 24
FINAL_REGISTER = 34

NAVY = "#16324F"
TEXT = "#34495E"
MUTED = "#667788"
GRID = "#D9E1E8"
BLUE = "#2878B5"
BASELINE = "#8EA6BA"
ORANGE = "#D55E00"
GREEN = "#16845B"
TOLERANCE_FILL = "#EAF6F1"


def read_characterization_batches() -> list[dict[str, object]]:
    groups: dict[int, list[dict[str, str]]] = defaultdict(list)
    with CAPTURE_SOURCE.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            phase = row["capture_phase"]
            if phase.startswith("actuator_characterization_") and row["primary_valid"] == "1":
                groups[int(row["capture_group_index"])].append(row)

    batches: list[dict[str, object]] = []
    for group_index in sorted(groups):
        rows = groups[group_index]
        phase = rows[0]["capture_phase"]
        register = int(rows[0]["delay_register_active"])
        skew_values = [float(row["measured_skew_ps"]) for row in rows]
        if phase.endswith("baseline"):
            kind = "baseline"
            probe_codes = 0
        elif phase.endswith("repeat"):
            kind = "repeat"
            probe_codes = register - INITIAL_REGISTER
        else:
            kind = "first"
            probe_codes = register - INITIAL_REGISTER
        batches.append(
            {
                "group": group_index,
                "kind": kind,
                "probe_codes": probe_codes,
                "register": register,
                "median_skew_ps": median(skew_values),
                "frame_count": len(skew_values),
            }
        )
    return batches


def read_controller_metadata() -> tuple[float, float, str]:
    with ITERATION_SOURCE.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))

    tolerance_ps = float(rows[0]["tolerance_ps"])
    resolution_ps_per_code = next(
        float(row["actuator_resolution_ps_per_step"])
        for row in rows
        if row.get("actuator_resolution_ps_per_step", "").strip()
    )
    progression = [int(rows[0]["delay_register_before"])]
    for row in rows:
        after = int(row["delay_register_after"])
        if after != progression[-1]:
            progression.append(after)
    progression_text = " \u2192 ".join(str(value) for value in progression)
    return tolerance_ps, resolution_ps_per_code, progression_text


def style_axis(ax: plt.Axes, tolerance_ps: float) -> None:
    ax.set_ylim(-75, 15)
    ax.axhspan(-tolerance_ps, tolerance_ps, color=TOLERANCE_FILL, zorder=0)
    ax.axhline(tolerance_ps, color=GREEN, lw=1.6, ls=(0, (6, 5)), zorder=1)
    ax.axhline(-tolerance_ps, color=GREEN, lw=1.6, ls=(0, (6, 5)), zorder=1)
    ax.axhline(0, color="#7C8996", lw=1.4, zorder=1)
    ax.yaxis.set_major_locator(MultipleLocator(15))
    ax.grid(axis="y", color=GRID, lw=1.0, alpha=0.80)
    ax.grid(axis="x", visible=False)
    ax.tick_params(axis="both", labelsize=12.5, length=0, pad=8)
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)
    ax.spines["bottom"].set_color("#9AA8B5")
    ax.spines["bottom"].set_linewidth(1.1)


def main() -> None:
    batches = read_characterization_batches()
    tolerance_ps, resolution_ps_per_code, progression = read_controller_metadata()

    expected_counts = {0: 8, 1: 2, 2: 2, 4: 2, 8: 1}
    actual_counts = {
        probe: sum(batch["probe_codes"] == probe for batch in batches)
        for probe in expected_counts
    }
    if actual_counts != expected_counts:
        raise ValueError(f"Unexpected recoverable characterization batches: {actual_counts}")
    if any(batch["frame_count"] != 10 for batch in batches):
        raise ValueError("Each characterization batch must contain 10 valid frames")

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.labelcolor": TEXT,
            "xtick.color": TEXT,
            "ytick.color": TEXT,
        }
    )

    fig, (char_ax, result_ax) = plt.subplots(
        1,
        2,
        figsize=(16, 9),
        dpi=240,
        facecolor="white",
        sharey=True,
        gridspec_kw={"width_ratios": (2.15, 1.0)},
    )
    fig.subplots_adjust(left=0.095, right=0.95, bottom=0.225, top=0.72, wspace=0.07)

    fig.text(
        0.095,
        0.925,
        "Stage 4 \u2014 Adaptive Skew Characterization and Closed-Loop Result",
        fontsize=19.5,
        fontweight="bold",
        color=NAVY,
        ha="left",
        va="center",
    )
    fig.text(
        0.095,
        0.870,
        "Adaptive probe evidence and authoritative UART endpoints",
        fontsize=14,
        color=MUTED,
        ha="left",
        va="center",
    )
    fig.text(
        0.885,
        0.912,
        f"WITHIN TOLERANCE\nFINAL  {FINAL_SKEW_PS:.2f} ps",
        fontsize=13.5,
        fontweight="bold",
        color="white",
        ha="center",
        va="center",
        linespacing=1.30,
        bbox={
            "boxstyle": "round,pad=0.60,rounding_size=0.18",
            "facecolor": GREEN,
            "edgecolor": "none",
        },
    )

    for ax in (char_ax, result_ax):
        style_axis(ax, tolerance_ps)

    char_ax.set_title(
        "ADAPTIVE ACTUATOR CHARACTERIZATION",
        loc="left",
        fontsize=14.5,
        fontweight="bold",
        color=NAVY,
        pad=34,
    )
    char_ax.text(
        0,
        1.025,
        "Characterization measurements \u2014 not controller convergence iterations",
        transform=char_ax.transAxes,
        fontsize=11.5,
        color=MUTED,
        ha="left",
        va="bottom",
    )

    categorical_x = {0: 0, 1: 1, 2: 2, 4: 3, 8: 4}
    baseline_batch_medians = [
        float(batch["median_skew_ps"])
        for batch in batches
        if batch["kind"] == "baseline"
    ]
    baseline_median = median(baseline_batch_medians)
    char_ax.scatter(
        0,
        baseline_median,
        s=155,
        marker="D",
        facecolor=BASELINE,
        edgecolor="white",
        linewidth=2.0,
        zorder=5,
    )
    for batch in batches:
        kind = str(batch["kind"])
        if kind == "baseline":
            continue
        base_x = categorical_x[int(batch["probe_codes"])]
        if kind == "first":
            marker, offset, face, edge, size = "o", -0.055, BLUE, "white", 120
        else:
            marker, offset, face, edge, size = "s", 0.055, "white", BLUE, 115
        char_ax.scatter(
            base_x + offset,
            float(batch["median_skew_ps"]),
            s=size,
            marker=marker,
            facecolor=face,
            edgecolor=edge,
            linewidth=1.8,
            zorder=4,
        )

    char_ax.set_xlim(-0.45, 4.50)
    char_ax.set_xticks(range(5), ["Fresh\nbaseline", "1", "2", "4", "8"])
    char_ax.set_xlabel(
        "Characterization setting / probe amplitude (codes)",
        fontsize=14,
        fontweight="bold",
        labelpad=14,
    )
    char_ax.set_ylabel("Measured skew (ps)", fontsize=14, fontweight="bold", labelpad=16)

    legend_handles = [
        Line2D(
            [], [], marker="D", ls="", markersize=8, markerfacecolor=BASELINE,
            markeredgecolor="white", label="Fresh-baseline median (8 batches)"
        ),
        Line2D(
            [], [], marker="o", ls="", markersize=9, markerfacecolor=BLUE,
            markeredgecolor="white", label="First probe"
        ),
        Line2D(
            [], [], marker="s", ls="", markersize=8, markerfacecolor="white",
            markeredgecolor=BLUE, markeredgewidth=1.7, label="Repeat probe"
        ),
    ]
    char_ax.legend(
        handles=legend_handles,
        loc="upper left",
        bbox_to_anchor=(0.012, 0.982),
        frameon=False,
        fontsize=9.8,
        ncol=3,
        columnspacing=1.3,
        handletextpad=0.55,
        borderaxespad=0,
    )
    char_ax.text(
        0.985,
        0.955,
        f"ADAPTIVE PROBE  1 \u2192 2 \u2192 4 \u2192 8 CODES\n"
        f"Measured resolution: {resolution_ps_per_code:.2f} ps/code",
        transform=char_ax.transAxes,
        fontsize=10.8,
        fontweight="bold",
        color="#145F47",
        ha="right",
        va="top",
        linespacing=1.30,
        bbox={
            "boxstyle": "round,pad=0.55,rounding_size=0.14",
            "facecolor": TOLERANCE_FILL,
            "edgecolor": "#B9DCCF",
            "linewidth": 1.1,
        },
    )

    result_ax.set_title(
        "FINAL CLOSED-LOOP RESULT",
        loc="left",
        fontsize=14.5,
        fontweight="bold",
        color=NAVY,
        pad=34,
    )
    result_ax.text(
        0,
        1.025,
        "Authoritative UART endpoints only",
        transform=result_ax.transAxes,
        fontsize=11.5,
        color=MUTED,
        ha="left",
        va="bottom",
    )
    result_ax.scatter(
        [0], [INITIAL_SKEW_PS], s=250, marker="D", color=ORANGE,
        edgecolor="white", linewidth=2.2, zorder=5
    )
    result_ax.scatter(
        [1], [FINAL_SKEW_PS], s=280, marker="D", color=GREEN,
        edgecolor="white", linewidth=2.2, zorder=6
    )
    result_ax.set_xlim(-0.48, 1.48)
    result_ax.set_xticks([0, 1], ["Initial", "Final"])
    result_ax.set_xlabel("Measured UART endpoint", fontsize=14, fontweight="bold", labelpad=14)
    result_ax.tick_params(axis="y", labelleft=False)

    result_ax.annotate(
        f"{INITIAL_SKEW_PS:.2f} ps\n@ reg {INITIAL_REGISTER}\nOUTSIDE TOLERANCE",
        xy=(0, INITIAL_SKEW_PS),
        xytext=(0.11, -52),
        fontsize=11.5,
        fontweight="bold",
        color="#8E3B00",
        ha="left",
        va="bottom",
        linespacing=1.25,
        arrowprops={"arrowstyle": "-", "color": "#9A6A4A", "lw": 1.2},
    )
    result_ax.annotate(
        f"{FINAL_SKEW_PS:.2f} ps\n@ reg {FINAL_REGISTER}\nINSIDE TOLERANCE",
        xy=(1, FINAL_SKEW_PS),
        xytext=(0.72, -19),
        fontsize=11.8,
        fontweight="bold",
        color="#145F47",
        ha="center",
        va="top",
        linespacing=1.22,
        arrowprops={"arrowstyle": "-", "color": "#5D907E", "lw": 1.2},
    )
    result_ax.text(
        0.50,
        0.895,
        f"REGISTER PROGRESSION\n{progression}",
        transform=result_ax.transAxes,
        fontsize=11.5,
        fontweight="bold",
        color=NAVY,
        ha="center",
        va="top",
        linespacing=1.30,
        bbox={
            "boxstyle": "round,pad=0.55,rounding_size=0.14",
            "facecolor": "#EEF3F8",
            "edgecolor": "#C6D3DE",
            "linewidth": 1.1,
        },
    )
    result_ax.text(
        1.44,
        tolerance_ps + 1.2,
        f"+{tolerance_ps:.2f}",
        fontsize=10.8,
        color="#145F47",
        ha="right",
        va="bottom",
    )
    result_ax.text(
        1.44,
        -tolerance_ps - 1.2,
        f"\u2212{tolerance_ps:.2f}",
        fontsize=10.8,
        color="#145F47",
        ha="right",
        va="top",
    )
    fig.text(
        0.095,
        0.055,
        "Characterization medians shown; missing controller post-update batches are not reconstructed.",
        fontsize=11.5,
        color=MUTED,
        ha="left",
        va="center",
    )

    fig.savefig(OUTPUT, dpi=240, facecolor="white", edgecolor="none")
    print(
        f"Saved {OUTPUT} with {len(batches)} recovered characterization medians; "
        f"tolerance=\u00b1{tolerance_ps:.2f} ps, resolution={resolution_ps_per_code:.2f} ps/code"
    )


if __name__ == "__main__":
    main()
