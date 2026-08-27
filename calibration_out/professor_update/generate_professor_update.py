"""Generate verified board-sweep data, figures, and the two-page update PDF.

This script uses only committed board captures and summaries. It performs no
hardware access and no estimator or calibration reruns.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import mean

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import (
    Image,
    KeepTogether,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


REPO = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
AMP_SUMMARY = REPO / "amplitude_sweep_board_results.json"
PULSE_SUMMARY = REPO / "pulse_sweep_board_results.json"
def _firmware_root() -> Path:
    """Support the repository's in-progress test_platform -> firmware relocation."""
    original = REPO / "test_platform/thesis_v3_500mhz_appl"
    relocated = REPO / "firmware/thesis_v3_500mhz_appl"
    if original.exists():
        return original
    if relocated.exists():
        return relocated
    raise FileNotFoundError("Neither the original nor relocated firmware tree exists")


FIRMWARE_ROOT = _firmware_root()
AMP_RAW_ROOT = FIRMWARE_ROOT / "adc_data/calibration_exports/amplitude pulse"
PULSE_RAW_ROOT = FIRMWARE_ROOT / "adc_data/calibration_exports/pulse_width sweep"
PDF_PATH = REPO / "TIADC_IMPULSE_BOARD_UPDATE_EN.pdf"

NAVY = "#17324D"
BLUE = "#2878B5"
TEAL = "#2A9D8F"
ORANGE = "#E76F51"
GOLD = "#E9C46A"
GRID = "#D8DEE5"
TEXT = "#20252B"


def _read_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def _float_values(rows: list[dict[str, str]], key: str) -> list[float]:
    values = []
    for row in rows:
        value = row.get(key, "")
        if value and value.lower() != "nan":
            values.append(float(value))
    return values


def _assert_close(label: str, field: str, actual: float, expected: float) -> None:
    if not math.isclose(actual, expected, rel_tol=1e-10, abs_tol=1e-10):
        raise ValueError(
            f"{label}: raw {field}={actual!r} does not match summary {expected!r}"
        )


def _verify_and_enrich(
    sweep: str, runs: list[dict], raw_root: Path
) -> list[dict]:
    enriched = []
    for run in runs:
        label = run["label"]
        run_dir = raw_root / label
        timing = _read_csv(run_dir / "calibration_timing_captures.csv")
        skew = _read_csv(run_dir / "calibration_skew_captures.csv")
        perf_path = run_dir / "calibration_performance.csv"
        performance = _read_csv(perf_path) if perf_path.exists() else []

        raw_corr = mean(_float_values(timing, "correlation"))
        raw_rmse = mean(_float_values(timing, "tone_rmse_codes"))
        raw_a_valid = mean(_float_values(skew, "dither_A_valid"))
        raw_b_valid = mean(_float_values(skew, "dither_B_valid"))
        raw_skew_valid = mean(_float_values(skew, "dither_skew_valid"))
        _assert_close(label, "timing correlation", raw_corr, run["timing_correlation_mean"])
        _assert_close(label, "tone RMSE", raw_rmse, run["tone_rmse_codes_mean"])
        _assert_close(label, "dither A valid", raw_a_valid, run["dither_A_valid_rate"])
        _assert_close(label, "dither B valid", raw_b_valid, run["dither_B_valid_rate"])
        _assert_close(
            label, "dither skew valid", raw_skew_valid, run["dither_skew_valid_rate"]
        )

        if performance:
            final_skews = _float_values(performance, "final_skew_ps")
            if run["final_skew_ps"] is None:
                raise ValueError(f"{label}: performance CSV exists but summary skew is null")
            _assert_close(label, "final skew", final_skews[0], run["final_skew_ps"])
            stage5 = {
                "stage5_capture_count": len(performance),
                "stage5_mean_cal_parallel_sndr_db": mean(
                    _float_values(performance, "cal_parallel_avg_sndr_db")
                ),
                "stage5_mean_cal_parallel_sfdr_db": mean(
                    _float_values(performance, "cal_parallel_avg_sfdr_db")
                ),
                "stage5_mean_cal_parallel_enob_bits": mean(
                    _float_values(performance, "cal_parallel_avg_enob")
                ),
            }
        else:
            if run["final_skew_ps"] is not None:
                raise ValueError(f"{label}: summary skew exists but performance CSV is absent")
            stage5 = {
                "stage5_capture_count": 0,
                "stage5_mean_cal_parallel_sndr_db": None,
                "stage5_mean_cal_parallel_sfdr_db": None,
                "stage5_mean_cal_parallel_enob_bits": None,
            }

        record = dict(run)
        record.update(
            {
                "sweep": sweep,
                "timing_capture_count": len(timing),
                "skew_capture_count": len(skew),
                "dither_ab_valid_rate": (raw_a_valid + raw_b_valid) / 2.0,
                "raw_run_dir": run_dir.relative_to(REPO).as_posix(),
                **stage5,
            }
        )
        enriched.append(record)
    return enriched


def _configure_plotting() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10.0,
            "axes.titlesize": 11.5,
            "axes.labelsize": 10.5,
            "axes.edgecolor": NAVY,
            "axes.labelcolor": TEXT,
            "axes.titlecolor": NAVY,
            "xtick.color": TEXT,
            "ytick.color": TEXT,
            "text.color": TEXT,
            "grid.color": GRID,
            "grid.linewidth": 0.7,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.bbox": "tight",
        }
    )


def _save(fig: plt.Figure, filename: str) -> None:
    fig.savefig(OUT / filename, dpi=300, pad_inches=0.08)
    plt.close(fig)


def _amplitude_detection(ax: plt.Axes, amp: list[dict]) -> None:
    x = [r["amplitude_lsb"] for r in amp]
    y = [100.0 * r["dither_ab_valid_rate"] for r in amp]
    ax.plot(x, y, color=BLUE, marker="o", linewidth=2.1, markersize=6)
    ax.fill_between(x, 95.0, 100.0, color=TEAL, alpha=0.08, zorder=0)
    for xv, yv in zip(x, y):
        ax.annotate(f"{yv:.1f}%", (xv, yv), xytext=(0, 7),
                    textcoords="offset points", ha="center", fontsize=8.5)
    ax.set_title("Impulse amplitude vs detection reliability")
    ax.set_xlabel("Impulse amplitude (LSB)")
    ax.set_ylabel("Dither A/B valid rate (%)")
    ax.set_xticks(x)
    ax.set_ylim(95.0, 100.3)
    ax.yaxis.set_major_formatter(PercentFormatter(xmax=100, decimals=0))
    ax.grid(axis="y")


def _amplitude_rmse(ax: plt.Axes, amp: list[dict]) -> None:
    x = [r["amplitude_lsb"] for r in amp]
    y = [r["tone_rmse_codes_mean"] for r in amp]
    ax.plot(x, y, color=BLUE, marker="o", linewidth=2.1, markersize=6)
    ax.scatter([x[-1]], [y[-1]], color=ORANGE, s=55, zorder=4)
    for xv, yv in zip(x, y):
        ax.annotate(f"{yv:.2f}", (xv, yv), xytext=(0, 7),
                    textcoords="offset points", ha="center", fontsize=8.5)
    ax.annotate(
        "3.88x the 250-LSB RMSE",
        xy=(x[-1], y[-1]),
        xytext=(-105, -18),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": ORANGE, "lw": 1.2},
        color=ORANGE,
        fontsize=8.5,
    )
    ax.set_title("Impulse amplitude vs tone disturbance")
    ax.set_xlabel("Impulse amplitude (LSB)")
    ax.set_ylabel("Tone-fit RMSE (codes)")
    ax.set_xticks(x)
    ax.set_ylim(0, 21)
    ax.grid(axis="y")


def _pulse_labels(pulse: list[dict]) -> list[str]:
    return [
        f"{r['label']}\n{r['pulse_len_dac']} samp, {100*r['duty_cycle']:.1f}% duty"
        for r in pulse
    ]


def _pulse_detection(ax: plt.Axes, pulse: list[dict], compact: bool = False) -> None:
    labels = _pulse_labels(pulse)
    y = [100.0 * r["dither_ab_valid_rate"] for r in pulse]
    bar_colors = [TEAL if r["period_dac"] == 260 else ORANGE for r in pulse]
    positions = range(len(pulse))
    bars = ax.barh(positions, y, color=bar_colors, height=0.65)
    for bar, value in zip(bars, y):
        ax.text(value + 1.0, bar.get_y() + bar.get_height() / 2, f"{value:.1f}%",
                ha="left", va="center", fontsize=8.5)
    if compact:
        ax.set_title("Detection reliability\n260 period (teal) | 520 period (orange)",
                     fontsize=9.6)
    else:
        ax.set_title("Pulse width / duty vs detection reliability\n"
                     "Teal: 260-sample period | Orange: 520-sample period")
    ax.set_xlabel("Dither A/B valid rate (%)")
    ax.set_yticks(positions, labels, fontsize=8.2)
    ax.set_xlim(0, 108)
    ax.xaxis.set_major_formatter(PercentFormatter(xmax=100, decimals=0))
    ax.invert_yaxis()
    ax.grid(axis="x")


def _pulse_rmse(ax: plt.Axes, pulse: list[dict], compact: bool = False) -> None:
    labels = _pulse_labels(pulse)
    y = [r["tone_rmse_codes_mean"] for r in pulse]
    bar_colors = [TEAL if r["period_dac"] == 260 else ORANGE for r in pulse]
    positions = range(len(pulse))
    bars = ax.barh(positions, y, color=bar_colors, height=0.65)
    for bar, value in zip(bars, y):
        ax.text(value + 0.12, bar.get_y() + bar.get_height() / 2, f"{value:.2f}",
                ha="left", va="center", fontsize=8.5)
    if compact:
        ax.set_title("Tone disturbance\n260 period (teal) | 520 period (orange)",
                     fontsize=9.6)
    else:
        ax.set_title("Pulse width / duty vs tone disturbance\n"
                     "Teal: 260-sample period | Orange: 520-sample period")
    ax.set_xlabel("Tone-fit RMSE (codes)")
    ax.set_yticks(positions, labels, fontsize=8.2)
    ax.set_xlim(0, 8)
    ax.invert_yaxis()
    ax.grid(axis="x")


def _generate_figures(amp: list[dict], pulse: list[dict]) -> None:
    _configure_plotting()
    for filename, draw in (
        ("amplitude_detection.png", lambda ax: _amplitude_detection(ax, amp)),
        ("amplitude_tone_rmse.png", lambda ax: _amplitude_rmse(ax, amp)),
        ("pulse_detection.png", lambda ax: _pulse_detection(ax, pulse)),
        ("pulse_tone_rmse.png", lambda ax: _pulse_rmse(ax, pulse)),
    ):
        fig, ax = plt.subplots(figsize=(6.6, 4.25), constrained_layout=True)
        draw(ax)
        _save(fig, filename)

    fig, axes = plt.subplots(1, 2, figsize=(7.25, 3.05), constrained_layout=True)
    _amplitude_detection(axes[0], amp)
    _amplitude_rmse(axes[1], amp)
    _save(fig, "amplitude_summary.png")

    fig, axes = plt.subplots(1, 2, figsize=(7.25, 3.05), constrained_layout=True)
    _pulse_detection(axes[0], pulse, compact=True)
    _pulse_rmse(axes[1], pulse, compact=True)
    _save(fig, "pulse_summary.png")


def _write_plot_data(amp: list[dict], pulse: list[dict]) -> None:
    fields = [
        "sweep",
        "label",
        "amplitude_lsb",
        "pulse_len_dac",
        "period_dac",
        "duty_cycle_pct",
        "timing_correlation_mean",
        "tone_rmse_codes_mean",
        "final_skew_ps",
        "dither_ab_valid_rate_pct",
        "timing_capture_count",
        "skew_capture_count",
        "stage5_capture_count",
        "stage5_mean_cal_parallel_sndr_db",
        "stage5_mean_cal_parallel_sfdr_db",
        "stage5_mean_cal_parallel_enob_bits",
        "raw_run_dir",
    ]
    with (OUT / "board_sweep_plot_data.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for run in amp + pulse:
            writer.writerow(
                {
                    "sweep": run["sweep"],
                    "label": run["label"],
                    "amplitude_lsb": run.get("amplitude_lsb", ""),
                    "pulse_len_dac": run.get("pulse_len_dac", ""),
                    "period_dac": run.get("period_dac", ""),
                    "duty_cycle_pct": (
                        f"{100.0 * run['duty_cycle']:.4f}" if "duty_cycle" in run else ""
                    ),
                    "timing_correlation_mean": run["timing_correlation_mean"],
                    "tone_rmse_codes_mean": run["tone_rmse_codes_mean"],
                    "final_skew_ps": (
                        "" if run["final_skew_ps"] is None else run["final_skew_ps"]
                    ),
                    "dither_ab_valid_rate_pct": 100.0 * run["dither_ab_valid_rate"],
                    "timing_capture_count": run["timing_capture_count"],
                    "skew_capture_count": run["skew_capture_count"],
                    "stage5_capture_count": run["stage5_capture_count"],
                    "stage5_mean_cal_parallel_sndr_db": (
                        "" if run["stage5_mean_cal_parallel_sndr_db"] is None
                        else run["stage5_mean_cal_parallel_sndr_db"]
                    ),
                    "stage5_mean_cal_parallel_sfdr_db": (
                        "" if run["stage5_mean_cal_parallel_sfdr_db"] is None
                        else run["stage5_mean_cal_parallel_sfdr_db"]
                    ),
                    "stage5_mean_cal_parallel_enob_bits": (
                        "" if run["stage5_mean_cal_parallel_enob_bits"] is None
                        else run["stage5_mean_cal_parallel_enob_bits"]
                    ),
                    "raw_run_dir": run["raw_run_dir"],
                }
            )


def _write_derived_metrics(amp: list[dict], pulse: list[dict]) -> dict:
    amp_valid = [100.0 * r["dither_ab_valid_rate"] for r in amp]
    amp_corr = [r["timing_correlation_mean"] for r in amp]
    amp_rmse = [r["tone_rmse_codes_mean"] for r in amp]
    standard = [r for r in pulse if r["period_dac"] == 260]
    long_period = [r for r in pulse if r["period_dac"] == 520]
    complete = [r for r in amp + pulse if r["stage5_capture_count"]]
    metrics = {
        "definitions": {
            "dither_ab_valid_rate_pct": (
                "100 times the arithmetic mean of all dither_A_valid and "
                "dither_B_valid flags in calibration_skew_captures.csv. "
                "A and B rates are equal in every included run."
            ),
            "duty_cycle_pct": (
                "100 times pulse_len_dac / period_dac; a temporal occupancy "
                "derived from waveform geometry."
            ),
            "stage5_means": (
                "Arithmetic means across each available calibration_performance.csv "
                "of cal_parallel_avg_sndr_db, cal_parallel_avg_sfdr_db, and "
                "cal_parallel_avg_enob."
            ),
        },
        "amplitude_sweep": {
            "validity_range_pct": [min(amp_valid), max(amp_valid)],
            "validity_span_percentage_points": max(amp_valid) - min(amp_valid),
            "timing_correlation_range": [min(amp_corr), max(amp_corr)],
            "timing_correlation_span": max(amp_corr) - min(amp_corr),
            "rmse_250_to_2000_increase_codes": amp_rmse[-1] - amp_rmse[0],
            "rmse_250_to_2000_ratio": amp_rmse[-1] / amp_rmse[0],
            "rmse_250_to_2000_increase_pct": 100.0 * (amp_rmse[-1] / amp_rmse[0] - 1),
        },
        "pulse_sweep": {
            "standard_period_validity_range_pct": [
                min(100.0 * r["dither_ab_valid_rate"] for r in standard),
                max(100.0 * r["dither_ab_valid_rate"] for r in standard),
            ],
            "long_period_validity_range_pct": [
                min(100.0 * r["dither_ab_valid_rate"] for r in long_period),
                max(100.0 * r["dither_ab_valid_rate"] for r in long_period),
            ],
            "e8t8_period_doubling_validity_drop_percentage_points": 100.0 * (
                next(r for r in pulse if r["label"] == "e8t8")["dither_ab_valid_rate"]
                - next(r for r in pulse if r["label"] == "p520_e8t8")["dither_ab_valid_rate"]
            ),
            "e16t32_period_doubling_validity_drop_percentage_points": 100.0 * (
                next(r for r in pulse if r["label"] == "e16t32")["dither_ab_valid_rate"]
                - next(r for r in pulse if r["label"] == "p520_e16t32")["dither_ab_valid_rate"]
            ),
        },
        "stage5_complete_runs": {
            "count": len(complete),
            "missing": [r["label"] for r in amp + pulse if not r["stage5_capture_count"]],
            "mean_cal_parallel_sndr_db_range": [
                min(r["stage5_mean_cal_parallel_sndr_db"] for r in complete),
                max(r["stage5_mean_cal_parallel_sndr_db"] for r in complete),
            ],
            "mean_cal_parallel_sfdr_db_range": [
                min(r["stage5_mean_cal_parallel_sfdr_db"] for r in complete),
                max(r["stage5_mean_cal_parallel_sfdr_db"] for r in complete),
            ],
            "mean_cal_parallel_enob_bits_range": [
                min(r["stage5_mean_cal_parallel_enob_bits"] for r in complete),
                max(r["stage5_mean_cal_parallel_enob_bits"] for r in complete),
            ],
        },
        "provenance": {
            "amplitude_summary": AMP_SUMMARY.relative_to(REPO).as_posix(),
            "pulse_summary": PULSE_SUMMARY.relative_to(REPO).as_posix(),
            "new_board_tests_run": False,
            "injected_energy_metric": (
                "Not computed: the committed artifacts provide waveform geometry and "
                "nominal DAC amplitude, but no calibrated ADC-domain injected-energy "
                "measurement for these runs."
            ),
        },
    }
    with (OUT / "derived_metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(metrics, stream, indent=2)
        stream.write("\n")
    return metrics


def _report_styles() -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "title": ParagraphStyle(
            "ReportTitle", parent=base["Title"], fontName="Helvetica-Bold",
            fontSize=17, leading=19, textColor=colors.HexColor(NAVY),
            alignment=TA_LEFT, spaceAfter=5,
        ),
        "date": ParagraphStyle(
            "Date", parent=base["Normal"], fontName="Helvetica",
            fontSize=8.2, leading=10, textColor=colors.HexColor("#5A6470"),
            spaceAfter=7,
        ),
        "heading": ParagraphStyle(
            "Heading", parent=base["Heading2"], fontName="Helvetica-Bold",
            fontSize=10.8, leading=12.5, textColor=colors.HexColor(NAVY),
            spaceBefore=4, spaceAfter=3,
        ),
        "body": ParagraphStyle(
            "Body", parent=base["BodyText"], fontName="Helvetica",
            fontSize=8.7, leading=11.2, textColor=colors.HexColor(TEXT),
            spaceAfter=4,
        ),
        "takeaway": ParagraphStyle(
            "Takeaway", parent=base["BodyText"], fontName="Helvetica-Bold",
            fontSize=9.0, leading=11.8, textColor=colors.HexColor(NAVY),
            leftIndent=8, rightIndent=8, spaceBefore=4, spaceAfter=4,
        ),
        "small": ParagraphStyle(
            "Small", parent=base["BodyText"], fontName="Helvetica",
            fontSize=7.4, leading=9.2, textColor=colors.HexColor("#4D5966"),
            spaceAfter=3,
        ),
        "question": ParagraphStyle(
            "Question", parent=base["BodyText"], fontName="Helvetica",
            fontSize=8.5, leading=10.8, leftIndent=11, firstLineIndent=-9,
            textColor=colors.HexColor(TEXT), spaceAfter=2,
        ),
    }


def _footer(canvas, doc) -> None:
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor(GRID))
    canvas.setLineWidth(0.5)
    canvas.line(0.58 * inch, 0.43 * inch, 7.92 * inch, 0.43 * inch)
    canvas.setFont("Helvetica", 7)
    canvas.setFillColor(colors.HexColor("#69737E"))
    canvas.drawString(0.58 * inch, 0.27 * inch, "TIADC calibration project - board evidence only")
    canvas.drawRightString(7.92 * inch, 0.27 * inch, f"Page {doc.page}")
    canvas.restoreState()


def _generate_pdf(metrics: dict) -> None:
    styles = _report_styles()
    doc = SimpleDocTemplate(
        str(PDF_PATH), pagesize=letter,
        leftMargin=0.58 * inch, rightMargin=0.58 * inch,
        topMargin=0.47 * inch, bottomMargin=0.52 * inch,
        title="TIADC Impulse Dither - Board-Level Operating-Region Update",
        author="TIADC Calibration Project",
        subject="Board-level amplitude and pulse/duty sweep update",
    )
    story = []
    story.append(Paragraph("TIADC Impulse Dither - Board-Level Operating-Region Update", styles["title"]))
    story.append(Paragraph("Research update for Professor Liu | 27 August 2026", styles["date"]))
    takeaway = Table(
        [[Paragraph(
            "<b>Key takeaway.</b> Board sweeps show that sparse, low-amplitude impulse "
            "excitation remains reliably detectable while introducing limited tone-path "
            "disturbance. Very narrow pulses are viable; excessively sparse repetition "
            "reduces detection. This strengthens the practical injection operating-region "
            "case, while the known gain and fine-skew limitations remain unchanged.",
            styles["takeaway"],
        )]],
        colWidths=[7.2 * inch],
    )
    takeaway.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), colors.HexColor("#EEF5F7")),
        ("BOX", (0, 0), (-1, -1), 0.7, colors.HexColor(TEAL)),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
    ]))
    story.extend([takeaway, Spacer(1, 3)])
    story.append(Paragraph("1. Board-Level Amplitude Sweep", styles["heading"]))
    story.append(Image(str(OUT / "amplitude_summary.png"), width=7.2 * inch, height=3.03 * inch))
    story.append(Paragraph(
        "Across 250-2000 LSB, A/B event validity is 96.4-99.5% and timing "
        "correlation is 0.99881-0.99993. Detection therefore remains strong across "
        "the tested range. The cost of stronger injection is visible in the tone fit: "
        "RMSE rises from 4.83 codes at 250 LSB to 18.71 codes at 2000 LSB "
        "(+13.89 codes; 3.88x). The sweep identifies a broad low-disturbance "
        "operating region, not a single globally optimal amplitude; larger impulses "
        "are not inherently better.", styles["body"]))
    story.append(Paragraph(
        "Figure 1. Values are means from the committed board timing/skew captures. "
        "The shaded detection band marks 95-100%, not a pass threshold.", styles["small"]))

    story.append(PageBreak())
    story.append(Paragraph("2. Board-Level Pulse / Duty Sweep", styles["heading"]))
    story.append(Image(str(OUT / "pulse_summary.png"), width=7.2 * inch, height=3.03 * inch))
    story.append(Paragraph(
        "At the standard 260-DAC-sample period, a 12-sample pulse occupies only "
        "4.6% of the waveform and still gives 99.4% validity. The 24- and 64-sample "
        "cases give 98.0% and 99.4%. Doubling the period to 520 samples reduces "
        "validity to 75.3% (24-sample pulse) and 88.2% (64-sample pulse), despite "
        "similar tone RMSE. Impulses can be narrow and low-duty, but repetition "
        "should not be made arbitrarily sparse.", styles["body"]))
    story.append(Paragraph(
        "Figure 2. Duty is derived as pulse length / repetition period. The p520_e16t32 "
        "run has no calibration_performance.csv; no final-skew or Stage 5 value is assigned.",
        styles["small"]))

    implication = [
        [Paragraph("Evidence supports", styles["small"]), Paragraph("Not yet demonstrated", styles["small"])],
        [Paragraph("Sparse impulse generation; reliable recovery through the existing DAC-to-ADC chain; low temporal occupancy; a measured strength/disturbance trade-off.", styles["small"]),
         Paragraph("Independent passive coupling into an otherwise unmodified mission-signal path. A later coupling demo remains a publication-level option.", styles["small"])],
    ]
    table = Table(implication, colWidths=[3.55 * inch, 3.55 * inch])
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#EEF2F6")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.HexColor(NAVY)),
        ("BOX", (0, 0), (-1, -1), 0.6, colors.HexColor(GRID)),
        ("INNERGRID", (0, 0), (-1, -1), 0.4, colors.HexColor(GRID)),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    story.extend([Paragraph("3. Implication for Selling Point 1", styles["heading"]), table])
    story.append(Paragraph("4. Status of Selling Point 2", styles["heading"]))
    story.append(Paragraph(
        "Event detection is strong. Offset observability remains limited; dither gain "
        "has no stable calibration factor; fine-skew is unreliable under measured "
        "dispersion; bandwidth calibration was not pursued. Tone-based calibration "
        "remains the stable main path. No three-parameter separation is claimed.", styles["body"]))
    story.append(Paragraph("5. Decisions for the September Meeting", styles["heading"]))
    for index, question in enumerate(
        [
            "Are the current board results sufficient to begin consolidating the work toward publication?",
            "Should the next hardware step focus on an independent passive-coupling demonstration to strengthen selling point 1?",
            "What scope and division of work should be assigned to the additional FPGA student?",
        ],
        start=1,
    ):
        story.append(Paragraph(f"{index}. {question}", styles["question"]))
    story.append(Spacer(1, 2))
    stage5 = metrics["stage5_complete_runs"]
    story.append(Paragraph(
        "Data note. All ten summary rows were independently recomputed from committed "
        "CSV captures. Stage 5 exists for nine runs; its per-run calibrated parallel-average "
        f"means span {stage5['mean_cal_parallel_sndr_db_range'][0]:.2f}-"
        f"{stage5['mean_cal_parallel_sndr_db_range'][1]:.2f} dB SNDR, "
        f"{stage5['mean_cal_parallel_sfdr_db_range'][0]:.2f}-"
        f"{stage5['mean_cal_parallel_sfdr_db_range'][1]:.2f} dB SFDR, and "
        f"{stage5['mean_cal_parallel_enob_bits_range'][0]:.2f}-"
        f"{stage5['mean_cal_parallel_enob_bits_range'][1]:.2f} bits ENOB. These ranges "
        "are reported for provenance, not as evidence of dither-based calibration.", styles["small"]))

    doc.build(story, onFirstPage=_footer, onLaterPages=_footer)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    amp_summary = _read_json(AMP_SUMMARY)
    pulse_summary = _read_json(PULSE_SUMMARY)
    amp = _verify_and_enrich("amplitude", amp_summary["runs"], AMP_RAW_ROOT)
    pulse = _verify_and_enrich("pulse_duty", pulse_summary["runs"], PULSE_RAW_ROOT)
    _write_plot_data(amp, pulse)
    metrics = _write_derived_metrics(amp, pulse)
    _generate_figures(amp, pulse)
    _generate_pdf(metrics)
    print("Verified all 10 summary rows against committed raw CSV files.")
    print(f"Wrote figures/data to {OUT}")
    print(f"Wrote report PDF to {PDF_PATH}")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
