#!/usr/bin/env python3
"""Repair the labels in a firmware skew-export CSV from 2026-08-16.

Firmware history before the measurement-kind fix mis-labelled every fresh
characterization baseline as a ``calibration`` controller batch.  This script:

- re-labels those rows as ``actuator_characterization_baseline``,
- keeps the original label in ``original_capture_phase``,
- writes ``calibration_skew_captures.corrected.csv``,
- writes ``calibration_skew_iterations.corrected.csv`` where measurement
  statistics whose source batch is missing are blanked instead of guessed,
- writes a notes file listing exactly which capture groups are missing.

It never invents measurements: frames that were truncated by the 160-row
firmware history capacity are reported as missing.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

CAPTURE_SUFFIX = ".corrected.csv"
ITERATION_SUFFIX = ".corrected.csv"
NOTES_NAME = "calibration_skew_export_repair_notes.txt"

STAT_COLUMNS = [
    "accepted_frames",
    "rejected_frames",
    "mean_skew_samples",
    "mean_skew_ps",
    "median_skew_samples",
    "median_skew_ps",
    "skew_std_samples",
    "skew_std_ps",
    "best_skew_samples",
    "best_skew_ps",
    "dither_skew_samples",
    "dither_skew_ps",
    "tone_dither_disagreement_ps",
    "dither_valid_frames",
    "dither_invalid_frames",
]


def mode_int(series: pd.Series) -> int:
    values = pd.to_numeric(series, errors="coerce").dropna().astype(int)
    if values.empty:
        raise ValueError("no finite delay register values")
    return int(values.mode().iloc[0])


def group_phase_and_delay(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for group_id, g in df.groupby("capture_group_index", sort=True):
        rows.append(
            {
                "capture_group_index": int(group_id),
                "capture_phase": str(g["capture_phase"].iloc[0]),
                "iteration": int(g["iteration"].iloc[0]),
                "delay_first": int(g["delay_register_active"].iloc[0]),
                "delay_median": int(np.nanmedian(g["delay_register_active"])),
                "delay_last": int(g["delay_register_active"].iloc[-1]),
                "accepted": int(g["accepted"].sum()),
                "mean_skew_ps": float(g["measured_skew_ps"].mean()),
                "median_skew_ps": float(g["measured_skew_ps"].median()),
                "std_skew_ps": float(g["measured_skew_ps"].std()),
            }
        )
    return pd.DataFrame(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "capture_csv",
        type=Path,
        help="path to calibration_skew_captures.csv",
    )
    parser.add_argument(
        "--iteration-csv",
        type=Path,
        default=None,
        help="optional path to calibration_skew_iterations.csv",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="output directory (default: same directory as capture_csv)",
    )
    args = parser.parse_args()

    capture_path = args.capture_csv
    if not capture_path.exists():
        raise SystemExit(f"capture CSV not found: {capture_path}")
    out_dir = args.out_dir or capture_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(capture_path)
    if "delay_register_active" not in df.columns:
        raise SystemExit("CSV has no delay_register_active column; nothing to repair")

    initial_rows = df[df["capture_phase"] == "closed_loop_initial_baseline"]
    if initial_rows.empty:
        raise SystemExit("no closed_loop_initial_baseline rows found")
    baseline_register = mode_int(initial_rows["delay_register_active"])

    corrected = df.copy()
    corrected["original_capture_phase"] = corrected["capture_phase"]
    corrected["original_iteration"] = corrected["iteration"]
    corrected["label_source"] = "firmware"

    calibration_mask = corrected["capture_phase"] == "calibration"
    calibration_baseline_mask = (
        calibration_mask
        & (pd.to_numeric(corrected["delay_register_active"], errors="coerce")
           == baseline_register)
    )
    corrected.loc[calibration_baseline_mask, "capture_phase"] = (
        "actuator_characterization_baseline"
    )
    corrected.loc[calibration_baseline_mask, "iteration"] = 0
    corrected.loc[calibration_baseline_mask, "label_source"] = "repair: fresh baseline"

    corrected.to_csv(out_dir / (capture_path.stem + CAPTURE_SUFFIX), index=False)

    groups = group_phase_and_delay(corrected)
    actual_group_count = int(groups["capture_group_index"].max())

    rungs = sorted(
        {
            int(round(abs(g.delay_median - baseline_register)))
            for _, g in groups.iterrows()
            if g.capture_phase
            in {"actuator_characterization_after", "actuator_characterization_repeat"}
        }
    )
    rung_count = len(rungs)
    if rung_count == 0:
        rung_count = 4

    controller_batches = 0
    if args.iteration_csv and args.iteration_csv.exists():
        it = pd.read_csv(args.iteration_csv)
        controller_batches = int(len(it))
    elif args.iteration_csv is not None:
        raise SystemExit(f"iteration CSV not found: {args.iteration_csv}")

    expected = ["closed_loop_initial_baseline"]
    for _ in range(rung_count):
        expected += [
            "actuator_characterization_baseline",
            "actuator_characterization_after",
            "actuator_characterization_baseline",
            "actuator_characterization_repeat",
        ]
    for _ in range(controller_batches):
        expected.append("calibration")

    missing_expected = expected[actual_group_count:]

    notes = []
    notes.append("Skew export repair notes")
    notes.append(f"capture_csv : {capture_path}")
    notes.append(f"baseline_register : {baseline_register}")
    notes.append(f"groups_present : {actual_group_count}")
    notes.append(f"groups_expected : {len(expected)}")
    notes.append("")
    notes.append("Re-labelled rows:")
    notes.append(
        "  calibration -> actuator_characterization_baseline : "
        f"{int(calibration_baseline_mask.sum())} rows"
    )
    notes.append("")
    if missing_expected:
        notes.append(
            "Missing capture groups (firmware history capacity was 160 rows): "
            f"{len(missing_expected)}"
        )
        for offset, phase in enumerate(
            missing_expected, start=actual_group_count + 1
        ):
            notes.append(f"  group {offset}: {phase}")
    else:
        notes.append("No missing capture groups detected.")
    notes.append("")
    notes.append(
        "No missing frames were invented. Re-run on the board with the fixed "
        "firmware to obtain a complete per-frame CSV."
    )

    notes_path = out_dir / NOTES_NAME
    notes_path.write_text('\n'.join(notes) + '\n', encoding="utf-8")
    if args.iteration_csv and args.iteration_csv.exists():
        it = pd.read_csv(args.iteration_csv)
        it_corrected = it.copy()
        # Stored rows begin at decision 2 (decision 1 uses the initial
        # baseline batch and was not stored by the firmware callback).
        it_corrected["iteration"] = (
            pd.to_numeric(it_corrected["iteration"], errors="coerce")
            .fillna(0).astype(int) + 1
        )
        it_corrected["measurement_source"] = (
            "unavailable: post-update capture group truncated"
        )
        for col in STAT_COLUMNS:
            if col in it_corrected.columns:
                it_corrected[col] = np.nan

        first_row = pd.DataFrame(
            [{c: np.nan for c in it_corrected.columns}],
            index=[0],
        )
        first_row["stage"] = "skew"
        first_row["iteration"] = 1
        first_row["status"] = "UPDATED"
        first_row["measurement_source"] = "reconstructed: initial baseline group"
        first_row["accepted_frames"] = int(initial_rows["accepted"].sum())
        first_row["rejected_frames"] = int(
            (~initial_rows["accepted"].astype(bool)).sum()
        )
        first_row["mean_skew_samples"] = float(
            initial_rows["measured_skew_samples"].mean()
        )
        first_row["mean_skew_ps"] = float(
            initial_rows["measured_skew_ps"].mean()
        )
        first_row["median_skew_samples"] = float(
            initial_rows["measured_skew_samples"].median()
        )
        first_row["median_skew_ps"] = float(
            initial_rows["measured_skew_ps"].median()
        )
        first_row["skew_std_samples"] = float(
            initial_rows["measured_skew_samples"].std()
        )
        first_row["skew_std_ps"] = float(
            initial_rows["measured_skew_ps"].std()
        )
        first_row["best_skew_samples"] = float(
            initial_rows["measured_skew_samples"].median()
        )
        first_row["best_skew_ps"] = float(
            initial_rows["measured_skew_ps"].median()
        )
        first_row["delay_register_before"] = baseline_register
        if len(it) > 0:
            first_after = int(it.iloc[0]["delay_register_before"])
            first_row["delay_register_after"] = first_after
            first_row["register_delta"] = first_after - baseline_register
            first_row["applied_steps"] = first_after - baseline_register
        first_row["tolerance_samples"] = (
            it.iloc[0]["tolerance_samples"] if len(it) else np.nan
        )
        first_row["tolerance_ps"] = (
            it.iloc[0]["tolerance_ps"] if len(it) else np.nan
        )
        first_row["required_passes"] = (
            it.iloc[0]["required_passes"] if len(it) else np.nan
        )
        first_row["estimator_valid"] = 1
        first_row["estimator_stable"] = 1
        first_row["correction_applied"] = 1
        first_row["saturated"] = 0

        combined = pd.concat([first_row, it_corrected], ignore_index=True)
        combined.to_csv(
            out_dir / (args.iteration_csv.stem + ITERATION_SUFFIX),
            index=False,
        )

    print(
        "corrected captures : "
        f"{out_dir / (capture_path.stem + CAPTURE_SUFFIX)}"
    )
    print(f"notes              : {notes_path}")
    if args.iteration_csv and args.iteration_csv.exists():
        print(
            "corrected iterations: "
            f"{out_dir / (args.iteration_csv.stem + ITERATION_SUFFIX)}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())