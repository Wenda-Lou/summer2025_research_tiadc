"""
Prepare a pulse-width / duty-cycle sweep for the FPGA_simulator and bench.

This is the second dimension of the "low-cost impulse" study:

  - pulse width: how narrow can the injected pulse be while staying detectable?
  - duty cycle / sparsity: how sparse can the pulses be (larger period)?

All waveforms use:

  - tone: 299.375 MHz (coherent, 29.9375 cycles/event at period 260)
  - amplitude: 500 LSB (best point from the amplitude sweep)
  - DAC/ADC: 2600 / 1300 MSPS

Usage:

    python -m calibration_loop.sweep_pulse \
        --out calibration_out/pulse_sweep

Outputs:

    calibration_out/pulse_sweep/pulse_<label>.txt
    calibration_out/pulse_sweep/pulse_<label>.json
    calibration_out/pulse_sweep/sweep_manifest.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .dither import DitherConfig, write_dac_files

BASE = dict(
    fs_dac=2600.0e6,
    n_dac_points=16640,
    adc_ratio=2,
    sig_cycles=1916,
    amp_dbfs=-1.5,
    dither_position_dac=96,
    dither_scale_lsb=500.0,
    seed=20260725,
)

# label -> (edge_dac, top_dac, period_dac)
PULSE_WIDTH_CONFIGS = [
    ("e4t4", 4, 4, 260),
    ("e8t4", 8, 4, 260),
    ("e8t8", 8, 8, 260),
    ("e16t8", 16, 8, 260),
    ("e16t32", 16, 32, 260),  # baseline geometry
    ("e32t32", 32, 32, 260),
]

DUTY_CONFIGS = [
    ("p260_e16t32", 16, 32, 260),  # baseline
    ("p520_e16t32", 16, 32, 520),
    ("p1040_e16t32", 16, 32, 1040),
    ("p520_e8t8", 8, 8, 520),
    ("p1040_e8t8", 8, 8, 1040),
]

ALL_CONFIGS = PULSE_WIDTH_CONFIGS + DUTY_CONFIGS


def cfg_for(edge_dac: int, top_dac: int, period_dac: int) -> DitherConfig:
    cfg = DitherConfig(**{
        **BASE,
        "dither_period_dac": period_dac,
        "dither_edge_dac": edge_dac,
        "dither_top_dac": top_dac,
    })
    cfg.validate()
    return cfg


def generate_one(
    label: str,
    edge_dac: int,
    top_dac: int,
    period_dac: int,
    out_dir: str | Path,
) -> dict:
    cfg = cfg_for(edge_dac, top_dac, period_dac)
    stem = f"pulse_{label}"
    result = write_dac_files(cfg, out_dir, stem=stem)
    d = result["meta"]["derived"]
    return {
        "label": label,
        "edge_dac": edge_dac,
        "top_dac": top_dac,
        "period_dac": period_dac,
        "pulse_len_dac": 2 * edge_dac + top_dac,
        "slot_period_adc": cfg.slot_period,
        "events_per_loop": cfg.n_events,
        "duty_cycle": d["dither_duty_cycle"],
        "txt": str(result["txt"]),
        "json": str(result["json"]),
        "tone_hz": d["main_tone_hz"],
        "coherence": cfg.tone_phase_coherence(),
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--out", default="calibration_out/pulse_sweep")
    parser.add_argument(
        "--configs",
        nargs="+",
        choices=[label for label, *_ in ALL_CONFIGS],
        default=[label for label, *_ in ALL_CONFIGS],
        help="Which configs to generate (default: all)",
    )
    args = parser.parse_args(argv)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    by_label = {label: (edge, top, period) for label, edge, top, period in ALL_CONFIGS}
    manifest = {"base": BASE, "configs": []}

    for label in args.configs:
        edge, top, period = by_label[label]
        info = generate_one(label, edge, top, period, out_dir)
        manifest["configs"].append(info)
        print(
            f"{label:14s} edge={edge:2d} top={top:2d} period={period:4d} "
            f"pulse={info['pulse_len_dac']:3d} DAC duty={info['duty_cycle']:.4f} "
            f"coherence={info['coherence']:.4f}"
        )

    manifest_path = out_dir / "sweep_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"\nManifest: {manifest_path}")


if __name__ == "__main__":
    main()
