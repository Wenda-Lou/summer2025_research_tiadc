"""
Prepare and sanity-check an impulse-dither amplitude sweep.

This matches the current bench baseline waveform:

    fs_dac          = 2.600 GSPS
    fs_adc          = 1.300 GSPS
    dither period   = 260 DAC samples = 130 ADC samples
    pulse geometry  = 16/32/16 DAC samples (raised cosine)
    tone            = 199.375 MHz (coherent, 0.9375 cycles/event)

Usage:

    # Generate waveforms only
    python -m calibration_loop.sweep_amplitude \
        --amplitudes 500 1000 2000 3000 4000

    # Generate and run a quick simulation sanity check for each amplitude
    python -m calibration_loop.sweep_amplitude --sim --iterations 40

The generated TXT files are DPG-compatible (one signed 16-bit integer per line)
and can be uploaded to the AD9164 DPG in place of the baseline waveform.  The
JSON sidecar files carry the DitherConfig needed by calibration_loop probe/bench.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .dither import DitherConfig, write_dac_files
from .estimator import CalibrationState
from .loop import CalibrationLoop, LoopOptions
from .simulate import BenchModel

# Baseline board waveform parameters (see generated_samples/*.json).
BASELINE = dict(
    fs_dac=2600.0e6,
    n_dac_points=16640,
    adc_ratio=2,
    sig_cycles=1276,
    amp_dbfs=-1.5,
    dither_period_dac=260,
    dither_position_dac=96,
    dither_edge_dac=16,
    dither_top_dac=32,
    dither_scale_lsb=2000.0,
    seed=20260725,
)

DEFAULT_AMPLITUDES_LSB = [500.0, 1000.0, 2000.0, 3000.0, 4000.0]


def board_cfg(amplitude_lsb: float) -> DitherConfig:
    """Return a DitherConfig for the bench baseline with the given amplitude."""
    cfg = DitherConfig(**{**BASELINE, "dither_scale_lsb": float(amplitude_lsb)})
    cfg.validate()
    return cfg


def generate_one(
    amplitude_lsb: float,
    out_dir: str | Path,
    prefix: str = "sine_199p375MHz_2p6GSPS_impulse_dither",
) -> dict:
    """Generate one DPG TXT + JSON pair for the requested dither amplitude."""
    cfg = board_cfg(amplitude_lsb)
    amp = int(round(amplitude_lsb))
    stem = f"{prefix}_a{amp}"
    result = write_dac_files(cfg, out_dir, stem=stem)
    d = result["meta"]["derived"]
    return {
        "amplitude_lsb": float(amplitude_lsb),
        "stem": stem,
        "txt": str(result["txt"]),
        "json": str(result["json"]),
        "n_events": cfg.n_events,
        "duty_cycle": d["dither_duty_cycle"],
        "clipping": d["clipping"],
        "min_sample": d["min_sample"],
        "max_sample": d["max_sample"],
        "tone_hz": d["main_tone_hz"],
    }


def run_sim(cfg: DitherConfig, iterations: int) -> dict:
    """Run the closed-loop simulator once and return the converged residual stats."""
    bench = BenchModel(cfg=cfg)
    state = CalibrationState()
    options = LoopOptions()
    loop = CalibrationLoop(bench, cfg, state=state, options=options)
    loop.run(iterations, verbose=False)

    if not loop.log:
        return {"iterations": iterations, "rows": 0}

    tail = loop.log[-max(1, len(loop.log) // 5):]

    def avg(key):
        vals = [r[key] for r in tail if np.isfinite(r[key])]
        return float(np.mean(vals)) if vals else float("nan")

    accepted = [r for r in loop.log if not r.get("rejected")]
    detected = sum(1 for r in loop.log if not r.get("rejected") and r.get("align_margin", 0) >= 6.0)
    return {
        "iterations": iterations,
        "rows": len(loop.log),
        "accepted_rows": len(accepted),
        "gain_ratio": avg("gain_ratio"),
        "offset_mismatch_codes": avg("offset_b_codes") - avg("offset_a_codes"),
        "skew_mismatch_ps": avg("skew_mismatch_ps"),
        "raw_sndr_db": avg("raw_sndr_db"),
        "cal_sndr_db": avg("cal_sndr_db"),
        "detected_rows": detected,
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--amplitudes",
        nargs="+",
        type=float,
        default=DEFAULT_AMPLITUDES_LSB,
        help="Dither amplitudes in LSB (default: %(default)s)",
    )
    parser.add_argument(
        "--out",
        default="calibration_out/amplitude_sweep",
        help="Output directory for waveforms and sweep manifest",
    )
    parser.add_argument(
        "--prefix",
        default="sine_199p375MHz_2p6GSPS_impulse_dither",
        help="Stem prefix for generated files",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Also run a quick simulation sanity check for each amplitude",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=40,
        help="Simulation iterations when --sim is used",
    )
    args = parser.parse_args(argv)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "baseline": BASELINE,
        "amplitudes": [],
    }

    for amplitude in args.amplitudes:
        info = generate_one(amplitude, out_dir, prefix=args.prefix)
        print(
            f"amplitude {info['amplitude_lsb']:7.0f} LSB -> {Path(info['txt']).name} "
            f"(clipping={info['clipping']}, min={info['min_sample']}, max={info['max_sample']})"
        )

        if args.sim:
            cfg = board_cfg(amplitude)
            sim = run_sim(cfg, args.iterations)
            info["sim"] = sim
            print(
                f"  sim: accepted={sim['accepted_rows']}/{sim['rows']}, "
                f"gain_ratio={sim['gain_ratio']:+.4f}, "
                f"offset={sim['offset_mismatch_codes']:+.3f} LSB, "
                f"skew={sim['skew_mismatch_ps']:+.3f} ps, "
                f"SNDR={sim['cal_sndr_db']:.2f} dB"
            )

        manifest["amplitudes"].append(info)

    manifest_path = out_dir / "sweep_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"\nManifest: {manifest_path}")

    # Print ready-to-run bench commands for the hardware phase.
    print("\nBench commands (after uploading the corresponding TXT to DPG):")
    for amplitude in args.amplitudes:
        amp = int(round(amplitude))
        print(
            f"  python -m calibration_loop.run_calibration bench "
            f"--uart COM3 --dither-period 260 --dither-position 96 "
            f"--dither-edge 16 --dither-top 32 --dither-scale {amp} "
            f"--sig-cycles 1276 --n-dac-points 16640 --amp-dbfs -1.5 "
            f"--seed 20260725 --stem sweep_a{amp}"
        )


if __name__ == "__main__":
    main()
