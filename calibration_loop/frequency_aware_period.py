"""
Frequency-aware dither period selection.

The impulse-dither estimator needs the main tone to advance a *non-integer*
number of cycles between dither events; otherwise the tone leaks into the
event-averaged dither statistics.  For a fixed dither period this silently
excludes some frequencies (e.g. exact 300 MHz with a 260-DAC period gives
30 cycles/event and coherence 1.0).

This tool searches the DPG loop length / dither period space for a coherent
configuration close to a requested tone frequency.

Usage:

    # Find a valid config for exact 300 MHz
    python -m calibration_loop.frequency_aware_period --target-mhz 300

    # Generate the corresponding DPG waveform
    python -m calibration_loop.frequency_aware_period --target-mhz 300 --out calibration_out/freq_aware_300

    # Tighten the allowed frequency error
    python -m calibration_loop.frequency_aware_period --target-mhz 300 --tol-hz 100
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .dither import DitherConfig, write_dac_files

DEFAULT_FS_DAC = 2600.0e6
DEFAULT_ADC_RATIO = 2
DEFAULT_EDGE_DAC = 16
DEFAULT_TOP_DAC = 32
DEFAULT_AMPLITUDE_LSB = 500.0
DEFAULT_SEED = 20260725
DEFAULT_MIN_PERIOD_DAC = 128
DEFAULT_MAX_PERIOD_DAC = 2048
DEFAULT_MAX_N_DAC = 65536
DEFAULT_TOL_HZ = 1.0e6  # 1 MHz default tolerance


def _position_for(pulse_len_dac: int, period_dac: int, adc_ratio: int) -> int:
    """Center the pulse in its slot, rounded down to an ADC-grid multiple."""
    raw = (period_dac - pulse_len_dac) // 2
    return max(0, (raw // adc_ratio) * adc_ratio)


def select_frequency_aware_period(
    target_hz: float,
    fs_dac: float = DEFAULT_FS_DAC,
    adc_ratio: int = DEFAULT_ADC_RATIO,
    edge_dac: int = DEFAULT_EDGE_DAC,
    top_dac: int = DEFAULT_TOP_DAC,
    min_period_dac: int = DEFAULT_MIN_PERIOD_DAC,
    max_period_dac: int = DEFAULT_MAX_PERIOD_DAC,
    max_n_dac: int = DEFAULT_MAX_N_DAC,
    tol_hz: float = DEFAULT_TOL_HZ,
    coherence_limit: float = 0.3,
    amplitude_lsb: float = DEFAULT_AMPLITUDE_LSB,
    seed: int = DEFAULT_SEED,
) -> list[dict]:
    """Return valid DitherConfig candidates sorted by frequency error."""
    pulse_len_dac = 2 * edge_dac + top_dac
    candidates: list[dict] = []

    for n_dac in range(256, max_n_dac + 1, 256):
        # Coherent tone bin closest to the target.
        k = int(round(target_hz * n_dac / fs_dac))
        if k <= 0 or k >= n_dac // 2:
            continue
        actual_hz = k * fs_dac / n_dac
        if abs(actual_hz - target_hz) > tol_hz:
            continue

        for period_dac in range(min_period_dac, max_period_dac + 1, adc_ratio):
            if n_dac % period_dac != 0:
                continue
            events = n_dac // period_dac
            if events % 2 != 0:
                continue
            if pulse_len_dac > period_dac:
                continue
            position_dac = _position_for(pulse_len_dac, period_dac, adc_ratio)
            if position_dac + pulse_len_dac > period_dac:
                continue

            try:
                cfg = DitherConfig(
                    fs_dac=fs_dac,
                    n_dac_points=n_dac,
                    adc_ratio=adc_ratio,
                    sig_cycles=k,
                    amp_dbfs=-1.5,
                    dither_period_dac=period_dac,
                    dither_position_dac=position_dac,
                    dither_edge_dac=edge_dac,
                    dither_top_dac=top_dac,
                    dither_scale_lsb=amplitude_lsb,
                    seed=seed,
                )
                cfg.validate()
            except ValueError:
                continue

            coherence = cfg.tone_phase_coherence()
            if coherence >= coherence_limit:
                continue

            candidates.append({
                "target_hz": target_hz,
                "actual_hz": actual_hz,
                "freq_error_hz": actual_hz - target_hz,
                "n_dac_points": n_dac,
                "dither_period_dac": period_dac,
                "dither_position_dac": position_dac,
                "dither_edge_dac": edge_dac,
                "dither_top_dac": top_dac,
                "events_per_loop": events,
                "slot_period_adc": period_dac // adc_ratio,
                "pulse_len_dac": pulse_len_dac,
                "duty_cycle": pulse_len_dac / period_dac,
                "cycles_per_event": k * period_dac / n_dac,
                "coherence": coherence,
                "sig_cycles": k,
            })

    candidates.sort(key=lambda c: (abs(c["freq_error_hz"]), c["duty_cycle"]))
    return candidates


def build_cfg(candidate: dict) -> DitherConfig:
    return DitherConfig(
        fs_dac=DEFAULT_FS_DAC,
        n_dac_points=candidate["n_dac_points"],
        adc_ratio=DEFAULT_ADC_RATIO,
        sig_cycles=candidate["sig_cycles"],
        amp_dbfs=-1.5,
        dither_period_dac=candidate["dither_period_dac"],
        dither_position_dac=candidate["dither_position_dac"],
        dither_edge_dac=candidate["dither_edge_dac"],
        dither_top_dac=candidate["dither_top_dac"],
        dither_scale_lsb=DEFAULT_AMPLITUDE_LSB,
        seed=DEFAULT_SEED,
    )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--target-mhz", type=float, required=True)
    parser.add_argument("--tol-hz", type=float, default=DEFAULT_TOL_HZ)
    parser.add_argument("--min-period-dac", type=int, default=DEFAULT_MIN_PERIOD_DAC)
    parser.add_argument("--max-period-dac", type=int, default=DEFAULT_MAX_PERIOD_DAC)
    parser.add_argument("--max-n-dac", type=int, default=DEFAULT_MAX_N_DAC)
    parser.add_argument("--edge-dac", type=int, default=DEFAULT_EDGE_DAC)
    parser.add_argument("--top-dac", type=int, default=DEFAULT_TOP_DAC)
    parser.add_argument("--amplitude-lsb", type=float, default=DEFAULT_AMPLITUDE_LSB)
    parser.add_argument("--out", help="If set, generate the best waveform here")
    parser.add_argument("--top", type=int, default=5, help="Show top N candidates")
    args = parser.parse_args(argv)

    target_hz = args.target_mhz * 1.0e6
    candidates = select_frequency_aware_period(
        target_hz=target_hz,
        edge_dac=args.edge_dac,
        top_dac=args.top_dac,
        min_period_dac=args.min_period_dac,
        max_period_dac=args.max_period_dac,
        max_n_dac=args.max_n_dac,
        tol_hz=args.tol_hz,
        amplitude_lsb=args.amplitude_lsb,
    )

    if not candidates:
        print("No valid configuration found for the requested target.")
        return

    print(f"Found {len(candidates)} valid configurations "
          f"(showing top {min(args.top, len(candidates))}):\n")
    print(f"{'#':>2} {'actual MHz':>12} {'err Hz':>10} {'N':>7} "
          f"{'period':>6} {'pos':>5} {'events':>6} {'duty':>7} "
          f"{'cyc/ev':>8} {'coh':>6}")
    for i, c in enumerate(candidates[: args.top], 1):
        print(f"{i:>2} {c['actual_hz'] / 1e6:12.6f} {c['freq_error_hz']:10.1f} "
              f"{c['n_dac_points']:7d} {c['dither_period_dac']:6d} "
              f"{c['dither_position_dac']:5d} {c['events_per_loop']:6d} "
              f"{c['duty_cycle']:7.4f} {c['cycles_per_event']:8.4f} "
              f"{c['coherence']:6.4f}")

    if args.out:
        best = candidates[0]
        cfg = build_cfg(best)
        out_dir = Path(args.out)
        out_dir.mkdir(parents=True, exist_ok=True)
        stem = f"freq_aware_{int(round(args.target_mhz * 10)) // 10}MHz"
        result = write_dac_files(cfg, out_dir, stem=stem)
        print(f"\nGenerated waveform: {result['txt']}")
        print(f"Metadata: {result['json']}")

        manifest = {"target_hz": target_hz, "best": best}
        manifest_path = out_dir / "frequency_aware_manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
