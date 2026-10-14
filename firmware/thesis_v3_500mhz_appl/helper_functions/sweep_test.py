"""Receive and analyse an AD9695 input-full-scale (IFC) sweep.

The board command ``adc -gain`` -> ``IFC`` -> ``sweep`` holds the injected
signal constant, selects each supported AD9695 full-scale range, and sends one
raw DMA capture per range.  This module verifies the property the experiment
is meant to measure:

    fitted_peak_codes * selected_full_scale_vpp = constant

When the actual differential input voltage is supplied, it also estimates the
absolute ADC full-scale voltage and volts per code for each setting.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import sys

import numpy as np
import pandas as pd

from .receive_data import receive_adc_data
from .frame import ADC_VALID_SAMPLE_COUNT as ADC_VALID_WORD_COUNT


# ``udp_receiver.py`` is commonly launched from the firmware application
# directory, which does not put the repository root on sys.path. Import the
# production de-framer explicitly so this diagnostic follows the same
# [A A A A B B B B] boundary rule as the calibration loop.
REPO_DIR = Path(__file__).resolve().parents[3]
if str(REPO_DIR) not in sys.path:
    sys.path.insert(0, str(REPO_DIR))

from calibration_loop.estimator import (  # noqa: E402
    deframe,
    fit_tone,
    split_at,
    unpack_words,
)


PROJECT_DIR = Path(__file__).resolve().parents[1]
SAVE_DIR = PROJECT_DIR / "adc_data"
SAVE_DIR.mkdir(parents=True, exist_ok=True)

ADC_SAMPLE_RATE_HZ = 1.3e9
ADC_CODE_COUNT = 2 ** 14
ADC_PEAK_CODES = ADC_CODE_COUNT / 2.0
ADC_MIN_CODE = -8192
ADC_MAX_CODE = 8191

IFC_VALUES_VPP = (2.04, 1.93, 1.81, 1.70, 1.59, 1.47, 1.36)


def _normalise_driven_channel(driven_channel: str | None) -> str | None:
    if driven_channel is None:
        return None
    channel = str(driven_channel).strip().upper()
    if channel not in {"A", "B"}:
        raise ValueError("Driven channel must be 'A', 'B', or None.")
    return channel


def deframe_single_tone(
    words: np.ndarray,
    f0: float,
    driven_channel: str,
    group: int = 8,
    half: int = 4,
    minimum_isolation_db: float = 6.0,
) -> dict:
    """De-frame a capture in which only one physical ADC has the test tone.

    The normal two-channel de-framer uses A/B correlation and therefore cannot
    identify the group boundary when the other ADC has no matching waveform.
    Here the selected physical channel supplies the missing orientation: the
    correct ``[A A A A B B B B]`` boundary is the rotation that concentrates
    the known-frequency tone in the selected half and rejects it from the idle
    half.  Requiring tone isolation prevents a low-SNR or accidentally
    dual-driven capture from being silently assigned to the wrong channel.
    """
    channel = _normalise_driven_channel(driven_channel)
    if channel is None:
        raise ValueError("Single-channel de-framing requires channel A or B.")
    if not 0.0 < f0 < 0.5:
        raise ValueError("Tone frequency must be between DC and Nyquist.")

    best = None
    epsilon = np.finfo(np.float64).eps
    for rotation in range(group):
        try:
            candidate = split_at(words, rotation, group, half)
        except ValueError:
            continue

        # Do not refine during boundary selection.  Both halves must be tested
        # at the exact frequency entered by the user; otherwise idle-channel
        # noise can win by fitting a nearby unrelated component.
        amplitude_a = abs(float(
            fit_tone(candidate["ch_a"], f0, refine=False)["amplitude"]
        ))
        amplitude_b = abs(float(
            fit_tone(candidate["ch_b"], f0, refine=False)["amplitude"]
        ))
        driven_amplitude = amplitude_a if channel == "A" else amplitude_b
        idle_amplitude = amplitude_b if channel == "A" else amplitude_a
        isolation_db = 20.0 * np.log10(
            (driven_amplitude + epsilon) / (idle_amplitude + epsilon)
        )

        candidate.update({
            "tone_a_peak_codes_boundary": amplitude_a,
            "tone_b_peak_codes_boundary": amplitude_b,
            "tone_isolation_db": float(isolation_db),
            "score": float(isolation_db),
        })
        if best is None or candidate["score"] > best["score"]:
            best = candidate

    if best is None:
        raise ValueError("Capture too short to de-frame.")
    if best["tone_isolation_db"] < minimum_isolation_db:
        raise ValueError(
            f"Cannot identify Channel {channel}: tone isolation is only "
            f"{best['tone_isolation_db']:.2f} dB; require at least "
            f"{minimum_isolation_db:.2f} dB. Check that only Channel "
            f"{channel} is driven and that the entered frequency is correct."
        )

    best["driven_channel"] = channel
    best["lagged"] = False
    return best


def analyse_ifc_frame(
    raw_bytes: bytes,
    ifc_vpp: float,
    tone_frequency_mhz: float,
    input_vpp_diff: float | None,
    sample_rate_hz: float = ADC_SAMPLE_RATE_HZ,
    driven_channel: str | None = None,
):
    """Return tone/full-scale metrics and the two correctly de-framed channels."""
    if ifc_vpp <= 0.0:
        raise ValueError("IFC full-scale voltage must be positive.")
    if sample_rate_hz <= 0.0:
        raise ValueError("ADC sample rate must be positive.")
    if input_vpp_diff is not None and input_vpp_diff <= 0.0:
        raise ValueError("Measured differential input Vpp must be positive.")

    tone_hz = tone_frequency_mhz * 1e6
    f0 = tone_hz / sample_rate_hz
    if not 0.0 < f0 < 0.5:
        raise ValueError(
            f"Tone must be between 0 and Nyquist; got {tone_frequency_mhz} MHz."
        )

    driven_channel = _normalise_driven_channel(driven_channel)
    words = unpack_words(raw_bytes)
    # The UDP transport carries the complete 4096-byte DDR region, while the
    # firmware's production adc_frame.c intentionally consumes only the first
    # 2032 decoded words.  The tail is not ADC data and can retain identical
    # near-rail values across captures, so exclude it before boundary search,
    # tone fitting, min/max checks, and CSV export.
    if words.size < ADC_VALID_WORD_COUNT:
        raise ValueError(
            f"Capture contains {words.size} ADC words; expected at least "
            f"{ADC_VALID_WORD_COUNT}."
        )
    words = words[:ADC_VALID_WORD_COUNT]
    if driven_channel is None:
        view = deframe(words)
    else:
        view = deframe_single_tone(words, f0, driven_channel)
    channel_a = np.asarray(view["ch_a"], dtype=np.float64)
    channel_b = np.asarray(view["ch_b"], dtype=np.float64)

    fit_a = fit_tone(channel_a, f0, refine=True)
    fit_b = fit_tone(channel_b, f0, refine=True)
    amplitude_a = abs(float(fit_a["amplitude"]))
    amplitude_b = abs(float(fit_b["amplitude"]))
    amplitude_mean = 0.5 * (amplitude_a + amplitude_b)
    driven_amplitude = (
        amplitude_a
        if driven_channel == "A"
        else amplitude_b
        if driven_channel == "B"
        else amplitude_mean
    )
    idle_amplitude = (
        amplitude_b
        if driven_channel == "A"
        else amplitude_a
        if driven_channel == "B"
        else np.nan
    )
    if driven_amplitude <= 0.0 or (
        driven_channel is None and (amplitude_a <= 0.0 or amplitude_b <= 0.0)
    ):
        raise ValueError("Fitted tone amplitude is zero.")

    min_a = int(np.min(channel_a))
    max_a = int(np.max(channel_a))
    min_b = int(np.min(channel_b))
    max_b = int(np.max(channel_b))
    clip_samples_a = int(np.count_nonzero(
        (channel_a <= ADC_MIN_CODE) | (channel_a >= ADC_MAX_CODE)
    ))
    clip_samples_b = int(np.count_nonzero(
        (channel_b <= ADC_MIN_CODE) | (channel_b >= ADC_MAX_CODE)
    ))

    # This product should be invariant across the seven programmed ranges,
    # even when the absolute voltage at the ADC pins is not yet known.
    scale_product = driven_amplitude * ifc_vpp
    estimated_input_a = amplitude_a * ifc_vpp / ADC_PEAK_CODES
    estimated_input_b = amplitude_b * ifc_vpp / ADC_PEAK_CODES

    expected_amplitude = np.nan
    amplitude_error_pct = np.nan
    estimated_fs_a = np.nan
    estimated_fs_b = np.nan
    estimated_fs_mean = np.nan
    estimated_lsb_uv = np.nan
    range_error_pct = np.nan
    if input_vpp_diff is not None:
        expected_amplitude = ADC_PEAK_CODES * input_vpp_diff / ifc_vpp
        amplitude_error_pct = 100.0 * (
            driven_amplitude / expected_amplitude - 1.0
        )
        estimated_fs_a = input_vpp_diff * ADC_PEAK_CODES / amplitude_a
        estimated_fs_b = input_vpp_diff * ADC_PEAK_CODES / amplitude_b
        estimated_fs_mean = input_vpp_diff * ADC_PEAK_CODES / driven_amplitude
        estimated_lsb_uv = estimated_fs_mean / ADC_CODE_COUNT * 1e6
        range_error_pct = 100.0 * (estimated_fs_mean / ifc_vpp - 1.0)

    metrics = {
        "ifc_vpp": float(ifc_vpp),
        "rotation": int(view["rotation"]),
        "abs_channel_correlation": float(abs(view["corr"])),
        "sample_count_per_channel": int(channel_a.size),
        "tone_frequency_mhz": float(tone_frequency_mhz),
        "input_vpp_diff": input_vpp_diff,
        "tone_a_peak_codes": amplitude_a,
        "tone_b_peak_codes": amplitude_b,
        "tone_mean_peak_codes": amplitude_mean,
        "driven_channel": driven_channel or "BOTH",
        "driven_tone_peak_codes": driven_amplitude,
        "idle_tone_peak_codes": idle_amplitude,
        "tone_isolation_db": view.get("tone_isolation_db", np.nan),
        "gain_ratio_b_over_a": (
            amplitude_b / amplitude_a if driven_channel is None else np.nan
        ),
        "expected_peak_codes": expected_amplitude,
        "amplitude_error_pct": amplitude_error_pct,
        "scale_product_code_v": scale_product,
        "estimated_input_a_vpp": estimated_input_a,
        "estimated_input_b_vpp": estimated_input_b,
        "estimated_fs_a_vpp": estimated_fs_a,
        "estimated_fs_b_vpp": estimated_fs_b,
        "estimated_fs_mean_vpp": estimated_fs_mean,
        "estimated_lsb_uv": estimated_lsb_uv,
        "range_error_pct": range_error_pct,
        "dc_a_codes": float(fit_a["dc"]),
        "dc_b_codes": float(fit_b["dc"]),
        "min_a": min_a,
        "max_a": max_a,
        "min_b": min_b,
        "max_b": max_b,
        "clip_samples_a": clip_samples_a,
        "clip_samples_b": clip_samples_b,
        "clipped": bool(clip_samples_a or clip_samples_b),
    }
    return metrics, channel_a.astype(np.int32), channel_b.astype(np.int32)


def receive_ifc_sweep(
    bind_ip="0.0.0.0",
    port=6666,
    expected_packets=8,
    packet_size=512,
    timeout=15.0,
    reconstruct=True,
    offset_threshold_codes=None,
    tone_frequency_mhz=100.0,
    input_vpp_diff=None,
    sample_rate_hz=ADC_SAMPLE_RATE_HZ,
    scale_tolerance_pct=3.0,
    driven_channel=None,
    source_vpp_at_sma=None,
):
    """Receive seven IFC captures and verify ADC range scaling.

    ``input_vpp_diff`` must be the measured differential sine-wave Vpp at the
    ADC input. Leave it as ``None`` when only the relative range scaling is
    being checked. Set ``driven_channel`` to ``"A"`` or ``"B"`` when the
    generator is connected directly to only that input; the known tone then
    replaces A/B correlation for de-framing. ``source_vpp_at_sma`` is recorded
    as connector-level metadata and is never treated as ADC-die differential
    voltage. ``reconstruct`` and ``offset_threshold_codes`` remain in the
    signature for compatibility with older GUI callers.
    """
    del reconstruct, offset_threshold_codes

    driven_channel = _normalise_driven_channel(driven_channel)
    if not 0.0 < tone_frequency_mhz * 1e6 < sample_rate_hz / 2.0:
        raise ValueError("Tone frequency must be between 0 and ADC Nyquist.")
    if input_vpp_diff is not None and input_vpp_diff <= 0.0:
        raise ValueError("Measured differential input Vpp must be positive.")

    if source_vpp_at_sma is not None and source_vpp_at_sma <= 0.0:
        raise ValueError("Generator Vpp at the SMA must be positive.")

    mode_stem = (
        f"ifc_single_{driven_channel.lower()}_verification"
        if driven_channel is not None
        else "ifc_verification"
    )
    sweep_dir = SAVE_DIR / (
        f"{mode_stem}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    sweep_dir.mkdir(parents=True, exist_ok=True)

    results = []
    saved_files = []

    def empty_result(index, ifc, status):
        return {
            "index": index,
            "ifc_vpp": float(ifc),
            "status": status,
            "csv_file": status,
        }

    for index, ifc in enumerate(IFC_VALUES_VPP, start=1):
        print(
            f"\nWaiting for verification frame {index}/{len(IFC_VALUES_VPP)}, "
            f"IFC = {ifc:.2f} Vpp"
        )

        csv_file = receive_adc_data(
            bind_ip=bind_ip,
            port=port,
            expected_packets=expected_packets,
            packet_size=packet_size,
            timeout=timeout,
        )

        if csv_file is None:
            results.append(empty_result(index, ifc, "TIMEOUT"))
            print(
                "Stopping the receiver: without this frame, later UDP frames "
                "cannot be assigned to IFC settings safely."
            )
            break

        try:
            raw_df = pd.read_csv(csv_file)
            if "byte" not in raw_df.columns:
                raise ValueError(
                    f"Raw capture does not contain a 'byte' column: {csv_file}"
                )
            raw = raw_df["byte"].to_numpy(dtype=np.uint8)
            metrics, channel_a, channel_b = analyse_ifc_frame(
                raw.tobytes(),
                ifc_vpp=ifc,
                tone_frequency_mhz=tone_frequency_mhz,
                input_vpp_diff=input_vpp_diff,
                sample_rate_hz=sample_rate_hz,
                driven_channel=driven_channel,
            )
            metrics["source_vpp_at_sma"] = source_vpp_at_sma

            out_csv = sweep_dir / (
                f"sweep_{index:02d}_ifc_{str(ifc).replace('.', 'p')}_vpp.csv"
            )
            sample_count = min(channel_a.size, channel_b.size)
            driven_samples = (
                channel_a if driven_channel in {None, "A"} else channel_b
            )
            pd.DataFrame({
                "sample_index": np.arange(sample_count, dtype=np.int32),
                # Keep adc_code for the existing plotter. In single-channel
                # mode it follows the selected driven channel.
                "adc_code": driven_samples[:sample_count],
                "channel_a_code": channel_a[:sample_count],
                "channel_b_code": channel_b[:sample_count],
                "driven_channel": driven_channel or "BOTH",
                "tone_frequency_mhz": float(tone_frequency_mhz),
                "source_vpp_at_sma": (
                    source_vpp_at_sma
                    if source_vpp_at_sma is not None
                    else np.nan
                ),
                "ifc_vpp": float(ifc),
                "sweep_index": index,
            }).to_csv(out_csv, index=False)

            saved_files.append(out_csv)
            result = {
                "index": index,
                "status": "CAPTURED",
                **metrics,
                "csv_file": str(out_csv),
            }
            results.append(result)

            absolute_text = ""
            if input_vpp_diff is not None:
                absolute_text = (
                    f", estimated FS={metrics['estimated_fs_mean_vpp']:.4f} Vpp"
                    f", LSB={metrics['estimated_lsb_uv']:.2f} uV"
                )
            if driven_channel is None:
                frame_text = (
                    f"A={metrics['tone_a_peak_codes']:.1f}, "
                    f"B={metrics['tone_b_peak_codes']:.1f} peak codes, "
                    f"|corr|={metrics['abs_channel_correlation']:.4f}"
                )
            else:
                frame_text = (
                    f"Channel {driven_channel}="
                    f"{metrics['driven_tone_peak_codes']:.1f} peak codes, "
                    f"isolation={metrics['tone_isolation_db']:.1f} dB"
                )
            print(
                f"IFC {ifc:.2f} Vpp: {frame_text}, "
                f"clipped={metrics['clipped']}{absolute_text}"
            )

        except Exception as exc:
            print(f"Processing failed for IFC {ifc:.2f} Vpp: {exc}")
            results.append(empty_result(index, ifc, "PROCESSING_ERROR"))
        finally:
            try:
                Path(csv_file).unlink()
            except OSError:
                pass

    summary_df = pd.DataFrame(results)
    valid = summary_df["status"].eq("CAPTURED")
    if valid.any():
        reference_product = float(
            summary_df.loc[valid, "scale_product_code_v"].median()
        )
        summary_df.loc[valid, "scale_product_error_pct"] = 100.0 * (
            summary_df.loc[valid, "scale_product_code_v"] / reference_product
            - 1.0
        )
        summary_df.loc[valid, "relative_scale_pass"] = (
            summary_df.loc[valid, "scale_product_error_pct"].abs()
            <= scale_tolerance_pct
        )

        max_error = float(
            summary_df.loc[valid, "scale_product_error_pct"].abs().max()
        )
        overall = "PASS" if max_error <= scale_tolerance_pct else "FAIL"
        print(
            f"\nRelative IFC scaling: {overall}; maximum product deviation "
            f"{max_error:.2f}% (limit {scale_tolerance_pct:.2f}%)."
        )
        if input_vpp_diff is not None:
            mean_range_error = float(
                summary_df.loc[valid, "range_error_pct"].mean()
            )
            print(
                "Absolute range check: mean error "
                f"{mean_range_error:+.2f}% using {input_vpp_diff:.6f} Vpp "
                "differential at the ADC input."
            )

    # Keep the historical filename so "Open Existing IFC Sweep" continues to
    # work for both old offset-oriented runs and the new scale verification.
    summary_file = sweep_dir / "ifc_sweep_summary.csv"
    summary_df.to_csv(summary_file, index=False)

    return sweep_dir, summary_file, summary_df, saved_files
