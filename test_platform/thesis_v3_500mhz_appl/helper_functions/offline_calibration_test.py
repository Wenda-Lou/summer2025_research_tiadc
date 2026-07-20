"""Offline/PC validation scaffold; not the production FPGA calibration loop."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .calibration import calculate_calibration_error, update_gain_coefficient, update_offset_coefficient
from .frame import align_frame_to_reference, build_adc_grid_reference_from_frame1
from .receive_data import receive_adc_frame
from .reference_upload import send_reference


@dataclass
class CalibrationSettings:
    tone_frequency_hz: float = 350_000_000.0
    adc_sample_rate_hz: float = 1_450_000_000.0
    target_amplitude_codes: float | None = None
    target_offset_codes: float = 0.0
    offset_learning_rate: float = 0.25
    gain_learning_rate: float = 0.25
    offset_iterations: int = 10
    gain_iterations: int = 10


def initialize_calibration(settings: CalibrationSettings, upload_reference: bool = False):
    print("Start the prepared DAC TXT playback, then trigger one ADC capture.")
    frame1 = receive_adc_frame()
    reference, metadata = build_adc_grid_reference_from_frame1(
        frame1,
        tone_frequency_hz=settings.tone_frequency_hz,
        adc_sample_rate_hz=settings.adc_sample_rate_hz,
        target_amplitude_codes=settings.target_amplitude_codes,
        target_offset_codes=settings.target_offset_codes,
    )
    if upload_reference:
        send_reference(reference)
    return frame1, reference, metadata


def run_calibration(settings: CalibrationSettings, apply_coefficients=None):
    """
    Run offset first, then gain.

    apply_coefficients is an optional callback:
        apply_coefficients(gain_correction, offset_correction)
    It should send the new coefficients to the FPGA/ARM implementation.
    """
    frame1, reference, metadata = initialize_calibration(settings)
    gain_correction = 1.0
    offset_correction = 0.0
    history = []

    for stage, iterations in (("offset", settings.offset_iterations), ("gain", settings.gain_iterations)):
        for iteration in range(1, iterations + 1):
            print(f"Trigger ADC capture for {stage} iteration {iteration}/{iterations}.")
            current = receive_adc_frame()
            aligned = align_frame_to_reference(frame1, current)
            metrics = calculate_calibration_error(aligned["aligned_adc"], reference)

            if stage == "offset":
                offset_correction = update_offset_coefficient(
                    offset_correction,
                    metrics["offset_error_codes"],
                    settings.offset_learning_rate,
                )
            else:
                gain_correction = update_gain_coefficient(
                    gain_correction,
                    metrics["measured_gain"],
                    settings.gain_learning_rate,
                )

            if apply_coefficients is not None:
                apply_coefficients(gain_correction, offset_correction)

            history.append({
                "stage": stage,
                "iteration": iteration,
                "lag_samples": aligned["lag_samples"],
                "correlation_to_frame1": aligned["correlation"],
                "gain_correction": gain_correction,
                "offset_correction": offset_correction,
                **{k: v for k, v in metrics.items() if np.isscalar(v)},
            })
            print(history[-1])

    return history, metadata
