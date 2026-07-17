"""Calibration error equations and coefficient update helpers."""

from __future__ import annotations

from typing import Dict

import numpy as np


def calculate_calibration_error(aligned_adc: np.ndarray, reference: np.ndarray) -> Dict[str, object]:
    """Compare one timing-aligned ADC frame with the fixed expected reference."""
    adc = np.asarray(aligned_adc, dtype=np.float64).reshape(-1)
    ref = np.asarray(reference, dtype=np.float64).reshape(-1)
    n = min(adc.size, ref.size)
    if n < 16:
        raise ValueError("Not enough samples for calibration error calculation.")
    adc = adc[:n]
    ref = ref[:n]

    ref_centered = ref - np.mean(ref)
    denominator = float(np.dot(ref_centered, ref_centered))
    if denominator <= 0:
        raise ValueError("Calibration reference is constant.")

    gain = float(np.dot(ref_centered, adc - np.mean(adc)) / denominator)
    offset = float(np.mean(adc) - gain * np.mean(ref))
    predicted_adc = gain * ref + offset
    model_residual = adc - predicted_adc
    absolute_error = adc - ref

    return {
        "measured_gain": gain,
        "measured_offset_codes": offset,
        "gain_error": gain - 1.0,
        "offset_error_codes": float(np.mean(absolute_error)),
        "rmse_to_reference_codes": float(np.sqrt(np.mean(absolute_error**2))),
        "model_residual_rmse_codes": float(np.sqrt(np.mean(model_residual**2))),
        "absolute_error": absolute_error,
        "model_residual": model_residual,
        "predicted_adc": predicted_adc,
    }


def update_offset_coefficient(current_offset: float, offset_error: float, learning_rate: float = 0.25) -> float:
    """Return the next additive correction coefficient."""
    if not 0 < learning_rate <= 1:
        raise ValueError("learning_rate must be in (0, 1].")
    return float(current_offset - learning_rate * offset_error)


def update_gain_coefficient(current_gain: float, measured_gain: float, learning_rate: float = 0.25) -> float:
    """Return the next multiplicative correction coefficient."""
    if not 0 < learning_rate <= 1:
        raise ValueError("learning_rate must be in (0, 1].")
    if measured_gain == 0:
        raise ValueError("measured_gain cannot be zero.")
    target = current_gain / measured_gain
    return float(current_gain + learning_rate * (target - current_gain))
