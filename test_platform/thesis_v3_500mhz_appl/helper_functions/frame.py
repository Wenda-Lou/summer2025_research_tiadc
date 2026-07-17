"""ADC frame reconstruction, alignment, and reference construction utilities."""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np


SELECTED_RECONSTRUCTION_MODE = "grouped_halves_interleave"


def reconstruct_adc_bytes(
    raw_bytes: bytes,
    remove_trailing_words: int = 8,
    mode: str = SELECTED_RECONSTRUCTION_MODE,
) -> np.ndarray:
    """Convert one DMA frame into the reconstructed ADC sample sequence."""
    raw = np.frombuffer(raw_bytes, dtype=np.uint8)
    if raw.size % 2:
        raw = raw[:-1]
    if raw.size < 2:
        raise ValueError("DMA frame contains too few bytes.")

    samples = raw.view("<i2")
    samples = (samples >> 2).astype(np.int16)

    if remove_trailing_words:
        if samples.size <= remove_trailing_words:
            raise ValueError("DMA frame is too short after removing trailing words.")
        samples = samples[:-remove_trailing_words]

    usable = samples.size - (samples.size % 8)
    if usable <= 0:
        raise ValueError("DMA frame does not contain a complete 8-word group.")
    samples = samples[:usable]

    if mode == "grouped_halves_interleave":
        words = samples.reshape(-1, 8)
        pos = words[:, :4].reshape(-1).astype(np.int32)
        neg = (-words[:, 4:]).reshape(-1).astype(np.int32)
        adc = np.empty(pos.size + neg.size, dtype=np.int32)
        adc[0::2] = pos
        adc[1::2] = neg
        return adc

    if mode == "raw_order":
        return samples.astype(np.int32)

    raise ValueError(f"Unsupported reconstruction mode: {mode}")


def estimate_circular_lag(reference: np.ndarray, signal: np.ndarray) -> Tuple[int, float]:
    """Return integer lag and normalized correlation for two equal-length frames."""
    n = min(len(reference), len(signal))
    if n < 16:
        raise ValueError("Not enough samples for timing alignment.")

    ref = np.asarray(reference[:n], dtype=np.float64)
    sig = np.asarray(signal[:n], dtype=np.float64)
    ref -= np.mean(ref)
    sig -= np.mean(sig)

    ref_norm = np.linalg.norm(ref)
    sig_norm = np.linalg.norm(sig)
    if ref_norm == 0 or sig_norm == 0:
        raise ValueError("Cannot align a constant waveform.")

    corr = np.fft.ifft(np.fft.fft(sig) * np.conj(np.fft.fft(ref))).real
    peak_index = int(np.argmax(corr))
    lag = peak_index if peak_index <= n // 2 else peak_index - n
    correlation = float(corr[peak_index] / (ref_norm * sig_norm))
    return lag, correlation


def align_frame_to_reference(reference: np.ndarray, signal: np.ndarray) -> Dict[str, object]:
    """Align an ADC frame to frame 1 using integer circular lag."""
    reference = np.asarray(reference, dtype=np.float64).reshape(-1)
    signal = np.asarray(signal, dtype=np.float64).reshape(-1)
    n = min(reference.size, signal.size)
    if n < 16:
        raise ValueError("Not enough samples for frame-to-frame alignment.")

    reference = reference[:n]
    signal = signal[:n]
    lag, correlation = estimate_circular_lag(reference, signal)
    aligned = np.roll(signal, -lag)

    design = np.column_stack((reference, np.ones(n, dtype=np.float64)))
    gain, offset = np.linalg.lstsq(design, aligned, rcond=None)[0]
    fitted = gain * reference + offset
    residual = aligned - fitted
    rmse = float(np.sqrt(np.mean(residual**2)))

    ref_std = float(np.std(reference))
    aligned_std = float(np.std(aligned))
    if ref_std <= 0 or aligned_std <= 0:
        raise ValueError("Cannot normalize a constant ADC frame.")

    return {
        "lag_samples": int(lag),
        "correlation": float(correlation),
        "aligned_adc": aligned,
        "reference_zscore": (reference - np.mean(reference)) / ref_std,
        "aligned_zscore": (aligned - np.mean(aligned)) / aligned_std,
        "fitted_gain_to_frame1": float(gain),
        "fitted_offset_to_frame1": float(offset),
        "rmse_codes": rmse,
        "residual_error": residual,
    }


def load_dac_reference_txt(filename: str) -> np.ndarray:
    """Load the waveform values from a DPG TXT file."""
    data = np.loadtxt(filename, comments="#", ndmin=1)
    if data.ndim == 2:
        data = data[:, -1]
    data = np.asarray(data, dtype=np.float64).reshape(-1)
    if data.size < 16:
        raise ValueError("The DAC reference must contain at least 16 samples.")
    if not np.all(np.isfinite(data)):
        raise ValueError("The DAC reference contains invalid values.")
    return data


def estimate_tone_phase(
    frame1: np.ndarray,
    tone_frequency_hz: float,
    adc_sample_rate_hz: float,
) -> Dict[str, float]:
    """Fit DC + cosine + sine to frame 1 and return amplitude and phase."""
    y = np.asarray(frame1, dtype=np.float64).reshape(-1)
    if y.size < 16:
        raise ValueError("Frame 1 is too short for sine fitting.")
    if not (0 < tone_frequency_hz < adc_sample_rate_hz / 2):
        raise ValueError("tone_frequency_hz must be between DC and Nyquist.")

    n = np.arange(y.size, dtype=np.float64)
    omega = 2.0 * np.pi * tone_frequency_hz / adc_sample_rate_hz
    design = np.column_stack((np.cos(omega * n), np.sin(omega * n), np.ones(y.size)))
    cosine_coeff, sine_coeff, offset = np.linalg.lstsq(design, y, rcond=None)[0]

    amplitude = float(np.hypot(cosine_coeff, sine_coeff))
    phase = float(np.arctan2(-sine_coeff, cosine_coeff))
    fitted = amplitude * np.cos(omega * n + phase) + offset
    rmse = float(np.sqrt(np.mean((y - fitted) ** 2)))

    return {
        "amplitude_codes": amplitude,
        "phase_radians": phase,
        "offset_codes": float(offset),
        "fit_rmse_codes": rmse,
    }


def build_adc_grid_reference_from_frame1(
    frame1: np.ndarray,
    tone_frequency_hz: float,
    adc_sample_rate_hz: float,
    target_amplitude_codes: float | None = None,
    target_offset_codes: float = 0.0,
) -> Tuple[np.ndarray, Dict[str, float]]:
    """
    Build the fixed calibration reference on the ADC sample grid.

    The TXT supplies the known tone frequency. Frame 1 supplies the phase.
    A configured target amplitude may replace the measured frame-1 amplitude.
    """
    fit = estimate_tone_phase(frame1, tone_frequency_hz, adc_sample_rate_hz)
    amplitude = fit["amplitude_codes"] if target_amplitude_codes is None else float(target_amplitude_codes)

    n = np.arange(np.asarray(frame1).size, dtype=np.float64)
    omega = 2.0 * np.pi * tone_frequency_hz / adc_sample_rate_hz
    reference = amplitude * np.cos(omega * n + fit["phase_radians"]) + float(target_offset_codes)

    metadata = dict(fit)
    metadata["reference_amplitude_codes"] = float(amplitude)
    metadata["reference_offset_codes"] = float(target_offset_codes)
    metadata["tone_frequency_hz"] = float(tone_frequency_hz)
    metadata["adc_sample_rate_hz"] = float(adc_sample_rate_hz)
    return reference, metadata
