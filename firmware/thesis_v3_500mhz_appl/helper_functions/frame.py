"""ADC frame reconstruction utilities."""

from __future__ import annotations

import numpy as np


SELECTED_RECONSTRUCTION_MODE = "grouped_halves_interleave"

# Keep these values synchronized with adc_frame.h.
ADC_RAW_FRAME_BYTES = 4096
ADC_RAW_WORD_COUNT = 2048
ADC_TRAILING_WORDS = 8
ADC_VALID_SAMPLE_COUNT = 2032


def reconstruct_adc_bytes(
    raw_bytes: bytes,
    remove_trailing_words: int = ADC_TRAILING_WORDS,
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

    # Match adc_frame.c exactly: only the first 2032 decoded words
    # are used. A 4095-byte DMA frame still contains these words.
    if samples.size < ADC_VALID_SAMPLE_COUNT:
        raise ValueError(
            f"DMA frame is too short: found {samples.size} words, "
            f"need at least {ADC_VALID_SAMPLE_COUNT}."
        )

    samples = samples[:ADC_VALID_SAMPLE_COUNT]

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
