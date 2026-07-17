import numpy as np



RECONSTRUCTION_MODES = (
    "grouped_halves_interleave",
    "grouped_halves_interleave_reversed",
    "grouped_halves_swapped",
    "grouped_halves_swapped_reversed",
    "already_interleaved_even_pos",
    "already_interleaved_odd_pos",
    "raw_order",
    "raw_order_negated",
    "reverse_each_group",
    "swap_16bit_pairs",
    "swap_32bit_words",
)


def _decode_dma_words(raw_bytes, remove_trailing_words=8):
    """
    Convert raw UDP/DMA bytes into signed 14-bit words without reordering.
    """
    raw = np.frombuffer(raw_bytes, dtype=np.uint8)

    if raw.size % 2:
        raw = raw[:-1]

    if raw.size < 2:
        raise ValueError("DMA frame contains too few bytes.")

    # DMA bytes are sent unchanged by ethernet.c, so interpret each 16-bit
    # word as little-endian, then drop the two unused LSBs.
    words = raw.view("<i2")
    words = (words >> 2).astype(np.int32)

    if remove_trailing_words:
        if words.size <= remove_trailing_words:
            raise ValueError(
                "DMA frame is too short after removing trailing words."
            )
        words = words[:-remove_trailing_words]

    usable = words.size - (words.size % 8)

    if usable <= 0:
        raise ValueError(
            "DMA frame does not contain a complete 8-word group."
        )

    return words[:usable].reshape(-1, 8)


def reconstruct_adc_bytes(
    raw_bytes,
    remove_trailing_words=8,
    mode="grouped_halves_interleave",
):
    """
    Reconstruct one ADC sample sequence using a selectable packing mode.

    The detector can test all modes in RECONSTRUCTION_MODES and select the
    mode with the highest DAC-reference correlation.
    """
    groups = _decode_dma_words(
        raw_bytes,
        remove_trailing_words=remove_trailing_words,
    )

    if mode == "grouped_halves_interleave":
        pos = groups[:, :4].reshape(-1)
        neg = (-groups[:, 4:]).reshape(-1)
        adc = np.empty(pos.size + neg.size, dtype=np.int32)
        adc[0::2] = pos
        adc[1::2] = neg
        return adc

    if mode == "grouped_halves_interleave_reversed":
        pos = groups[:, :4][:, ::-1].reshape(-1)
        neg = (-groups[:, 4:][:, ::-1]).reshape(-1)
        adc = np.empty(pos.size + neg.size, dtype=np.int32)
        adc[0::2] = pos
        adc[1::2] = neg
        return adc

    if mode == "grouped_halves_swapped":
        pos = groups[:, 4:].reshape(-1)
        neg = (-groups[:, :4]).reshape(-1)
        adc = np.empty(pos.size + neg.size, dtype=np.int32)
        adc[0::2] = pos
        adc[1::2] = neg
        return adc

    if mode == "grouped_halves_swapped_reversed":
        pos = groups[:, 4:][:, ::-1].reshape(-1)
        neg = (-groups[:, :4][:, ::-1]).reshape(-1)
        adc = np.empty(pos.size + neg.size, dtype=np.int32)
        adc[0::2] = pos
        adc[1::2] = neg
        return adc

    if mode == "already_interleaved_even_pos":
        ordered = groups.reshape(-1).copy()
        ordered[1::2] *= -1
        return ordered.astype(np.int32)

    if mode == "already_interleaved_odd_pos":
        ordered = groups.reshape(-1).copy()
        ordered[0::2] *= -1
        return ordered.astype(np.int32)

    if mode == "raw_order":
        return groups.reshape(-1).astype(np.int32)

    if mode == "raw_order_negated":
        return (-groups.reshape(-1)).astype(np.int32)

    if mode == "reverse_each_group":
        return groups[:, ::-1].reshape(-1).astype(np.int32)

    if mode == "swap_16bit_pairs":
        # [0,1,2,3,4,5,6,7] -> [1,0,3,2,5,4,7,6]
        swapped = groups[:, [1, 0, 3, 2, 5, 4, 7, 6]]
        return swapped.reshape(-1).astype(np.int32)

    if mode == "swap_32bit_words":
        # Swap adjacent two-word blocks:
        # [0,1,2,3,4,5,6,7] -> [2,3,0,1,6,7,4,5]
        swapped = groups[:, [2, 3, 0, 1, 6, 7, 4, 5]]
        return swapped.reshape(-1).astype(np.int32)

    raise ValueError(
        f"Unknown reconstruction mode: {mode}. "
        f"Available modes: {', '.join(RECONSTRUCTION_MODES)}"
    )


def rank_reconstruction_modes(
    raw_frames,
    reference_period,
    max_test_frames=5,
):
    """
    Test common DMA packing assumptions and rank them by reference match.

    The score is based primarily on mean normalized correlation across the
    selected frames. RMSE is used as a secondary diagnostic.

    Returns
    -------
    ranking : list[dict]
        Sorted best-first.
    """
    if not raw_frames:
        raise ValueError("No raw frames were supplied for mode detection.")

    test_frames = raw_frames[: max(1, min(max_test_frames, len(raw_frames)))]
    ranking = []

    for mode in RECONSTRUCTION_MODES:
        correlations = []
        rmses = []
        sample_counts = []
        failure = None

        for raw_bytes in test_frames:
            try:
                adc = reconstruct_adc_bytes(raw_bytes, mode=mode)
                result = align_adc_to_periodic_reference(
                    adc,
                    reference_period,
                )
                correlations.append(float(result["correlation"]))
                rmses.append(float(result["rmse_codes"]))
                sample_counts.append(int(adc.size))
            except Exception as exc:
                failure = str(exc)
                break

        if failure is not None or not correlations:
            ranking.append({
                "mode": mode,
                "test_frames": len(correlations),
                "mean_correlation": float("-inf"),
                "min_correlation": float("nan"),
                "max_correlation": float("nan"),
                "mean_rmse_codes": float("inf"),
                "sample_count": None,
                "status": f"FAILED: {failure}",
            })
            continue

        ranking.append({
            "mode": mode,
            "test_frames": len(correlations),
            "mean_correlation": float(np.mean(correlations)),
            "min_correlation": float(np.min(correlations)),
            "max_correlation": float(np.max(correlations)),
            "mean_rmse_codes": float(np.mean(rmses)),
            "sample_count": int(sample_counts[0]),
            "status": "OK",
        })

    ranking.sort(
        key=lambda row: (
            row["mean_correlation"],
            -row["mean_rmse_codes"],
        ),
        reverse=True,
    )

    return ranking

def estimate_circular_lag(reference, signal):
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

    # Circular cross-correlation using FFT. Positive lag means the signal is
    # shifted right relative to the reference and must be rolled left.
    corr = np.fft.ifft(np.fft.fft(sig) * np.conj(np.fft.fft(ref))).real
    peak_index = int(np.argmax(corr))
    lag = peak_index if peak_index <= n // 2 else peak_index - n
    correlation = float(corr[peak_index] / (ref_norm * sig_norm))
    return lag, correlation



def load_dac_reference_txt(filename):
    data = np.loadtxt(filename, comments="#", ndmin=1)
    if data.ndim == 2:
        data = data[:, -1]
    data = np.asarray(data, dtype=np.float64).reshape(-1)
    if data.size < 16:
        raise ValueError("The DAC reference must contain at least 16 samples.")
    if not np.all(np.isfinite(data)):
        raise ValueError("The DAC reference contains invalid values.")
    return data


def resample_periodic_dac_reference(
    dac_reference,
    dac_sample_rate_hz,
    adc_sample_rate_hz,
    expected_txt_samples=65536,
):
    """
    Build the exact repeating DAC/ADC joint reference period.

    For the fixed project rates:
        DAC = 2.4576 GSa/s
        ADC = 1.3 GSa/s

    a 65,536-sample DAC TXT file spans 34,666 2/3 ADC samples.
    Therefore, the exact joint sample-grid period is:
        3 DAC TXT repetitions = 196,608 DAC samples
        104,000 ADC samples
    """
    dac = np.asarray(dac_reference, dtype=np.float64).reshape(-1)

    if dac.size != expected_txt_samples:
        raise ValueError(
            f"Expected exactly {expected_txt_samples} DAC samples, "
            f"but the TXT file contains {dac.size}."
        )
    if dac_sample_rate_hz <= 0 or adc_sample_rate_hz <= 0:
        raise ValueError("DAC and ADC sample rates must be positive.")

    # Exact reduced sample-rate ratio for the fixed project clocks:
    # ADC / DAC = 1.3e9 / 2.4576e9 = 1625 / 3072.
    from math import gcd

    dac_rate_int = int(round(dac_sample_rate_hz))
    adc_rate_int = int(round(adc_sample_rate_hz))
    divisor = gcd(dac_rate_int, adc_rate_int)

    adc_ratio_num = adc_rate_int // divisor
    dac_ratio_den = dac_rate_int // divisor

    # Smallest number of TXT repetitions that gives an integer ADC sample count.
    repetitions = dac_ratio_den // gcd(expected_txt_samples, dac_ratio_den)
    joint_dac_samples = expected_txt_samples * repetitions

    numerator = joint_dac_samples * adc_ratio_num
    if numerator % dac_ratio_den != 0:
        raise ValueError(
            "The DAC/ADC sample-rate relationship did not produce an exact "
            "joint period. Check the configured sample rates."
        )

    joint_adc_samples = numerator // dac_ratio_den

    repeated_dac = np.tile(dac, repetitions)

    # ADC-domain sample positions expressed in DAC-sample units.
    positions = (
        np.arange(joint_adc_samples, dtype=np.float64)
        * float(dac_sample_rate_hz)
        / float(adc_sample_rate_hz)
    )
    positions %= joint_dac_samples

    # Periodic interpolation over the full joint period.
    xp = np.arange(joint_dac_samples + 1, dtype=np.float64)
    fp = np.concatenate((repeated_dac, repeated_dac[:1]))
    adc_joint_period = np.interp(positions, xp, fp)

    metadata = {
        "txt_samples": int(expected_txt_samples),
        "dac_repetitions": int(repetitions),
        "joint_dac_samples": int(joint_dac_samples),
        "joint_adc_samples": int(joint_adc_samples),
        "dac_sample_rate_hz": float(dac_sample_rate_hz),
        "adc_sample_rate_hz": float(adc_sample_rate_hz),
    }

    return adc_joint_period, metadata



def _normalized_signal(x):
    """Return zero-mean, unit-norm signal and its original mean."""
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    mean = float(np.mean(x))
    centered = x - mean
    norm = float(np.linalg.norm(centered))

    if norm <= 0:
        raise ValueError("Cannot normalize a constant waveform.")

    return centered / norm, mean


def align_adc_to_periodic_reference(adc, reference_period):
    """
    Align one ADC capture to a periodic DAC reference.

    Timing is estimated only from normalized correlation, independent of
    amplitude and DC offset. After the best timing position is found, gain and
    offset are estimated with least squares:

        adc[n] ~= gain * reference[n] + offset

    Returns the aligned raw reference, normalized waveforms, fitted ADC
    waveform, residual error, correlation, gain, offset, and RMSE.
    """
    adc = np.asarray(adc, dtype=np.float64).reshape(-1)
    period = np.asarray(reference_period, dtype=np.float64).reshape(-1)

    n = adc.size
    p = period.size

    if n < 16:
        raise ValueError("ADC capture is too short for timing alignment.")
    if p < 16:
        raise ValueError("Reference period is too short for timing alignment.")

    adc_normalized, adc_mean = _normalized_signal(adc)

    # Build enough repeated reference samples so every possible start index
    # within one complete joint period has an n-sample candidate window.
    repeats = int(np.ceil((p + n - 1) / p))
    extended = np.tile(period, repeats)[: p + n - 1]

    # Sliding window sums and energies for normalized correlation.
    csum = np.concatenate(([0.0], np.cumsum(extended)))
    csum2 = np.concatenate(([0.0], np.cumsum(extended * extended)))

    starts = np.arange(p, dtype=np.int64)
    window_sum = csum[starts + n] - csum[starts]
    window_sum2 = csum2[starts + n] - csum2[starts]

    window_mean = window_sum / n
    window_energy = window_sum2 - (window_sum * window_sum) / n
    window_energy = np.maximum(window_energy, 0.0)

    # Dot product between every candidate window and the zero-mean ADC.
    # Since adc_normalized has zero mean, raw reference dot-products are
    # equivalent to centered-window dot-products.
    conv_len = extended.size + n - 1
    fft_len = 1 << (conv_len - 1).bit_length()

    dot_all = np.fft.irfft(
        np.fft.rfft(extended, fft_len)
        * np.fft.rfft(adc_normalized[::-1], fft_len),
        fft_len,
    )
    dots = dot_all[n - 1 : n - 1 + p]

    denom = np.sqrt(window_energy)
    correlations = np.full(p, -np.inf, dtype=np.float64)
    valid = denom > 0
    correlations[valid] = dots[valid] / denom[valid]

    start_index = int(np.argmax(correlations))
    correlation = float(correlations[start_index])

    aligned_reference = extended[start_index : start_index + n].copy()
    reference_mean = float(window_mean[start_index])
    reference_centered = aligned_reference - reference_mean
    reference_norm = float(np.linalg.norm(reference_centered))

    if reference_norm <= 0:
        raise ValueError("Best reference window is constant.")

    reference_normalized = reference_centered / reference_norm

    # Estimate gain and offset only after timing is fixed.
    design = np.column_stack(
        (aligned_reference, np.ones(n, dtype=np.float64))
    )
    gain, offset = np.linalg.lstsq(design, adc, rcond=None)[0]

    fitted_adc = gain * aligned_reference + offset
    residual = adc - fitted_adc
    rmse = float(np.sqrt(np.mean(residual * residual)))

    # Common normalized representation for plotting overlap.
    adc_zscore = (adc - adc_mean) / float(np.std(adc))
    ref_std = float(np.std(aligned_reference))
    if ref_std <= 0:
        raise ValueError("Aligned reference has zero standard deviation.")
    reference_zscore = (
        aligned_reference - np.mean(aligned_reference)
    ) / ref_std

    return {
        "reference_start_index": start_index,
        "reference_period_samples": int(p),
        "correlation": correlation,
        "gain": float(gain),
        "offset": float(offset),
        "rmse_codes": rmse,
        "aligned_reference": aligned_reference,
        "reference_normalized": reference_normalized,
        "adc_normalized": adc_normalized,
        "reference_zscore": reference_zscore,
        "adc_zscore": adc_zscore,
        "fitted_adc": fitted_adc,
        "residual_error": residual,
    }


def align_frame_to_reference(reference, signal):
    """
    Align one ADC frame to a reference ADC frame using integer circular lag.

    Returns a dictionary containing the detected lag, normalized correlation,
    aligned signal, fitted gain/offset, and residual RMSE.
    """
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
    rmse = float(np.sqrt(np.mean(residual ** 2)))

    ref_std = float(np.std(reference))
    sig_std = float(np.std(aligned))
    if ref_std <= 0 or sig_std <= 0:
        raise ValueError("Cannot normalize a constant ADC frame.")

    return {
        "lag_samples": int(lag),
        "correlation": float(correlation),
        "aligned_adc": aligned,
        "reference_zscore": (reference - np.mean(reference)) / ref_std,
        "aligned_zscore": (aligned - np.mean(aligned)) / sig_std,
        "fitted_gain_to_frame1": float(gain),
        "fitted_offset_to_frame1": float(offset),
        "rmse_codes": rmse,
        "residual_error": residual,
    }
