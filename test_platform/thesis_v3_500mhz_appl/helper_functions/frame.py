import numpy as np


def reconstruct_adc_bytes(raw_bytes, remove_trailing_words=8):
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

    words = samples.reshape(-1, 8)
    pos = words[:, :4].reshape(-1).astype(np.int32)
    neg = (-words[:, 4:]).reshape(-1).astype(np.int32)

    adc = np.empty(pos.size + neg.size, dtype=np.int32)
    adc[0::2] = pos
    adc[1::2] = neg
    return adc


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


def align_adc_to_periodic_reference(adc, reference_period):
    adc = np.asarray(adc, dtype=np.float64).reshape(-1)
    period = np.asarray(reference_period, dtype=np.float64).reshape(-1)

    n = adc.size
    p = period.size
    if n < 16 or p < 16:
        raise ValueError("ADC capture or reference is too short.")

    adc_centered = adc - np.mean(adc)
    adc_energy = float(np.dot(adc_centered, adc_centered))
    if adc_energy <= 0:
        raise ValueError("Cannot align a constant ADC capture.")

    repeats = int(np.ceil((p + n - 1) / p))
    extended = np.tile(period, repeats)[:p + n - 1]

    conv_len = extended.size + n - 1
    fft_len = 1 << (conv_len - 1).bit_length()
    dot_all = np.fft.irfft(
        np.fft.rfft(extended, fft_len)
        * np.fft.rfft(adc_centered[::-1], fft_len),
        fft_len,
    )
    dots = dot_all[n - 1:n - 1 + p]

    csum = np.concatenate(([0.0], np.cumsum(extended)))
    csum2 = np.concatenate(([0.0], np.cumsum(extended * extended)))
    starts = np.arange(p)
    sums = csum[starts + n] - csum[starts]
    sums2 = csum2[starts + n] - csum2[starts]
    energies = np.maximum(sums2 - sums * sums / n, 0.0)

    denom = np.sqrt(adc_energy * energies)
    corr = np.full(p, -np.inf)
    valid = denom > 0
    corr[valid] = dots[valid] / denom[valid]

    start_index = int(np.argmax(corr))
    aligned_ref = extended[start_index:start_index + n].copy()

    design = np.column_stack((aligned_ref, np.ones(n)))
    scale, offset = np.linalg.lstsq(design, adc, rcond=None)[0]
    fitted = scale * aligned_ref + offset
    error = adc - fitted

    return {
        "reference_start_index": start_index,
        "reference_period_samples": int(p),
        "correlation": float(corr[start_index]),
        "scale": float(scale),
        "offset": float(offset),
        "rmse_codes": float(np.sqrt(np.mean(error ** 2))),
        "aligned_reference": aligned_ref,
        "fitted_adc": fitted,
        "error": error,
    }
