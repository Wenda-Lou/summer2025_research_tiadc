"""
Spectral figures of merit and time-interleaving mismatch spurs.

Captures are short (about 1 k samples per channel with the present 4095-byte DMA
buffer) and are not guaranteed coherent with the DPG loop, so everything here
uses a 7-term Blackman-Harris window and excludes a few bins around each
component.  Numbers are therefore comparable across iterations, which is what a
learning curve needs, even if they are a little pessimistic in absolute terms.
"""

from __future__ import annotations

import numpy as np

# 7-term Blackman-Harris: -180 dB sidelobes, so leakage never limits the SFDR.
_BH7 = np.array([
    0.27105140069342, -0.43329793923448, 0.21812299954311, -0.06592544638803,
    0.01081174209837, -0.00077658482522, 0.00001388721735,
])


def blackman_harris7(n: int) -> np.ndarray:
    k = np.arange(n)
    w = np.zeros(n)
    for i, c in enumerate(_BH7):
        w += c * np.cos(2.0 * np.pi * i * k / n)
    return w


def spectrum(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    """Single-sided power spectrum of a windowed record."""
    x = np.asarray(x, dtype=np.float64)
    x = x - x.mean()
    w = blackman_harris7(x.size)
    w = w / np.sqrt(np.mean(w ** 2))
    spec = np.fft.rfft(x * w)
    power = (np.abs(spec) ** 2) / (x.size ** 2)
    freq = np.fft.rfftfreq(x.size, d=1.0 / fs)
    return freq, power


def analyse(
    x: np.ndarray,
    fs: float,
    n_harmonics: int = 5,
    guard: int = 8,
    dc_bins: int = 12,
) -> dict:
    """SNDR / SFDR / ENOB / THD of one record.

    ``fs`` is the *output* rate of the stream handed in: fs_adc for a single
    channel, 2*fs_adc for the interleaved stream.
    """
    x = np.asarray(x, dtype=np.float64)
    freq, power = spectrum(x, fs)
    n_bins = power.size

    work = power.copy()
    work[:dc_bins] = 0.0

    sig_bin = int(np.argmax(work))
    sig_slice = slice(max(0, sig_bin - guard), min(n_bins, sig_bin + guard + 1))
    sig_power = float(work[sig_slice].sum())

    noise = work.copy()
    noise[sig_slice] = 0.0

    harmonic_power = 0.0
    harmonic_bins = []
    for h in range(2, n_harmonics + 1):
        # Harmonics above Nyquist fold back.
        hb = (h * sig_bin) % (2 * (n_bins - 1))
        if hb > n_bins - 1:
            hb = 2 * (n_bins - 1) - hb
        if hb < dc_bins:
            continue
        sl = slice(max(0, hb - guard), min(n_bins, hb + guard + 1))
        harmonic_power += float(noise[sl].sum())
        harmonic_bins.append(hb)

    noise_and_dist = float(noise.sum())
    spur_bin = int(np.argmax(noise))
    spur_power = float(noise[max(0, spur_bin - guard):spur_bin + guard + 1].sum())

    sndr = 10.0 * np.log10(sig_power / noise_and_dist) if noise_and_dist > 0 else np.inf
    sfdr = 10.0 * np.log10(sig_power / spur_power) if spur_power > 0 else np.inf
    thd = 10.0 * np.log10(harmonic_power / sig_power) if harmonic_power > 0 else -np.inf

    return {
        "sndr_db": float(sndr),
        "sfdr_db": float(sfdr),
        "thd_db": float(thd),
        "enob_bits": float((sndr - 1.76) / 6.02),
        "signal_hz": float(freq[sig_bin]),
        "worst_spur_hz": float(freq[spur_bin]),
        "signal_bin": sig_bin,
        "freq": freq,
        "power": power,
    }


def channel_difference_dbc(
    ch_a: np.ndarray, ch_b: np.ndarray, fs: float, f_in: float, guard: int = 8
) -> dict:
    """Residual carrier in ``a - b``, relative to the carrier in ``a``.

    This is the right figure of merit while the two channels still sample at the
    *same* instant, which is the case on this bench until a half-period offset
    exists in the clock path (the AD9695 on-chip delay only reaches ~363 ps,
    against the 1000 ps needed at 500 MS/s).  Any residual gain, offset or
    timing mismatch leaves the tone standing in the difference, so one number
    tracks all three and it needs no interleaving at all.  Once real 2x
    interleaving is available, ``mismatch_spurs`` takes over.
    """
    n = min(ch_a.size, ch_b.size)
    a = np.asarray(ch_a[:n], dtype=np.float64)
    b = np.asarray(ch_b[:n], dtype=np.float64)

    freq, pa = spectrum(a, fs)
    _, pd = spectrum(a - b, fs)

    bin_width = fs / (2.0 * (pa.size - 1))
    b_in = int(np.clip(round(f_in / bin_width), 0, pa.size - 1))
    sl = slice(max(0, b_in - guard), min(pa.size, b_in + guard + 1))

    carrier = float(pa[sl].sum())
    residual = float(pd[sl].sum())
    ratio = 10.0 * np.log10(residual / carrier) if carrier > 0 and residual > 0 else -np.inf

    return {
        "carrier_power": carrier,
        "difference_power": residual,
        "difference_dbc": float(ratio),
        "dc_difference_codes": float(a.mean() - b.mean()),
    }


def mismatch_spurs(x: np.ndarray, fs: float, f_in: float, guard: int = 8) -> dict:
    """Power of the two-channel interleaving spurs, relative to the carrier.

    For a 2x interleaved converter running at ``fs``:
      * offset mismatch  -> tone at fs/2;
      * gain and timing mismatch -> image at fs/2 - f_in.
    Tracking these two numbers is the cleanest way to show the loop working,
    because each maps to one calibrated parameter.
    """
    freq, power = spectrum(x, fs)
    n_bins = power.size

    def band_power(f_target: float) -> tuple[float, int]:
        b = int(round(f_target / (fs / (2.0 * (n_bins - 1)))))
        b = int(np.clip(b, 0, n_bins - 1))
        sl = slice(max(0, b - guard), min(n_bins, b + guard + 1))
        return float(power[sl].sum()), b

    carrier, _ = band_power(f_in)
    offset_spur, _ = band_power(fs / 2.0)
    image_spur, _ = band_power(fs / 2.0 - f_in)

    def dbc(p: float) -> float:
        return 10.0 * np.log10(p / carrier) if carrier > 0 and p > 0 else -np.inf

    return {
        "carrier_power": carrier,
        "offset_spur_dbc": dbc(offset_spur),
        "gain_skew_image_dbc": dbc(image_spur),
        "offset_spur_hz": fs / 2.0,
        "image_spur_hz": fs / 2.0 - f_in,
    }
