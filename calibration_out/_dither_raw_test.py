"""Instrument check for tools/dither_raw_evidence.py against known ground truth.

The tone-free raw tool measures the A-B timing from folded impulse replicas.  Before that
number is trusted on the bench, the measurement itself has to be shown to recover a *known*
delay -- otherwise a bench reading is just a plausible-looking artifact.

The synthetic frames here are built from the real generator (``build_dac_waveform``), so they
carry the real raised-cosine pulse shape, the real 130-sample period and the real balanced
random +/- polarity sequence, which is what decides the folded replica's amplitude.  They
then get a known fractional delay on channel B, Gaussian noise, 14-bit quantisation and the
real 8-word group packing with a random rotation per frame.

Reported: recovered lag against the delay that was put in, and the per-frame scatter, which
is the number that decides how long a state has to be recorded.

    python calibration_out/_dither_raw_test.py
"""

from __future__ import annotations

import os
import shutil
import sys
import tempfile

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "tools"))

from calibration_loop.dither import DitherConfig, build_dac_waveform   # noqa: E402
from dither_raw_evidence import analyse                                # noqa: E402

FS_ADC = 1.300e9
PS_PER_SAMPLE = 1e12 / FS_ADC
FRAME_SAMPLES = 1020                       # 255 groups of 4, what one record holds
PAYLOAD = 4095


def synth(cfg: DitherConfig, pulse_pp: float, noise_rms: float, delay_samples: float,
          n_frames: int, seed: int):
    """One state: n_frames 4095-byte payloads with channel B delayed by delay_samples."""
    rng = np.random.default_rng(seed)
    wave, _ = build_dac_waveform(cfg)
    adc = wave[::cfg.adc_ratio].astype(np.float64)          # what the ADC grid sees
    n = adc.size

    # scale so the impulse peak-to-peak lands on the requested code amplitude
    probe = adc - adc.mean()
    span = float(probe.max() - probe.min())
    adc = adc * (pulse_pp / span) if span else adc

    # exact fractional delay on the seamless loop: circular shift, the loop closes
    k = np.fft.rfftfreq(n) * n
    shifted = np.fft.irfft(np.fft.rfft(adc) * np.exp(-2j * np.pi * k * delay_samples / n), n=n)

    frames = []
    for _ in range(n_frames):
        start = int(rng.integers(0, n))
        idx = (np.arange(FRAME_SAMPLES) + start) % n
        a = np.rint(adc[idx] + rng.normal(0.0, noise_rms, FRAME_SAMPLES))
        b = np.rint(shifted[idx] + rng.normal(0.0, noise_rms, FRAME_SAMPLES))
        words = np.empty(FRAME_SAMPLES // 4 * 8, dtype="<i2")
        aa = (a.astype(np.int32) << 2).astype("<i2").reshape(-1, 4)
        bb = (b.astype(np.int32) << 2).astype("<i2").reshape(-1, 4)
        words.reshape(-1, 8)[:, :4] = aa
        words.reshape(-1, 8)[:, 4:] = bb
        groups = words.reshape(-1, 8)
        groups = np.roll(groups, int(rng.integers(0, 8)), axis=0)   # DMA restart phase
        blob = groups.tobytes() + b"\x00" * (PAYLOAD - groups.nbytes)
        frames.append(blob[:PAYLOAD])
    return frames


def run_case(cfg: DitherConfig, pulse_pp: float, noise_rms: float, delays, n_frames: int):
    print(f"\ngeometry {cfg.dither_edge_dac}/{cfg.dither_top_dac} DAC "
          f"= {2 * cfg.edge_r + cfg.top_w:.0f} ADC samples, "
          f"replica {pulse_pp:.0f} codes pk-pk, noise {noise_rms:.0f} codes rms, "
          f"{n_frames} frames")
    print(f"  {'true ps':>9}{'slope dt':>10}{'err':>8}{'xcorr':>9}{'err':>9}"
          f"{'per-frame sd':>14}{'rep p-p':>9}{'aligned':>9}")
    for delay in delays:
        tmp = tempfile.mkdtemp(prefix="dither_raw_")
        try:
            for i, blob in enumerate(synth(cfg, pulse_pp, noise_rms, delay, n_frames, 1234)):
                with open(os.path.join(tmp, f"frame_{i:04d}.bin"), "wb") as fh:
                    fh.write(blob)
            r = analyse(tmp, cfg, strip_tone=False)
            true_ps = delay * PS_PER_SAMPLE
            print(f"  {true_ps:>9.1f}{r['dt_ps']:>10.1f}{r['dt_ps'] - true_ps:>8.1f}"
                  f"{r['lag_ps']:>9.1f}{r['lag_ps'] - true_ps:>9.1f}"
                  f"{r['per_dt'][1]:>14.1f}{r['pp_a']:>9.1f}"
                  f"{r['aligned']:>6d}/{r['frames']}")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


def main() -> int:
    base = dict(amp_dbfs=-120.0, dither_scale_lsb=2000.0, dither_period_dac=260,
                n_dac_points=16640, adc_ratio=2, sig_cycles=1276, dither_position_dac=96)

    delays = (0.0, 0.0572, -0.0572, 0.5, 1.0)          # 0, +-44 ps, half a sample, one sample
    print("known-delay recovery, the tone-free raw tool (positive = B late)")
    run_case(DitherConfig(**{**base, "dither_edge_dac": 16, "dither_top_dac": 32}),
             pulse_pp=26.0, noise_rms=8.0, delays=delays, n_frames=40)
    run_case(DitherConfig(**{**base, "dither_edge_dac": 4, "dither_top_dac": 4}),
             pulse_pp=26.0, noise_rms=8.0, delays=delays, n_frames=40)
    run_case(DitherConfig(**{**base, "dither_edge_dac": 4, "dither_top_dac": 4}),
             pulse_pp=8.0, noise_rms=8.0, delays=(0.0, 0.0572, -0.0572), n_frames=40)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
