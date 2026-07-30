"""
Bench-free model of the DAC -> splitter -> AD9695 -> JESD -> DMA -> UDP path.

Run the whole calibration loop against this before touching hardware.  It
reproduces the parts that actually break experiments: an arbitrary DMA start
phase (so the [A A A A B B B B] framing must be recovered), an inverted splitter
branch, sub-sample timing skew, quantisation and thermal noise, and clipping.

The "analog" waveform is evaluated in closed form rather than interpolated from
the DAC vector, so a commanded skew of a fraction of a picosecond is represented
exactly and the estimator is tested against a known ground truth.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dither import DitherConfig, polarity_sequence, pulse, DAC_FULL_SCALE

ADC_FULL_SCALE = 8191  # 14-bit signed


@dataclass
class BenchModel:
    cfg: DitherConfig

    gain_a: float = 1.000
    gain_b: float = 1.021
    """Channel gains.  A 2 % mismatch is typical for two inputs of the same
    evaluation board driven through a splitter."""

    offset_a_codes: float = 14.0
    offset_b_codes: float = -23.0

    skew_a_ps: float = 0.0
    skew_b_ps: float = 3.6
    """Static sampling-instant mismatch before any calibration."""

    noise_rms_codes: float = 3.0
    adc_scale: float = ADC_FULL_SCALE / DAC_FULL_SCALE
    """Codes per DAC LSB through the analog path (0.25 = no extra loss)."""

    invert_b: bool = True
    """The evaluation board is fed from a splitter with one inverted branch."""

    seed: int = 7

    def __post_init__(self):
        self._rng = np.random.RandomState(self.seed)
        self._signs = polarity_sequence(self.cfg)
        self._phase = float(self._rng.randint(0, self.cfg.n_adc_period))
        self.subsample_phase = 0.37
        """Fixed sub-sample offset between the ADC clock and the DPG loop.  With
        an integer fs_dac / fs_adc and a shared reference this is a constant of
        the clock path, so it must not be mistaken for a channel mismatch."""
        self.skew_cmd_b_ps = 0.0
        """Delay currently commanded to channel B by the loop, in ps."""

    # -- analog source ------------------------------------------------------
    def analog(self, t: np.ndarray) -> np.ndarray:
        """Intended DAC output at arbitrary ADC-grid time ``t`` (DAC LSBs)."""
        cfg = self.cfg
        t = np.asarray(t, dtype=np.float64)
        tm = np.mod(t, cfg.n_adc_period)

        out = (cfg.a_sine * DAC_FULL_SCALE) * np.sin(
            2.0 * np.pi * cfg.sig_cycles * tm / cfg.n_adc_period
        )

        # The pulse never straddles a slot boundary (validate() enforces it), so
        # a slot lookup evaluates the whole dither train exactly.
        slot = np.floor(tm / cfg.slot_period).astype(int) % cfg.n_events
        local = tm - slot * cfg.slot_period - cfg.pulse_offset
        out += (
            self._signs[slot]
            * cfg.a_dither
            * DAC_FULL_SCALE
            * pulse(local, cfg.edge_r, cfg.top_w)
        )
        return out

    # -- actuator -----------------------------------------------------------
    def command_skew(self, delay_ps: float) -> None:
        """What the AD9695 fine / super-fine clock delay does to channel B."""
        self.skew_cmd_b_ps = float(delay_ps)

    # -- capture ------------------------------------------------------------
    def capture(self, n_words: int = 2040) -> bytes:
        """Return one DMA buffer worth of raw little-endian JESD words."""
        cfg = self.cfg
        n_pairs = n_words // 8
        n_samples = n_pairs * 4

        n = np.arange(n_samples, dtype=np.float64)
        ts_ps = 1e12 / cfg.fs_adc

        base = self._phase + self.subsample_phase + n
        t_a = base + (self.skew_a_ps / ts_ps)
        t_b = base + ((self.skew_b_ps + self.skew_cmd_b_ps) / ts_ps)

        a = self.gain_a * self.adc_scale * self.analog(t_a) + self.offset_a_codes
        b = self.gain_b * self.adc_scale * self.analog(t_b) + self.offset_b_codes

        a += self._rng.normal(0, self.noise_rms_codes, a.shape)
        b += self._rng.normal(0, self.noise_rms_codes, b.shape)

        if self.invert_b:
            b = -b

        a = np.clip(np.rint(a), -ADC_FULL_SCALE - 1, ADC_FULL_SCALE)
        b = np.clip(np.rint(b), -ADC_FULL_SCALE - 1, ADC_FULL_SCALE)

        words = np.empty(n_pairs * 8, dtype=np.int16)
        words.reshape(-1, 8)[:, :4] = a.reshape(-1, 4)
        words.reshape(-1, 8)[:, 4:] = b.reshape(-1, 4)

        # 14-bit data sits left-justified in the 16-bit JESD word.
        packed = (words.astype(np.int32) << 2).astype(np.int16)

        # The DMA starts at an arbitrary point in the stream.
        rot = self._rng.randint(0, 8)
        packed = np.roll(packed, rot)

        # The loop keeps running between captures.
        self._phase = (self._phase + n_samples + self._rng.randint(0, 64)) % cfg.n_adc_period

        return packed.tobytes()

    # -- ground truth -------------------------------------------------------
    def truth(self) -> dict:
        return {
            "gain_ratio": self.gain_b / self.gain_a,
            "offset_a_codes": self.offset_a_codes,
            "offset_b_codes": self.offset_b_codes,
            "skew_mismatch_ps": (self.skew_b_ps + self.skew_cmd_b_ps) - self.skew_a_ps,
        }
