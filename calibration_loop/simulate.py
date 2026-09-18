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

    pulse_gain_a: float = 1.0
    pulse_gain_b: float = 1.0
    """Extra gain applied to the *dither pulses* only, per channel.

    The default 1.0 leaves the model exactly as it was.  Setting them apart
    reproduces a measured bench fact: the narrow pulse replicas and the 199 MHz tone
    do not see quite the same channel gain, and the dither amplitude ratio reads
    ~1.2 % above the tone amplitude ratio.  That single effect is why a gain loop
    driven by the pulse windows seats its correction away from the point that nulls
    the *tone* -- so without a knob for it the model cannot exhibit the failure, and
    a fix for it cannot be tested without the bench.
    """

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
    def tone(self, t: np.ndarray) -> np.ndarray:
        """Main tone at arbitrary ADC-grid time ``t`` (DAC LSBs)."""
        cfg = self.cfg
        tm = np.mod(np.asarray(t, dtype=np.float64), cfg.n_adc_period)
        return (cfg.a_sine * DAC_FULL_SCALE) * np.sin(
            2.0 * np.pi * cfg.sig_cycles * tm / cfg.n_adc_period
        )

    def pulses(self, t: np.ndarray) -> np.ndarray:
        """The dither pulse train alone, at the same times ``t`` (DAC LSBs).

        Split out of :meth:`analog` so the two components can be given different
        channel gains (see ``pulse_gain_a`` / ``pulse_gain_b``); ``analog`` is still
        their sum, so callers that want the whole waveform are unaffected.
        """
        cfg = self.cfg
        tm = np.mod(np.asarray(t, dtype=np.float64), cfg.n_adc_period)
        # The pulse never straddles a slot boundary (validate() enforces it), so
        # a slot lookup evaluates the whole dither train exactly.
        slot = np.floor(tm / cfg.slot_period).astype(int) % cfg.n_events
        local = tm - slot * cfg.slot_period - cfg.pulse_offset
        return (
            self._signs[slot]
            * cfg.a_dither
            * DAC_FULL_SCALE
            * pulse(local, cfg.edge_r, cfg.top_w)
        )

    def analog(self, t: np.ndarray) -> np.ndarray:
        """Intended DAC output at arbitrary ADC-grid time ``t`` (DAC LSBs)."""
        return self.tone(t) + self.pulses(t)

    # -- actuator -----------------------------------------------------------
    def command_skew(self, delay_ps: float) -> None:
        """What the AD9695 fine / super-fine clock delay does to channel B."""
        self.skew_cmd_b_ps = float(delay_ps)

    # -- capture ------------------------------------------------------------
    def capture(self, n_words: int = 2048) -> bytes:
        """Return one DMA buffer worth of raw little-endian JESD words.

        ``n_words`` counts *16-bit JESD words*, matching the DMA transfer size
        the board reports (4096 bytes = 2048 words).  Each 8-word beat carries 4
        samples per channel, so the usable channel length is
        ``n_words // 8 * 4`` samples.
        """
        cfg = self.cfg
        n_pairs = n_words // 8
        n_samples = n_pairs * 4
        n_words_used = n_pairs * 8

        n = np.arange(n_samples, dtype=np.float64)
        ts_ps = 1e12 / cfg.fs_adc

        base = self._phase + self.subsample_phase + n
        t_a = base + (self.skew_a_ps / ts_ps)
        t_b = base + ((self.skew_b_ps + self.skew_cmd_b_ps) / ts_ps)

        # The pulses carry their own per-channel gain (see pulse_gain_a/_b); with the
        # defaults of 1.0 this is exactly gain * adc_scale * analog(t) as before.
        a = self.gain_a * self.adc_scale * (
            self.tone(t_a) + self.pulse_gain_a * self.pulses(t_a)
        ) + self.offset_a_codes
        b = self.gain_b * self.adc_scale * (
            self.tone(t_b) + self.pulse_gain_b * self.pulses(t_b)
        ) + self.offset_b_codes

        a += self._rng.normal(0, self.noise_rms_codes, a.shape)
        b += self._rng.normal(0, self.noise_rms_codes, b.shape)

        if self.invert_b:
            b = -b

        a = np.clip(np.rint(a), -ADC_FULL_SCALE - 1, ADC_FULL_SCALE)
        b = np.clip(np.rint(b), -ADC_FULL_SCALE - 1, ADC_FULL_SCALE)

        words = np.empty(n_words_used, dtype=np.int16)
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
