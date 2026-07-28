"""
Impulse-dither waveform generation for the AD9164 (DPG) transmitter.

The whole calibration rests on one idea: the analog input applied to the ADC is

    x(t) = s(t) + sum_k  p[k] * A_d * pulse(t - t_k)

where s(t) is a coherent main tone, pulse() is a short flat-topped impulse with
smooth, analytically known edges, and p[k] = +-1 is a *balanced* pseudo-random
polarity sequence.  Because the AD9164 is driven from a DPG vector file, the
dither is summed with the signal in the digital domain of the DAC -- no analog
summing network is required, which is what makes the scheme testable on a
commercial ADC.

Why an impulse and not a ramp (Su, TCAS-I 2022):

  * flat top   -> d(x)/dt = 0, so those samples carry *only* gain + offset
                  information;
  * edges      -> d(x)/dt is maximal and exactly known, so those samples carry
                  *only* timing-skew information.

One dither waveform therefore yields three observables that are separated by
sample position inside the pulse rather than by using three different dithers.
Combined with the +-1 polarity:

    sum_k  p[k] * y[n_k + m]  ->  gain and skew   (offset and input cancel)
    sum_k         y[n_k + m]  ->  offset          (dither and input cancel)

Both come from the same capture, at the same time.

Units convention: all pulse geometry is expressed in *ADC sample periods*.
The DAC grid is an integer multiple K of the ADC grid (fs_dac = K * fs_adc),
which keeps every template analytic and removes any need for resampling.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np

DAC_FULL_SCALE = 32767


@dataclass
class DitherConfig:
    """Everything needed to reproduce the DAC vector and the analysis templates.

    Parameters are written in *DAC* units, because the AD9164 side is the fixed
    part of the bench: DPG plays a vector of ``n_dac_points`` samples at
    ``fs_dac``, and that is not negotiable.  The ADC-side quantities the
    estimator works in are derived from ``adc_ratio``.

    ``adc_ratio`` must be an integer, and it is the one constraint worth
    designing the experiment around.  A non-integer ratio means one DPG loop is
    not a whole number of ADC samples, so the loop stops closing on a sample
    boundary, every impulse lands on a different sub-sample phase, and the
    averaged pulse replica smears out.  Gain comes out biased and skew is not
    recoverable at all.  Both converters already run off the same reference on
    this bench, so satisfying it is a clock setting, not new hardware.
    """

    # --- DAC side (fixed by the DPG / AD9164 setup) -------------------------
    fs_dac: float = 2000.0e6
    """DAC sample rate [Hz].

    Not the 2457.6 MS/s the bench currently runs at: that rate divided by the
    500 MS/s ADC clock is 4.9152, and ``adc_ratio`` has to be an integer (see
    below).  2000 MS/s is the cheapest way to satisfy that -- it is one setting
    on the external clock source feeding J31, and it leaves the ADC side, the
    JESD lane rate and the bitstream untouched.  Raising the ADC clock to
    614.4 MS/s would work equally well and costs a great deal more."""

    n_dac_points: int = 65536
    """Samples in one DPG vector."""

    adc_ratio: int = 4
    """fs_dac / fs_adc.  Must be an integer -- see the class docstring."""

    # --- main tone ----------------------------------------------------------
    sig_cycles: int = 6079
    """Integer cycles of the main tone per vector, so the loop is seamless.

    The value is not free. What matters is how far the tone advances between one
    dither impulse and the next: that phase step is what makes the tone average
    away across events. A step close to a whole cycle means every impulse sees
    the same phase and the tone survives the averaging, inflating the offset and
    gain estimates. 6079 gives 0.746 cycles per slot at the default geometry;
    6143 would give 0.996 and is a trap. :meth:`tone_phase_coherence` measures
    this and :meth:`validate` rejects bad choices."""

    amp_dbfs: float = -6.0
    """Main tone amplitude [dBFS]."""

    # --- dither (all lengths in DAC samples) --------------------------------
    dither_period_dac: int = 256
    """Impulse repetition period [DAC samples]."""

    dither_position_dac: int = 96
    """Impulse start inside its period [DAC samples]."""

    dither_edge_dac: int = 16
    """Rise/fall length [DAC samples]."""

    dither_top_dac: int = 32
    """Flat-top length [DAC samples]."""

    dither_scale_lsb: float = 2000.0
    """Impulse amplitude [LSB].  Convergence time goes as 1/A^2 (Wang et al.,
    TCAS-I 2025, Eq. 7), so this is the main speed knob; the ceiling is the
    headroom left by the main tone."""

    seed: int = 20260725
    """Seed of the balanced polarity sequence.  Must match between the TXT that
    goes into DPG and the analysis code."""

    # --- derived: DAC ------------------------------------------------------
    @property
    def n_dac(self) -> int:
        return self.n_dac_points

    @property
    def n_events(self) -> int:
        return self.n_dac_points // self.dither_period_dac

    # --- derived: ADC ------------------------------------------------------
    @property
    def fs_adc(self) -> float:
        return self.fs_dac / self.adc_ratio

    @property
    def n_adc_period(self) -> int:
        return self.n_dac_points // self.adc_ratio

    @property
    def slot_period(self) -> int:
        return self.dither_period_dac // self.adc_ratio

    @property
    def pulse_offset(self) -> float:
        return self.dither_position_dac / self.adc_ratio

    @property
    def edge_r(self) -> float:
        return self.dither_edge_dac / self.adc_ratio

    @property
    def top_w(self) -> float:
        return self.dither_top_dac / self.adc_ratio

    @property
    def pulse_len(self) -> float:
        """Total pulse support in ADC samples."""
        return 2.0 * self.edge_r + self.top_w

    # --- derived: amplitudes ----------------------------------------------
    @property
    def a_sine(self) -> float:
        return 10.0 ** (self.amp_dbfs / 20.0)

    @property
    def a_dither(self) -> float:
        return self.dither_scale_lsb / DAC_FULL_SCALE

    @property
    def f_sig(self) -> float:
        return self.sig_cycles * self.fs_dac / self.n_dac_points

    def tone_phase_step(self) -> float:
        """Tone phase advance from one dither impulse to the next, in cycles."""
        return (self.sig_cycles * self.slot_period / self.n_adc_period) % 1.0

    def tone_phase_coherence(self, n_events_visible: int = 15) -> float:
        """How badly the main tone survives averaging over the visible events.

        Returns the resultant length of the per-event tone phasors: 0 means the
        phases are spread evenly and the tone cancels, 1 means every event sees
        the same phase and the tone passes straight through into the estimates.
        A capture holds only about 15 events, so the spread has to be good over
        that few — not merely in the limit.
        """
        k = np.arange(max(1, n_events_visible))
        return float(abs(np.mean(np.exp(2j * np.pi * k * self.tone_phase_step()))))

    def validate(self) -> None:
        if int(self.adc_ratio) != self.adc_ratio or self.adc_ratio < 1:
            raise ValueError("adc_ratio must be a positive integer")
        for name, value in (
            ("n_dac_points", self.n_dac_points),
            ("dither_period_dac", self.dither_period_dac),
            ("dither_edge_dac", self.dither_edge_dac),
            ("dither_top_dac", self.dither_top_dac),
            ("dither_position_dac", self.dither_position_dac),
        ):
            if value % self.adc_ratio:
                raise ValueError(
                    f"{name}={value} must be a multiple of adc_ratio={self.adc_ratio}, "
                    "otherwise the ADC-rate templates are not exact"
                )
        if self.n_dac_points % self.dither_period_dac:
            raise ValueError("n_dac_points must be a whole number of dither periods")
        if self.pulse_offset + self.pulse_len > self.slot_period:
            raise ValueError(
                f"pulse ({self.pulse_len} ADC samples at offset {self.pulse_offset}) "
                f"does not fit in a {self.slot_period}-sample slot"
            )
        if self.n_events % 2:
            raise ValueError("n_events must be even so the polarity can be exactly balanced")
        if self.top_w < 2:
            raise ValueError(
                f"flat top is only {self.top_w} ADC samples; the gain and offset "
                "statistics need several samples where dx/dt = 0"
            )
        if self.edge_r < 2:
            raise ValueError(
                f"edge is only {self.edge_r} ADC samples; the skew estimate needs "
                "the ADC to actually land on the ramp"
            )
        if self.a_sine + self.a_dither > 1.0:
            raise ValueError("main tone plus dither would clip the DAC")
        coherence = self.tone_phase_coherence()
        if coherence > 0.3:
            raise ValueError(
                f"sig_cycles={self.sig_cycles} advances the tone by "
                f"{self.tone_phase_step():.4f} cycles per dither slot, so the visible "
                f"events all see nearly the same tone phase (coherence {coherence:.3f} "
                "> 0.3) and the tone will not average out of the estimates. "
                "Pick sig_cycles so the step lands near the middle of a cycle."
            )


# ---------------------------------------------------------------------------
# Pulse shape.  Raised-cosine edges: C1-continuous, band-limited enough that the
# DAC reconstruction filter and the ADC front end do not reshape it much, and
# with a closed-form derivative used for skew extraction.
# ---------------------------------------------------------------------------

def pulse(t: np.ndarray, edge_r: float, top_w: float) -> np.ndarray:
    """Normalised impulse, argument in ADC sample periods, peak value 1."""
    t = np.asarray(t, dtype=np.float64)
    total = 2.0 * edge_r + top_w
    out = np.zeros_like(t)

    rise = (t >= 0.0) & (t < edge_r)
    top = (t >= edge_r) & (t < edge_r + top_w)
    fall = (t >= edge_r + top_w) & (t < total)

    out[rise] = 0.5 * (1.0 - np.cos(np.pi * t[rise] / edge_r))
    out[top] = 1.0
    out[fall] = 0.5 * (1.0 - np.cos(np.pi * (total - t[fall]) / edge_r))
    return out


def pulse_derivative(t: np.ndarray, edge_r: float, top_w: float) -> np.ndarray:
    """d(pulse)/dt with t in ADC sample periods (units: 1 / ADC sample)."""
    t = np.asarray(t, dtype=np.float64)
    total = 2.0 * edge_r + top_w
    out = np.zeros_like(t)

    rise = (t >= 0.0) & (t < edge_r)
    fall = (t >= edge_r + top_w) & (t < total)

    out[rise] = (np.pi / (2.0 * edge_r)) * np.sin(np.pi * t[rise] / edge_r)
    out[fall] = -(np.pi / (2.0 * edge_r)) * np.sin(np.pi * (total - t[fall]) / edge_r)
    return out


def polarity_sequence(cfg: DitherConfig) -> np.ndarray:
    """Exactly balanced +-1 sequence.

    Exact balance (equal number of +1 and -1) matters: it makes sum_k p[k] = 0
    *identically* rather than only in expectation, so the offset estimate stays
    unbiased even over a short record and the dither contributes exactly zero DC
    to the DAC vector.
    """
    half = cfg.n_events // 2
    seq = np.concatenate([np.ones(half), -np.ones(half)])
    rng = np.random.RandomState(cfg.seed)
    rng.shuffle(seq)
    return seq.astype(np.float64)


# ---------------------------------------------------------------------------
# DAC vector
# ---------------------------------------------------------------------------

def build_dac_waveform(cfg: DitherConfig) -> tuple[np.ndarray, np.ndarray]:
    """Return (int16 DAC vector of length cfg.n_dac, polarity sequence)."""
    cfg.validate()
    signs = polarity_sequence(cfg)

    j = np.arange(cfg.n_dac, dtype=np.float64)
    t_adc = j / cfg.adc_ratio  # DAC index expressed on the ADC time grid

    wave = (cfg.a_sine * DAC_FULL_SCALE) * np.sin(
        2.0 * np.pi * cfg.sig_cycles * j / cfg.n_dac
    )

    amp = cfg.a_dither * DAC_FULL_SCALE
    span = int(np.ceil(cfg.pulse_len)) + 2
    for k in range(cfg.n_events):
        start = k * cfg.slot_period + cfg.pulse_offset
        lo = int(np.floor(start * cfg.adc_ratio))
        hi = lo + span * cfg.adc_ratio
        idx = np.arange(lo, hi)
        wave[idx] += signs[k] * amp * pulse(t_adc[idx] - start, cfg.edge_r, cfg.top_w)

    peak = np.max(np.abs(wave))
    if peak > DAC_FULL_SCALE:
        raise ValueError(f"waveform clips: peak {peak:.1f} > {DAC_FULL_SCALE}")

    return np.rint(wave).astype(np.int16), signs


def write_dac_files(cfg: DitherConfig, out_dir: str | Path, stem: str = "impulse_dither") -> dict:
    """Write the DPG TXT vector plus the metadata JSON the analysis code reads."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    wave, signs = build_dac_waveform(cfg)

    txt_path = out_dir / f"{stem}.txt"
    # One signed integer per line, no header -- DPG Downloader format.
    np.savetxt(txt_path, wave, fmt="%d")

    meta = {
        "config": asdict(cfg),
        "derived": {
            "adc_ratio": cfg.adc_ratio,
            "fs_adc_hz": cfg.fs_adc,
            "dither_scale_lsb": cfg.dither_scale_lsb,
            "amp_dbfs": cfg.amp_dbfs,
            "fs_dac_hz": cfg.fs_dac,
            "n_dac_samples": cfg.n_dac,
            "n_adc_samples_per_loop": cfg.n_adc_period,
            "main_tone_hz": cfg.f_sig,
            "main_tone_amplitude_lsb": cfg.a_sine * DAC_FULL_SCALE,
            "dither_amplitude_lsb": cfg.a_dither * DAC_FULL_SCALE,
            "dither_type": "balanced random-polarity raised-cosine impulse",
            "slot_period_adc_samples": cfg.slot_period,
            "pulse_len_adc_samples": cfg.pulse_len,
            "dither_duty_cycle": cfg.pulse_len / cfg.slot_period,
            "min_sample": int(wave.min()),
            "max_sample": int(wave.max()),
            "mean_sample": float(wave.mean()),
            "clipping": bool(np.max(np.abs(wave)) >= DAC_FULL_SCALE),
            "seamless_loop": True,
            "polarity_sum": float(signs.sum()),
        },
        "polarity": signs.astype(int).tolist(),
    }

    json_path = out_dir / f"{stem}.json"
    json_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")

    return {"txt": txt_path, "json": json_path, "meta": meta}


# ---------------------------------------------------------------------------
# ADC-rate templates used by the estimator
# ---------------------------------------------------------------------------

def adc_templates(cfg: DitherConfig, guard: int = 2) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Pulse and pulse-derivative sampled on the ADC grid.

    Returns (m, d, dprime) where m are integer ADC-sample offsets relative to
    the nominal pulse start, spanning [-guard, pulse_len + guard).
    """
    lo = -guard
    hi = int(np.ceil(cfg.pulse_len)) + guard
    m = np.arange(lo, hi, dtype=np.float64)
    return m, pulse(m, cfg.edge_r, cfg.top_w), pulse_derivative(m, cfg.edge_r, cfg.top_w)


def reference_loop(cfg: DitherConfig, scale: float = 1.0) -> np.ndarray:
    """Ideal ADC-rate view of one full DPG loop (used for alignment)."""
    signs = polarity_sequence(cfg)
    n = cfg.n_adc_period
    t = np.arange(n, dtype=np.float64)

    ref = (cfg.a_sine * scale) * np.sin(2.0 * np.pi * cfg.sig_cycles * t / n)

    span = int(np.ceil(cfg.pulse_len)) + 2
    for k in range(cfg.n_events):
        start = k * cfg.slot_period + cfg.pulse_offset
        lo = int(np.floor(start))
        idx = np.arange(lo, lo + span) % n
        ref[idx] += signs[k] * cfg.a_dither * scale * pulse(
            t[idx] - start, cfg.edge_r, cfg.top_w
        )
    return ref


def dither_only_loop(cfg: DitherConfig, scale: float = 1.0) -> np.ndarray:
    """Dither component alone over one loop -- the alignment matched filter."""
    signs = polarity_sequence(cfg)
    n = cfg.n_adc_period
    t = np.arange(n, dtype=np.float64)
    ref = np.zeros(n)

    span = int(np.ceil(cfg.pulse_len)) + 2
    for k in range(cfg.n_events):
        start = k * cfg.slot_period + cfg.pulse_offset
        lo = int(np.floor(start))
        idx = np.arange(lo, lo + span) % n
        ref[idx] += signs[k] * cfg.a_dither * scale * pulse(
            t[idx] - start, cfg.edge_r, cfg.top_w
        )
    return ref
