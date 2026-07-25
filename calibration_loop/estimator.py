"""
Impulse-dither estimator: offset, gain and timing skew from one capture.

Signal model for channel i (ADC codes):

    y_i[n] = g_i * x(t_n + dt_i) + o_i + noise

with x(t) = s(t) + sum_k p[k] * A_d * pulse(t - t_k).

Let n_k be the captured index of the pulse start of event k and let
r[n] = y[n] - s_hat[n] be the record after the (known-frequency) main tone has
been fitted and removed.  Averaging over the visible events with and without the
polarity weight gives two orthogonal statistics:

    V[m] = mean_k  p[k] * r[n_k + m]  =  G * d(m + dt)      (offset + tone cancel)
    U[m] = mean_k         r[n_k + m]  =  o                  (dither cancels exactly)

because sum_k p[k] = 0 by construction.  From V, a first-order expansion
d(m + dt) ~ d(m) + dt * d'(m) separates amplitude from timing:

    G_hat  = <V, d> / <d, d>
    dt_hat = <V - G_hat*d, d'> / (G_hat * <d', d'>)

and the two are almost uncorrelated because <d, d'> ~ 0 for a symmetric pulse.
That orthogonality is the reason a single impulse dither can resolve gain and
skew at once, and it is what a ramp dither cannot do (a ramp has d' = const, so
its amplitude and timing errors are indistinguishable).

Removing the main tone before averaging is the direct analogue of the
sub-ADC interference cancellation in Wang et al., TCAS-I 2025: it does not change
the expected value of the estimate, it shrinks its variance, and therefore cuts
the number of cycles needed to converge.  Set ``cancel_signal=False`` to measure
the un-cancelled baseline for the comparison figure.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .dither import (
    DitherConfig,
    adc_templates,
    dither_only_loop,
    polarity_sequence,
    pulse,
    pulse_derivative,
)


# ---------------------------------------------------------------------------
# Frame de-interleaving
# ---------------------------------------------------------------------------

def unpack_words(raw_bytes: bytes | np.ndarray, shift: int = 2) -> np.ndarray:
    """Raw DMA bytes -> signed 14-bit ADC codes (one entry per JESD word)."""
    raw = np.frombuffer(bytes(raw_bytes), dtype=np.uint8)
    if raw.size % 2:
        raw = raw[:-1]
    words = raw.view("<i2")
    return (words >> shift).astype(np.int32)


def split_at(words: np.ndarray, rotation: int, group: int = 8, half: int = 4) -> dict:
    """Split the word stream into two channels at a given group phase."""
    body = words[rotation:]
    body = body[: (body.size // group) * group]
    if body.size < 4 * group:
        raise ValueError("capture too short to de-frame")
    g = body.reshape(-1, group)
    return {
        "rotation": rotation,
        "ch_a": g[:, :half].reshape(-1).astype(np.float64),
        "ch_b": g[:, half:].reshape(-1).astype(np.float64),
    }


def deframe(words: np.ndarray, group: int = 8, half: int = 4) -> dict:
    """Find the [A A A A B B B B] group boundary in the word stream.

    The DMA is restarted for every capture and the AXI-Stream from the JESD link
    runs continuously, so the group boundary lands at an arbitrary one of
    ``group`` phases.  The correct phase is the one whose two de-interleaved
    streams are each a clean single tone, measured as the fraction of AC power in
    the strongest FFT bin.

    Note that phases ``r`` and ``r + half`` score identically -- both split the
    stream cleanly, they just start on the other converter.  Those two are *not*
    interchangeable: at ``r + half`` the second output is shifted by ``half``
    samples relative to the first, because it picks up the next group.  Choosing
    between them is :func:`prepare_capture`'s job, and it must re-split rather
    than swap the two arrays.
    """
    best = None
    for rot in range(group):
        try:
            cand = split_at(words, rot, group, half)
        except ValueError:
            continue
        cand["score"] = _tone_purity(cand["ch_a"]) + _tone_purity(cand["ch_b"])
        if best is None or cand["score"] > best["score"]:
            best = cand

    if best is None:
        raise ValueError("capture too short to de-frame")
    return best


def prepare_capture(
    raw_bytes: bytes | np.ndarray,
    cfg: DitherConfig,
    signature: dict | None = None,
) -> dict:
    """Turn one raw DMA buffer into two canonically ordered, upright channels.

    Three ambiguities have to be resolved on *every* capture, because the DMA is
    restarted each time and starts at an arbitrary point in a continuously
    running AXI-Stream:

    1. *Group phase.*  Which of the 8 word positions begins an
       ``[A A A A B B B B]`` group.  Recovered by tone purity.

    2. *Channel order.*  Tone purity cannot separate phase ``r`` from ``r+4``,
       because both split the stream into two clean tones -- they just swap the
       two channels.  If this is left to chance the channels trade places
       between iterations and the loop chases its own tail.  It is pinned here
       by the splitter's inverted branch, which is a stable physical marker: the
       branch whose dither correlates *positively* with the template is always
       reported as channel A.

    3. *Absolute sign.*  The inverted branch is flipped upright so both channels
       share one sense.

    When both branches happen to have the same sense the inversion marker is
    unavailable, and the stored ``signature`` (the raw DC level of each channel,
    which is a physical constant of the board) breaks the tie instead.
    """
    words = unpack_words(raw_bytes)
    f0 = cfg.sig_cycles / cfg.n_adc_period

    def inspect(rotation: int) -> dict:
        frame = split_at(words, rotation)
        out = {"rotation": rotation}
        for key in ("ch_a", "ch_b"):
            fit = fit_tone(frame[key], f0, refine=True)
            al = align_to_loop(fit["residual"], cfg)
            out[key] = frame[key] * al["sign"]
            out[f"{key}_sign"] = al["sign"]
            out[f"{key}_align"] = al
        return out

    rot = deframe(words)["rotation"]
    view = inspect(rot)

    sign_a, sign_b = view["ch_a_sign"], view["ch_b_sign"]
    if sign_a != sign_b:
        # Exactly one branch is inverted -> unambiguous ordering.
        wrong_order = sign_a < 0
    elif signature is not None:
        keep = ((view["ch_a"].mean() - signature["dc_a"]) ** 2
                + (view["ch_b"].mean() - signature["dc_b"]) ** 2)
        swap = ((view["ch_b"].mean() - signature["dc_a"]) ** 2
                + (view["ch_a"].mean() - signature["dc_b"]) ** 2)
        wrong_order = swap < keep
    else:
        wrong_order = False

    if wrong_order:
        rot = (rot + 4) % 8
        view = inspect(rot)

    a, b = view["ch_a"], view["ch_b"]
    al_a, al_b = view["ch_a_align"], view["ch_b_align"]

    # Both channels see the same DPG loop, so one alignment serves both; take
    # whichever correlation peak was stronger.
    n0 = al_a["n0"] if abs(al_a["peak"]) >= abs(al_b["peak"]) else al_b["n0"]

    return {
        "ch_a": a,
        "ch_b": b,
        "n0": n0,
        "rotation": rot,
        "swapped": wrong_order,
        "sign_a": view["ch_a_sign"],
        "sign_b": view["ch_b_sign"],
        "align_margin": float(min(al_a["margin"], al_b["margin"])),
        "signature": signature or {"dc_a": float(a.mean()), "dc_b": float(b.mean())},
    }


def _tone_purity(x: np.ndarray) -> float:
    """Fraction of AC power sitting in the single strongest FFT bin."""
    x = x - x.mean()
    if not np.any(x):
        return 0.0
    spec = np.abs(np.fft.rfft(x * np.hanning(x.size))) ** 2
    spec[0] = 0.0
    total = spec.sum()
    return float(spec.max() / total) if total > 0 else 0.0


# ---------------------------------------------------------------------------
# Main tone fit
# ---------------------------------------------------------------------------

def fit_tone(y: np.ndarray, f0: float, refine: bool = True) -> dict:
    """Least-squares fit of ``a*cos + b*sin + dc`` at normalised frequency f0.

    ``f0`` is in cycles per ADC sample.  When ``refine`` is set the frequency is
    polished by a local search, which absorbs any residual drift between the
    DAC and ADC reference clocks.
    """
    n = np.arange(y.size, dtype=np.float64)

    def solve(freq: float):
        w = 2.0 * np.pi * freq * n
        design = np.column_stack([np.cos(w), np.sin(w), np.ones_like(n)])
        coef, *_ = np.linalg.lstsq(design, y, rcond=None)
        resid = y - design @ coef
        return coef, design, float(resid @ resid)

    coef, design, sse = solve(f0)

    if refine:
        step = 0.25 / y.size  # a quarter of an FFT bin
        for _ in range(40):
            improved = False
            for cand in (f0 - step, f0 + step):
                if cand <= 0 or cand >= 0.5:
                    continue
                c2, d2, s2 = solve(cand)
                if s2 < sse:
                    f0, coef, design, sse = cand, c2, d2, s2
                    improved = True
                    break
            if not improved:
                step *= 0.5
                if step < 1e-12:
                    break

    a, b, dc = coef
    return {
        "f0": f0,
        "amplitude": float(np.hypot(a, b)),
        "phase": float(np.arctan2(-b, a)),
        "dc": float(dc),
        "tone": design[:, :2] @ coef[:2],
        "residual": y - design @ coef,
        "residual_with_dc": y - design[:, :2] @ coef[:2],
    }


# ---------------------------------------------------------------------------
# Alignment
# ---------------------------------------------------------------------------

def align_to_loop(residual: np.ndarray, cfg: DitherConfig) -> dict:
    """Find where the capture sits inside the repeating DPG loop.

    ``residual`` must be the record with the main tone removed, so what is left
    is essentially the dither train.  Correlating it against the known
    dither-only loop gives a sharp, unambiguous peak: the balanced pseudo-random
    polarity makes the template behave like a sync pattern, whereas correlating
    against the tone alone would be periodic and ambiguous.

    The peak is taken on the *magnitude*, so a branch that arrives inverted
    through the splitter still aligns correctly and reports ``sign = -1``.
    Alignment and inversion detection therefore come out of one correlation.

    Returns ``n0`` such that captured sample ``n`` is loop position
    ``(n + n0) mod n_adc_period``.
    """
    n_loop = cfg.n_adc_period
    if residual.size > n_loop:
        residual = residual[:n_loop]

    ref = dither_only_loop(cfg)
    ref = ref - ref.mean()

    padded = np.zeros(n_loop)
    padded[: residual.size] = residual - residual.mean()

    # score[n0] = sum_n padded[n] * ref[n + n0]
    score = np.fft.irfft(np.conj(np.fft.rfft(padded)) * np.fft.rfft(ref), n=n_loop)

    n0 = int(np.argmax(np.abs(score)))
    peak = float(score[n0])
    others = np.delete(score, n0)
    margin = float((abs(peak) - np.abs(others).mean()) / (np.abs(others).std() + 1e-30))

    return {
        "n0": n0,
        "peak": peak,
        "sign": 1.0 if peak >= 0 else -1.0,
        "margin": margin,
        "score": score,
    }


def visible_events(n0: int, n_capture: int, cfg: DitherConfig, m_lo: int, m_hi: int):
    """Indices of dither events fully contained in the capture, and their starts."""
    n_loop = cfg.n_adc_period
    ks, starts = [], []
    for k in range(cfg.n_events):
        loop_pos = k * cfg.slot_period + cfg.pulse_offset
        n_k = (loop_pos - n0) % n_loop
        if n_k + m_lo >= 0 and n_k + m_hi < n_capture:
            ks.append(k)
            starts.append(int(round(n_k)))
    return np.asarray(ks, dtype=int), np.asarray(starts, dtype=int)


# ---------------------------------------------------------------------------
# Per-channel estimate
# ---------------------------------------------------------------------------

@dataclass
class ChannelEstimate:
    offset_codes: float = np.nan
    """DC offset in ADC codes, from the polarity-summed dither statistic."""

    offset_record_codes: float = np.nan
    """DC offset from the whole-record least-squares fit (cross-check)."""

    gain_codes: float = np.nan
    """Observed dither amplitude in ADC codes; proportional to channel gain."""

    skew_samples: float = np.nan
    """Sampling-instant error in ADC sample periods (positive = samples late)."""

    skew_ps: float = np.nan
    tone_amplitude: float = np.nan
    n_events_used: int = 0
    align_margin: float = np.nan
    align_n0: int = -1
    residual_rms: float = np.nan
    v_profile: np.ndarray = field(default=None, repr=False)
    u_profile: np.ndarray = field(default=None, repr=False)


def estimate_channel(
    y: np.ndarray,
    cfg: DitherConfig,
    cancel_signal: bool = True,
    n0: int | None = None,
    refine_frequency: bool = True,
    passes: int = 2,
    skew_prior_samples: float = 0.0,
) -> ChannelEstimate:
    """Extract offset, gain and skew of one channel from one capture.

    The tone fit and the dither estimate are coupled: over a record of about a
    thousand samples the dither has a non-zero projection onto the tone basis, so
    a single-pass fit quietly absorbs part of the dither and leaves a residual
    that correlates with the polarity sequence.  ``passes`` alternates the two --
    subtract the current dither estimate, re-fit the tone, re-estimate the dither
    -- which decouples them.  Two passes are enough; the third changes nothing
    measurable.

    ``skew_prior_samples`` shifts the template by a known amount before fitting.
    True 2x interleaving puts channel B half a sample from channel A, which is
    far outside the first-order expansion the skew estimate relies on; declaring
    that half sample as a prior makes the estimator measure only the *error*
    around it, where the expansion is valid again.
    """
    y = np.asarray(y, dtype=np.float64)
    signs = polarity_sequence(cfg)
    m, _, _ = adc_templates(cfg)
    d = pulse(m + skew_prior_samples, cfg.edge_r, cfg.top_w)
    dprime = pulse_derivative(m + skew_prior_samples, cfg.edge_r, cfg.top_w)
    m_lo, m_hi = int(m[0]), int(m[-1])
    dd = float(d @ d)
    dpdp = float(dprime @ dprime)

    f0 = cfg.sig_cycles / cfg.n_adc_period
    est = ChannelEstimate()
    dither_hat = np.zeros_like(y)
    n0_used = None if n0 is None else int(n0)

    for pass_index in range(max(1, passes)):
        fit = fit_tone(y - dither_hat, f0, refine=refine_frequency)

        if n0_used is None:
            align = align_to_loop(fit["residual"], cfg)
            n0_used = align["n0"]
            est.align_margin = align["margin"]

        est.align_n0 = n0_used
        est.offset_record_codes = fit["dc"]
        est.tone_amplitude = fit["amplitude"]
        est.residual_rms = float(np.std(fit["residual"]))

        # With cancellation on, the fitted tone is removed but the DC term is
        # kept, because DC is exactly what the offset statistic measures.  With
        # it off, the raw record goes through untouched -- the un-cancelled
        # baseline used for the convergence-speed comparison.
        resid = (y - fit["tone"]) if cancel_signal else y

        ks, starts = visible_events(n0_used, y.size, cfg, m_lo, m_hi)
        est.n_events_used = int(ks.size)
        if ks.size < 2:
            return est

        idx = starts[:, None] + np.arange(m_lo, m_hi + 1)[None, :]
        windows = resid[idx]

        p = signs[ks][:, None]
        v = (p * windows).mean(axis=0)   # dither replica: offset and tone cancel
        u = windows.mean(axis=0)         # offset: dither cancels exactly

        est.v_profile, est.u_profile = v, u

        gain = float(v @ d) / dd if dd > 0 else np.nan
        est.gain_codes = gain

        if np.isfinite(gain) and abs(gain) > 1e-9 and dpdp > 0:
            est.skew_samples = skew_prior_samples + float(
                (v - gain * d) @ dprime
            ) / (gain * dpdp)
            est.skew_ps = est.skew_samples / cfg.fs_adc * 1e12

        # Offset from the flat top only: those samples are insensitive to skew,
        # so a residual timing error cannot leak into the offset estimate.
        top = (m >= cfg.edge_r) & (m < cfg.edge_r + cfg.top_w)
        est.offset_codes = float(u[top].mean()) if np.any(top) else float(u.mean())

        if pass_index + 1 < passes and np.isfinite(gain):
            dither_hat = synthesize_dither(y.size, n0_used, cfg, gain)

    return est


@dataclass
class BlockEstimate:
    ch_a: ChannelEstimate
    ch_b: ChannelEstimate
    gain_ratio: float = np.nan
    """g_B / g_A -- the quantity a two-channel TI ADC actually needs."""

    offset_mismatch_codes: float = np.nan
    skew_mismatch_ps: float = np.nan
    rotation: int = -1


def estimate_block(
    ch_a: np.ndarray,
    ch_b: np.ndarray,
    cfg: DitherConfig,
    cancel_signal: bool = True,
    n0: int | None = None,
    skew_prior_samples: float = 0.0,
) -> BlockEstimate:
    """Estimate both channels of the pair from one capture.

    Both channels must use the *same* loop position ``n0``: they see one DPG
    loop, so a per-channel alignment slip of one sample would show up as a
    2 ns skew mismatch that is not there.  When ``n0`` is not supplied, channel
    A's alignment is measured and reused for channel B.
    """
    # Pass 1: rough estimate of channel A, which also fixes the loop alignment.
    a = estimate_channel(ch_a, cfg, cancel_signal=cancel_signal, n0=n0)

    # The ADC clock has a fixed but unknown sub-sample phase with respect to the
    # DPG loop, and it shows up identically in both channels.  Only the
    # difference is a mismatch, so channel A's own estimate is promoted to a
    # common prior and both channels are re-fitted around it.  Without this the
    # first-order expansion would be evaluated far from its centre whenever that
    # common phase happens to be large, and both estimates would degrade.
    common = a.skew_samples if np.isfinite(a.skew_samples) else 0.0
    a = estimate_channel(
        ch_a, cfg, cancel_signal=cancel_signal, n0=a.align_n0,
        skew_prior_samples=common,
    )
    b = estimate_channel(
        ch_b, cfg, cancel_signal=cancel_signal, n0=a.align_n0,
        skew_prior_samples=common + skew_prior_samples,
    )

    out = BlockEstimate(ch_a=a, ch_b=b)
    if np.isfinite(a.gain_codes) and abs(a.gain_codes) > 1e-9:
        out.gain_ratio = b.gain_codes / a.gain_codes
    out.offset_mismatch_codes = b.offset_codes - a.offset_codes
    out.skew_mismatch_ps = b.skew_ps - a.skew_ps
    return out


# ---------------------------------------------------------------------------
# Block LMS state
# ---------------------------------------------------------------------------

@dataclass
class CalibrationState:
    """Digital correction coefficients, updated once per captured block.

    Correction applied to raw codes:  y_cal = (y - offset) * gain_corr.
    The skew term is *not* a digital coefficient: it is pushed back into the
    AD9695 sample-clock delay, so the loop closes through the hardware.
    """

    offset_a: float = 0.0
    offset_b: float = 0.0
    gain_corr_a: float = 1.0
    gain_corr_b: float = 1.0
    skew_target_ps: float = 0.0
    """Wanted B-minus-A sampling instant: 0 for parallel mode, Ts/2 for true
    2x interleaving."""

    skew_cmd_ps: float = 0.0
    """Accumulated delay commanded to channel B, in picoseconds."""

    mu_offset: float = 0.35
    mu_gain: float = 0.35
    mu_skew: float = 0.30

    iteration: int = 0

    def update(self, est: BlockEstimate) -> dict:
        """One block-LMS step.

        ``est`` must have been computed on data that already went through
        :meth:`apply`, so every quantity below is a *residual* error and the
        updates are incremental.  That is what makes the recorded trajectory a
        genuine closed-loop learning curve rather than a sequence of independent
        one-shot measurements.
        """
        e_off_a = est.ch_a.offset_codes
        e_off_b = est.ch_b.offset_codes

        # offset_x is subtracted before the gain correction, so a residual seen
        # after the gain stage has to be referred back through it.
        if np.isfinite(e_off_a) and abs(self.gain_corr_a) > 1e-9:
            self.offset_a += self.mu_offset * e_off_a / self.gain_corr_a
        if np.isfinite(e_off_b) and abs(self.gain_corr_b) > 1e-9:
            self.offset_b += self.mu_offset * e_off_b / self.gain_corr_b

        # Normalise both channels to their common mean gain.  Only the mismatch
        # is observable, so pinning the mean keeps the loop from drifting in
        # absolute scale.
        ga, gb = est.ch_a.gain_codes, est.ch_b.gain_codes
        e_gain = np.nan
        if np.isfinite(ga) and np.isfinite(gb) and abs(ga) > 1e-9 and abs(gb) > 1e-9:
            mean_g = 0.5 * (ga + gb)
            self.gain_corr_a *= 1.0 + self.mu_gain * (mean_g / ga - 1.0)
            self.gain_corr_b *= 1.0 + self.mu_gain * (mean_g / gb - 1.0)
            e_gain = gb / ga - 1.0

        e_skew = np.nan
        if np.isfinite(est.skew_mismatch_ps):
            e_skew = est.skew_mismatch_ps - self.skew_target_ps
            # Channel B sampled e_skew ps too late -> command that much less delay.
            self.skew_cmd_ps -= self.mu_skew * e_skew

        self.iteration += 1
        return {
            "offset_error_a": e_off_a,
            "offset_error_b": e_off_b,
            "gain_error": e_gain,
            "skew_error_ps": e_skew,
        }

    def apply(self, ch_a: np.ndarray, ch_b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        a = (np.asarray(ch_a, dtype=np.float64) - self.offset_a) * self.gain_corr_a
        b = (np.asarray(ch_b, dtype=np.float64) - self.offset_b) * self.gain_corr_b
        return a, b


def synthesize_dither(
    n_samples: int, n0: int, cfg: DitherConfig, gain_codes: float
) -> np.ndarray:
    """The injected dither as this channel saw it, in ADC codes.

    Needed because the dither is a real perturbation of the ADC input: leaving it
    in the record would put a wideband floor about 18 dB below the carrier and
    cap the measured SNDR at roughly that value, whatever the calibration did.
    Every dither-based converter subtracts the known injected sequence from the
    output before scoring, and the same applies here -- the amplitude is not
    assumed, it is the one the loop just measured.
    """
    if not np.isfinite(gain_codes):
        return np.zeros(n_samples)
    ref = dither_only_loop(cfg) / cfg.a_dither  # unit peak amplitude
    idx = (np.arange(n_samples) + n0) % cfg.n_adc_period
    return gain_codes * ref[idx]


def interleave(ch_a: np.ndarray, ch_b: np.ndarray) -> np.ndarray:
    """Assemble the 2x time-interleaved output stream."""
    n = min(ch_a.size, ch_b.size)
    out = np.empty(2 * n, dtype=np.float64)
    out[0::2] = ch_a[:n]
    out[1::2] = ch_b[:n]
    return out
