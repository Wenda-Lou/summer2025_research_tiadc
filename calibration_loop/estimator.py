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

from dataclasses import dataclass, field, replace

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


def _corr(x: np.ndarray, y: np.ndarray) -> float:
    """Normalised correlation of two equal-length streams."""
    if x.size != y.size or x.size < 8:
        return 0.0
    xs, ys = x - x.mean(), y - y.mean()
    d = float(np.sqrt((xs @ xs) * (ys @ ys)))
    return float(xs @ ys) / d if d > 0 else 0.0


def _lag_corr(x: np.ndarray, y: np.ndarray, lag: int) -> float:
    """Correlation of ``x`` against ``y`` delayed by ``lag`` samples.

    ``lag == 0`` is the plain same-instant correlation; ``lag == half`` pairs
    ``x[m + half]`` with ``y[m]`` and is how the half-group split error is
    detected: it asks whether the two streams line up better when one of them is
    slid by half a group.
    """
    if lag == 0:
        return _corr(x, y)
    if lag > 0:
        return _corr(x[lag:], y[:-lag])
    return _corr(x[:lag], y[-lag:])


def split_at(words: np.ndarray, rotation: int, group: int = 8, half: int = 4) -> dict:
    """Split the word stream into two channels at a given group phase."""
    body = words[rotation:]
    body = body[: (body.size // group) * group]
    if body.size < 4 * group:
        raise ValueError("capture too short to de-frame")
    g = body.reshape(-1, group)
    ch_a = g[:, :half].reshape(-1).astype(np.float64)
    ch_b = g[:, half:].reshape(-1).astype(np.float64)
    return {
        "rotation": rotation,
        "ch_a": ch_a,
        "ch_b": ch_b,
        "corr": _corr(ch_a, ch_b),
    }


def deframe(words: np.ndarray, group: int = 8, half: int = 4) -> dict:
    """Find the [A A A A B B B B] group boundary in the word stream.

    The DMA is restarted for every capture and the AXI-Stream from the JESD link
    runs continuously, so the group boundary lands at an arbitrary one of
    ``group`` phases.

    Phases ``r`` and ``r + half`` both produce a clean tone in both outputs, so
    tone purity cannot separate them -- they merely start on the other converter.
    They are still *not* interchangeable.  At ``r + half`` the two outputs no
    longer describe the same instants: they are offset by ``half`` samples, so the
    pair stops being two views of one sampling instant.  (Both streams stay
    spectrally clean -- a half-group mis-split slides them, it does not interleave
    them -- which is why purity never detected the fault.)

    The channels are physically parallel -- both converters sample the same
    instant -- so at the correct phase the two outputs are proportional and
    ``|corr(ch_a, ch_b)|`` is ~1.  At ``r + half`` the same converters are
    compared ``half`` samples apart, which scales the correlation by
    ``cos(2*pi*half*f0)``: 0.756 at the current geometry, and in general below 1
    unless ``half * f0`` lands on a multiple of a half cycle.  ``|corr|``
    therefore picks the phase that puts both streams on the same instants, and
    that phase is by construction the one starting on converter A (word positions
    0..3 of a beat).  No stored DC signature is involved, and the phase is stable
    from capture to capture.

    Returns ``lagged=True`` when the raw winner had to be replaced by the other
    half-group candidate; the returned ``ch_a``/``ch_b`` are always the aligned
    pair.
    """
    best = None
    for rot in range(group):
        try:
            cand = split_at(words, rot, group, half)
        except ValueError:
            continue
        cand["purity"] = _tone_purity(cand["ch_a"]) + _tone_purity(cand["ch_b"])
        cand["abs_corr"] = abs(cand["corr"])
        # Primary key: |correlation| (physical).  Tie-break: tone purity.
        cand["score"] = cand["abs_corr"] + 1e-3 * cand["purity"]
        if best is None or cand["score"] > best["score"]:
            best = cand

    if best is None:
        raise ValueError("capture too short to de-frame")

    # ``|corr|`` decides on its own only while ``cos(2*pi*half*f0) < 1``.  Test
    # the half-group lag explicitly instead of relying on that inequality: if
    # sliding one stream by ``half`` raises the correlation, this boundary is the
    # shifted candidate and the other one is the pair that shares its instants.
    aligned = abs(_lag_corr(best["ch_a"], best["ch_b"], 0))
    shifted = abs(_lag_corr(best["ch_a"], best["ch_b"], half))
    lagged = shifted > aligned
    if lagged:
        best = split_at(words, (best["rotation"] + half) % group, group, half)
        best["purity"] = _tone_purity(best["ch_a"]) + _tone_purity(best["ch_b"])
        best["abs_corr"] = abs(best["corr"])
        best["score"] = best["abs_corr"] + 1e-3 * best["purity"]

    best["lagged"] = bool(lagged)
    return best


def prepare_capture(
    raw_bytes: bytes | np.ndarray,
    cfg: DitherConfig,
    signature: dict | None = None,
) -> dict:
    """Turn one raw DMA buffer into two canonically ordered, upright channels.

    Two ambiguities have to be resolved on *every* capture, because the DMA is
    restarted each time and starts at an arbitrary point in a continuously
    running AXI-Stream:

    1. *Group phase.*  Which of the 8 word positions begins an
       ``[A A A A B B B B]`` group, and which of the two clean-splitting phases
       puts both streams on the same instants.  :func:`deframe` settles both.

    2. *Polarity.*  One branch of the rf splitter is inverted, so the two streams
       are anti-proportional.  The inverted one is flipped upright here.  That is
       not cosmetic: the loop's figure of merit is the *signed* ratio
       ``g_B / g_A`` (:class:`BlockEstimate.gain_ratio`), so an unflipped
       inverted branch reads as a near-1.0 mismatch with the wrong sign.

    Both markers are physical and frame-to-frame stable: the converters are
    parallel, so they sample the same instants and their streams are proportional
    up to that one inversion.  A stored DC level is never consulted -- its
    discriminating quantity (~8 codes) was smaller than its frame-to-frame drift
    (~5 codes), which is why the ordering it produced flipped at random.

    The half-group re-split that used to accompany a swap is gone.  It was never a
    harmless way to exchange the two converters: comparing them half a group
    apart means (a) the pair no longer shares one loop position ``n0``, and
    ``n0`` is a *single* value that :func:`estimate_block` slices **both**
    channels with, so channel B's dither windows land four samples off its own
    impulses; and (b) the reported correlation collapses from 1.00 to
    ``cos(2*pi*half*f0)`` = 0.756 at the current geometry.  The returned pair is
    now always that same two converters, aligned.

    ``signature`` is accepted and echoed for metadata compatibility only; nothing
    in the ordering depends on it any more.
    """
    words = unpack_words(raw_bytes)
    f0 = cfg.sig_cycles / cfg.n_adc_period

    view = deframe(words)
    a, b = view["ch_a"], view["ch_b"]

    # One branch is inverted; normalise it upright so both channels share one
    # sense.  The zero-lag correlation is the inversion marker: an inverted
    # branch makes it negative, and unlike a DC level the sign is stable.
    corr = float(view["corr"])
    if corr < 0:
        b = -b

    # Tone fit and loop alignment on the two final streams only -- the earlier
    # implementation ran them on every candidate rotation, which is both wasteful
    # and the reason the alignment it returned could belong to a stream that was
    # later discarded.
    align_a = align_to_loop(fit_tone(a, f0, refine=True)["residual"], cfg)
    align_b = align_to_loop(fit_tone(b, f0, refine=True)["residual"], cfg)

    # Both channels see the same DPG loop, so one alignment serves both; take
    # whichever correlation peak was stronger.
    n0 = align_a["n0"] if abs(align_a["peak"]) >= abs(align_b["peak"]) else align_b["n0"]

    return {
        "ch_a": a,
        "ch_b": b,
        "n0": n0,
        "rotation": view["rotation"],
        # True when deframe's raw winner was the half-group-shifted candidate and
        # had to be corrected -- i.e. the ordering ambiguity did occur here.
        "swapped": bool(view["lagged"]),
        # Pre-normalisation: negative is the healthy anti-proportional pair.
        "corr": corr,
        "abs_corr": abs(corr),
        "sign_a": align_a["sign"],
        "sign_b": align_b["sign"],
        "align_margin": float(min(align_a["margin"], align_b["margin"])),
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
    """Sampling-instant error in ADC sample periods (positive = samples late).

    Measured from observables that are insensitive to pulse *shape*: the main
    tone's fitted phase and the dither event centroid.  See
    :func:`skew_from_phase` and :func:`event_centroid_samples` for why the
    first-order template expansion cannot be used on real hardware."""

    phase_rad: float = np.nan
    """Fitted main-tone phase at the record origin [rad].

    Only the *difference* between the two channels is meaningful.  Available so
    the block estimator can turn it into a timing skew with a shared frequency."""

    skew_phase_samples: float = np.nan
    """Skew from the tone-phase observable alone (shape-independent)."""

    skew_centroid_samples: float = np.nan
    """Skew from the event-centroid observable alone (shape-independent)."""

    centroid_samples: float = np.nan
    """Amplitude-weighted centre of the dither events, in ADC samples."""

    tone_f0: float = np.nan
    """Fitted tone frequency actually used for :attr:`phase_rad` [cycles/sample]."""

    skew_ps: float = np.nan
    tone_amplitude: float = np.nan
    n_events_used: int = 0
    align_margin: float = np.nan
    align_n0: int = -1
    residual_rms: float = np.nan
    gain_sign_fit: float = np.nan
    """Sign the windowed fit produced before ``pin_polarity`` pinned it [+-1].

    Recorded rather than discarded: a frame that fits the opposite sign to the
    session anchor is telling you its local polarity balance was one-sided, which
    is worth counting even though the sign itself carries no per-frame
    information."""
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
    f0_override: float | None = None,
    polarity_sign: float = 1.0,
    pin_polarity: bool = False,
    accumulator: "JointEventAccumulator | None" = None,
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

    ``polarity_sign`` is the session-level absolute dither polarity: ``-1`` when
    the bench path presents the injected dither inverted with respect to
    ``polarity_sequence(cfg)``, which is what this hardware does (see
    :func:`polarity_anchor`).  It scales the polarity sequence, so the *gain*
    sign follows the bench instead of the model.  Offset and skew are unaffected:
    ``pbar`` flips together with ``p``, so the two terms of the joint solution
    cancel the flip exactly.  The record itself is never flipped -- the DC offset
    is a measurement in ADC codes, and inverting the data would invert it.

    With ``pin_polarity`` set, the fitted gain magnitude is kept but its sign is
    forced to the anchored convention, because that sign is a session constant
    rather than a per-frame measurement (see the comment at the assignment, and
    :attr:`ChannelEstimate.gain_sign_fit`).  Leave it off when the sign itself is
    what you are looking at -- the lag diagnostics rely on being able to see it.
    """
    y = np.asarray(y, dtype=np.float64)
    signs = polarity_sequence(cfg) * polarity_sign
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

        # Phase of the main tone at the record origin, on a frequency the caller
        # can hold common to both channels.  A time shift dt multiplies the tone
        # by exp(-j*2*pi*f*dt), so this phase is a direct, pulse-shape-independent
        # timing observable -- unlike the template derivative used below.
        phase_fit = fit if f0_override is None else fit_tone(
            y - dither_hat, f0_override, refine=False,
        )
        est.phase_rad = float(phase_fit["phase"])
        est.tone_f0 = float(phase_fit["f0"])

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
        v = (p * windows).mean(axis=0)
        u = windows.mean(axis=0)

        # The polarity sequence is balanced over the whole loop, but a capture
        # only sees a short run of consecutive events and their partial sum is
        # not zero -- over ~15 events it is typically +-4, i.e. a mean polarity
        # near 0.25.  That fraction of the dither leaks straight into the plain
        # average and would be read as offset.  Both statistics are therefore
        # solved jointly instead of being treated as already separated:
        #
        #   u[m] = o        + pbar * (G d[m])
        #   v[m] = pbar * o +        (G d[m])
        #
        # which inverts exactly, because pbar is known from the polarity of the
        # events actually present.
        pbar = float(signs[ks].mean())
        denom = 1.0 - pbar * pbar
        if denom < 1e-6:
            return est
        dither_profile = (v - pbar * u) / denom
        offset_profile = (u - pbar * v) / denom

        est.v_profile, est.u_profile = dither_profile, offset_profile

        gain = float(dither_profile @ d) / dd if dd > 0 else np.nan
        if pin_polarity and np.isfinite(gain):
            # The dither polarity of the bench path is a *session* constant, so a
            # frame that fits the opposite sign is not carrying information -- it
            # is the local polarity balance of the 7-8 visible events going
            # one-sided, which flips the fitted sign on a few per cent of frames
            # while |gain| stays consistent with its neighbours.  Estimate the
            # magnitude, pin the sign to the session anchor, and keep the raw sign
            # in ``gain_sign_fit`` so the flips stay countable.
            est.gain_sign_fit = 1.0 if gain >= 0 else -1.0
            gain = abs(gain)
        est.gain_codes = gain
        v = dither_profile
        u = offset_profile

        # Pooling hook: record only the final, tone-decoupled pass.  The previous
        # implementation added the same events once per pass (and skipped the
        # dither/tone refinement below), so a 75-event batch was reported as 150
        # events.  More importantly, that made the pooled statistic depend on the
        # estimator's implementation detail ``passes`` rather than on the data.
        if accumulator is not None:
            if pass_index + 1 < passes and np.isfinite(gain):
                dither_hat = synthesize_dither(
                    y.size, n0_used, cfg, gain,
                    polarity_sign=polarity_sign,
                )
            else:
                accumulator.add(starts, signs[ks], resid, m_lo, m_hi)
            est.n_events_used = int(ks.size)
            continue

        if np.isfinite(gain) and abs(gain) > 1e-9 and dpdp > 0:
            est.skew_samples = skew_prior_samples + float(
                (v - gain * d) @ dprime
            ) / (gain * dpdp)
            est.skew_ps = est.skew_samples / cfg.fs_adc * 1e12

        # Shape-independent companion observable: the amplitude-weighted centre of
        # the pulse energy over the visible events.  Recorded (not used) here; the
        # block estimator combines it with the tone phase.
        est.centroid_samples = event_centroid_samples(resid, starts, m_lo, m_hi)

        # Offset from the flat top only: those samples are insensitive to skew,
        # so a residual timing error cannot leak into the offset estimate.
        top = (m >= cfg.edge_r) & (m < cfg.edge_r + cfg.top_w)
        est.offset_codes = float(u[top].mean()) if np.any(top) else float(u.mean())

        if pass_index + 1 < passes and np.isfinite(gain):
            dither_hat = synthesize_dither(y.size, n0_used, cfg, gain,
                                           polarity_sign=polarity_sign)

    return est


@dataclass
class BlockEstimate:
    ch_a: ChannelEstimate
    ch_b: ChannelEstimate
    gain_ratio: float = np.nan
    """g_B / g_A -- the quantity a two-channel TI ADC actually needs."""

    tone_ratio: float = np.nan
    """|tone_B| / |tone_A| from the coherent fit of the main tone in each channel.

    The second, independent route to the same gain mismatch, and the one the A-B
    difference spur at f_in actually depends on.  It is fitted over the whole record
    (the dither is present, but it is ~30 dB down and sits on its own comb, not at
    f0), so it shares no arithmetic with the pulse-window amplitudes behind
    :attr:`gain_ratio`.  On this bench the two disagree by ~1.2 % -- per-channel
    dispersion of the narrow pulse replicas -- and the tone route is ~10x more
    precise (per-frame scatter 0.25 % against 2.3 %), which is why the loop may
    choose it as the observable the gain correction integrates.
    """

    offset_mismatch_codes: float = np.nan
    skew_mismatch_ps: float = np.nan
    rotation: int = -1

    skew_mismatch_samples: float = np.nan
    """Skew mismatch in ADC sample periods, from the shape-robust observables."""

    skew_phase_ps: float = np.nan
    """Skew mismatch from the tone-phase observable [ps]."""

    skew_diff_route_ps: float = np.nan
    """Skew mismatch from the A-B difference signal's tone amplitude [ps].

    A second, nearly fit-free route to the same quantity: if the two channels
    differ only by a delay, then ``a - b`` is a tone of amplitude
    ``2*A*|sin(pi*f*dt)|``, so ``dt = asin(amp_ab / (2*A)) / (pi*f)``.  It shares
    no arithmetic with the phase difference (one uses the phase of each channel's
    projection, the other the amplitude of the difference projection), which is
    what makes it useful evidence: on this bench the two agree to 0.29 ps rms
    over 100 frames, so a frame where they *disagree* is genuinely suspect rather
    than merely unlucky.  Recorded, never used to veto on its own."""

    skew_centroid_ps: float = np.nan
    """Skew mismatch from the event-centroid observable [ps].

    Recorded only: on this bench it scatters +-600 ps (it averages |residual|
    over 7-8 events), so it must never be allowed to veto a good phase estimate.
    """

    skew_source: str = "none"
    """Which observable produced :attr:`skew_mismatch_ps`.

    ``phase`` is the normal path, ``profile-fallback`` means both shape-robust
    observables were unavailable, and ``observables-disagree`` means they
    contradicted each other so the frame was discarded."""


def event_centroid_samples(resid: np.ndarray, starts: np.ndarray,
                           m_lo: int, m_hi: int) -> float:
    """Amplitude-weighted centre of the dither events, in ADC samples.

    This is a *shape-independent* timing observable.  A sampling-instant error
    ``dt`` translates the whole pulse in time, so the centroid of the pulse
    energy moves by exactly ``dt`` no matter what the pulse looks like.  Pulse
    dispersion, bandwidth limiting or any other symmetric distortion changes the
    width but not the centroid, so unlike a template-derivative fit this stays
    unbiased on real hardware.

    Only the samples that actually carry pulse energy are used, with the local
    baseline removed first.  Integrating across the whole event window instead
    would mix in ~20 samples of pure noise per event and, because channel B's
    residual is several times noisier than channel A's on this bench, that noise
    is what limits the observable.  Restricting the sum to the pulse and
    subtracting the window's outer samples as a baseline keeps the estimate
    anchored to the pulse itself.

    Returns NaN when there are too few usable events for the mean to be
    meaningful.
    """
    if starts.size < 3:
        return np.nan
    idx = starts[:, None] + np.arange(m_lo, m_hi + 1)[None, :]
    w = np.abs(resid[idx])
    m = np.arange(m_lo, m_hi + 1)
    # Pulse occupies [offset, offset + edge + top + edge); everything else in the
    # window is baseline.  Use the outer 25 % of the window on each side.
    span = m_hi - m_lo + 1
    guard = max(2, span // 4)
    base_mask = (m < m_lo + guard) | (m > m_hi - guard)
    pulse_mask = ~base_mask
    if not np.any(pulse_mask) or not np.any(base_mask):
        return np.nan
    w = w - w[:, base_mask].mean(axis=1, keepdims=True)
    w = np.clip(w, 0.0, None)          # energy above the local baseline only
    wp = w[:, pulse_mask]
    tot = wp.sum(axis=1)
    good = tot > 0
    if np.count_nonzero(good) < 3:
        return np.nan
    offs = m[pulse_mask].astype(np.float64)
    per_event = (wp[good] * offs[None, :]).sum(axis=1) / tot[good]
    return float(np.mean(per_event))


def wrap_to_half_period(delta: float, f0: float) -> float:
    """Map a time difference into +-1/(2*f0) seconds.

    The tone-phase observable is only defined modulo one cycle, so a candidate
    skew is only recoverable within half a tone period.  That is far wider than
    any physical mismatch on this bench (full on-chip delay range ~363 ps
    against a 5.01 ns period), so wrapping is unambiguous in practice.
    """
    if not np.isfinite(delta) or f0 <= 0:
        return np.nan
    period = 1.0 / f0
    return float(delta - period * np.round(delta / period))


def estimate_block(
    ch_a: np.ndarray,
    ch_b: np.ndarray,
    cfg: DitherConfig,
    cancel_signal: bool = True,
    n0: int | None = None,
    skew_prior_samples: float = 0.0,
    polarity_sign: float = 1.0,
    pin_polarity: bool = False,
) -> BlockEstimate:
    """Estimate both channels of the pair from one capture.

    Both channels must use the *same* loop position ``n0``: they see one DPG
    loop, so a per-channel alignment slip of one sample would show up as a
    2 ns skew mismatch that is not there.  When ``n0`` is not supplied, channel
    A's alignment is measured and reused for channel B.

    ``polarity_sign`` is one session-level value shared by both channels (see
    :func:`polarity_anchor`); a per-channel sign here would make ``gain_ratio``
    meaningless, since it is the ratio of the two signed gains.
    """
    # Pass 1: rough estimate of channel A, which also fixes the loop alignment.
    a = estimate_channel(ch_a, cfg, cancel_signal=cancel_signal, n0=n0,
                         polarity_sign=polarity_sign, pin_polarity=pin_polarity)

    # The ADC clock has a fixed but unknown sub-sample phase with respect to the
    # DPG loop, and it shows up identically in both channels.  Only the
    # difference is a mismatch, so channel A's own estimate is promoted to a
    # common prior and both channels are re-fitted around it.  Without this the
    # first-order expansion would be evaluated far from its centre whenever that
    # common phase happens to be large, and both estimates would degrade.
    common = a.skew_samples if np.isfinite(a.skew_samples) else 0.0
    a = estimate_channel(
        ch_a, cfg, cancel_signal=cancel_signal, n0=a.align_n0,
        skew_prior_samples=common, polarity_sign=polarity_sign,
        pin_polarity=pin_polarity,
    )
    b = estimate_channel(
        ch_b, cfg, cancel_signal=cancel_signal, n0=a.align_n0,
        skew_prior_samples=common + skew_prior_samples, polarity_sign=polarity_sign,
        pin_polarity=pin_polarity,
    )

    out = BlockEstimate(ch_a=a, ch_b=b)
    if np.isfinite(a.gain_codes) and abs(a.gain_codes) > 1e-9:
        out.gain_ratio = b.gain_codes / a.gain_codes
    if (np.isfinite(a.tone_amplitude) and np.isfinite(b.tone_amplitude)
            and abs(a.tone_amplitude) > 1e-9):
        out.tone_ratio = abs(b.tone_amplitude) / abs(a.tone_amplitude)
    out.offset_mismatch_codes = b.offset_codes - a.offset_codes

    # --- skew, from shape-independent observables ---------------------------
    #
    # The template-derivative estimate in estimate_channel is a first-order
    # expansion of d(m + dt).  On the bench it reports a ~620 ps offset that does
    # not move when the true skew changes, because the measured pulse profile is
    # not the ideal raised-cosine template: dispersion and bandwidth limiting
    # reshape it, and the fitting basis -- not the timing -- then drives the
    # answer.  Two observables survive that distortion because they depend only
    # on *when* energy arrives, not on the pulse form:
    #
    #   tone phase   : dt = -(phi_B - phi_A) / (2*pi*f), exact to first order in
    #                  dt and independent of pulse shape, but wraps every 1/f.
    #   event centroid: dt = centroid_B - centroid_A, unwrapped, less precise
    #                  because it averages |residual| over few events.
    #
    # Both need the two channels fitted at one common frequency: a frequency
    # difference would masquerade as a phase difference.
    ts_ps = 1e12 / cfg.fs_adc
    f_sh = 0.5 * (a.tone_f0 + b.tone_f0)
    if not np.isfinite(f_sh) or f_sh <= 0:
        f_sh = cfg.sig_cycles / cfg.n_adc_period
    if not (0.0 < f_sh < 0.5):
        f_sh = cfg.sig_cycles / cfg.n_adc_period

    a2 = estimate_channel(ch_a, cfg, cancel_signal=cancel_signal, n0=a.align_n0,
                          skew_prior_samples=common, f0_override=f_sh,
                          polarity_sign=polarity_sign, pin_polarity=pin_polarity)
    b2 = estimate_channel(ch_b, cfg, cancel_signal=cancel_signal, n0=a.align_n0,
                          skew_prior_samples=common + skew_prior_samples,
                          f0_override=f_sh, polarity_sign=polarity_sign,
                          pin_polarity=pin_polarity)
    a, b = a2, b2
    out.ch_a, out.ch_b = a, b

    dphi = b.phase_rad - a.phase_rad
    if np.isfinite(dphi):
        dphi = float(np.arctan2(np.sin(dphi), np.cos(dphi)))  # +-pi
        # fit_tone returns phase = arctan2(-b, a) with y = a*cos + b*sin, i.e.
        # the negated "signal peaks at n = phase/(2 pi f0)" angle.  Under that
        # convention a sampling delay dt *decreases* the phase, so the skew is
        # +dphi/(2 pi f) rather than -dphi/(2 pi f).  Verified against
        # BenchModel ground truth (skew_b = +3.6 ps -> +3.59 ps estimated).
        skew_phase = dphi / (2.0 * np.pi * f_sh)
        skew_phase = wrap_to_half_period(skew_phase, f_sh)
    else:
        skew_phase = np.nan
    a.skew_phase_samples = skew_phase
    b.skew_phase_samples = skew_phase

    # --- second, independent route to the same delay -------------------------
    #
    # a - b is itself a tone: if b is a delayed by dt then
    # a - b = 2A sin(pi f dt) cos(2 pi f t + ...), so its amplitude measures dt
    # without using either channel's phase.  This is recorded, not used to veto:
    # on this bench the two routes agree to 0.29 ps rms over 100 frames (corr
    # 1.0000), so a disagreement is real evidence that a frame is suspect -- and
    # conversely, frames the old agreement logic threw away because the *centroid*
    # disagreed can be recovered when phase and this route agree.
    d_ab = np.asarray(ch_a, dtype=np.float64) - np.asarray(ch_b, dtype=np.float64)
    amp_ref = a.tone_amplitude if np.isfinite(a.tone_amplitude) else b.tone_amplitude
    if d_ab.size > 8 and np.isfinite(amp_ref) and amp_ref > 0:
        amp_ab = abs(fit_tone(d_ab - d_ab.mean(), f_sh, refine=False)["amplitude"])
        ratio = min(1.0, amp_ab / (2.0 * amp_ref))
        out.skew_diff_route_ps = float(
            -2.0 * np.arcsin(ratio) / (2.0 * np.pi * f_sh) * ts_ps
        )

    if np.isfinite(a.centroid_samples) and np.isfinite(b.centroid_samples):
        skew_centroid = float(b.centroid_samples - a.centroid_samples)
    else:
        skew_centroid = np.nan
    a.skew_centroid_samples = skew_centroid
    b.skew_centroid_samples = skew_centroid

    # --- resolve the half-tone-period ambiguity -----------------------------
    #
    # The two channels sit at a fixed +-pi relation, so the tone-phase observable
    # determines the skew only modulo half a tone period: on the bench the phase
    # estimate lands on a small set of values separated by exactly T/2 = 2507.7 ps
    # (measured: -1924 ps and +577 ps on successive frames).  Phase alone cannot
    # choose between them.
    #
    # Two things can break the tie, and they are NOT equally good:
    #
    #   skew_prior_samples -- the skew the controller has already commanded.  The
    #       true mismatch is prior + residual and the residual is small, so the
    #       prior is within a few ps of the answer and resolves the branch
    #       unambiguously.  This is the closed-loop case.
    #   the event centroid -- an absolute event-time difference, available when
    #       nothing has been commanded (the probe).  It has no ambiguity but only
    #       ~600 ps resolution against a 1253.8 ps half-period decision, so it is
    #       a weak arbiter and must not override a good prior.
    #
    # Order matters: letting the centroid win made a controller transient select
    # the wrong branch and the closed loop diverged (sim gain ratio 1.0004 ->
    # 0.767 at iteration 36).
    half_period_s = 0.5 / f_sh if f_sh > 0 else np.nan
    skew_samples = skew_phase
    branch_by = "none"
    if np.isfinite(skew_phase) and np.isfinite(half_period_s):
        cands = [skew_phase - half_period_s, skew_phase, skew_phase + half_period_s]
        if abs(skew_prior_samples) > 1e-12:
            skew_samples = min(cands, key=lambda c: abs(c - skew_prior_samples))
            branch_by = "prior"
        elif np.isfinite(skew_centroid):
            skew_samples = min(cands, key=lambda c: abs(c - skew_centroid))
            branch_by = "centroid"

    skew_source = "phase"
    if not (np.isfinite(skew_samples) and abs(skew_samples) < 2.0):
        skew_samples = a.skew_samples if np.isfinite(a.skew_samples) else np.nan
        skew_source = "profile-fallback"
    elif (
        np.isfinite(skew_centroid)
        and abs(skew_samples - skew_centroid) > 1.0
        and branch_by != "prior"
    ):
        # The centroid only gets to veto when it was also the branch arbiter.  If
        # the commanded skew chose the branch, a centroid disagreement is expected
        # -- the centroid is the coarser observable -- and must not discard an
        # otherwise good frame, or the closed loop loses its update.
        skew_samples = np.nan
        skew_source = "observables-disagree"
    elif np.isfinite(skew_phase) and np.isfinite(half_period_s):
        if abs(skew_samples - skew_phase) > 1e-12:
            skew_source = f"phase+branch({branch_by})"

    out.skew_mismatch_samples = skew_samples
    out.skew_mismatch_ps = skew_samples * ts_ps if np.isfinite(skew_samples) else np.nan
    out.skew_source = skew_source
    out.skew_phase_ps = skew_phase * ts_ps if np.isfinite(skew_phase) else np.nan
    out.skew_centroid_ps = (
        skew_centroid * ts_ps if np.isfinite(skew_centroid) else np.nan
    )

    # Per-channel fields are kept consistent with the mismatch so that logging and
    # any downstream "b - a" reconstruction still agree.  Only the *difference* of
    # these two is physically meaningful; the common part is the unknown ADC-clock
    # phase against the DPG loop and is deliberately parked on channel A.
    a.skew_samples = 0.0 if np.isfinite(skew_samples) else np.nan
    a.skew_ps = 0.0 if np.isfinite(skew_samples) else np.nan
    b.skew_samples = skew_samples
    b.skew_ps = out.skew_mismatch_ps
    return out


# ---------------------------------------------------------------------------
# Cross-frame joint aggregation
# ---------------------------------------------------------------------------

class JointEventAccumulator:
    """Pool dither event windows from many captures before fitting amplitude.

    A single 4095-byte capture holds only ``floor(n_samples / slot_period)`` whole
    dither events -- 7 or 8 with the bench geometry (1020 samples, 130-sample
    slot).  Fitting gain from 7 events leaves ~9.6 % frame-to-frame scatter on
    channel B, far above the 2 % the sanity check wants, and no amount of tuning
    inside one capture can fix that: the estimate is variance-limited by the
    number of events, and the count is capped by the frame length.

    The bench firmware solves the same problem by stacking event windows across
    frames (see ``adc_cal_skew_estimate_joint_frames``).  Aggregation helps for a
    second, subtler reason too: each capture starts at an arbitrary point in the
    DPG loop, so consecutive frames expose *different* events and therefore
    different slices of the balanced polarity sequence.  Pooling them makes the
    local polarity sum average towards zero instead of leaving whichever
    imbalance one short window happened to have.

    Only the amplitude statistics are pooled here.  Skew is *not* taken from the
    pooled profile, because the ADC-clock sub-sample phase differs slightly from
    frame to frame and would smear the pooled pulse shape; the per-frame skew
    observables are combined robustly by :func:`estimate_block_joint` instead.
    """

    def __init__(self, m_lo: int, m_hi: int):
        self.m_lo = int(m_lo)
        self.m_hi = int(m_hi)
        self.span = self.m_hi - self.m_lo + 1
        self.n_events = 0
        self.n_frames = 0
        # sum over events of (polarity * window) and of (window)
        self._v = np.zeros(self.span, dtype=np.float64)
        self._u = np.zeros(self.span, dtype=np.float64)
        self._pol_sum = 0.0

    def add(self, starts, pol, resid, m_lo, m_hi) -> None:
        if starts.size == 0:
            return
        idx = starts[:, None] + np.arange(m_lo, m_hi + 1)[None, :]
        w = resid[idx]
        p = np.asarray(pol, dtype=np.float64)[:, None]
        self._v += (p * w).sum(axis=0)
        self._u += w.sum(axis=0)
        self._pol_sum += float(np.sum(p))
        self.n_events += int(starts.size)
        self.n_frames += 1

    @property
    def pbar(self) -> float:
        return self._pol_sum / self.n_events if self.n_events else 0.0

    def profiles(self) -> tuple[np.ndarray, np.ndarray]:
        """Polarity-weighted and plain averaged profiles over all pooled events."""
        if self.n_events == 0:
            return np.zeros(self.span), np.zeros(self.span)
        return self._v / self.n_events, self._u / self.n_events


def estimate_block_joint(
    frames: list[tuple[np.ndarray, np.ndarray, int]],
    cfg: DitherConfig,
    cancel_signal: bool = True,
    skew_prior_samples: float = 0.0,
    polarity_sign: float = 1.0,
    pin_polarity: bool = False,
) -> BlockEstimate:
    """Joint estimate over several captures.

    ``frames`` is a list of ``(ch_a, ch_b, n0)`` triples, all from the same
    register state.  Amplitude and offset come from the pooled event windows;
    skew is the robust median of the per-frame, branch-resolved estimates, which
    rejects the occasional frame whose half-period branch was mis-selected.
    """
    if not frames:
        raise ValueError("no frames supplied")

    m, _, _ = adc_templates(cfg)
    d = pulse(m, cfg.edge_r, cfg.top_w)
    dd = float(d @ d)

    acc_a = JointEventAccumulator(int(m[0]), int(m[-1]))
    acc_b = JointEventAccumulator(int(m[0]), int(m[-1]))

    per_frame = []
    for ch_a, ch_b, n0 in frames:
        a1 = estimate_channel(ch_a, cfg, cancel_signal=cancel_signal, n0=n0,
                              polarity_sign=polarity_sign, pin_polarity=pin_polarity,
                              accumulator=acc_a)
        b1 = estimate_channel(ch_b, cfg, cancel_signal=cancel_signal, n0=n0,
                              polarity_sign=polarity_sign, pin_polarity=pin_polarity,
                              accumulator=acc_b)
        # Full per-frame pass (no accumulator) supplies gain/offset/skew/phase.
        est = estimate_block(ch_a, ch_b, cfg, cancel_signal=cancel_signal,
                             n0=a1.align_n0, skew_prior_samples=skew_prior_samples,
                             polarity_sign=polarity_sign, pin_polarity=pin_polarity)
        per_frame.append(est)
        # Keep the shared alignment consistent for the accumulator passes.
        if a1.align_n0 != est.ch_a.align_n0:
            a1.align_n0 = est.ch_a.align_n0

    va, ua = acc_a.profiles()
    vb, ub = acc_b.profiles()

    def _fit(acc, v, u):
        pbar = acc.pbar
        denom = 1.0 - pbar * pbar
        if denom < 1e-6:
            denom = 1.0
        dith = (v - pbar * u) / denom
        off = (u - pbar * v) / denom
        gain = float(dith @ d) / dd if dd > 0 else float("nan")
        sign_fit = float(np.sign(gain)) if np.isfinite(gain) else np.nan
        if pin_polarity and np.isfinite(gain):
            # Pooling hundreds of events makes this sign far more reliable than the
            # per-frame one, but it is still a session constant, so pin it to the
            # anchored convention and keep the raw sign for the record.
            gain = abs(gain)
        top = (m >= cfg.edge_r) & (m < cfg.edge_r + cfg.top_w)
        offset = float(off[top].mean()) if np.any(top) else float(off.mean())
        return gain, offset, sign_fit

    ga, oa, sign_a_fit = _fit(acc_a, va, ua)
    gb, ob, sign_b_fit = _fit(acc_b, vb, ub)

    last = per_frame[-1]
    a = replace(last.ch_a)
    b = replace(last.ch_b)
    a.gain_codes, a.offset_codes, a.gain_sign_fit = ga, oa, sign_a_fit
    b.gain_codes, b.offset_codes, b.gain_sign_fit = gb, ob, sign_b_fit
    a.n_events_used = acc_a.n_events
    b.n_events_used = acc_b.n_events

    out = BlockEstimate(ch_a=a, ch_b=b)
    out.gain_ratio = gb / ga if np.isfinite(ga) and abs(ga) > 1e-9 else np.nan
    out.offset_mismatch_codes = ob - oa

    # Skew: robust combination of the per-frame branch-resolved values.  The
    # median rejects frames whose half-period branch was mis-selected, which is
    # the dominant failure mode for this observable (it is defined modulo T/2).
    ts_ps = 1e12 / cfg.fs_adc

    # Pooled centroid: the per-frame centroid is the only unambiguous timing
    # observable, but averaging |residual| over just 7-8 events leaves it with
    # several hundred ps of scatter -- too coarse to arbitrate a +-1253.8 ps
    # half-period branch.  Pooling all frames' centroids cuts that by ~sqrt(N),
    # which is what makes branch selection reliable without a commanded-skew
    # prior (the probe case).
    cent_vals = [e.skew_centroid_ps for e in per_frame
                 if np.isfinite(e.skew_centroid_ps)]
    skew_centroid_joint_ps = float(np.median(cent_vals)) if cent_vals else np.nan
    skew_centroid_joint = (skew_centroid_joint_ps / ts_ps
                           if np.isfinite(skew_centroid_joint_ps) else np.nan)

    half_s = 0.5 / cfg.f_sig if cfg.f_sig > 0 else np.nan

    def _branch(x):
        if not np.isfinite(x) or not np.isfinite(half_s):
            return x
        return x - half_s * np.round(x / half_s)

    resolved = []
    for e in per_frame:
        ph = (e.skew_phase_ps / ts_ps if np.isfinite(e.skew_phase_ps) else np.nan)
        if np.isfinite(ph) and np.isfinite(half_s):
            cands = [ph - half_s, ph, ph + half_s]
            if skew_prior_samples != 0.0:
                resolved.append(min(cands, key=lambda c: abs(c - skew_prior_samples)))
            elif np.isfinite(skew_centroid_joint):
                resolved.append(min(cands, key=lambda c: abs(c - skew_centroid_joint)))
            else:
                resolved.append(_branch(ph))
        elif np.isfinite(e.skew_mismatch_samples):
            resolved.append(e.skew_mismatch_samples)
    vals = [v for v in resolved if np.isfinite(v)]
    skew_samples = float(np.median(vals)) if vals else np.nan

    out.skew_mismatch_samples = skew_samples
    out.skew_mismatch_ps = skew_samples * ts_ps if np.isfinite(skew_samples) else np.nan
    ph = [e.skew_phase_ps for e in per_frame if np.isfinite(e.skew_phase_ps)]
    out.skew_phase_ps = float(np.median(ph)) if ph else np.nan
    out.skew_centroid_ps = skew_centroid_joint_ps
    out.skew_source = (
        f"joint(n={len(vals)}/{len(per_frame)},cent={len(cent_vals)})"
    )
    out.rotation = last.rotation
    a.skew_samples = 0.0 if np.isfinite(skew_samples) else np.nan
    a.skew_ps = 0.0 if np.isfinite(skew_samples) else np.nan
    b.skew_samples = skew_samples
    b.skew_ps = out.skew_mismatch_ps
    return out


# ---------------------------------------------------------------------------
# Batch skew: accept frames, then average arithmetically
# ---------------------------------------------------------------------------

@dataclass
class SkewBatch:
    """Skew from a batch of frames, after the acceptance filter."""

    mean_ps: float = np.nan
    se_ps: float = np.nan
    """Standard error of the mean, ``std / sqrt(n_used)``.

    The arithmetic mean is the right estimator here, not a median or a trimmed
    mean: a 50 000-draw bootstrap over 100 bench frames gives SE 4.23 ps for the
    mean at N = 20 against 6.94 ps for the median and 5.04 ps for a 10 % trimmed
    mean.  The scatter is close to uniform rather than heavy-tailed, so discarding
    samples only throws information away."""

    ci95_half_ps: float = np.nan
    """``1.96 * se``.  The bootstrap distribution of the mean is normal to the
    resolution that matters, so the normal approximation is adequate."""

    n_total: int = 0
    n_used: int = 0
    route_center_ps: float = np.nan
    """Median of (phase route - difference route) for this batch.

    The two routes carry a small state-dependent offset, so the agreement test
    compares each frame against the batch centre rather than against zero."""

    route_bound_ps: float = np.nan
    """Rejection bound actually used, in picoseconds, from the batch's own MAD."""

    rejected: dict = field(default_factory=dict)
    """Reason -> count, for judging the yield of a batch."""


def skew_batch(frames, tone_tol: float = 0.10, resid_lo: float = 0.5,
               resid_hi: float = 2.0, margin_min: float = 6.0,
               agree_ps: float | None = None, agree_k: float = 6.0,
               agree_floor_ps: float = 1.5) -> SkewBatch:
    """Arithmetic-mean skew over the frames of one batch that pass the filter.

    ``frames`` is a sequence of mappings with ``phase_ps``, ``diff_route_ps``,
    ``source``, ``tone``, ``resid`` and ``margin``.

    Acceptance, in order of what actually matters:

    * ``phase_ps`` finite -- the phase route is the precise one;
    * ``phase_ps`` and ``diff_route_ps`` agree -- two nearly independent routes to
      the same delay, so a frame where they part company is suspect;
    * tone amplitude within ``tone_tol`` of the batch median, residual within
      ``resid_lo..resid_hi`` of it, ``margin >= margin_min``;
    * ``source != 'profile-fallback'`` -- that path uses the centroid observable,
      which scatters +-600 ps on this bench.

    The route-agreement test is *relative*, not a fixed number of picoseconds.
    The two routes carry a small state-dependent offset (measured: 0.69 ps median
    disagreement before the actuator was initialized, 4.55 ps in the fine-delay
    neutral state, where the spread is 6.2 ps maximum).  A fixed 5 ps threshold
    rejected 48 of 100 perfectly healthy neutral-state frames.  What matters is a
    frame that departs from the *batch's* behaviour, so the test is: reject when
    ``|(phase - diff) - median(phase - diff)|`` exceeds ``agree_k`` robust sigmas
    (1.4826 * MAD) plus a floor.  Pass ``agree_ps`` to pin it to an absolute value
    instead.

    Deliberately NOT used: a veto from ``skew_source == 'observables-disagree'``.
    That verdict came from the unusable centroid contradicting a good phase
    estimate; measured over 100 frames, 19 of the 22 such frames are perfectly
    good by the criteria above, and rejecting them cost 6 points of yield.
    The centroid is recorded, never allowed to veto.
    """
    rows = list(frames)
    out = SkewBatch(n_total=len(rows))
    if not rows:
        return out

    def val(row, key):
        try:
            return float(row.get(key, np.nan))
        except (TypeError, ValueError):
            return np.nan

    tones = [val(r, "tone") for r in rows]
    resids = [val(r, "resid") for r in rows]
    tone_med = float(np.nanmedian(tones)) if np.any(np.isfinite(tones)) else np.nan
    resid_med = float(np.nanmedian(resids)) if np.any(np.isfinite(resids)) else np.nan

    # self-calibrating route-agreement bound for this batch
    deltas = np.array([val(r, "phase_ps") - val(r, "diff_route_ps") for r in rows])
    deltas = deltas[np.isfinite(deltas)]
    if agree_ps is not None:
        agree_center = 0.0
        agree_bound = float(agree_ps)
    elif deltas.size >= 5:
        agree_center = float(np.median(deltas))
        mad = float(np.median(np.abs(deltas - agree_center))) * 1.4826
        agree_bound = max(agree_floor_ps, agree_k * mad)
    else:
        agree_center = 0.0
        agree_bound = agree_floor_ps
    out.route_center_ps = agree_center
    out.route_bound_ps = agree_bound

    kept: list[float] = []
    why: dict = {}
    for row in rows:
        phase = val(row, "phase_ps")
        diff = val(row, "diff_route_ps")
        tone = val(row, "tone")
        resid = val(row, "resid")
        margin = val(row, "margin")
        source = str(row.get("source", ""))

        if source == "profile-fallback":
            why["profile-fallback source"] = why.get("profile-fallback source", 0) + 1
            continue
        if not np.isfinite(phase):
            why["phase not finite"] = why.get("phase not finite", 0) + 1
            continue
        if np.isfinite(diff) and abs((phase - diff) - agree_center) > agree_bound:
            why["route outlier"] = why.get("route outlier", 0) + 1
            continue
        if np.isfinite(tone_med) and np.isfinite(tone) \
                and abs(tone - tone_med) > tone_tol * abs(tone_med):
            why["tone out of range"] = why.get("tone out of range", 0) + 1
            continue
        if np.isfinite(resid_med) and np.isfinite(resid) \
                and not (resid_lo * resid_med < resid < resid_hi * resid_med):
            why["residual out of range"] = why.get("residual out of range", 0) + 1
            continue
        if not (np.isfinite(margin) and margin >= margin_min):
            why["margin below threshold"] = why.get("margin below threshold", 0) + 1
            continue
        kept.append(phase)

    out.rejected = why
    out.n_used = len(kept)
    if kept:
        arr = np.asarray(kept, dtype=float)
        out.mean_ps = float(arr.mean())
        if arr.size > 1:
            out.se_ps = float(arr.std(ddof=1) / np.sqrt(arr.size))
            out.ci95_half_ps = float(1.96 * out.se_ps)
    return out


# ---------------------------------------------------------------------------
# Block LMS state
# ---------------------------------------------------------------------------

def gain_observable_pair(est: BlockEstimate, source: str = "tone"):
    """The (A, B) amplitudes whose ratio *is* the gain error, and which one was used.

    ``"tone"`` reads :attr:`ChannelEstimate.tone_amplitude` (the coherent main-tone
    fit), ``"dither"`` reads :attr:`ChannelEstimate.gain_codes` (the pulse-window
    amplitudes).  A source whose amplitudes are unusable falls back to the other, and
    the name actually used comes back with the pair so a run can be audited after the
    fact — a silent fallback would look like a converged gain loop.
    """
    candidates = (
        ("tone", (abs(est.ch_a.tone_amplitude), abs(est.ch_b.tone_amplitude))),
        ("dither", (est.ch_a.gain_codes, est.ch_b.gain_codes)),
    )
    if source != "tone":
        candidates = (candidates[1], candidates[0])
    for name, (x, y) in candidates:
        if np.isfinite(x) and np.isfinite(y) and abs(x) > 1e-9 and abs(y) > 1e-9:
            return float(x), float(y), name
    return np.nan, np.nan, "none"


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

    def update(self, est: BlockEstimate, integrate_skew: bool = True,
               gain_observable: str = "tone") -> dict:
        """One block-LMS step.

        ``est`` must have been computed on data that already went through
        :meth:`apply`, so every quantity below is a *residual* error and the
        updates are incremental.  That is what makes the recorded trajectory a
        genuine closed-loop learning curve rather than a sequence of independent
        one-shot measurements.

        ``integrate_skew=False`` leaves :attr:`skew_cmd_ps` untouched.  With the
        actuator disabled (``--open-skew``) the command is never written to the
        hardware, so integrating it only makes the log misleading: a 200-frame
        measurement-only run reached ``skew_cmd_ps = 5637 ps`` against a hardware
        authority of 165 ps while nothing was ever programmed.
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
        # absolute scale.  *Which* observable is the caller's choice (see
        # LoopOptions.gain_observable): the pulse-window amplitudes or the coherent
        # main-tone amplitudes, both measured on the same block.
        ga, gb, gain_source = gain_observable_pair(est, gain_observable)
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
            if integrate_skew:
                self.skew_cmd_ps -= self.mu_skew * e_skew

        self.iteration += 1
        return {
            "offset_error_a": e_off_a,
            "offset_error_b": e_off_b,
            "gain_error": e_gain,
            "gain_source": gain_source,
            "skew_error_ps": e_skew,
        }

    def apply(self, ch_a: np.ndarray, ch_b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        a = (np.asarray(ch_a, dtype=np.float64) - self.offset_a) * self.gain_corr_a
        b = (np.asarray(ch_b, dtype=np.float64) - self.offset_b) * self.gain_corr_b
        return a, b


def synthesize_dither(
    n_samples: int, n0: int, cfg: DitherConfig, gain_codes: float,
    polarity_sign: float = 1.0,
) -> np.ndarray:
    """The injected dither as this channel saw it, in ADC codes.

    Needed because the dither is a real perturbation of the ADC input: leaving it
    in the record would put a wideband floor about 18 dB below the carrier and
    cap the measured SNDR at roughly that value, whatever the calibration did.
    Every dither-based converter subtracts the known injected sequence from the
    output before scoring, and the same applies here -- the amplitude is not
    assumed, it is the one the loop just measured.

    ``polarity_sign`` must be the *same* value that produced ``gain_codes``
    (:func:`polarity_anchor`).  A measured gain is expressed in the polarity
    convention it was fitted in, so mixing conventions here turns the subtraction
    into an addition and doubles the dither instead of removing it.
    """
    if not np.isfinite(gain_codes):
        return np.zeros(n_samples)
    ref = dither_only_loop(cfg) / cfg.a_dither  # unit peak amplitude
    idx = (np.arange(n_samples) + n0) % cfg.n_adc_period
    return polarity_sign * gain_codes * ref[idx]


def polarity_anchor(prep: dict) -> float:
    """Session-level dither polarity sign from one prepared capture.

    The DPG loop and the analog path between it and the ADC are one physical
    chain, so whether the injected impulses reach the converter upright or
    inverted is a fixed property of the bench -- not of a frame.  It is measured
    here from channel A's own loop alignment (a full-record correlation against
    the known dither train, and therefore the most robust observable available)
    and then held for the whole session.

    Why channel A: it is the reference branch.  ``prepare_capture`` normalises
    the inverted branch to match it, so A is the branch whose sign is untouched
    by that normalisation, and it is the branch whose word positions the
    firmware also calls channel A.

    Applying the returned sign scales the estimator's polarity *model*
    (``p[k] -> sign * p[k]``); the record is deliberately left alone, because the
    DC offset is a measurement in ADC codes and flipping the data would flip it.
    """
    return -1.0 if float(prep.get("sign_a", 1.0)) < 0 else 1.0


def interleave(ch_a: np.ndarray, ch_b: np.ndarray) -> np.ndarray:
    """Assemble the 2x time-interleaved output stream."""
    n = min(ch_a.size, ch_b.size)
    out = np.empty(2 * n, dtype=np.float64)
    out[0::2] = ch_a[:n]
    out[1::2] = ch_b[:n]
    return out
