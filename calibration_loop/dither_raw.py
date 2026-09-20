"""Tone-free raw measurement of the three channel mismatches, from the impulses alone.

Every number here comes from folding the captured record on the impulse period with the
*known* polarity train, which is what makes the impulses usable without a reference tone:

  * gain    -- the per-event peak-to-peak magnitude ratio, sign-free, so no session polarity
               convention enters.  It agrees with the polarity-corrected folded replica ratio to
               0.02 % (measured 2026-09-19) while the estimator's ``gain_ratio`` sits ~3 % away,
               so this is the route to quote.
  * offset  -- the record mean difference B - A, in ADC codes.  The tone-free equivalent of what
               the estimator gets from the pulse-window flat top, and it needs no tone to begin
               with.
  * skew    -- the *phase* route: fit each channel's folded replica against the known pulse
               template **evaluated at a fractional sampling phase**, and difference the two
               fitted phases.  This is the only sub-sample route: the sampling grid is 769 ps, so
               a 10 ps shift barely moves the samples themselves.

The tone-phase and centroid routes the estimator uses for skew need a tone (the first fits the
tone's phase, the second is listed as unusable at +-600 ps), which is why this module exists: a
tone-free loop has to take its timing from here.

Kept in ``calibration_loop`` rather than in ``tools/`` so the bench tooling and the loop share
one implementation.  Two implementations of one estimator is exactly how the host and firmware
paths drifted apart.

Two things about the timing route are easy to get wrong, and both cost a debugging session on
2026-09-20 -- they are stated once here because the code around them is short:

**Sign convention.**  There are two conventions and they differ by a sign.  ``dither_raw``
reports the *sampling-instant* error, the one the loop integrates and the one
:class:`~calibration_loop.estimator.ChannelEstimate.skew_samples` documents: positive means
channel B's sampling instant is *later* than channel A's, which is what ``CalibrationState``
converts into "command B less delay".  The alternative -- the shift of B's *index* sequence
against A's -- has the opposite sign, because sampling later means the sequence runs ahead.  A
route written in the index convention and fed to the loop closes the loop with the wrong sign
and the actuator walks away from the target: measured in the model on 2026-09-20, the commanded
delay ran 0 -> 390 ps while the residual grew to -371 ps, with 81 % of captures rejected on the
way.  The model's ground truth is the arbiter, and the check is cheap: put a known delay on
channel B and require the route to read it back with the right sign.

**The template must be evaluated at a fractional phase.**  The pulse expressible at this
geometry is six ADC samples wide (two-sample edges, the generator's floor), so an edge is
covered by two samples and its derivative is badly under-sampled: projecting the folded A-B
difference onto the *integer*-sampled derivative -- the obvious "slope route" -- is nonlinear
long before a quarter sample.  Measured in the model at a bench-like replica amplitude (200
codes pk-pk, 3 codes rms noise, 25 frames per point): the integer-sample projection reads 2.2 ps
for a true 3.6 ps, 86.6 ps for 103.6 ps and 165.9 ps for 203.6 ps, with 15-25 ps per-frame
scatter, while the fractional-phase fit below reads 4.0 / 102.7 / 205.0 ps with 4.5-6.6 ps
scatter.  The template is known in closed form, so there is no reason to approximate it on the
sample grid: evaluate it where it is being compared.

The fitted phase is the *sampling* phase of the replica, so a difference between the channels is
a sampling-instant difference and the common part -- the ADC clock's phase against the DPG loop,
which is not a channel mismatch -- cancels in the difference.  ``skew_fold_ps`` keeps the older
projection available as an independent cross-check; on a healthy bench the two agree, and a
disagreement is a frame worth looking at.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .dither import DitherConfig, polarity_sequence, pulse, pulse_derivative
from .estimator import align_to_loop, synthesize_dither, visible_events

GUARD = 2          # samples of margin either side of the pulse in the fold window


@dataclass
class DitherRawEstimate:
    """One capture, measured from the impulses only."""

    gain_mag_ratio: float = np.nan
    """|B| / |A| from the per-event peak-to-peak magnitudes.  Sign-free: quote this one."""

    gain_fold_ratio: float = np.nan
    """|B| / |A| from the polarity-corrected folded replicas.  Shares the session sign."""

    offset_codes: float = np.nan
    """mean(B) - mean(A), the loop's sign convention."""

    skew_slope_ps: float = np.nan
    """Timing from the phase route, in ps.

    **Sampling-instant convention: positive means B samples later than A** -- the same
    convention the loop integrates (see the module docstring on why the sign is a trap).
    """

    skew_fold_ps: float = np.nan
    """The same delay from the folded A-B difference projected onto the pulse slope.

    An independent route to the same number, kept as a cross-check.  It is the estimator the
    module used before 2026-09-20 and it is nonlinear beyond ~0.1 sample (see the module
    docstring), so it is reported, never acted on.
    """

    phase_a: float = np.nan
    phase_b: float = np.nan
    """Fitted fractional sampling phase of each channel's replica, in ADC samples.

    The common part is the ADC clock's phase against the DPG loop (not a mismatch); only the
    difference is a channel property, which is why the two are reported separately.
    """

    mag_a: float = np.nan
    mag_b: float = np.nan
    """Mean peak-to-peak of the impulse windows, in ADC codes.

    The per-event route, and what the loop's ``dither_mag`` observable is built from.  It
    carries a noise bias at small amplitudes -- the range of ten noisy samples is a few sigma
    wide whatever is inside them -- so use :attr:`rep_pp_a` when quoting an amplitude.
    """

    rep_pp_a: float = np.nan
    rep_pp_b: float = np.nan
    """Folded replica peak-to-peak, in ADC codes.

    The amplitude to quote.  Folding averages the noise over the ~8 visible events before the
    range is taken, so unlike :attr:`mag_a` it does not grow with the noise floor.
    """

    peak_a: float = np.nan
    """One-sided peak of channel A's folded replica above its baseline, in ADC codes.

    This is the amplitude convention the professor asked for: one-sided peak against the 14-bit
    code span, so a 296-code peak is 1.8 % of 16384."""

    fwhm_a: float = np.nan
    """Full width at half maximum of the folded replica, in ADC samples."""

    slope_peak_a: float = np.nan
    """Steepest slope of the folded replica, codes per sample -- what the timing route gains."""

    dbc_ab_coherent: float = np.nan
    """Coherent dither power in A-B relative to channel A, in dBc.

    The tone-free analogue of the A-B difference spur: both terms are the *coherent* (folded)
    dither power, so a matched pair tends to -inf and a residual mismatch shows up as a finite
    number.  Comparable run to run, not to a tone-referenced dBc."""

    snr_dither_db: float = np.nan
    """Coherent dither power over what is left after subtracting it, channel A, in dB.

    The injected sequence is known, so it can be subtracted; what remains is the converter plus
    the bench.  This is the honest tone-free stand-in for an SNDR."""

    align_margin: float = np.nan
    n0: int = -1
    valid: bool = False
    reasons: list = field(default_factory=list)


def fold_replica(x: np.ndarray, cfg: DitherConfig, n0: int | None = None,
                 sign: float | None = None):
    """Sign-corrected co-add of the impulses in one record.

    ``n0`` must come from channel A and be reused for channel B: a per-channel alignment would
    absorb the very timing difference being measured.  Returns
    ``(replica, m, n0, sign, margin, magnitude)``.
    """
    align = align_to_loop(x, cfg)
    if n0 is None:
        n0 = align["n0"]
    if sign is None:
        sign = align["sign"]
    m_lo, m_hi = -GUARD, int(np.ceil(cfg.pulse_len)) + GUARD
    ks, starts = visible_events(n0, x.size, cfg, m_lo, m_hi)
    m = np.arange(m_lo, m_hi)
    if ks.size == 0:
        return np.zeros(m.size), m, n0, sign, 0.0, float("nan")
    signs = polarity_sequence(cfg) * sign
    acc = np.zeros(m.size)
    mags = []
    for k, s in zip(ks, starts):
        idx = s + m
        if idx.min() < 0 or idx.max() >= x.size:
            continue
        acc += signs[k] * x[idx]
        w = x[idx]
        mags.append(float(w.max() - w.min()))
    acc /= ks.size
    return acc, m, n0, sign, float(align["margin"]), \
        (float(np.mean(mags)) if mags else float("nan"))


def replica_baseline(rep: np.ndarray) -> float:
    return float(np.median(np.concatenate([rep[:3], rep[-3:]])))


def phase_coarse(rep: np.ndarray, m: np.ndarray, cfg: DitherConfig) -> float:
    """First-order estimate of a replica's sampling phase, in ADC samples.

    The projection of the replica onto the template's derivative.  It is *compressed* -- the
    six-sample pulse carries its edges in two samples, so the integer-sampled derivative
    understates a shift (model: +0.349 samples for a true +0.370) -- but it is a stable basin
    locator, which is what :func:`replica_phase` needs, and it is also the scale reference the
    historical ladder numbers were measured on.
    """
    dp = pulse_derivative(m, cfg.edge_r, cfg.top_w)
    den = float(dp @ dp)
    if den <= 0:
        return np.nan
    return float(rep @ dp) / den


def replica_phase(rep: np.ndarray, m: np.ndarray, cfg: DitherConfig,
                  coarse: float | None = None) -> float:
    """Fractional sampling phase of one folded replica, in ADC samples.

    Least-squares fit of ``rep(m) ~ g * pulse(m + phi)`` over the fractional phase ``phi``, with
    the template evaluated exactly where it is compared -- the whole point of the route (see the
    module docstring).

    ``coarse`` seeds the search (callers pass :func:`phase_coarse`).  The search then stays inside
    ``coarse +- 0.35`` samples; a missing or implausible seed (``|coarse| > 1``) falls back to the
    full-range grid.  The band is what keeps the route usable on real captures: an unconstrained
    search over a whole sample can and does land in a neighbouring basin when a frame is degraded,
    and measured on the archived 2026-09-19 ladder that happened often enough to put 1512 ps of
    scatter on a state whose real per-frame scatter is tens of ps.  The band is safe because the
    two estimates agree to a few hundredths of a sample whenever the replica is measurable at all
    -- the projection's *scale* is what is biased, not its basin.

    Returns the phase *shift* the replica carries: a later sampling instant gives a larger phase,
    which is the sampling-instant convention.  NaN when the replica is too flat to locate.
    """
    span = float(rep.max() - rep.min())
    if not np.isfinite(span) or span <= 0:
        return np.nan

    def fit(phi: float) -> tuple[float, float]:
        tmpl = pulse(m + phi, cfg.edge_r, cfg.top_w)
        den = float(tmpl @ tmpl)
        if den <= 0:
            return np.inf, 0.0
        g = float(rep @ tmpl) / den
        return float(((rep - g * tmpl) ** 2).sum()), g

    if coarse is None or not np.isfinite(coarse) or abs(coarse) > 1.0:
        # No usable seed: search the whole plausible range.  This is the fallback for a replica
        # whose first-order estimate is itself meaningless -- the 32-sample ladder pulse, whose
        # flat top makes the derivative projection read +-8 samples (measured on the archived
        # 2026-09-19 frames) -- and it is why the seed is checked rather than trusted.
        lo, hi = -0.75, 1.75
        best, best_res = 0.0, np.inf
        for n_grid in (60, 40, 40):
            for phi in np.linspace(lo, hi, n_grid):
                res, _ = fit(float(phi))
                if res < best_res:
                    best_res, best = res, float(phi)
            lo, hi = best - (hi - lo) / n_grid, best + (hi - lo) / n_grid
    else:
        lo, hi = coarse - 0.35, coarse + 0.35
        best, best_res = float(coarse), np.inf
        for n_grid in (40, 30, 30):
            for phi in np.linspace(lo, hi, n_grid):
                res, _ = fit(float(phi))
                if res < best_res:
                    best_res, best = res, float(phi)
            lo, hi = best - (hi - lo) / n_grid, best + (hi - lo) / n_grid
    if not np.isfinite(best_res):
        return np.nan

    # Newton polish on the same fractional template.  The step is the residual's projection onto
    # the template's own derivative, which is the direction the phase moves the model in.
    phi = best
    for _ in range(4):
        tmpl = pulse(m + phi, cfg.edge_r, cfg.top_w)
        dphi = pulse_derivative(m + phi, cfg.edge_r, cfg.top_w)
        den = float(tmpl @ tmpl)
        dden = float(dphi @ dphi)
        if den <= 0 or dden <= 0:
            break
        g = float(rep @ tmpl) / den
        if abs(g) < 1e-12:
            break
        step = float((rep - g * tmpl) @ dphi) / (g * dden)
        phi += step
        if abs(step) < 1e-5:
            break
    return float(phi)


def fold_delay_ps(rep_d: np.ndarray, rep_ref: np.ndarray, m: np.ndarray,
                  cfg: DitherConfig) -> float:
    """Cross-check timing from the folded A-B difference, in ps.

    ``dt = <A-B, d/dt> / <d/dt, d/dt>``, with the slope taken from the injected pulse shape (a
    known input, not a fit of the data) and scaled to the amplitude the replica actually has.
    This is the projection the module used before 2026-08-20 and it is *nonlinear* beyond ~0.1
    sample, because a two-sample edge does not carry its derivative on the sample grid; it is
    kept only because it is an arithmetic-independent route to the same delay.

    Sign: the projection returns the *index*-domain shift, which is the negative of the
    sampling-instant error, so it is negated here to match :func:`replica_phase`.
    """
    dp = pulse_derivative(m, cfg.edge_r, cfg.top_w)
    pp = float(pulse(m, cfg.edge_r, cfg.top_w).max() - pulse(m, cfg.edge_r, cfg.top_w).min())
    scale = (float(rep_ref.max() - rep_ref.min()) / pp) if pp else 0.0
    dp = dp * scale
    den = float(dp @ dp)
    if not den:
        return float("nan")
    return -float(rep_d @ dp) / den * 1e12 / cfg.fs_adc


def width_metrics(rep: np.ndarray, m: np.ndarray) -> tuple[float, float]:
    """Full width at half maximum (ADC samples) and steepest slope (codes/sample)."""
    base = replica_baseline(rep)
    hi = float(rep.max() - base)
    if hi <= 0:
        return float("nan"), float("nan")
    slope = float(np.max(np.abs(np.diff(rep))))
    half = base + 0.5 * hi
    idx = np.where(rep >= half)[0]
    if idx.size == 0:
        return float("nan"), slope
    i0, i1 = int(idx[0]), int(idx[-1])

    def cross(i, j):
        if rep[j] == rep[i]:
            return float(m[i])
        return float(m[i] + (half - rep[i]) / (rep[j] - rep[i]) * (m[j] - m[i]))

    left = cross(i0 - 1, i0) if i0 > 0 else float(m[i0])
    right = cross(i1, i1 + 1) if i1 < rep.size - 1 else float(m[i1])
    return abs(right - left), slope


def measure(a: np.ndarray, b: np.ndarray, cfg: DitherConfig,
            min_margin: float = 6.0) -> DitherRawEstimate:
    """Measure one capture (both channels, already de-framed) from the impulses only."""
    out = DitherRawEstimate()
    rep_a, m, n0, sign_a, margin, mag_a = fold_replica(a, cfg)
    rep_b, _, _, _, _, mag_b = fold_replica(b, cfg, n0=n0, sign=sign_a)
    out.n0, out.align_margin = n0, margin
    out.mag_a, out.mag_b = mag_a, mag_b
    out.offset_codes = float((b - a).mean())

    pp_a = float(rep_a.max() - rep_a.min())
    pp_b = float(rep_b.max() - rep_b.min())
    out.rep_pp_a, out.rep_pp_b = pp_a, pp_b
    if pp_a:
        out.gain_fold_ratio = pp_b / pp_a
    if mag_a and np.isfinite(mag_a) and np.isfinite(mag_b):
        out.gain_mag_ratio = mag_b / mag_a

    # Timing: one phase per channel, then their difference.  The common part is the ADC clock's
    # phase against the DPG loop and cancels here; only the difference is a channel mismatch.
    # Each fit is seeded by its own replica's first-order estimate when that estimate is sane,
    # which is what keeps a degraded frame from landing in a neighbouring basin (see
    # replica_phase).
    out.phase_a = replica_phase(rep_a, m, cfg, coarse=phase_coarse(rep_a, m, cfg))
    out.phase_b = replica_phase(rep_b, m, cfg, coarse=phase_coarse(rep_b, m, cfg))
    if np.isfinite(out.phase_a) and np.isfinite(out.phase_b):
        out.skew_slope_ps = (out.phase_b - out.phase_a) * 1e12 / cfg.fs_adc
    out.skew_fold_ps = fold_delay_ps(rep_a - rep_b, rep_a, m, cfg)
    out.peak_a = float(rep_a.max() - replica_baseline(rep_a))
    out.fwhm_a, out.slope_peak_a = width_metrics(rep_a, m)

    # coherent A-B power relative to the channel, the tone-free stand-in for the difference spur
    if pp_a:
        rep_d = rep_a - rep_b
        out.dbc_ab_coherent = 20.0 * np.log10(max(float(np.sqrt((rep_d ** 2).mean())), 1e-12)
                                              / max(float(np.sqrt((rep_a ** 2).mean())), 1e-12))

    # coherent dither power over the residual, after subtracting the injected sequence.
    # synthesize_dither takes the template's *amplitude* (a positive number) plus the same
    # polarity sign the fold used; folding already applied sign_a, so passing sign_a * peak_a as
    # the amplitude and 1.0 as the sign would count the sign twice.
    dither_hat = synthesize_dither(a.size, n0, cfg, out.peak_a, polarity_sign=sign_a)
    resid = a - dither_hat
    psig, pres = float((dither_hat ** 2).mean()), float((resid ** 2).mean())
    if pres > 0:
        out.snr_dither_db = 10.0 * np.log10(psig / pres) if psig > 0 else float("nan")

    out.valid = bool(margin >= min_margin and np.isfinite(out.gain_mag_ratio)
                     and np.isfinite(out.skew_slope_ps))
    if not out.valid:
        if margin < min_margin:
            out.reasons.append("align_margin")
        if not np.isfinite(out.skew_slope_ps):
            out.reasons.append("no_slope")
        if not np.isfinite(out.gain_mag_ratio):
            out.reasons.append("no_gain")
    return out
