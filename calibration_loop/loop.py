"""
The calibration loop itself.

One iteration:

    capture -> de-frame -> apply current correction -> estimate residual errors
            -> block-LMS update -> push skew to the AD9695 clock delay -> log

Estimating *after* the correction is applied is what makes the recorded
trajectory a real closed-loop learning curve, directly comparable with Fig. 23 of
Wang et al., TCAS-I 2025.  Every iteration consumes one DMA buffer, so the x axis
of that curve is ``iteration * samples_per_channel`` ADC cycles.
"""

from __future__ import annotations

import csv
import hashlib
import json
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np

from .dither import DitherConfig
from .estimator import (
    BlockEstimate,
    CalibrationState,
    estimate_block,
    gain_observable_pair,
    interleave,
    polarity_anchor,
    prepare_capture,
    skew_batch,
    skew_batch_dither,
    synthesize_dither,
)
from .metrics import analyse, channel_difference_dbc, mismatch_spurs


@dataclass
class LoopOptions:
    cancel_signal: bool = True
    """Remove the fitted main tone before averaging the dither windows.  This is
    the interference-cancellation switch; turn it off to measure the slow
    baseline for the convergence comparison."""

    close_skew_loop: bool = True
    """Push the skew estimate back into the AD9695 sample-clock delay.  Set False
    to leave the hardware alone and characterise skew open-loop."""

    interleaved: bool = False
    """False: the two channels sample at the same instant (what this bench can do
    today), so the figures of merit are per channel plus the A-minus-B residual.
    True: a half-period offset exists in the clock path, so the interleaved
    stream and its mismatch spurs become meaningful."""

    max_capture_retries: int = 3

    max_consecutive_rejects: int = 20
    """Stop after this many captures in a row are rejected.

    A rejected capture is logged for forensics and then retried, because it must not
    consume a sample slot: ``run(N)`` owes the caller N qualified samples.  Rejections
    are routine (~20 % measured on the bench, almost all ``align_margin`` on torn
    UDP frames), but a *streak* this long is not a bad frame, it is a bench that
    stopped producing usable data — a half-synced JESD link, or the wrong waveform
    loaded — and the honest response is to stop rather than spend the rest of the
    session on it.  At the measured 20 % rate, 20 in a row is about 1e-14."""

    min_align_margin: float = 6.0
    """Reject a capture whose dither correlation peak is this many sigma or less
    above the rest of the lag profile.  On hardware a dropped UDP datagram or a
    torn DMA frame shows up exactly this way, and one bad frame driven into the
    LMS undoes many good ones."""

    max_skew_samples: float = 0.25
    """Reject a skew estimate larger than this fraction of a sample period.  The
    first-order expansion behind the skew estimate is only valid for small
    errors, so a large value means the fit failed, not that the skew is large."""

    max_skew_samples_dither: float = 0.5
    """The same gate for the *tone-free* routes (:attr:`skew_observable` = ``dither_phase``
    or ``dither_fold``).

    A separate bound, because the two routes fail differently.  The tone-phase estimate wraps
    every half tone period (1254 ps), so its gate has to be tight enough to catch a wrapped
    branch; the fractional-phase route does not wrap, it only compresses, and it is linear to
    better than 2 % out to +-0.25 samples (measured in the model 2026-09-20).  What the bound
    has to catch here is a frame whose folded difference is dominated by noise, so it can sit
    where a genuine acquisition-time mismatch still passes: the loop starts within a few hundred
    ps of its target, and rejecting the frames that carry the error signal would leave the
    actuator unable to acquire at all."""

    max_gain_deviation: float = 0.20
    """Reject a block whose measured gain ratio is further than this from 1."""

    gain_observable: str = "tone"
    """Which measurement the gain correction integrates: ``"tone"``, ``"dither"`` or
    ``"dither_mag"``.

    Three estimates of the same channel gain mismatch.  On this bench the tone route and the
    pulse-window route disagree by ~1.2 %: the dither route reads the narrow pulse replicas
    (dispersed differently by the two channels), the tone route reads the 199 MHz tone that the
    A-B difference spur is actually made of.  The tone route is also ~10x quieter (0.25 %
    per-frame scatter against 2.3 %), which is why it became the default.

    Without a tone the choice narrows to the two dither routes, and they are not equivalent:
    measured 2026-09-19, ``"dither"`` (the estimator's ``gain_ratio``) reads ~3 % high and does
    not track the replica amplitude at all (correlation -0.055 across frames, against +0.90 for
    the genuine common-mode movement of the two channels), while ``"dither_mag"`` -- the sign-free
    per-event magnitude ratio from ``dither_raw`` -- agrees with the polarity-corrected folded
    replica ratio to 0.02 %.  So a tone-free run wants ``"dither_mag"``.

    The log records the ratio of every route, so the seating stays auditable."""

    skew_observable: str = "phase"
    """Which measurement the skew decision integrates: ``"phase"``, ``"dither_phase"`` or
    ``"dither_fold"``.

    ``"phase"`` is the tone-based route and needs a tone: it fits the phase of the main tone in
    each channel.  The two dither routes are tone-free and take their timing from the folded
    impulse replicas (:mod:`calibration_loop.dither_raw`):

    * ``"dither_phase"`` fits each channel's replica against the known template **at a fractional
      sampling phase** and differences the two phases.  It is the linear route: in the bench
      *model* it is unbiased to +-2 ps over +-400 ps with 3-7 ps of per-frame scatter, against a
      projection that reads 86.6 ps where the truth is 103.6 -- and on the bench it is the one
      whose scale matches the tone route.
    * ``"dither_fold"`` projects the folded A-B difference onto the pulse's derivative.  It needs
      no template fit and is therefore quieter on a reshaped bench pulse, but it compresses: its
      *scale* ran 2.8x low on this bench, which widens the effective deadband to ~28 ps (see
      below).  Its zero crossing stays honest, so it is still usable as a cross-check.

    Which to use is a hardware question, and it is answerable before closing the loop:
    ``tools/timing_route_check.py`` measures both against known actuator codes (read-only, offline
    on archived or fresh captures), and -- when the captures hold a tone -- against the
    tone-phase route as well, which is shape-independent and therefore the scale anchor.

    **Measured 2026-09-20 on the bench, and the answer is not the one the noise figures suggest.**
    On ``w06`` 16000 LSB with 200 frames per state (alignment margin 8.1 against a 6.0 floor):

    | route | ps/code | per-frame scatter |
    |---|---|---|
    | ``dither_phase`` | **+18.2** | 4.8 ps |
    | ``dither_fold`` | +6.5 | **3.1 ps** |
    | tone route (next hour, one code, tone+dither waveform) | **+19.5** | (se 0.07 ps on the batch mean) |

    The fold projection is *quieter* and still has the **wrong scale** -- 2.8x low -- and the two
    consequences are the reason ``dither_phase`` is what ``--tone-free`` selects.  A dither-only
    run driving ``dither_fold`` parked at code 34 reading **-7.5 ps**, i.e. inside its 10 ps
    deadband, while the tone route measured **-16.9 ps** at that same code and an independent
    functional on the same frames (the raw A-B spur at f_in, after removing the known 1.5 % gain
    term) implied **17.7 ps**.  So a scale-biased route does not just converge slowly: it stops
    with a real residual nearly 3x its own deadband.  The *zero points* agreed throughout, to
    0.3 code (fold 35.2, phase 35.2, tone 34.9), so this is a scale error, not an offset.

    A 20-frame batch on ``dither_phase`` then has 1.1 ps of standard error against the 10 ps
    deadband -- still more than precise enough.  The earlier 4.85 and 7.4 ps/code
    characterizations are *not* an independent anchor: both were measured with the tone route
    (``tools/skew_step_characterize.py`` takes ``est.skew_phase_ps``), and today's tone reading of
    19.5 ps/code at the working point disagrees with them, so the step at the code you actually
    use should be re-measured before any figure from it is quoted.

    A tone-free run must also set ``cancel_signal = False``: with no tone to remove, the
    least-squares tone fit subtracts structure from the record instead of a signal.  Both are set
    for you by ``--tone-free``.
    """

    skew_batch_frames: int = 20
    """Accepted frames per skew decision.

    Skew actuation is a *batch* decision, never per frame.  The measured step is
    ~7.4 ps of differential skew per control code, which is larger than the
    residual error once the loop is anywhere near its target, so a per-frame
    integrator would move a code every couple of frames and chase its own
    measurement noise.  With 20 frames the batch mean has a standard error of
    ~0.7 ps, so the deadband below is ~14 sigma wide and a move means something."""

    skew_deadband_ps: float = 10.0
    """Do not move the actuator for an error inside this band.

    Measured residual wobble at the converged point is a few ps over minutes, and
    the per-frame scatter is 3.1 ps RMS, so this band separates signal from noise
    without leaving a visible offset (it is ~1.3 % of a sample period)."""

    skew_direction_tol_ps: float = 8.0
    """Give up on skew actuation if the error moves the wrong way after a move.

    A corrective move of ``steps`` codes must shift the measured error by
    ``+HOST_STEP_PS * steps``: that sign is a bench measurement, not a
    convention.  If the error instead moves the other way, the hardware no longer
    agrees with the calibration -- a stale code belief, a swapped channel, a
    reloaded DPG -- and one more code per batch would walk the actuator away from
    the target instead of towards it.

    The check requires **both** a large disagreement *and* the opposite sign, so a step-size error
    alone cannot latch it.  That matters on this bench, because the actuator's step is not
    uniform: measured 2026-09-20 with `tools/skew_step_characterize.py`, one code is 19.08 +- 0.06
    ps between codes 34 and 35 (where the loop parks) against ~7.4 ps near neutral (2026-09-17).
    Whichever value ``HOST_STEP_PS`` carries, an honest move in the other region disagrees with
    the expectation by ~12 ps > this tolerance but keeps the sign, so it is not a fault.  The
    tolerance itself absorbs the batch noise on a single move: a 20-frame batch mean has ~1-2 ps
    of standard error on the bench, so 8 ps is several sigma and only a real wrong-direction move
    trips it."""

    skew_min_yield: float = 0.4
    """Fraction of a batch that must survive the acceptance filter to be acted on.

    The batch decision is only as good as the number of frames behind it: a mean
    taken from 2 of 20 frames has a standard error of several ps, against a 10 ps
    deadband, so it must not be allowed to move the actuator.  Below this yield the
    batch is logged and nothing else happens -- the same floor
    ``tools/skew_close_loop.py`` uses.  Measured batches on the bench yield
    0.75-1.0, so this only fires on a genuinely broken capture run (a half-synced
    JESD link, dropped datagrams), which is exactly when one bad mean would
    otherwise walk the actuator."""


class CalibrationLoop:
    def __init__(
        self,
        bench,
        cfg: DitherConfig,
        state: CalibrationState | None = None,
        options: LoopOptions | None = None,
    ):
        self.bench = bench
        self.cfg = cfg
        self.state = state or CalibrationState()
        self.opt = options or LoopOptions()
        self.log: list[dict] = []
        self.captures = 0
        """Captures attempted by the last :meth:`run` (qualified + rejected)."""
        self.qualified = 0
        """Samples that passed the acceptance filter in the last :meth:`run`."""
        self.rejected = 0
        """Captures the acceptance filter threw away in the last :meth:`run`."""
        self._signature: dict | None = None
        self._polarity: float | None = None
        self._skew_frames: list[dict] = []
        self._raw_measure = None
        """Tone-free measurement of the current frame's residual, when a tone-free route runs."""
        self._raw_excitation = None
        """Tone-free measurement of the raw capture: excitation peak, coherent A-B power, SNR."""
        """Accepted frames of the skew batch in progress (see LoopOptions)."""
        self._skew_prev_error: float | None = None
        """Batch error at the previous decision, for the direction check."""
        self._skew_expected_shift = 0.0
        """Error shift the last commanded move should have produced, in ps."""
        self._skew_abort: str | None = None
        """Latched reason skew actuation stopped; None while it is still trusted."""

    # -- skew actuation ------------------------------------------------------
    def _skew_decision(self, est: BlockEstimate, row: dict) -> None:
        """Collect one frame, and act only when a whole batch has accumulated.

        The order matters and is the one validated on the bench: measure a batch,
        take its arithmetic mean through the acceptance filter, compare against
        the deadband, and only then request at most one control code through the
        firmware transaction -- whose ACK is the only thing that updates the local
        actuator state.  Nothing is retried: a failed transaction ends the move,
        it does not trigger another write.

        A move also has to be confirmed, not assumed.  The next batch's error must
        have shifted the way the calibrated step size predicts
        (``skew_direction_tol_ps``); a disagreement, or a transaction whose ACK
        never arrived, latches skew actuation off for the rest of the run.  The
        digital loop keeps running when that happens, because it is host-side and
        has no hardware state to corrupt.
        """
        self._skew_frames.append({
            "phase_ps": (est.skew_slope_ps if self.opt.skew_observable.startswith("dither")
                         else est.skew_phase_ps),
            "diff_route_ps": est.skew_diff_route_ps,
            "centroid_ps": est.skew_centroid_ps,
            # The number this batch averages, whichever route the options selected, plus both
            # tone-free routes for context (`skew_fold_ps` is the projection, `skew_slope_ps` the
            # fractional-phase fit -- see LoopOptions.skew_observable).
            "used_ps": est.skew_used_ps,
            "slope_ps": est.skew_slope_ps,
            "fold_ps": est.skew_fold_ps,
            "source": est.skew_source,
            "tone": est.ch_a.tone_amplitude,
            "resid": est.ch_a.residual_rms,
            "gain_mag_ratio": est.gain_mag_ratio,
            "margin": row.get("align_margin", float("nan")),
        })
        row["skew_batch_n"] = len(self._skew_frames)
        if len(self._skew_frames) < self.opt.skew_batch_frames:
            return

        if self.opt.skew_observable.startswith("dither"):
            batch = skew_batch_dither(self._skew_frames,
                                      margin_min=self.opt.min_align_margin)
        else:
            batch = skew_batch(self._skew_frames)
        self._skew_frames = []
        row["skew_batch_used"] = batch.n_used
        row["skew_batch_mean_ps"] = batch.mean_ps
        row["skew_batch_se_ps"] = batch.se_ps
        error = batch.mean_ps - self.state.skew_target_ps
        row["skew_batch_error_ps"] = error

        actuator = getattr(self.bench, "actuator", None)
        if not self.opt.close_skew_loop or actuator is None:
            row["skew_action"] = "measure-only"
            return
        if self._skew_abort is not None:
            # Latched: the actuator already answered badly, and writing it again
            # is not a recovery strategy (see capture.SkewActuator).
            row["skew_action"] = "aborted"
            row["skew_error"] = self._skew_abort
            return
        if batch.n_used == 0:
            row["skew_action"] = "no-usable-frames"
            return
        if batch.n_total and batch.n_used / batch.n_total < self.opt.skew_min_yield:
            # Too few frames to justify a write; see LoopOptions.skew_min_yield.
            row["skew_action"] = f"low-yield:{batch.n_used}/{batch.n_total}"
            return

        # Direction check, before this batch may write anything: the previous move
        # should have shifted the error by +HOST_STEP_PS * steps.
        if self._skew_prev_error is not None and self._skew_expected_shift != 0.0:
            moved = error - self._skew_prev_error
            if (abs(moved - self._skew_expected_shift) > self.opt.skew_direction_tol_ps
                    and np.sign(moved) != np.sign(self._skew_expected_shift)):
                self._skew_abort = (
                    f"direction: error moved {moved:+.2f} ps after a "
                    f"{self._skew_expected_shift:+.1f} ps expected move")
                row["skew_action"] = "abort:direction"
                row["skew_error"] = self._skew_abort
                return

        steps = actuator.error_to_steps(error)
        if steps == 0:
            row["skew_action"] = "inside-deadband" if abs(error) <= actuator.deadband_ps \
                else "at-range-limit"
            return
        ack = actuator.request_steps(steps)
        row["skew_action"] = f"move{steps:+d}:{ack.get('result')}"
        row["skew_step_codes"] = steps
        row["skew_code"] = ack.get("code_after")
        if not ack.get("ok"):
            # No retry, no second write: report it and stop driving the actuator.
            self._skew_abort = f"{ack.get('result')} stage={ack.get('stage')}"
            row["skew_error"] = self._skew_abort
            return
        self._skew_prev_error = error
        self._skew_expected_shift = steps * getattr(actuator, "HOST_STEP_PS", 7.4)

    # -- one iteration ------------------------------------------------------
    def step(self) -> dict | None:
        raw = self.bench.capture()
        if raw is None:
            return None

        prep = prepare_capture(raw, self.cfg, signature=self._signature)
        if self._signature is None:
            self._signature = prep["signature"]
        if self._polarity is None:
            # The dither polarity of the bench path is a session constant, not a
            # per-frame quantity: measure it once from channel A and hold it, so
            # a noisy frame cannot flip the gain signs mid-run.
            self._polarity = polarity_anchor(prep)
        ch_a, ch_b = prep["ch_a"], prep["ch_b"]

        cal_a, cal_b = self.state.apply(ch_a, ch_b)
        est = estimate_block(
            cal_a, cal_b, self.cfg,
            cancel_signal=self.opt.cancel_signal,
            n0=prep["n0"],
            skew_prior_samples=self.state.skew_target_ps * 1e-12 * self.cfg.fs_adc,
            polarity_sign=self._polarity,
            pin_polarity=True,
        )
        est.rotation = prep["rotation"]

        # Tone-free routes, measured only when the run asks for them.  The observables the loop
        # integrates are taken on the *corrected* streams (the loop integrates residual errors,
        # not absolutes); the excitation and the raw mismatch are measured on the raw capture,
        # because those describe the bench rather than the residual.  The coherent A-B power is
        # logged both ways: raw is the native mismatch, corrected is what the loop left, and
        # their difference is the tone-free answer to "what did this run buy".
        if self.opt.skew_observable.startswith("dither") or self.opt.gain_observable == "dither_mag":
            from .dither_raw import measure as measure_raw

            res = measure_raw(cal_a, cal_b, self.cfg, min_margin=self.opt.min_align_margin)
            exc = measure_raw(ch_a, ch_b, self.cfg, min_margin=self.opt.min_align_margin)
            est.gain_mag_ratio = res.gain_mag_ratio
            est.mag_a, est.mag_b = res.mag_a, res.mag_b
            est.skew_slope_ps = res.skew_slope_ps
            est.skew_fold_ps = res.skew_fold_ps
            est.phase_a, est.phase_b = res.phase_a, res.phase_b
            est.peak_a = exc.peak_a
            est.dbc_ab_coherent = exc.dbc_ab_coherent
            est.dbc_ab_coherent_cal = res.dbc_ab_coherent
            est.snr_dither_db = exc.snr_dither_db
            est.snr_dither_db_cal = res.snr_dither_db
            self._raw_measure = res
            self._raw_excitation = exc
        else:
            self._raw_measure = None
            self._raw_excitation = None

        # The number this iteration integrates, on whichever route the options selected.  Set
        # *before* the row is written and before the acceptance filter runs, so the CSV column,
        # the learning curve and the state update can never disagree about which observable was
        # in use -- the failure mode that made a dead tone-based route look like a converging
        # loop (see LoopOptions.skew_observable).
        est.skew_used_ps = {
            "phase": est.skew_mismatch_ps,
            "dither_phase": est.skew_slope_ps,
            "dither_fold": est.skew_fold_ps,
        }.get(self.opt.skew_observable, est.skew_mismatch_ps)

        row = self._measure(ch_a, ch_b, cal_a, cal_b, est, prep["n0"])
        row["swapped"] = prep["swapped"]
        row["align_margin"] = prep["align_margin"]
        # Frame identity: with the firmware's capture generation this is what
        # makes a re-sent or stale buffer detectable from the log alone.
        row["frame_sha1"] = hashlib.sha1(raw).hexdigest()[:16]

        reject = self._reject_reason(prep, est)
        row["rejected"] = reject or ""
        if reject:
            self.log.append(row)
            return row

        if getattr(self.bench, "actuator", None) is not None:
            # Hardware: skew is a batch decision, gain/offset stay per-frame
            # (digital corrections applied on the host, no hardware side effect).
            errors = self.state.update(est, integrate_skew=False,
                                       gain_observable=self.opt.gain_observable)
            row.update(errors)
            self._skew_decision(est, row)
        else:
            # Bench model: no registers to disturb and no measurement noise worth
            # batching, so the original per-frame integrator stays.  The observable choice
            # still applies, which is what lets a tone-free run be exercised offline.
            errors = self.state.update(est, integrate_skew=self.opt.close_skew_loop,
                                       gain_observable=self.opt.gain_observable,
                                       skew_ps=est.skew_used_ps)
            row.update(errors)
            if self.opt.close_skew_loop and hasattr(self.bench, "command_skew"):
                self.bench.command_skew(self.state.skew_cmd_ps)

        row.update(
            {
                "offset_a_state": self.state.offset_a,
                "offset_b_state": self.state.offset_b,
                "gain_corr_a": self.state.gain_corr_a,
                "gain_corr_b": self.state.gain_corr_b,
                "skew_cmd_ps": self.state.skew_cmd_ps,
            }
        )

        self.log.append(row)
        return row

    def _reject_reason(self, prep: dict, est: BlockEstimate) -> str | None:
        """Guard the LMS against frames the estimator could not trust.

        Every gate judges the observable this run actually integrates, not the one the estimator
        happens to compute.  On a tone-free capture the estimator's tone-based fields are noise
        -- the fit latches onto a dither comb line and the phase route then reports hundreds of
        ps -- so gating on them would reject the majority of good frames while the loop's own
        route was fine (measured 2026-09-20: 81 % rejected that way), and gating the *gain* on
        the estimator's pulse-window ratio would ignore a silent fallback away from the
        requested one.
        """
        if prep["align_margin"] < self.opt.min_align_margin:
            return f"align_margin={prep['align_margin']:.1f}"
        if est.ch_a.n_events_used < 2 or est.ch_b.n_events_used < 2:
            return "too few dither events in the capture"

        tone_free = self.opt.skew_observable.startswith("dither")
        if tone_free:
            # The tone-based per-channel fields are not part of this run's answer; what has to be
            # finite is the route that is.  ``dither_raw`` returns NaN for it exactly when the
            # replica was too flat or too corrupt to locate.
            if not np.isfinite(est.skew_used_ps):
                return "no tone-free skew estimate"
            if not (np.isfinite(est.mag_a) and np.isfinite(est.mag_b)):
                return "no tone-free gain estimate"
        else:
            for tag, ch in (("A", est.ch_a), ("B", est.ch_b)):
                if not np.isfinite(ch.gain_codes) or not np.isfinite(ch.skew_samples):
                    return f"channel {tag} estimate not finite"

        # Only the mismatch is a defect; the sub-sample phase both channels share
        # against the DPG loop is a property of the clock path, not an error.
        residual = (est.skew_used_ps - self.state.skew_target_ps) * 1e-12 * self.cfg.fs_adc
        bound = self.opt.max_skew_samples_dither if tone_free else self.opt.max_skew_samples
        if abs(residual) > bound:
            return f"skew mismatch residual={residual:.3f} samples out of range"

        # The gain gate follows the chosen observable, and a *fallback* is itself a rejection in
        # a tone-free run: falling back there means falling back onto the tone.
        ga, gb, source = gain_observable_pair(est, self.opt.gain_observable)
        if tone_free and source != "dither_mag":
            return f"gain observable fell back to {source}"
        if np.isfinite(ga) and np.isfinite(gb) and abs(ga) > 1e-9:
            ratio = gb / ga
            if abs(ratio - 1.0) > self.opt.max_gain_deviation:
                return f"gain ratio={ratio:.3f} out of range ({source})"
        return None

    def _measure(self, raw_a, raw_b, cal_a, cal_b, est: BlockEstimate, n0: int) -> dict:
        fs = self.cfg.fs_adc
        f_in = self.cfg.f_sig

        # Strip the injected dither before scoring; see synthesize_dither().  The
        # gains were fitted with the session polarity sign, so the synthesised
        # dither must use it too or the subtraction would become an addition.
        pol = self._polarity or 1.0
        d_a = synthesize_dither(cal_a.size, n0, self.cfg, est.ch_a.gain_codes,
                                polarity_sign=pol)
        d_b = synthesize_dither(cal_b.size, n0, self.cfg, est.ch_b.gain_codes,
                                polarity_sign=pol)
        cal_a, cal_b = cal_a - d_a, cal_b - d_b
        # The raw records have not been through the gain correction, so the
        # dither sits there at a proportionally different amplitude.
        ga = self.state.gain_corr_a or 1.0
        gb = self.state.gain_corr_b or 1.0
        raw_a, raw_b = raw_a - d_a / ga, raw_b - d_b / gb

        row = {
            "iteration": self.state.iteration,
            "cycles": self.state.iteration * cal_a.size,
            "rotation": est.rotation,
            "events_used": est.ch_a.n_events_used,
            "align_n0": est.ch_a.align_n0,
            "offset_a_codes": est.ch_a.offset_codes,
            "offset_b_codes": est.ch_b.offset_codes,
            "gain_a_codes": est.ch_a.gain_codes,
            "gain_b_codes": est.ch_b.gain_codes,
            "gain_ratio": est.gain_ratio,
            "tone_ratio": est.tone_ratio,
            "skew_a_ps": est.ch_a.skew_ps,
            "skew_b_ps": est.ch_b.skew_ps,
            "skew_mismatch_ps": est.skew_mismatch_ps,
            # the two independent routes to that number, plus the centroid that
            # must never veto it (see SkewBatch)
            "skew_phase_ps": est.skew_phase_ps,
            "skew_diff_route_ps": est.skew_diff_route_ps,
            "skew_centroid_ps": est.skew_centroid_ps,
            "skew_source": est.skew_source,
            # The number this run integrates, on whichever route the options selected, and the
            # name of that route.  `skew_mismatch_ps` above stays the estimator's own answer, so
            # a reader can always see both and tell which one moved the actuator.
            "skew_used_ps": est.skew_used_ps,
            "skew_observable": self.opt.skew_observable,
            # tone-free routes (NaN unless a dither-based observable is in use)
            "gain_mag_ratio": est.gain_mag_ratio,
            "skew_slope_ps": est.skew_slope_ps,
            "skew_fold_ps": est.skew_fold_ps,
            "phase_a_samples": est.phase_a,
            "phase_b_samples": est.phase_b,
            "dither_peak_a_codes": est.peak_a,
            "dbc_ab_coherent": est.dbc_ab_coherent,
            "dbc_ab_coherent_cal": est.dbc_ab_coherent_cal,
            "snr_dither_db": est.snr_dither_db,
            "snr_dither_db_cal": est.snr_dither_db_cal,
        }

        # Per-channel dynamic performance, always meaningful.
        for tag, x in (("raw_a", raw_a), ("raw_b", raw_b), ("cal_a", cal_a), ("cal_b", cal_b)):
            m = analyse(x, fs)
            row[f"{tag}_sndr_db"] = m["sndr_db"]
            row[f"{tag}_sfdr_db"] = m["sfdr_db"]
            row[f"{tag}_enob"] = m["enob_bits"]

        raw_d = channel_difference_dbc(raw_a, raw_b, fs, f_in)
        cal_d = channel_difference_dbc(cal_a, cal_b, fs, f_in)
        row["raw_difference_dbc"] = raw_d["difference_dbc"]
        row["cal_difference_dbc"] = cal_d["difference_dbc"]
        row["cal_dc_difference_codes"] = cal_d["dc_difference_codes"]

        if self.opt.interleaved:
            fs_out = 2.0 * fs
            raw_stream = interleave(raw_a, raw_b)
            cal_stream = interleave(cal_a, cal_b)
            raw_m = analyse(raw_stream, fs_out)
            cal_m = analyse(cal_stream, fs_out)
            raw_s = mismatch_spurs(raw_stream, fs_out, f_in)
            cal_s = mismatch_spurs(cal_stream, fs_out, f_in)
            row.update({
                "raw_sndr_db": raw_m["sndr_db"],
                "raw_sfdr_db": raw_m["sfdr_db"],
                "raw_enob": raw_m["enob_bits"],
                "cal_sndr_db": cal_m["sndr_db"],
                "cal_sfdr_db": cal_m["sfdr_db"],
                "cal_enob": cal_m["enob_bits"],
                "raw_offset_spur_dbc": raw_s["offset_spur_dbc"],
                "raw_image_spur_dbc": raw_s["gain_skew_image_dbc"],
                "cal_offset_spur_dbc": cal_s["offset_spur_dbc"],
                "cal_image_spur_dbc": cal_s["gain_skew_image_dbc"],
            })
        else:
            # Interleaving two channels that sample at the same instant would
            # produce a hold-and-repeat sequence, not a 2x converter, so those
            # numbers would be meaningless here.  Mirror the single-channel
            # result instead so the log keeps one schema.
            row.update({
                "raw_sndr_db": row["raw_a_sndr_db"],
                "raw_sfdr_db": row["raw_a_sfdr_db"],
                "raw_enob": row["raw_a_enob"],
                "cal_sndr_db": row["cal_a_sndr_db"],
                "cal_sfdr_db": row["cal_a_sfdr_db"],
                "cal_enob": row["cal_a_enob"],
                "raw_offset_spur_dbc": float("nan"),
                "raw_image_spur_dbc": row["raw_difference_dbc"],
                "cal_offset_spur_dbc": float("nan"),
                "cal_image_spur_dbc": row["cal_difference_dbc"],
            })

        return row

    # -- driver -------------------------------------------------------------
    def run(self, iterations: int, verbose: bool = True) -> list[dict]:
        """Collect ``iterations`` *qualified* samples.

        ``iterations`` counts samples that pass the acceptance filter, not captures.
        A rejected frame is logged — its reason is the diagnostic — and then retried,
        so a run of N ends with N points on the learning curve and the log's accepted
        rows are exactly those N.  Rejected rows stay in the CSV (marked) for
        forensics but are excluded from :meth:`plot` and from every sample count.  A
        300-sample run therefore takes roughly 370 captures at the measured ~20 %
        rejection rate.

        Two things still stop a run early, and both mean the bench rather than the
        frame is the problem: ``max_capture_retries`` consecutive captures that
        returned nothing, and ``max_consecutive_rejects`` captures in a row that the
        filter refused.
        """
        qualified = 0
        captures = 0
        rejected = 0
        failures = 0
        streak = 0
        while qualified < iterations:
            row = self.step()
            captures += 1
            if row is None:
                failures += 1
                if verbose:
                    print(f"  capture failed ({failures})")
                if failures >= self.opt.max_capture_retries:
                    print("  too many capture failures, stopping")
                    break
                continue
            failures = 0
            if row.get("rejected"):
                rejected += 1
                streak += 1
                if verbose:
                    print(f"  rejected #{rejected} ({streak} in a row): "
                          f"{row['rejected']}")
                if streak >= self.opt.max_consecutive_rejects:
                    print(f"  {streak} captures rejected in a row -- the bench is not "
                          f"producing usable data, stopping")
                    break
                continue
            streak = 0
            qualified += 1
            if verbose:
                print(
                    f"  it {row['iteration']:4d}  "
                    f"g_B/g_A={row['gain_ratio']:+.5f}  "
                    f"tone={row['tone_ratio']:+.5f}  "
                    f"dOffset={row['offset_b_codes'] - row['offset_a_codes']:+8.3f} LSB  "
                    f"dSkew={row['skew_used_ps']:+7.3f} ps  "
                    f"SNDR={row['cal_sndr_db']:5.2f} dB  "
                    f"image={row['cal_image_spur_dbc']:6.1f} dBc"
                )
        self.qualified = qualified
        self.captures = captures
        self.rejected = rejected
        if verbose:
            rate = 100.0 * rejected / captures if captures else 0.0
            print(f"  {qualified}/{iterations} qualified samples from {captures} "
                  f"captures ({rejected} rejected, {rate:.0f} %)")
        return self.log

    # -- output -------------------------------------------------------------
    def save(self, out_dir: str | Path, stem: str = "calibration_run") -> dict:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        csv_path = out_dir / f"{stem}.csv"
        if self.log:
            # Accepted and rejected rows do not carry the same keys: a rejection
            # returns before the state update, so it has no offset_error_*/gain_*
            # /skew_cmd_ps/state columns.  Taking the header from log[0] alone
            # therefore crashes the whole save whenever the first iteration is
            # rejected -- losing the entire run after the bench time was spent.
            # Use the union of every row's keys, in first-seen order.
            fieldnames: list[str] = []
            for entry in self.log:
                for key in entry:
                    if key not in fieldnames:
                        fieldnames.append(key)
            with csv_path.open("w", newline="", encoding="utf-8") as fh:
                writer = csv.DictWriter(fh, fieldnames=fieldnames, restval="")
                writer.writeheader()
                writer.writerows(self.log)

        json_path = out_dir / f"{stem}_meta.json"
        json_path.write_text(
            json.dumps(
                {
                    "dither": asdict(self.cfg),
                    "options": asdict(self.opt),
                    "final_state": asdict(self.state),
                    "channel_signature": self._signature,
                    "polarity_sign": self._polarity,
                    "skew_abort": self._skew_abort,
                    # Sample accounting: the CSV holds every capture, qualified and
                    # rejected, so the run's real sample count has to be recorded
                    # rather than inferred from the row count.
                    "samples_qualified": self.qualified,
                    "captures_attempted": self.captures,
                    "captures_rejected": self.rejected,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        return {"csv": csv_path, "meta": json_path}

    def plot(self, out_dir: str | Path, stem: str = "calibration_run"):
        """Learning curves: the figures a TCAS submission needs.

        Only qualified samples are drawn.  A rejected capture stays in the CSV for
        forensics, but its metrics come from a frame the loop explicitly refused to
        trust and drive the state with — putting those on the learning curve would
        show the acceptance filter's rejects as loop behaviour, which is precisely
        what a reader of this figure is meant to be able to believe.
        """
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        rows = [r for r in self.log if not r.get("rejected")]
        if not rows:
            return None

        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        it = np.array([r["iteration"] for r in rows])
        fig, ax = plt.subplots(2, 2, figsize=(12, 8))

        # Gain: the controlled observable leads.  With the tone route the dither
        # ratio is no longer what is being nulled -- it settles at the dither-vs-tone
        # disagreement (~1.2 % on this bench) -- so every route that was measured is
        # drawn instead of letting one masquerade as the loop's error signal.  Which
        # one was controlled comes from ``gain_source``, so a fallback cannot hide.
        rows_last = rows[-1]
        controlled = rows_last.get("gain_source", "tone")
        routes = (
            ("tone", "tone_ratio", "tone route"),
            ("dither_mag", "gain_mag_ratio", "dither magnitude (sign-free)"),
            ("dither", "gain_ratio", "dither route (pulse windows)"),
        )
        for name, key, label in routes:
            vals = np.array([r.get(key, np.nan) for r in rows], dtype=float)
            if not np.isfinite(vals).any():
                continue
            is_controlled = name == controlled
            ax[0, 0].plot(it, vals, lw=2.0 if is_controlled else 1.0,
                          ls="-" if is_controlled else ":",
                          label=f"{label}{' (controlled)' if is_controlled else ''}")
        ax[0, 0].axhline(1.0, ls="--", lw=1, color="k")
        ax[0, 0].legend(fontsize=8)
        ax[0, 0].set_title("Gain ratio $g_B/g_A$ (residual)")

        ax[0, 1].plot(
            it, [r["offset_b_codes"] - r["offset_a_codes"] for r in rows]
        )
        ax[0, 1].axhline(0.0, ls="--", lw=1, color="k")
        ax[0, 1].set_title("Offset mismatch [LSB] (residual)")

        ax[1, 0].plot(it, [r.get("skew_used_ps", r.get("skew_mismatch_ps")) for r in rows])
        ax[1, 0].axhline(0.0, ls="--", lw=1, color="k")
        ax[1, 0].set_title("Timing skew mismatch [ps] (residual, "
                           f"{rows[-1].get('skew_observable', 'phase')} route)")

        ax[1, 1].plot(it, [r["cal_sndr_db"] for r in rows], label="SNDR calibrated")
        ax[1, 1].plot(it, [r["cal_sfdr_db"] for r in rows], label="SFDR calibrated")
        ax[1, 1].plot(
            it, [r["raw_sndr_db"] for r in rows], ls=":", label="SNDR raw"
        )
        ax[1, 1].legend()
        ax[1, 1].set_title("Dynamic performance [dB]")

        for a in ax.ravel():
            a.set_xlabel("qualified sample")
            a.grid(True, alpha=0.3)

        excluded = len(self.log) - len(rows)
        if excluded:
            fig.suptitle(f"{len(rows)} qualified samples "
                         f"({excluded} rejected captures excluded)", fontsize=10)
        fig.tight_layout()
        path = out_dir / f"{stem}_learning.png"
        fig.savefig(path, dpi=140)
        plt.close(fig)
        return path
