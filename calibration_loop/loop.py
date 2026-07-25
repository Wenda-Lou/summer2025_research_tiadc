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
import json
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np

from .dither import DitherConfig
from .estimator import (
    BlockEstimate,
    CalibrationState,
    estimate_block,
    interleave,
    prepare_capture,
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

    min_align_margin: float = 6.0
    """Reject a capture whose dither correlation peak is this many sigma or less
    above the rest of the lag profile.  On hardware a dropped UDP datagram or a
    torn DMA frame shows up exactly this way, and one bad frame driven into the
    LMS undoes many good ones."""

    max_skew_samples: float = 0.25
    """Reject a skew estimate larger than this fraction of a sample period.  The
    first-order expansion behind the skew estimate is only valid for small
    errors, so a large value means the fit failed, not that the skew is large."""

    max_gain_deviation: float = 0.20
    """Reject a block whose measured gain ratio is further than this from 1."""


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
        self._signature: dict | None = None

    # -- one iteration ------------------------------------------------------
    def step(self) -> dict | None:
        raw = self.bench.capture()
        if raw is None:
            return None

        prep = prepare_capture(raw, self.cfg, signature=self._signature)
        if self._signature is None:
            self._signature = prep["signature"]
        ch_a, ch_b = prep["ch_a"], prep["ch_b"]

        cal_a, cal_b = self.state.apply(ch_a, ch_b)
        est = estimate_block(
            cal_a, cal_b, self.cfg,
            cancel_signal=self.opt.cancel_signal,
            n0=prep["n0"],
            skew_prior_samples=self.state.skew_target_ps * 1e-12 * self.cfg.fs_adc,
        )
        est.rotation = prep["rotation"]

        row = self._measure(ch_a, ch_b, cal_a, cal_b, est, prep["n0"])
        row["swapped"] = prep["swapped"]
        row["align_margin"] = prep["align_margin"]

        reject = self._reject_reason(prep, est)
        row["rejected"] = reject or ""
        if reject:
            self.log.append(row)
            return row

        errors = self.state.update(est)
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
        """Guard the LMS against frames the estimator could not trust."""
        if prep["align_margin"] < self.opt.min_align_margin:
            return f"align_margin={prep['align_margin']:.1f}"
        if est.ch_a.n_events_used < 2 or est.ch_b.n_events_used < 2:
            return "too few dither events in the capture"
        for tag, ch in (("A", est.ch_a), ("B", est.ch_b)):
            if not np.isfinite(ch.gain_codes) or not np.isfinite(ch.skew_samples):
                return f"channel {tag} estimate not finite"
        # Only the mismatch is a defect; the sub-sample phase both channels share
        # against the DPG loop is a property of the clock path, not an error.
        residual = (est.skew_mismatch_ps - self.state.skew_target_ps) * 1e-12 * self.cfg.fs_adc
        if abs(residual) > self.opt.max_skew_samples:
            return f"skew mismatch residual={residual:.3f} samples out of range"
        if abs(est.gain_ratio - 1.0) > self.opt.max_gain_deviation:
            return f"gain ratio={est.gain_ratio:.3f} out of range"
        return None

    def _measure(self, raw_a, raw_b, cal_a, cal_b, est: BlockEstimate, n0: int) -> dict:
        fs = self.cfg.fs_adc
        f_in = self.cfg.f_sig

        # Strip the injected dither before scoring; see synthesize_dither().
        d_a = synthesize_dither(cal_a.size, n0, self.cfg, est.ch_a.gain_codes)
        d_b = synthesize_dither(cal_b.size, n0, self.cfg, est.ch_b.gain_codes)
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
            "skew_a_ps": est.ch_a.skew_ps,
            "skew_b_ps": est.ch_b.skew_ps,
            "skew_mismatch_ps": est.skew_mismatch_ps,
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
        failures = 0
        for _ in range(iterations):
            row = self.step()
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
                if verbose:
                    print(f"  it {row['iteration']:4d}  rejected: {row['rejected']}")
                continue
            if verbose:
                print(
                    f"  it {row['iteration']:4d}  "
                    f"g_B/g_A={row['gain_ratio']:+.5f}  "
                    f"dOffset={row['offset_b_codes'] - row['offset_a_codes']:+8.3f} LSB  "
                    f"dSkew={row['skew_mismatch_ps']:+7.3f} ps  "
                    f"SNDR={row['cal_sndr_db']:5.2f} dB  "
                    f"image={row['cal_image_spur_dbc']:6.1f} dBc"
                )
        return self.log

    # -- output -------------------------------------------------------------
    def save(self, out_dir: str | Path, stem: str = "calibration_run") -> dict:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        csv_path = out_dir / f"{stem}.csv"
        if self.log:
            with csv_path.open("w", newline="", encoding="utf-8") as fh:
                writer = csv.DictWriter(fh, fieldnames=list(self.log[0].keys()))
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
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        return {"csv": csv_path, "meta": json_path}

    def plot(self, out_dir: str | Path, stem: str = "calibration_run"):
        """Learning curves: the figures a TCAS submission needs."""
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        if not self.log:
            return None

        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        it = np.array([r["iteration"] for r in self.log])
        fig, ax = plt.subplots(2, 2, figsize=(12, 8))

        ax[0, 0].plot(it, [r["gain_ratio"] for r in self.log])
        ax[0, 0].axhline(1.0, ls="--", lw=1, color="k")
        ax[0, 0].set_title("Gain ratio $g_B/g_A$ (residual)")

        ax[0, 1].plot(
            it, [r["offset_b_codes"] - r["offset_a_codes"] for r in self.log]
        )
        ax[0, 1].axhline(0.0, ls="--", lw=1, color="k")
        ax[0, 1].set_title("Offset mismatch [LSB] (residual)")

        ax[1, 0].plot(it, [r["skew_mismatch_ps"] for r in self.log])
        ax[1, 0].axhline(0.0, ls="--", lw=1, color="k")
        ax[1, 0].set_title("Timing skew mismatch [ps] (residual)")

        ax[1, 1].plot(it, [r["cal_sndr_db"] for r in self.log], label="SNDR calibrated")
        ax[1, 1].plot(it, [r["cal_sfdr_db"] for r in self.log], label="SFDR calibrated")
        ax[1, 1].plot(
            it, [r["raw_sndr_db"] for r in self.log], ls=":", label="SNDR raw"
        )
        ax[1, 1].legend()
        ax[1, 1].set_title("Dynamic performance [dB]")

        for a in ax.ravel():
            a.set_xlabel("iteration")
            a.grid(True, alpha=0.3)

        fig.tight_layout()
        path = out_dir / f"{stem}_learning.png"
        fig.savefig(path, dpi=140)
        plt.close(fig)
        return path
