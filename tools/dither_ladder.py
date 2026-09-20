"""Amplitude ladder: how far the dither can be pushed, and what it buys.

The full-scale question (2026-09-19 session, stage B) is where the excitation stops being
linear.  The ladder measured so far is 2000 / 8000 / 16000 LSB at a 6-sample pulse, and what it
showed is that the *timing* readout keeps improving with amplitude while ``B/A mag`` drifts
~2.4 % over the range -- so a gain figure is only meaningful at a stated dither amplitude, and
the open question is whether that drift is a real differential nonlinearity or a readout
artifact.  Pushing on to 24000 and 30000 LSB (73 % and 92 % of the DAC's full scale) is what
answers it, and this tool is what turns those captures into the answer.

Per state it reports, from the impulses alone (no tone needed):

  * replica peak-to-peak, one-sided peak, FWHM and peak slope (channel A);
  * ``B/A mag``, the sign-free per-event magnitude ratio, and ``DC(B-A)`` in codes;
  * the per-frame timing scatter from *both* routes -- ``dt_phase`` (the fractional-phase fit
    the loop uses) and ``dt_fold`` (the older derivative projection, kept because every
    published ladder number was measured with it);
  * the coherent dither SNR and the coherent A-B power, the tone-free performance stand-ins.

and across states the linearity check that matters: the measured amplitude ratio between
consecutive states against the commanded one (x1.5 for 16000->24000, x1.875 for 16000->30000).
A ratio that tracks the command means the chain is still linear there; a ratio below it locates
the compression knee, and the state's own numbers say where it comes from (a flat-topped,
clipped replica has a collapsed slope, which the timing route needs).

    python tools/dither_ladder.py \
        --state 16000=calibration_out/response/B_w06_amp16000/raw \
        --state 24000=calibration_out/response/B_w06_amp24000/raw \
        --state 30000=calibration_out/response/B_w06_amp30000/raw \
        --state-json 16000=waveforms/pulse_ladder/w06adc_e04_t04_amp16000.json \
        --state-json 24000=waveforms/pulse_ladder/w06adc_e04_t04_amp24000.json \
        --state-json 30000=waveforms/pulse_ladder/w06adc_e04_t04_amp30000.json

Selection rule: one ``--state`` per *directory* (or per glob) of frames captured at one setting.
Pointing two states at the same directory averages them together, which is never what is wanted;
the tool warns when it sees the same file in two states.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                      # noqa: E402
from calibration_loop.dither_raw import measure, fold_replica, replica_baseline  # noqa: E402
from calibration_loop.estimator import prepare_capture                # noqa: E402

FRAME_BYTES = 4095
DEFAULT_OUT = os.path.join(REPO, "calibration_out", "dither_ladder")


def load_states(specs) -> dict:
    out = {}
    for item in specs:
        if "=" not in item:
            raise SystemExit(f"--state needs NAME=PATH or NAME=GLOB, got {item!r}")
        name, path = item.split("=", 1)
        if os.path.isdir(path):
            files = sorted(glob.glob(os.path.join(path, "*.bin")))
        else:
            files = sorted(glob.glob(path))
        if not files:
            raise SystemExit(f"no frames found for state {name!r} at {path!r}")
        out[name] = files
    seen: dict = {}
    for name, files in out.items():
        for f in files:
            if f in seen:
                print(f"warning: {f} is in both {seen[f]!r} and {name!r} -- the two states "
                      f"will be averaged together (give each run its own --out)")
            seen[f] = name
    return out


def frames_in(blob: bytes) -> list[bytes]:
    """Split a stored file into 4095-byte frame payloads, accepting both layouts.

    ``dither_response_test.py`` writes one file per frame; a run concatenated into a single file
    is also accepted, and the record layout (4096 with a trailing byte) is distinguished from the
    payload layout (4095) exactly as ``dither_raw_evidence.py`` does.
    """
    if len(blob) <= 4096:
        return [blob[:FRAME_BYTES]]
    if len(blob) % 4096 == 0:
        return [blob[i * 4096:i * 4096 + FRAME_BYTES] for i in range(len(blob) // 4096)]
    if len(blob) % FRAME_BYTES == 0:
        return [blob[i * FRAME_BYTES:(i + 1) * FRAME_BYTES]
                for i in range(len(blob) // FRAME_BYTES)]
    raise ValueError(f"{len(blob)} bytes is neither a frame, a record, nor a run of them")


def measure_state(files, cfg: DitherConfig, min_margin: float) -> dict:
    rows = []
    n_frames = 0
    pooled_a, pooled_b = [], []
    for path in files:
        for blob in frames_in(open(path, "rb").read()):
            n_frames += 1
            prep = prepare_capture(blob, cfg)
            # The amplitude is quoted from the *pooled* fold: folding averages the noise over the
            # ~8 visible events, and pooling the frames averages it again over the whole state.
            # Taking the peak-to-peak of each frame and averaging those instead leaves a residual
            # noise bias -- measured on the 2026-09-19 bench ladder, a 48.7-code replica reads
            # 51.1 codes that way, i.e. +5 %, which is enough to fabricate a compression at the
            # bottom of the ladder.
            rep_a, m, n0, sign, margin, mag_a = fold_replica(prep["ch_a"], cfg)
            rep_b, _, _, _, _, mag_b = fold_replica(prep["ch_b"], cfg, n0=n0, sign=sign)
            pooled_a.append(rep_a)
            pooled_b.append(rep_b)
            mm = measure(prep["ch_a"], prep["ch_b"], cfg, min_margin=min_margin)
            if np.isfinite(mm.gain_mag_ratio) and np.isfinite(mm.skew_slope_ps):
                rows.append(mm)
    if not rows:
        return {"frames": n_frames, "aligned": 0}

    def col(attr):
        return np.array([getattr(r, attr) for r in rows], dtype=float)

    def mean(attr):
        v = col(attr)
        v = v[np.isfinite(v)]
        return float(np.mean(v)) if v.size else float("nan")

    def sd(attr):
        v = col(attr)
        v = v[np.isfinite(v)]
        return float(np.std(v, ddof=1)) if v.size > 1 else float("nan")

    rep_a_pool = np.mean(np.asarray(pooled_a), axis=0)
    rep_b_pool = np.mean(np.asarray(pooled_b), axis=0)
    return {
        "frames": n_frames,
        "aligned": len(rows),
        "rep_pp_a": float(rep_a_pool.max() - rep_a_pool.min()),
        "rep_pp_b": float(rep_b_pool.max() - rep_b_pool.min()),
        "rep_pp_a_frame": mean("rep_pp_a"),
        "mag_a": mean("mag_a"),
        "mag_b": mean("mag_b"),
        "peak_a": float(rep_a_pool.max() - replica_baseline(rep_a_pool)),
        "fwhm_a": mean("fwhm_a"),
        "slope_a": mean("slope_peak_a"),
        "gain_mag": mean("gain_mag_ratio"),
        "gain_mag_sd": sd("gain_mag_ratio"),
        "gain_fold": mean("gain_fold_ratio"),
        "gain_fold_sd": sd("gain_fold_ratio"),
        "dc_codes": mean("offset_codes"),
        "dc_sd": sd("offset_codes"),
        "dt_phase": mean("skew_slope_ps"),
        "dt_phase_sd": sd("skew_slope_ps"),
        "dt_fold": mean("skew_fold_ps"),
        "dt_fold_sd": sd("skew_fold_ps"),
        "snr_dither": mean("snr_dither_db"),
        "dbc_ab": mean("dbc_ab_coherent"),
        "margin": mean("align_margin"),
    }


def ordered(names, amp_of) -> list[str]:
    def key(name):
        v = amp_of.get(name, np.nan)
        return (not np.isfinite(v), v)
    return sorted(names, key=key)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", action="append", required=True, metavar="NAME=PATH",
                    help="one captured state: a directory of *_frame_*.bin files, a glob, or a "
                         "single .bin")
    ap.add_argument("--state-json", action="append", default=[], metavar="NAME=JSON",
                    help="waveform JSON per state (defaults to --waveform-json)")
    ap.add_argument("--waveform-json",
                    default=os.path.join(REPO, "waveforms", "pulse_ladder",
                                         "w06adc_e04_t04_amp16000.json"),
                    help="waveform JSON used for any state without --state-json")
    ap.add_argument("--min-margin", type=float, default=6.0,
                    help="alignment-margin floor; below it the frame is counted but not measured")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--step-ps", type=float, default=None,
                    help="ps per actuator control code, for the 'one code is N sigma' line.  "
                         "Default: SkewActuator.HOST_STEP_PS (19.1, the working-point measurement "
                         "of 2026-09-20), so there is one source of truth.  The step is not "
                         "uniform (~7 ps/code near neutral), so pass the value for the code you "
                         "are actually working at.")
    args = ap.parse_args(argv)
    if args.step_ps is None:
        from calibration_loop.capture import SkewActuator
        args.step_ps = SkewActuator.HOST_STEP_PS

    states = load_states(args.state)
    json_of = {}
    for item in args.state_json:
        name, path = item.split("=", 1)
        json_of[name] = path

    results, amps = {}, {}
    for name in states:
        path = json_of.get(name, args.waveform_json)
        meta = json.loads(open(path, encoding="utf-8").read())
        cfg = DitherConfig(**meta["config"])
        cfg.validate()
        amps[name] = float(meta["derived"]["dither_amplitude_lsb"])
        results[name] = measure_state(states[name], cfg, args.min_margin)
        results[name]["json"] = path

    order = ordered(list(states), amps)
    print("dither amplitude ladder, measured from the folded impulses (no tone needed)")
    print(f"  alignment-margin floor {args.min_margin:.1f}; a frame below it is counted, "
          f"not measured")
    print("  rep = pooled fold of the whole state, peak-to-peak (quote this); "
          "rep/f = mean of the per-frame peak-to-peaks (noise-biased at low amplitude)")
    header = (f"  {'state':>8}{'LSB DAC':>9}{'%FS':>6}{'frames':>8}{'rep A':>8}{'rep B':>8}"
              f"{'rep/f':>7}{'FWHM':>6}{'slope':>8}{'B/A mag':>9}{'sd':>7}{'DC(B-A)':>9}"
              f"{'dt phase':>10}{'sd':>7}{'dt fold':>9}{'sd':>7}{'dith SNR':>9}")
    print(header)
    print("  " + "-" * (len(header) - 2))
    for name in order:
        r = results[name]
        if not r.get("aligned"):
            print(f"  {name:>8}{amps[name]:>9.0f}{100 * amps[name] / 32767:>6.1f}"
                  f"{r['frames']:>8}   no measurable frame (margin floor {args.min_margin})")
            continue
        print(f"  {name:>8}{amps[name]:>9.0f}{100 * amps[name] / 32767:>6.1f}"
              f"{r['aligned']:>5}/{r['frames']:<2}"
              f"{r['rep_pp_a']:>8.1f}{r['rep_pp_b']:>8.1f}{r['rep_pp_a_frame']:>7.1f}"
              f"{r['fwhm_a']:>6.1f}"
              f"{r['slope_a']:>8.2f}{r['gain_mag']:>9.4f}{r['gain_mag_sd']:>7.4f}"
              f"{r['dc_codes']:>9.2f}{r['dt_phase']:>10.1f}{r['dt_phase_sd']:>7.1f}"
              f"{r['dt_fold']:>9.1f}{r['dt_fold_sd']:>7.1f}{r['snr_dither']:>9.2f}")

    # ---- linearity across states -------------------------------------------
    print("\nlinearity: measured replica ratio against the commanded amplitude ratio")
    print(f"  {'pair':>18}{'commanded':>11}{'rep A ratio':>13}{'rep B ratio':>13}"
          f"{'slope ratio':>13}{'dt scatter':>12}")
    checks: list[tuple[str, bool, str]] = []
    usable = [n for n in order if results[n].get("aligned")]
    pairs = []
    for lo, hi in zip(usable, usable[1:]):
        a_lo, a_hi = amps[lo], amps[hi]
        if a_lo <= 0:
            continue
        cmd = a_hi / a_lo
        ra = results[hi]["rep_pp_a"] / results[lo]["rep_pp_a"]
        rb = results[hi]["rep_pp_b"] / results[lo]["rep_pp_b"]
        rs = results[hi]["slope_a"] / results[lo]["slope_a"]
        # scatter should fall as the replica grows, if the route is slope-limited
        sc = results[hi]["dt_phase_sd"] / results[lo]["dt_phase_sd"]
        pairs.append((lo, hi, cmd, ra, rb, rs, sc, results[hi]["gain_mag"]))
        print(f"  {lo + '->' + hi:>18}{cmd:>11.3f}{ra:>13.3f}{rb:>13.3f}{rs:>13.3f}"
              f"{sc:>12.3f}")

    for lo, hi, cmd, ra, rb, rs, sc, mag in pairs:
        checks.append((
            f"[{lo}->{hi}] the replica still tracks the command (within 4 %)",
            abs(ra / cmd - 1.0) < 0.04,
            f"commanded x{cmd:.3f}, measured x{ra:.3f} ({100 * (ra / cmd - 1):+.1f} %)"))

    for name in usable[1:]:
        r = results[name]
        checks.append((f"[{name}] the replica is not clipped or flat-topped",
                       r["fwhm_a"] > 0 and r["slope_a"] > 0
                       and r["peak_a"] < 0.95 * 8192,
                       f"peak {r['peak_a']:.0f} codes = "
                       f"{100 * r['peak_a'] / 8192:.1f} % of ADC full scale, "
                       f"FWHM {r['fwhm_a']:.1f} samples, slope {r['slope_a']:.1f} codes/sample"))

    # ---- what the amplitude buys ------------------------------------------
    print("\ntiming precision against replica amplitude "
          "(scatter should fall as 1/replica if the route is slope-limited)")
    for name in usable:
        r = results[name]
        print(f"  {name:>8}: rep A {r['rep_pp_a']:>7.1f} codes, slope {r['slope_a']:>7.2f} "
              f"codes/sample, dt scatter {r['dt_phase_sd']:>6.1f} ps "
              f"({r['dt_fold_sd']:>6.1f} ps on the fold route); one actuator code "
              f"({args.step_ps:.1f} ps) is {args.step_ps / r['dt_phase_sd']:.2f} sigma per frame")

    out_dir = args.out
    os.makedirs(out_dir, exist_ok=True)

    def table_identity(path: str):
        """The (state, amplitude) set a table file already holds, or None if unreadable."""
        try:
            with open(path, newline="", encoding="utf-8") as fh:
                return {(row.get("state", ""), row.get("amplitude_lsb", ""))
                        for row in csv.DictReader(fh)}
        except (OSError, csv.Error):
            return None

    # Several different runs want this tool -- a one-state health check ("is the waveform I loaded
    # the one I think it is?"), a multi-state amplitude ladder, and an off/on pair for a known step
    # at one amplitude -- and they collide on the file name.  Measured twice on 2026-09-20: a
    # one-state check replaced a recorded three-state ladder, and then an off/on pair at another
    # amplitude replaced the off/on pair at the first (protecting only the canonical name was not
    # enough).  So: never overwrite a table that holds a different (state, amplitude) set.  Same
    # content is a re-run and is allowed to overwrite.
    mine = {(name, f"{amps[name]:.0f}") for name in order}
    stem = "dither_ladder"
    for attempt in range(1, 20):
        candidate = os.path.join(out_dir, f"{stem}.csv")
        if not os.path.exists(candidate):
            break
        previous = table_identity(candidate)
        if previous is None or previous == mine:
            break
        if attempt == 1:
            stem = "dither_ladder_" + "_".join(order)
            continue
        stem = "dither_ladder_" + "_".join(order) + f"_{attempt}"
    csv_path = os.path.join(out_dir, f"{stem}.csv")
    if stem != "dither_ladder":
        print(f"note: this directory already holds other tables -- writing this run to "
              f"{os.path.basename(csv_path)} so nothing is overwritten")
    keys = ["state", "amplitude_lsb", "pct_dac_fs", "json"] + [
        k for k in results[order[0]].keys() if k != "json"]
    with open(csv_path, "w", encoding="utf-8") as fh:
        fh.write(",".join(keys) + "\n")
        for name in order:
            r = results[name]
            fh.write(",".join([name, f"{amps[name]:.0f}",
                               f"{100 * amps[name] / 32767:.2f}", r.get("json", "")]
                              + ["" if r.get(k) is None else str(r.get(k))
                                 for k in keys[4:]])
                     + "\n")
    print(f"\ntable  : {csv_path}")

    if len(usable) >= 2:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(1, 2, figsize=(13, 5))
            x = np.array([results[n]["rep_pp_a"] for n in usable])
            y = np.array([amps[n] for n in usable])
            ax[0].plot(x, y, "o-", label="measured")
            ideal = y[0] * x / x[0]
            ax[0].plot(x, ideal, "--", color="k", lw=1,
                       label="linear from the lowest state")
            ax[0].set_xlabel("replica A peak-to-peak [ADC codes]")
            ax[0].set_ylabel("commanded dither amplitude [DAC LSB]")
            ax[0].set_title("amplitude response (a sagging curve is compression)")
            ax[0].legend(fontsize=8)
            ax[0].grid(True, alpha=0.3)

            sd = np.array([results[n]["dt_phase_sd"] for n in usable])
            ax[1].loglog(x, sd, "o-", label="dt scatter (phase route)")
            ax[1].loglog(x, sd[0] * x[0] / x, "--", color="k", lw=1,
                         label="$1/\\mathrm{replica}$")
            for n, xi, yi in zip(usable, x, sd):
                ax[1].annotate(n, (xi, yi), fontsize=8)
            ax[1].set_xlabel("replica A peak-to-peak [ADC codes]")
            ax[1].set_ylabel("per-frame dt scatter [ps]")
            ax[1].set_title("what the amplitude buys")
            ax[1].legend(fontsize=8)
            ax[1].grid(True, alpha=0.3, which="both")

            fig.tight_layout()
            png = os.path.join(out_dir, f"{stem}.png")
            fig.savefig(png, dpi=140)
            plt.close(fig)
            print(f"figure : {png}")
        except Exception as exc:                    # matplotlib is optional
            print(f"(figure skipped: {exc})")

    print()
    ok = True
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} -- {detail}")
        ok = ok and passed
    print(f"\nLADDER: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
