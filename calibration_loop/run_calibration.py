"""
Command-line driver.

    python -m calibration_loop.run_calibration gen   --out waveforms
    python -m calibration_loop.run_calibration sim   --iterations 60
    python -m calibration_loop.run_calibration bench --uart COM3 --iterations 200

``sim`` needs no hardware: it runs the identical loop against the bench model and
prints the recovered parameters next to the ground truth.  Use it to sanity-check
any change before spending bench time.
"""

from __future__ import annotations

import argparse
import numpy as np

from .dither import DitherConfig, write_dac_files
from .estimator import CalibrationState
from .loop import CalibrationLoop, LoopOptions
from .simulate import BenchModel


def _cfg_from_args(args) -> DitherConfig:
    cfg = DitherConfig()
    for name in ("fs_dac", "n_dac_points", "adc_ratio", "sig_cycles", "amp_dbfs",
                 "dither_period_dac", "dither_position_dac", "dither_edge_dac",
                 "dither_top_dac", "dither_scale_lsb", "seed"):
        value = getattr(args, name, None)
        if value is not None:
            setattr(cfg, name, value)
    cfg.validate()
    return cfg


def cmd_gen(args) -> None:
    cfg = _cfg_from_args(args)
    result = write_dac_files(cfg, args.out, stem=args.stem)
    d = result["meta"]["derived"]

    print(f"DAC vector : {result['txt']}")
    print(f"Metadata   : {result['json']}")
    print()
    print(f"  DAC sample rate     : {d['fs_dac_hz'] / 1e6:.3f} MSPS")
    print(f"  vector length       : {d['n_dac_samples']} samples")
    print(f"  loop length         : {d['n_adc_samples_per_loop']} ADC samples")
    print(f"  main tone           : {d['main_tone_hz'] / 1e6:.4f} MHz "
          f"({cfg.sig_cycles} cycles/loop, coherent)")
    print(f"  ADC sample rate     : {d['fs_adc_hz'] / 1e6:.3f} MSPS "
          f"(fs_dac / {cfg.adc_ratio})")
    print(f"  main amplitude      : {d['main_tone_amplitude_lsb']:.0f} LSB "
          f"({cfg.amp_dbfs:+.1f} dBFS)")
    print(f"  dither              : {d['dither_type']}")
    print(f"  dither amplitude    : {d['dither_amplitude_lsb']:.0f} LSB "
          f"({cfg.a_dither * 100:.2f} % FS)")
    print(f"  pulse geometry      : {cfg.dither_edge_dac}/{cfg.dither_top_dac}/"
          f"{cfg.dither_edge_dac} DAC samples = {cfg.edge_r:.0f}/{cfg.top_w:.0f}/"
          f"{cfg.edge_r:.0f} ADC samples")
    print(f"  impulses per loop   : {cfg.n_events} (period {d['slot_period_adc_samples']} "
          f"ADC samples, duty {d['dither_duty_cycle'] * 100:.1f} %)")
    print(f"  polarity seed       : {cfg.seed}  (sum = {d['polarity_sum']:.0f}, exactly balanced)")
    print(f"  min / max / mean    : {d['min_sample']} / {d['max_sample']} / {d['mean_sample']:.3f}")
    print(f"  clipping            : {d['clipping']}")
    print(f"  seamless loop       : {d['seamless_loop']}")


def skew_writes_ok(bench, args) -> bool:
    """May this run actuate the skew delay?

    Hardware: only with the explicit ``--allow-skew-writes`` unlock.  That path
    is now the firmware transaction (``capture.SkewActuator``): one command, one
    answer, readback inside the ACK, and at most one control code of movement per
    call.  The old raw UDP double-write is frozen regardless of this flag.  The
    bench model has no registers, so it is unaffected.
    """
    if not hasattr(bench, "delay"):
        return True
    return bool(getattr(args, "allow_skew_writes", False))


def _run(bench, cfg, args, label: str):
    state = CalibrationState(
        mu_offset=args.mu_offset,
        mu_gain=args.mu_gain,
        mu_skew=args.mu_skew,
        skew_target_ps=args.skew_target_ps,
    )
    tone_free = bool(getattr(args, "tone_free", False))
    # `--tone-free` supplies the *defaults* for a run with no tone; an explicitly chosen route
    # still wins.  Silently discarding `--skew-observable dither_phase` because `--tone-free` was
    # also passed would be the same class of fault as a silent gain fallback: the run would look
    # fine and integrate something the caller did not ask for.
    gain_observable = args.gain_observable
    if tone_free and gain_observable == "tone":
        gain_observable = "dither_mag"
    skew_observable = args.skew_observable
    if tone_free and skew_observable == "phase":
        # `dither_phase`, not `dither_fold`: measured 2026-09-20, the fold projection's scale is
        # ~2.8x low on this bench (6.46 ps/code against the tone route's 19.5 and the phase
        # route's 18.2), so driving it puts the 10 ps deadband ~28 ps wide in truth.
        skew_observable = "dither_phase"
    options = LoopOptions(
        # A tone-free run has no tone to cancel: leaving the least-squares tone fit on would
        # subtract structure from the record instead of a signal.
        cancel_signal=not (args.no_cancellation or tone_free),
        # Skew actuation stays off unless the delay-write path is explicitly
        # unlocked, so plain `bench` behaves like `--open-skew`.  The bench model
        # has no registers to disturb, so it keeps driving the loop.
        close_skew_loop=(not args.open_skew) and skew_writes_ok(bench, args),
        interleaved=args.interleaved,
        gain_observable=gain_observable,
        skew_observable=skew_observable,
        # Only when asked: the default bound is per route (see LoopOptions), and a tone-free
        # run takes its own, wider one.
        **({"max_skew_samples": args.max_skew_samples}
           if getattr(args, "max_skew_samples", None) is not None else {}),
    )
    loop = CalibrationLoop(bench, cfg, state=state, options=options)

    gate = (options.max_skew_samples_dither if tone_free else options.max_skew_samples)
    print(f"\nRunning {label} for {args.iterations} qualified samples"
          + ("  (tone-free)" if tone_free else ""))
    print(f"routes: gain {gain_observable}, skew {skew_observable}"
          + (", tone cancellation OFF" if not options.cancel_signal
             else ", tone cancellation ON")
          + f", skew gate +/-{gate:.2f} samples ({gate * 1e12 / cfg.fs_adc:.0f} ps)")
    print("-" * 100)
    loop.run(args.iterations)

    paths = loop.save(args.out, stem=args.stem)
    print(f"\nlog  : {paths['csv']}")
    print(f"meta : {paths['meta']}")
    try:
        png = loop.plot(args.out, stem=args.stem)
        if png:
            print(f"plot : {png}")
    except Exception as exc:  # matplotlib is optional
        print(f"(plot skipped: {exc})")
    return loop


def cmd_sim(args) -> None:
    cfg = _cfg_from_args(args)
    bench = BenchModel(
        cfg=cfg,
        gain_b=1.0 + args.gain_mismatch,
        offset_a_codes=args.offset_a,
        offset_b_codes=args.offset_b,
        skew_b_ps=args.skew_ps,
        noise_rms_codes=args.noise,
    )
    truth = bench.truth()
    print("Ground truth")
    print(f"  gain ratio        : {truth['gain_ratio']:+.6f}")
    print(f"  offset A / B      : {truth['offset_a_codes']:+.2f} / "
          f"{truth['offset_b_codes']:+.2f} LSB")
    print(f"  skew mismatch     : {truth['skew_mismatch_ps']:+.3f} ps")

    loop = _run(bench, cfg, args, "simulation")

    if not loop.log:
        return
    tail = loop.log[-max(1, len(loop.log) // 5):]
    last = tail[-1]
    controlled_gain = last.get("gain_source", "tone")
    tone_free = controlled_gain == "dither_mag"

    def avg(key):
        vals = [r[key] for r in tail if np.isfinite(r[key])]
        return float(np.mean(vals)) if vals else float("nan")

    print("\nResidual error after convergence (mean of last 20 % of iterations)")
    print(f"  gain ratio        : {avg('gain_ratio'):+.6f}   "
          f"(pulse windows; target +1.000000)")
    if not tone_free:
        print(f"  tone ratio        : {avg('tone_ratio'):+.6f}   "
              f"(target +1.000000; the controlled observable)")
    else:
        # A tone-free run has no tone: `tone_ratio` is then a fit of noise, and quoting it as a
        # controlled observable would be exactly the kind of silent nonsense the route log exists
        # to prevent.  What the loop controls here is the sign-free magnitude ratio.
        print(f"  magnitude ratio   : {avg('gain_mag_ratio'):+.6f}   "
              f"(target +1.000000; the controlled observable)")
    print(f"  offset mismatch   : "
          f"{avg('offset_b_codes') - avg('offset_a_codes'):+.4f} LSB   (target 0)")
    print(f"  skew mismatch     : {avg('skew_used_ps'):+.4f} ps   "
          f"(target {args.skew_target_ps:+.3f}; {last.get('skew_observable', 'phase')} route)")
    if not tone_free:
        print(f"  SNDR raw -> cal   : {avg('raw_sndr_db'):.2f} -> {avg('cal_sndr_db'):.2f} dB")
        print(f"  SFDR raw -> cal   : {avg('raw_sfdr_db'):.2f} -> {avg('cal_sfdr_db'):.2f} dB")
        print(f"  image spur        : {avg('raw_image_spur_dbc'):.1f} -> "
              f"{avg('cal_image_spur_dbc'):.1f} dBc")
        print(f"  offset spur       : {avg('raw_offset_spur_dbc'):.1f} -> "
              f"{avg('cal_offset_spur_dbc'):.1f} dBc")
    else:
        # The spectral columns are scored at the tone frequency, so with the tone at -120 dBFS
        # they describe the noise floor and must not be read as performance.  The tone-free
        # stand-ins are the injected dither's own SNR and the coherent A-B power.
        print("  (no tone: the SNDR/SFDR/image columns are scored at f_in and describe the")
        print("   noise floor -- read the tone-free pair below instead)")
        print(f"  coherent A-B      : {avg('dbc_ab_coherent'):.1f} -> "
              f"{avg('dbc_ab_coherent_cal'):.1f} dBc   (native -> residual, tone-free)")
        print(f"  dither SNR (A)    : {avg('snr_dither_db'):.2f} -> "
              f"{avg('snr_dither_db_cal'):.2f} dB   (raw -> corrected)")
        print(f"  skew routes       : fold {avg('skew_fold_ps'):+.2f} / "
              f"phase {avg('skew_slope_ps'):+.2f} ps   "
              f"(controlled: {last.get('skew_observable', 'phase')})")


def cmd_check(args) -> None:
    """Cross-check a waveform against the rate the converter actually ran at."""
    from .check import check_waveform, suggest_tone
    import json
    from pathlib import Path

    results = check_waveform(args.waveform_json, args.adc_rate)
    print(f"Checking {args.waveform_json}")
    print(f"against a measured ADC rate of {args.adc_rate / 1e6:.3f} MS/s")
    print()
    for r in results:
        print(f"  [{'PASS' if r['ok'] else 'FAIL'}] {r['name']}")
        print(f"         {r['detail']}")
    failed = [r for r in results if not r["ok"]]
    print()
    print(f"{len(results) - len(failed)}/{len(results)} checks passed")

    if failed:
        meta = json.loads(Path(args.waveform_json).read_text(encoding="utf-8"))
        fs_dac = float(meta.get("dac_sample_rate_hz") or meta["config"]["fs_dac"])
        n_dac = int(meta.get("num_samples")
                    or meta["config"]["n_dac_points"])
        period = int(meta.get("dither_period_dac")
                     or meta["config"]["dither_period_dac"])
        tone = float(meta.get("actual_tone_hz")
                     or meta["derived"]["main_tone_hz"])
        picks = suggest_tone(fs_dac, args.adc_rate, n_dac, period, tone)
        if picks:
            print()
            print("Tone bins near the current one that satisfy every check at this rate:")
            for b, f, step, coh in picks:
                print(f"  bin {b:5d}  {f / 1e6:9.4f} MHz   "
                      f"{step:.4f} cycles/event   coherence {coh:.3f}")
        raise SystemExit(1)


def cmd_probe(args) -> None:
    """Measure without actuating.

    Bring-up order matters: a closed loop that is fed a bad estimate will drive
    the hardware somewhere useless and the log will show a converging-looking
    trajectory around the wrong point.  This captures a few frames, prints what
    the estimator saw, and touches nothing.  Run it, check the numbers below are
    sane, and only then close the loop.
    """
    from .estimator import estimate_block, polarity_anchor, prepare_capture

    cfg = _cfg_from_args(args)

    if args.uart:
        from .capture import HardwareBench
        bench = HardwareBench(uart_port=args.uart, bind_ip=args.bind_ip)
        label = f"hardware on {args.uart}"
    else:
        bench = BenchModel(cfg=cfg)
        label = "bench model (no hardware)"

    print(f"Probing {label}: {args.frames} frames, open loop, nothing is driven\n")
    print(f"  expected main tone : {cfg.f_sig / 1e6:.4f} MHz")
    print(f"  expected ADC rate  : {cfg.fs_adc / 1e6:.1f} MS/s "
          f"(fs_dac {cfg.fs_dac / 1e6:.1f} / {cfg.adc_ratio})")
    print(f"  dither events/loop : {cfg.n_events}, "
          f"{cfg.slot_period} ADC samples apart\n")
    print(f"  {'#':>3} {'rot':>4} {'swap':>5} {'margin':>7} {'events':>7} "
          f"{'gainA':>9} {'gainB':>9} {'gB/gA':>9} {'offA':>8} {'offB':>8} {'dSkew ps':>9}")
    print("  " + "-" * 96)

    rows, sig, ests = [], None, []
    captured = []
    try:
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                print(f"  {i:>3}  capture failed (no UDP frame)")
                continue
            prep = prepare_capture(raw, cfg, signature=sig)
            if sig is None:
                sig = prep["signature"]
            captured.append((i, prep))
    finally:
        if args.uart:
            bench.close()

    if not captured:
        print("\nNo usable frames. See the troubleshooting table in README / BENCH_GUIDE.md.")
        return

    # Absolute polarity is a property of the analog path, not of an individual
    # capture.  A single low-SNR frame occasionally votes for the opposite sign,
    # so anchoring the whole session to frame zero made otherwise identical probe
    # runs disagree.  Take the majority over the complete measurement-only batch;
    # ties are broken by the better-aligned half of the votes.
    polarity_votes = np.asarray([polarity_anchor(prep) for _, prep in captured])
    vote_sum = float(polarity_votes.sum())
    if vote_sum == 0.0:
        weighted_vote = sum(
            polarity_anchor(prep) * prep["align_margin"] for _, prep in captured
        )
        polarity = -1.0 if weighted_vote < 0.0 else 1.0
    else:
        polarity = -1.0 if vote_sum < 0.0 else 1.0
    polarity_agreement = float(np.mean(polarity_votes == polarity))
    n_agree = int(np.count_nonzero(polarity_votes == polarity))
    print(f"  dither polarity anchor: {polarity:+.0f} "
          f"(batch consensus {n_agree}/{len(captured)} frames)")

    batch = []
    for i, prep in captured:
        est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, n0=prep["n0"],
                             polarity_sign=polarity, pin_polarity=True)
        ests.append(est)
        rows.append((prep, est))
        batch.append((prep["ch_a"], prep["ch_b"], prep["n0"]))
        print(f"  {i:>3} {prep['rotation']:>4} {str(prep['swapped']):>5} "
              f"{prep['align_margin']:>7.1f} {est.ch_a.n_events_used:>7} "
              f"{est.ch_a.gain_codes:>9.2f} {est.ch_b.gain_codes:>9.2f} "
              f"{est.gain_ratio:>9.5f} {est.ch_a.offset_codes:>8.2f} "
              f"{est.ch_b.offset_codes:>8.2f} {est.skew_mismatch_ps:>9.2f}")

    # Cross-frame joint aggregation.  A single capture only holds 7-8 dither
    # events (1020 samples / 130-sample slot), which leaves the per-frame gain
    # estimate noisy.  Pooling every frame cuts that variance and averages away
    # each frame's local polarity imbalance.
    joint = None
    joint_ratio_uncertainty = np.nan
    if len(batch) >= 2:
        from .estimator import estimate_block_joint
        joint = estimate_block_joint(batch, cfg, polarity_sign=polarity,
                                     pin_polarity=True)
        print(f"\n  JOINT over {len(batch)} frames: "
              f"{joint.ch_a.n_events_used} events/channel")
        print(f"    gain ratio      {joint.gain_ratio:+.5f}")
        print(f"    offset mismatch {joint.offset_mismatch_codes:+.3f} LSB")
        print(f"    skew mismatch   {joint.skew_mismatch_ps:+.3f} ps  "
              f"[{joint.skew_source}]")

        # Estimate the uncertainty of the pooled result with a delete-one-frame
        # jackknife.  The old check compared just two arbitrary five-frame halves;
        # one unlucky half could fail at 0.03 while the next probe passed at 0.003.
        # Every frame now contributes to a deterministic standard-error estimate.
        if len(batch) >= 3:
            leave_one_out = []
            for omitted in range(len(batch)):
                part = batch[:omitted] + batch[omitted + 1:]
                ratio = estimate_block_joint(
                    part, cfg, polarity_sign=polarity,
                    pin_polarity=True,
                ).gain_ratio
                if np.isfinite(ratio):
                    leave_one_out.append(ratio)
            if len(leave_one_out) == len(batch):
                loo = np.asarray(leave_one_out, dtype=np.float64)
                joint_ratio_uncertainty = float(np.sqrt(
                    (len(loo) - 1.0) / len(loo)
                    * np.sum((loo - loo.mean()) ** 2)
                ))
                print(f"    jackknife gain-ratio uncertainty "
                      f"{joint_ratio_uncertainty:.5f}")

    def stat(vals):
        vals = [v for v in vals if np.isfinite(v)]
        return (np.mean(vals), np.std(vals)) if vals else (np.nan, np.nan)

    gm, gs = stat([e.gain_ratio for e in ests])
    om, os_ = stat([e.offset_mismatch_codes for e in ests])
    sm, ss = stat([e.skew_mismatch_ps for e in ests])
    mm = np.mean([p["align_margin"] for p, _ in rows])

    print(f"\n  gain ratio      {gm:+.5f}  +/- {gs:.5f}")
    print(f"  offset mismatch {om:+.3f}  +/- {os_:.3f} LSB")
    print(f"  skew mismatch   {sm:+.3f}  +/- {ss:.3f} ps")
    print(f"  align margin    {mm:.1f} (mean)")

    print("\nSanity checks")
    # The event-count and uncertainty criteria are judged on the JOINT estimate when
    # one is available: a single 1020-sample capture holds at most
    # floor(1020/130) = 7 whole events, so "8 events in one capture" is not
    # reachable with this geometry, while the pooled batch comfortably exceeds it.
    ev_checked = joint.ch_a.n_events_used if joint is not None else None
    if joint is not None and np.isfinite(joint.gain_ratio):
        gm_chk = joint.gain_ratio
        gs_chk = joint_ratio_uncertainty
    else:
        gm_chk, gs_chk = gm, np.nan
    checks = [
        (mm > 6.0, f"alignment margin {mm:.1f} > 6 -- the dither was found"),
        ((ev_checked is None and all(e.ch_a.n_events_used >= 8 for e in ests))
         or (ev_checked is not None and ev_checked >= 8),
         (f"at least 8 dither events "
          + (f"(joint {ev_checked} pooled from {len(batch)} frames)"
             if ev_checked is not None
             else "per capture"))),
        (np.isfinite(gm_chk) and 0.8 < gm_chk < 1.25,
         f"gain ratio {gm_chk:.4f} is physical"),
        (np.isfinite(gs_chk) and gs_chk < 0.02,
         f"gain ratio uncertainty {gs_chk:.5f} < 0.02"
         + (" (joint estimate)" if joint is not None and np.isfinite(joint.gain_ratio)
            else "")),
        (polarity_agreement >= 0.70
         and (joint is None or (joint.ch_a.gain_sign_fit > 0
                                and joint.ch_b.gain_sign_fit > 0)),
         f"channel polarity resolved ({n_agree}/{len(captured)} frame consensus)"),
    ]
    for ok, text in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {text}")
    if all(ok for ok, _ in checks):
        print("\nAll checks passed -- safe to close the loop with the bench command.")
    else:
        print("\nDo NOT close the loop yet. See the troubleshooting table in BENCH_GUIDE.md.")

    if args.plot and rows:
        _plot_probe(rows[-1][1], cfg, args.out, args.stem)


def _plot_probe(est, cfg, out_dir, stem):
    """Plot the averaged pulse replica -- the single most diagnostic figure.

    V[m] must look like the injected pulse. If it is noise, the alignment or the
    DPG vector is wrong; if it is the right shape at the wrong height, the gain
    scale is off; if it is asymmetric, there is skew.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from pathlib import Path

    from .dither import adc_templates

    if est.ch_a.v_profile is None:
        return None
    m, d, _ = adc_templates(cfg)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    for a, tag in ((est.ch_a, "A"), (est.ch_b, "B")):
        ax[0].plot(m, a.v_profile, marker="o", ms=3, label=f"measured ch {tag}")
        ax[1].plot(m, a.u_profile, marker="o", ms=3, label=f"ch {tag}")
    ax[0].plot(m, est.ch_a.gain_codes * d, "k--", lw=1, label="ideal pulse x gain")
    ax[0].set_title("V[m] — polarity-weighted average (gain + skew)")
    ax[1].axhline(0, color="k", lw=1, ls="--")
    ax[1].set_title("U[m] — plain average (offset)")
    for a in ax:
        a.set_xlabel("sample offset from pulse start")
        a.set_ylabel("ADC codes")
        a.grid(True, alpha=0.3)
        a.legend()
    fig.tight_layout()
    path = out_dir / f"{stem}_probe.png"
    fig.savefig(path, dpi=140)
    plt.close(fig)
    print(f"\nplot : {path}")
    return path


def cmd_bench(args) -> None:
    from .capture import HardwareBench  # imported late: needs pyserial

    cfg = _cfg_from_args(args)
    writes = bool(getattr(args, "allow_skew_writes", False))
    if writes:
        print("!! --allow-skew-writes: the skew loop will drive the actuator through\n"
              "   the firmware transaction (adc -cal skew step +/-N).  Skew is a\n"
              "   batch decision, not a per-frame one: 20 accepted frames, then at\n"
              "   most one control code, only outside the 10 ps deadband, and only\n"
              "   after the previous move shifted the error the way the calibrated\n"
              "   step predicts.  Measured step: 7.4 ps per code (the firmware's\n"
              "   nominal 13.8 ps is ~1.9x larger).  The raw UDP delay path stays\n"
              "   frozen.\n")
    bench = HardwareBench(
        uart_port=args.uart,
        bind_ip=args.bind_ip,
        skew_bias_ps=args.skew_bias_ps,
        allow_super_fine=args.super_fine,
        allow_skew_writes=writes,
    )
    try:
        _run(bench, cfg, args, f"hardware loop on {args.uart}")
    finally:
        bench.close()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)

    def add_common(sp):
        sp.add_argument("--out", default="calibration_out")
        sp.add_argument("--stem", default="run")
        sp.add_argument("--fs-dac", dest="fs_dac", type=float)
        sp.add_argument("--n-dac-points", dest="n_dac_points", type=int)
        sp.add_argument("--adc-ratio", dest="adc_ratio", type=int)
        sp.add_argument("--sig-cycles", dest="sig_cycles", type=int)
        sp.add_argument("--amp-dbfs", dest="amp_dbfs", type=float)
        sp.add_argument("--dither-period", dest="dither_period_dac", type=int)
        sp.add_argument("--dither-position", dest="dither_position_dac", type=int)
        sp.add_argument("--dither-edge", dest="dither_edge_dac", type=int)
        sp.add_argument("--dither-top", dest="dither_top_dac", type=int)
        sp.add_argument("--dither-scale", dest="dither_scale_lsb", type=float)
        sp.add_argument("--seed", type=int)

    def add_loop(sp):
        sp.add_argument("--iterations", type=int, default=60,
                        help="qualified samples to collect.  A capture the "
                             "acceptance filter rejects is logged and retried, so it "
                             "does not consume a sample: at the measured ~20 %% "
                             "rejection rate a 300-sample run takes ~370 captures")
        sp.add_argument("--mu-offset", dest="mu_offset", type=float, default=0.35)
        sp.add_argument("--mu-gain", dest="mu_gain", type=float, default=0.35)
        sp.add_argument("--mu-skew", dest="mu_skew", type=float, default=0.30)
        sp.add_argument("--skew-target-ps", dest="skew_target_ps", type=float, default=0.0,
                        help="0 for parallel channels; Ts/2 for true 2x interleaving")
        sp.add_argument("--no-cancellation", action="store_true",
                        help="disable main-tone cancellation (slow baseline)")
        sp.add_argument("--gain-observable", dest="gain_observable",
                        choices=("tone", "dither", "dither_mag"), default="tone",
                        help="which measurement the gain correction integrates: the "
                             "coherent main-tone amplitude ratio (default; what the "
                             "A-B difference spur depends on, and ~10x quieter), the "
                             "dither pulse-window ratio (older behaviour), or "
                             "dither_mag -- the sign-free per-event magnitude ratio, "
                             "which is the route to use with no tone")
        sp.add_argument("--skew-observable", dest="skew_observable",
                        choices=("phase", "dither_phase", "dither_fold"), default="phase",
                        help="which measurement the skew decision integrates: the "
                             "tone-phase route (default; needs a tone), "
                             "dither_phase -- tone-free, fits each channel's folded "
                             "impulse replica against the template at a fractional "
                             "sampling phase and differences the phases (the default for "
                             "--tone-free, because its scale matches the tone route: "
                             "18.2 against 19.5 ps/code measured 2026-09-20), or "
                             "dither_fold -- tone-free, projects the folded A-B difference "
                             "onto the pulse slope (quieter, but its scale read 2.8x low "
                             "on the same run, which widens the deadband to ~28 ps in "
                             "truth).  Check any waveform with tools/timing_route_check.py")
        sp.add_argument("--max-skew-samples", dest="max_skew_samples", type=float, default=None,
                        help="override the per-frame skew gate, in fractions of a sample "
                             "period.  Default: 0.25 with the tone-phase route (0.5 with the "
                             "tone-free routes, where acquisition is expected to start far "
                             "from zero).  Raise it when the run must *acquire*: on the "
                             "2026-09-20 bench the neutral code 24 carries ~-195 ps, i.e. "
                             "0.25 samples, so the default gate cuts through the middle of "
                             "the distribution -- most captures come back 'skew mismatch "
                             "residual=... samples out of range', the batch yield falls "
                             "under skew_min_yield and the actuator never moves.  0.5 is "
                             "what a tone-free run uses and it is inside the tone phase's "
                             "half-period wrap (0.63 samples)")
        sp.add_argument("--tone-free", dest="tone_free", action="store_true",
                        help="shorthand for a run with no reference tone: sets "
                             "--gain-observable dither_mag, --skew-observable "
                             "dither_phase and --no-cancellation (there is no tone to "
                             "cancel, and the least-squares tone fit would otherwise "
                             "subtract structure from the record).  Load a waveform "
                             "generated with `gen --amp-dbfs -120`")
        sp.add_argument("--open-skew", action="store_true",
                        help="measure skew but do not drive the clock delay")
        sp.add_argument("--interleaved", action="store_true",
                        help="the clock path already provides a Ts/2 offset between "
                             "the channels, so score the interleaved stream")

    g = sub.add_parser("gen", help="write the DPG waveform TXT and its metadata")
    add_common(g)
    g.set_defaults(func=cmd_gen, out="waveforms", stem="impulse_dither")

    s = sub.add_parser("sim", help="run the loop against the bench model")
    add_common(s)
    add_loop(s)
    s.add_argument("--gain-mismatch", dest="gain_mismatch", type=float, default=0.021)
    s.add_argument("--offset-a", dest="offset_a", type=float, default=14.0)
    s.add_argument("--offset-b", dest="offset_b", type=float, default=-23.0)
    s.add_argument("--skew-ps", dest="skew_ps", type=float, default=3.6)
    s.add_argument("--noise", type=float, default=3.0)
    s.set_defaults(func=cmd_sim)

    ck = sub.add_parser(
        "check",
        help="cross-check a waveform's metadata against the real ADC rate")
    ck.add_argument("--waveform-json", required=True,
                    help="the .json written next to the DAC vector")
    ck.add_argument("--adc-rate", type=float, required=True,
                    help="ADC sample rate actually used, in Hz")
    ck.set_defaults(func=cmd_check)

    pr = sub.add_parser("probe", help="measure once without driving anything (bring-up)")
    add_common(pr)
    pr.add_argument("--uart", help="e.g. COM3; omit to probe the bench model instead")
    pr.add_argument("--bind-ip", dest="bind_ip", default="0.0.0.0")
    pr.add_argument("--frames", type=int, default=10)
    pr.add_argument("--plot", action="store_true", help="save the pulse-replica diagnostic plot")
    pr.set_defaults(func=cmd_probe, stem="probe")

    b = sub.add_parser("bench", help="run the loop against the ZCU102")
    add_common(b)
    add_loop(b)
    b.add_argument("--uart", required=True, help="e.g. COM3")
    b.add_argument("--bind-ip", dest="bind_ip", default="0.0.0.0")
    b.add_argument("--skew-bias-ps", dest="skew_bias_ps", type=float, default=165.0)
    b.add_argument("--super-fine", action="store_true",
                   help="use the 0.25 ps super-fine field (needs the ad9695_api.c fix)")
    b.add_argument("--allow-skew-writes", dest="allow_skew_writes", action="store_true",
                   help="let the loop drive the actuator through the firmware "
                        "transaction (frozen by default; the raw UDP delay path is "
                        "frozen unconditionally)")
    b.set_defaults(func=cmd_bench)

    return p


def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
