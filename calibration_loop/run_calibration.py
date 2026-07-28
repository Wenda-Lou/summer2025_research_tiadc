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


def _run(bench, cfg, args, label: str):
    state = CalibrationState(
        mu_offset=args.mu_offset,
        mu_gain=args.mu_gain,
        mu_skew=args.mu_skew,
        skew_target_ps=args.skew_target_ps,
    )
    options = LoopOptions(
        cancel_signal=not args.no_cancellation,
        close_skew_loop=not args.open_skew,
        interleaved=args.interleaved,
    )
    loop = CalibrationLoop(bench, cfg, state=state, options=options)

    print(f"\nRunning {label} for {args.iterations} iterations")
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

    def avg(key):
        vals = [r[key] for r in tail if np.isfinite(r[key])]
        return float(np.mean(vals)) if vals else float("nan")

    print("\nResidual error after convergence (mean of last 20 % of iterations)")
    print(f"  gain ratio        : {avg('gain_ratio'):+.6f}   (target +1.000000)")
    print(f"  offset mismatch   : "
          f"{avg('offset_b_codes') - avg('offset_a_codes'):+.4f} LSB   (target 0)")
    print(f"  skew mismatch     : {avg('skew_mismatch_ps'):+.4f} ps   "
          f"(target {args.skew_target_ps:+.3f})")
    print(f"  SNDR raw -> cal   : {avg('raw_sndr_db'):.2f} -> {avg('cal_sndr_db'):.2f} dB")
    print(f"  SFDR raw -> cal   : {avg('raw_sfdr_db'):.2f} -> {avg('cal_sfdr_db'):.2f} dB")
    print(f"  image spur        : {avg('raw_image_spur_dbc'):.1f} -> "
          f"{avg('cal_image_spur_dbc'):.1f} dBc")
    print(f"  offset spur       : {avg('raw_offset_spur_dbc'):.1f} -> "
          f"{avg('cal_offset_spur_dbc'):.1f} dBc")


def cmd_probe(args) -> None:
    """Measure without actuating.

    Bring-up order matters: a closed loop that is fed a bad estimate will drive
    the hardware somewhere useless and the log will show a converging-looking
    trajectory around the wrong point.  This captures a few frames, prints what
    the estimator saw, and touches nothing.  Run it, check the numbers below are
    sane, and only then close the loop.
    """
    from .estimator import estimate_block, prepare_capture

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
    try:
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                print(f"  {i:>3}  capture failed (no UDP frame)")
                continue
            prep = prepare_capture(raw, cfg, signature=sig)
            if sig is None:
                sig = prep["signature"]
            est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, n0=prep["n0"])
            ests.append(est)
            rows.append((prep, est))
            print(f"  {i:>3} {prep['rotation']:>4} {str(prep['swapped']):>5} "
                  f"{prep['align_margin']:>7.1f} {est.ch_a.n_events_used:>7} "
                  f"{est.ch_a.gain_codes:>9.2f} {est.ch_b.gain_codes:>9.2f} "
                  f"{est.gain_ratio:>9.5f} {est.ch_a.offset_codes:>8.2f} "
                  f"{est.ch_b.offset_codes:>8.2f} {est.skew_mismatch_ps:>9.2f}")
    finally:
        if args.uart:
            bench.close()

    if not rows:
        print("\nNo usable frames. See the troubleshooting table in README / BENCH_GUIDE_CN.")
        return

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
    checks = [
        (mm > 6.0, f"alignment margin {mm:.1f} > 6 -- the dither was found"),
        (all(e.ch_a.n_events_used >= 8 for e in ests),
         "at least 8 dither events per capture"),
        (np.isfinite(gm) and 0.8 < gm < 1.25, f"gain ratio {gm:.4f} is physical"),
        (np.isfinite(gs) and gs < 0.02, f"gain ratio scatter {gs:.5f} < 0.02"),
        (all(e.ch_a.gain_codes > 0 for e in ests), "channel polarity resolved"),
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
    bench = HardwareBench(
        uart_port=args.uart,
        bind_ip=args.bind_ip,
        skew_bias_ps=args.skew_bias_ps,
        allow_super_fine=args.super_fine,
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
        sp.add_argument("--iterations", type=int, default=60)
        sp.add_argument("--mu-offset", dest="mu_offset", type=float, default=0.35)
        sp.add_argument("--mu-gain", dest="mu_gain", type=float, default=0.35)
        sp.add_argument("--mu-skew", dest="mu_skew", type=float, default=0.30)
        sp.add_argument("--skew-target-ps", dest="skew_target_ps", type=float, default=0.0,
                        help="0 for parallel channels; Ts/2 for true 2x interleaving")
        sp.add_argument("--no-cancellation", action="store_true",
                        help="disable main-tone cancellation (slow baseline)")
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
    b.set_defaults(func=cmd_bench)

    return p


def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
