"""Per-rotation table for LIVE frames: is the beat layout still [A A A A B B B B]?

``deframe_rotation_table.py`` does this for saved fixture CSVs.  On the bench the
question is often about the *current* state of the capture path -- a waveform
reload, a link glitch, or a stalled DMA all show up here as a collapse of either
tone purity or the aligned-channel correlation.

Per frame it prints, for all 8 candidate group phases: |corr(ch_a, ch_b)|, tone
purity, and the tone residual of each stream, then the per-channel tone
amplitude / DC / RMS for the phase the de-framer would pick.

A healthy bench frame: one rotation with |corr| ~ 1.0 (the converters are
parallel), clean residuals, and tone amplitudes within a per cent between the
two channels.  Two rotations near |corr| = 0.756 and low purity everywhere means
the beat structure is gone and the fault is upstream of the de-framer.

Usage:
    python tools/live_frame_table.py --uart COM5 --frames 3
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--bind-ip", dest="bind_ip", default=None)
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import (
        _corr, _tone_purity, fit_tone, split_at, unpack_words,
    )

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    print(f"expected tone {cfg.f_sig / 1e6:.4f} MHz  (f0 = {f0:.6f} cycles/sample)")

    kwargs = {"uart_port": args.uart}
    if args.bind_ip:
        kwargs["bind_ip"] = args.bind_ip
    bench = HardwareBench(**kwargs)
    try:
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                print(f"\n-- frame {i}: capture failed --")
                continue
            words = unpack_words(raw)
            print("\n" + "=" * 78)
            print(f"-- frame {i}: {len(raw)} bytes = {words.size} words --")
            print("  rot  |corr|   purity   resid_a  resid_b   tone_a   tone_b")
            best = None
            for rot in range(8):
                try:
                    fr = split_at(words, rot)
                except ValueError as exc:
                    print(f"  {rot:3d}  {exc}")
                    continue
                ra = fit_tone(fr["ch_a"], f0, refine=True)
                rb = fit_tone(fr["ch_b"], f0, refine=True)
                corr = abs(_corr(fr["ch_a"], fr["ch_b"]))
                pur = _tone_purity(fr["ch_a"]) + _tone_purity(fr["ch_b"])
                if best is None or corr > best[1]:
                    best = (rot, corr)
                print(f"  {rot:3d} {corr:7.4f} {pur:8.3f} "
                      f"{np.std(ra['residual']):9.2f} {np.std(rb['residual']):8.2f} "
                      f"{ra['amplitude']:8.2f} {rb['amplitude']:8.2f}")
            fr = split_at(words, best[0])
            print(f"  -> de-framer picks rot={best[0]} (|corr|={best[1]:.4f})")
            resid = {}
            for tag, y in (("A", fr["ch_a"]), ("B", fr["ch_b"])):
                ft = fit_tone(y, f0, refine=True)
                resid[tag] = ft["residual"]
                frac = 0.5 * ft["amplitude"] ** 2 / float(np.mean(y * y))
                print(f"     ch{tag}: dc={y.mean():+9.2f} rms={np.std(y):9.2f} "
                      f"tone_amp={ft['amplitude']:9.2f} fitted_f={ft['f0'] * cfg.fs_adc / 1e6:9.3f} MHz"
                      f"  tone_power_fraction={frac:5.3f}")
            # After the shared tone is removed, what is left?  Independent noise
            # per channel means the corruption sits in each channel's own path
            # (link/data errors); a high correlation means both channels still see
            # one common analog signal, so the fault is upstream in the source.
            rc = float(_corr(resid["A"], resid["B"]))
            print(f"     corr(residual_A, residual_B) = {rc:+.4f}")
            if abs(rc) > 0.5:
                print("     -> residuals are COMMON: the two channels still see one "
                      "analog signal (look upstream of the ADC)")
            else:
                print("     -> residuals are INDEPENDENT: per-channel corruption, "
                      "above the splitter (check the link / capture path)")
    finally:
        bench.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
