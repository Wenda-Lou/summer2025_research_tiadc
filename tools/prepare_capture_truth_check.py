"""Ground-truth check of prepare_capture's channel ordering and alignment.

``prepare_capture`` has to answer two questions from the bytes alone: where the
``[A A A A B B B B]`` group boundary is, and which converter is channel A.  Both
are invisible in the metrics the loop prints, so this tool checks them against
the samples :class:`~calibration_loop.simulate.BenchModel` actually generated.

For each frame it verifies four things:

  align  |corr(ch_a, ch_b)| ~ 1   -- the two streams describe the SAME instants.
         A half-group mis-split leaves both streams spectrally clean but slides
         them 4 samples apart, which drops this to cos(2*pi*4*f0) = 0.756.

  order  ch_a matches the model's converter A, not converter B, on every frame.
         A flip between frames is the "loop chases its own tail" failure.

  label  ch_b matches converter B (upright, so that g_B/g_A is positive).

  sign   estimate_block's gain_ratio is positive and near the model's truth.

Usage:
    python tools/prepare_capture_truth_check.py [--frames 8] [--no-old]
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
import tempfile

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ADC_FULL_SCALE = 8191


def load_old_estimator():
    """The committed pre-fix estimator, imported as a sibling module."""
    try:
        src = subprocess.run(
            ["git", "-C", REPO, "show", "HEAD:calibration_loop/estimator.py"],
            capture_output=True, check=True,
        ).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None
    tmp = os.path.join(tempfile.mkdtemp(prefix="old_estimator_"), "estimator.py")
    with open(tmp, "wb") as fh:
        fh.write(src)
    spec = importlib.util.spec_from_file_location("calibration_loop._old_estimator", tmp)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["calibration_loop._old_estimator"] = mod
    spec.loader.exec_module(mod)
    return mod


def truth_samples(bench, n_words: int):
    """The exact a/b arrays the next ``capture()`` will pack, with its noise.

    The model draws its noise from ``_rng`` inside ``capture``; draw the identical
    values here and rewind the generator, so the bytes under test and the truth
    used to grade them come from the same draw.
    """
    cfg = bench.cfg
    n_samples = (n_words // 8) * 4
    n = np.arange(n_samples, dtype=np.float64)
    ts_ps = 1e12 / cfg.fs_adc

    base = bench._phase + bench.subsample_phase + n
    t_a = base + (bench.skew_a_ps / ts_ps)
    t_b = base + ((bench.skew_b_ps + bench.skew_cmd_b_ps) / ts_ps)

    a = bench.gain_a * bench.adc_scale * bench.analog(t_a) + bench.offset_a_codes
    b = bench.gain_b * bench.adc_scale * bench.analog(t_b) + bench.offset_b_codes
    a = a + bench._rng.normal(0, bench.noise_rms_codes, a.shape)
    b = b + bench._rng.normal(0, bench.noise_rms_codes, b.shape)
    if bench.invert_b:
        b = -b
    a = np.clip(np.rint(a), -ADC_FULL_SCALE - 1, ADC_FULL_SCALE).astype(np.float64)
    b = np.clip(np.rint(b), -ADC_FULL_SCALE - 1, ADC_FULL_SCALE).astype(np.float64)
    return a, b


def corr(x: np.ndarray, y: np.ndarray) -> float:
    xs, ys = x - x.mean(), y - y.mean()
    d = float(np.sqrt((xs @ xs) * (ys @ ys)))
    return float(xs @ ys) / d if d > 0 else 0.0


def grade(prep: dict, a_true, b_true) -> dict:
    a, b = prep["ch_a"], prep["ch_b"]
    n = a.size
    c_ab = corr(a, b)
    c_aA = corr(a, a_true[:n])
    c_aB = corr(a, b_true[:n])
    return {
        "n": n,
        "corr_ab": c_ab,
        "corr_aA": c_aA,
        "corr_aB": c_aB,
        "align_ok": abs(c_ab) > 0.99,
        "order_ok": abs(c_aA) > 0.99 and abs(c_aA) > abs(c_aB),
        "labelled_a": "A" if abs(c_aA) > abs(c_aB) else "B",
    }


def run_case(tag, cfg, invert_b, frames, noise, old, words):
    from calibration_loop.estimator import estimate_block, prepare_capture
    from calibration_loop.simulate import BenchModel

    print("\n" + "=" * 78)
    print(f"{tag}: invert_b={invert_b}  frames={frames}  noise={noise} codes  "
          f"({words} words = {words // 8 * 4} samples/channel)")
    print("=" * 78)

    new_bench = BenchModel(cfg=cfg, invert_b=invert_b, noise_rms_codes=noise, seed=7)
    truth_ratio = new_bench.gain_b / new_bench.gain_a
    print(f"  model truth: gain_A={new_bench.gain_a:.4f} gain_B={new_bench.gain_b:.4f} "
          f"-> g_B/g_A={truth_ratio:.5f}")

    rows_new, rows_old, ratios = [], [], []
    for i in range(frames):
        raw, a_true, b_true = _frame_with_truth(new_bench, words)

        prep = prepare_capture(raw, cfg)
        g = grade(prep, a_true, b_true)
        g["rot"] = prep["rotation"]
        g["lagged"] = prep["swapped"]
        rows_new.append(g)

        est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, n0=prep["n0"])
        ratios.append(est.gain_ratio)

        if old is not None:
            op = old.prepare_capture(raw, cfg)
            og = grade(op, a_true, b_true)
            og["rot"] = op["rotation"]
            og["lagged"] = None            # the pre-fix code has no such concept
            rows_old.append(og)

    _table("CURRENT prepare_capture", rows_new, ratios, truth_ratio)
    if rows_old:
        _table("PRE-FIX (HEAD) prepare_capture", rows_old, None, truth_ratio)
    return rows_new, rows_old, ratios


def _rng_get(bench):
    """``RandomState.get_state`` / ``Generator.bit_generator.state`` shim."""
    rng = bench._rng
    if hasattr(rng, "get_state"):
        return rng.get_state()
    return rng.bit_generator.state


def _rng_set(bench, state):
    rng = bench._rng
    if hasattr(rng, "set_state"):
        rng.set_state(state)
    else:
        rng.bit_generator.state = state


def _frame_with_truth(bench, words):
    state = _rng_get(bench)
    a_true, b_true = truth_samples(bench, words)
    _rng_set(bench, state)
    return bench.capture(n_words=words), a_true, b_true


def _table(title, rows, ratios, truth_ratio):
    print(f"\n  --- {title} ---")
    print("   frame  rot  lagged   n   |corr(A,B)|   corr(ch_a,trueA)  ch_a is")
    for i, g in enumerate(rows):
        flag = "OK " if (g["align_ok"] and g["order_ok"]) else "BAD"
        print(f"   {flag} {i:>2}  {g['rot']:>3}  {str(g.get('lagged')):>6} {g['n']:>4}   "
              f"{abs(g['corr_ab']):>9.4f}        {g['corr_aA']:>+8.4f}     "
              f"{g['labelled_a']}")

    n_align = sum(g["align_ok"] for g in rows)
    n_order = sum(g["order_ok"] for g in rows)
    labels = {g["labelled_a"] for g in rows}
    n_lag = sum(bool(g.get("lagged")) for g in rows)
    print(f"   aligned (|corr|>0.99): {n_align}/{len(rows)}   "
          f"ordered as A: {n_order}/{len(rows)}   "
          f"labels seen: {sorted(labels)}   lagged corrections: {n_lag}")
    if ratios:
        finite = [r for r in ratios if np.isfinite(r)]
        if finite:
            print(f"   gain_ratio: mean={np.mean(finite):+.5f} "
                  f"std={np.std(finite):.5f} min={min(finite):+.5f} max={max(finite):+.5f}"
                  f"   (truth {truth_ratio:+.5f}, sign "
                  f"{'POSITIVE ok' if min(finite) > 0 else 'NEGATIVE -- polarity broken'})")


def guard_selftest(cfg, words: int = 2048) -> bool:
    """Check the half-group lag detector in both directions.

    ``deframe`` only consults the lag test when the raw winner might be the
    shifted candidate, which does not happen at the nominal geometry -- so the
    detector itself is exercised here on a frame whose two adjacent splits (the
    correct one and the half-group one) are both built explicitly.
    """
    from calibration_loop.estimator import _lag_corr, prepare_capture, split_at, unpack_words
    from calibration_loop.simulate import BenchModel

    bench = BenchModel(cfg=cfg, seed=11)
    raw = bench.capture(n_words=words)
    prep = prepare_capture(raw, cfg)
    rot = prep["rotation"]

    ok = True
    print("\n  --- half-group lag detector ---")
    for tag, r, expect_lagged in (("aligned (rot)", rot, False),
                                  ("lagged (rot+4)", (rot + 4) % 8, True)):
        fr = split_at(unpack_words(raw), r)
        c0 = abs(_lag_corr(fr["ch_a"], fr["ch_b"], 0))
        c4 = abs(_lag_corr(fr["ch_a"], fr["ch_b"], 4))
        detected = c4 > c0
        good = detected == expect_lagged
        ok = ok and good
        print(f"   {'OK ' if good else 'BAD'} {tag:<16} |corr@lag0|={c0:.4f}  "
              f"|corr@lag4|={c4:.4f}  -> detected lagged={detected} "
              f"(expected {expect_lagged})")
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--words", type=int, default=2048, help="16-bit JESD words/frame")
    ap.add_argument("--no-old", action="store_true")
    args = ap.parse_args(argv)

    sys.path.insert(0, REPO)
    from calibration_loop.dither import DitherConfig

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    print(f"geometry: f0={f0:.6f} cycles/sample  "
          f"lag-4 correlation cos(2*pi*4*f0)={np.cos(2*np.pi*4*f0):+.4f}")

    old = None if args.no_old else load_old_estimator()

    failures = 0
    if not guard_selftest(cfg, args.words):
        failures += 1
    for invert_b in (True, False):
        for noise in (3.0, 12.0):
            rows, rows_old, ratios = run_case(
                f"bench-like capture (noise {noise:g} codes)", cfg, invert_b,
                args.frames, noise, old, args.words)
            if not all(g["align_ok"] and g["order_ok"] for g in rows):
                failures += 1
            finite = [r for r in ratios if np.isfinite(r)]
            if finite and min(finite) <= 0:
                failures += 1

    print("\n" + "=" * 78)
    if failures:
        print(f"RESULT: {failures} case(s) FAILED -- ordering/alignment not trustworthy.")
    else:
        print("RESULT: PASS -- every frame aligned, ordered as converter A, "
              "with a positive gain ratio.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
