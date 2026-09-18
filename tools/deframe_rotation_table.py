"""Per-rotation de-frame table: which split is physically correct, and why.

The DMA restarts on every capture, so the ``[A A A A B B B B]`` group boundary
lands on an arbitrary one of 8 word phases.  Two phases -- ``r`` and ``r + 4`` --
both produce a clean tone in both outputs; they merely start on the other
converter.  They are NOT equivalent: at ``r + 4`` the two output streams are
offset by 4 samples relative to each other, so they are no longer the *same
instants* and their correlation is no longer -1.

For a tone at ``f0`` cycles/sample the lagged correlation is exactly
``cos(2*pi*4*f0)``, which the tool prints as ``theory`` below.  This gives an
analytic fingerprint to check the measurement against:

    rotation r      -> |corr| ~ 1.000   (channels are parallel: same instant)
    rotation r + 4  -> |corr| ~ |cos(2*pi*4*f0)| = 0.756 at the bench geometry

Every other rotation mixes converters inside a group and destroys tone purity.

The tool also replays ``git show HEAD:calibration_loop/estimator.py`` (the
pre-fix implementation) on the identical bytes, so the old failure mode and the
new one can be compared directly.

Usage:
    python tools/deframe_rotation_table.py [--fixture NAME] [--no-old]
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


def load_fixture(path: str) -> bytes:
    arr = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.uint8)
    return arr.astype(np.uint8).tobytes()


def load_old_estimator():
    """Import the committed pre-fix estimator under the calibration_loop package."""
    try:
        src = subprocess.run(
            ["git", "-C", REPO, "show", "HEAD:calibration_loop/estimator.py"],
            capture_output=True, check=True,
        ).stdout
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f"  (old estimator unavailable: {exc})")
        return None
    tmp = os.path.join(tempfile.mkdtemp(prefix="old_estimator_"), "estimator.py")
    with open(tmp, "wb") as fh:
        fh.write(src)
    spec = importlib.util.spec_from_file_location("calibration_loop._old_estimator", tmp)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["calibration_loop._old_estimator"] = mod
    spec.loader.exec_module(mod)
    return mod


def report(tag: str, a, b, cfg, f0) -> None:
    from calibration_loop.estimator import fit_tone
    print(f"\n  --- {tag} ---")
    for nm, y in (("A", a), ("B", b)):
        ft = fit_tone(y, f0, refine=True)
        print(f"    ch{nm}: n={y.size:5d}  dc={y.mean():+9.2f}  tone_amp={ft['amplitude']:8.2f}"
              f"  resid_rms={np.std(ft['residual']):8.2f}")
    if a.size == b.size:
        c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1])
        print(f"    corr(A,B) = {c:+.4f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fixture", default=None, help="single fixture filename")
    ap.add_argument("--no-old", action="store_true", help="skip the pre-fix replay")
    args = ap.parse_args(argv)

    sys.path.insert(0, REPO)
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import _corr, fit_tone, prepare_capture, split_at, unpack_words

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    theory = abs(float(np.cos(2.0 * np.pi * 4.0 * f0)))

    print(f"geometry: sig_cycles={cfg.sig_cycles} n_adc_period={cfg.n_adc_period} "
          f"-> f0={f0:.6f} cycles/sample")
    print(f"analytic lag-4 correlation |cos(2*pi*4*f0)| = {theory:.4f}")

    base = os.path.join(REPO, "firmware", "thesis_v3_500mhz_appl", "adc_data")
    files = sorted(f for f in os.listdir(base) if f.startswith("adc_capture_"))
    if args.fixture:
        files = [f for f in files if f == args.fixture]
    if not files:
        print(f"no fixtures under {base}")
        return 1

    old = None if args.no_old else load_old_estimator()

    for name in files:
        raw = load_fixture(os.path.join(base, name))
        words = unpack_words(raw)
        print("\n" + "=" * 78)
        print(f"{name}   {len(raw)} bytes = {words.size} words")

        print("\n  rot  purity   corr      |corr|   resid_a  resid_b   note")
        best = None
        for rot in range(8):
            try:
                fr = split_at(words, rot)
            except ValueError as exc:
                print(f"  {rot:3d}  -- {exc}")
                continue
            pa = fit_tone(fr["ch_a"], f0, refine=True)["residual"]
            pb = fit_tone(fr["ch_b"], f0, refine=True)["residual"]
            corr = _corr(fr["ch_a"], fr["ch_b"])
            note = ""
            if abs(corr) > 0.99:
                note = "parallel, same instants  <-- VALID"
            elif abs(abs(corr) - theory) < 0.05:
                note = "clean swap, LAG 4 samples"
            if best is None or abs(corr) > best[1]:
                best = (rot, abs(corr))
            print(f"  {rot:3d}  {np.std(pa):6.2f}  {corr:+8.4f}  {abs(corr):6.4f}  "
                  f"{np.std(pa):8.2f} {np.std(pb):8.2f}   {note}")
        print(f"  -> deframe must pick rot={best[0]} (max |corr| = {best[1]:.4f})")

        if old is not None:
            try:
                a, b, pr = (lambda p: (p["ch_a"], p["ch_b"], p))(
                    old.prepare_capture(raw, cfg))
                print(f"\n  PRE-FIX prepare_capture: rot={pr['rotation']} n0={pr['n0']}")
                report("pre-fix", a, b, cfg, f0)
            except Exception as exc:  # noqa: BLE001
                print(f"\n  PRE-FIX prepare_capture raised: {type(exc).__name__}: {exc}")

        try:
            p = prepare_capture(raw, cfg)
            print(f"\n  CURRENT prepare_capture: rot={p['rotation']} swapped={p['swapped']} "
                  f"n0={p['n0']} align_margin={p['align_margin']:.1f}")
            report("current", p["ch_a"], p["ch_b"], cfg, f0)
        except Exception as exc:  # noqa: BLE001
            print(f"\n  CURRENT prepare_capture raised: {type(exc).__name__}: {exc}")

    print("\nExpected: CURRENT must report rot aligned to the max-|corr| row and "
          "corr(A,B) = -1.0000 (parallel converters, one branch inverted).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
