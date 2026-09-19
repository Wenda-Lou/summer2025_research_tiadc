"""Which gain route is actually data-driven?  Three routes on the same saved frames.

The conflict this resolves, measured 2026-09-19 on two independent capture sets at the code-24
baseline: the estimator's ``dither gain`` read **0.99378** both times (identical to five
decimals with 20 and with 100 frames), while the raw folded-replica ratio read 0.9888 and
0.9643 on the two sets.  A quantity that does not move when the data changes cannot be the one
used to claim gain detectability, so the routes are separated here:

  1. ``estimate_block(..., pin_polarity=True)``  -- what ``tools/dither_response_test.py`` prints
  2. ``estimate_block(..., pin_polarity=False)`` -- the same fit without the sign pinning
  3. sign-free per-event magnitude: the peak-to-peak of each impulse window, averaged over
     events with no polarity convention at all.  Polarity only flips a pulse's sign, so a
     magnitude cannot be affected by a session sign or by the pinned anchor.

Route 3 is also split by event polarity: for a linear channel the positive and negative events
must give the same magnitude, so a disagreement is evidence of saturation, a DC-dependent
window, or residual interference rather than of a gain mismatch.

    python calibration_out/_gain_route_test.py
    python calibration_out/_gain_route_test.py --state "calibration_out/response/raw/*.bin"
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "tools"))

from calibration_loop.dither import DitherConfig, polarity_sequence          # noqa: E402
from calibration_loop.estimator import (align_to_loop, estimate_block,      # noqa: E402
                                        polarity_anchor, prepare_capture,
                                        visible_events)
from dither_raw_evidence import decode, iter_frames                         # noqa: E402

DEFAULT_JSON = os.path.join(REPO, "waveforms", "pulse_ladder", "w32adc_e16_t32_amp2000.json")


def magnitudes(x, starts, m):
    """Sign-free per-event magnitudes (peak-to-peak of each impulse window)."""
    out = []
    for s in starts:
        idx = s + m
        if idx.min() < 0 or idx.max() >= x.size:
            continue
        w = x[idx]
        out.append(float(w.max() - w.min()))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", default=os.path.join(REPO, "calibration_out", "response",
                                                    "raw", "*.bin"))
    ap.add_argument("--waveform-json", default=DEFAULT_JSON)
    ap.add_argument("--limit", type=int, default=0, help="use only the first N frames")
    ap.add_argument("--chunks", type=int, default=5,
                    help="split the frames into this many consecutive chunks and report each "
                         "chunk's mean.  A metric whose value does not move between chunks is "
                         "not being driven by the data, whatever its per-frame scatter says.")
    args = ap.parse_args(argv)

    meta = __import__("json").loads(open(args.waveform_json, encoding="utf-8").read())
    cfg = DitherConfig(**meta["config"])
    cfg.validate()

    files = sorted(glob.glob(args.state))
    if args.limit:
        files = files[:args.limit]
    print(f"{len(files)} frames from {args.state}")
    print(f"waveform {os.path.basename(args.waveform_json)}: "
          f"pulse {2 * cfg.edge_r + cfg.top_w:.0f} ADC samples, period {cfg.slot_period}\n")

    m = np.arange(-2, int(np.ceil(cfg.pulse_len)) + 2)
    signs = polarity_sequence(cfg)

    rows = {k: [] for k in ("gain_pin", "gain_nopin", "tone_pin", "tone_nopin",
                            "offset_pin", "offset_nopin", "mag_a", "mag_b",
                            "mag_a_pos", "mag_a_neg", "mag_b_pos", "mag_b_neg")}
    polarity, n_used = None, 0
    for path in files:
        raw = open(path, "rb").read()[:4095]
        rep = next(iter(iter_frames(raw)))
        prep = prepare_capture(rep, cfg)
        if prep.get("n0") is None:
            continue
        if polarity is None:
            polarity = polarity_anchor(prep)
        try:
            e_pin = estimate_block(prep["ch_a"], prep["ch_b"], cfg, cancel_signal=True,
                                   n0=prep["n0"], polarity_sign=polarity, pin_polarity=True)
            e_no = estimate_block(prep["ch_a"], prep["ch_b"], cfg, cancel_signal=True,
                                  n0=prep["n0"], polarity_sign=polarity, pin_polarity=False)
        except Exception as exc:                      # noqa: BLE001
            print(f"  {os.path.basename(path)}: estimate failed: {exc}")
            continue
        rows["gain_pin"].append(e_pin.gain_ratio)
        rows["gain_nopin"].append(e_no.gain_ratio)
        rows["tone_pin"].append(e_pin.tone_ratio)
        rows["tone_nopin"].append(e_no.tone_ratio)
        rows["offset_pin"].append(e_pin.offset_mismatch_codes)
        rows["offset_nopin"].append(e_no.offset_mismatch_codes)

        a, b, _, _ = decode(rep)
        al = align_to_loop(a, cfg)
        ks, starts = visible_events(al["n0"], a.size, cfg, m[0], m[-1] + 1)
        if ks.size == 0:
            continue
        ma, mb = magnitudes(a, starts, m), magnitudes(b, starts, m)
        rows["mag_a"].append(float(np.mean(ma)))
        rows["mag_b"].append(float(np.mean(mb)))
        pos = signs[ks] * al["sign"] > 0
        rows["mag_a_pos"].append(float(np.mean(np.asarray(ma)[pos])) if pos.any() else np.nan)
        rows["mag_a_neg"].append(float(np.mean(np.asarray(ma)[~pos])) if (~pos).any() else np.nan)
        rows["mag_b_pos"].append(float(np.mean(np.asarray(mb)[pos])) if pos.any() else np.nan)
        rows["mag_b_neg"].append(float(np.mean(np.asarray(mb)[~pos])) if (~pos).any() else np.nan)
        n_used += 1

    print(f"estimator routes (n={n_used} frames), gain = gain_ratio, tone = tone_ratio:")
    print(f"  {'route':>18}{'mean':>12}{'sd':>12}")
    for key, label in (("gain_pin", "gain pin=True"), ("gain_nopin", "gain pin=False"),
                       ("tone_pin", "tone pin=True"), ("tone_nopin", "tone pin=False"),
                       ("offset_pin", "offset pin=True"), ("offset_nopin", "offset pin=False")):
        v = np.asarray(rows[key], dtype=float)
        v = v[np.isfinite(v)]
        if v.size:
            print(f"  {label:>18}{v.mean():>12.5f}{v.std(ddof=1):>12.5f}")

    ma = np.asarray(rows["mag_a"], dtype=float)
    mb = np.asarray(rows["mag_b"], dtype=float)
    ok = np.isfinite(ma) & np.isfinite(mb) & (ma > 0)
    if ok.sum() > 1:
        ratio = mb[ok] / ma[ok]
        print(f"\nsign-free per-event magnitude (no polarity convention):")
        print(f"  {'route':>18}{'mean':>12}{'sd':>12}")
        print(f"  {'|A| codes':>18}{ma[ok].mean():>12.2f}{ma[ok].std(ddof=1):>12.2f}")
        print(f"  {'|B| codes':>18}{mb[ok].mean():>12.2f}{mb[ok].std(ddof=1):>12.2f}")
        print(f"  {'B/A magnitude':>18}{ratio.mean():>12.5f}{ratio.std(ddof=1):>12.5f}")

    for ch in ("a", "b"):
        pos = np.asarray(rows[f"mag_{ch}_pos"], dtype=float)
        neg = np.asarray(rows[f"mag_{ch}_neg"], dtype=float)
        ok = np.isfinite(pos) & np.isfinite(neg)
        if ok.sum() > 1:
            print(f"  channel {ch.upper()} by event polarity: + {pos[ok].mean():7.2f} "
                  f"+- {pos[ok].std(ddof=1):5.2f}   - {neg[ok].mean():7.2f} "
                  f"+- {neg[ok].std(ddof=1):5.2f}   (+/- = {pos[ok].mean() / neg[ok].mean():.4f})")

    # does the metric actually move with the data?
    if args.chunks > 1:
        print(f"\nmean per consecutive chunk of frames ({args.chunks} chunks) -- a pinned "
              f"metric repeats the same value, a data-driven one scatters:")
        keys = ("gain_pin", "gain_nopin", "offset_pin", "mag_a")
        print(f"  {'chunk':>8}" + "".join(f"{k:>14}" for k in keys))
        for start in range(args.chunks):
            cells = ""
            for k in keys:
                v = np.asarray(rows[k], dtype=float)[start::args.chunks]
                cells += f"{np.nanmean(v):>14.5f}" if v.size else f"{'n/a':>14}"
            print(f"  {start:>8}{cells}")

    print("\ncross-route correlation across frames:")
    for k1, k2 in (("gain_pin", "gain_nopin"), ("gain_pin", "mag_a"), ("gain_nopin", "mag_a"),
                   ("mag_a", "mag_b"), ("gain_pin", "offset_pin")):
        v1 = np.asarray(rows[k1], dtype=float)[:len(rows[k2])]
        v2 = np.asarray(rows[k2], dtype=float)
        n = min(v1.size, v2.size)
        v1, v2 = v1[:n], v2[:n]
        ok = np.isfinite(v1) & np.isfinite(v2)
        if ok.sum() > 2 and v1[ok].std() > 0 and v2[ok].std() > 0:
            print(f"  corr({k1}, {k2}) = {np.corrcoef(v1[ok], v2[ok])[0, 1]:+.3f}")
        else:
            print(f"  corr({k1}, {k2}) = n/a (one side has no scatter)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
