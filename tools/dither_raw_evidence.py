"""Raw two-channel evidence from the impulses alone -- no tone, no estimator.

The tone-free counterpart of ``tools/raw_waveform_evidence.py``.  With the main tone removed
(``gen --amp-dbfs -120``) the only signal in the captures is the impulse train, so every
number below is read off the folded impulse replica that the ADC actually saw:

  * per-channel folded replica, its peak-to-peak and peak position;
  * the folded **A-B difference replica projected on the known pulse slope** -- the timing
    (skew) signature, in ps.  A shift estimator cannot work here: the sampling grid is 769 ps,
    so a 44 ps skew moves the sampled replica values by about one percent.  Projecting on the
    slope is the only sub-sample route, and it is the one the production estimator uses;
  * the replica amplitude ratio B/A -- the raw gain signature;
  * the record mean difference B-A -- the raw offset signature.

Two things are needed to fold at all, and both come from what was injected rather than from
any fit of the data:

  1. the impulse period (130 ADC samples), and
  2. the **polarity of each event**.  The sequence is a balanced pseudo-random +-1 train, so
     a sign-blind fold cancels the pulses against each other and recovers nothing.  The
     polarity train doubles as a sync word: correlating against the known dither-only loop
     (``align_to_loop``) locates the capture inside the DPG loop, and the sign-corrected
     co-add then recovers the full pulse.

Nothing here runs the calibration loop, projects the pulse template onto the data, anchors a
polarity or corrects anything.

    python tools/dither_raw_evidence.py --state c24=calibration_out/response/raw \\
        --waveform-json waveforms/pulse_ladder/w06adc_e04_t04_amp8000.json
    python tools/dither_raw_evidence.py --state c24=DIR1 --state c32=DIR2 --waveform-json ...

With two or more states the table ends with the change between them, which is what a
"different fixed settings, record long enough" session is for.  ``--remove-tone`` strips a
least-squares sinusoid first, for captures that still contain the tone; it also removes the
DC, so the offset column reads zero -- it is off by default and the mode used is printed.

Accuracy note: on captures that still hold the tone the sinusoid-removal residual is of the
same order as the replica, which biases the cross-correlation lag towards zero.  Those runs
are a mechanics check only; the absolute ps readout is valid on tone-free captures.
``calibration_out/_dither_raw_test.py`` proves both, against known delays.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import (DitherConfig, polarity_sequence,   # noqa: E402
                                     pulse, pulse_derivative)
from calibration_loop.estimator import align_to_loop, visible_events     # noqa: E402

FRAME_BYTES = 4095
RECORD_BYTES = 4096
GROUP, HALF = 8, 4
FS_ADC = 1.300e9
TONE_F0 = 1276 / 8320            # only used by --remove-tone
PS_PER_SAMPLE = 1e12 / FS_ADC
DEFAULT_JSON = os.path.join(REPO, "waveforms", "impulse_dither.json")


def iter_frames(blob: bytes) -> list[bytes]:
    """Split a file into 4095-byte frame payloads, accepting both stored layouts."""
    if len(blob) <= RECORD_BYTES:
        return [blob[:FRAME_BYTES]]
    if len(blob) % RECORD_BYTES == 0:
        return [blob[i * RECORD_BYTES:i * RECORD_BYTES + FRAME_BYTES]
                for i in range(len(blob) // RECORD_BYTES)]
    if len(blob) % FRAME_BYTES == 0:
        return [blob[i * FRAME_BYTES:(i + 1) * FRAME_BYTES]
                for i in range(len(blob) // FRAME_BYTES)]
    raise ValueError(f"{len(blob)} bytes is neither a frame, a record, nor a run of them")


def decode(raw: bytes) -> tuple[np.ndarray, np.ndarray, int, float]:
    """Unpack one frame into (A, B) with B sign-normalised.  Same rules as the firmware path."""
    words = np.frombuffer(raw[:FRAME_BYTES - (FRAME_BYTES % 2)], dtype="<i2")
    groups = words[:words.size // GROUP * GROUP].reshape(-1, GROUP)

    def split(g):
        a = (g[:, :HALF].astype(np.int32) >> 2).ravel().astype(float)
        b = (g[:, HALF:].astype(np.int32) >> 2).ravel().astype(float)
        return a, b

    best, best_c = 0, -1.0
    for r in range(GROUP):
        a, b = split(np.roll(groups, -r, axis=0))
        c = abs(float(np.corrcoef(a, b)[0, 1]))
        if c > best_c:
            best, best_c = r, c
    a, b = split(np.roll(groups, -best, axis=0))
    corr = float(np.corrcoef(a, b)[0, 1])
    if corr < 0:                      # one branch of the splitter is inverted
        b = -b
    return a, b, best, abs(corr)


def remove_tone(x: np.ndarray) -> np.ndarray:
    """Least-squares sinusoid + DC removal (only for captures that still hold the tone)."""
    n = np.arange(x.size)
    span = 0.5 / x.size
    best = None
    for f in np.linspace(TONE_F0 - span, TONE_F0 + span, 41):
        w = 2.0 * np.pi * f * n
        design = np.column_stack([np.cos(w), np.sin(w), np.ones_like(w)])
        coef, *_ = np.linalg.lstsq(design, x, rcond=None)
        sse = float(((x - design @ coef) ** 2).sum())
        if best is None or sse < best[0]:
            best = (sse, design @ coef)
    return x - best[1]


def replica_of(x: np.ndarray, cfg: DitherConfig, n0: int | None = None,
               sign: float | None = None):
    """Sign-corrected co-add of the impulses in one record.

    ``n0`` is the capture's position in the DPG loop; supplied from channel A so both
    channels are sliced at the *same* loop position (a per-channel alignment would absorb
    the very timing difference being measured).

    Returns ``(replica, m, n0, sign, margin, magnitude)`` where ``magnitude`` is the mean
    peak-to-peak of the *raw* impulse windows.  That magnitude is the sign-free gain route:
    polarity only flips a pulse's sign, so a peak-to-peak cannot be affected by a session sign
    or by the polarity anchor.  Measured 2026-09-19 it agrees with the folded replica ratio to
    0.02 % while the estimator's ``gain_ratio`` sits ~3 % away, which is why both are reported.
    """
    guard = 2
    align = align_to_loop(x, cfg)
    if n0 is None:
        n0 = align["n0"]
    if sign is None:
        sign = align["sign"]
    m_lo, m_hi = -guard, int(np.ceil(cfg.pulse_len)) + guard
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
    mag = float(np.mean(mags)) if mags else float("nan")
    return acc, m, n0, sign, float(align["margin"]), mag


def slope_delay(rep_d: np.ndarray, rep_ref: np.ndarray, m: np.ndarray,
                cfg: DitherConfig) -> float:
    """Timing from the folded A-B difference, in ADC samples.

    A relative delay turns the pulse into its own derivative, so the difference replica is
    projected onto the known pulse slope: ``dt = <A-B, d/dt> / <d/dt, d/dt>``.  This is the
    only route with sub-sample resolution -- the sampling grid is 769 ps, so a 44 ps skew
    moves the sampled replica values by about one percent and a shift estimator cannot see it.
    The slope comes from the injected pulse shape, not from a fit of the data.
    """
    dp = pulse_derivative(m, cfg.edge_r, cfg.top_w)
    pp = float(pulse(m, cfg.edge_r, cfg.top_w).max() - pulse(m, cfg.edge_r, cfg.top_w).min())
    scale = (float(rep_ref.max() - rep_ref.min()) / pp) if pp else 0.0
    dp = dp * scale
    den = float(dp @ dp)
    return float(rep_d @ dp) / den if den else float("nan")


def peak_pos(rep: np.ndarray, m: np.ndarray) -> float:
    """Sub-sample peak position of a replica, in ADC samples from the nominal pulse start."""
    k = int(np.argmax(np.abs(rep)))
    if 0 < k < rep.size - 1:
        y0, y1, y2 = abs(rep[k - 1]), abs(rep[k]), abs(rep[k + 1])
        den = y0 - 2 * y1 + y2
        d = 0.5 * (y0 - y2) / den if den else 0.0
    else:
        d = 0.0
    return float(m[k] + d)


def orient(rep: np.ndarray) -> np.ndarray:
    """Make the replica's dominant extremum positive, so a lag has one sign convention."""
    k = int(np.argmax(np.abs(rep)))
    return rep if rep[k] >= 0 else -rep


def width_metrics(rep: np.ndarray, m: np.ndarray) -> tuple[float, float]:
    """Full width at half maximum (ADC samples) and steepest slope (codes/sample).

    These are the two quantities that answer "the dither should be shorter": the FWHM says
    whether the ADC actually sees a narrower impulse, and the peak slope says whether the
    timing route gains anything from it (the slope route's sensitivity is proportional to
    ``d(pulse)/dt``).  Half-maximum is taken above a baseline read from the replica's outer
    edges, so a DC or artifact offset in the window cannot inflate the width.
    """
    base = float(np.median(np.concatenate([rep[:3], rep[-3:]])))
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
        t = (half - rep[i]) / (rep[j] - rep[i])
        return float(m[i] + t * (m[j] - m[i]))

    left = cross(i0 - 1, i0) if i0 > 0 else float(m[i0])
    right = cross(i1, i1 + 1) if i1 < rep.size - 1 else float(m[i1])
    return abs(right - left), slope


def xcorr_lag(ra: np.ndarray, rb: np.ndarray) -> tuple[float, float]:
    """Lag of B against A from the cross-correlation peak, sub-sample, plus its strength.

    Positive means B arrives late.  Both replicas are mean-removed and oriented so a DC
    difference or a branch inversion cannot pull the peak.
    """
    a = orient(ra - ra.mean())
    b = orient(rb - rb.mean())
    denom = float(np.sqrt((a * a).sum() * (b * b).sum()))
    if denom <= 0:
        return float("nan"), 0.0
    c = np.correlate(a, b, mode="full") / denom
    k = int(np.argmax(c))
    if 0 < k < c.size - 1:
        y0, y1, y2 = c[k - 1], c[k], c[k + 1]
        den = y0 - 2 * y1 + y2
        d = 0.5 * (y0 - y2) / den if den else 0.0
    else:
        d = 0.0
    # c[k] compares a against b delayed by (k - (N-1)); a late B peaks below N-1
    return float((a.size - 1) - (k + d)), float(c[k])


def load_state(path: str):
    """Collect frames from a directory, a glob, or a single file.

    ``dither_response_test.py`` writes every state into one directory as
    ``<tag>_frame_<iii>.bin``, so a state is selected with a glob such as
    ``calibration_out/response/raw/code24_frame_*.bin``.  Pointing at the whole directory after
    a multi-state run would mix the states together, so that case warns rather than silently
    averaging them.
    """
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*.bin")))
        tags = {os.path.basename(f).rsplit("_frame_", 1)[0] for f in files}
        if len(tags) > 1:
            print(f"WARNING: {path} holds {len(tags)} different states "
                  f"({', '.join(sorted(tags))}) and they are being averaged together. "
                  f"Select one with a glob, e.g. {path}/<tag>_frame_*.bin")
    elif any(ch in path for ch in "*?["):
        files = sorted(glob.glob(path))
        if not files:
            raise SystemExit(f"{path}: glob matched no files")
    else:
        files = [path]
    frames = []
    for f in files:
        frames.extend(iter_frames(open(f, "rb").read()))
    return files, frames


def analyse(path: str, cfg: DitherConfig, strip_tone: bool):
    files, frames = load_state(path)
    reps_a, reps_b, m = [], [], None
    mags_a, mags_b = [], []
    dc, corr, aligned, margins = [], [], 0, []
    for payload in frames:
        a, b, rot, c = decode(payload)
        if strip_tone:
            a, b = remove_tone(a), remove_tone(b)
        else:
            dc.append(float((b - a).mean()))          # B - A, the loop's convention
        ra, m, n0, sign_a, margin, mag_a = replica_of(a, cfg)
        rb, _, _, _, _, mag_b = replica_of(b, cfg, n0=n0, sign=sign_a)
        if margin <= 3.0:
            continue
        reps_a.append(ra)
        reps_b.append(rb)
        corr.append(c)
        margins.append(margin)
        mags_a.append(mag_a)
        mags_b.append(mag_b)
        aligned += 1
    if not reps_a:
        raise SystemExit(f"{path}: no frame aligned -- check the waveform JSON and the period")

    rep_a = np.mean(reps_a, axis=0)
    rep_b = np.mean(reps_b, axis=0)
    rep_d = np.mean([a - b for a, b in zip(reps_a, reps_b)], axis=0)
    lag, strength = xcorr_lag(rep_a, rep_b)
    dt = slope_delay(rep_d, rep_a, m, cfg)
    fwhm_a, slope_a = width_metrics(rep_a, m)
    fwhm_b, slope_b = width_metrics(rep_b, m)

    per_lag, per_dt, per_dc, per_gain, per_pp = [], [], [], [], []
    for a, b in zip(reps_a, reps_b):
        per_lag.append(xcorr_lag(a, b)[0])
        per_dt.append(slope_delay(a - b, a, m, cfg))
        pa, pb = float(a.max() - a.min()), float(b.max() - b.min())
        per_pp.append(pa)
        if pa:
            per_gain.append(pb / pa)
    if not strip_tone:
        for payload in frames:
            a, b, _, _ = decode(payload)
            per_dc.append(float((b - a).mean()))

    ma = np.asarray(mags_a, dtype=float)
    mb = np.asarray(mags_b, dtype=float)
    ok = np.isfinite(ma) & np.isfinite(mb) & (ma > 0)
    per_gain_mag = (mb[ok] / ma[ok]) if ok.any() else np.asarray([])

    def s(v):
        v = np.asarray(v, dtype=float)
        v = v[np.isfinite(v)]
        return (float(v.mean()), float(v.std(ddof=1)), int(v.size)) if v.size > 1 \
            else (float(v.mean()) if v.size else float("nan"), float("nan"), int(v.size))

    return {
        "path": path, "files": len(files), "frames": len(frames), "aligned": aligned,
        "rep_a": rep_a, "rep_b": rep_b, "rep_d": rep_d, "m": m,
        "pp_a": float(rep_a.max() - rep_a.min()), "pp_b": float(rep_b.max() - rep_b.min()),
        "pk_a": peak_pos(rep_a, m), "pk_b": peak_pos(rep_b, m),
        "lag_ps": lag * PS_PER_SAMPLE, "xcorr": strength, "pp_d": float(rep_d.max() - rep_d.min()),
        "dt_ps": dt * PS_PER_SAMPLE, "margin": s(margins),
        "per_lag_ps": np.array(per_lag) * PS_PER_SAMPLE,
        "per_dt_ps": np.array(per_dt) * PS_PER_SAMPLE,
        "per_lag": s(np.array(per_lag) * PS_PER_SAMPLE),
        "per_dt": s(np.array(per_dt) * PS_PER_SAMPLE),
        "per_dc": s(per_dc), "per_gain": s(per_gain), "per_pp": s(per_pp), "corr": s(corr),
        "mag_a": s(ma), "mag_b": s(mb), "per_gain_mag": s(per_gain_mag),
        "fwhm_a": fwhm_a, "fwhm_b": fwhm_b, "slope_a": slope_a, "slope_b": slope_b,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", action="append", required=True, metavar="NAME=PATH",
                    help="one fixed setting: a directory of *.bin frames, or one file")
    ap.add_argument("--waveform-json", default=DEFAULT_JSON,
                    help="the metadata JSON of the waveform that was loaded (period, polarity)")
    ap.add_argument("--state-json", action="append", default=[], metavar="NAME=JSON",
                    help="per-state waveform JSON, overriding --waveform-json for that state.  "
                         "Needed whenever the states were captured with different waveforms "
                         "(stage B): the fold window and the slope template both come from the "
                         "pulse geometry, so one JSON cannot describe six pulse widths.")
    ap.add_argument("--remove-tone", action="store_true",
                    help="strip a LS sinusoid first (captures that still hold the tone)")
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_out", "dither_raw"))
    args = ap.parse_args(argv)

    if not os.path.exists(args.waveform_json):
        print(f"waveform JSON not found: {args.waveform_json}")
        return 2
    meta = json.loads(open(args.waveform_json, encoding="utf-8").read())
    cfg = DitherConfig(**meta["config"])
    cfg.validate()
    geo = f"{cfg.dither_edge_dac}/{cfg.dither_top_dac} DAC = {2 * cfg.edge_r + cfg.top_w:.0f} ADC"
    print(f"waveform: {os.path.basename(args.waveform_json)}  pulse {geo} samples, "
          f"period {cfg.slot_period} ADC samples, {cfg.n_events} events/loop, "
          f"amplitude {cfg.dither_scale_lsb:.0f} LSB, tone {meta['derived']['main_tone_hz'] / 1e6:.3f} MHz")
    print(f"tone: {'REMOVED by a LS fit -- NOT raw, and DC goes with it' if args.remove_tone else 'not fitted; whatever is in the capture is what is measured'}")

    states = []
    cfgs = {}
    for spec in args.state_json:
        if "=" not in spec:
            print(f"--state-json needs NAME=JSON, got {spec!r}")
            return 2
        sname, jpath = spec.split("=", 1)
        if not os.path.exists(jpath):
            print(f"--state-json {sname}: {jpath} does not exist")
            return 2
        cfgs[sname] = DitherConfig(**json.loads(open(jpath, encoding="utf-8").read())["config"])

    for spec in args.state:
        if "=" not in spec:
            print(f"--state needs NAME=PATH, got {spec!r}")
            return 2
        name, path = spec.split("=", 1)
        if not os.path.exists(path) and not any(ch in path for ch in "*?["):
            print(f"{name}: {path} does not exist")
            return 2
        cfg_i = cfgs.get(name, cfg)
        if cfg_i is not cfg:
            print(f"{name}: using its own waveform JSON -- pulse "
                  f"{2 * cfg_i.edge_r + cfg_i.top_w:.0f} ADC samples")
        states.append((name, analyse(path, cfg_i, args.remove_tone)))

    print(f"\n{'state':>14}{'frames':>7}{'aligned':>8}{'rep A':>8}{'rep B':>8}{'B/A':>8}"
          f"{'B/A mag':>9}{'FWHM A':>8}{'slope A':>9}{'dt ps':>9}{'dt sd':>9}{'lag ps':>9}{'|A-B| rep':>11}")
    for name, r in states:
        gm = r["per_gain_mag"][0]
        print(f"{name:>14}{r['frames']:>7}{r['aligned']:>8}{r['pp_a']:>8.2f}{r['pp_b']:>8.2f}"
              f"{r['pp_b'] / r['pp_a'] if r['pp_a'] else float('nan'):>8.4f}"
              f"{gm:>9.4f}{r['fwhm_a']:>8.1f}{r['slope_a']:>9.2f}"
              f"{r['dt_ps']:>9.1f}{r['per_dt'][1]:>9.1f}{r['lag_ps']:>9.1f}{r['pp_d']:>11.2f}")
    print("\nFWHM A = full width at half maximum of the folded replica, in ADC samples:")
    print("  this is what answers 'the dither should be shorter' -- if the FWHM stops shrinking")
    print("  as the waveform's pulse gets shorter, the analog path is the limit.")
    print("slope A = steepest slope of the replica, codes/sample -- the slope route's gain.")
    print("\nB/A      = folded replica ratio (polarity-corrected, shares the session sign).")
    print("B/A mag  = **sign-free**: mean peak-to-peak of the raw impulse windows.  No polarity")
    print("  convention enters, so this is the gain route to quote.  Measured 2026-09-19 the two")
    print("  agreed to 0.02 % while the estimator's gain_ratio sat 3 % away.")
    print("dt ps = timing from the folded A-B difference projected on the known pulse slope")
    print("  (the only sub-sample route; positive = B late).  dt sd = per-frame scatter.")
    print("lag ps = cross-correlation shift of the replicas -- coarse: the 769 ps sampling grid")
    print("  quantises it, so treat it as a sanity check, not a measurement.")
    print("rep = folded impulse replica in codes pk-pk; |A-B| rep = difference replica pk-pk.")

    print("\nper-frame, folding each frame on its own (this is what sets how many frames a "
          "real change needs):")
    for name, r in states:
        dt_m, dt_sd, dt_n = r["per_dt"]
        g_m, g_sd, _ = r["per_gain"]
        gm_m, gm_sd, _ = r["per_gain_mag"]
        pp_m, pp_sd, _ = r["per_pp"]
        dc_m, dc_sd, _ = r["per_dc"]
        mi_m, _, _ = r["margin"]
        print(f"  {name:>14}: dt {dt_m:+8.1f} +-{dt_sd:8.1f} ps (n={dt_n:3d})"
              f"   B/A {g_m:.4f} +-{g_sd:.4f}   B/A mag {gm_m:.4f} +-{gm_sd:.4f}"
              f"   repA {pp_m:6.2f} +-{pp_sd:5.2f} codes"
              f"   align margin {mi_m:.1f}"
              + (f"   DC(B-A) {dc_m:+6.2f} +-{dc_sd:5.2f} codes" if dc_m == dc_m else
                 "   DC(B-A) n/a under --remove-tone"))
    print("  (one state's precision is sigma/sqrt(N); compare it with the change between states)")

    if len(states) > 1:
        bn, br = states[0]
        print(f"\nchange against {bn}:")
        for name, r in states[1:]:
            print(f"  {name:>14}: d(dt) {r['dt_ps'] - br['dt_ps']:+8.1f} ps"
                  f"   d(B/A) {r['per_gain'][0] - br['per_gain'][0]:+.4f}"
                  f"   d(|A-B| rep) {r['pp_d'] - br['pp_d']:+.2f} codes"
                  + (f"   d(DC) {r['per_dc'][0] - br['per_dc'][0]:+.2f} codes"
                     if r["per_dc"][0] == r["per_dc"][0] and br["per_dc"][0] == br["per_dc"][0]
                     else ""))

    os.makedirs(args.out, exist_ok=True)
    csv = os.path.join(args.out, "dither_raw_states.csv")
    with open(csv, "w", encoding="utf-8", newline="") as fh:
        fh.write("state,frames,aligned,rep_a_pp,rep_b_pp,gain_mean,gain_sd,"
                 "dt_ps,dt_ps_sd,lag_ps,lag_ps_sd,xcorr,rep_ab_pp,dc_codes,dc_sd\n")
        for name, r in states:
            fh.write(f"{name},{r['frames']},{r['aligned']},{r['pp_a']:.4f},{r['pp_b']:.4f},"
                     f"{r['per_gain'][0]:.6f},{r['per_gain'][1]:.6f},{r['dt_ps']:.2f},"
                     f"{r['per_dt'][1]:.2f},{r['lag_ps']:.2f},{r['per_lag'][1]:.2f},"
                     f"{r['xcorr']:.4f},{r['pp_d']:.4f},"
                     f"{r['per_dc'][0]:.4f},{r['per_dc'][1]:.4f}\n")
    print(f"\nstates : {csv}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                                  # pragma: no cover
        print(f"(no figure: {exc})")
        return 0

    ncol = 2 + (1 if len(states) > 1 else 0)
    fig, ax = plt.subplots(1, ncol, figsize=(5.6 * ncol, 4.4))
    ax = np.atleast_1d(ax)
    for a, (name, r) in zip(ax, states):
        a.plot(r["m"], r["rep_a"], "-o", ms=3, lw=1.2, label="A")
        a.plot(r["m"], r["rep_b"], "-s", ms=3, lw=1.2, alpha=0.75, label="B (sign-normalised)")
        a.plot(r["m"], r["rep_d"], "-^", ms=3, lw=1.0, alpha=0.8, label="A - B")
        a.set_title(f"{name}: folded impulse replica\nrep {r['pp_a']:.1f}/{r['pp_b']:.1f} codes, "
                    f"dt {r['dt_ps']:+.1f} ps, |A-B| {r['pp_d']:.1f}", fontsize=10)
        a.set_xlabel("ADC samples from the nominal pulse start")
        a.set_ylabel("codes")
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    if len(states) > 1:
        for name, r in states:
            ax[-1].hist(r["per_dt_ps"], bins=30, alpha=0.55,
                        label=f"{name}: {r['per_dt'][1]:.0f} ps")
        ax[-1].set_title("per-frame timing (slope route)\n(width = what one frame buys)",
                         fontsize=10)
        ax[-1].set_xlabel("ps")
        ax[-1].legend(fontsize=8)
        ax[-1].grid(True, alpha=0.3)
    fig.tight_layout()
    png = os.path.join(args.out, "dither_raw_evidence.png")
    fig.savefig(png, dpi=140)
    print(f"figure : {png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
