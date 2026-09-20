"""Plot the DAC code and the captured ADC samples on one time axis, with sample markers.

The two traces are aligned by recovering the capture's position inside the DPG loop: the
balanced pseudo-random polarity of the impulse train acts as a sync word, so ``align_to_loop``
returns ``n0`` such that capture sample ``n`` is loop position ``(n + n0) mod loop``.  That is
the same alignment the estimators use, so the DAC trace is not a visual overlay -- it is where
the DAC was when each sample was taken.

Left axis: ADC codes (14-bit, left-aligned in the 16-bit words).  Right axis: DAC code
(signed 16-bit, full scale 32767).  The two are plotted on separate axes because the path
attenuates the impulse by a large factor; the measured factor is printed and drawn as a dashed
"what the DAC asked for, scaled to ADC codes" trace.

    python tools/dac_vs_adc_plot.py \
        --frame calibration_out/response/B_w06adc_amp16000/raw/code24_frame_000.bin \
        --waveform-json waveforms/pulse_ladder/w06adc_e04_t04_amp16000.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "tools"))

from calibration_loop.dither import DitherConfig, build_dac_waveform   # noqa: E402
from calibration_loop.estimator import align_to_loop, visible_events  # noqa: E402
from dither_raw_evidence import decode, replica_of                    # noqa: E402

FS_ADC = 1.300e9
PS_PER_SAMPLE = 1e12 / FS_ADC


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frame", required=True, help="one 4095-byte capture payload")
    ap.add_argument("--waveform-json", required=True)
    ap.add_argument("--zoom", type=int, default=300, help="width of the zoom panel, in samples")
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_out", "dac_vs_adc"))
    args = ap.parse_args(argv)

    meta = json.loads(open(args.waveform_json, encoding="utf-8").read())
    cfg = DitherConfig(**meta["config"])
    cfg.validate()

    raw = open(args.frame, "rb").read()[:4095]
    a, b, rot, corr = decode(raw)

    # DAC vector on the DAC grid -> the value the ADC grid should see (decimate by adc_ratio)
    dac_full, signs = build_dac_waveform(cfg)
    dac_adc = dac_full[::cfg.adc_ratio].astype(float)          # 8320 values, one per ADC sample

    align = align_to_loop(a, cfg)
    n0 = align["n0"]
    loop = dac_adc.size
    idx = (np.arange(a.size) + n0) % loop                       # loop position of each sample
    dac_trace = dac_adc[idx]

    # measured path gain and polarity: replica height at the ADC per DAC LSB of impulse
    rep_a, m, _, _, margin, _ = replica_of(a, cfg)
    pulse_height = float(rep_a.max() - np.median(np.concatenate([rep_a[:3], rep_a[-3:]])))
    dac_amp = float(cfg.a_dither * 32767.0)
    gain_codes_per_lsb = pulse_height / dac_amp if dac_amp else float("nan")

    # polarity of the path: compare the ADC excursion at an impulse with the DAC drive there
    dac_adc_aligned = dac_adc[(np.arange(a.size) + n0) % loop]
    kpk = int(np.argmax(np.abs(dac_adc_aligned)))
    polarity = float(np.sign(a[kpk] - a.mean())) * float(np.sign(dac_adc_aligned[kpk])) or 1.0
    inverted = polarity < 0

    fs_dbfs = 20 * np.log10(dac_amp / 32767.0)
    print(f"frame     : {os.path.basename(args.frame)}  (rot {rot}, |corr| {corr:.4f}, "
          f"align margin {margin:.1f})")
    print(f"impulse   : {dac_amp:.0f} DAC LSB peak = {fs_dbfs:+.1f} dBFS "
          f"({100 * dac_amp / 32767:.1f} % of DAC full scale)")
    print(f"at the ADC: {pulse_height:.0f} codes peak (channel A), "
          f"{100 * pulse_height / 8192:.1f} % of ADC full scale (14-bit, +/-8192 codes)")
    print(f"path gain : {gain_codes_per_lsb:.5f} ADC codes per DAC LSB "
          f"({1 / gain_codes_per_lsb:.0f} : 1 attenuation)")
    print(f"path sign : {'INVERTED' if inverted else 'non-inverting'} "
          f"(DAC drive {dac_adc_aligned[kpk]:+.0f} LSB -> ADC {a[kpk] - a.mean():+.0f} codes)")
    print(f"dither    : one impulse every {cfg.slot_period} ADC samples "
          f"({cfg.slot_period * PS_PER_SAMPLE / 1000:.0f} ns), "
          f"{cfg.pulse_len:.0f} samples wide as generated")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t_ns = np.arange(a.size) * PS_PER_SAMPLE / 1000.0
    fig, ax = plt.subplots(2, 1, figsize=(13, 8.5))

    # ---- panel 1: whole capture ------------------------------------------
    ax0 = ax[0]
    ax0.plot(t_ns, a, "-", color="#1f4e79", lw=0.9, label="ADC channel A")
    ax0.plot(t_ns, b, "-", color="#a0522d", lw=0.8, alpha=0.75, label="ADC channel B")
    ax0.plot(t_ns, a, ".", color="#1f4e79", ms=2.6)            # sample markers
    ax1 = ax0.twinx()
    ax1.plot(t_ns, dac_trace, "-", color="#8e44ad", lw=1.0, alpha=0.85,
             label="DAC code (right axis)")
    ax0.set_xlabel("time (ns)")
    ax0.set_ylabel("ADC codes")
    ax1.set_ylabel("DAC code", color="#8e44ad")
    ax0.set_title(f"DAC excitation and ADC response, same time axis "
                  f"({a.size} samples = {t_ns[-1]:.0f} ns, one impulse every 100 ns)", fontsize=10)
    h0, l0 = ax0.get_legend_handles_labels()
    h1, l1 = ax1.get_legend_handles_labels()
    ax0.legend(h0 + h1, l0 + l1, fontsize=8, loc="upper right")
    ax0.grid(True, alpha=0.3)

    # ---- panel 2: zoom on two impulses -----------------------------------
    k0 = int(np.argmax(dac_trace > dac_amp * 0.5))
    lo = max(0, k0 - 40)
    hi = min(a.size, lo + args.zoom)
    z = slice(lo, hi)
    ax2 = ax[1]
    ax2.plot(t_ns[z], a[z], "-", color="#1f4e79", lw=1.0, label="ADC channel A")
    ax2.plot(t_ns[z], b[z], "-", color="#a0522d", lw=1.0, alpha=0.8, label="ADC channel B")
    ax2.plot(t_ns[z], a[z], "o", color="#1f4e79", ms=5, mfc="none", mew=1.0,
             label="ADC A samples")
    ax2.plot(t_ns[z], b[z], "s", color="#a0522d", ms=4, mfc="none", mew=0.9,
             label="ADC B samples")
    ax2.plot(t_ns[z], polarity * dac_trace[z] * gain_codes_per_lsb, "--", color="#8e44ad",
             lw=1.2, label="DAC request, scaled by the measured path gain and polarity "
                           f"({gain_codes_per_lsb:.4f} codes/LSB, "
                           f"{'inverted' if inverted else 'non-inverting'})")
    ax3 = ax2.twinx()
    ax3.plot(t_ns[z], dac_trace[z], "-", color="#8e44ad", lw=1.6, alpha=0.55,
             label="DAC code (right axis)")
    ax2.set_xlabel("time (ns)")
    ax2.set_ylabel("ADC codes")
    ax3.set_ylabel("DAC code", color="#8e44ad")
    ax2.set_title("Zoom: the impulse as sent (DAC) and as seen (ADC), with every sample marked",
                  fontsize=10)
    h2, l2 = ax2.get_legend_handles_labels()
    h3, l3 = ax3.get_legend_handles_labels()
    ax2.legend(h2 + h3, l2 + l3, fontsize=8, loc="upper right")
    ax2.grid(True, alpha=0.3)

    for x in ax:
        x.tick_params(labelsize=8)
    fig.tight_layout()
    os.makedirs(args.out, exist_ok=True)
    png = os.path.join(args.out, "dac_vs_adc.png")
    fig.savefig(png, dpi=140)
    plt.close(fig)
    print(f"figure    : {png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
