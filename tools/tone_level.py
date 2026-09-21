"""Measure the tone level the ADC actually sees, in codes and in volts, for whatever DPG vector
is loaded.  This is the ADC half of a "signal strength from either end" check; the DAC half is the
vector's own digital amplitude converted with the DAC's *measured* volts per LSB.

It needs **no** estimator and **no** dither: the capture is de-framed with the channel-correlation
rule and the tone is read by a coherent DFT at the frequency you say is loaded.  A pure-tone vector
therefore works, which is what a level/response measurement wants.

    python tools/tone_level.py --uart COM5 --frames 20 --freq-mhz 199.375 \
        --dac-lsb 16385 --fs-dac-v-peak 0.215

Procedure for the whole exercise, all at **50 ohm** (never high-Z: it reads ~2x and unloads the
DAC's matching network):

  1. DAC full scale.  Load ``waveforms/scale_check/fs_5MHz_amp32729.txt`` (a 5 MHz sine at exactly
     32729 LSB) on the DPG and read its amplitude on the scope at the DAC output, 50 ohm input.
     ``FS_peak = V_measured/2 * 32768/32729``, i.e. 1 LSB = FS_peak/32768.  Check the reading with a
     known source through the same cable and scope settings, then repeat with
     ``fs_5MHz_amp16384.txt`` (16385 LSB): the voltage must halve.
  2. Chain response.  For each ``waveforms/scale_check/sweep_*.txt`` (same amplitude, different
     frequency) load it, run this tool, and keep the tone codes.  codes -> volts with the ADC
     full scale below; volts/codes against the DAC side gives the path gain at that frequency.
  3. ADC full scale.  Drive the ADC input from a signal generator (50 ohm) at a known amplitude and
     run this tool with ``--dac-lsb 0``; compare the measured codes with
     ``codes_pp = V_pp_diff / 1.59 * 16384``.  This is the only way to reach ADC full scale here --
     the DAC plus link arrives ~10 dB low (measured 2026-09-20: a -1.5 dBFS tone lands at 555
     codes, i.e. -23 dBFS, at 199.375 MHz).

Conversions used: ADC 1.59 Vpp differential over 16384 codes = 97.05 uV per code, differential
(IFC register 0x1910 = 0x0C -- datasheet, to be confirmed by step 3); DAC 2**15 LSB either side of
zero.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.estimator import deframe, unpack_words  # noqa: E402

ADC_FS_VPP_DIFF = 1.59
ADC_CODES = 2 ** 14
DAC_LSB = 2 ** 15
FS_ADC = 1.3e9


def tone_amplitude(x: np.ndarray, freq_hz: float, fs: float = FS_ADC) -> float:
    """Coherent amplitude of a sine at ``freq_hz``, in the units of ``x`` (codes here)."""
    n = x.size
    w = np.hanning(n)
    ph = np.exp(-2j * np.pi * (freq_hz / fs) * np.arange(n))
    return float(2.0 * abs(np.sum(np.asarray(x, float) * w * ph)) / w.sum())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uart", required=True, help="e.g. COM5")
    ap.add_argument("--bind-ip", dest="bind_ip", default="0.0.0.0")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--freq-mhz", dest="freq_mhz", type=float, required=True,
                    help="the tone frequency that is loaded on the DPG (or driven by the generator)")
    ap.add_argument("--dac-lsb", dest="dac_lsb", type=float, default=0.0,
                    help="tone amplitude in DAC LSB; 0 skips the DAC-side comparison")
    ap.add_argument("--fs-dac-v-peak", dest="fs_dac_v_peak", type=float, default=0.215,
                    help="DAC volts per LSB source: measured full scale, V peak (0.215 = the "
                         "430 mVpp read at 5 MHz into 50 ohm)")
    ap.add_argument("--adc-fs-vpp", dest="adc_fs_vpp", type=float, default=ADC_FS_VPP_DIFF)
    ap.add_argument("--out", default=None, help="write the per-frame rows to this CSV")
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench

    u_per_code = args.adc_fs_vpp / ADC_CODES
    u_per_lsb = args.fs_dac_v_peak / DAC_LSB
    f0 = args.freq_mhz * 1e6

    print(f"ADC scale: {args.adc_fs_vpp} Vpp differential / {ADC_CODES} codes "
          f"= {u_per_code * 1e6:.2f} uV per code")
    if args.dac_lsb:
        print(f"DAC side : {args.dac_lsb:.0f} LSB = {args.dac_lsb * u_per_lsb:7.4f} V peak "
              f"(at {args.fs_dac_v_peak} V peak full scale)")
    print(f"tone     : {args.freq_mhz:.4f} MHz, {args.frames} frames\n")
    print(f"  {'#':>3} {'rot':>4} {'|corr|':>7} {'A codes':>9} {'B codes':>9} {'A pk-pk':>9} "
          f"{'A mV pk':>9}")
    print("  " + "-" * 60)

    bench = HardwareBench(uart_port=args.uart, bind_ip=args.bind_ip)
    rows = []
    try:
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                print(f"  {i:>3}  capture failed (no UDP frame)")
                continue
            try:
                d = deframe(unpack_words(raw))
            except ValueError as exc:
                print(f"  {i:>3}  {exc}")
                continue
            a = np.asarray(d["ch_a"], float)
            b = np.asarray(d["ch_b"], float)
            amp_a, amp_b = tone_amplitude(a, f0), tone_amplitude(b, f0)
            print(f"  {i:>3} {d['rotation']:>4} {abs(d['corr']):>7.3f} {amp_a:>9.1f} {amp_b:>9.1f} "
                  f"{a.max() - a.min():>9.0f} {amp_a * u_per_code * 1e3:>9.2f}")
            rows.append({"frame": i, "rotation": d["rotation"], "abs_corr": abs(d["corr"]),
                         "tone_a_codes": amp_a, "tone_b_codes": amp_b,
                         "a_pk_pk_codes": float(a.max() - a.min()),
                         "tone_a_mv_peak": amp_a * u_per_code * 1e3,
                         "freq_mhz": args.freq_mhz})
    finally:
        bench.close()

    if not rows:
        print("\nNo usable frames -- check the link (raw_frame_probe.py) and the DPG content.")
        return 1

    ta = np.array([r["tone_a_codes"] for r in rows])
    tb = np.array([r["tone_b_codes"] for r in rows])
    print(f"\n  tone A: {ta.mean():8.1f} +- {ta.std():5.1f} codes = "
          f"{ta.mean() * u_per_code * 1e3:7.1f} mV peak differential "
          f"({ta.mean() / (ADC_CODES / 2):6.2%} of ADC peak)")
    print(f"  tone B: {tb.mean():8.1f} +- {tb.std():5.1f} codes = "
          f"{tb.mean() * u_per_code * 1e3:7.1f} mV peak differential")
    if args.dac_lsb:
        v_adc = ta.mean() * u_per_code
        v_dac = args.dac_lsb * u_per_lsb
        print(f"  DAC -> ADC at {args.freq_mhz:.4f} MHz: {v_dac:.4f} V peak in, "
              f"{v_adc * 1e3:.1f} mV peak out => {v_adc / v_dac:.3%} "
              f"({20 * np.log10(v_adc / v_dac):+.1f} dB)")

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"  csv: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
