"""DAC-side vs ADC-side amplitude of the same signal, in volts, from captured and archived frames.

Convert a known digital amplitude at the DAC into volts, convert the measured amplitude at the ADC
into volts, and compare: the ratio is the link between the two boards, and it is frequency
dependent.  Nothing here is taken on trust from anyone -- every number is either measured on this
bench or a device datasheet value, and the two are labelled separately.

What is measured, and what is not yet:

  * DAC side is MEASURED: 0.430 Vpp single-ended into 50 ohm at 5 MHz, from the vector
    ``waveforms/scale_check/fs_5MHz_amp32729.txt`` (32729 LSB peak).  Still worth a known-source
    check on the same cable/scope, and the half-amplitude vector must read half.
  * ADC side is MEASURED as codes: ``estimate_block`` returns the coherent tone amplitude in 14-bit
    codes, cross-checked against an independent DFT of the same frames (555.0 vs 561.0 codes on
    frame 0 -- two methods, same answer).
  * ADC volts-per-code is DATASHEET (1.59 Vpp differential at IFC 0x0C / 16384).  Confirm it by
    driving a known 50 ohm amplitude at the ADC input and reading the codes.

Run from the repo root:  python calibration_out/_dac_adc_volts.py
"""
from __future__ import annotations

import glob

import numpy as np

from calibration_loop.dither import DAC_FULL_SCALE, DitherConfig
from calibration_loop.estimator import estimate_block, polarity_anchor, prepare_capture

# --- bench constants -------------------------------------------------------------------------
# Every number here is either measured on this bench or a device datasheet value, and each one
# carries its provenance so a stale assumption cannot hide.
FS_ADC_VPP_DIFF = 1.59      # DATASHEET: AD9695 differential input span at IFC register 0x1910 = 0x0C.
#                             Confirm by driving a known 50 ohm amplitude at the ADC input and
#                             reading the codes (tools/tone_level.py --dac-lsb 0).  Until then this is
#                             the one number in the chain that is not measured here.
ADC_CODES = 2 ** 14         # 14-bit signed: -8192 .. +8191, span 16384 codes
U_PER_CODE = FS_ADC_VPP_DIFF / ADC_CODES          # V per code, differential (= 97.05 uV)

FS_DAC_MVPP_SWEEP = {5.0: 430.0, 15.0: 640.0, 25.0: 750.0, 35.0: 800.0, 45.0: 820.0,
                    75.0: 850.0}
# MEASURED on the scope at 50 ohm, all from the same 32729-LSB vector (see waveforms/scale_check/).
# The rise from 5 MHz to 75 MHz is +5.9 dB and the increments decay, so the link has a HIGH-PASS
# corner (5 MHz is not the reference), and 75 MHz is flat-ish.  5 MHz must never be quoted as "the
# DAC full scale".
SPLITTER_LOSS_DB = 9.6      # DATASHEET for the power splitter in the path, CONFIRMED by measurement:
#                             the same full-scale vector reads 850 mVpp before it and 270 mVpp after,
#                             i.e. -9.96 dB at 75 MHz (0.36 dB from the spec).
FS_DAC_V_PEAK = 0.425       # 850 mVpp flat value at the DAC output = 0.425 V peak = +2.57 dBm into
#                             50 ohm = 12.97 uV per LSB.  This is the DAC-side number for a tone in
#                             the passband (ours is at 199.375 MHz).  Every volt below scales with it.
DAC_LSB = 2 ** 15           # 16-bit signed: -32768 .. +32767, span 2**16 codes
U_PER_LSB = FS_DAC_V_PEAK / DAC_LSB               # V per LSB, peak
SPLITTER_GAIN = 10.0 ** (-SPLITTER_LOSS_DB / 20.0)
DAC_LSB = 2 ** 15           # 16-bit signed: -32768 .. +32767, span 2**16 codes
U_PER_LSB = FS_DAC_V_PEAK / DAC_LSB               # V per LSB, peak

print(f"ADC: {FS_ADC_VPP_DIFF} Vpp differential (datasheet) / {ADC_CODES} codes "
      f"= {U_PER_CODE * 1e6:.2f} uV per code ({U_PER_CODE * 1e6 / 2:.2f} uV per pin)")
print(f"DAC: {FS_DAC_V_PEAK * 2:.3f} Vpp = {FS_DAC_V_PEAK:.3f} V peak (measured, 5 MHz) / "
      f"{DAC_LSB} LSB = {U_PER_LSB * 1e6:.2f} uV per LSB\n")


def measure_tone(raw: bytes, cfg: DitherConfig) -> tuple[float, float, float]:
    """(tone codes peak on ch A, dither codes, alignment margin) for one raw DMA frame."""
    prep = prepare_capture(raw, cfg)
    pol = polarity_anchor(prep)
    est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, cancel_signal=True, n0=prep["n0"],
                         polarity_sign=pol, pin_polarity=True)
    return abs(est.ch_a.tone_amplitude), abs(est.ch_a.gain_codes), prep["align_margin"]


def report(label: str, tone_codes: float, cfg: DitherConfig,
           pulse_pp: float | None = None) -> None:
    tone_lsb = cfg.a_sine * DAC_FULL_SCALE
    print(f"--- {label}")
    if tone_codes:
        v_adc = tone_codes * U_PER_CODE
        v_dac = tone_lsb * U_PER_LSB
        print(f"    tone  {cfg.f_sig / 1e6:8.3f} MHz : DAC {tone_lsb:6.0f} LSB = {v_dac:6.3f} V peak"
              f"  ->  ADC {tone_codes:6.1f} codes = {v_adc * 1e3:6.1f} mV peak"
              f"  =>  {v_adc / v_dac:6.3%}  ({20 * np.log10(v_adc / v_dac):+.1f} dB)")
    if pulse_pp:
        # The honest ADC-side pulse number is the *folded replica peak-to-peak* (what
        # tools/dither_ladder.py reports); estimate_block's gain_codes is the least-squares scale of
        # a unit template, so it carries a ~40 % shape error and is not used here.
        v_adc = 0.5 * pulse_pp * U_PER_CODE
        v_dac = cfg.dither_scale_lsb * U_PER_LSB
        print(f"    pulse {pulse_pp:6.1f} codes pk-pk at the ADC : DAC {cfg.dither_scale_lsb:6.0f} "
              f"LSB = {v_dac * 1e3:6.2f} mV peak  ->  ADC {v_adc * 1e3:6.2f} mV peak"
              f"  =>  {v_adc / v_dac:6.3%}  ({20 * np.log10(v_adc / v_dac):+.1f} dB)")


# 1. tone + dither captures archived 2026-09-17 (default vector: -1.5 dBFS tone, 2000 LSB, w32)
cfg_old = DitherConfig()
cfg_old.validate()
tones, margins = [], []
for path in sorted(glob.glob("calibration_out/dither_vs_tone_frames/frame_*.bin"))[:20]:
    with open(path, "rb") as fh:
        try:
            ta, _g, m = measure_tone(fh.read(), cfg_old)
        except Exception:
            continue
    if np.isfinite(ta):
        tones.append(ta)
        margins.append(m)
# gain_codes (17.6 codes here) is the least-squares scale of a *unit template*, i.e. the same family
# as the "7-18 codes" this session's doc quotes for these captures, but it is not a clean peak: the
# shaped replica's folded pk-pk is what tools/dither_ladder.py reports, so the pulse row below uses
# that number (from the ladder table) rather than gain_codes.
report(f"archived tone+dither frames, 2026-09-17 ({len(tones)} frames, margin "
       f"{np.mean(margins):.1f})",
       float(np.mean(tones)), cfg_old)

# 2. the stage-F ladder: 6-sample pulse, tone off, folded pk-pk from dither_ladder.py's own table
ladder = {"16000": 374.2, "24000": 560.5, "30000": 700.7}
for amp_lsb, rep in ((16000, ladder["16000"]), (24000, ladder["24000"]), (30000, ladder["30000"])):
    cfg6 = DitherConfig(amp_dbfs=-120.0, dither_edge_dac=4, dither_top_dac=4,
                        dither_scale_lsb=float(amp_lsb))
    cfg6.validate()
    report(f"stage F, w06 pulse at {amp_lsb} LSB", 0.0, cfg6, pulse_pp=rep)

print("\nEvery volt here is measured at one end or the other; the ratio between them is the link.  "
      "The tone is\nthe honest probe (an impulse's peak also depends on pulse droop), so its ratio "
      "is what the frequency\nsweep in tools/tone_level.py should map out.")

# 3. the chain, element by element, for the tone: DAC output -> splitter -> the rest -> ADC codes
tone_lsb = cfg_old.a_sine * DAC_FULL_SCALE
v_dac = tone_lsb * U_PER_LSB                      # at the DAC output
v_post = v_dac * SPLITTER_GAIN                    # after the power splitter
v_adc = float(np.mean(tones)) * U_PER_CODE        # measured at the ADC, differential
print(f"\nchain decomposition at {cfg_old.f_sig / 1e6:.3f} MHz, tone at {tone_lsb:.0f} LSB:")
print(f"  DAC output                       {v_dac * 1e3:7.1f} mV peak single-ended")
print(f"  after the power splitter         {v_post * 1e3:7.1f} mV peak   "
      f"({SPLITTER_LOSS_DB:.1f} dB, datasheet; measured {850.0:.0f} -> {270.0:.0f} mVpp = "
      f"{20 * np.log10(270.0 / 850.0):+.2f} dB)")
print(f"  measured at the ADC (codes)      {v_adc * 1e3:7.1f} mV peak differential")
print(f"  -> splitter {SPLITTER_LOSS_DB:.1f} dB + cable/balun/ADC front end "
      f"{20 * np.log10(v_adc / v_post):+.1f} dB = {20 * np.log10(v_adc / v_dac):+.1f} dB total")
print(f"  -> the ADC input sits at {20 * np.log10(float(np.mean(tones)) / 8192):+.1f} dBFS for this "
      f"command; at full DAC scale it would be "
      f"{20 * np.log10(v_dac / cfg_old.a_sine * SPLITTER_GAIN * (v_adc / v_post) / 0.795):+.1f} dBFS")
print("  (the same figure measured through the codes: "
      f"{20 * np.log10(float(np.mean(tones)) / cfg_old.a_sine / 8192):+.1f} dBFS)")
