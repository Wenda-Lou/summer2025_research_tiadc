# Dither-only calibration vectors

Use wave 3 for the full-scale DAC peak requested for the limit tests.
Load the TXT into the DPG; JSON is metadata, not the input waveform.

| Firmware selection | TXT | DAC peak |
|---|---|---|
| 3 (default) | `w06adc_e04_t04_amp32767_dither_only.txt` | +/-32767 codes, 100% |
| 2 (comparison) | `w06adc_e04_t04_amp5190_dither_only.txt` | +/-5190 codes, 15.84% |

Both vectors use DAC 2.6 GS/s, ADC 1.3 GS/s, 16640 DAC samples,
64 impulses (32 positive / 32 negative), 260 DAC samples between events,
rise/top/fall 4/4/4 DAC samples, and seed 20260725. The sine frequency and
amplitude are exactly zero. The calibration equations match the dither branch
of commit `d22d23ff6753b4503c54157529ad8396483cb006`.

Full scale here describes DAC peak amplitude, not RMS amplitude or ADC input
voltage. The full-scale TXT has mean zero and 320 samples at the legal DAC
rails. Metadata distinguishes `touches_dac_rails=true` from `clipping=false`
(no integer overflow/truncation). Actual ADC rail samples still reject a frame.
The PNG and verification JSON adjacent to each TXT show its samples and checks.

Regenerate the full-scale vector from the repository root:

```powershell
python -m calibration_loop.run_calibration gen --out waveforms/dither_only --stem w06adc_e04_t04_amp32767_dither_only --amp-dbfs -120 --sig-cycles 0 --dither-edge 4 --dither-top 4 --dither-scale 32767
```

Rebuild/download the new `thesis_v3_loop_appl` ELF before selecting wave 3.
The board defaults to 3; `test status` must show
`gain=dither_mag skew=dither_phase tone_free=1`. Tone 0/1 remain disabled.
Alignment, offset, gain, timing and frame gates use dither only, with no
carrier-fit or purity calls.

Full-scale software preflight: 60/60 qualified, zero rejected, no ADC rail
samples, using an explicit model analog scale of 0.20 ADC codes/DAC code.
Last-12-frame means: magnitude ratio 0.999965, offset -0.0084 codes,
skew -0.0138 ps. Outputs: `calibration_sim/output/fullscale_dither_preflight/`.
This model scale is a test assumption, not a measurement of the current bench.
The 32767-code waveform is not bench-validated; prior recorded amplitude ladders
reached 30000 codes. No UART connection, register write or DPG download was
performed to prepare these files.

Full procedure: [`CALIBRATION_LIMIT_TEST.md`](../../calibration_loop/CALIBRATION_LIMIT_TEST.md).
