# Dither power ladder

Question: what is the smallest dither that still calibrates offset, gain and
skew together reliably?

Six dither-only DPG vectors, identical to the bench vector
`../dither_only/w06adc_e04_t04_amp32767_dither_only_p130.txt` except the DAC
peak: same seed and polarity, 4/4/4 pulse, 130-DAC (65 ADC sample) spacing,
128 events per loop, no tone. Each is that vector scaled exactly; the 32767 file
equals it sample for sample. (Wave 3 without `_p130` has 260-DAC spacing and
half the events; it is not the bench vector and must not be mixed into this
ladder.)

| TXT | DAC peak | Relative |
|---|---|---|
| `w06adc_e04_t04_amp32767_dither_only_p130.txt` | 32767 | 0 dB (bench vector) |
| `w06adc_e04_t04_amp16384_dither_only_p130.txt` | 16384 | -6 dB |
| `w06adc_e04_t04_amp8192_dither_only_p130.txt` | 8192 | -12 dB |
| `w06adc_e04_t04_amp4096_dither_only_p130.txt` | 4096 | -18 dB |
| `w06adc_e04_t04_amp2048_dither_only_p130.txt` | 2048 | -24 dB |
| `w06adc_e04_t04_amp1024_dither_only_p130.txt` | 1024 | -30 dB |

Regenerate (repository root), changing only `--dither-scale` and `--stem`:

```powershell
python -m calibration_loop.run_calibration gen --out waveforms/dither_power --stem w06adc_e04_t04_amp8192_dither_only_p130 --amp-dbfs -120 --sig-cycles 0 --dither-period 130 --dither-edge 4 --dither-top 4 --dither-scale 8192
```

Calibration is reference-free, so the firmware needs no new wave number: load
the TXT into the DPG and keep the RF-combiner tone path as for the joint runs.
The DPG does not reject a wrong vector; note which TXT is playing for every run.

## Prediction (simulation, 2026-11-25)

`python checks/dither_power_sim.py` runs the joint 9-corner matrix per level
(65-sample spacing) and grades HOLD with the bench rule. Model ADC noise 1 code
RMS, tone -6 dBFS: all corners pass down to a dither peak of ~300 ADC codes
(300x noise RMS, worst gain bound 0.78x tolerance); gain fails first (6/9 at
150 codes, 0/9 at 75), offset and skew hold to ~37-75 codes. The model has no
analog dispersion and never rejects these frames with `phase-spread`; the bench
can fail earlier at acquisition (baseline), as seen at code 24. The model
DAC->ADC scale is assumed: compare bench levels by measured ADC-side dither
amplitude and noise, not by DAC code.

## Bench procedure, per level (largest first)

1. Load the TXT, bring up, `adc -cal skew step 0`, then park at the known
   working point so acquisition starts there: `adc -cal loop test park 12`.
   Close the console.
2. Measure the ADC-side dither (read-only, no writes):
   `python tools/dither/dither_response_test.py --uart COM7 --frames 300 --out calibration_out/dither_power/amp<N>`
3. Joint run, fresh baseline (dither level changes the baseline estimate):
   `python tools/board_loop/board_axis_test.py suite --uart COM7 --joint --offset-points=32 --gain-points=0 --skew-points=32 --train 600 --hold 400 --repeats 2 --out calibration_out/dither_power/joint_amp<N> --execute`
4. After all levels, compare amplitudes with `tools/dither/dither_ladder.py`
   (`--state <N>=calibration_out/dither_power/amp<N>/raw` and the matching
   `--state-json`).

Record per level: baseline admitted or not (and the `LOOP-TEST-SPECTRUM`
reason if not), PASS per axis, HOLD mean +/- 1.96 SE, TRAIN convergence frame,
rejected frames. A level whose baseline is rejected is a result, not an error:
note it and stop lowering.

2026-11-25 note: one baseline at the old 260-spacing amp8192 vector (since
replaced) was rejected 600/600 `phase-spread` (155-324 mrad) from code 24; it
mixed -12 dB with half the events and is not a ladder point.
