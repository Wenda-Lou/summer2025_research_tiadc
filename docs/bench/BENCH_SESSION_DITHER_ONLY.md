# Dither-only bench runbook

Use this document as an execution checklist for the ZCU102/AD9695 bench. It intentionally omits
session narrative and result discussion. Run commands from the repository root.

## 1. Rules that apply to every run

- Run `sim` before a closed-loop bench run.
- Change one variable at a time. Hold the actuator, IFC, offset-calibration state and waveform
  fixed unless the current stage explicitly changes one of them.
- Put every state in its own `--out` directory. A repeated actuator code in one directory can
  overwrite earlier frames with the same `code<NN>` tag.
- Keep each waveform's TXT and JSON together. Load only the TXT into the DPG; pass the matching
  JSON to host tools.
- Match the CLI geometry to the TXT loaded on the DPG. A stale JSON or wrong CLI geometry can
  produce plausible captures with invalid folding.
- Every ADC sample-clock delay write resets the JESD link. After any delay transaction, verify
  link health and confirm consecutive frames differ.
- Actuator codes are limited to `0..48`. Check every target before running a step list. Firmware
  may return `OK` even when a move is clamped at a rail.
- Keep a skew baseline and all comparison states in the same run. Cross-session drift is larger
  than one actuator code.
- `--steps` values are relative to the starting code, not cumulative. From code 24,
  `--steps +8,+1` captures codes 24, 32 and 25.
- Use decimal register data with `spi -w`. The firmware truncates strings such as `0x0D` and may
  write `0x00` while reporting success. Hexadecimal register addresses are safe.
- Channel selection register `0x0008` is a bitmask: `1 = A`, `2 = B`, `3 = both`.
- On tone-free data, use `dither_mag` for gain and `dither_phase` for skew. `dither_fold` is a
  cross-check only; its scale is low on this bench.
- Quote folded replica peak-to-peak for amplitude ladders. Per-event peak-to-peak is noise-biased
  at low amplitudes.
- Interpret timing as a difference between fixed states. Do not use the slope projection as an
  absolute delay measurement.

## 2. Waveforms

### Tone-free pulse ladder

All files are under `waveforms/pulse_ladder/` and use `--amp-dbfs -120`.

| stem | ADC pulse width | duty | dither amplitude |
|---|---:|---:|---:|
| `w32adc_e16_t32_amp2000` | 32 samples | 24.6 % | 2000 LSB |
| `w16adc_e08_t16_amp2000` | 16 samples | 12.3 % | 2000 LSB |
| `w08adc_e04_t08_amp2000` | 8 samples | 6.2 % | 2000 LSB |
| `w06adc_e04_t04_amp2000` | 6 samples | 4.6 % | 2000 LSB |
| `w06adc_e04_t04_amp8000` | 6 samples | 4.6 % | 8000 LSB |
| `w06adc_e04_t04_amp16000` | 6 samples | 4.6 % | 16000 LSB |
| `w06adc_e04_t04_amp24000` | 6 samples | 4.6 % | 24000 LSB |
| `w06adc_e04_t04_amp30000` | 6 samples | 4.6 % | 30000 LSB |

Six ADC samples is the generator minimum because both edge and flat-top lengths must be at least
two samples.

Validate the JSON before loading the corresponding TXT:

```sh
python -m calibration_loop.run_calibration check \
  --waveform-json waveforms/pulse_ladder/<stem>.json --adc-rate 1.3e9
```

### Tone-plus-dither waveforms

Use one of these files from `waveforms/tone_dither/`:

| stem | use |
|---|---|
| `w06adc_e04_t04_amp5190_tone_m1p5` | default; highest alignment margin |
| `w12adc_e04_t16_amp5190_tone_m1p5` | lower offset scatter; use `--dither-top 16` |

Generate or regenerate the default pair:

```sh
python -m calibration_loop.run_calibration gen --out waveforms/tone_dither \
  --stem w06adc_e04_t04_amp5190_tone_m1p5 \
  --amp-dbfs -1.5 --dither-edge 4 --dither-top 4 --dither-scale 5190

python -m calibration_loop.run_calibration check \
  --waveform-json waveforms/tone_dither/w06adc_e04_t04_amp5190_tone_m1p5.json \
  --adc-rate 1.3e9
```

At a -1.5 dBFS tone, keep the dither at or below 5197 LSB to avoid DAC clipping.

## 3. Common preflight

Load the intended TXT on the DPG, then run:

```sh
python tools/raw_frame_probe.py --uart COM5
python tools/geometry_id.py --uart COM5 --frames 4
```

On the firmware console:

```text
adc -cal diagnose skewprep fullprep
adc -cal skew step 0
```

Require `result=OK`, `neutral_initialized=YES`, and record the reported actuator code. Then park
at code 24 through the ACK-checked transaction path:

```sh
python tools/skew_park.py --uart COM5 --code 24 --verify-frames 0
```

Use `--verify-frames 0` for tone-free waveforms because the park tool's verification route needs
a tone.

Check IFC on both channels. Equalise both to `0x0C` before stages that require the 1.59 Vpp
baseline:

```text
spi -w 0x0008 1
spi -r 0x1910
spi -w 0x0008 2
spi -r 0x1910
spi -w 0x0008 3
spi -w 0x1910 12
spi -r 0x1910
```

Take a short capture in a new directory:

```sh
python tools/dither_response_test.py --uart COM5 --frames 100 \
  --out calibration_out/response/preflight

python tools/dither_raw_evidence.py \
  --state ck=calibration_out/response/preflight/raw \
  --waveform-json <matching-waveform.json>
```

Proceed only when:

- consecutive frames differ;
- both channels contain the pulse train;
- the alignment margin is at least `6.0`;
- `dither_gain` is near `1.0`;
- the folded replica amplitude and width match the loaded waveform;
- neither channel is byte-identical across frames or near full scale unexpectedly.

Useful tone-free amplitude checks for a healthy 6-sample waveform are approximately 374 codes
pk-pk at 16000 LSB, 561 at 24000 LSB and 701 at 30000 LSB. Treat these as setup checks, not
calibration targets.

## 4. Recovery and stop conditions

| symptom | action |
|---|---|
| first UDP capture times out after boot | retry once |
| repeated or byte-identical frames | stop; reset/re-sync the JESD link, then rerun preflight |
| dead or grossly asymmetric channel after a delay write | stop; perform link recovery before changing registers |
| folded replica is only 1-2 codes or margin collapses | reload the TXT and verify the matching JSON/CLI geometry |
| actuator reports `OK` without changing code | stop; target is likely outside `0..48` |
| `skew_action` reports `abort:direction` | stop skew actuation; recheck route sign and waveform geometry |
| nearly all closed-loop frames are rejected | stop; inspect rejection reasons and rerun preflight |
| 325 MHz (`fs/4`) line remains with DPG stopped | treat it as a board/framing artifact; do not identify it as dither |

After any recovery, rerun `raw_frame_probe.py`, `geometry_id.py`, neutral preparation and the short
capture. Do not reuse pre-recovery captures as the baseline for a post-recovery state.

## 5. IFC register guide

| full scale | register code | decimal write value |
|---:|---:|---:|
| 1.36 Vpp | `0x0A` | `10` |
| 1.47 Vpp | `0x0B` | `11` |
| 1.59 Vpp | `0x0C` | `12` |
| 1.70 Vpp | `0x0D` | `13` |
| 1.81 Vpp | `0x0E` | `14` |
| 1.93 Vpp | `0x0F` | `15` |
| 2.04 Vpp | `0x00` | `0` |

Smaller full scale produces larger digital amplitude. Before using IFC as a known gain step,
verify per-channel addressing and link stability:

```text
spi -w 0x0008 1
spi -r 0x1910
spi -w 0x0008 2
spi -r 0x1910

spi -w 0x0008 1
spi -w 0x1910 13
spi -r 0x1910
spi -w 0x0008 2
spi -r 0x1910

spi -w 0x0008 3
spi -w 0x1910 12
spi -r 0x1910
```

After the differential write:

```sh
python tools/raw_frame_probe.py --uart COM5 --frames 3
python tools/frame_consistency.py --uart COM5 --frames 6
```

If the write disturbs the link, perform link recovery and rerun the checks after every later IFC
change.

## 6. Known-gain check

Keep the actuator fixed. Capture the baseline with both channels at `0x0C`, then move channel B
only to `0x0D`:

```sh
python tools/dither_response_test.py --uart COM5 --frames 500 \
  --out calibration_out/response/A_base
```

```text
spi -w 0x0008 2
spi -w 0x1910 13
spi -r 0x1910
```

Verify the link, then capture the stepped state:

```sh
python tools/raw_frame_probe.py --uart COM5
python tools/frame_consistency.py --uart COM5 --frames 6

python tools/dither_response_test.py --uart COM5 --frames 500 \
  --out calibration_out/response/A_step

python tools/dither_raw_evidence.py \
  --state base=calibration_out/response/A_base/raw \
  --state step=calibration_out/response/A_step/raw \
  --waveform-json waveforms/pulse_ladder/<stem>.json
```

Read `B/A mag`. The nominal change for `0x0C -> 0x0D` is `1.59/1.70`, or about -6.5 %.

Restore channel B:

```text
spi -w 0x0008 2
spi -w 0x1910 12
spi -r 0x1910
```

## 7. Pulse-width and amplitude sweep

Keep IFC at `0x0C` on both channels, offset calibration off and the actuator fixed. Load and
capture these states in order:

1. `w32adc_e16_t32_amp2000`
2. `w16adc_e08_t16_amp2000`
3. `w08adc_e04_t08_amp2000`
4. `w06adc_e04_t04_amp2000`
5. `w06adc_e04_t04_amp8000`
6. `w06adc_e04_t04_amp16000`

For each state:

```sh
python tools/dither_response_test.py --uart COM5 --frames 200 \
  --out calibration_out/response/B_<stem>

python tools/dither_raw_evidence.py \
  --state <stem>=calibration_out/response/B_<stem>/raw \
  --waveform-json waveforms/pulse_ladder/<stem>.json
```

Record folded replica peak-to-peak, FWHM, peak slope, alignment margin and timing scatter. Confirm
that width changes with geometry and replica amplitude scales with the commanded amplitude.

## 8. Full-scale amplitude ladder

Run common preflight, park the actuator, and confirm both IFC registers are `0x0C`. Keep the
actuator untouched throughout the ladder.

Load each TXT before its capture:

```sh
python tools/dither_response_test.py --uart COM5 --frames 300 \
  --out calibration_out/response/F_amp16000

python tools/dither_response_test.py --uart COM5 --frames 300 \
  --out calibration_out/response/F_amp24000

python tools/dither_response_test.py --uart COM5 --frames 300 \
  --out calibration_out/response/F_amp30000
```

Analyse all three states:

```sh
python tools/dither_ladder.py \
  --state 16000=calibration_out/response/F_amp16000/raw \
  --state 24000=calibration_out/response/F_amp24000/raw \
  --state 30000=calibration_out/response/F_amp30000/raw \
  --state-json 16000=waveforms/pulse_ladder/w06adc_e04_t04_amp16000.json \
  --state-json 24000=waveforms/pulse_ladder/w06adc_e04_t04_amp24000.json \
  --state-json 30000=waveforms/pulse_ladder/w06adc_e04_t04_amp30000.json
```

Use the folded replica ratios to locate compression. A ratio that falls short of the commanded
`1.5`, `1.25` or `1.875` is the compression signal. Also check that FWHM remains stable and timing
scatter decreases as replica slope increases.

Tie the top state to a known gain step:

```text
spi -w 0x0008 2
spi -w 0x1910 13
spi -r 0x1910
```

```sh
python tools/dither_response_test.py --uart COM5 --frames 300 \
  --out calibration_out/response/F_amp30000_ifcD

python tools/dither_ladder.py \
  --state off=calibration_out/response/F_amp30000/raw \
  --state on=calibration_out/response/F_amp30000_ifcD/raw \
  --state-json off=waveforms/pulse_ladder/w06adc_e04_t04_amp30000.json \
  --state-json on=waveforms/pulse_ladder/w06adc_e04_t04_amp30000.json
```

Restore B to `0x0C` immediately:

```text
spi -w 0x0008 2
spi -w 0x1910 12
spi -r 0x1910
```

## 9. Known-skew check

After preflight, confirm channel B is healthy with a short state capture. Then capture the starting
code, `+8` codes and `+1` code in one run:

```sh
python tools/dither_response_test.py --uart COM5 --frames 1000 --steps +8,+1 \
  --allow-skew-writes --out calibration_out/response/ladder
```

Analyse each code separately:

```sh
python tools/dither_raw_evidence.py \
  --state base=calibration_out/response/ladder/raw/code24_frame_*.bin \
  --state c32=calibration_out/response/ladder/raw/code32_frame_*.bin \
  --state b1=calibration_out/response/ladder/raw/code25_frame_*.bin \
  --waveform-json waveforms/pulse_ladder/w32adc_e16_t32_amp2000.json
```

Judge the slope of timing difference versus actuator code and the significance of the one-code
state against the same run's baseline. Do not compare its absolute timing value with another
waveform or session.

To size a run from a 100-frame pilot, use the measured per-frame timing scatter `sigma`:

```text
N = (k * sigma / expected_step_ps)^2
```

Use the step measured at the working code. Do not assume a uniform ps/code value across the
actuator range.

## 10. Optional multi-point gain check

Only run this after the IFC and link checks pass. Fix channel A at 1.70 Vpp and capture channel B
at 1.70, 1.59 and 1.47 Vpp, 500 frames per state. Confirm both registers after every write and
keep the actuator fixed.

Expected nominal B/A ratios are approximately `1.000`, `+6.9 %` and `+15.6 %`. IFC's smallest
step is about 6 %, so do not present this as direct proof of sub-1 % gain detection.

## 11. Known-offset check

Restore channel B IFC to `0x0C`, keep the actuator fixed and inspect both channels' offset state:

```text
spi -w 0x0008 2
spi -w 0x1910 12
spi -r 0x1910

adc -offset
status
back

spi -w 0x0008 1
spi -r 0x0701
spi -w 0x0008 2
spi -r 0x0701
```

Capture the baseline:

```sh
python tools/dither_response_test.py --uart COM5 --frames 500 \
  --out calibration_out/response/E_base
```

Enable offset calibration on channel B through the firmware menu. Do not replace this with a raw
write; the firmware updates both `0x0701` and `0x073B`.

```text
spi -w 0x0008 2
adc -offset
on
status
back

spi -w 0x0008 2
spi -r 0x0701
spi -w 0x0008 1
spi -r 0x0701
```

Verify channel A did not change, probe the link, and capture the stepped state:

```sh
python tools/raw_frame_probe.py --uart COM5
python tools/frame_consistency.py --uart COM5 --frames 6

python tools/dither_response_test.py --uart COM5 --frames 500 \
  --out calibration_out/response/E_step

python tools/dither_raw_evidence.py \
  --state base=calibration_out/response/E_base/raw \
  --state step=calibration_out/response/E_step/raw \
  --waveform-json waveforms/pulse_ladder/<stem>.json
```

Judge `DC(B-A)`. Turn B's offset calibration off after the test:

```text
spi -w 0x0008 2
adc -offset
off
back
```

## 12. Tone-free closed loop

Run the offline checks:

```sh
python calibration_out/_tone_free_loop_test.py
python tools/loop_direction_check.py
```

Load `w06adc_e04_t04_amp16000.txt`, run common preflight and confirm the matching geometry. Before
the loop, compare timing routes on a known code pair:

```sh
python tools/timing_route_check.py \
  --state 24=calibration_out/response/R24/raw \
  --state 32=calibration_out/response/R32/raw \
  --waveform-json waveforms/pulse_ladder/w06adc_e04_t04_amp16000.json
```

Proceed only if `dither_phase` has the correct sign and has been scale-checked for the working
waveform. Do not select `dither_fold` merely because it has lower scatter.

Run the simulation preflight:

```sh
python -m calibration_loop.run_calibration sim --iterations 60 --tone-free \
  --amp-dbfs -120 --dither-edge 4 --dither-top 4 --dither-scale 16000
```

Run the bench loop:

```sh
python -m calibration_loop.run_calibration bench --uart COM5 --tone-free \
  --gain-observable dither_mag --skew-observable dither_phase --no-cancellation \
  --allow-skew-writes --iterations 300 \
  --amp-dbfs -120 --dither-edge 4 --dither-top 4 --dither-scale 16000 \
  --out calibration_out/closed_loop --stem run_tonefree_phase
```

Monitor:

| field | requirement |
|---|---|
| `gain_source` | `dither_mag` on every accepted row |
| `skew_observable` | `dither_phase` |
| `skew_action` | no more than one code per 20-frame batch; every move ACKs `OK` |
| `skew_abort` | `null` |
| `skew_used_ps` | magnitude falls toward the deadband |
| `gain_mag_ratio` | approaches `1.0000` |
| signed offset | approaches zero; do not mistake absolute scatter for bias |
| alignment margin | remains above the acceptance floor |

For a tone-free run, use `gain_mag_ratio`, signed offset, `skew_used_ps`,
`dbc_ab_coherent[_cal]` and `snr_dither_db`. Ignore tone ratio, tone-phase and SNDR/SFDR/ENOB at
`f_in`; they describe the noise floor when no tone is present.

## 13. Tone-mode closed loop

Load `w06adc_e04_t04_amp5190_tone_m1p5.txt`, then run common preflight against its JSON. The short
capture should show a folded replica near 121 codes pk-pk and `dither_gain` near 1.0.

Run simulation:

```sh
python -m calibration_loop.run_calibration sim --iterations 60 \
  --amp-dbfs -1.5 --dither-edge 4 --dither-top 4 --dither-scale 5190
```

Run the bench loop. Do not pass `--tone-free`; the defaults are tone gain, tone phase and
cancellation enabled.

```sh
python -m calibration_loop.run_calibration bench --uart COM5 --allow-skew-writes \
  --iterations 300 \
  --amp-dbfs -1.5 --dither-edge 4 --dither-top 4 --dither-scale 5190 \
  --out calibration_out/closed_loop --stem run_tone_w06
```

If the 12-sample offset-first waveform is loaded, replace `--dither-top 4` with
`--dither-top 16` and use a distinct output stem.

Read `tone_ratio`, signed `DC(B-A)`, tone-phase skew, SNDR, SFDR, ENOB,
`raw_difference_dbc` and `cal_difference_dbc`.

## 14. Recording checklist

Record one line per state:

```text
state | actuator code | IFC(A/B) | offset-cal state | waveform TXT/JSON |
frames attempted/accepted | start/end time | raw directory | recovery actions | notes
```

For each state, save:

- folded replica peak-to-peak for A and B;
- `B/A mag` or the active loop gain observable;
- signed `DC(B-A)`;
- active timing route, mean, per-frame scatter and batch standard error;
- alignment margin;
- register read-backs after every write;
- ACK result for every actuator transaction.

Do not combine states separated by a reboot, JESD recovery, waveform change or actuator
reinitialisation without recording that boundary.
