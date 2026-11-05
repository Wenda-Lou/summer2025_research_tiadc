# calibration_loop_c

2026-10-28: the board now enables `options.reference_free` by default. This
branch uses only simultaneous A/B broadband dither samples and the ADC sample
rate. It does not build/use `loop_ref`, pulse geometry, polarity tables, or tone
fits. `adc_calibration_loop_blind.h` estimates relative gain from spectral power,
relative sampling delay from cross-spectrum phase slope (positive B later), and
relative DC from `mean(B) - gain_ratio * mean(A)`. Common absolute errors are
unidentifiable. Spectra must have adequate broadband excitation and compatible
channel transfer shapes; constant/single-frequency/weak/incoherent inputs are
rejected. Historical template/tone modes remain for replay, not board defaults.
Tests use unknown-to-estimator waveforms, corrupted waveform metadata, signed
known delays, three pulse widths/amplitudes, fresh frozen holdouts and independent
noise rejection. Bench validation of this new estimator is still pending.

C port of the host calibration loop (`calibration_loop/`), as the first step of
moving the loop from the PC to the Zynq ARM core. Hardware-independent C11: no
Xilinx headers, no heap, libm only. Used by `thesis_v3_loop_appl` on the A53 and
built and tested by `calibration_sim`.

| File | Contents |
|---|---|
| `adc_calibration_loop.h/.c` | The port: waveform config, de-framing, tone fit, loop alignment, block estimator, block-LMS state, acceptance filter, batch skew decision with direction latch, `run()` driver. |
| `export_golden.py` | Runs the Python loop against `BenchModel` and writes a golden file (raw DMA bytes plus the Python results for every capture). |

## What is ported

The tone route, which is the host default:
`cancel_signal=True`, `gain_observable="tone"` (with the same fallback to
`"dither"`), `skew_observable="phase"`.

With `options.tone_free=1`, the controller selects sign-free per-event
`dither_mag` gain and fractional-template `dither_phase` timing from
`dither_raw.py` as present in commit `d22d23ff6753b4503c54157529ad8396483cb006`.
That file's dither equations are unchanged. Strict dither preparation uses only
time-domain channel correlation and the pulse template. It does not call tone
purity, carrier fitting, tone subtraction, or tone phase reconstruction.
Offset comes from the polarity-decoupled flat-top profile, not a tone-fit DC.
Tone cancellation and tone gates are disabled; missing dither
observables reject the frame without falling back to a tone fit. Tone gain and
timing outputs are NaN in this mode. Offset keeps the polarity-decoupled flat-top
observable. The route-specific batch uses median/MAD rejection and shares the
same ACK and direction latch. `skew_fold_ps` is a diagnostic cross-check only.
The firmware's loop and single-axis commands require dither-only wave 2 or 3.
Wave 3 is the boot default, at DAC full-scale peak (32767 codes); wave 2 retains
5190 codes. Tone selections are refused before capture or actuation. ADC
clipping is rejected independently of the legitimate full-scale DAC peaks.
Historical tone code remains for offline regression and is not the board's
calibration route.

| Python | C |
|---|---|
| `DitherConfig`, `validate()` | `adc_cal_loop_config_t`, `adc_cal_loop_config_validate()` |
| `pulse`, `pulse_derivative`, `dither_only_loop` | `adc_cal_loop_pulse*`, precomputed in `adc_cal_loop_init()` |
| `unpack_words`, `deframe`, `prepare_capture` | `adc_cal_loop_prepare_capture()` |
| `fit_tone` | `adc_cal_loop_fit_tone()` |
| `align_to_loop`, `estimate_channel`, `estimate_block` | `adc_cal_loop_estimate_block()` |
| `CalibrationState.update/apply` | `adc_cal_loop_state_update()` |
| `skew_batch`, `SkewActuator.error_to_steps` | `adc_cal_loop_skew_batch()`, `adc_cal_loop_error_to_steps()` |
| `CalibrationLoop.step/_reject_reason/_skew_decision/run` | `adc_cal_loop_step()`, `adc_cal_loop_run()` |

Not ported: the joint
multi-frame accumulator, and the SNDR/SFDR/difference-spur metrics in
`metrics.py`. Those metrics are logged by the host but are not used by the loop.

Deliberate differences from a line-by-line translation. None of them changes a
result:

- The polarity sequence is carried as data (`cfg.polarity`). Python builds it with
  numpy's `RandomState.shuffle`. The default table is the seed-20260725 sequence.
  For another waveform, copy the `"polarity"` list from its JSON.
- `align_to_loop` sums over the nonzero template samples instead of using an FFT.
- `fit_tone` solves the 3×3 normal equations instead of calling `lstsq`.
- The host calls `estimate_channel` a fifth and sixth time only to refit the tone
  phase at the shared frequency. The C refits just the phase.
- `_tone_purity` (an FFT) is evaluated only when it could break a de-framing tie.

## Hardware hooks

The loop never touches hardware itself. The caller supplies:

- **Captures.** Pass the raw DMA buffer to `adc_cal_loop_step()`, or give
  `adc_cal_loop_run()` a capture callback.
- **Skew actuator** (`adc_cal_loop_actuator_t`):
  - `request_steps(ctx, delta, &ack)` selects the hardware branch. Gain and offset
    are updated per frame. Skew is decided per batch, in control codes, one code at
    a time, and is trusted only through `ack.ok` / `ack.code_after`. On the board,
    this is where `handle_adc_skew_transaction_cmd()` belongs.
  - `request_steps == NULL` selects the model branch: a per-frame ps integrator
    pushed out through `command_skew_ps`.

The correction convention is the host one, `y_cal = (y - offset) * gain_corr`.
It is not the `butils_calibration.c` convention.

## Test

The board experiment adapter also uses this core. `update_axes` defaults to all
three axes; zero freezes correction coefficients while continuing measurement.
Optional post-ADC offset injection, raw-rail rejection and a tone requirement
are enabled only for experiments. See
[`CALIBRATION_LIMIT_TEST.md`](../calibration_loop/CALIBRATION_LIMIT_TEST.md).

`python checks/test_dither_only.py` runs with every tone fit, purity and
carrier-metric function patched to raise. Native tests also count every tone
fit/purity call and require zero during dither calibration. Both paths verify
that changing unused carrier metadata leaves results unchanged.

```sh
cd calibration_sim
cmake -S . -B build && cmake --build build
./build/adc_cal_sim --run-loop-tests                 # native, no input files
python ../calibration_loop_c/export_golden.py         # needs numpy; writes output/loop_golden.txt
./build/adc_cal_sim --loop-golden output/loop_golden.txt
```

The golden file is generated, so do not commit it. Tone cases:
`model_nominal`, `model_mismatch`, `batch_honest`, and `batch_lying` (the direction
latch must fire after exactly one move).
Five additional dither-only cases cover model convergence, acquisition from
+200 ps, honest batch actuation and a reversed actuator. Version 2 records the
tone-free option, dither magnitudes, phases, both timing routes and the timing
actually used. Unsupported tone diagnostics are NaN, while the driven estimates,
controller states and actions are compared with Python. The reader also accepts
version 1 files. The full-scale DAC case explicitly models ADC headroom.

Result on 2026-09-30 (gcc 15, WSL): 227/227 pass under ASan and UBSan. Across all
140 replayed captures, the largest relative deviation between C and Python in any
estimate or state field is 9.3e-10. Rotation, polarity, skew source, skew actions
and control codes agree exactly.

### The n0 tie (a property of the host algorithm, not of the port)

A capture sees about 8 impulses, and their polarity pattern recurs in the 64-event
sequence. Two loop positions a whole number of 130-sample slots apart can therefore
score identically in `align_to_loop`. Python breaks the tie by FFT rounding and the
C breaks it by summation rounding, so they may pick different `n0`. That happens on
about 10 % of the golden frames. Usually both choices give identical estimates. On
one frame (`batch_lying`, frame 36) they did not: offset read 0.21 codes with one
choice and 0.07 with the other.

The golden replay handles this with `loop.n0_override`. The C checks that the host's
`n0` scores within 1e-9 of its own peak, then adopts it.
`prep.n0_natural` and `prep.n0_score_ratio` expose the tie. It is worth resolving
properly (for example, a longer non-repeating polarity sequence) before relying
on single-frame offsets.

## Cost

About 58 ms per capture on x86 at `-O2`. The cost is dominated by the frequency
refinement in `fit_tone` (up to 80 cos/sin solves over the record, five fits per
channel). Expect a few hundred ms per capture on the Cortex-A53 until that is
optimised, for example with a trig recurrence or fewer refine steps.

## Next steps toward the ARM

1. Board glue in `butils_calibration.c`: add an `adc -cal loop N` command. It
   should capture from the DMA buffer (invalidate the cache first), call
   `adc_cal_loop_step()`, route `request_steps` to the skew transaction, and print
   one short row per iteration.
2. Load the waveform config and polarity to match the vector on the DPG, instead
   of using the compiled-in default.
3. Run a matched pair on the bench: park at code 24, run the host loop, park again,
   then run the ARM loop.
