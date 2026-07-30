# ADC Calibration Board Test Plan

This plan verifies the FPGA ADC calibration path on hardware after the shared
calibration estimator modules were integrated into the board build.

The desktop simulator validates estimator algorithms with synthetic captures.
This board plan validates acquisition, timing context, UART command routing,
hardware capture behavior, and the FPGA adapters that feed the shared
estimators.

## Test Scope

Validate on board:

- AD9695 initialization and register access
- JESD link stability
- DMA capture and cache coherency
- uploaded DAC reference availability
- `adc -cal` full staged flow
- standalone timing, offset, gain, skew, and status commands
- shared dither/skew/performance estimator integration through the FPGA adapters
- final performance CSV/UART output
- invalidation behavior after reset or changed timing context

Do not validate here:

- new skew correction register writes; skew is measurement-only
- promotion of dither diagnostics into hard offset/gain PASS criteria
- artificial A/B interleaving; A and B remain parallel same-instant captures

## Required Setup

- ZCU102 or target board powered and programmed with the updated ELF/bitstream
- AD9695 connected and clocked
- AD9164 DAC reference waveform downloaded
- shared ADC/DAC clocking locked
- UART terminal logging to a file
- Ethernet/DPG/reference upload path available if used by the workflow
- input amplitude set below clipping

Record before testing:

- firmware git commit or build timestamp
- bitstream/ELF used
- ADC sample rate
- DAC sample rate
- input tone frequency
- input amplitude
- clock source
- UART log filename

## 1. Boot And Hardware Bring-Up

1. Power cycle the board.
2. Program FPGA and boot the application.
3. Confirm the UART banner appears without fatal errors.
4. Confirm AD9695 initialization reports success.
5. Confirm AD9695 PLL/JESD lock reports locked.
6. Confirm no repeated JESD, DMA, or SPI errors appear at idle.

Pass if:

- UART is responsive.
- AD9695 initializes successfully.
- JESD link remains stable for at least 30 seconds.
- No DMA error is reported before manual capture.

## 2. Basic Command And DMA Smoke Test

Run:

```text
adc -cal help
```

Confirm these commands are listed:

```text
adc -cal
adc -cal timing [frames]
adc -cal diagnose [frames]
adc -cal offset
adc -cal gain
adc -cal skew
adc -cal skew diagnose
adc -cal status
adc -cal reset
```

Run the existing DMA capture/readback command used in this firmware build.
If the project menu exposes DMA commands, perform one capture and readback.

Pass if:

- command parser remains responsive;
- DMA capture completes;
- no timeout or cache/DMA error is printed;
- captured samples are nonzero and not stuck at rail codes.

## 3. Upload Or Verify DAC Reference

Download the DAC waveform using the normal DPG/reference workflow.

Then run:

```text
adc -ref
```

If more detail is needed:

```text
adc -ref diagnose
```

Pass if:

- reference buffer is ready;
- reference length is nonzero;
- reported sample-rate relationship matches the intended DAC/ADC rates;
- dominant/reference tone matches the expected input tone;
- no reference checksum or format error appears.

## 4. Reset Calibration State

Run:

```text
adc -cal reset
adc -cal status
```

Pass if:

- software gain returns to `1.0`;
- software offset returns to `0`;
- calibration state is not valid;
- performance status is `NOT RUN`;
- no stale performance VALID/INVALID data is shown.

## 5. Timing Alignment

Run:

```text
adc -cal timing
```

Optional deeper diagnostic:

```text
adc -cal diagnose
```

Record:

- selected channel
- canonical phase
- integer lag
- fractional lag
- total lag
- fixed window start and length
- waveform correlation
- timing validation summary
- tone fit validation
- dither alignment validation
- channel consistency
- existing-vs-dither disagreement

Pass if:

- existing timing correlation reports `PASS`;
- overall timing status is `PASS`, or `WARNING` only for new diagnostics;
- fixed window length matches the configured calibration window;
- selected channel/canonical phase remain stable over repeated runs;
- no numerical validation failure is reported.

Fail if:

- existing timing reports `FAIL`;
- no valid fixed window is stored;
- numerical validation fails;
- reference upload or sample-rate state is reported invalid.

## 6. Offset Stage

Run after a successful timing stage:

```text
adc -cal offset
adc -cal status
```

Record:

- accepted/rejected frames
- existing offset loop status
- dither estimator status
- applied offset correction
- filtered residual
- verification residual
- convergence/stability indicators

Pass if:

- existing offset loop status is `PASS` or the expected provisional status;
- required accepted-frame count is met;
- final residual is within configured offset tolerance;
- dither estimator is `PASS` or diagnostic `WARNING`, not fatal to the stage;
- no hardware/register update error appears.

Fail if:

- timing context is invalid;
- too few valid frames are available;
- existing controller cannot produce a valid correction;
- fatal numerical or hardware error appears.

## 7. Gain Stage

Run after successful timing and offset:

```text
adc -cal gain
adc -cal status
```

Record:

- accepted/rejected frames
- existing gain loop status
- dither gain status
- nominal system gain
- normalized final gain
- applied gain correction
- gain verification error
- saturation/clamping messages

Pass if:

- existing gain loop status is `PASS`;
- minimum accepted-frame count is met;
- filtered gain error is within tolerance;
- applied correction is not clamped;
- dither gain estimator is `PASS` or diagnostic `WARNING`;
- no fatal numerical or hardware error appears.

Fail if:

- gain status is `SATURATED`, `NOT CONVERGED`, or `FAIL`;
- final correction is clamped while residual remains outside tolerance;
- no reliable gain estimate is produced.

## 8. Open-Loop Skew Measurement

Run after successful timing, offset, and gain:

```text
adc -cal skew
```

For detailed diagnostics:

```text
adc -cal skew diagnose
```

Record:

- accepted/rejected skew frames
- estimator status
- relative skew in samples and ps
- best/median/final skew
- rising/falling edge estimates
- edge disagreement
- dither event count
- failure reason if not PASS

Pass if:

- command runs without hardware writes;
- UART states register writes are `NONE`;
- estimator status is `PASS` or diagnostic `WARNING`;
- accepted skew frame count meets the configured minimum;
- relative skew and edge disagreement are finite.

Do not run:

```text
adc -cal skew step +/-N
```

unless skew actuator register semantics have been independently verified. The
current expected behavior is measurement-only.

## 9. Full Automatic Calibration

After reset and reference upload, run:

```text
adc -cal
```

Record the complete UART log from command start to summary end.

Expected stage order:

1. Timing Alignment
2. Offset Calibration
3. Gain Calibration
4. Open-Loop Skew Measurement
5. Performance Measurement

Pass if:

- timing failure prevents later stages from running;
- offset failure prevents gain and later stages from running;
- gain failure/saturation behavior matches the existing production rules;
- skew remains measurement-only and performs no register writes;
- performance measurement runs after skew;
- `adc -cal status` reports performance `VALID` or `INVALID`, not `NOT RUN`,
  after a performance attempt.

## 10. Final Performance Measurement

The performance stage is not a calibration stage and must not update hardware.
It measures the fully calibrated output.

From the full `adc -cal` run, or from any standalone performance command if one
is exposed in the build, record:

- frames attempted
- frames valid
- frames rejected
- SNDR
- ENOB
- SFDR
- THD
- signal frequency
- worst spur frequency
- raw and calibrated Channel A metrics
- raw and calibrated Channel B metrics
- raw and calibrated combined metrics
- raw/cal channel-difference metrics
- CSV rows

Pass if:

- one CSV row is printed per attempted capture;
- valid batch reports `VALID`;
- poor-quality batch reports `INVALID` with a reason;
- combined metrics follow the current FPGA limitation: Channel A alias for
  parallel same-instant A/B capture, not artificial interleaving;
- no coefficient or hardware register changes occur during the stage.

## 11. Invalidation Checks

After a successful full run:

```text
adc -cal status
adc -cal reset
adc -cal status
```

Pass if performance changes from available to `NOT RUN`.

Then repeat:

1. run `adc -cal timing`;
2. run `adc -cal offset`;
3. run `adc -cal gain`;
4. confirm performance remains `NOT RUN` until performance is attempted.

If practical, invalidate the timing context by changing/reuploading the
reference waveform, then run:

```text
adc -cal status
```

Pass if stale performance availability is cleared.

## 12. Repeatability Test

Run the full flow three times without changing hardware:

```text
adc -cal reset
adc -cal
adc -cal status
```

For each run, compare:

- selected channel
- canonical phase
- fixed window
- total lag
- offset correction
- gain correction
- final skew
- SNDR/ENOB/SFDR/THD

Pass if:

- selected timing context is stable;
- offset/gain corrections vary only within expected measurement noise;
- performance metrics do not jump unexpectedly;
- no run reports a fatal numerical error.

## 13. Failure-Injection Checks

Run only when it is safe to disturb the setup.

Reference mismatch:

1. load an intentionally wrong reference or tone;
2. run `adc -cal timing`;
3. confirm timing fails and later stages do not run.

Low amplitude or missing input:

1. reduce or remove input tone;
2. run `adc -cal timing`;
3. confirm low correlation/no accepted frames are reported.

Clipping:

1. increase input amplitude near clipping;
2. run `adc -cal timing` or `adc -cal diagnose`;
3. confirm clipped input is rejected or clearly reported.

Restore the normal reference and signal after each failure-injection test.

## 14. Required Artifacts To Save

Save these files for review:

- full UART log for boot and hardware bring-up
- full UART log for `adc -cal help`
- full UART log for `adc -ref` or `adc -ref diagnose`
- full UART log for standalone timing/offset/gain/skew
- full UART log for one successful `adc -cal`
- final performance CSV block
- `adc -cal status` before and after reset
- build ELF and size output
- note of any warning, rejection reason, or unstable metric

## Board Smoke-Test Summary Template

```text
Date:
Operator:
Firmware commit:
Bitstream:
Board:
ADC sample rate:
DAC sample rate:
Input tone:
Input amplitude:
Clock source:

Boot/AD9695/JESD: PASS/FAIL
DMA capture: PASS/FAIL
Reference upload: PASS/FAIL
Timing: PASS/WARNING/FAIL
Offset: PASS/RUNNING/NOT CONVERGED/FAIL
Gain: PASS/SATURATED/NOT CONVERGED/FAIL
Skew measurement: PASS/WARNING/FAIL
Performance: VALID/INVALID/NOT RUN
Invalidation after reset: PASS/FAIL
Repeatability: PASS/FAIL

Notes:
```
