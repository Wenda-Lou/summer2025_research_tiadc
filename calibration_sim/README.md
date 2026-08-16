# ADC Calibration Desktop Simulation

This directory contains a pure desktop C simulation and test harness for the ADC
calibration firmware.  It is intended to run on Linux or WSL without the FPGA,
Vitis, Xilinx BSP, DMA, SPI, UART, JESD, or Ethernet hardware.

## What It Reuses

The simulator directly compiles these production modules:

- `adc_frame.c`
- `adc_calibration_pipeline.c`
- `adc_calibration_dither.c`
- `adc_calibration_skew.c`
- `adc_calibration_performance.c`
- `calibration.c`
- `timing_alignment.c`
- `reference_buffer.c`

The tests call production APIs such as `adc_reconstruct_channels()`,
`adc_reconstruct_frame()`, `timing_find_circular_lag()`,
`timing_estimate_fractional_lag()`, `calibration_init()`,
`calibration_analyze_frame()`, `calibration_update()`,
`calibration_process_frame()`, software correction setters, loop resets, and
reference-buffer upload/finalize functions. Pipeline integration scenarios call
the shared production stage sequencer in `adc_calibration_pipeline.c`.

The unit suite also performs an end-to-end deterministic replay using the exact
`sine_100MHz_2p6GSPS_impulse_dither.txt` DAC reference and
`adc_capture_20260801_180451.csv` DMA capture.  It advances the shared five-stage
pipeline through timing, offset, gain, open-loop skew, and performance.  The
recorded frame is replayed for each requested batch/frame; this validates the
complete software path but does not pretend that one capture contains
independent analog-noise realizations.

The replay suite also retains the historical bin-71 DMA capture
`adc_capture_20260805_100121.csv` as an explicit test of the general-purpose
rate-matching estimator. Production calibration no longer applies that
diagnostic correction: a waveform tagged for 1.45 GSPS is rejected when the
configured ADC rate is 1.30 GSPS. Matching 1.30/2.60 GSPS metadata is accepted.

`butils.c` and `butils_calibration.c` remain the production UART and FPGA
adapter source. They are not compiled wholesale in this host target because they
still contain Xilinx BSP interfaces and board-side orchestration, capture,
printing, and register-update code. The simulator provides only the two required
invalidation hooks from `calibration_pending.h`.

## What Is Simulated

- Synthetic DAC-rate and ADC-rate reference generation
- Parallel same-instant Channel A and Channel B capture generation
- Offset, gain, delay/skew, noise, drift, clipping, harmonics, and spur injection
- Firmware DMA byte layout:
  - `w0..w3` are Channel A samples
  - `w4..w7` are Channel B samples
  - samples are signed 14-bit values left aligned in `int16_t`
- Unit tests for production reconstruction, timing, calibration, reference upload,
  correction sign, dither/skew/performance estimators, boundary behavior, and
  host invalidation-hook wiring
- Controller-style synthetic calibration loops using production calibration APIs
- Scenario logs and CSV files

## What Is Not Simulated

The simulator cannot validate:

- JESD transport ordering on real hardware
- DMA hardware state
- cache coherency
- analog clock jitter
- real ADC noise
- SPI register behavior
- hardware skew correction registers

Skew remains open-loop characterization only.  The simulated channels are not
treated as an interleaved stream; they are parallel Channel A and Channel B
captures, matching the current FPGA implementation.

## Build

```sh
cd calibration_sim
cmake -S . -B build
cmake --build build
```

Optional sanitizer build:

```sh
cd calibration_sim
cmake -S . -B build-asan -DADC_CAL_ENABLE_SANITIZERS=ON
cmake --build build-asan
```

## Run

```sh
./build/adc_cal_sim --help
./build/adc_cal_sim --list-scenarios
./build/adc_cal_sim --run-unit-tests
./build/adc_cal_sim --run-controller-tests
./build/adc_cal_sim --scenario nominal
./build/adc_cal_sim --run-pipeline nominal
./build/adc_cal_sim --run-all-pipeline-scenarios
./build/adc_cal_sim --run-all
./build/adc_cal_sim --stress-seeds 100
./build/adc_cal_sim --run-all --seed 1234 --output-dir output
```

The process exits with `0` only when every selected test and scenario expectation
passes.

## Coverage Matrix

| Component | Host coverage | FPGA validation |
| --- | --- | --- |
| automatic stage sequencer | Direct | Built |
| production dither estimator | Direct | Built |
| production skew estimator | Direct | Built |
| production performance analysis | Direct | Built |
| production performance batch | Direct | Built |
| FPGA adapters to shared estimators | Source inspected | Built |
| UART parser | Not host tested | Not tested |
| DMA/JESD/cache | Not host tested | Not tested |
| SPI/register behavior | Not host tested | Not tested |
| analog hardware | Synthetic only | Not tested |

The desktop pipeline flow tests the same shared production stage sequencer used
by the FPGA `adc -cal` full-run adapter. It is not a full board validation of the
`butils_calibration.c` board adapters, UART formatting, or hardware side
effects.

Correction-convention note: direct `calibration.c` tests use that module's local
documented model, `corrected = raw * gain + offset`. The integrated firmware
path in `butils_calibration.c` uses `final_code = round(gain * (raw + offset))`;
the simulator mirrors that integrated convention in pipeline performance
captures and does not claim to test the full integrated `butils` controller.

## Scenarios

- `nominal`: small offset, small gain error, zero relative skew, moderate noise
- `timing_shift`: frame-to-frame integer shifts plus fractional delay
- `noisy`: noisy captures with periodic low-correlation frame injection
- `bad_reference`: reference/tone mismatch that should fail cleanly
- `offset_positive`: positive ADC offset
- `offset_negative`: negative ADC offset
- `gain_low`: ADC gain below nominal
- `gain_high`: ADC gain above nominal
- `gain_saturation`: required correction exceeds the supported range
- `skew_positive`: Channel B delayed relative to Channel A
- `skew_negative`: Channel B advanced relative to Channel A
- `skew_outside_range`: relative skew beyond the supported linear range
- `insufficient_dither`: too few complete dither events
- `clipped_input`: clipped captures
- `performance_noise`: high noise, expected lower SNDR/ENOB than nominal
- `performance_harmonic`: increased harmonic distortion, expected worse THD
- `performance_spur`: added spur, expected worse SFDR
- `invalidation`: host-model invalidation scenario; verifies simulator hook
  wiring only, not the production automatic calibration state in `butils.c`

## Pipeline Scenarios

`--run-all-pipeline-scenarios` executes these shared-sequencer integration
cases:

- `nominal`
- `timing_shift`
- `noisy`
- `gain_saturation`
- `skew_positive`
- `skew_negative`
- `performance_distortion`
- `timing_failure`
- `offset_nonconvergence`
- `gain_verification_failure`
- `insufficient_dither`
- `performance_invalid`
- `invalidation_after_complete`
- `standalone_sequence`

## Scenario Classification

True production-function tests:

- Unit tests for `adc_reconstruct_channels()`, `adc_reconstruct_frame()`,
  timing lag/fractional lag, `calibration_analyze_frame()`,
  `calibration_update()`, `calibration_process_frame()`, correction setters,
  reference-buffer upload/finalize, `adc_cal_dither_analyze()`,
  `adc_cal_skew_estimate_from_residuals()`, and
  `adc_cal_perf_run_batch()`.

Simulator-model tests using production primitives:

- `nominal`
- `timing_shift`
- `noisy`
- `offset_positive`
- `offset_negative`
- `gain_low`
- `gain_high`
- `gain_saturation`
- `performance_noise`
- `performance_harmonic`
- `performance_spur`

Placeholder or simplified tests:

- `bad_reference`: synthetic scenario gate, not the production timing validator
- `clipped_input`: synthetic clipping gate, not every production rejection path
- `invalidation`: host invalidation-hook wiring only

Shared-pipeline integration tests:

- Stage sequencing, fail-fast behavior, performance NOT RUN invalidation, and
  standalone timing/offset/gain/skew/performance command ordering are direct
  tests of `adc_calibration_pipeline.c`.
- Timing and offset/gain callback bodies use production
  `timing_alignment.c` and `calibration.c` primitives.
- Skew callback bodies synthesize dither residuals but call
  `adc_cal_skew_estimate_from_residuals()`, which internally exercises the
  shared dither detector.
- Performance callback bodies synthesize host captures but call
  `adc_cal_perf_run_batch()` for frame and batch metric calculation.

## Outputs

The simulator writes:

- `output/test_summary.txt`
- `output/unit_test_results.csv`
- `output/calibration_iterations.csv`
- `output/performance.csv`
- `output/stress_summary.csv`

The performance CSV is intentionally wide and row-oriented so it can be compared
with board-side output.  The current desktop harness includes the core production
columns needed for plotting and inspection; the board firmware remains the source
of truth for the full UART CSV printer.

## Adding A Scenario

Add the scenario name to `k_scenarios` in `sim_tests.c`, then add its signal
configuration in `sim_signal_configure_scenario()`.  Prefer deterministic changes
controlled by `random_seed` so failures can be reproduced.

## Interpreting Failures

Start with `output/test_summary.txt`, then inspect
`output/calibration_iterations.csv` for rejection reasons and coefficient
movement.  For spectral regressions, compare `output/performance.csv` across
scenarios or seeds.  Performance tests intentionally use monotonic relationships
instead of exact SNDR/ENOB/SFDR/THD numbers.

Boundary tests record hard pass/fail checks for supported behavior and
`KNOWN_GAP` rows for robustness issues that require production changes.
Non-finite calibration configuration values are rejected: every floating-point
`calibration_config_t` field is covered with NaN, +Inf, and -Inf regression
tests. The remaining known gap is the inability to force the reference
generation counter through wraparound without adding a production test seam.

## Stress Mode

`--stress-seeds N` runs stochastic scenarios for seeds `1..N` and continues
after failures so every seed/scenario result is written. Results go to
`output/stress_summary.csv` with:

- seed
- scenario
- pass/fail
- measured value
- expected range
- failure reason
