# AGENTS.md

Short guide for AI coding agents. The full history (dated status notes, bench
numbers, rationale) is in `docs/AGENTS_HISTORY.md`; read the relevant section
there before changing an area it covers. Put new dated notes there, and keep
this file to rules, layout, commands and current status.

## Hard rules

- **Board calibration is reference-free, dither-only.** No supplied reference,
  tone, waveform parameters, template or polarity sequence enters feedback.
  `calibration_loop_c` runs with `reference_free`; `wave` is a legacy no-op.
  Never restore the older tone/template defaults to the board. Scope:
  `calibration_loop/CALIBRATION_LIMIT_HANDOFF.md`.
- **Output stage is OUTPUT ONLY.** `adc_calibration_loop_output.h` learns pulse
  spacing/shape/signs from ADC samples and cancels dither from a separate
  frozen snapshot after a run (or after independent HOLD). Its tone fit never
  enters feedback and never changes HOLD scores. Unsupported data passes
  through with an explicit non-cancelled status.
- **Never grade training residuals as independent precision.** Do not claim
  three-axis precision or joint convergence (see Current status).
- **Skew writes only via the sanctioned firmware transaction**
  (`adc -cal skew step +/-N`, `adc -cal loop test park CODE`), trusted only on
  `result=OK`. The raw host delay path is frozen (`AdcClockDelay.allow_writes=False`).
  No automatic recovery writes from host tools.
- **Never commit** Vitis-regenerated files (`firmware/final_ver_1/`, the
  original app's `app.yaml`/`vitis-comp.json`, `firmware/thesis_v3_loop_appl/src/app.yaml`),
  ELF/bitstreams/build outputs, or large captures. Stage files explicitly.
- Do not hand-edit Vitis-generated files (`src/CMakeLists.txt`, `_ide/`,
  `vitis-comp.json`, `compile_commands.json`, `lscript.ld`). The user-editable
  `src/UserConfig.cmake` includes `cmake/vitis_compat.cmake`.
- Two correction conventions exist (`calibration.c`: `raw*gain + offset`;
  `butils_calibration.c`: `round(gain*(raw + offset))`). Do not unify casually.
- Personal branches (e.g. `hans`) merge into `main` by PR.

## Project

UofT TI-ADC calibration research: correct offset, gain and timing skew between
AD9695 channels A/B using an injected impulse dither and a feedback loop.
Bench: ZCU102 (A53 bare metal, Vitis), AD9695 (JESD204B), AD9164 DAC via DPG.

## Layout

| Path | Contents |
|---|---|
| `firmware/thesis_v3_loop_appl/` | Current Vitis app: original firmware + `adc -cal loop` running `calibration_loop_c` on the A53. Local: `butils.c`, `butils_loop*.c`, `loop_waveform.h`. See its README. |
| `firmware/thesis_v3_500mhz_appl/` | Original firmware (most loop-app sources reference it): UART console, DMA, lwIP UDP, AD9695 SPI, five-stage pipeline. |
| `firmware/final_ver_1/`, `final_ver_1.0.xsa` | Vitis platform and hardware spec. |
| `calibration_loop_c/` | BSP-free C loop (de-framing, blind estimator, LMS, batch skew decision, output stage). Compiled into the sim and the firmware. |
| `calibration_sim/` | Host C simulator/test harness (CMake). **Primary test suite.** Also `adc_signal_replay`. |
| `calibration_loop/` | Python loop package (`python -m calibration_loop.run_calibration`), reference for the C port. |
| `tools/` | Bench/host scripts by group (index `tools/README.md`): `board_loop/`, `skew/`, `dither/`, `frames/`, `console/`. Run from repo root. |
| `checks/` | Offline proofs and figure scripts (`python checks/<x>.py`), read data from `calibration_out/`. |
| `calibration_out/` | Run outputs only, one folder per run (`<name>/<name>.log/.csv/.json/.png/.svg/.html`, plus `_waveform.*`, `_signal.*`). |
| `docs/` | `bench/` procedures (`CALIBRATION_STARTUP.md`, `RF_COMBINER_MANUAL_TEST.md`, `COMBINER_REJECTION_DIAGNOSIS.md`, `BENCH_SESSION_DITHER_ONLY.md`), `reports/`, `TIADC_definitions.md`, `AGENTS_HISTORY.md`. |
| `rtl/` | AXI Lite/Full wrapper Verilog + SV testbenches (Vivado xsim). |
| `waveforms/` | DPG vectors (one int per line + `.json` config). Board default wave 3 = `dither_only/w06adc_e04_t04_amp32767_dither_only.txt`. |
| `archive/` | Superseded material (`archive/README.md`). |

## Commands

Simulator (exit 0 only if all pass):

```sh
cd calibration_sim && cmake -S . -B build && cmake --build build
./build/adc_cal_sim --run-all          # also: --run-loop-tests, --run-unit-tests, --stress-seeds N
./build/adc_cal_sim --run-joint        # joint 3-axis matrix only, writes joint_trace.csv
python checks/plot_joint_sim.py [--corner N] [--no-run] [--show]   # build+run+plot it
./build/adc_signal_replay INPUT.csv OUTPUT.csv RATE_HZ
```

Python (deps: `numpy matplotlib pyserial`):

```sh
python -m calibration_loop.run_calibration sim --iterations 60
python checks/test_dither_only.py
python tools/board_loop/test_loop_result_export.py   # also test_board_axis_test.py, test_board_axis_suite.py,
                                                     # test_board_signal_output.py, test_loop_viewer.py, test_signal_replay.py
```

Bench (console disconnected from the port first; COM port is per machine):

```powershell
python tools/board_loop/board_loop_log.py --uart COMx --out calibration_out/<name>.log [--park-after 24] [--waveform-frames N] [--no-signal] [--no-viewer]
python tools/board_loop/board_axis_test.py suite --uart COMx --execute   # plan-only without --execute
python tools/board_loop/board_signal_output.py --uart COMx ...            # download frozen output snapshot
```

Single-command calibration: load the wave 3 TXT, bring up, `adc -cal skew step 0`
(expect `neutral_initialized=YES`, code 24), then `adc -cal loop 300`.
Procedures: `docs/bench/CALIBRATION_STARTUP.md`, `calibration_loop/CALIBRATION_LIMIT_TEST.md`.

## Testing rules

1. Shared estimator/pipeline changes: keep `adc_cal_sim --run-all` green.
2. `calibration_loop_c/` changes: keep `--run-loop-tests` green and a fresh
   `python calibration_loop_c/export_golden.py` replay (`--loop-golden`) at 0 failures.
3. Skew decision changes: `python checks/loop_direction_check.py`. Gain loop:
   `checks/_gain_loop_test.py`. Tone-free routes: `checks/_tone_free_loop_test.py`
   (only check that catches a wrong-sign timing route). Ladders: `checks/_dither_ladder_test.py`.
4. Board command/SPI behaviour: `calibration_sim/sim_board_axis_tests.c` spies.
5. New sim scenario: add to `k_scenarios` in `sim_tests.c` and
   `sim_signal_configure_scenario()`, deterministic via `random_seed`.
6. Keep `KNOWN_GAP` rows as gaps; performance tests use monotonic relations.
7. Hardware behaviour is validated on the bench only; the sim does not cover
   JESD, DMA state, caches, SPI or analog noise.

## Architecture facts that bite

- Shared C estimators are compiled into both firmware and sim: no Xilinx headers;
  board glue stays in `butils*.c`.
- DMA frame: `w0..w3` = A, `w4..w7` = B, signed 14-bit left-aligned in `int16_t`.
  Channels are parallel (same instants), not interleaved.
- De-framing picks the split by `|corr(A,B)|` plus the half-group lag test.
  Never re-split at `rot + 4` to "swap" channels.
- Every sample-clock delay write resets the JESD link; captures inside that
  window are stale. Initialise the actuator to neutral after every bring-up
  (after a reload `loop open` reports `code=-1`).
- Skew is a batch decision (20 frames, 10 ps deadband, one code per move) with a
  direction guard that latches skew off on a clearly reversed move (> 3x combined
  batch SE) or a failed ACK. ps/code is non-uniform (~7 near neutral, ~19 at
  the working point): re-measure at the code used.
- Timing sign is the sampling-instant convention (positive = B later); check any
  new timing route against a known delay.
- Networking: board `192.168.1.10`, host NIC static `192.168.1.100` (link-local
  `169.254.x.x` = every capture fails `no UDP frame`). No auth; isolated network only.

## Current status (2026-11-19, details in history)

- Output revision 2 (separate core <= period/3 and ringing support <= 3*period/4
  gates) cancelled dither on one fresh board frame (~22 dB A/B). Not tone
  metrology, not three-axis precision.
- Skew direction guard 3x-SE rule: offline only, not bench-validated.
- Axis suite: skew points 16..32 passed strict HOLD (one repeat each); offset
  47/65 and gain 0/7 at unchanged limits. Frozen offset bias unresolved.
- Joint run (`loop test run all`, `suite --joint`): sim corners converge; skew
  (~20 frames/code) is the bottleneck. Bench 2026-11-25, one repeat each:
  offset+32/IFC 0/code 32 offset+skew PASS, gain FAIL on SE (HOLD 100);
  offset+16/IFC 14 all PASS (HOLD 400). No multi-corner or repeat statistics yet.

## Gotchas

- Opening the project in Vitis 2025.2 regenerates `final_ver_1/` (~1900 files)
  and rewrites app paths; never commit those (local clone may use `--skip-worktree`).
- A wrong DPG vector is not rejected: check `LOOP-START` waveform fields and margin.
- First UDP transfer after boot often times out; retry first.
- `fs_dac/fs_adc` must be an integer (bench 2600/1300).
- `xemacif_input()` runs only after a console line; host tools send a bare newline.
- Replay fixtures in `firmware/thesis_v3_500mhz_appl/adc_data/` are noise-dominated;
  use `BenchModel` or live frames for de-framing questions.
- `tools/skew/skew_park.py` verifies with the default `DitherConfig`; with `w06`
  use `--verify-frames 0`.

## Style

- C11, no extensions; sim builds warning-clean with `-Wall -Wextra -Wpedantic -Wconversion -Wshadow`.
- Many files are CRLF; match the file's line endings.
- Python: stdlib-style, `from __future__ import annotations`, self-validating config dataclasses.
- Docs: terse, factual engineering English. Keep READMEs and this file in sync.
