# AGENTS.md

Guidance for AI coding agents working in this repository. Read this before
touching anything.

## Project Overview

This is a research repository (University of Toronto, summer 2025) for the
**Time-Interleaved (TI) ADC calibration project**. The goal is a hardware
calibration algorithm that corrects timing, gain, offset, and bandwidth
mismatches in high-speed TI ADCs, using a reference **dither signal** injected
into each channel plus a negative-feedback loop.

The target bench is:

- **ZCU102** (Zynq UltraScale+ MPSoC, ARM Cortex-A53 PS + PL fabric)
- **AD9695** dual ADC (JESD204B interface)
- **AD9164** DAC driven by a DPG pattern generator (provides the reference tone
  plus impulse dither)

The repository mixes four kinds of code that support that bench:

1. **Firmware** (`test_platform/thesis_v3_500mhz_appl/`) — bare-metal C on the
   Zynq PS, built with Vitis. Runs the five-stage calibration pipeline, UART
   command console, lwIP UDP data offload, AXI DMA capture, SPI register
   control of the AD9695.
2. **Desktop C simulator/test harness** (`calibration_sim/`) — compiles the
   production firmware estimator modules on a host PC (Linux/WSL) with CMake
   and runs unit tests, scenarios, pipeline integration tests, and stress
   seeds. **This is the primary test suite.**
3. **Python calibration loop** (`calibration_loop/`) — a self-contained
   impulse-dither closed-loop calibration package that drives the bench over
   UART/UDP from a host PC (generate DPG vectors, simulate, probe, close the
   loop). Runs against the existing firmware unchanged.
4. **RTL** (`fpga/skew_actuator/`, `axi_lite_wrapper/`, `axi_full_wrapper/`) —
   Verilog modules with SystemVerilog testbenches and Vivado TCL integration
   scripts.

## Repository Layout

| Path | Contents |
|---|---|
| `test_platform/thesis_v3_500mhz_appl/` | Main Vitis firmware application (production C code). See module list below. |
| `test_platform/final_ver_1/` | Vitis platform project (FSBL, hw handoff). |
| `test_platform/final_ver_1.0.xsa` | Exported hardware specification for Vitis. |
| `calibration_sim/` | Host C simulator + test harness (CMake). Reuses production firmware sources. |
| `calibration_loop/` | Python impulse-dither calibration package (`python -m calibration_loop.run_calibration`). `fix_skew_export.py` repairs pre-2026-08-16 skew CSV labels from recorded board data (does not invent truncated frames). |
| `calibration_out/` | Example output of `calibration_loop` runs (CSV, JSON, plots). |
| `tools/` | Ad-hoc bench/host diagnostics, run from the repo root with `PYTHONPATH=.` if invoked as a path. `prepare_capture_truth_check.py` and `live_before_after.py` grade de-framing/channel ordering against `BenchModel` truth or live bytes; `half_group_lag_impact.py`, `deframe_rotation_table.py`, `compare_reconstruction.py` quantify extraction errors. `skew_step_characterize.py` measures the actuator's step response, `skew_close_loop.py` converges it with the batch decision, `skew_park.py` returns it to a chosen control code through the ACK-checked firmware transaction, and `loop_direction_check.py` proves the loop's direction guard offline against `BenchModel`; `dither_vs_tone_gain.py` compares the dither and tone routes to the gain mismatch on the same frames (and `--save-frames` archives the captures so a state can be re-analysed without bench time); `dither_raw_evidence.py` reads gain, offset and timing straight off the folded impulse replicas of tone-free captures — folding must be polarity-corrected (the balanced ± train cancels a sign-blind fold) and timing comes from the slope route, readable as a difference between fixed states, not per frame. |
| `fpga/skew_actuator/` | `adc_channel_skew_actuator_gpio.v` fractional-delay AXI-Stream module + Vivado TCL integration scripts + xsim validation logs. |
| `axi_lite_wrapper/`, `axi_full_wrapper/` | AXI Lite / AXI Full master wrapper Verilog with SystemVerilog testbenches (Vivado xsim). |
| `lwip_platform/` | Standalone lwIP Ethernet platform code (`ethernet.c/h`, static IPv4 config) and a minimal Vitis application. |
| `py_UDP_interface/` | Host UDP capture script (`sampling_clk_config_script.py`, 32 KB captures from the board). |
| `py_to_matlab_module/` | MATLAB engine bridge experiment (`matlab_engine.py`). |
| `waveforms/` | Generated DPG DAC vectors (`impulse_dither.txt`, one signed integer per line). |
| `BOARD_TEST_PLAN.md` | On-board validation plan for the firmware calibration flow. |
| `adc_calibration_output_example_log.log` | Example UART log of a full `adc -cal` run. |
| `xelab.pb`, `xvlog.pb`, `xsim.dir/`, `.Xil/`, `_ide/` | Vivado xsim / IDE leftovers — generated, do not edit. |

### Firmware modules (`test_platform/thesis_v3_500mhz_appl/`)

- `main.c` — init UART/SPI/GPIO/DMA/lwIP, AD9695 bring-up, main loop.
- `butils.c` / `butils_calibration.c` — UART command parser and board-side
  calibration orchestration/adapters (Xilinx BSP dependent, **not** compiled
  into the desktop simulator).
- `adc_calibration_pipeline.c` — shared five-stage sequencer (timing → offset
  → gain → open-loop skew → performance). Hardware-independent.
- `adc_calibration_dither.c`, `adc_calibration_skew.c`,
  `adc_calibration_performance.c` — shared estimator modules.
- `calibration.c`, `timing_alignment.c`, `reference_buffer.c`,
  `adc_frame.c` — offset/gain loop, lag/correlation timing alignment,
  reference upload/finalize, DMA frame reconstruction.
- `calibration_pending.h` — invalidation hooks implemented by board code.
- `ad9695*.c/h` — AD9695 SPI API and register map.
- `baxidma`, `bjesdlink`, `bjesdphy`, `peripherals`, `ethernet` — BSP-level
  drivers.
- `helper_functions/` — Python host GUI/CLI (`udp_receiver.py` entry point)
  for receiving UDP captures, plotting CSVs, IFC sweeps, and uploading DAC
  reference TXT files. Uses `pandas`, `tkinter`, `numpy`, `matplotlib`.
- `waveform_generation/generate_dac_waveform.py` — generates AD9164 DPG
  vectors (plain text, one signed 16-bit integer per line, length divisible
  by 256).
- `adc_data/` — recorded DMA captures used as replay fixtures by the simulator.

## Build and Test Commands

### Desktop simulator (primary test suite)

```sh
cd calibration_sim
cmake -S . -B build
cmake --build build
```

Run tests (process exits 0 only if everything passes):

```sh
./build/adc_cal_sim --run-unit-tests          # unit tests of production APIs
./build/adc_cal_sim --run-controller-tests    # synthetic closed-loop controller tests
./build/adc_cal_sim --run-pipeline nominal    # one shared-sequencer integration case
./build/adc_cal_sim --run-all-pipeline-scenarios
./build/adc_cal_sim --run-all                 # everything
./build/adc_cal_sim --stress-seeds 100        # stochastic seeds 1..N, continues on failure
./build/adc_cal_sim --list-scenarios
```

Optional sanitizer build:

```sh
cmake -S . -B build-asan -DADC_CAL_ENABLE_SANITIZERS=ON
cmake --build build-asan
```

Results land in `calibration_sim/output/` (`test_summary.txt`,
`unit_test_results.csv`, `calibration_iterations.csv`, `performance.csv`,
`stress_summary.csv`). Start with `test_summary.txt` when a test fails.
`output_*/` directories at the top of `calibration_sim/` are saved runs from
past debug sessions — do not treat them as source.

### Python calibration loop

No packaging files exist; install dependencies ad hoc:

```bash
pip install numpy matplotlib pyserial
```

```bash
python -m calibration_loop.run_calibration gen --out waveforms   # generate DPG vector
python -m calibration_loop.run_calibration sim --iterations 60   # no-hardware sanity run
python -m calibration_loop.run_calibration probe --uart COM3 --frames 10 --plot
python -m calibration_loop.run_calibration bench --uart COM3 --iterations 300
```

The `sim` mode is the required pre-flight check before any bench work: it must
end with gain ratio near 1.0000, offset mismatch under 0.1 LSB, skew under
1 ps. There is no pytest suite in the Python code.

### Firmware

Built with **Xilinx Vitis** from `test_platform/final_ver_1.0.xsa` (import the
XSA, build the platform + `thesis_v3_500mhz_appl` application). There is no
command-line build checked in; the `src/CMakeLists.txt` is Vitis-generated.
Do not hand-edit Vitis-generated files (`src/`, `_ide/`, `vitis-comp.json`,
`compile_commands.json`, `lscript.ld`).

### RTL

Verilog testbenches (`tb.sv`) run under Vivado xsim. `fpga/skew_actuator/`
also has Vivado TCL scripts (`integrate_skew_actuator.tcl`,
`finalize_and_synthesize_skew_actuator.tcl`) that patch a block design — note
they contain hard-coded absolute paths (`C:/thesis_v3/...`) and must be
adjusted before use.

### Board testing

Follow `BOARD_TEST_PLAN.md` (staged `adc -cal` flow over UART) and
`calibration_loop/BENCH_GUIDE.md` (bench bring-up procedure). These are
manual hardware procedures; do not attempt them without the bench.

## Architecture Notes That Matter

- **Shared estimators**: the same production C files
  (`adc_calibration_pipeline.c`, `adc_calibration_dither.c`,
  `adc_calibration_skew.c`, `adc_calibration_performance.c`,
  `calibration.c`, `timing_alignment.c`, `reference_buffer.c`, `adc_frame.c`)
  are compiled both into the firmware and into `calibration_sim`. When you
  change one of them, you change both targets — keep them free of Xilinx BSP
  dependencies. Board-only glue stays in `butils*.c`.
- **DMA frame layout**: `w0..w3` are Channel A samples, `w4..w7` are Channel B
  samples; samples are signed 14-bit values left-aligned in `int16_t`.
- **Channels are parallel, not interleaved**: A and B sample the same instant.
  Combined metrics alias Channel A. True 2× interleaving is impossible in the
  current clocking configuration (needs roughly 8 cm external coax delay at
  1.300 GSPS).
- **De-framing is decided by physics, not by a stored DC level**: the DMA restart
  puts the `[A A A A B B B B]` boundary at an arbitrary phase, and the two
  candidates `r` and `r + 4` *both* yield spectrally clean channels. Only one of
  them puts the two converters on the same instants, so `deframe()` picks the
  split by `|corr(ch_a, ch_b)|` (~1 for the aligned pair, `cos(2*pi*4*f0)` = 0.756
  for the shifted one) plus an explicit half-group lag test. Never re-split at
  `rot + 4` to "swap" the channels: that split slides the pair four samples apart,
  and `n0` is a single value `estimate_block()` slices **both** channels with, so
  the dither windows of one channel stop lining up with its own impulses.
- **The dither polarity of the bench path is a session constant**, not a per-frame
  measurement: this bench presents the impulses inverted with respect to
  `polarity_sequence()`, so `polarity_anchor()` measures the sign once from
  channel A and `pin_polarity` pins the reported gain sign to it. The anchor
  scales the estimator's polarity *model* (`p[k] -> sign * p[k]`) and never the
  captured data — the DC offset is a measurement in ADC codes, so flipping the
  record would flip the offset and invert the loop's offset feedback.
- **Every ADC sample-clock delay write resets the JESD link.** `ethernet.c`'s
  remote delay handler ends with `jesdlink_reset()`, and the firmware's own skew
  path does the same — `butils_calibration.c` states why: *"Changing either ADC
  sample-clock delay disrupts JESD framing"*, so the receiver link must re-sync
  (`bjesdlink.c` asserts and releases the JESD204C reset bit, ~2 ms plus
  re-alignment).  A capture taken inside that window comes back dead or
  half-synced: `dma -w` times out, `RxBufferPtr` keeps the *previous* frame and
  `udp` ships it, so every metric repeats exactly — a 296-iteration bench run
  "converged" on one frozen frame that way while its skew integrator walked the
  actuator to the rail (the integrator is unclamped).  The firmware also does not
  put the chip into a known actuator state first: it runs a *neutral* setup
  (`mode 0x04`, `0x0112=0x60`, `0x0114=0x60` on both channels) before enabling
  fine-delay mode, and `adc -cal diagnose skewprep MODE` exists to isolate that
  preparation — the host path writes `0x0110`+`0x0112` only, never `0x0114`, and
  performs the mode transition itself.  That raw host path is now **frozen in code**
  (`AdcClockDelay.allow_writes=False`); the sanctioned route is the firmware
  transaction `adc -cal skew step +/-N`, which the host drives through
  `capture.SkewActuator` and believes only via its `SKEW-TXN ... result=OK` ACK.
- **Skew actuation, as validated on the bench (2026-09-17)**: the firmware
  transaction is repeatable — 20+ transactions, every one `result=OK`, no
  `RECOVERY_REQUIRED`, no reboot, and a zero-step request leaves the code
  unchanged.  Measured differential step: **7.4 ps per control code** on a
  single-step characterization taken near neutral (nominal 13.8; ~half, cause
  unconfirmed — either one channel's taps dominate or the tap is ~0.9 ps), but
  **~4.8 ps per code across the whole working range**: two independent traversals
  between the neutral code 24 and the converged code 31 (a closed-loop run walking
  up, a `tools/skew_park.py` walk down) both measured 4.8–4.9 ps/code end to end.
  The step is therefore not uniform; `SkewActuator.HOST_STEP_PS = 7.4` sizes the
  moves from the single-step number, which is harmless because every move is
  clamped to one code, and the direction check's 8 ps tolerance absorbs the 1.5×
  disagreement (run1 never tripped it).  **Initialize to neutral after every
  bring-up** (`adc -cal skew step 0`): doing so cuts the per-frame skew scatter
  from 18.9 ps to **3.1 ps**, six times better, and the first transaction
  legitimately shifts the operating point by ~57 ps.  With that done, the A-B
  difference spur goes from −24.6 dBc to **−38.6 dBc**, and that state is stable:
  over 340 s of read-only measurement it drifted **+0.01 ps/minute** (slope
  uncertainty ±0.20 ps/min) with the spur spread at 0.57 dB, so no periodic
  re-convergence is needed on a minutes timescale.  A full 300-capture closed-loop
  run (300-sample curve in `run1_300_from_log.csv` — that CSV was later overwritten by a
  shorter run reusing the stem and the curve was recovered from its console log;
  243 accepted frames) walked the code
  24 → 31 in seven batch moves — every transaction `result=OK` — and stopped at
  −9.70 ps, holding −8.7 to −9.8 ps over the remaining four batches.  The matched
  no-cancellation control (`run2_nocancel.csv`, parked at code 24 first) reproduced
  that trajectory exactly — same seven moves, same codes, deadband first reached at
  the same iteration 159, −9.44 ± 1.37 ps — while the digital loop degraded only
  mildly (gain residual 2.6 % → 3.1 %, offset per-frame scatter 0.85 → 1.08 LSB, rejected
  frames 57 → 64 of 300).  So the skew curve is not what cancellation is visible in:
  one code per 20-frame batch is quantisation-dominated rather than
  noise-dominated, which is the point of the batch decision.  In the loop,
  skew is a **batch** decision (`LoopOptions.skew_batch_frames`, deadband
  `skew_deadband_ps`) — never per frame, because one code moves more than the
  residual error near convergence.  That logic lives in
  `CalibrationLoop._skew_decision` and is guarded by a direction check
  (`LoopOptions.skew_direction_tol_ps`): after a move the next batch must shift the
  error the way the calibrated step predicts, or skew actuation latches off for the
  rest of the run — a failed ACK latches it too (the digital loop keeps running,
  since it is host-side).  `tools/loop_direction_check.py` proves both outcomes
  offline against the bench model: an honest bench never latches, a lying one
  latches after exactly one move.  Use `tools/skew_park.py` (sanctioned transaction
  path, one code at a time, ACK-required) to put the actuator back at code 24
  before a repeat run — a run that inherits code 31 is not a matched pair with one
  that started from 24.
- **Which A-B spur to quote**: a converged run yields three different numbers and
  only the first is skew-limited.  (1) The raw channel difference, measured
  read-only *without* dither subtraction: **−38.8 ± 0.6 dBc**, matching the
  analytic `20log10|2 sin(pi f dt)|` to 0.7 dB (and to 0.22 dB across run1's 84
  post-convergence frames).  (2) The loop's `raw_difference_dbc` column — the same
  records with the per-frame dither estimate subtracted: the same level (−38.6 dBc)
  at 3–4× the scatter (± 1.9 dB), because the ~2.2 % per-frame gain-estimate wobble
  enters the metric.  (3) The loop's `cal_difference_dbc` column, with the host
  gain/offset correction applied: only **−35.1 dBc** while the gain loop was driven by
  the dither ratio, and that gap is *not* skew.  After the gain-observable fix it is
  **−38.12 dBc**, i.e. the two columns agree to 0.01 dB (see the gain-loop bullet).
  It is the applied gain-correction ratio error, `1 − gain_corr_b/gain_corr_a` =
  1.26 %, which alone is a −38.0 dBc term; power-combining it with the skew term
  predicts −35.4 dBc against −35.1 dBc measured (0.3 dB), and the per-frame model
  tracks the column (corr 0.69).  **Quote (1).**  The disagreement is now measured
  directly rather than inferred (`tools/dither_vs_tone_gain.py`, read-only, both
  routes on the same frames): they are two measurements of one gain mismatch and
  they do not agree.  Over three runs the dither route read 1.010–1.014 (mismatch
  −37 to −40 dBc, per-frame scatter 2.3 %) while the tone route read 1.0004–1.0010
  (**−62 to −67 dBc**, scatter 0.25 %); the paired difference is −1.1 to −1.4 %
  with t = −2.7 to −3.3, which reproduces the 1.26 % and 2.05 % seatings the two
  runs imply — three independent estimates agreeing.  Two consequences.  The tone
  route is ~10× more precise and is the one that matters for a spur at f_in, and
  its native mismatch (~0.08 %) sits far below the skew term — the independent
  reason the raw column is skew-limited.  And the loop equalises the *dither*
  route, i.e. the noisier and ~1 % offset proxy, so it seats the gain correction
  1.1–1.3 % away from the point that would null the tone; that is what caps any
  corrected metric.  That is now fixed by making the tone ratio the controlled
  observable — see the gain-loop bullet below.
- **Gain loop observable (fixed 2026-09-17)**: the gain correction used to integrate
  the **dither** pulse-window ratio, which on the bench left `g_B/g_A` at 2–3 %
  (against 0.04 % in the model) *and* seated the correction ~1.2 % away from the point
  that nulls the tone — capping the corrected A-B spur at −35.1 dBc while the raw one
  reached −38.6 dBc, because the dither ratio and the tone ratio are two estimates of
  one mismatch that disagree by that much.  It now integrates the **tone** ratio
  (`LoopOptions.gain_observable`, default `"tone"`; `--gain-observable dither` restores
  the old behaviour): the coherent main-tone fit is what the f_in spur is made of and
  is ~10× quieter (0.25 % per-frame scatter against 2.3 %).  Consequences to expect:
  `tone_ratio` (logged, and drawn as the solid line in the gain panel) goes to 1.0000,
  while `g_B/g_A` settles at ~1.012 *by design*; `gain_source` in each row records
  which observable was actually used, so a silent fallback cannot look like
  convergence.  Offline proof: `calibration_out/_gain_loop_test.py` — with
  `BenchModel.pulse_gain_a/_b` reproducing the bench's 1.2 % disagreement, the dither
  route leaves the tone 1.18 % out and the corrected spur at −38.0 dBc, the tone route
  nulls it and reaches −46.9 dBc (the model's skew limit, +8.9 dB), and the sim's own
  corrected image spur improves from −66.9 to −75.3 dBc.  Bench-confirmed
  (`calibration_out/closed_loop/run3_gainfix.csv`: 300 qualified samples from 380
  captures, after a board bring-up): the applied gain ratio nulled to 0.076 %, the
  corrected A-B spur went from −35.07 ± 3.66 to **−38.12 ± 1.43 dBc** against a raw
  −38.13 ± 1.41 dBc — the two columns now agree to 0.01 dB, the power balance closes
  to 0.25 dB with the applied-ratio term at −62.4 dBc, and the per-frame model tracks
  the column at corr 0.96 (was 0.69).  The cost of the choice is explicit: the
  correction is now ~1.0002 on both channels, i.e. it nulls f_in and deliberately
  leaves the ~2.1 % *broadband* (dither) mismatch alone — that is the meaning of
  ``gain_observable``, not a defect.
- **Offset loop: verified, not broken (2026-09-17)**: reading a `|dOffset|` row made
  this look like the gain loop's disease — a loop stalled 0.7-1.1 LSB from zero.  It
  is not: that number is the *per-frame scatter* (a mean of absolute values).  The
  signed residual after convergence is **−0.05 to −0.18 codes** in all three runs, so
  the loop does drive the DC mismatch to ~0.1 LSB, and its controlling observable
  (`ChannelEstimate.offset_codes`, the flat-top mean of the polarity-decoupled window
  profile) is not biased the way the gain one was.  On 40 archived bench frames the
  pulse-window route reads **−3.907** codes against **−3.975** (whole-record tone-fit
  DC) and **−3.977** (plain record mean) for the same mismatch: agreement to 0.07
  codes, per-frame correlation +0.92
  (`calibration_out/_offset_routes_test.py`).  Two traps cost time here, both worth
  remembering: **(a)** two logged columns use *opposite* sign conventions —
  `cal_dc_difference_codes` is `mean(A) − mean(B)` while the loop's `dOffset` (and
  `offset_b_codes − offset_a_codes`) is `B − A`, so correlating them as-is yields
  −0.92 where the true agreement is +0.92; **(b)** `|dOffset|` rows and
  `_analyse_run.py`'s "offset mismatch" columns are scatter, never a bias — check the
  sign before calling a loop stalled.  Per-frame offset scatter is ~0.9-1.5 codes
  against a 3.9-code native mismatch, and `mu_offset = 0.35` settles it in ~10
  samples, so this axis needs no observable change.
- **Dither-only excitation (what the impulses do without the tone)**: `gen --amp-dbfs -120`
  gives a waveform with the tone at 0 LSB and the identical pulse train
  (`waveforms/impulse_dither_only.txt`).  Anything running the loop with it **must** pass
  `--gain-observable dither`: the default tone observable becomes a noise integrator and the
  loop collapses (bench model: 1 of 21 captures accepted, offset residual tens of LSB),
  whereas with the dither observable it converges (`gain_ratio` 1.00094, offset ~0).  With
  that set, alignment, offset and gain behave **identically** to the tone+dither case
  (offset scatter +0.670 ± 2.313 against +0.668 ± 2.297 codes; gain +1.001 ± 0.004 both) —
  so the impulses carry those three on their own and are not disturbed by tone interference,
  which in turn means the 1.2 % dither-vs-tone gain disagreement is a property of the pulse
  path rather than of the tone.  **Skew has no usable route without the tone** and not
  because of dispersion: even in the dispersion-free model the impulse train's content at f0
  is too small for a coherent phase fit (+53 ± 556 ps against +0.20 ± 0.62 ps with the tone),
  and the loop's own skew-range gate then rejects ~71 % of captures — use
  `tools/dither_response_test.py`, which measures the estimator directly and saves the raw
  frames, for a dither-only session.

- **Sample accounting in the loop**: `CalibrationLoop.run(N)` collects N *qualified*
  samples.  A capture the acceptance filter rejects is logged with its reason and
  retried, so it never consumes a slot; `plot()` draws qualified rows only, so a
  learning curve can never show the filter's rejects as loop behaviour; and `save()`
  records `samples_qualified` / `captures_attempted` / `captures_rejected` in the meta
  JSON, because the CSV deliberately keeps the rejected rows for forensics.  Measured
  rejection rate is ~20 % (almost all `align_margin` on torn UDP frames), so a
  300-sample run takes ~370 captures, and `LoopOptions.max_consecutive_rejects = 20`
  stops a run whose bench has stopped producing usable data at all.  Offline proof of
  all of it: `calibration_out/_qualified_budget_test.py`; a run saved before this rule
  existed can be redrawn from its qualified rows with
  `calibration_out/_replot_qualified.py <run>.csv`.

- **Skew on the firmware path**: `adc -cal skew` measures only (no register
  writes).  `adc -cal skew step +/-N` **is** implemented — it is the single
  sanctioned write path, routed through `handle_adc_skew_transaction_cmd()` (see
  the actuation note above), and it takes *relative control codes*, never
  per-channel ps or raw register values.  The RTL skew actuator
  (`fpga/skew_actuator/`) remains a separate, not-yet-integrated effort (Q8
  fractional delay, AXI GPIO controlled, reset code 256).
- **Two correction conventions exist**: `calibration.c` documents
  `corrected = raw * gain + offset`; the integrated `butils_calibration.c`
  path uses `final_code = round(gain * (raw + offset))`. The simulator mirrors
  the integrated convention in pipeline captures. Do not "unify" these
  casually.
- **Networking**: static IPv4 — board `192.168.1.10`, host `192.168.1.100`,
  gateway `192.168.1.1`. UDP offload on ports 5002/6666 depending on the
  script. No authentication, no encryption: lab-bench research code, keep it
  on an isolated network segment.
- **UART console commands** (firmware): `adc -cal` full staged flow plus
  `timing`, `diagnose`, `offset`, `gain`, `skew`, `skew diagnose`, `status`,
  `export`, `reset`, `help`; also `dma -r/-w/-d/-c`, `adc -ref`, register
  peek/poke. See `butils.c` help printer for the authoritative list.

## Code Style Guidelines

- **Firmware/simulator C**: C11, no compiler extensions. The simulator builds
  with `-Wall -Wextra -Wpedantic -Wconversion -Wshadow`; keep it warning-clean.
  Hardware-independent modules must not include Xilinx headers.
- Many files use **CRLF** line endings (Windows workflow). Match the existing
  file's line endings when editing.
- **Python**: plain stdlib-style modules, type hints used sparingly
  (`from __future__ import annotations` in newer files). Follow
  `calibration_loop/` module layout — configuration dataclasses validate
  themselves (e.g. `DitherConfig.validate()`).
- **Determinism**: simulator scenarios must be reproducible from
  `random_seed`; prefer seed-controlled changes when adding scenarios.
- Documentation tone is terse, factual engineering English. Keep READMEs and
  this file in sync when you change structure, commands, or conventions.

## Testing Instructions

1. Any change to the shared estimator/pipeline modules **must** keep
   `adc_cal_sim --run-all` green (and ideally the sanitizer build).
2. Adding a simulator scenario: add the name to `k_scenarios` in
   `calibration_sim/sim_tests.c`, add its signal configuration in
   `sim_signal_configure_scenario()`. Keep it deterministic via
   `random_seed`.
3. Boundary tests record hard pass/fail rows plus `KNOWN_GAP` rows for
   robustness issues that require production changes — do not silently
   convert `KNOWN_GAP` into PASS.
4. Performance tests use monotonic relationships (e.g. more noise ⇒ lower
   SNDR), not exact metric numbers.
5. For `calibration_loop`, run `sim` mode before and after changes; it prints
   recovered parameters against ground truth. `sim` exercises only the
   bench-model branch of `CalibrationLoop.step()`, so the decision logic that
   lives on the hardware branch (the batch skew decision and its direction latch)
   has its own offline proof: run `python tools/loop_direction_check.py` after
   touching `_skew_decision`.  Gain-loop changes have one too: `python
   calibration_out/_gain_loop_test.py`, which uses `BenchModel.pulse_gain_a/_b` to
   reproduce the bench's dither-vs-tone disagreement and checks that the chosen
   observable actually nulls the tone.
6. Hardware behavior is validated by `BOARD_TEST_PLAN.md` stages on the bench,
   not by the host simulator — the simulator explicitly does not cover JESD,
   DMA hardware state, cache coherency, SPI registers, or analog noise.

## Known Issues and Gotchas

- `ad9695_adc_super_fine_delay()` was reported to write
  `AD9695_CLK_FINE_DELAY_REG` (0x0112) instead of the super-fine register, so the
  0.25 ps field is never programmed. **Re-check before relying on either claim**:
  in the tracked `firmware/thesis_v3_500mhz_appl/` copy `ad9695_registers.h` maps
  `AD9695_CLK_SUPER_FINE_DELAY_REG` to 0x0111 and the function writes that macro,
  so the mismatch is not present there — it may apply to the tree the board is
  actually built from. `calibration_loop` uses the 1.725 ps fine step by default
  regardless (`--super-fine` is opt-in).
- `xemacif_input()` runs after the blocking `uart_get_line()` in `main.c`, so
  incoming UDP packets are not serviced until the next console line;
  `calibration_loop/capture.py` works around this by sending a bare newline.
- The dither scheme requires `fs_dac / fs_adc` to be an **integer** (bench
  default: DAC 2600 MS/s, ADC 1300 MS/s, ratio 2). Non-integer ratios smear the
  averaged pulse replica and bias the gain estimate.
- First UDP transfer after boot often times out — retry before debugging.
- The committed replay fixtures `test_platform/thesis_v3_500mhz_appl/adc_data/adc_capture_*.csv`
  are noise-dominated and were captured on an older geometry: their tone sits
  where the current `DitherConfig` does not look, so a tone fit reports residual
  RMS of 350–500 codes against a tone amplitude of 2–29. They are **not** a
  usable baseline for de-framing or channel-order questions. Verify that class of
  change against `BenchModel` (`tools/prepare_capture_truth_check.py`) or live
  frames (`tools/live_before_after.py`).
- `python -m calibration_loop ...` hardware modes (`probe`, `bench`) need the
  bench powered and the DPG waveform loaded; use `sim` otherwise.
- Git: active work happens on `main`; the bench guide references the
  `HG_Loop` branch for the loop code history.

## Security Considerations

- Bench-only research code: UART console accepts raw register peek/poke, UDP
  services have no authentication, and IP addresses are hard-coded. Never
  expose the board network to untrusted networks.
- Python host scripts open UDP sockets and serial ports with broad timeouts;
  they trust whatever the board sends (packet sizes are expected constants).
- Do not commit bitstreams, ELF files, or large capture CSVs beyond the
  existing replay fixtures; `.gitignore` already covers `/build`, `/export`,
  logs, `.bin`/`.pdi`.
