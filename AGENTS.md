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
| `calibration_loop/` | Python impulse-dither calibration package (`python -m calibration_loop.run_calibration`). |
| `calibration_out/` | Example output of `calibration_loop` runs (CSV, JSON, plots). |
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
  current clocking configuration (needs ~20 cm external coax delay).
- **Skew is measurement-only** on the firmware path: `adc -cal skew` performs
  no register writes. `adc -cal skew step +/-N` is reserved and must stay
  unimplemented until the actuator register semantics are verified. The RTL
  skew actuator (`fpga/skew_actuator/`) is a separate, not-yet-integrated
  effort (Q8 fractional delay, AXI GPIO controlled, reset code 256).
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
   recovered parameters against ground truth.
6. Hardware behavior is validated by `BOARD_TEST_PLAN.md` stages on the bench,
   not by the host simulator — the simulator explicitly does not cover JESD,
   DMA hardware state, cache coherency, SPI registers, or analog noise.

## Known Issues and Gotchas

- `ad9695_adc_super_fine_delay()` in `ad9695_api.c` writes to
  `AD9695_CLK_FINE_DELAY_REG` (0x0112) instead of
  `AD9695_CLK_SUPER_FINE_DELAY_REG` (0x0111); the 0.25 ps field is never
  programmed. `calibration_loop` defaults to the 1.725 ps fine step because of
  this (`--super-fine` only after the firmware fix).
- `xemacif_input()` runs after the blocking `uart_get_line()` in `main.c`, so
  incoming UDP packets are not serviced until the next console line;
  `calibration_loop/capture.py` works around this by sending a bare newline.
- The dither scheme requires `fs_dac / fs_adc` to be an **integer** (bench
  default: DAC 2000 MS/s, ADC 500 MS/s, ratio 4). Non-integer ratios smear the
  averaged pulse replica and bias the gain estimate.
- First UDP transfer after boot often times out — retry before debugging.
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
