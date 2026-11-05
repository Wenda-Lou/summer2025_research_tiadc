# Archive

This directory holds legacy/superseded materials and the cleanup record for the
2026-08-20 repository tidy-up.

## Contents

- `legacy_host/` (moved here 2026-11-04)
  - `py_UDP_interface/`, `py_to_matlab_module/`, `lwip_platform/`: the 2025
    host/platform experiments (UDP capture script, MATLAB bridge, standalone
    lwIP Vitis app). Not used by the current firmware, `calibration_loop` or
    `tools/`.
- `legacy_reports/`
  - Older report versions that were removed from the repository root. They are
    preserved here for reference and are superseded by:
    - `TIADC_IMPULSE_DITHER_SUMMARY_EN.md`
    - `TIADC_IMPULSE_DITHER_SUMMARY_CN_REF.md`
    - `TIADC_IMPULSE_DITHER_SUMMARY_EN.pdf`
    - `TIADC TEAM REPORT CN.pdf`

## Cleanup record (2026-08-20)

Deleted generated/temporary items (regenerable or no longer needed):

- `_chrome_tmp/`, `_chrome_tmp2/` — browser temporary profiles
- `_pylibs/`, `.agents/`, `.continue/` — empty tool/scratch directories
- `calibration_loop/__pycache__/` — Python cache

**Correction (user instruction): Vitis files must NOT be deleted.**
The following Vitis-related paths were deleted before this instruction and are
regenerable from `test_platform/final_ver_1.0.xsa` / Vitis sources:

- `test_platform/_ide/` — Vitis IDE generated
- `test_platform/final_ver_1/export/` — Vitis platform export
- `test_platform/final_ver_1/logs/` — Vitis log dir
- `test_platform/final_ver_1/zynqmp_fsbl/build/` — FSBL build output
- `test_platform/thesis_v3_500mhz_appl/build/` — firmware build output

Do not delete these in future. If they are needed again, regenerate them in
Vitis from `test_platform/final_ver_1.0.xsa`.

Still present because the files were locked at cleanup time (retry later):

- `calibration_sim/output/` — simulator output (ignored, regenerable)
- `calibration_sim/adc_cal_sim.exe` — simulator executable (ignored, regenerable)

## Note

Do not delete `test_platform/thesis_v3_500mhz_appl/adc_data/calibration_exports/`.
Those directories are the board-run evidence used by the September publication
decision.

## Cleanup record (2026-11-04)

Moves only (`git mv`, history kept); nothing deleted except the empty
`test_platform/` (it held only a `.gitignore`; the Vitis tree it once held now
lives in `firmware/`, so the `test_platform/...` paths above refer to the old
location).

- Root documents -> `docs/`: `CALIBRATION_STARTUP.md` and
  `BENCH_SESSION_DITHER_ONLY.md` -> `docs/bench/`;
  `TIADC_CALIBRATION_LOOP_UPDATE_EN.md` and `next_steps.md`
  (-> `next_steps_2026-09-21.md`) -> `docs/reports/`; `TIADC_defenition.md`
  -> `docs/TIADC_definitions.md`; `frequency sweep.xlsx` ->
  `docs/data/frequency_sweep.xlsx`.
- `axi_lite_wrapper/`, `axi_full_wrapper/` -> `rtl/`.
- `py_UDP_interface/`, `py_to_matlab_module/`, `lwip_platform/` ->
  `archive/legacy_host/`.

## Cleanup record (2026-11-05)

Moves only (`git mv`). `calibration_out/` now holds run outputs only:

- `calibration_out/_*.py` (18 offline checks and figure scripts) -> `checks/`.
  They locate the repo as their parent's parent, so their data paths are
  unchanged; run them from the repo root (`python checks/_gain_loop_test.py`).
- Loose files regrouped: `closed_loop_run1.txt`, `closed_loop_run2_nocancel.txt`,
  `stability_after_run1.txt` -> `closed_loop/`; `frozen_run_regression_296*` ->
  `lockup_fix_backup/`; `dither_vs_tone*.csv` -> `dither_vs_tone_frames/`;
  `dither_saved.*` -> `dither_saved/`; `offset_p16_r1.*`, `offset_p16_r2.*` ->
  `offset_p16_r1/`, `offset_p16_r2/` (the per-run folder layout
  `board_loop_log.py` now writes); `gain_skew_20261028_212902.runner.*` -> that
  sweep's folder; `adc_data.zip`, `probe_probe.png`, `run.csv`,
  `run_learning.png`, `run_meta.json` (old default-name outputs) -> `misc/`.

### Phase 3 (2026-11-05): `tools/` grouped

`tools/*.py` moved into `tools/board_loop/`, `skew/`, `dither/`, `frames/` and
`console/` (see `tools/README.md`); `loop_direction_check.py` and
`test_dither_only.py` moved to `checks/`. Scripts that set `REPO` now look three
levels up; `checks/` scripts that import dither tools add `tools/dither` to
`sys.path`. Every `tools/<name>.py` reference in docs and code was rewritten.
