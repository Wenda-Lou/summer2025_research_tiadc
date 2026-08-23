# Archive

This directory holds legacy/superseded materials and the cleanup record for the
2026-08-20 repository tidy-up.

## Contents

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
