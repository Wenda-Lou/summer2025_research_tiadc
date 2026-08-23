# Evidence Manifest (2026-08-20)

This manifest maps the current publication-relevant evidence to concrete files in
the repository. It is intended as the frozen baseline before the mid-September
decision meeting.

## Core numbers (from committed docs)

| Item | Value | Source |
|---|---|---|
| Tone main-path skew convergence | **<1 ps** (best reported run `+0.166 ps` / `0.34 ps` std; confirmed doc values include `−0.87 ps`, `+0.25 ps`) | `DITHER_CONCLUSION.md`, `PLAN_DITHER_FIX.md`, `archive/legacy_reports/TIADC_IMPULSE_DITHER_REPORT_EN.md` |
| Stage 5 parallel average | **39.5 dB / 6.27 bits** | `DITHER_CONCLUSION.md` |
| Dither event detection | **10/10** | `DITHER_CONCLUSION.md` |
| Offset separation | **weak PASS** (3–4 codes deviation, run-to-run unstable) | `DITHER_CONCLUSION.md` |
| Gain calibration factor | **not constant** (0.24–0.44; flat ≈ dither 0.249 vs 0.235) | `DITHER_CONCLUSION.md` |
| Dither fine-skew | **INVALID** (advisory; 14 methods rejected) | `DITHER_CONCLUSION.md`, `PLAN_DITHER_FIX.md` |
| Pulse broadening | injected 32–48 samples → measured 74–123 samples | `DITHER_CONCLUSION.md`, `PLAN_DITHER_FIX.md` |
| Coded-sequence simulation | Golay skew ~0.35 ps / PRBS ~2.1 ps (20 frames) | `TIADC_IMPULSE_DITHER_SUMMARY_EN.md` |

## Key documents

- `DITHER_CONCLUSION.md` — final board-level conclusions and paper narrative
- `PLAN_DITHER_FIX.md` — investigation log, rejected methods, mechanism evidence
- `TIADC_IMPULSE_DITHER_SUMMARY_EN.md` — English research update
- `TIADC_IMPULSE_DITHER_SUMMARY_CN_REF.md` — Chinese reference version
- `TIADC_IMPULSE_DITHER_SUMMARY_EN.pdf`
- `TIADC TEAM REPORT CN.pdf`
- `README.md`, `AGENTS.md` — project overview and repository guidance

## Board-run data

- `test_platform/thesis_v3_500mhz_appl/adc_data/calibration_exports/`
  - 13 board runs (`calibration_run_20260816_*`, `calibration_run_20260819_*`)
  - Each contains stage CSVs: timing/offset/gain/skew/performance captures + iterations
- `test_platform/thesis_v3_500mhz_appl/adc_data/adc_capture_*.csv`
  - Replay fixtures used by `calibration_sim`

## Simulator baseline

- Source: `calibration_sim/`
- Tracked root `output/` contains a simulator regression snapshot
  (`test_summary.txt`, `unit_test_results.csv`, `calibration_iterations.csv`,
  `performance.csv`, `stress_summary.csv`)
- Regenerable ignored outputs under `calibration_sim/output/` are not part of
  the frozen manifest

## Plots / publication figures

- `calibration_out/*.png`
  - gain/offset convergence, stage-4 skew, stage-5 performance, stage-5
    polarity correction, timing alignment correlation
- Generation scripts in `calibration_out/*.py`

## Waveform

- `waveforms/impulse_dither.txt`
- `waveforms/impulse_dither.json`

## Legacy / cleanup

- `archive/README.md` — cleanup record
- `archive/legacy_reports/` — superseded report versions preserved from git
