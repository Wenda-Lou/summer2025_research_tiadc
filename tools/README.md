# tools/

Automate all three mismatch tests after the usual hardware bring-up:

```powershell
python tools/board_loop/board_axis_test.py suite --uart COM5 --execute
```

Select remaining tests with `--axes gain,skew --gain-points=0`; custom absolute
skew points use `--skew-points=0,8,16,24,32`. For larger skew excursions use
`--train 600`: the default batch of 20 accepted frames permits at most 15
single-code moves in 300 attempts. Updated firmware supports the explicit
experiment `--acquire-phase-mrad 750` (default 150): only manual PRE/TRAIN
spectral phase quality is relaxed; baseline/HOLD use 150 mrad and HOLD precision
is unchanged. This needs the updated ELF. The final board sweep passed skew
starts 16,20,24,28,32 at strict HOLD, one repeat each; see
`docs/bench/COMBINER_REJECTION_DIAGNOSIS.md` for the limited measured scope.

The suite checks the current gain-freeze firmware, sets the experiment config,
builds one valid baseline, then scans offset +32 down to -32 in steps of one,
all IFC full-scale settings from 1.36 to 2.04 Vpp (codes 10,11,12,13,14,15,0),
and absolute skew hardware codes 0 through 32 in steps of one (33 points).
Each point restores first and uses independent PRE/TRAIN/HOLD
40/300/100. Successful completion restores the baseline hardware/injection
state. Without `--execute` it only prints the plan. `--reuse-baseline` validates
and reuses an existing baseline without config or baseline-training writes.
Defaults are one repeat and channel B; `--repeats 5`, `--offset-points=-32,32`,
`--gain-points=0x0D,0x0E` and `--skew-points=0,16,32` customize the test matrix.
IFC points are register codes; suite skew points are absolute hardware control
codes, not ps or baseline offsets. The default round has 105 points: 65 offset,
7 IFC and 33 skew, with no unreachable targets. A point equal to the baseline
code is labelled as a control. Metadata records `skew_point_mode=absolute`,
`target_code` and `relative_code_offset` separately. Legacy single-axis skew
`sweep --points` remains relative. To explicitly request relative suite points,
use `--skew-mode relative --skew-points=-1,1`; unreachable relative targets are
SKIP in `skipped.csv`/`meta.json`, with no actuator command or fabricated score.

The suite's gain acquisition gate defaults to 600000 ppm to accommodate the
full IFC span. HOLD limits remain offset 0.5 codes, gain 0.1%, skew 10 ps with
the existing confidence margin. `--reuse-baseline` still requires matching
configuration and never rewrites it; a prior 200000-ppm baseline can be reused
with `--gain-gate-ppm 200000`, with capture-range failures recorded as such.

Results go to a new `calibration_out/axis_suite_<timestamp>/` directory:
baseline files, per-axis/per-point CSV/JSON/PNG/SVG/HTML, saved signal CSV/JSON,
combined `frames.csv`/`summary.csv`, `meta.json`, `console.log` and `viewer.html`.
The viewer opens once after the suite. `--no-viewer` skips HTML/browser output;
`--no-signal` skips saved-waveform downloads. HOLD grades and waveform
cancellation status are separate. Failed baseline admission stops before
injection; a normal failed HOLD or a complete acquisition phase rejected only
by gain/skew range or quality gates is saved as FAIL without retraining and
the next point continues. Hardware faults, clipping, timeouts or waveform-transfer
errors stop the sequence with completed/partial data retained and no automatic
recovery writes. Keep tone/dither sources and cabling fixed during the suite.
Offline suite protocol checks: `python tools/board_loop/test_board_axis_suite.py`.
The host automation needs no additional firmware build beyond the current
gain-freeze ELF.

Manual mismatch tests keep the existing `loop test run AXIS PRE TRAIN HOLD`
workflow. `board_loop_log.py` now reads an advertised final waveform snapshot
after those phases, saves `<stem>_signal.csv/json`, and embeds it with the phase
curves in the same HTML. `--no-signal` skips this read-only download. HOLD
assessment and cancellation status remain separate. See the current procedure
at the top of `calibration_loop/CALIBRATION_LIMIT_TEST.md`.

After a completed closed calibration run, download the final frozen signal
snapshot with `board_loop/board_signal_output.py --uart COM5 --out calibration_out/signal.csv`
(run with Python from the repo root, using the `tools/` prefix).
This reads learned-dither-cancelled A/B samples
using `adc -cal loop output`, not raw UDP. CSV/JSON retain non-cancelled status
when the waveform could not be separated reliably; the tool then exits 1.
It also creates/opens a self-contained `.html` in the existing loop viewer
(`--no-viewer` skips this). The waveform page shows A/B time traces with a
movable sample window, A-B and a Hann-windowed output spectrum. Cancellation
status comes from the matching JSON; a CSV alone is explicitly unconfirmed.
To view an existing snapshot: `python tools/board_loop/loop_viewer_export.py
calibration_out/signal.csv --open`. The file/folder picker also accepts the
signal CSV and its same-stem JSON alongside normal calibration results.

Bench and host scripts. Run them from the repo root, for example
`python tools/board_loop/board_loop_log.py --uart COM5 --out calibration_out/run.log`.
Scripts that do not set up their own `REPO` path need `PYTHONPATH=.` to import
`calibration_loop`. Offline proofs that never touch the bench live in `checks/`.

| Folder | What it is for | Main entry points |
|---|---|---|
| `board_loop/` | Run the on-board loop (`thesis_v3_loop_appl`) over UART and export every record: CSV, JSON, PNG/SVG and the offline HTML viewer. | `board_loop_log.py` (one calibration run, `--park-after CODE`), `board_axis_test.py suite` (automatic offset/gain/skew), `board_axis_test.py sweep/step` (single-axis tests), `run_gain_skew_tests.py`, `loop_viewer_export.py`; tests `test_loop_result_export.py`, `test_board_axis_test.py`, `test_board_axis_suite.py` |
| `skew/` | Drive the AD9695 skew actuator through the ACK-checked firmware transaction. | `skew_park.py` (return to a code), `skew_step_characterize.py` (ps per code), `skew_close_loop.py`, `skew_stability_check.py` |
| `dither/` | Study the dither excitation from captured frames: raw evidence, amplitude ladder, timing routes, tone level. | `dither_raw_evidence.py`, `dither_ladder.py`, `timing_route_check.py`, `dither_vs_tone_gain.py`, `tone_level.py` |
| `frames/` | Diagnose DMA frames and de-framing: packing, channel order, rotation, reconstruction. | `prepare_capture_truth_check.py`, `live_before_after.py`, `raw_frame_probe.py`, `deframe_rotation_table.py` |
| `console/` | Find the board console port and send it single commands. | `find_console.py`, `console_cmd.py`, `classify_port.py`, `udp_path_diag.py` |

Scripts import their neighbours from the same folder; keep a script next to the
modules it imports when moving it.
