# tools/

Manual mismatch tests keep the existing `loop test run AXIS PRE TRAIN HOLD`
workflow. `board_loop_log.py` now reads an advertised final waveform snapshot
after those phases, saves `<stem>_signal.csv/json`, and embeds it with the phase
curves in the same HTML. `--no-signal` skips this read-only download. HOLD
assessment and cancellation status remain separate. See the current procedure
at the top of `calibration_loop/CALIBRATION_LIMIT_TEST.md`.

After a completed closed calibration run, download the final frozen signal
snapshot with `board_loop/board_signal_output.py --uart COM7 --out calibration_out/signal.csv`
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
`python tools/board_loop/board_loop_log.py --uart COM7 --out calibration_out/run.log`.
Scripts that do not set up their own `REPO` path need `PYTHONPATH=.` to import
`calibration_loop`. Offline proofs that never touch the bench live in `checks/`.

| Folder | What it is for | Main entry points |
|---|---|---|
| `board_loop/` | Run the on-board loop (`thesis_v3_loop_appl`) over UART and export every record: CSV, JSON, PNG/SVG and the offline HTML viewer. | `board_loop_log.py` (one calibration run, `--park-after CODE`, final ADC waveform via `final_waveform.py`), `board_axis_test.py` (single-axis limit tests), `run_gain_skew_tests.py`, `loop_viewer_export.py`; tests `test_loop_result_export.py`, `test_board_axis_test.py` |
| `skew/` | Drive the AD9695 skew actuator through the ACK-checked firmware transaction. | `skew_park.py` (return to a code), `skew_step_characterize.py` (ps per code), `skew_close_loop.py`, `skew_stability_check.py` |
| `dither/` | Study the dither excitation from captured frames: raw evidence, amplitude ladder, timing routes, tone level. | `dither_raw_evidence.py`, `dither_ladder.py`, `timing_route_check.py`, `dither_vs_tone_gain.py`, `tone_level.py` |
| `frames/` | Diagnose DMA frames and de-framing: packing, channel order, rotation, reconstruction. | `prepare_capture_truth_check.py`, `live_before_after.py`, `raw_frame_probe.py`, `deframe_rotation_table.py` |
| `console/` | Find the board console port and send it single commands. | `find_console.py`, `console_cmd.py`, `classify_port.py`, `udp_path_diag.py` |

Scripts import their neighbours from the same folder; keep a script next to the
modules it imports when moving it.
