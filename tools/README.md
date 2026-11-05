# tools/

Bench and host scripts. Run them from the repo root, for example
`python tools/board_loop/board_loop_log.py --uart COM7 --out calibration_out/run.log`.
Scripts that do not set up their own `REPO` path need `PYTHONPATH=.` to import
`calibration_loop`. Offline proofs that never touch the bench live in `checks/`.

| Folder | What it is for | Main entry points |
|---|---|---|
| `board_loop/` | Run the on-board loop (`thesis_v3_loop_appl`) over UART and export every record: CSV, JSON, PNG/SVG and the offline HTML viewer. | `board_loop_log.py` (one calibration run, `--park-after CODE`), `board_axis_test.py` (single-axis limit tests), `run_gain_skew_tests.py`, `loop_viewer_export.py`; tests `test_loop_result_export.py`, `test_board_axis_test.py` |
| `skew/` | Drive the AD9695 skew actuator through the ACK-checked firmware transaction. | `skew_park.py` (return to a code), `skew_step_characterize.py` (ps per code), `skew_close_loop.py`, `skew_stability_check.py` |
| `dither/` | Study the dither excitation from captured frames: raw evidence, amplitude ladder, timing routes, tone level. | `dither_raw_evidence.py`, `dither_ladder.py`, `timing_route_check.py`, `dither_vs_tone_gain.py`, `tone_level.py` |
| `frames/` | Diagnose DMA frames and de-framing: packing, channel order, rotation, reconstruction. | `prepare_capture_truth_check.py`, `live_before_after.py`, `raw_frame_probe.py`, `deframe_rotation_table.py` |
| `console/` | Find the board console port and send it single commands. | `find_console.py`, `console_cmd.py`, `classify_port.py`, `udp_path_diag.py` |

Scripts import their neighbours from the same folder; keep a script next to the
modules it imports when moving it.
