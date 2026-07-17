"""Compatibility notice for the old PC-side calibration loop.

The production closed-loop calibration is now intended to run in FPGA/ARM
firmware. Use `offline_calibration_test.py` only for PC-side equation and
data-path validation.
"""

raise RuntimeError(
    "The PC-side calibration loop is disabled. "
    "Use offline_calibration_test.py for validation, or run adc -cal "
    "for the firmware calibration path."
)
