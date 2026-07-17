# Python Fixes

The reference uploader keeps the existing REFB/REFD/REFE/REFC packet
structure, but its FPGA reception is **not claimed as verified after this
code revision**.

To re-verify it:

1. Run `send_known_test_sequence()`.
2. Confirm on the FPGA UART:
   - 2032 samples received
   - first sample = 0
   - last sample = 2031
   - reference ready
3. Only then mark the revised uploader as end-to-end verified.

Other changes:

- Timing GUI no longer requires a DAC TXT and no longer has overlapping rows.
- Plotting reuses the shared ADC reconstruction function.
- `receive_adc_frame()` was added for offline validation.
- Dead reference-comparison fields were removed from the IFC sweep.
- The PC calibration loop is explicitly disabled as a production path;
  the old scaffold is retained as `offline_calibration_test.py`.
