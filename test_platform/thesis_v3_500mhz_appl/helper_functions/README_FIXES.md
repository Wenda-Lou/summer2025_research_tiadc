# Python Fixes

The reference uploader keeps the existing REFB/REFD/REFE/REFC packet
structure, but its FPGA reception is **not claimed as verified after this
code revision**.

The Upload Reference TXT path now treats the selected file as the periodic
signed 16-bit waveform played by the AD9164. It reconstructs 2032 ideal ADC
samples by periodically resampling from 2.4576 GSPS to the 1.3 GSPS rate
configured in `main.c`, then converts full scale to signed 14-bit ADC codes
(`-8192` through `8191`). The starting DAC index selects the initial waveform
phase. This reconstruction does not model the unknown DAC/ADC clock phase,
analog-path delay, bandwidth, gain, or offset; real captures still require
alignment and calibration.

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
