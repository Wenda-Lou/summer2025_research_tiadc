# Python Fixes

The reference uploader keeps the existing REFB/REFD/REFE/REFC packet
structure, but its FPGA reception is **not claimed as verified after this
code revision**.

The Upload Reference TXT path now sends one raw DAC-rate block to the FPGA.
Firmware reconstructs the ADC-rate EVEN/ODD reference candidates and performs
timing alignment, DAC-referenced analysis, and offset calibration on the
FPGA/ARM side.

To re-verify it:

1. Run `send_known_test_sequence()`.
2. Confirm on the FPGA UART:
   - 2032 samples received
   - first sample = 0
   - last sample = 2031
   - reference ready
3. Only then mark the revised uploader as end-to-end verified.

Current Python scope:

- Plotting reuses the shared ADC reconstruction function.
- Dead reference-comparison fields were removed from the IFC sweep.
- Python receives UDP captures, plots CSVs, runs IFC sweep capture, and uploads
  raw DAC TXT references. It no longer performs timing alignment or calibration.
