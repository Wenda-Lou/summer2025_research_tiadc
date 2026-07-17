# Updated ADC Calibration Code

The code now separates timing alignment from calibration error calculation.

1. `frame.py` reconstructs DMA data, aligns every capture to frame 1, and creates a fixed ADC-grid sine reference using the known TXT tone frequency and the phase measured from frame 1.
2. `receive_data.py` performs frame-1 timing alignment. It no longer searches the full DAC TXT for every frame. An optional fixed calibration reference can be supplied to calculate absolute gain/offset/RMSE metrics after alignment.
3. `calibration.py` contains the gain, offset, residual, and coefficient-update equations.
4. `reference_upload.py` only uploads an already prepared ADC-domain reference; it no longer requires an unreliable global TXT start index.
5. `calibration_loop.py` is a scaffold for offset-first, gain-second calibration. The hardware coefficient write is intentionally provided as an `apply_coefficients` callback because the exact FPGA/ARM coefficient command has not yet been defined.

## Timing-only test

```python
from adc_tools.receive_data import receive_timing_captures

receive_timing_captures(frame_count=20)
```

Then run on the FPGA UART:

```text
adc -timing 20
```

## Build the expected reference

```python
from adc_tools.frame import build_adc_grid_reference_from_frame1

reference, metadata = build_adc_grid_reference_from_frame1(
    frame1,
    tone_frequency_hz=350e6,
    adc_sample_rate_hz=1.3e9,
    target_amplitude_codes=None,  # replace with the known target when available
    target_offset_codes=0.0,
)
```

## Important limitation

The current reference construction is intended for the pure-sine test. The TXT provides the tone frequency, while frame 1 supplies phase. When the final TXT contains deterministic dither or a unique pattern, a separate one-time TXT-to-frame-1 synchronization method can be added.
