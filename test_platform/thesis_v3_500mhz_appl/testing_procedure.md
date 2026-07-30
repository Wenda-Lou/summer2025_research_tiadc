# ADC Calibration Loop Testing Procedure

## Overview

This document describes the recommended testing procedure for the FPGA-based ADC calibration loop. The calibration consists of the following stages:

1. Timing Alignment
2. Offset Calibration
3. Gain Calibration
4. Open-Loop Skew Measurement
5. Performance Measurement

The purpose of these tests is to verify that each stage functions correctly before enabling a complete closed-loop calibration.

---

# Test Environment

## Hardware

- ZCU102
- AD9695 ADC
- AD9164 DAC
- External clock source
- Ethernet connection
- UART connection

## Software

- FPGA firmware
- UART terminal
- DPG Downloader
- Python plotting scripts (optional)

---

# Before Testing

Verify the following:

- AD9695 initializes successfully.
- JESD link is stable.
- DMA is operational.
- ADC sample rate is correct.
- DAC waveform has been downloaded successfully.
- Input tone is present.
- No previous calibration state remains.

Recommended reset sequence:

```
Power cycle FPGA
↓

Download DAC waveform

↓

Reset DMA

↓

Start calibration
```

---

# Stage 1 – Timing Alignment

## Command

```
adc -cal
```

or

```
adc -cal timing
```

depending on the implementation.

---

## Expected Behaviour

The system should:

- capture multiple frames
- align each frame against the uploaded DAC reference
- select the best reference frame
- store timing information
- store the fixed calibration window

The output should include:

- selected channel
- correlation
- lag
- canonical phase
- fixed window
- PASS status

Example:

```
Stage 1/5: Timing Alignment

Frames accepted : 10/10
Mean correlation : 0.995
Stored reference : Frame 5
Status : PASS
```

---

## Acceptance Criteria

Typical values:

Correlation

```
> 0.97
```

Accepted frames

```
100%
```

No rejected frames under normal conditions.

---

## Common Problems

### Low Correlation

Symptoms

```
Correlation decreases
```

Possible causes

- incorrect sample rate
- incorrect DAC frequency
- poor clock quality
- high input frequency
- incorrect uploaded reference
- DMA reconstruction issue

Recommended actions

- verify DAC waveform
- verify ADC sample rate
- rerun timing alignment
- inspect diagnostic output

---

### No Accepted Frames

Possible causes

- incorrect reference
- DMA capture failure
- JESD issue

Check

```
adc -cal diagnose
```

---

# Stage 2 – Offset Calibration

This stage uses the fixed timing reference selected during Stage 1.

---

## Expected Behaviour

The controller should

- capture new frames
- reuse the stored timing
- estimate DC residual
- update software offset
- gradually converge

Typical output

```
Batch 4

Residual mean
Filtered residual
Offset correction
Status
```

---

## Acceptance Criteria

Residual should gradually decrease.

Correction should stabilize.

Residual should fluctuate around zero.

---

## Common Problems

### Offset Does Not Converge

Possible causes

- poor timing alignment
- unstable ADC
- clipping
- incorrect reference

Recommended actions

Repeat timing alignment first.

---

### Oscillating Offset

Possible causes

Controller gain too large.

Recommended action

Reduce controller gain.

---

# Stage 3 – Gain Calibration

The gain stage uses the calibrated offset from Stage 2.

---

## Expected Behaviour

The controller should

- estimate gain mismatch
- update software gain
- reduce gain error
- stabilize

---

## Acceptance Criteria

Gain estimate should converge.

Residual error should decrease.

No gain saturation.

---

## Common Problems

### Gain Saturation

Symptoms

```
Gain reaches limit
```

Possible causes

Incorrect scaling

Incorrect reference amplitude

---

### Gain Diverges

Possible causes

Offset stage incomplete

Reference mismatch

Incorrect timing

---

# Stage 4 – Open-Loop Skew Measurement

Current implementation performs measurement only.

No hardware delay registers are modified.

---

## Expected Behaviour

The system should

- estimate skew
- estimate dither gain
- estimate pulse timing
- report skew values

Example

```
Skew A

Skew B

Relative skew
```

---

## Acceptance Criteria

Results should be repeatable across runs.

Large variation usually indicates poor alignment.

---

## Common Problems

### Large Skew Variation

Possible causes

Poor timing alignment

Insufficient impulse events

Noisy capture

---

### Skew Not Updated

Expected.

Current implementation intentionally performs measurement only.

---

# Stage 5 – Performance Measurement

This stage measures system performance after calibration.

No coefficients are modified.

---

## Expected Behaviour

The system performs multiple fresh captures.

For every capture it computes

- SNDR
- ENOB
- SFDR
- THD
- signal frequency
- spur frequency

A CSV row is generated for every capture.

---

## Acceptance Criteria

Typical observations

- calibrated SNDR higher than raw SNDR
- calibrated ENOB higher than raw ENOB
- stable measurements
- sufficient valid captures

---

## CSV Output

One row per capture.

The format follows the uploaded calibration project's logging design.

The CSV includes

- timing information
- offset
- gain
- skew
- Channel A metrics
- Channel B metrics
- combined metrics
- validity flag
- rejection reason

---

## Common Problems

### No Valid Frames

Possible causes

Timing stage failed

Incorrect reference

DMA capture problems

---

### Raw and Calibrated Results Identical

Possible causes

Calibration coefficients remain near zero

Offset or gain loop did not converge

---

### Performance Measurement Invalid

Possible causes

Insufficient valid captures

Invalid spectral analysis

Poor signal quality

Calibration stages may still be valid.

Performance measurement is independent of calibration success.

---

# Recommended Testing Sequence

```
Power on

↓

Download DAC waveform

↓

Verify clocks

↓

Verify JESD

↓

Verify DMA

↓

Run

adc -cal

↓

Review Timing

↓

Review Offset

↓

Review Gain

↓

Review Skew

↓

Review Performance

↓

Export CSV

↓

Generate SNDR / ENOB plots
```

---

# Debug Checklist

If timing fails

- Verify uploaded DAC reference
- Verify sample rate
- Verify clock source
- Verify DMA capture

If offset fails

- Check timing first
- Verify reference window
- Verify controller gain

If gain fails

- Verify offset convergence
- Verify reference scaling
- Verify clipping

If skew measurements fluctuate

- Verify timing quality
- Verify dither events
- Repeat measurement

If performance measurements fail

- Verify timing stage
- Verify signal quality
- Check spectral diagnostics
- Verify sufficient valid captures

---

# Current Limitations

Current implementation limitations include:

- Skew stage is measurement only (no hardware correction).
- Combined performance metrics currently mirror Channel A because the hardware is operating in parallel-channel mode rather than a true interleaved mode.
- Offset and gain corrections currently use one shared software correction for both channels.
- No automatic SNDR or ENOB PASS/FAIL thresholds are applied.
- Performance measurement is intended for characterization and plotting rather than acceptance testing.

---

# Expected Successful Flow

```
Power On
    │
    ▼
Upload DAC Reference
    │
    ▼
Timing Alignment
    │
    ▼
Offset Calibration
    │
    ▼
Gain Calibration
    │
    ▼
Open-Loop Skew Measurement
    │
    ▼
Performance Measurement
    │
    ▼
Export CSV
    │
    ▼
Generate SNDR / ENOB Curves
```