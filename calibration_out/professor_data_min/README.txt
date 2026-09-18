Raw ADC captures -- AD9695 on ZCU102, 17 September 2026

  adc_raw_neutral_code24.bin     100 frames, 409600 bytes
  adc_raw_converged_code31.bin    40 frames, 163840 bytes

Two states of the timing actuator, nothing else changed: channel B's sample-clock delay at
control code 24 (uncalibrated, skew residual about -44 ps) and at control code 31 (loop
converged, about -9.7 ps).  Both sets are uncorrected raw captures, in the order taken.

FORMAT
  Each frame is a 4095-byte payload stored as a 4096-byte record, so records stay
  word-aligned: the payload is followed by one zero pad byte.

      int16 little-endian words
      every 8 words = 4 samples of channel A, then 4 samples of channel B
      codes = word >> 2         (14-bit value left-aligned in the int16)
      255 complete groups = 1020 samples per channel per frame

  The DMA restart puts the 8-word group boundary at an arbitrary phase in each frame, so
  every frame needs its own rotation: try all 8 and keep the one that maximises
  |corr(A, B)| -- about 1 at the correct phase, and 0.756 for the half-group-shifted
  candidate (that candidate compares the two converters 4 samples apart).  One branch of
  the splitter is inverted, so negate channel B whenever that correlation is negative,
  before the two are compared or subtracted.

      import numpy as np
      x = np.fromfile('adc_raw_neutral_code24.bin', '<i2').reshape(100, 2048)
      g = x[:, :2040].reshape(100, 255, 8)          # [frame, group, 8 words]
      a = (g[:, :, :4].astype(np.int32) >> 2)       # [frame, group, 4 samples]
      b = (g[:, :, 4:].astype(np.int32) >> 2)

ACQUISITION
  ADC 1.300 GS/s, DAC 2.600 GS/s (ratio exactly 2)
  main tone 199.375 MHz
  impulse dither: one pulse every 130 ADC samples, 64 pulses per 8320-sample loop,
  fixed +/- polarity sequence
  channels A and B are parallel (same sampling instant), not 2x interleaved

CHECK VALUE
  A-B difference spur = 20*log10( |tone(A-B)| / |tone(A)| ) at 199.375 MHz, averaged over
  the frames in each file:  neutral_code24  -24.582 +- 0.481 dBc (median -24.488)   converged_code31  -38.170 +- 1.365 dBc (median -37.856)
  i.e. a difference of -13.6 dB between the two states.
