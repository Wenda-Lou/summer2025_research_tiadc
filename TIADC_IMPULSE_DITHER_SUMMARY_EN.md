# TI ADC Digital Impulse Dither Calibration — Research Update

**Key takeaway:** The original impulse-dither estimator is limited by measured DAC-to-ADC dispersion, while coded sequences—especially Golay with a dedicated estimator—show promising fine-skew performance in simulation at the current operating point.

## 1. Board-Level Findings

- The tone-based main calibration path is stable and accurate: skew converges to **<1 ps**, Stage 5 parallel-average **39.5 dB / 6.27 bits**.
- Dither event detection is reliable (10/10), but:
  - offset separation is weak (3–4 codes, unstable across runs);
  - gain does not show a constant calibration factor (0.24–0.44);
  - the current fine-skew estimator is **not reliable on the board** (edge disagreement 100–1000 ps vs 23.1 ps).
- The key limitation is link dispersion: injected 32–48 sample pulses arrive as **74–123 sample-wide pulses**; a sufficiently clean flat top is not preserved through the current DAC-to-ADC signal chain.

## 2. Simulation Findings

We tested four dither structures (no-flat-top triangular, flat-top linear, PRBS, Golay) in ideal and simulated-board (dispersion + noise) modes.

- With the current production estimators, pulse-type dithers keep 100% event detection but fine-skew drops to **~1% under dispersion**; PRBS/Golay are not recognized.
- A dedicated correlation/least-squares estimator was prototyped:
  - at the current operating point, Golay recovers offset within **~0.3 codes** and gain within **~2%**; PRBS offset on channel B remains biased (~6–7 codes);
  - fine-skew with multi-frame averaging: **Golay mean error ~0.35 ps** (20 frames), PRBS ~2.1 ps;
  - at higher sample rate/frequency (e.g., 2.7 GSPS / 1200 MHz), tone fitting remains accurate, but the code-based estimator degrades (Golay skew ~4 ps, gain ~3.6%).

## 3. Conclusions

- A sufficiently clean flat top is not preserved through the current DAC-to-ADC signal chain; removing it alone does not fix fine-skew because the bottleneck is **dispersion/bandwidth**.
- Among the structures tested so far, **coded sequences are the most promising direction** for dither-based fine-skew estimation; **Golay currently gives the best overall simulated performance**.

## 4. Current Thoughts

Golay currently appears more promising, with a simulated multi-frame skew error of **~0.35 ps** versus **~2.1 ps** for PRBS at the present operating point.
