# TIADC Calibration Loop Structure and Error Root-Cause Analysis

## 1. Scope and Approach

This report analyses the structure and error sources of the TIADC calibration loop. Three classes of problems are distinguished:

1. **Algorithmic / fundamental limitations**: dither three-way separation, link dispersion, flat-top assumption;
2. **Link / hardware limitations**: DAC-to-ADC analog bandwidth, clock jitter, actuator resolution;
3. **Control / implementation issues**: stage failure propagation, state invalidation, gain self-normalisation, closed-loop rejection logic.

The report draws on board measurements, desktop simulation, and production-flow reproductions on a simulated board.

---

## 2. Calibration Loop Structure

### 2.1 Five-Stage Firmware Pipeline

The firmware calibration flow (`adc -cal`) consists of five serial stages:

| Stage | Main task | Pass condition | Effect of failure |
|---|---|---|---|
| Prepare (implicit) | Reference upload, actuator neutral initialisation, warm-up | reference/readiness checks pass | Timing is marked failed |
| 1 Timing | Reference alignment, cross-correlation, lag/fractional lag, channel selection | alignment quality and correlation thresholds pass | Timing context invalidated; later stages cannot run |
| 2 Offset | Tone DC fit + closed-loop offset correction | offset converges and verifies | Gain input invalidated; later stages cannot run |
| 3 Gain | Tone amplitude fit + closed-loop gain correction + verification | gain converges, verification passes, output valid | Gain input invalidated |
| 4 Skew | Tone phase-difference closed loop + dither advisory check | skew measurement/correction succeeds | Calibration fails |
| 5 Performance | SNDR/SFDR/THD/ENOB, A/B matching | spectrum/matching valid | Marked unavailable; no rollback of earlier corrections |

Key structural properties:

- The five stages are strictly serial; a failing stage blocks later stages;
- An unusable Offset result clears the Gain input;
- Skew requires a usable correction output from the first three stages;
- Performance is the last stage that may fail; it does not roll back earlier corrections.

### 2.2 Host-Side Closed Loop

One host-side loop iteration is:

```
capture → de-frame → apply current correction → estimate residual
        → block-LMS update → push skew to the clock-delay actuator → log
```

Design characteristics:

- **Correction is applied before residual estimation**, so the recorded trajectory is a true closed-loop learning curve;
- **Skew is written back through the AD9695 fine-delay register**;
- Frame-level rejection logic (alignment margin, event count, finiteness, skew/gain ranges);
- Capture retries to prevent bad frames from entering the LMS;
- Depends on UART/UDP capture; network timing affects loop speed.

### 2.3 Estimator Structure

| Branch | Principle | Current status |
|---|---|---|
| Tone primary | 800-sample DC+cos+sin least-squares per frame; A/B phase difference | Stable; <1 ps on the board |
| Dither offset | Polarity-weighted event averaging + flat-top sampling | Weak pass on board; simulation shows the flat-top assumption is not valid |
| Dither gain | Least-squares projection onto the template (full/flat) | No constant calibration factor |
| Dither skew | Polarity-weighted event aggregation → A template → B fit → derivative projection | Not reliable on board; usable only in ideal simulation |
| Joint aggregation | Cross-frame stacking of event windows | Not reliable on board; occasional offline pass |

---

## 3. Error Categories and Root Causes

### 3.1 System / Link-Level Causes

#### (1) DAC-to-ADC link dispersion
- Observation: a 32–48 sample digital pulse arrives as a **74–123 sample-wide** pulse at the ADC (10–90% width);
- Cause: DAC zero-order hold and reconstruction filtering, finite analog-injection bandwidth;
- Effect: the flat top is not preserved, edges are rounded, and event localisation / template projection becomes sensitive.

#### (2) DAC/ADC sample-rate ratio constraint
- The dither must land on a fixed ADC sample phase, so `fs_dac / fs_adc` must be an integer;
- A non-integer ratio smears the averaged pulse replica and directly biases the gain estimate.

#### (3) Tone-fit pollution
- Dither and tone share the same frame; pulse energy enters the tone-fit residual;
- A larger pulse duty cycle increases batch noise (baseline std 2–9 → 23 ps).

#### (4) Jitter and analog bandwidth at high frequency
- Phase noise scales linearly with frequency;
- ADC input bandwidth and aperture uncertainty become more significant at high frequency;
- The algorithm may remain accurate, but the analog front end can become the limit.

### 3.2 Timing-Stage Errors

| Symptom | Cause |
|---|---|
| Timing fails | Reference not uploaded, length/format mismatch, correlation below threshold |
| Lag anomaly | Arbitrary reference start phase, uncertain channel selection, poor fractional-lag condition |
| Everything downstream fails | Timing failure invalidates the timing context, so Offset/Gain/Skew cannot start |
| Repeated frames | Port/firmware mismatch, UDP packet loss, capture not aligned with reference |

### 3.3 Offset-Loop Errors

#### (1) The original estimator relies on a "true flat top"
- It searches the digital template for a region with near-zero derivative and near-peak amplitude;
- That region is defined in the digital domain, not measured at the ADC;
- After dispersion, those samples lie on slopes, so the offset estimate is contaminated by shape and localisation errors.

#### (2) Window DC / slow-varying bias
- A/B window means correlate with polarity (about +1.8/−1.6 codes);
- Rising/falling masks are asymmetric, producing opposite edge biases of ~100–240 ps.

#### (3) Event-centre jumps and insufficient event count
- In-frame phase jumps (44→9) and spacing outliers;
- With few events, averaging noise is high and polarity balance degrades.

#### (4) Controller accuracy
- The offset loop converges, but dither offset deviates 3–4 codes from tone DC;
- This is a weak in-tolerance pass, not evidence that dither independently estimates offset accurately.

> **Correction, 2026-09-17**: the 3–4 code figure is the *channel DC mismatch itself* —
> the quantity the loop is in the middle of correcting — not a disagreement between
> routes.  Measured on 40 archived bench frames (real hardware, converged state), the
> pulse-window route that the loop integrates, the whole-record tone-fit DC and the
> plain record mean all agree on that mismatch: **−3.907 / −3.975 / −3.977 codes**,
> i.e. to 0.07 codes, with per-frame correlation +0.92.  After convergence the signed
> residual mismatch is −0.05 to −0.18 codes, so this axis converges and needs no
> observable change.  Reproduce with `calibration_out/_offset_routes_test.py`; see the
> offset bullet in `AGENTS.md`.  The earlier reading conflated a per-frame |scatter|
> with a bias.

### 3.4 Gain-Loop Errors

#### (1) The production gain loop is self-normalising
- It applies the current correction before measuring, so a constant channel-gain error is invisible to the loop;
- The correction stays at 1.0 and the mismatch appears only in the nominal system gain.

#### (2) Dither-gain template-domain to ADC-code-domain mapping
- Board dither gain is 0.24–0.44 against a true value of 1.0;
- flat ≈ full (about 0.249 vs 0.235);
- Both measure the same dispersion-rounded pulse, and no physical flat top exists;
- There is no constant ratio across runs, so online absolute calibration is not possible.

#### (3) Fit quality and event count
- Dither-gain valid rate is about 2/3;

> **Verified against the firmware exports, 2026-09-17** (`adc_data/calibration_exports/calibration_run_20260917_145113`,
> a run that ends `valid=1`).  The two claims above hold, and the picture is now
> stage-by-stage:
> * **timing/alignment** -- dither works: `dither_valid` 10/10, peak 0.56-0.71.
> * **offset** -- dither is not used at all: the offset stage's CSVs have no dither
>   columns; it estimates from the aligned frame.
> * **gain** -- the dither-only estimate is evaluated on 60 of 90 captures and carries
>   a `FIT_QUALITY` warning on **59** of them; its value sits at **0.39-0.57** (never
>   1.0).  The gain stage still reports `PASS` because it drives the loop from
>   `measured_gain` / `batch_gain` (1.002-1.008), not from dither.
> * **skew** -- the dither branch is rejected on **189 of 190** frames
>   (`dither_skew_valid=0`; per-iteration `dither_valid_frames=0`), because
>   `dither_edge_disagreement_ps` scatters **0.2-592 ps** against the 23 ps
>   (`0.03` sample) gate.  `adc_calibration_skew.c:260` makes the dither branch
>   conditional on `dither_valid`, so the skew loop that converged
>   (mean skew -86.2 -> -38.7 -> -26.5 -> -13.0 -> +0.97 -> +1.25 ps, std shrinking
>   10.7 -> 0.5 ps, register 29 -> 35) ran on the **tone-phase** route.
>
> So "dither does not work in the firmware" is really: **the firmware uses the
> impulses for alignment only**, and takes gain from a waveform fit and skew from the
> tone phase.  The host-side Python pipeline reached the same conclusion
> independently (see the gain-loop and A-B-spur bullets in `AGENTS.md`): the
> low-energy broadband pulse amplitude is too noisy to drive a gain loop (2.3 %
> per-frame scatter, 1.2 % away from the tone) and the dispersion-rounded pulse shape
> is useless for timing (the +-600 ps scatter appears in both implementations).
> Note also that claim (1) here is **corroborated** by that export: the gain stage
> passed with `final_gain_correction = 1.000000` while `cal_gain_ratio_b_over_a` was
> still 0.9873, i.e. the loop self-normalised and left a 1.3 % AC gain mismatch.
- Few events, template pollution, and poor fit quality lead to WARNING/INVALID.

### 3.5 Skew-Loop Errors

#### (1) Tone primary path
- Board convergence: **−0.87 ps, 2/2 pass**;
- Loop structure: characterise actuator → step by batch median → converge with consecutive passes;
- Main limits are actuator resolution and run-to-run characterisation variation (about 4–15 ps/code).

#### (2) Dither fine-skew advisory
Not reliable on the board. Mechanism evidence chain:

1. Link dispersion: 32–48 sample pulse → 74–123 samples wide;
2. Window/period constraint: with a 130-sample period, the ±64 window occupies 129 samples — no gap, no margin;
3. Window DC/slow-varying bias: A/B window means correlate with polarity; opposite edge biases of 100–240 ps;
4. Event-centre jumps: in-frame phase 44→9, spacing outliers;
5. Template/localisation sensitivity: same frame with different templates gives 0.72 ↔ 243 ps;
6. Tone-fit pollution: pulse duty cycle amplifies batch noise.

Fourteen methods were rejected with data (waveform geometry, window/gating sweeps, per-edge gain, detrending, two-parameter models, gap background estimation, median aggregation, event-centre refinement, gain-ratio calibration, etc.).

#### (3) Joint aggregation and differential measurement
- Joint cross-frame aggregation: board results 113–666 ps; occasional offline pass;
- Differential measurement (Δdither vs Δtone around a register step):
  - Tone differential works in simulation (about +11.8 to +14.1 ps, expected +13.8 ps);
  - Dither differential does not: dither valid frames drop from 1/10 to 0/10 on the simulated board.

### 3.6 Performance-Stage Errors

- A Stage-5 summary anomaly occurred: Mean RMSE 759, Mean correlation −0.998;
- Root cause: the canonical channel polarity was opposite to the actual signal;
- After the fix, board results recovered: Mean RMSE 23.15, Mean correlation +0.998354;
- The simulated board has no calibrated analog nonlinearity, jitter, or harmonic model, so SNDR/SFDR/ENOB are not used as numerical evidence.

---

## 4. Simulator Environment

### 4.1 What the simulator is

- A **simulated-board** environment that runs the **unmodified production firmware** on a host PC, with a virtual hardware model replacing the low-level board support package;
- The firmware's command parser, five-stage calibration flow, estimators, and DMA/SPI/UDP call paths all execute as real code;
- The virtual hardware model provides ADC/DAC behaviour, SPI registers, JESD state, DMA state machine, UDP bridge, and GPIO.

### 4.2 What the simulator models

- Virtual ADC signal chain: tone + dither sampling, channel gain/offset/skew, noise, pulse dispersion, sample-rate mismatch, saturation;
- Actuator model: fine-delay register writes change the inter-channel skew in real time;
- Ideal mode and simulated-board mode (dispersion + noise + skew drift);
- Fault injection: DMA errors, SPI timeouts, PLL unlock, etc.

### 4.3 What the simulator is used for

- Reproducing board behaviour without hardware: the simulated board reproduces "event detection valid, fine-skew unreliable";
- Rapid comparison of different dither structures (no-flat-top triangular, flat-top linear, PRBS, Golay);
- Validating estimator changes and failure modes without modifying production sources or using the board;
- Pre-screening and root-cause localisation before board experiments.

### 4.4 Limitations of the simulator

- It is not an RTL/transistor-level co-simulation; it does not cover the JESD electrical layer, detailed DMA timing, or cache coherency;
- It has no calibrated analog front-end model: no jitter, no ADC nonlinearity, no harmonics, no true SNDR/SFDR/ENOB;
- Pulse dispersion is a first-order proxy, not a physical electromagnetic model;
- Therefore simulator results are used for **relative comparison, failure-mode reproduction, and algorithm screening**, not as absolute hardware-accuracy predictions.

---

## 5. Simulation Evidence

### 5.1 Production Estimators on the Simulated Board (four dither structures)

| Structure | Mode | dither_A valid | fine-skew valid |
|---|---:|---:|---:|
| No-flat-top triangular | ideal | 1.000 | 1.000 |
| No-flat-top triangular | simulated board | 1.000 | 0.008 |
| Flat-top linear 48/32 | ideal | 1.000 | 1.000 |
| Flat-top linear 48/32 | simulated board | 1.000 | 0.015 |
| PRBS | ideal/simulated board | 0–0.013 | 0 |
| Golay | ideal/simulated board | 0/1.000 | 0/0.080 |

### 5.2 Dedicated Correlation Estimator (simulation)

- At the current operating point: Golay offset error ~0.3 codes, gain error ~2%;
- Fine-skew with multi-frame averaging: Golay **~0.35 ps** (20 frames), PRBS ~2.1 ps;
- At high frequency (2.7 GSPS / 1200 MHz): tone remains accurate, but the code-based estimator degrades (Golay skew ~4 ps, gain ~3.6%).

### 5.3 Differential Measurement (simulated board)

- Tone differential: ~+11.8 to +14.1 ps (expected +13.8 ps);
- Dither differential: 0/10 valid frames on the simulated board; Δ not reliable.

---

## 6. Root-Cause Summary

| Symptom | Root cause | Key evidence | Status / recommendation |
|---|---|---|---|
| Dither fine-skew unreliable | Link dispersion + window/period constraint + event jumps + template sensitivity | edge 100–1000 ps; fine-skew valid rate ~1% on simulated board | Current implementation unusable; coded sequences + dedicated estimator are the direction |
| Offset weak pass | Digital-template flat-top assumption invalid | "flat-top" samples are template-defined; 3–4 code deviation | Switch to polarity-balanced full-window averaging |
| Gain 0.24–0.44 | Template-to-code mapping + dispersion + no constant ratio | flat ≈ full (0.249/0.235) | A/B relative estimation avoids the mapping problem |
| Stage-5 summary anomaly | Canonical polarity inverted | Mean RMSE 759 / corr −0.998 | Fixed and verified on board |
| Tone primary <1 ps | LS phase difference + actuator loop | −0.87 ps, 2/2 pass | Current primary calibration mechanism |
| High-frequency accuracy uncertain | Jitter/bandwidth not modelled | 2.7 GSPS/1200 MHz simulation still accurate | Real board requires front-end modelling |

---

## 7. Conclusions and Recommendations

1. **The tone primary path is currently the most reliable**: <1 ps on board, Stage-5 parallel average 39.5 dB / 6.27 bits;
2. **Dither three-way separation is not usable in the current impulse + estimator + link combination**:
   - the offset flat-top assumption is invalid;
   - gain has no constant ratio;
   - skew is limited by dispersion and event localisation;
3. **If dither fine-skew is pursued**: PRBS/Golay with a dedicated estimator are the direction; in simulation Golay reaches ~0.35 ps with multi-frame averaging, clearly better than PRBS at ~2.1 ps;
4. **Short term**: keep the tone path as the calibration mechanism, and use dither only for event detection / structural verification;
5. **Medium term**: refine the Golay correlation/edge-aware estimator in simulation before porting to firmware;
6. **Engineering**: focus on the firmware pipeline's stage invalidation and estimator reliability, so that algorithmic improvements can be reproduced reliably in the production flow.
