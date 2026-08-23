# Digitally Injected Impulse-Dither TI-ADC Calibration - Full Report (English)

**Date**: 2026-08-19
**Version**: v1.0 (final)
**Related docs**: `PLAN_DITHER_FIX.md` (execution history), `DITHER_CONCLUSION.md` (conclusions), `AGENTS.md` (repo guide)

---

## 1. Project Overview

University of Toronto summer-2025 research project: **time-interleaved (TI)
ADC calibration**. The goal is a hardware calibration algorithm using a
**digitally generated impulse dither** (injected via an AD9164 DAC driven by
a DPG pattern generator) to replace the ramp-dither / analog-summing
approaches of prior work, and to exploit the **slope characteristics at
different positions of the impulse** to separate the **gain, offset and
timing-skew** mismatches.

**Target bench**:
- **ZCU102** (Zynq UltraScale+ MPSoC, ARM Cortex-A53 + PL)
- **AD9695** dual ADC (JESD204B, 1.3 GSPS/channel)
- **AD9164** DAC (2.6 GSPS), DPG generates reference sine + impulse dither

**Core metrics**: post-calibration channel matching, SNDR/ENOB, and an
independent verification of the three-way dither separation.

---

## 2. System Architecture and Calibration Pipeline

### 2.1 Five-stage automatic calibration flow (`adc -cal`)

| Stage | Purpose | Primary estimator | Role of dither |
|---|---|---|---|
| 1 Timing | integer/fractional delay alignment, dither event detection | cross-correlation + tone fit | event detection / structural validation |
| 2 Offset | channel DC mismatch correction | tone DC fit | **dither offset separation** (flat-top sampling) |
| 3 Gain | channel gain mismatch correction | tone amplitude fit | **dither gain separation** (full/flat reference) |
| 4 Skew | closed-loop B-vs-A timing mismatch correction | **tone phase difference** (primary) | **dither fine-skew cross-check** (advisory) |
| 5 Performance | SNDR/SFDR/THD/ENOB, A/B matching | spectral analysis | — |

### 2.2 Three-layer code structure

1. **Firmware** (`test_platform/thesis_v3_500mhz_appl/`): bare-metal C,
   UART console, lwIP UDP offload, AXI DMA capture, SPI AD9695 control;
   the shared estimator modules (`adc_calibration_skew/dither/performance.c`)
   compile into both firmware and the desktop simulator.
2. **Desktop C simulator** (`calibration_sim/`): compiles the production
   estimator sources and runs unit/scenario/pipeline/stress tests - the
   **primary test suite** (currently 5728+ unit / 18 scenario / 14 pipeline
   all green).
3. **Python loop and tools** (`calibration_loop/`): `dither_replay.py`
   (offline raw-frame replay), `capture_frames.py` (batch capture),
   waveform generator, `run_calibration.py check` (waveform consistency).

### 2.3 Signal chain

```
DPG waveform (tone + impulse dither) -> AD9164 DAC -> analog injection
-> AD9695 ADC (A/B) -> DMA capture (4096 B) -> firmware estimation
(tone fit + residual + dither analysis) -> corrections
-> Stage-5 spectral evaluation -> UDP/CSV export
```

---

## 3. Principles

### 3.1 Tone-based primary estimation

- Per-frame 800-sample fixed window; least-squares tone fit (DC + cos + sin
  at a refined common frequency) on A and B, yielding amplitude, phase, RMSE
  and correlation;
- **Relative skew** = B/A phase difference (with global-polarity branch
  resolution; INVERTED -> +/-pi adjustment);
- Stage-4 closed loop: actuator (AD9695 fine-delay register) characterization
  of per-code response -> controller steps on the batch median -> 2/2
  consecutive PASS converges.

### 3.2 Impulse-dither three-way separation

| Quantity | Principle | Waveform requirement |
|---|---|---|
| **Offset** | polarity-weighted event averaging -> **flat-top sampling** (template derivative ~ 0) | true flat top (top >= 16 ADC samples) |
| **Gain** | residual-to-template **least-squares projection gain** (full) and flat-top gain (flat) | template/amplitude-domain consistency |
| **Skew** | polarity-weighted event-window aggregation -> measured A profile as local template -> fit B -> rising/falling/full derivative projections; rising - falling = edge disagreement | long linear ramp, window integrity |

- Event detection: cross-correlation of the residual with the template,
  strongest peak per event-period slot (130 ADC samples);
- Cross-check (advisory): `|dither - tone| <= 0.03 samples (23.1 ps @ 1.3
  GSPS)` AND estimator PASS; **never gates any stage**.

### 3.3 Cross-frame joint aggregation (experimental)

- Stack all event windows of a batch (10 frames) across frames and estimate
  once; the per-frame polarity sequences differ, so window-level bias
  randomizes;
- Implemented as shared `adc_cal_skew_estimate_joint_frames()`, invoked per
  Stage-4 iteration, exported to CSV and UART.

### 3.4 Stage-5 metrics

- Spectrum: Blackman-Harris7 window + per-bin Goertzel -> fundamental /
  harmonic / spur power -> SNDR/SFDR/THD/ENOB, for raw and calibrated;
- Matching: A/B correlation, RMSE, offset/gain/skew mismatch (after polarity
  normalization);
- Parallel-average output ((A+B)/2) SNDR/ENOB.

---

## 4. Results

### 4.1 Per-stage board results (latest run 19:03, linear 48/32 waveform, window +-64)

| Stage | Result |
|---|---|
| 1 Timing | PASS (correlation 0.9985, dither structural validation PASS) |
| 2 Offset | CONVERGED (+5.13 codes, verify residual 1.79, corr 0.9983); dither estimator PASS (flat-top 31 samples, 6 events) |
| 3 Gain | CONVERGED (gain 1.0000, verification error 0.00054); dither gain WARNING (dither 0.235 / flat 0.249, FIT_QUALITY) |
| 4 Skew | tone convergence **-0.87 ps**, 2/2 PASS, reg 35, actuator 12.1 ps/code; dither fine-skew PARTIAL (**1/10 frames VALID**), joint INVALID, edge disagreement 666 ps |
| 5 Performance | SNDR 38.8 dB / ENOB 6.13 bits (A/B); parallel average **39.5 dB / 6.27 bits**; cal A/B correlation 0.99995, RMSE 18.2 codes; **Mean RMSE 23.15 / Mean correlation +0.998354** (normal after fix) |
| Overall | **PASS** (skew MARGINAL warning) |

### 4.2 Tone primary path history

| run | final skew | batch std | convergence | reg |
|---|---|---|---|---|
| 13:18 | -4.99 ps | 1.81 ps | CONVERGED | 33 |
| 17:55 | **+0.166 ps** | 0.34 ps | CONVERGED | 35 |
| 19:03 | -0.87 ps | 0.62 ps | CONVERGED | 35 |

### 4.3 Final dither three-way separation status

| Branch | Status | Key data |
|---|---|---|
| Event detection | **VALID 10/10** | stable across runs |
| **Offset** | **estimator PASS / controller converged (limited accuracy)** | flat-top 31 samples; Dither offset vs fitted-tone DC deviates 3-4 codes, unstable across runs |
| **Gain** | **limited (concluded)** | dither_gain 0.24-0.44; **flat ~= dither (0.249 vs 0.235)**; no calibration constant |
| **Skew** | **concluded INVALID (advisory)** | per-frame edge disagreement median 148-305 ps (latest 666 ps); joint all zero; 1/10 frames VALID |

### 4.4 Tooling outputs

- **Per-frame diagnostics** (`calibration_skew_captures.csv`): dither_reason /
  rising / falling / edge / SNDR / ENOB (whole-pipeline tracking);
- **SNDR/ENOB tracking**: timing ~37 dB, offset/gain ~37 dB, skew fit ~32 dB,
  Stage 5 ~39 dB (plottable per capture/iteration);
- **sim**: 5728+ unit / 18 scenario / 14 pipeline green;
- **Offline replay**: `dither_replay.py` supports template/window/gate/
  detrend/diagnostics options.

---

## 5. Problems

### 5.1 Core problem: dither fine-skew cross-check unusable

**Symptoms**: per-frame edge disagreement 100-1000 ps (gate 23.1 ps); dither
estimate drifts +-100-200 ps frame-to-frame while tone stays within +-1 ps;
joint aggregation passed offline on 6 frames by chance (4.7/12.4 ps) but
failed on board with 10 frames (113-666 ps).

**14 methods tested and rejected (all with data)**: waveform geometries
(RC 16/32, linear 48/0, linear 48/32, p260 doubled period), window/gate
parameter scan (25 combinations), per-edge gain, self-calibrated event
centers, detrend, DC removal, two-parameter delay+width model, gap-background
estimation, median aggregation, event-quality gating, center refinement,
gain ratio calibration, flat-gain calibration.

**Mechanism evidence chain (6 items)**:
1. **Chain dispersion**: injected 32-48-sample pulse -> 74-123-sample
   10-90% width at the ADC;
2. **Window/period constraint**: at 130-sample period, the +-64 window fills
   129 samples - no gap to measure background, no tolerance for event-center
   jumps (35 samples); doubling the period (260) removes the constraint but
   the wider pulse (112 samples) pollutes the tone fit -> primary path
   degrades (baseline std 2-9 -> 23 ps) - **a fundamental trade-off between
   dither geometry and the tone primary estimator**;
3. **Window DC/slow bias**: A/B window means constant +1.8/-1.6 codes
   (polarity-correlated); asymmetric rising/falling masks -> opposite edge
   biases (100-240 ps);
4. **Event-center jumps**: intra-frame phase 44->9 (spacing 95/131
   artifacts);
5. **Template/location sensitivity**: same frame, different template ->
   0.72 vs 243 ps disagreement (17x);
6. **Tone-fit pollution**: pulse share of the window directly amplifies
   batch noise.

### 5.2 Offset / Gain separation accuracy

- **Offset**: PASS is a weak within-tolerance pass; Dither offset deviates
  3-4 codes from the tone-DC truth, direction/magnitude unstable across runs;
- **Gain**: dither_gain consistently low (0.24-0.44 vs 1.0), flat ~= full ->
  a global template-domain vs ADC-code-domain amplitude mapping problem, with
  no stable ratio across runs -> not calibratable.

### 5.3 Stage-5 summary anomaly (fixed)

From the joint-firmware versions, `Mean RMSE 759 / Mean correlation -0.998`
(baseline 19 / +0.999). Root cause: the canonical-channel polarity was
mislabelled relative to the actual signal (polarity[A] = -1 while cal_a is
in phase with the reference) -> normalized cal anti-correlates. **Fix**:
polarity self-correction in `analyze_frame` (negative canonical correlation
-> global two-channel flip); board validation recovered
`Mean RMSE 23.15 / Mean correlation +0.998354`.

### 5.4 Other issues

- **Initial baseline MARGINAL** (std 19-23 ps vs 2-9 ps on older geometry):
  appears with the 48/32 waveform; characterization needs larger steps
  (1-2 codes fail -> 4-8 codes pass);
- **UART output flooding** (20+ lines/frame diagnostics) -> switched to
  summary-only;
- **Batch capture**: single-frame UART loop works but is slow; the `dma
  -burst` firmware command is implemented and awaits flashing; port/firmware
  mismatch can produce duplicate frames (identical MD5);
- **sim fixtures**: two `--run-all` hard-dependency fixtures were deleted by
  accident (restored; do not delete again).

---

## 6. Recommendations

### 6.1 Paper narrative

- **Claim**: "digitally generated impulse dither separates three
  mismatches" -> "**complete three-way separation architecture + offset
  separation estimator integrated and passing + gain/skew limitation
  analysis**";
- **Positive results**: digital impulse-dither chain (waveform/injection/
  event detection 10/10) OK; tone primary calibration system (convergence
  <1 ps, Stage-5 6.27 bits parallel) OK; offset estimator PASS OK; Stage-5
  metrics restored OK;
- **Limitations section**: 6-item mechanism chain + systematic rejection of
  14 methods (a publishable engineering contribution: complete attribution
  and quantified trade-offs);
- **Careful wording**: the offset PASS is a weak within-tolerance pass
  (3-4 codes deviation); the "no stable calibration constant" conclusion for
  gain/skew must be explicit.

### 6.2 Technical recommendations (future work)

1. **Explicit amplitude-mapping calibration for gain**: the template-vs-ADC
   code-domain ratio (0.24-0.44, waveform-dependent) could be characterized
   once offline as a lookup table - an engineering workaround under the
   limitation;
2. **Differential route for skew**: compare Delta(dither) vs Delta(tone)
   between two batches at neighbouring register settings (absolute bias
   cancels); requires correct batch capture (`capture_frames.py` +
   `dma -burst`);
3. **Robust event localization**: centroid/neighbourhood-weighted peaks
   instead of argmax, or spacing-consistency gating for jump events
   (sensitivity 177 -> 105 ps already observed);
4. **Smaller dither amplitude sweep**: reduce tone-fit pollution
   (`DITHER_SCALE_LSB`);
5. **Analog-chain dispersion compensation**: if the chain impulse response
   can be characterized, pre-disperse the template to match the ADC-domain
   measurement;
6. **Document Stage-5 summary fields**: Mean RMSE/correlation are canonical-
   channel normalized-cal-vs-reference metrics (including the polarity
   self-correction semantics).

### 6.3 Engineering recommendations

- Flash the current firmware sources (window-64 baseline + joint +
  `dma -burst` + Stage-5 self-correction + UART summary-only) as the **final
  validation build**;
- Protect or relocate the `--run-all` fixtures
  (`adc_capture_20260801_180451.csv` etc.) - hard dependencies;
- Waveform and firmware window macro must match (130-period <-> window 64;
  p260 <-> window 100);
- Capture validation: after a batch, check MD5 uniqueness (20 frames should
  yield 20 distinct hashes).

---

## Appendix A: Key Files

| File | Role |
|---|---|
| `PLAN_DITHER_FIX.md` | fix plan + 8-section execution history (all rejections) |
| `DITHER_CONCLUSION.md` | final conclusions and paper limitations material |
| `adc_calibration_skew.c/h` | shared skew estimator (joint, detrend, polarization) |
| `adc_calibration_dither.c/h` | shared dither events/analysis (polarize helper) |
| `adc_calibration_performance.c/h` | Stage-5 spectrum/matching (polarity self-correction) |
| `butils.c` / `butils_calibration.c` | board orchestration, export, diagnostics, joint wiring |
| `calibration_loop/dither_replay.py` | offline replay/diagnostics |
| `calibration_loop/capture_frames.py` | batch capture (burst mode) |
| `waveform_generation/generate_dac_waveform.py` | waveform generation (period/shape options) |

## Appendix B: Quick Reference Numbers

- Gates: edge disagreement 0.03 samples = 23.1 ps @ 1.3 GSPS; same for
  tone/dither disagreement;
- Dither period: 130 ADC samples (= 260 DAC); p260 = 260 ADC samples;
- Window: +-64 (130-period) or +-100 (p260);
- Stage 5: SNDR ~39 dB, ENOB ~6.3 bits (parallel average);
- sim: 5728+ unit / 18 scenario / 14 pipeline.
