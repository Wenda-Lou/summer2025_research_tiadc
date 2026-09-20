# TIADC calibration project — onboarding overview

**High-Speed ADC Calibration Project**
Project overview and current status for onboarding

**Current focus:** determine what gain, offset and timing information a sparse impulse dither can
reliably carry through the real AD9164-to-AD9695 signal chain, before adding more estimator or
closed-loop complexity.

## 1. Project Background

This is a legacy research project originally proposed by Bowen, a Master's student who has already
graduated. The idea is to calibrate the signal sampled by a high-speed ADC by injecting impulse
dithers into the signal path and extracting calibration information from the received dither
waveform.

- Current calibration targets: gain mismatch, offset mismatch, and timing skew.
- Bandwidth mismatch was part of the original concept, but it has not been implemented so far.

## 2. Original Selling Points

**Selling Point 1 — Simple impulse injection.** Use only impulses that are low cost to generate and
potentially easy to couple into channels, without relying on ramp or triangular calibration
waveforms. For experiments and demonstrations, the DAC directly synthesizes the signal together
with the dither. In a practical system the mission-signal path may be difficult to modify, so a
separately coupled sparse impulse could provide a practical advantage.

**Selling Point 2 — Simultaneous multi-parameter calibration.** Use the same injected signal to
calibrate several mismatch parameters in the background: gain, offset, timing skew, and originally
bandwidth mismatch. Our industrial collaborators were particularly interested in bandwidth
calibration, but it appeared significantly more difficult, so we did not pursue it. If gain, offset
and skew can all be calibrated from the same injected signal, that would still preserve most of the
original goal.

## 3. Hardware and Software Platform

| Component | Role / Requirement |
|---|---|
| AD9164 DAC | Generates the test waveform and dither for the current experimental setup. |
| AD9695 dual-channel ADC | Samples Channel A/B and provides programmable delay control for timing-skew correction. |
| Xilinx ZCU102 | Runs the FPGA/embedded application, controls the ADC, captures samples, and communicates with the host. |
| ACE x86 | Configures Analog Devices hardware. |
| DPG Downloader | Uploads waveform files to the DAC. |
| Vitis 2025.1 | Builds and runs the ZCU102 application and embedded C calibration code. |

**Key numbers**, worth knowing before reading any measurement:

- ADC (AD9695, dual channel): **1.300 GSPS per channel, 14-bit**, JESD204 link to the FPGA.
- DAC (AD9164, played by the DPG): **2.600 GSPS, 16-bit**. The ratio is exactly 2, which the dither
  scheme requires.
- ADC input full scale: 1.59 Vpp differential, programmable from 1.36 to 2.04 Vpp per channel.
- The dither is one short pulse every 100 ns. The current pulse is 6 ADC samples (≈4.6 ns) long.

**Important:** the Verilog/FPGA datapath was already completed by the previous summer student.
Current work is mainly on calibration algorithms, embedded C, host-side software, board
measurements, and analysis.

## 4. Calibration Targets

- **Offset:** difference in DC level between ADC Channel A and Channel B.
- **Gain:** difference in channel amplitude response.
- **Timing skew:** difference in effective sampling time between the two channels.

**One thing to know up front:** on this bench the two converters sample at the *same* instant — they
are parallel, not alternating. Two consequences. A mismatch therefore appears as a difference
between Channel A and Channel B, not as the usual interleaving image spur, and A−B is the metric
everything here is built on. And combining the two channels adds nothing: A+B is just a duplicate of
the signal.

**Why true 2× interleaving is not reachable right now.** Interleaving means the two converters take
turns, so the second one has to sample exactly half a sample period after the first — at 1.3 GSPS
that is 384.6 ps, which would give a combined 2.6 GSPS stream. Today both cores run off the same
sample clock with no deliberate offset. The chip can shift one channel's sampling instant, but its
whole on-chip clock-delay range is only about 360 ps at best (48 control codes), short of the
384.6 ps needed, so it cannot be reached from the chip alone; it would take about 8 cm of extra coax
in one channel's analog path. The parallel geometry is directly visible in the data: |corr(A, B)| at
the correct de-framing is ≈1 (measured 0.998), whereas a half-sample offset would put it at 0.886 for
a 199 MHz input.

## 5. Two Calibration Implementations

### 5.1 On-board firmware calibration pipeline

The first implementation runs automatically on the ZCU102 using embedded C. The current five-stage
flow is:

| Stage | Main task | Purpose |
|---|---|---|
| 1. Timing / alignment | Reference alignment, correlation, useful window, channel relationship | Establish reliable frame alignment. |
| 2. Offset | Estimate DC mismatch, update correction, verify convergence | Correct channel offset mismatch. |
| 3. Gain | Estimate relative gain, update correction, verify convergence | Correct channel amplitude mismatch. |
| 4. Skew | Estimate timing mismatch, update AD9695 delay actuator, verify residual | Correct relative sampling-time mismatch. |
| 5. Performance | Evaluate SNDR, SFDR, ENOB, matching, and related metrics | Check signal quality and channel agreement. |

**Current issue:** the firmware calibration does not reliably use the impulse dither to estimate gain
and fine timing skew. In practice, the stable gain and skew estimates come from the tone itself,
which does not support the original dither-calibration research claim.

**Main suspected cause:** the dither pulse is reshaped by the real DAC-to-ADC signal path — measured
recently, it arrives *narrower* than generated rather than broadened (a pulse laid out as 32 ADC
samples wide comes back about 17 samples wide at half maximum). The original template-based gain and
fine-skew estimators are sensitive to that reshaping; the direct raw measurements described in
sections 6 and 7 are not.

**Status:** development of this firmware approach is paused for now while we isolate the dither
observability problem more directly.

### 5.2 Host-side calibration loop

The second implementation performs most calibration processing on the host PC. The board mainly
captures ADC samples and accepts hardware actuator updates.

```
capture -> decode/de-frame -> estimate mismatch -> update controller -> write actuator if needed -> capture again
```

This path is easier for debugging estimators, inspecting raw waveforms, trying new algorithms,
plotting convergence, and comparing observables without rebuilding the full firmware pipeline.

**Current status:** recent tests of Harry's host-side loop have been more promising. It has closed on
real hardware and currently provides the clearest environment for understanding which information
comes from the impulse and which comes from the tone. It is the main focus right now. Note that the
tone-based calibration path is no longer used, per the direction below.

## 6. Current Dither Status

| Quantity | Status | Interpretation |
|---|---|---|
| Alignment | Reliable | Impulse events are easy to detect and are useful for synchronization and event location. |
| Offset | Reliable | The impulse carries the DC mismatch directly, and a known offset change is clearly visible in the raw data. |
| Gain | Observable | The impulse amplitude ratio follows a known gain change. Note the estimator's own impulse-based gain route is biased and does not track the pulse amplitude — the direct amplitude ratio is the one to use. |
| Fine skew | Observable | This was expected to be the hard one, and it is not: a single delay step (~5–10 ps) is clearly visible, and shortening the pulse improved the timing measurement about 3×. |

## 7. What We Are Doing Right Now

Professor Liu asked us to temporarily step away from the complicated closed-loop algorithms and
first perform the most basic **dither-only** characterization. The key question is:

> When gain, offset, or timing skew changes, can we directly observe the corresponding change in the
> raw dither waveform?

The experiment follows this sequence:

- Capture repeated raw Channel A and Channel B waveforms and establish a baseline.
- Introduce a known change — offset, then gain, then timing delay — one at a time.
- Compare the raw ADC outputs directly, before using any estimator or closed-loop controller.

**Current progress: the characterization is done, and the answer is yes for all three.**

| Change introduced | What the raw data did |
|---|---|
| One delay step (~5–10 ps) | Clearly visible |
| A 6.5 % gain change on one channel | Read as 6.1 % |
| The chip's own offset correction switched on one channel | DC difference moved ~8 counts |

Each test changed one thing, and the other two measurements stayed put. The pulse was also shortened
to what the DAC can generate (6 samples, ≈4.6 ns), which improved the timing measurement about 3×.

**Still to do:** confirm that the dither *estimator* recovers the same known changes, i.e. that the
algorithm agrees with the raw measurement.

## 8. Current Research Question

What gain, offset and timing information can a sparse injected impulse reliably carry through the
real high-speed DAC-to-ADC signal chain, and how can that information be used for calibration?

This is the question the current dither-only characterization is meant to answer, by measurement
rather than by assumption. It also sets the boundary for later work: whatever the impulses turn out
to carry reliably is what a dither-based calibration can be built on, and anything they cannot carry
has to come from elsewhere or be dropped. Any change of objective beyond that would be discussed
with Professor Liu first.

## 9. Suggested Starting Point for Onboarding

- Read `AGENTS.md` (repository map, plus the things that have already cost us time) and
  `BENCH_SESSION_DITHER_ONLY.md` (the current dither-only procedure and results).
- Review the hardware signal chain: AD9164 → analog path → AD9695 → ZCU102.
- Install / verify ACE x86, DPG Downloader, and Vitis 2025.1.
- Review the firmware five-stage pipeline and the host-side capture / controller flow.
- Run what needs no hardware: the `calibration_sim` test suite (`--run-all`) and
  `python -m calibration_loop.run_calibration sim`.
- Start with raw-waveform comparison before adding new estimator complexity.
