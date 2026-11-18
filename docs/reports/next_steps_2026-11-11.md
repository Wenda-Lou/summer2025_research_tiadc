# Next steps after the meeting with Prof, Nov 11th 2026

Source: the Teams thread with Prof Xilin Liu and Wenda Lou on 2026-11-11, which discussed the RF-combiner runs of 2026-11-04.
Deadline: **show the scope waveforms next Wednesday, 2026-11-18.**

## What Prof asked

1. What "full scale dither" means, and **how large the dither really is**. Prof thinks it is larger than 74 mVpp.
2. **Check it on the scope, and show the waveforms with the tone and the dither combined.**
3. Prof's view: with a tone present, you need to average many dither pulses to remove the tone. The board does not do this, so **it is hard to draw conclusions** from the tone runs.

## Clear up the terms first

Prof had to ask three times what each number meant. Use these names and units every time:

| Term | Meaning | Value in the 11-04 runs |
|---|---|---|
| Tone frequency | DG922 sine frequency | 199.375 MHz (written "200 MHz" in the email) |
| Tone amplitude | DG922 amplitude setting, 50 Ω load, before the combiner | 100 / 200 / 280 / 400 mVpp |
| Dither repetition rate | How often a pulse is sent | 10 MHz (one pulse every 100 ns), 20 MHz, 100 MHz |
| Dither amplitude, digital | DAC code at the pulse peak | 32767 LSB |
| Dither amplitude, analog | Volts at a given point in the chain | **Not measured yet**; see below |

**Why "full scale"**: 32767 is the largest code of the AD9164's signed 16-bit input. So "full scale" means **DAC digital full scale**, not ADC full scale and not a measured voltage.

**Where "74 mVpp" came from**: it is a calculation, not a measurement on the current setup. In September, with the DAC wired directly to the ADC (no combiner), the measured gain of 0.0234 ADC codes per DAC LSB was extrapolated to 32767 LSB. That gives 765 codes, which at 1.59 Vpp ADC full scale is about 74 mVpp **at the ADC**. Three reasons it does not describe the dither today:
- It is the size **at the ADC**. At the DAC output the pulse is larger: the DAC can give roughly +4.8 dBm, about 1.1 Vpp for a sine, depending on the FSC current set in ACE. The path attenuates the pulse, and the band-limited path shrinks a narrow pulse further.
- The combiner and splitter now sit in the path (about −8 to −12 dB measured on 11-04), so the old number does not apply.
- So Prof is right that the dither is larger than 74 mVpp at the DAC. Nobody has measured either end.

**Correct two statements from the thread**:
- "The amplitude of the tone is around 74 mVpp": the 74 mVpp is the **dither** (calculated). The tone is the DG922 setting, 100 to 400 mVpp.
- "10 MHz dither converges up to 200 mVpp": on 11-04, 280 mVpp still converged with 40 of 300 frames rejected, and 340 mVpp was fully rejected. So the limit is about 280 mVpp. 20 MHz reached 400 mVpp and 100 MHz about 800 mVpp.

## 1. Scope measurement (for 2026-11-18)

Goal: one table and one set of screenshots that give the real dither and tone amplitudes at every point in the chain.

**Scope settings**
- **50 Ω input termination** (never 1 MΩ: it reads about 2× and unloads the source).
- Bandwidth and sample rate as high as available. The pulse is about 4.6 ns wide (6 ADC samples) with about 1.5 ns edges, so at least 500 MHz bandwidth and 5 GSa/s are needed to see its true peak. Record the scope's bandwidth with every screenshot: a slow scope reads the pulse peak too low.
- Use the same cable for every point and note its length.

**Measure at four points, in this order**

| # | Point | Source on | What to record |
|---|---|---|---|
| 1 | DAC output SMA | DPG plays the 10 MHz dither-only file; DG922 off | Pulse peak, peak-to-peak (+ and − pulses), width at half height, spacing (should be 100 ns) |
| 2 | DG922 output | DG922 on at 200 mVpp, 199.375 MHz; DPG stopped | Tone Vpp (check that it matches the panel setting) |
| 3 | Combiner output | Both on | Screenshot of the combined waveform: pulses riding on the tone |
| 4 | Splitter output A and B (what the ADC sees) | Both on, then dither only | Same as points 1 and 3; this is the number that matters for the ADC |

Then repeat point 1 or point 4 with the 20 MHz file (`_p130`) and the 100 MHz file (`_p26`). The pulse should keep its size and only get closer together.

**Triggering**: the pulses alternate polarity in a pseudo-random order, so trigger on the rising edge at a level just below the positive peak. Use a long timebase (at least 1 µs) for the overview and about 20 ns per division for the pulse shape. With the tone on, also capture one screenshot at a long timebase showing the tone with the pulses on it. That is the picture Prof asked for.

**Write down**: the DAC FSC current in ACE, the ADC IFC setting (1.36 or 1.59 Vpp), and the dither file name. All three change the numbers.

**Also record the same state on the ADC side** so the scope volts can be tied to ADC codes: with the board running, take `adc -cal loop open 20` (or the frame-capture script below) and note the pulse size in codes. Volts at point 4 divided by codes gives the real mV per code of the current chain.

## 2. Answer Prof's averaging question properly

Prof's point is correct, and this is how to state it precisely:

- The board estimator does **not** average pulses and does **not** remove the tone. Each frame gets one FFT of 512 samples. Gain, skew and offset come from all bins above 1 % of the strongest one. A strong tone becomes the strongest bin, pushes the dither bins under the 1 % line, and the frame is rejected (`st=events`). See `calibration_loop_c/adc_calibration_loop_blind.h`, lines 59–77.
- So the tone limits (280 / 400 / 800 mVpp) are limits of **this implementation**, not of the dither method.
- The skew and gain residuals in those runs are the estimator measuring itself. They have **not** been checked against an independent reference yet.

**Decision to raise with Prof**: classic averaging folds the pulses using their known positions and polarity. The 2026-10-28 requirement says the board must use **no supplied reference, template or polarity sequence**, so classic averaging conflicts with it. Options that keep the requirement:
- Remove the tone in the frequency domain: notch out narrow, strong peaks before weighting the bins. No reference is needed.
- Fit the strongest sinusoid from the data itself and subtract it before estimating.
- Or relax the requirement and allow the known polarity sequence, which brings back synchronous averaging.

Ask Prof which one he wants before changing the firmware.

## 3. Rewrite the SNDR claim

The email said: "the reference-free algorithm might not be able to recognize the dither, which wouldn't be able to remove the dither from the output". The precise version:

> In reference-free mode the dither is never subtracted from the output, by design, because the board is given no copy of it. It stays in the signal as added noise. Its power rises 3 dB each time the repetition rate doubles: −12.7 dBFS at 10 MHz, −9.7 dBFS at 20 MHz, −2.7 dBFS at 100 MHz (DAC digital). So a denser dither tolerates a larger tone, but costs signal SNDR. This is predicted, not measured yet.

To measure it: keep the tone fixed, play the 10, 20 and 100 MHz dither files in turn, and compute the tone SNDR from captured frames.

## 4. Bench runs to do after the scope session

- [ ] **Matched set in one bring-up**: dither only, then tone 100, 200 and 280 mVpp, all at 10 MHz dither. Start every run from code 24 and end it with `--park-after 24`. On 11-04 there was no tone-off reference run, so "what the tone changes" could not be answered. Commands: `docs/bench/CALIBRATION_STARTUP.md` section 7.
- [ ] **Independent check**: at the converged code, capture frames and fit the 199.375 MHz tone on A and B. The phase difference gives the skew in ps, and the amplitude ratio gives the gain. Neither passes through the board estimator. This needs a new read-only script; ask Claude to write it.
- [ ] **SNDR versus dither rate**: see section 3.
- [ ] **Tone removal**: after Prof chooses an option in section 2, implement it in `calibration_loop_c`, keep `adc_cal_sim --run-loop-tests` green, rebuild the ELF, and repeat the tone amplitude sweep. If the rejection limit rises while the independent check stays the same, the limit was the implementation.

## 5. To prepare for 2026-11-18

- [ ] Scope screenshots: points 1–4, plus the combined tone + dither picture.
- [ ] One table: dither and tone amplitude at each point, with scope bandwidth, FSC, IFC and file name.
- [ ] One sentence each on what "full scale" means and why 74 mVpp was a calculation.
- [ ] The averaging question from section 2, with the three options, so Prof can pick one.
