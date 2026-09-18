# Skew actuator: bench validation procedure

Staged validation for the AD9695 sample-clock-delay actuator. Do the stages in
order; each one gates the next. Nothing here is optional, and every stage has a
hard stop condition rather than a "look at it and decide" step.

## Invariants (true before you start)

| | |
|---|---|
| The raw UDP delay path is **frozen in code**, unconditionally. `AdcClockDelay.set()` raises. | `calibration_loop/capture.py` |
| The **only** sanctioned write is the firmware transaction, reached by `adc -cal skew step +/-N`. | `butils.c: handle_adc_skew_transaction_cmd()` |
| Plain `bench` behaves like `--open-skew`: the loop never calls the actuator unless `--allow-skew-writes` is given. | `run_calibration.py: skew_writes_ok()` |
| The firmware refuses to send a DMA buffer that is not from a completed transfer, and the generation counter advances only on a completed transfer. | `ethernet.c: dma_capture_*` |
| One control code = 4 raw `0x0112` taps per channel, opposite directions ≈ **13.8 ps differential**. Nominal until stage 6 measures it. | `butils.c` |

Measured inputs that size the runs (bench sessions 2026-09-17):

| quantity | value |
|---|---|
| per-frame skew scatter, **actuator NOT initialized** (mode 0x00) | ±18.9 ps RMS |
| per-frame skew scatter, **neutral** (mode 0x04 + 0x0114 = 0x60) | **±3.1 ps RMS** |
| accepted-frame yield in the neutral state | **83 %** |
| measured differential step | **7.4 ps per control code** (nominal 13.8, see below) |
| frames needed per point for 80 / 90 / 95 % power | **< 1 / < 1 / < 1 valid** |

Two consequences worth internalising:

- **The neutral initialization is not optional.** It changes the per-frame scatter
  by a factor of six, so every measurement before it is six times noisier than it
  has to be. Do it as the first thing after every bring-up; a zero-step
  transaction (`adc -cal skew step 0`) does it and reports what it did in the ACK
  (`neutral_initialized=YES`). This also explains the old ±19 ps "hardware jitter":
  it was mostly the un-programmed delay path, not the ADCs.
- **The step is ~7.4 ps/code, not the documented 13.8 ps** (measured: +7.17 ± 1.21
  ps at code 25, +7.6 repeat, −8.0 at code 23, +15.36 for two codes). The firmware
  quotes 13.8 ps from 4 raw taps per channel in opposite directions at 1.725 ps
  each; the measurement is ~half that, so either only one channel's taps move the
  measured relation or the tap is nearer 0.9 ps. `SkewActuator.HOST_STEP_PS` uses
  the measured value; the discrepancy is a firmware-side question.

## Stage 5 — neutral transactions only (no differential skew)

Question: does the write path survive repetition *without* moving the differential
code? If this fails, stop; stage 6 would be measuring nothing.

Repeat **10 times**, each round starting from a fresh bring-up:

0. After the bring-up, **before anything else**, do the neutral initialization with
   a zero-step transaction: `adc -cal skew step 0`. The ACK must say
   `neutral_initialized=YES` on the first one and `NO` afterwards, and the first
   one legitimately **shifts the operating point** (measured +57 ps) because
   enabling fine-delay mode moves where the converters sample. Everything measured
   before this step is six times noisier.
1. `probe --uart COM5 --frames 12` — the pre-write baseline. Requires 5/5 checks,
   alignment margin > 6.
3. Send the neutral transaction on the console (this is `step 0`: it verifies the
   actuator, initializes it to neutral if needed, and moves nothing):
   ```
   adc -cal skew step 0
   ```
4. Read the ACK:
   ```
   SKEW-TXN <n> code_before=24 code_after=24 applied_steps=0
   SKEW-TXN <n> nominal_differential_ps_x10=0 saturated=NO neutral_initialized=YES|NO
   SKEW-TXN <n> capture_generation=<g> previous_generation=<p>
   SKEW-TXN <n> result=OK
   ```
5. `probe --uart COM5 --frames 20` — the post-write health check.

**Pass (all 10 rounds):** `result=OK` every time; `code_after == code_before == 24`;
`capture_generation > previous_generation`; no reboot needed; no `Capture INVALID`;
no `REFUSING to send`; post-write probe still 5/5 with margin > 6 and
`gain ratio scatter`/`uncertainty` inside its threshold.

**Hard stop:** any `result=RECOVERY_REQUIRED`, any round needing a reboot, any
fall in margin, or any half-synced signature (`|corr(A,B)|` far below
`cos(2πf·120 ps) ≈ 0.99`, residuals independent per channel).

Two cheap gate tests worth doing once, while you are there:

- **Freshness gate**: after a bring-up, send `udp` *without* `dma -w` first. The
  board must answer `UDP request ignored: no valid DMA capture ... run "dma -w" first.`
- **Duplicate gate**: run `dma -w` then `udp` twice. The two frames are identical
  and so is the printed generation — expected; the host is what dedupes, via the
  `frame_sha1` column in the loop CSV.

## Stage 6 — characterize the step

Question: how many picoseconds does one control code actually move, and with which
sign? This is the number every later conversion depends on.

Paired within one session, so drift cancels:

```
code 24  (baseline, 40 frames)
code 25  (stepped,  40 frames)
code 24  (restore,  40 frames)
code 25  (stepped,  40 frames)
code 24  (restore,  40 frames)
```

40 frames per point ≈ 30 valid frames ≈ 80 % power for a 13.8 ps step. Use 55 per
point if you want 90 %.

For each point: send `adc -cal skew step <delta>` on the console, require
`result=OK`, then capture the frames. The `skew_batch` acceptance filter and the
arithmetic mean are already in `calibration_loop.estimator`, and
`tools/skew_step_characterize.py` drives the whole sequence and prints the table
below.

**Pass:** the two stepped points move the same way and their step sizes agree
within the 95 % CI; restoring baseline returns the mean to its original position
within the CI; every point's data面 healthy (probe-grade margin, `|corr(A,B)| ≈
0.99`, no stale generation); no reboot at any point.

**Hard stop:** opposite signs between repeats, a step inside the noise, no return
to baseline after restore, or any link/DMA anomaly. Record and stop — do not
"try another code to see".

Then repeat the same paired sequence with `code 23` to confirm the negative
direction, and record ps/code for both signs.

## Stage 7 — limited closed loop

Only with `--allow-skew-writes`, and only after stages 5 and 6 pass.

Per decision: at least **20 accepted frames** (≈27 acquisitions at 75 % yield),
then

```
mean_error = skew_batch.mean_ps - skew_target_ps
```

Move **one code** (sign from `mean_error`) only when `|mean_error| > 10 ps`
(the measured jitter floor is 19 ps RMS per frame; 20 frames gives SE ≈ 4.2 ps, so
10 ps is ≈ 2.4 σ of the batch). A stronger rule, if you want fewer false moves:
two consecutive batches both over 10 ps in the same direction.

Staging: 3 physical updates → check → 10 updates → check → full run. Abort the run
on any of: non-`OK` ACK, missing readback, repeated generation, DMA timeout,
correlation collapse, falling margin, half-synced data, actuator saturation, or an
error that does not move in the calibrated direction.

The loop keeps a hard clamp at 0..48 codes and a deadband, so it cannot wind up
against the limits (`SkewActuator.error_to_steps`).

## What to record

Per round/point: the raw ACK block, `probe` output, and — for stage 6 — the
per-frame `skew_phase_ps`, `skew_diff_route_ps`, `skew_source` and `frame_sha1`
(the loop CSV carries all four; `tools/skew_step_characterize.py` writes them to
its own CSV). Those are what make the result auditable rather than anecdotal.
