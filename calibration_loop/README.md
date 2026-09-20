# calibration_loop

Impulse-dither background calibration of gain, offset and timing skew for the
ZCU102 + AD9164 (DPG) + AD9695 test bench.

The default hardware model is ADC 1.300 GSPS, DAC 2.600 GSPS, for an exact
DAC/ADC ratio of 2.

A short flat-topped impulse with balanced random polarity is summed into the DPG
vector alongside the main tone. Averaging the dither windows with the polarity
weight gives gain and skew; averaging without it gives offset. All three come
from one capture, and the loop pushes the skew correction back into the AD9695
sample-clock delay.

Runs against the existing firmware unchanged.

## Install

```bash
pip install numpy matplotlib pyserial
```

`pyserial` is only needed for the hardware modes.

## Usage

Generate the DPG vector (one signed integer per line, no header):

```bash
python -m calibration_loop.run_calibration gen --out waveforms
```

Run the loop against the built-in bench model, no hardware required:

```bash
python -m calibration_loop.run_calibration sim --iterations 60
```

Check the hardware without driving anything — run this before closing the loop:

```bash
python -m calibration_loop.run_calibration probe --uart COM5 --frames 10 --plot
```

It ends with five sanity checks, all of which must pass. The probe pools the
complete event windows, reports a jackknife uncertainty for the joint gain ratio,
and prints the batch-consensus dither polarity anchor (see `BENCH_GUIDE.md` §4).

Close the loop on the board. `--allow-skew-writes` is required: without it the run
measures skew but never drives the actuator, which is the open-loop case.

```bash
python tools/skew_park.py --uart COM5 --code 24   # neutral start; also the required
                                                  # initialization after a bring-up
python -m calibration_loop.run_calibration bench --uart COM5 \
    --allow-skew-writes --iterations 300
```

Add `--interleaved --skew-target-ps <Ts/2>` once the clock path provides a
half-period offset between the channels. `--help` lists the waveform and loop
parameters, all of which can be overridden on the command line.

### Running with no reference tone

A dither-only session has no tone to align to, no tone to cancel and no tone phase to time with,
so the loop switches routes. `--tone-free` sets all three together
(`--gain-observable dither_mag --skew-observable dither_fold --no-cancellation`):

```bash
python -m calibration_loop.run_calibration sim --tone-free \
    --amp-dbfs -120 --dither-edge 4 --dither-top 4 --dither-scale 16000 --iterations 60
python -m calibration_loop.run_calibration bench --uart COM5 --tone-free --allow-skew-writes \
    --amp-dbfs -120 --dither-edge 4 --dither-top 4 --dither-scale 16000 --iterations 300
```

The flags must describe the waveform actually loaded on the DPG (the config comes from the CLI,
not from the JSON); a mismatch shows up as a collapsing alignment margin and the run stopping
after 20 consecutive rejects. What each route is, and why:

- `dither_mag` averages each impulse's peak-to-peak per event and ratios the channels — sign-free,
  so no session polarity convention enters, and it agrees with the polarity-corrected folded
  replica ratio to 0.02 %.
- Both timing routes take their answer from the folded impulse replicas and share the loop's
  convention — **positive means channel B samples later**; the index-domain sign is the opposite
  and closes the loop backwards.
  - `dither_fold` (the default for `--tone-free`) projects the folded A-B difference onto the
    pulse slope. It compresses beyond ~0.1 sample, so its *scale* runs low on a large residual
    (the zero crossing stays honest), but it needs no template fit.
  - `dither_phase` fits each channel's replica against the template at a **fractional** sampling
    phase and differences the phases. It is the linear route — unbiased to ±2 ps over ±400 ps in
    the bench model — but it needs the replica to resemble the template, and the bench's analog
    path reshapes the impulse.
- Which is quieter is a property of the waveform, so measure it instead of assuming:
  `python tools/timing_route_check.py --state 24=... --state 32=... --waveform-json <yours>.json`
  reports both routes' ps/code against the actuator's known 4.8–4.9 (end-to-end) and 7.4 (single
  step) ps/code, plus their per-frame scatter. Archived 2026-09-19 captures: `w06` 16000 LSB →
  fold **15.3** ps/frame against phase 26.0; `w32` 2000 LSB → fold **145** against phase 747.
- Because there is no tone, `tone_ratio`, the A−B difference spur and the SNDR/SFDR columns are
  scored at f_in and describe the noise floor. The tone-free stand-ins are `gain_mag_ratio`,
  `offset_*_codes`, `skew_used_ps`, `skew_fold_ps` / `skew_slope_ps`, `dbc_ab_coherent` and
  `snr_dither_db`.

Offline proof of all of it (run before any bench session that uses these routes):

```bash
python calibration_out/_tone_free_loop_test.py   # routes against known delays + closed loop
python tools/loop_direction_check.py             # batch decision and direction latch, both routes
```

The skew axis closes on hardware (`dSkew` to about -10 ps, the raw A-B difference
spur from -24.6 to -38.6 dBc). The gain loop integrates the coherent **tone** ratio by
default (`--gain-observable tone`): that is the mismatch the A-B difference spur at
f_in is made of, and it is ~10x quieter than the dither pulse-window ratio the loop
used to integrate. Because the dither ratio is no longer what gets nulled, `g_B/g_A`
parks near 1.012 *by design* while `tone_ratio` goes to 1.0000; `--gain-observable
dither` restores the old behaviour. Read the skew result from `raw_difference_dbc`;
`BENCH_GUIDE.md` §6 has the measured numbers.

`--iterations` counts **qualified samples**: a capture the acceptance filter rejects
is logged with its reason and retried, so it neither shortens the learning curve nor
appears on it. At the measured ~20 % rejection rate a 300-sample run takes about 370
captures, and the meta JSON's `samples_qualified` / `captures_attempted` /
`captures_rejected` record what actually happened.

Each run writes a CSV log, a JSON metadata file and a learning-curve plot.

## Requirements on the waveform

`DitherConfig.validate()` enforces these; it will tell you which one failed.

- `adc_ratio = fs_dac / fs_adc` must be an integer, and the vector length, the
  dither period, position and pulse geometry must all be multiples of it.
  Otherwise the impulses do not land on a fixed ADC sample phase.
- `sig_cycles` must advance the tone by a useful fraction of a cycle between one
  impulse and the next, so the tone averages out across the events in a capture.
- The pulse must fit inside its slot, with a flat top and edges at least two ADC
  samples each.
- Main tone plus dither must not clip.

## Modules

| File | Contents |
|---|---|
| `dither.py` | DPG vector generation, ADC-rate pulse and derivative templates |
| `estimator.py` | De-framing, loop alignment, joint estimate, block-LMS state |
| `dither_raw.py` | Tone-free measurement of all three mismatches from the folded impulses |
| `metrics.py` | SNDR, SFDR, ENOB, interleaving spurs, A−B residual |
| `capture.py` | UART-triggered capture, UDP frame collection, clock-delay actuator |
| `simulate.py` | Full-chain bench model with known ground truth |
| `loop.py` | Closed loop, CSV logging, learning curves |
| `run_calibration.py` | CLI: `gen`, `sim`, `probe`, `bench` |

## Notes on the firmware

Two things in the current firmware affect this code:

- `ad9695_adc_super_fine_delay()` in `ad9695_api.c` writes to
  `AD9695_CLK_FINE_DELAY_REG` (0x0112) rather than
  `AD9695_CLK_SUPER_FINE_DELAY_REG` (0x0111), so the 0.25 ps field is never
  programmed. `capture.py` defaults to the 1.725 ps step; pass `--super-fine`
  after fixing it.
- `xemacif_input()` runs after the blocking `uart_get_line()` in `main.c`, so an
  incoming UDP packet is not serviced until the next console line. `capture.py`
  sends a bare newline after any clock-delay write.

A `cal` console command wrapping the existing `adc_capture_frame()` and
`udp_send_mem()` would remove two UART round trips per iteration. Not required.

See `BENCH_GUIDE.md` for the bench procedure.
