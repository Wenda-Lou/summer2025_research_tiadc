# Bench procedure

Bring-up and measurement steps for the impulse-dither calibration loop.
Allow 2–3 hours for a first run.

## 0. Before going to the lab

```bash
git pull origin HG_Loop
pip install numpy matplotlib pyserial
python -m calibration_loop.run_calibration sim --iterations 40
```

The simulation should end with the gain ratio near 1.0000, offset mismatch under
0.1 LSB and skew mismatch under 1 ps. If it does not, stop here — something is
wrong with the environment, not the bench.

## 1. Confirm the configured converter clocks

The calibration requires `fs_dac / fs_adc` to be a whole number. The corrected
bench configuration is a 2.600 GSPS DAC update rate and a 1.300 GSPS ADC sample
rate, so the ratio is exactly 2. DPGDownloader should report the 2.6 Gbps lane
rate when the DAC uses internal clocking with the 100 MHz reference on J61.

Any integer ratio works — if 2600 MHz is changed, pass the
actual rate to every command below via `--fs-dac`.

## 2. Generate and load the waveform

```bash
python -m calibration_loop.run_calibration gen --out waveforms
```

Record the printed parameter summary; it is needed if anything later disagrees.
With the defaults it reports a 199.99 MHz tone, a 32768-sample ADC loop and 512
impulses per loop.

Load `waveforms/impulse_dither.txt` into DPG Downloader. The format is the same
as the vectors already in use: one signed integer per line, no header, 65536
lines. Keep the `unsigned data` option **disabled**.

On the scope the output should be a 200 MHz sine with periodic narrow pulses on
top, one every 49.23 ns, about 12.3 ns wide, roughly 12 % of the sine amplitude. No
pulses means DPG is not looping the file; a flattened waveform means `unsigned
data` was left on.

## 3. Confirm the capture path

Using the existing flow: `dma -d`, `dma -w`, `udp` on the console, received by
`udp_receiver.py`. Confirm the board is at 192.168.1.10, the host at
192.168.1.100, port 6666, and note the COM port.

The first UDP transfer after boot often times out. Repeat the `udp` command.

## 4. Measure without driving anything

Do not close the loop yet. `probe` captures and estimates but touches no
hardware register:

```bash
python -m calibration_loop.run_calibration probe --uart COM3 --frames 10 --plot
```

It prints a line per frame and ends with five sanity checks. All five must pass
before going further; if any fails see the troubleshooting table.

`--plot` writes `probe_probe.png`. The left panel is `V[m]`, the averaged pulse
replica, and it should reproduce the trapezoid of the injected pulse, overlaying
the dashed ideal. If it is noise, alignment failed or the waveform is not what
the code expects — stop and fix that first.

## 5. Open-loop baseline

```bash
python -m calibration_loop.run_calibration bench --uart COM3 \
    --open-skew --iterations 30 --out data/openloop --stem openloop
```

This measures without driving the clock delay and gives the uncalibrated
baseline. The columns of interest in the CSV are `skew_mismatch_ps` and
`raw_difference_dbc`.

## 6. Closed loop

```bash
python -m calibration_loop.run_calibration bench --uart COM3 \
    --iterations 300 --out data/closedloop --stem run1
```

`g_B/g_A` should approach 1.00000, `dOffset` and `dSkew` should approach zero,
and `image` should fall. The run writes `run1.csv`, `run1_meta.json` and
`run1_learning.png`.

Then repeat with main-tone cancellation disabled, changing nothing else:

```bash
python -m calibration_loop.run_calibration bench --uart COM3 \
    --iterations 300 --no-cancellation --out data/closedloop --stem run2_nocancel
```

Convergence should be visibly slower. The two runs form a matched pair, so both
are needed.

## 7. Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| `capture failed (no UDP frame)` | Common on the first transfer after boot | Retry; check the firewall and that the host is 192.168.1.100 |
| `align margin` below 6 | The dither is not in the capture | Confirm on the scope that DPG is looping and the pulses are present; confirm `--fs-dac` matches the actual clock |
| `V[m]` plot is noise | Same, or the two clocks are not locked | Check the 10 MHz reference on J61 |
| `events` is 0 or 1 | Alignment failed | As above |
| `gB/gA` scatter above 0.02 | Not enough dither SNR | Regenerate with `--dither-scale 4000` and reload |
| `gainA` negative | Channel polarity unresolved | Record it — both branches may be inverted |
| Every iteration `rejected` | Estimates not trusted | Record the rejection reasons |
| `dSkew` stuck at hundreds of ps | Channel pairing or a four-sample framing slip | Stop and record it |
| `--super-fine` makes things worse | The firmware writes the wrong register | Drop the flag and use the default 1.725 ps step |

The super-fine issue is a one-line fix:
`ad9695_adc_super_fine_delay()` in `ad9695_api.c` writes to
`AD9695_CLK_FINE_DELAY_REG` (0x0112) instead of
`AD9695_CLK_SUPER_FINE_DELAY_REG` (0x0111), which is defined but unused. The
super-fine field is therefore never programmed and the fine field is clobbered.
After fixing and rebuilding, `--super-fine` enables the 0.25 ps step.

## 8. What to bring back

The whole `data/` directory, `probe_probe.png`, a scope capture of the loaded
waveform, the clock frequencies and power levels actually used, and a note of
anything that did not match this document.

## 9. Not possible in this configuration

True 2× interleaving needs channel B to sample half a period after channel A,
which is 384.6 ps at 1.300 GSPS, against an on-chip delay range of
192 × 1.725 + 128 × 0.25 = 363 ps. So these runs are in parallel mode: both
channels sample the same instant and the loop calibrates the mismatch between
them, which is a valid measurement in its own right.

Once an external clock delay of roughly 8 cm of coaxial length difference is
added, the same code runs the interleaved case with
`--interleaved --skew-target-ps 384.615`. Nothing else changes.
