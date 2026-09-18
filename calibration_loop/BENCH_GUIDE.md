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
With the defaults it reports a 199.375 MHz tone, an 8320-sample ADC loop and 64
impulses per loop. All three matter: the board's reference and its event
detectors are built around this geometry, so a 65536-point vector with
128-sample dither periods (512 impulses, 64-sample ADC slots) is **not**
interchangeable — it fails silently, with the probe reporting an alignment
margin near 3 and meaningless gains. See the note on `n_dac_points` in
`dither.py`.

Load `waveforms/impulse_dither.txt` into DPG Downloader. The format is the same
as the vectors already in use: one signed integer per line, no header, 16640
lines. Keep the `unsigned data` option **disabled**.

On the scope the output should be a 199.375 MHz sine with periodic narrow pulses
on top, one every 100 ns, about 24.6 ns wide, roughly 7 % of the sine amplitude.
No pulses means DPG is not looping the file; a flattened waveform means
`unsigned data` was left on.

## 3. Confirm the capture path

Using the existing flow: `dma -d`, `dma -w`, `udp` on the console, received by
`udp_receiver.py`. Confirm the board is at 192.168.1.10, the host at
192.168.1.100, port 6666, and note the COM port.

The first UDP transfer after boot often times out. Repeat the `udp` command.

Placeholder note: the examples below write `COM3` for the console port, which is
where this guide was first written. On this bench the board is on **COM5** — that is
what §6 and the read-only tools use, and it is the default in `tools/`.

## 4. Measure without driving anything

Do not close the loop yet. `probe` captures and estimates but touches no
hardware register:

```bash
python -m calibration_loop.run_calibration probe --uart COM3 --frames 10 --plot
```

It prints a line per frame and ends with five sanity checks. All five must pass
before going further; if any fails see the troubleshooting table.

The probe also prints the session **dither polarity anchor**. It takes a majority
vote over the complete capture batch because the sign of one low-SNR frame can
flip even though the analog path has not changed. This bench normally reaches a
`-1` consensus relative to the generated vector. The anchor scales the
estimator's polarity *model*; the captured data is never flipped, because the DC
offset is a measurement in ADC codes and inverting the record would invert it.
A frame whose fit disagrees with the anchor keeps its fitted magnitude but has
its sign pinned to the batch consensus.

Gain and offset are fitted jointly over all complete event windows in the batch.
The reported gain-ratio uncertainty is a delete-one-frame jackknife standard
error. It uses every captured frame and is stable against an unlucky split of
ten frames into two arbitrary groups of five.

The `swap` column reports captures where the half-group de-frame candidate was
detected and replaced by the aligned pair. `False` is the normal reading; `True`
means that frame's raw winner was four samples out, so the two channels no longer
shared one loop position `n0`. To watch that quantity directly it must be ~1 and
not ~0.756 — `python tools/live_before_after.py --uart COM3` prints it per frame
alongside the gain ratio.

`--plot` writes `probe_probe.png`. The left panel is `V[m]`, the averaged pulse
replica, and it should reproduce the trapezoid of the injected pulse, overlaying
the dashed ideal. If it is noise, alignment failed or the waveform is not what
the code expects — stop and fix that first.

## 5. Open-loop baseline

```bash
python -m calibration_loop.run_calibration bench --uart COM3 \
    --open-skew --iterations 30 --out calibration_out/openloop --stem openloop
```

This measures without driving the clock delay and gives the uncalibrated
baseline. The columns of interest in the CSV are `skew_mismatch_ps` and
`raw_difference_dbc`.

## 6. Closed loop

Two writes come first. Both go through the firmware transaction, the only path with
an ACK and a readback:

1. **Neutral-initialize once after every bring-up.** Do not skip it: it cuts the
   per-frame skew scatter from 18.9 ps to 3.1 ps, and the first transaction
   legitimately moves the operating point by ~57 ps.
2. **Park at the same code before every repeat run**, because a run that inherits
   the previous run's converged code 31 (about -10 ps) is not a matched pair with
   one that started from the neutral code 24 (about -44 ps).

One command does both: it reads the current code with a zero-step transaction (which
is also the neutral initialization), then walks one code at a time and refuses to
continue if any ACK is not `result=OK`.

```bash
python tools/skew_park.py --uart COM5 --code 24 --verify-frames 20
```

Then the three-way loop:

```bash
python -m calibration_loop.run_calibration bench --uart COM5 \
    --allow-skew-writes --iterations 300 \
    --out calibration_out/closed_loop --stem run1_300
```

`--allow-skew-writes` is what makes this the three-way loop: per-frame digital
gain/offset plus the batch skew actuator. Without it the run measures skew and never
drives it, which is the `--open-skew` case above. The run writes `<stem>.csv`,
`<stem>_meta.json` and `<stem>_learning.png`.

**Give every session a unique stem or directory.** `save()` overwrites by stem: on
2026-09-17 a later 100-iteration run reusing `--stem run1` replaced the 300-capture
`run1.csv` (252,730 → 84,984 bytes) together with its plot. The CSV is a run's only
complete record — the console log keeps just the printed columns, and
`calibration_out/_recover_run_from_log.py` rebuilds those from it, which is how that
300-run was recovered. Date the directory, e.g.
`--out calibration_out/closed_loop_2026-09-17`.

**`--iterations N` means N qualified samples.** A capture the acceptance filter
rejects — about 20 % on the bench, nearly all `align_margin` on torn UDP frames — is
logged with its reason and then retried, so it does not shorten the learning curve: a
300-sample run takes roughly 370 captures there and then, about 4 minutes. Rejected
rows stay in the CSV, marked, for forensics; they are excluded from the learning plot
and from every sample count, and the meta JSON records `samples_qualified`,
`captures_attempted` and `captures_rejected` so the size of a run is never inferred
from the row count. A bench that rejects everything stops after 20 in a row rather
than spinning.

**N needs to be at least ~200.** Both 300-sample runs entered the 10 ps deadband at
qualified sample 159, and a 100-sample run stops part-way down the descent — measured
at -24 ps and -30 dBc, where the full run reaches -9.3 ps and -38.6 dBc.

Skew stays a **batch** decision — 20 accepted frames, then at most one control code,
and only outside the 10 ps deadband — so a 300-capture run makes roughly a dozen
decisions and moves on the order of seven codes. A run whose code moves every two or
three frames is not this loop; that is the per-frame integrator the batch decision
replaced, and its skew estimate is chasing its own noise.

### What a good run looks like

Measured 2026-09-17, 300 captures per run — note that the gain numbers here predate
the gain-observable change described further down, so a run made now will show a
`tone_ratio` near 1.0000 and a `g_B/g_A` near 1.012 instead of the 2-3 % below:

| | run1 (cancellation on) | run2_nocancel |
|---|---|---|
| accepted / rejected | 243 / 57 | 236 / 64 |
| skew decisions / moves | 12 / 7 | 11 / 7 |
| control code | 24 → 31 | 24 → 31 |
| deadband first reached | iteration 159 | iteration 159 |
| converged `dSkew` | -9.32 +- 2.09 ps | -9.44 +- 1.37 ps |
| **raw A-B difference spur** | **-38.62 +- 1.88 dBc** | **-38.33 +- 1.34 dBc** |
| gap to `20log10\|2 sin(pi f dt)\|` | +0.22 dB | +0.30 dB |

Every transaction `result=OK`, `skew_action` mostly `move+1:OK` and then
`inside-deadband`, and `frame_sha1` **distinct on every accepted row** — a repeated
hash means the DMA shipped the previous buffer (the frozen-frame failure in
AGENTS.md) and the run has to be thrown away. A ~20 % rejection rate is normal and
is almost all `align_margin` on torn UDP frames.

Read the improvement from **`raw_difference_dbc`**, not `cal_difference_dbc`: the raw
channel difference is skew-limited (-38.6 dBc against -38.84 predicted), while the
corrected column is limited by the applied gain correction (-35.1 dBc). See the
AGENTS.md note on which A-B spur to quote.

### The gain loop, and which observable it integrates

Until 2026-09-17 the gain correction integrated the **dither** pulse-window ratio, and
that produced two measured problems.  `g_B/g_A` sat at 2-3 % (run1 2.19 % → 2.58 %,
run2 2.60 % → 3.07 %, against 0.04 % in the model), because the dither amplitude
carries ~2.3 % per-frame scatter that `mu_gain = 0.35` integrates; and because the
dither ratio and the *tone* ratio are two estimates of one mismatch that disagree by
~1.2 % on this bench, the loop seated its correction ~1.2 % away from the point that
nulls the tone — capping the corrected A-B spur at -35.1 dBc while the raw one reached
-38.6 dBc.

The gain loop now integrates the **tone** ratio (`--gain-observable tone`, the
default; `LoopOptions.gain_observable`): the coherent main-tone fit, which is what the
A-B difference spur at f_in is made of, and ~10x quieter (0.25 % per-frame scatter
against 2.3 %).  `--gain-observable dither` restores the old behaviour for a
comparison run.

Two things to expect from that, so neither reads as a fault:

* `tone_ratio` in the log — and the solid line in the gain panel — goes to 1.0000;
  that is the controlled observable;
* the `g_B/g_A` column settles at **about 1.012**, not 1.0000, because the dither
  route is no longer being nulled.  The plot draws both on purpose.

`tools/dither_vs_tone_gain.py` measures the disagreement directly.  The offline proof
that the choice fixes the corrected spur is `calibration_out/_gain_loop_test.py`: on a
model whose pulse gains reproduce the bench's 1.2 % disagreement, the dither route
leaves the tone 1.18 % out with the corrected spur at -38.0 dBc, while the tone route
nulls it and reaches **-46.9 dBc** — the model's own skew limit, 8.9 dB better.

### Then the matched control

```bash
python tools/skew_park.py --uart COM5 --code 24     # same starting point as run1
python -m calibration_loop.run_calibration bench --uart COM5 \
    --allow-skew-writes --no-cancellation --iterations 300 \
    --out calibration_out/closed_loop --stem run2_nocancel_300
```

Cancellation changed the digital result a little and the skew result not at all:
same seven moves, same codes, same deadband iteration, spurs 0.3 dB apart. What it
changes is the estimator noise the digital loop integrates (gain residual
2.6 % → 3.1 %, offset 0.85 → 1.08 LSB, rejections 57 → 64) and the corrected spur
(2.2 dB worse, the correction seating 2.05 % instead of 1.26 % from the
tone-nulling point; both runs predate the gain-observable change below). **Do not expect the skew curve to be visibly slower** — one
code per 20-frame batch is quantisation-dominated, not noise-dominated, which is the
point of the batch decision.

## 7. Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| `capture failed (no UDP frame)` | Common on the first transfer after boot | Retry; check the firewall and that the host is 192.168.1.100 |
| Bind fails with `WinError 10048` (address in use) | Another process holds the UDP port before the run even starts | The loop uses **6666**. On this host an unrelated system service (`lkcitdl.exe`) permanently holds **5002**, so scripts bound to 5002 need a different port; otherwise close the GUI/other capture script first |
| `align margin` below 6 | The dither is not in the capture | Confirm on the scope that DPG is looping and the pulses are present; confirm `--fs-dac` matches the actual clock |
| `V[m]` plot is noise | Same, or the two clocks are not locked | Check the 10 MHz reference on J61 |
| `events` is 0 or 1 | Alignment failed | As above |
| `gB/gA` uncertainty above 0.02 | Not enough dither SNR or too few probe frames | Repeat with `--frames 20`; if it persists, regenerate with `--dither-scale 4000` and reload |
| `gainA` negative | Polarity anchor measured wrong | Check the printed anchor sign and the `align margin`; both channels must anchor together |
| `swap` is `True` on most frames | The tone advances close to a whole cycle per 8-sample group, so `\|corr\|` cannot separate the two clean-splitting phases | Record it — at that geometry the lag test cannot separate them either, so those frames may still be four samples out |
| Every iteration `rejected` | Estimates not trusted | Record the rejection reasons |
| `frame_sha1` repeats across rows | The DMA shipped the previous buffer: the capture landed inside the JESD re-sync window after a delay write | Throw the run away, re-park, retry |
| `skew_action` is `low-yield:*` | Fewer than 40 % of the batch survived the acceptance filter, so no write is justified | A capture run is degraded (half-synced link, dropped datagrams); nothing was moved, deliberately |
| `skew_action` is `aborted` | Skew actuation latched off — a move shifted the error the wrong way, or a transaction never ACKed | Read `skew_abort` in the run's `*_meta.json`. Nothing is retried by design, so re-park and re-run; if it recurs, the actuator direction disagrees with the calibration |
| `g_B/g_A` settles near 1.012, not 1.000 | Expected, not a fault: the gain loop integrates the *tone* ratio, so the dither ratio parks at the measured dither-vs-tone disagreement | Watch `tone_ratio` instead — that is the controlled observable |
| `tone_ratio` will not approach 1 even with `--gain-observable tone` | The tone amplitude could not be fitted on those frames and the loop fell back to the dither route | The `gain_source` column records which observable each sample actually used; check `align_margin` |
| `dSkew` stuck at hundreds of ps | Channel pairing, or pulse dispersion reshaping the replica | A four-sample framing slip is now detected and corrected (`swap` column); check the `V[m]` plot before changing anything |
| `--super-fine` makes things worse | The firmware writes the wrong register | Drop the flag and use the default 1.725 ps step |

The super-fine issue is a one-line fix:
`ad9695_adc_super_fine_delay()` in `ad9695_api.c` writes to
`AD9695_CLK_FINE_DELAY_REG` (0x0112) instead of
`AD9695_CLK_SUPER_FINE_DELAY_REG` (0x0111), which is defined but unused. The
super-fine field is therefore never programmed and the fine field is clobbered.
After fixing and rebuilding, `--super-fine` enables the 0.25 ps step.

## 8. What to bring back

The whole output directory — in particular both closed-loop CSVs with their
`*_meta.json` and `*_learning.png` — plus `probe_probe.png`, a scope capture of the
loaded waveform, the clock frequencies and power levels actually used, and a note of
anything that did not match this document.

Tee each run's console output to a file: the CSV is written only when the run
finishes, so the log is the only record of a run that dies part-way — and the only
thing left if a later run overwrites the CSV. On Windows PowerShell both `>` and
`Tee-Object` write **UTF-16LE with a BOM**, so anything parsing these logs has to
sniff the BOM instead of assuming UTF-8 (`calibration_out/_recover_run_from_log.py`
does exactly that).

```bash
python -m calibration_loop.run_calibration bench --uart COM5 --allow-skew-writes \
    --iterations 300 --out calibration_out/closed_loop --stem run1_300 \
    2>&1 | Tee-Object -FilePath calibration_out/closed_loop_run1_300.txt
```

Two more measurements belong in the same session, both read-only:

```bash
# does the state the run left behind hold?  prints the drift and the A-B spur
python tools/skew_stability_check.py --uart COM5

# do the dither and tone routes to the gain mismatch actually agree?
python tools/dither_vs_tone_gain.py --uart COM5 --frames 40 --save-frames
```

The read-only tools open COM5 and the UDP port exactly like a run does, so they
cannot share the bench with a run in flight — `WinError 10048` on bind means
something else (often another session's run) is already listening.

## 9. Not possible in this configuration

True 2× interleaving needs channel B to sample half a period after channel A,
which is 384.6 ps at 1.300 GSPS, against an on-chip delay range of
192 × 1.725 + 128 × 0.25 = 363 ps. So these runs are in parallel mode: both
channels sample the same instant and the loop calibrates the mismatch between
them, which is a valid measurement in its own right.

Once an external clock delay of roughly 8 cm of coaxial length difference is
added, the same code runs the interleaved case with
`--interleaved --skew-target-ps 384.615`. Nothing else changes.
