# Dither-only bench session — checklist and results

Run on **2026-09-19** on the ZCU102/AD9695 bench, per the 19 September direction: **no tone-based
calibration**, **a shorter dither**, **different fixed settings recorded long enough**, and first
confirm that gain, offset and skew can be detected reliably. Each section keeps its procedure next
to its measured result; §5 records the traps this session paid for.

Two sections were added off-bench on **2026-09-20** and are not results from that session:
**§F** (the full-scale amplitude ladder, waveforms generated and validated, procedure ready) and
**§7** (the tone-free *closed loop*, whose host-side routes and acceptance logic are now fixed and
proven offline — that is the "step 1" preparation).

## Session result

**All three axes are detected, each against a change made on purpose, all of it dither-only:**

| axis | known change | measured response | significance |
|---|---|---|---|
| skew | delay actuator +8 codes | −39.5 ± 2.0 ps = 4.94 ps/code (the actuator's own calibration says 4.8–4.9 ps/code) | 20σ |
| skew | delay actuator +1 code | −10.0 ± 2.1 ps — one code is resolved | 4.9σ |
| gain | full-scale step on one channel | `B/A mag` −6.08 % against −6.47 % nominal | 15.7σ |
| offset | chip's DC-offset calibration on one channel | `DC(B−A)` +7.93 / −8.16 codes over two toggles | ~130σ |

Every test moved one thing; the other two readouts stayed put.

**The shorter dither works, and the limit is the generator.** As the generated pulse went
32 → 16 → 8 → 6 samples, the replica the ADC sees narrowed 17.2 → 10.6 → 5.4 → 3.6 ADC samples with
no plateau, and 6 samples is the generator's floor (`edge ≥ 2`, `top ≥ 2`). What shortening buys is
the timing readout: the replica's slope rose ×3.6 and the per-frame timing scatter fell
44.9 → 22.5 ps, reaching **15.2 ps at 16000 LSB** — a 29× steeper replica than the
32-sample/2000 LSB waveform stage C ran with, which would put one actuator code at ~10σ instead of
4.9σ. Amplitude is linear: a commanded ×4/×8 gave ×4.00/×8.04.

**On the record:**

- drift at a fixed actuator code across sessions is **~20 ps — more than one code** — so a one-code
  resolution claim holds within a session only;
- the gain figure carries a **~2 % amplitude-dependent systematic** (B's replica grows ~1 % less than
  A's over 8× amplitude), so gain numbers are quoted at a stated dither amplitude. The known 6.5 %
  step is unaffected because both sides use the same setting;
- the raw readouts are **not exactly orthogonal**: the actuator code shifts the gain readout by
  +0.21 %/code and the DC readout by −0.12 codes/code, and a full-scale change shifts the DC readout
  by −3.8 codes. The staged design varies exactly one thing for that reason.

---

## 1. What is already established off-bench, and how to reproduce it

Every item below was verified against known ground truth, so it does not need bench time —
it is the reason the session is designed the way it is.

| finding | evidence | reproduce |
|---|---|---|
| **Sign-blind folding recovers nothing.** The polarity train is balanced pseudo-random ±1, so a plain period fold cancels the pulses: 2.4 codes peak-to-peak folded against a 13-code injected pulse. | synthetic frames built from `build_dac_waveform` | `python calibration_out/_dither_raw_test.py` |
| **The polarity train doubles as a sync word.** `align_to_loop` recovers the capture's loop position exactly (n0 = 3000 for a frame starting at 3000), margin ≈ 15. | same | same |
| **Polarity-corrected folding returns the exact injected pulse**, so the replica amplitude is a valid raw gain readout (B/A) and the record mean a valid raw offset readout (B−A). | same: folded replica equals `pulse()` on the ADC grid, sample for sample | same |
| **Timing cannot be read as a replica shift.** The sampling grid is 769 ps; a 44 ps skew changes the sampled replica values by about 1 %. Cross-correlation returns −5…+19 ps for a true 44 ps, and the integer alignment absorbs half-sample delays entirely. | same: known-delay table | same |
| **Timing must use the slope route** (`<A−B, d/dt> / <d/dt, d/dt>`, slope from the injected waveform, not fitted). Differential accuracy against truth: 0 → +44 ps reads +44 ps; 0 → +385 ps reads +352 ps; 0 → +769 ps reads +681 ps. A systematic ≈ −57 ps offset is present, so **read differences between states, not absolutes**. | same | same |
| **Per-frame timing scatter is what decides recording length.** 150–270 ps per frame at 8 codes of noise with 2000 LSB pulses. Precision goes as σ/√N, so 4.8 ps (one actuator code) needs ≈1000 frames at 2000 LSB, ≈60 frames at 16000 LSB (−12 dB more dither, 4× the amplitude). | same | same |

Consequence: the raw layer can confirm **gain and offset** directly and can confirm **timing
only as a difference between fixed states**, averaged over many frames. It cannot resolve
absolute ps-level timing per frame.

### 1a. Which gain route to quote (settled on the bench 2026-09-19)

Three routes on the *same* 100 saved frames at the code-24 baseline:

| route | B/A | per-frame sd | polarity convention enters? |
|---|---|---|---|
| estimator `gain_ratio` (`dither_response_test.py`) | **0.9938** | 0.0258 | yes (session sign + anchor) |
| folded replica ratio (`rep B / rep A`) | 0.9643 | 0.0403 | yes |
| **sign-free per-event magnitude (`B/A mag`)** | **0.9645** | 0.0382 | **no** |

The two replica-based routes agree to **0.02 %**; the estimator sits **3 % away**, and its
per-frame value does not track the replica amplitude at all (`corr(gain_ratio, magnitude) =
−0.055`, against `corr(mag_A, mag_B) = +0.900` for the genuine common-mode behaviour of the two
channels). So **quote `B/A mag`**, and treat the estimator's dither gain as biased — it is
driven by alignment/polarity balance rather than by the pulse amplitudes.

`pin_polarity` is *not* the cause (pin on/off agree at 0.998 correlation), and the metric is
genuinely data-driven: splitting 100 frames into five chunks moves its mean over
0.980–0.999. Reproduce with `python calibration_out/_gain_route_test.py`.

The sign-free route also passes a linearity check that the others cannot: the positive and
negative events must give the same magnitude, and they do — channel A `+/- = 1.0014`, channel B
`1.0004` — so there is no saturation and no polarity dependence hiding in the number.

### 1b. Bench state measured 2026-09-19 (`w32adc_e16_t32_amp2000`, dither-only)

| quantity | value |
|---|---|
| folded replica | **48.7 / 48.3 codes** pk-pk (A / B) — far above the 7–18 codes of the archived tone+dither captures |
| alignment | **20/20, 100/100 frames**, margin 6.5–6.6 |
| timing (slope route) | **+80.4 ± 30.6 ps** per frame (20 frames) / **+80.1 ± 50.7 ps** (100 frames) — the mean is stable across sessions, the scatter is not |
| gain `B/A mag` | 0.9645 ± 0.0382 per frame |
| offset `DC(B−A)` | −3.82 ± 0.67 codes per frame (drifted from −6.41 in an earlier session) |
| known interference | a **325 MHz = fs/4** clock component (the board has a 325 MHz clock), ≈16–17 codes rms, present with the DPG stopped and across reboots |
| a real data flaw | one word per frame lands on a fixed value (1087 codes) in **slot 7 of the 8-word group** — slot std 68.3 against 12–19 for the other seven. It corrupts about one fold window in eight, so it does not block the analysis, but it is why `raw_frame_probe`'s chB max is constant |

Two consequences worth carrying: the fs/4 line is **coherent**, so it does not average away —
but 130 mod 4 = 2, so successive fold windows see it with opposite phase and it largely cancels
in the fold, which is why the replica comes out clean. And `raw_frame_probe.py` reads the *high
16 bits of each 32-bit word*, i.e. every other sample, so its channel rms values are a decimated
view: use `live_frame_table.py` / `dump_raw_words.py`, or the folded replicas, when the question
is which channel is bigger.

Also measured: `neutral_initialized` reads `YES` on the **first** zero-step transaction after a
boot and `NO` on later ones, so the flag reports the action taken rather than the state. A
single zero step does **not** perform the neutral preparation (see stage G).

**Cross-session drift is real, and it is larger than one actuator code** — measured 2026-09-19 at
code 24 on either side of a board reboot, after a neutral init in both cases:

| quantity | c00a (1000 frames) | check_b (100 frames) | change | significance |
|---|---|---|---|---|
| timing `dt` | +80.3 ps | +58.7 ps | **−21.6 ps** | **4.1σ** |
| `DC(B−A)` | −3.78 codes | −1.45 codes | **+2.3 codes** | **26σ** |
| `B/A mag` | 0.9640 | 0.9720 | +0.8 % | 2.1σ |

21.6 ps is ≈ 4.5 actuator codes, i.e. **more than the one-code step the resolution test is trying
to demonstrate.** Two consequences, both binding:

- **Never use an earlier session's state as a baseline.** Put the baseline inside the same run as
  the steps (the tool captures at the starting code first), so drift cannot enter the slope.
- **A one-code resolution claim is only meaningful within a session.** State that explicitly
  rather than quoting a cross-session number.

Channel B also came back healthy after the reboot (replica 47.33 codes, `B/A mag` 0.9720) where
the corrupt run had read 94.77 / 20.5, so the earlier failure was a link/data-path state, not a
broken analog path.

## 2. Waveforms to load (already generated and validated)

`waveforms/pulse_ladder/` — all six pass `check` 6/6 at 1.3 GSPS. Tone is at 0 LSB in all of
them (`--amp-dbfs -120`).

| file | pulse | duty | dither amplitude |
|---|---|---|---|
| `w32adc_e16_t32_amp2000.*` | 32 ADC samples (24.6 ns) | 24.6 % | 2000 LSB (6.1 % FS) |
| `w16adc_e08_t16_amp2000.*` | 16 samples (12.3 ns) | 12.3 % | 2000 LSB |
| `w08adc_e04_t08_amp2000.*` | 8 samples (6.2 ns) | 6.2 % | 2000 LSB |
| `w06adc_e04_t04_amp2000.*` | 6 samples (4.6 ns) | 4.6 % | 2000 LSB |
| `w06adc_e04_t04_amp8000.*` | 6 samples | 4.6 % | 8000 LSB (24.4 % FS) |
| `w06adc_e04_t04_amp16000.*` | 6 samples | 4.6 % | 16000 LSB (48.8 % FS) |
| `w06adc_e04_t04_amp24000.*` | 6 samples | 4.6 % | 24000 LSB (73.2 % FS) |
| `w06adc_e04_t04_amp30000.*` | 6 samples | 4.6 % | 30000 LSB (91.6 % FS) |

The last two were generated for stage F (the full-scale ladder, below) and pass `check` 6/6 at
1.3 GSPS; 30000 LSB is within 8.4 % of the DAC's full scale, so the DAC is the element closest to
a limit in that state and the test is really "where does the amplitude chain stop being linear".

The generator's own floor is `edge ≥ 2` and `top ≥ 2` ADC samples (flat top needed for the
gain/offset statistics, ramp needed for skew), so 6 ADC samples is the shortest pulse
expressible, not a choice.

**Only the TXT goes to the DPG.** The JSON never leaves the host — the pattern generator takes
one signed integer per line and nothing else. The JSON is the host's description of the same
waveform (impulse period, event offset, pulse geometry, and the pseudo-random polarity
sequence), and the host tools need it because that polarity train is what locates each capture
inside the DPG loop. So:

- keep the pair together — the generator writes `stem.txt` and `stem.json` side by side, and
  uploading happens TXT by TXT, so the JSON must be passed explicitly:
  `--waveform-json waveforms/pulse_ladder/<stem>.json`;
- a stale JSON is the dangerous case: the capture is fine, the alignment template is wrong, and
  the fold window sits next to the pulses instead of on them. Nothing in the data would tell
  you, so **verify after every upload**:

| what | check | distinguishes |
|---|---|---|
| period | `python tools/geometry_id.py --uart COM5 --frames 4` | every period in the repo's history |
| pulse width | same run (it measures the width at the winning lag) | the 32 / 16 / 8 / 6-sample ladder entries |
| amplitude | `dither_raw_evidence.py` replica peak-to-peak: ×4 for amp8000, ×8 for amp16000 | the three 6-sample amplitude variants |
| polarity/offset | alignment margin in the same tool's output (≈15 when right, collapses when wrong) | a JSON from the wrong waveform |

Note the limit: all six ladder files share the same 130-sample period, so `geometry_id` alone
cannot name the loaded file — width plus replica amplitude is what closes it.

Order on the bench: `w32adc` (reference, comparable with all previous data) → `w08adc` →
`w06adc` → `w06adc_amp8000` → `w06adc_amp16000`. Record the replica peak-to-peak at each step:
that curve is the answer to "the dither should be shorter", and its knee is where the analog
path stops passing a shorter pulse.

```
python -m calibration_loop.run_calibration check \
  --waveform-json waveforms/pulse_ladder/w06adc_e04_t04_amp8000.json --adc-rate 1.3e9
```

## 3. Session order — 17 states, roughly 1.5 h

**Not a cross product.** 6 pulse widths × 7 IFC settings is 42 states and answers nothing
extra: IFC scales the code grid without touching the pulse shape or the channel-to-channel
mismatch, and a mismatch needs a *differential* setting (fix A, move B) — moving both
channels together cancels. So each stage varies exactly one thing.

| stage | varies | states | frames each | time | answers | status |
|---|---|---|---|---|---|---|
| **G** | nothing — gates | 2 | 1 frame each | ~5 min | is the plan still valid at all | **done 2026-09-19** (G1, G2); only the IFC↔JESD check of G3 remains, and it rides along with stage A |
| **C** | actuator code 24 / 32 / 25 in one run | 3 | 1000 | ~35 min | can timing resolve one code (4.8 ps) | **done 2026-09-19 — yes: 8 codes = 20σ, 1 code = 4.9σ** (see §C) |
| **A** | IFC on one channel: 0x0C → 0x0D | 2 | 500 | ~15 min | is 0x1910 a usable known gain step, and does the write disturb the link | mechanism proven in G; the recovery measurement is what remains |
| **B** | pulse width, IFC unchanged | 6 | 200 | ~15 min | where the analog path stops passing a shorter pulse | **done 2026-09-19 — no plateau: FWHM 17.2 → 3.6 samples, slope ×3.6, 6 samples is the generator's floor** (see §B) |
| **F** | dither amplitude to full scale (16000 → 24000 → 30000 LSB) | 3 | 300 | ~20 min | where the amplitude chain compresses, and whether the timing readout keeps improving to the top | **waveforms generated and validated; procedure below** (see §F) |
| **D** | IFC on B: 0x0C / 0x0D / 0x0B | 3 | 500 | ~20 min | is the gain readout linear over more than two points | optional: A already gives a two-point proportionality |
| **E** | B's internal DC-offset calibration on / off | 3 | 500 | ~15 min | is there a known offset step, and does the readout follow it | **done 2026-09-19 — yes: +7.93 / −8.16 codes, ~130σ** (see §E) |

**Why C comes first.** Skew is the axis with no fallback once the tone is gone, and the bench has
now shown the slope route resolves it (measured 2026-09-19 at the code-24 baseline with
`w32adc_e16_t32_amp2000`: `dt = +80.4 ± 30.6 ps` per frame over 20 frames, `+80.1 ± 50.7 ps`
over 100 — the mean is stable across sessions, the scatter is not). Stage B could *improve* C's
precision (a shorter pulse has a steeper slope for the same timing shift), but it cannot change
whether C's claim holds, so C is not gated on it — and if B later shows a clearly better
waveform, re-running C's two key states is cheap.

Stage A is ordered after C because it is the cheap one and its first step also closes the last
open item of G (does an IFC write disturb the JESD link). Stages D and E are the multi-point
extensions, and can be dropped without weakening the headline claim.

Total 17 states. Frame counts are *starting* values — re-size them from a measured σ as
described at the end of this section. On the bench the measured per-frame scatters at the code-24
baseline with `w32adc_e16_t32_amp2000` (20 frames) were: gain 0.0420, offset 0.83 codes,
timing 30.6 ps, against a 48.7-code replica and alignment margin 6.5.

### G1 — link and waveform gates, before spending any bench time

```
python tools/raw_frame_probe.py --uart COM5          # link + capture health, first UDP after
                                                     # boot often times out: retry once
python tools/geometry_id.py --uart COM5 --frames 4    # loaded period + measured pulse width,
                                                     # against the JSON you are about to pass
                                                     # (see the table in section 2 for what it
                                                     # can and cannot tell apart)
adc -cal diagnose skewprep fullprep                  # console: the neutral preparation
                                                     # (analog + digital + JESD + actuator).
                                                     # `combined` maps to the same mode; the
                                                     # single-step modes (jesd|ctrl|analog|
                                                     # digital|analogdigital|enableafter|
                                                     # actuator) isolate one part of it.
adc -cal skew step 0                                 # console: neutral init + baseline
```

**Do not assume the zero-step transaction performs the neutral preparation.** Measured
2026-09-19: `adc -cal skew step 0` returned `result=OK`, `code_before = code_after = 24`, and
`capture_generation` advanced 15 → 16 (so the frame was fresh, not the frozen-frame failure
mode) — but `neutral_initialized=NO`. That is consistent with the old exports: the "neutral
code 24" set in `professor_data_min/` measures **−24.6 dBc**, which is the *un-initialized*
figure that AGENTS.md records improving to −38.6 dBc after a genuine neutral init. So run
`skewprep fullprep` first, then the zero step, and require `neutral_initialized=YES` before
calling the state a baseline.

Each delay transaction also resets the JESD link (expected and documented). A half-synced link
produces exactly the class of garbage the gate exists to catch, so after any JESD reset re-run
`raw_frame_probe.py` and `geometry_id.py` before concluding anything about the analog path.

### G2 — what a failed gate looks like (measured 2026-09-19)

Recorded so it is not mistaken for a tool fault, and so the first move is the right one.

**The gross failure, and what cleared it.** A `adc -cal skew step 0` transaction (which resets
the JESD link as a side effect) changed the picture completely:

| observation | before the reset | after |
|---|---|---|
| chB/chA rms | **11×** (1951 / 174) | **1.2–1.35×** (230 / 191) |
| chB min/max | −8034.2 / 1737.2, **byte-identical every frame** | 36…1087 / −32…1087, varying |
| chB peak frequency | wandering (650.0 / 632.2 / 642.4 MHz) | stable at 325.000 MHz, same as A |
| firmware's own estimator | `Skew measurement: FAILED`, `Dither estimator INVALID` | — |

So the deterministic-huge-signal state was a **link/data-path state**, not registers and not a
broken analog path. Operational rule: when the gate shows catastrophic data, perform a link
reset and re-probe **before** chasing registers. (Frame 0 failing in `raw_frame_probe.py` while
later frames succeed is the documented first-UDP timeout, not this.)

**The remaining fs/4 line is small — which rules out a digital test pattern.** An injected test
pattern fills the scale, whereas these signals sit at tens of codes: chA p-p ≈ 73 codes, rms
≈ 48 codes, against ±8192. So `0x0550` is a low-probability suspect after all; check it, but do
not start there.

**Leading explanation for the residual fs/4 line: a 4-sample-periodic error acting on the
channel DC.** A periodicity of 4 samples is exactly the `[A A A A B B B B]` grouping granularity,
and a 4-sample-periodic error multiplied by a DC level puts a line at exactly fs/4:

| channel | DC (codes) | fs/4 rms (codes) | ratio |
|---|---|---|---|
| A | ≈ 146 | ≈ 48 | 0.33 |
| B | ≈ 150 | ≈ 58 | 0.39 |

This is falsifiable and cheap to test: the fs/4 amplitude must **track the DC**. Change the DC
(e.g. toggle B's `0x0701` internal DC-offset calibration) and re-probe — if the line scales with
the DC, it is a framing artifact rather than anything injected.

**Order of checks when the gate looks like this**

1. **Stop the DPG pattern and re-probe.** One capture separates the two families of cause: the
   line persists → it is not pattern content; the line disappears → the DPG is playing a
   different file than the one whose JSON you are passing.
2. **The DC test above** — falsifies or confirms the framing hypothesis.
3. **Re-upload our own vector** (`waveforms/pulse_ladder/w32adc_e16_t32_amp2000.txt`, tone at
   0 LSB) and re-probe. The correct state has **no in-band tone at all**, a 130-sample impulse
   train on **both** channels, and comparable rms on the two.
4. Only then, the per-channel digital registers, read the same way as the IFC check:
   `spi -w 0x0008 1` → `spi -r 0x0550` (test mode, expect 0x00; 0x4 = ALT_CHECKERBOARD,
   0x7 = ONE_ZERO_TOGGLE → fs/2, 0xF = RAMP, 0x5/0x6 = PN) → `spi -r 0x0561` (data format,
   expect 0x01 two's complement, 0x00 = offset binary) → repeat with index 2 for channel B.
   A non-zero test mode, or the two channels disagreeing on 0x0561, explains a lot; if both
   read clean, the next move is a controlled re-bring-up (`adc -cal reset` or power-cycle) and
   re-reading the same four registers.

Note the tool limit that shows up in every `geometry_id` run here: periods 260 and 520 always
score exactly 0.000 because a 512-sample channel record cannot hold even one window at those
periods. That is a structural zero, not evidence that those periods are absent.

### G3 — the IFC gate, step by step

Menu route (for what the firmware offers): `adc -gain` enters `gain-cmd$`; type `IFC` for the
`gain-ifc$` prompt, where `status` prints register 0x1910, `set 1.59` writes it, `sweep` walks
the range and `back` returns. Valid values 1.36 / 1.47 / 1.59 / 1.70 / 1.81 / 1.93 / 2.04 Vpp;
smaller full-scale = larger digital amplitude. `sweep` is a smoke test only — it stores no raw
frames, so it is not evidence.

**Why the raw route is the one to use:** `ad9695_set_input_full_scale()` writes 0x1910 without
touching the channel selector, so `IFC set` moves whichever channels are currently selected.
Register 0x1910 holds the full-scale code in its low nibble (`AD9695_INPUT_FS_MASK = 0x0F`):
1.36 → 0x0A, 1.47 → 0x0B, 1.59 → 0x0C, 1.70 → 0x0D, 1.81 → 0x0E, 1.93 → 0x0F, 2.04 → 0x00.
The console's register route is `spi -r <addr>` / `spi -w <addr> <data>`, addresses parsed
base-0.

> **Firmware bug, measured 2026-09-19: `spi -w` silently truncates the data value to three
> characters.** `handle_spi_cmd()` declares `char data_str[4]` and `next_tok()` copies
> `len - 1` = 3 characters, so `0x0D` becomes `"0x0"` → `strtol` → **0**, and the console still
> prints `Command Success: Wrote 0x00 to 0x1910`. Both `0x0D` and `0x0C` landed as 0x00 in a
> live session, which is how this was found. **Write data values in DECIMAL**; addresses are
> fine (`addr_str[8]` holds `0x1910`). A fix is `data_str[4]` → `data_str[8]` in `butils.c`,
> but it needs a Vitis rebuild — decimal is the workaround until then.

**0x0008 is a selection bitmask, not a single index** — measured: it read **0x03** on a fresh
board, and the firmware's own `adc_skew_ad9695_select()` treats it as a mask (`selection & 0x03`,
bit 0 = channel A, bit 1 = channel B). So **0x01 = A, 0x02 = B, 0x03 = both**, and because the
firmware's `IFC set` path never writes 0x0008, **`IFC set` is a broadcast: it moves both channels
together and can never create a mismatch.** That is why stage D has to use the `spi` route with
an explicit single-channel index.

Measured baseline before anything else was changed: **both channels read 0x1910 = 0x0C**
(1.59 Vpp), i.e. equal — so any channel-to-channel amplitude difference seen in the captures is
*not* an IFC setting and has to be explained by the analog path instead.

**Per-channel addressing: CONFIRMED (2026-09-19), by differential write.** The write value was
truncated to 0x00 by the bug above, which made the test accidentally cleaner: after
`0x0008=1` + a write to 0x1910, index 1 read back **0x00** while index 2 still read **0x0C** —
a write selected for A did not touch B. So 0x1910 is per channel and the bitmask works; stage D
stands.

**Register state to restore after that session: both channels were left at 0x00 (2.04 Vpp)**,
because both writes (including the intended restore) landed as 0x00. That is 2.04/1.59 = 1.28×
*larger* full-scale, i.e. digital amplitudes **22 % smaller** than every capture taken before —
so do not compare post-session captures with pre-session ones until this is restored.

```
# 1. baseline — what is selected, and what does each channel hold?
spi -r 0x0008            # 0x01 = A, 0x02 = B, 0x03 = both (measured 0x03 on a fresh boot)
spi -w 0x0008 1          # select A
spi -r 0x1910            # A's code, low nibble (measured 0x0C = 1.59 Vpp)
spi -w 0x0008 2          # select B
spi -r 0x1910            # B's code (measured 0x0C)

# 2. the decisive step — a *differential* write.  Two equal read-backs prove nothing:
#    they are equally consistent with one global register.  DECIMAL values only.
spi -w 0x0008 1
spi -w 0x1910 13         # move A only: 13 = 0x0D = 1.70 Vpp.  Read-modify-write if the
                         # read-back had other bits set.
spi -r 0x1910            # expect 0x0D
spi -w 0x0008 2
spi -r 0x1910            # 0x0C -> per channel, stage D stands
                         # 0x0D -> global/broadcast, no per-channel knob: delete stage D

# 3. restore both to the baseline
spi -w 0x0008 3
spi -w 0x1910 12         # 12 = 0x0C = 1.59 Vpp
spi -r 0x1910            # confirm 0x0C before capturing anything
```

Full-scale codes are a 4-bit field, so the whole range is expressible in decimal:
1.36 → `10`, 1.47 → `11`, 1.59 → `12`, 1.70 → `13`, 1.81 → `14`, 1.93 → `15`, 2.04 → `0`.

Note the direction: **smaller full-scale = larger digital amplitude**, so from the measured
baseline of 0x0C (1.59 Vpp) the natural step is 0x0C → 0x0D, which makes that channel *smaller*
by 1.59/1.70 = **−6.5 %**. Read the step size off the codes, not off the nominal Vpp.

```
# 4. does the write disturb the JESD link?
python tools/raw_frame_probe.py --uart COM5 --frames 3     # baseline, before the write
#   ... do the spi -w 0x1910 of step 2 ...
python tools/raw_frame_probe.py --uart COM5 --frames 3     # immediately after
python tools/frame_consistency.py --uart COM5 --frames 6   # consecutive frames must differ
```

**Decision.** Frames come back normal → IFC is safe to change between states, no re-sync
needed. Dead buffer, timeouts, or byte-identical consecutive frames → the write disturbs the
link: after every later IFC change, re-sync, re-capture, and re-verify before trusting the
state. (A byte-identical repeat is the documented failure mode of a capture taken while the
link is re-syncing: the buffer keeps the previous frame and the metrics repeat exactly.)

**What to do when a channel asymmetry shows up in the captures.** Check the two 0x1910
read-backs first (above). If they are equal, the asymmetry is *not* a gain setting. Then look at
the hardware before measuring anything: the observed A/B rms ratio of **1.35–1.44 is 2.6–3.2 dB
— almost exactly a 3 dB pad**, so check both input paths by eye for an attenuator, a different
splitter port, or a different termination. Only if the paths look identical is it worth swapping
the two input cables at the ADC and re-probing *with a signal present* (a ratio that follows the
cable is the input network; one that stays with the channel is the board/ADC path).

### A — is IFC a known gain step

The only known gain change available on silicon, and the only ground truth for the gain axis.
Two states, 500 frames each, **actuator untouched** — the raw gain readout carries a
+0.21 %/code dependence on the actuator code (§C), so the code must stay fixed across the pair
and be written into the log.

```
# 0. re-read the IFC state first: a reboot may have changed it, and both channels must start equal
spi -r 0x0008            # 1 = A, 2 = B, 3 = both
spi -w 0x0008 1
spi -r 0x1910            # A's code, low nibble
spi -w 0x0008 2
spi -r 0x1910            # B's code -- if it differs from A, equalise first and say so

# 1. baseline capture at the current setting, its own output directory
python tools/dither_response_test.py --uart COM5 --frames 500 \
    --out calibration_out/response/A_base

# 2. move channel B only, DECIMAL value (0x0D would be truncated to 0x00 -- see the G3 note)
spi -w 0x0008 2
spi -w 0x1910 13         # 13 = 0x0D = 1.70 Vpp
spi -r 0x1910            # confirm the read-back is 0x0D

# 3. does the write disturb the JESD link?  (this closes the last open item of G3)
python tools/raw_frame_probe.py --uart COM5
python tools/frame_consistency.py --uart COM5 --frames 6

# 4. stepped capture
python tools/dither_response_test.py --uart COM5 --frames 500 \
    --out calibration_out/response/A_step

# 5. compare
python tools/dither_raw_evidence.py \
  --state base=calibration_out/response/A_base/raw \
  --state step=calibration_out/response/A_step/raw \
  --waveform-json waveforms/pulse_ladder/w32adc_e16_t32_amp2000.json

# 6. restore B and confirm
spi -w 0x0008 2
spi -w 0x1910 12         # 12 = 0x0C = 1.59 Vpp
spi -r 0x1910
```

Expected `B/A mag` change: **−6.5 %** = 1.59/1.70, i.e. *smaller*, because a smaller full-scale
means a larger digital amplitude. At 500 frames the magnitude ratio has a per-state standard
error of ≈0.15 %, so the 6.5 % step is ~30σ — this stage cannot fail for lack of precision; it
fails only if IFC is not a pure gain, in which case record the measured ratio and use it rather
than the nominal one. **Quote `B/A mag`, never the estimator's `dither gain`** (it reads ~3 %
high, §1a).

Read step 3 as: normal frames → IFC is safe to change between states with no re-sync; dead
buffer, timeouts, or byte-identical consecutive frames → every later IFC change needs a re-sync
and a re-verify before the state can be trusted.

### B — pulse width: results in (2026-09-19)

| state | nominal pulse | `rep A` | **FWHM A** | **slope A** | `dt` scatter | A−B replica |
|---|---|---|---|---|---|---|
| w32 | 32 samples | 47.8 | **17.2** | 6.38 | 44.9 ps | 2.99 |
| w16 | 16 | 52.1 | **10.6** | 14.44 | 29.4 ps | 4.73 |
| w08 | 8 | 51.7 | **5.4** | 22.27 | 22.5 ps | 9.38 |
| w06 | 6 | 48.7 | **3.6** | 23.26 | 25.7 ps | 9.40 |
| w06 amp8000 | 6, ×4 amplitude | 194.9 | **3.6** | 92.24 | 16.2 ps | 39.6 |
| w06 amp16000 | 6, ×8 amplitude | 391.3 | **3.6** | 184.87 | 15.2 ps | 78.9 |

**The analog path is not the limit — a shorter dither does help.** FWHM tracks the nominal pulse
width all the way down: 17.2 → 10.6 → 5.4 → 3.6 ADC samples for 32 → 16 → 8 → 6, against ideal
values of 24 / 12 / 6 / 4. The measured widths sit below ideal (by 6.8 samples at 32, only 0.4 at
6), consistent with the AC-coupled path differentiating the long pulse's flat top while passing the
short ones nearly unchanged. No plateau appears, and 6 samples is the generator's floor, so the
answer to "the dither should be shorter" is: **as short as the generator can express — the
remaining limit is the generator, not the analog path.**

**The point of shortening is the timing readout, and it improves by 1.8×.** The replica's peak slope
rises ×3.6 (6.38 → 23.26 codes/sample) and the per-frame timing scatter falls from 44.9 ps to
22.5–25.7 ps. Adding amplitude, which removing the tone freed up, multiplies the slope linearly and
brings the scatter to 15.2 ps per frame — one actuator code is then 10–15σ at 1000 frames, against
3.4σ under the conditions stage C ran in.

**Amplitude linearity confirmed:** `rep A` 48.69 → 194.88 → 391.33 for a commanded ×4 / ×8 (ratios
**4.00 / 8.04**), and the slope 23.26 → 92.24 → 184.87 (3.97 / 7.95). The chain is linear over an 8×
range and 391 codes is 4.8 % of ±8192, so headroom remains. This validates the "replica amplitude
scales linearly with `dither_scale_lsb`" assumption the frame-count table rested on.

**Two caveats that must travel with these numbers:**

1. **`dt` is not comparable across waveforms.** The absolute value moved 59 → 95 ps between w32 and
   w06 because the slope route's systematic is template-shape dependent (the −57 ps figure is a
   property of the projection, not the bench). Only differences *within* one waveform mean anything.
2. **`B/A mag` drifts ~2.4 % with the dither amplitude:** 0.970 at amp2000 → 0.946 at
   amp8000/amp16000. **The first explanation tried — a fixed additive term (the fs/4 line)
   diluting the ratio — is falsified:** solving that model from two amplitude points gives
   inconsistent parameters (ε = 22.7 codes from amp2000 against 1.5 codes from amp16000, implying
   g = 0.9554 against 0.9456). What the data shows instead is in the scaling: A grows ×4.002 and
   ×8.037 where B grows ×3.965 and ×7.954, so **B grows ~1 % less than A over the range — a real
   differential nonlinearity of ~1 % in the amplitude chain, not a measurement artifact.** So a
   gain figure is only meaningful **at a stated dither amplitude**; its absolute accuracy is ~2 %
   even though its precision stays 0.15 % at 500 frames. Stage A's 6.5 % known step is unaffected
   (same waveform on both sides). **The open question is whether the *differential* gain chain is
   linear even though the absolute readout is not** — settle it by repeating stage A's IFC step at
   amp16000 and comparing step sizes (~12 min, one IFC write): the same −6.08 % means the step is
   clean and only the absolute number is amplitude-bound; a smaller step means the chain itself is
   nonlinear and every gain figure must carry its amplitude.

**Best excitation found:** `w06adc_e04_t04_amp16000` — 6-sample pulse at 16000 LSB. Steepest
replica (184.87 codes/sample), smallest timing scatter (15.2 ps/frame), largest difference signal
(78.9 codes), at 4.8 % of ADC full scale. Repeating stage C with it would raise its one-code
resolution from 4.9σ to ~10σ for the same recording length.

*Procedure, kept for repeatability.* Load each waveform **with its JSON**, in this order, and
capture 200 frames at each:

| order | file | pulse | duty |
|---|---|---|---|
| 1 | `w32adc_e16_t32_amp2000` | 32 samples / 24.6 ns | 24.6 % |
| 2 | `w16adc_e08_t16_amp2000` | 16 / 12.3 ns | 12.3 % |
| 3 | `w08adc_e04_t08_amp2000` | 8 / 6.2 ns | 6.2 % |
| 4 | `w06adc_e04_t04_amp2000` | 6 / 4.6 ns | 4.6 % |
| 5 | `w06adc_e04_t04_amp8000` | 6 / 4.6 ns | 4.6 % |
| 6 | `w06adc_e04_t04_amp16000` | 6 / 4.6 ns | 4.6 % |

200 frames is enough here because this stage measures the replica **shape**, and the fold
averages ~700 impulse windows per state. Only stage C's timing precision needs thousands.

Read **`FWHM A`** and **`slope A`** off the tool's table — the width says whether the ADC sees a
narrower impulse, and the slope is what the timing route actually gains from (its sensitivity is
proportional to the replica slope). `rep A` should scale with the amplitude variants (×4 for
amp8000, ×8 for amp16000), which is also the check that the right TXT is loaded: **all six files
share the same period and the same polarity sequence, so the alignment margin cannot tell them
apart** — only width and amplitude can.

**Reference, validated against ideal pulses:** FWHM / peak slope = 24.0 / 2.49, 12.0 / 4.60,
6.0 / 6.50 and 4.0 / 6.50 codes-per-sample for the 32 / 16 / 8 / 6-sample geometries.

**What the bench already shows at the 32-sample waveform is FWHM 17.2, slope 6.38** — a slope the
*ideal* geometry only reaches at 8 samples. So the analog path is not passing the pulse unchanged;
something in it (AC coupling differentiates a pulse train) is already sharpening the edges and
narrowing the positive lobe. The interesting outcome is therefore not "does it match the ideal
table" but **where it stops improving**: if states 3 and 4 give the same FWHM and slope, the path
has hit its limit and a shorter dither buys nothing — which is a result, and the answer to the
instruction. 6 samples is the generator's floor anyway (`edge ≥ 2`, `top ≥ 2`).

```
python tools/dither_response_test.py --uart COM5 --frames 200 --out calibration_out/response/B_<stem>
python tools/dither_raw_evidence.py --state <stem>=calibration_out/response/B_<stem>/raw \
    --waveform-json waveforms/pulse_ladder/<stem>.json
```

Hold everything else fixed while doing this: IFC at 0x0C on both channels, offset calibration off,
actuator untouched (record its code).

### F — the full-scale ladder: how far the excitation can be pushed

**Results — run 2026-09-20, `w06` 6-sample pulse, actuator parked at code 35, 300 frames per
state, every state 300/300 aligned at margin 16.6 (floor 6.0):**

| state | %FS DAC | rep A | rep B | FWHM A | slope A | `B/A mag` | `DC(B−A)` | `dt phase` | dt scatter |
|---|---|---|---|---|---|---|---|---|---|
| amp16000 | 48.8 | 374.2 | 369.4 | 3.7 | 211.5 | 0.9873 | −3.38 | −0.1 ps | 3.4 ps |
| amp24000 | 73.2 | 560.5 | 553.3 | 3.7 | 319.5 | 0.9873 | −2.72 | −0.7 ps | 2.3 ps |
| amp30000 | **91.6** | 700.7 | 692.1 | 3.7 | 402.0 | 0.9876 | −3.17 | −0.2 ps | 1.9 ps |

| pair | commanded | measured rep A | deviation |
|---|---|---|---|
| 16000 → 24000 | ×1.500 | ×1.498 | −0.1 % |
| 24000 → 30000 | ×1.250 | ×1.250 | 0.0 % |
| 16000 → 30000 | ×1.875 | ×1.873 | −0.1 % |

**There is no compression knee below 91.6 % of the DAC's full scale** — the amplitude chain is
linear to 0.1 %, the replica's FWHM is unchanged at every state (3.7 samples, so nothing is being
clipped or reshaped), and the ADC was never the limit: 700.7 codes pk-pk is 8.6 % of ±8192.  So
the answer to "how far can the dither be pushed" on this bench is **all the way to the DAC's
range**, and 30000 LSB (91.6 %) is a usable excitation.

**What the amplitude buys is exactly what the slope route predicts.**  The per-frame `dt` scatter
falls 3.4 → 2.3 → 1.9 ps as the replica grows 374 → 561 → 701 codes, and the ratios are 0.68 and
0.82 against a pure `1/replica` prediction of 0.67 and 0.81 — i.e. the route is slope-limited and
nothing else is.  At 30000 LSB one *working-point* actuator code (~18 ps) is 9.5σ per frame, so a
20-frame batch resolves a single code to ~0.2 ps.  (This also retires the 2026-09-19 worry that a
×2 amplitude bought only 6 % of scatter: on this session's configuration it buys the full 1/replica
improvement, which is a statement about how much of the old scatter was setup rather than slope.)

**And `B/A mag` turned out to be amplitude-independent, which retires §B's open question.**  0.9873
/ 0.9873 / 0.9876 across a ×1.875 amplitude range — constant to **0.03 %**, against the 2.4 %
drift (0.970 → 0.946) the 2026-09-19 session measured on the same waveform.  So that drift was a
property of that configuration, not an inherent differential nonlinearity of the chain, and on
this setup a gain figure does not need an amplitude qualifier.  Cross-check with the closed loop:
the native `B/A mag` here is 0.9873 (1.27 % mismatch) and the `run5_phase` loop's converged
applied correction ratio implies 1.216 % — agreeing to **0.05 %**.

**The IFC known step, at the top amplitude** (B: 0x0C → 0x0D, i.e. 1.59 → 1.70 Vpp, A untouched):

| | off | on | change |
|---|---|---|---|
| `B/A mag` | 0.987623 | 0.921324 | **−6.713 ± 0.022 %** (300σ: per-frame scatter 0.28 %, 300 frames → 0.017 % per state) |
| rep A | 700.73 | 701.02 | +0.04 % (the single-channel manipulation held) |
| FWHM A / slope A | 3.687 / 402.0 | 3.686 / 400.7 | unchanged to 0.3 % (the pulse is not reshaped by the IFC change) |
| `dt phase` | −0.24 ps | −0.52 ps | −0.28 ps (timing does not follow the gain change) |
| `DC(B−A)` | −3.17 | −6.56 | **−3.39 codes** — the documented cross-coupling, now measured to ~50σ |

So the differential gain chain answers a 6.7 % known change with 6.7 % at 91.6 % of DAC full
scale.  Two honest details: the measured step is **0.24 % larger in magnitude than the nominal
1.59/1.70 = 6.47 %**, so the nominal Vpp table is not ground truth at the 0.2 % level and the
*measured* step is what a calibration should use; and the 2026-09-19 session read −6.08 % for the
same nominal step on a different configuration.

**And the amplitude question is now answered within one session** — the same IFC step repeated at
amp16000, with A untouched and the shape unchanged in both cases:

| | amp16000 (48.8 % FS, code 24) | amp30000 (91.6 % FS, code 35) |
|---|---|---|
| IFC step from `B/A mag` | **−6.647 ± 0.044 %** | **−6.713 ± 0.022 %** |
| rep A / slope A change | +0.03 % / −0.16 % | +0.04 % / −0.32 % |
| FWHM A | 3.72 → 3.72 | 3.687 → 3.686 |
| `DC(B−A)` cross-coupling | −3.52 codes | −3.39 codes |

The two steps differ by **0.066 %** — 3.0σ statistically (the precision is 0.02–0.04 %), but 1 %
in relative terms on a 6.7 % step, i.e. **the differential gain chain is linear in amplitude over
48.8 → 91.6 % of DAC full scale.**  So a gain *step* does not need an amplitude qualifier on this
bench; combined with the amplitude-independent absolute readout above, §B's 2026-09-19 open
question ("is the differential chain linear even though the absolute readout is not") is closed in
the affirmative.  Two caveats to keep attached: the two pairs were taken at *different actuator
codes* (24 and 35), so 0.066 % is an upper bound on amplitude- *and* code-dependence together; and
the absolute `B/A mag` at amp16000 moved 0.9873 → 1.0021 (+1.5 %) between two captures 23 minutes
apart at those two codes, which is *not* a clean code cross-coupling (the DC readout moved the
wrong way for that) and is recorded here as an unexplained readout drift rather than as a
measurement.

Stage F answers three things, and the third is a ground-truth question rather than a readout
question:

1. **Where the amplitude chain compresses.** Stage B fixed the pulse at the generator's floor
   (6 samples); amplitude is what is left to spend, and amplitude is what the timing route's
   precision is proportional to. 2000 → 30000 LSB is a ×15 span, and the question is where the
   replica stops tracking the command.
2. **Whether the timing readout keeps improving to the top.** The per-frame `dt` scatter should
   fall as 1/replica *if* the route is slope-limited. On the bench it did not fall as fast as
   that over 8000 → 16000 (16.2 → 15.2 ps for a ×2 replica), so something else contributes a
   floor — jitter, the fs/4 line, quantisation. Stage F is what separates those: if the scatter
   stops improving at the top, the ceiling is not the excitation and no further amplitude buys
   precision.
3. **Is the differential gain chain still linear at the top?** Stage A measured the known IFC
   step (B: 0x0C → 0x0D) as `B/A mag` **−6.08 %** against a nominal −6.47 %, and §B left this
   open: the *absolute* readout drifts ~2.4 % with dither amplitude (B grows ~1 % less than A
   over an ×8 range), so a gain figure is only meaningful at a stated amplitude. Re-running the
   same IFC step at the top of the ladder decides whether that drift is a real differential
   nonlinearity: the same −6.08 % means the step is clean and only the absolute number is
   amplitude-bound; a smaller step means every gain figure must carry its amplitude.

**Prediction, from the states already measured** (bench, 6-sample pulse, 2026-09-19):

| state | commanded | rep A pk-pk (predicted) | slope (predicted) | dt scatter |
|---|---|---|---|---|
| amp16000 | ×1 | **391.3** (measured) | **184.9** (measured) | 15.2 ps (measured) |
| amp24000 | ×1.50 | ~587 | ~277 | ~10 ps if slope-limited, likely less improvement |
| amp30000 | ×1.875 | ~734 (≈9 % of ±8192) | ~347 | ~8 ps if slope-limited |

The predictions assume the chain is still linear; the tool below reports the measured ratios
against the commanded ones, and **a state whose ratio falls short of the command is the answer**,
not a failed run.

```
# 0. gates first (G1): link, geometry, neutral init, then park the actuator and record the code
python tools/raw_frame_probe.py --uart COM5
adc -cal diagnose skewprep fullprep
adc -cal skew step 0            # require neutral_initialized=YES and note the code
spi -r 0x0008                   # 1 = A, 2 = B, 3 = both
spi -w 0x0008 3
spi -r 0x1910                   # both channels must read 0x0C (1.59 Vpp) for this stage

# 1. one --out per state, 300 frames each, actuator untouched throughout
python tools/dither_response_test.py --uart COM5 --frames 300 \
    --out calibration_out/response/F_amp16000
#    ... load w06adc_e04_t04_amp24000.txt on the DPG (its JSON stays on the host) ...
python tools/dither_response_test.py --uart COM5 --frames 300 \
    --out calibration_out/response/F_amp24000
#    ... load w06adc_e04_t04_amp30000.txt ...
python tools/dither_response_test.py --uart COM5 --frames 300 \
    --out calibration_out/response/F_amp30000

# 2. the ladder, in one command
python tools/dither_ladder.py \
  --state 16000=calibration_out/response/F_amp16000/raw \
  --state 24000=calibration_out/response/F_amp24000/raw \
  --state 30000=calibration_out/response/F_amp30000/raw \
  --state-json 16000=waveforms/pulse_ladder/w06adc_e04_t04_amp16000.json \
  --state-json 24000=waveforms/pulse_ladder/w06adc_e04_t04_amp24000.json \
  --state-json 30000=waveforms/pulse_ladder/w06adc_e04_t04_amp30000.json

# 3. the ground-truth tie-in: repeat stage A's IFC step at the top amplitude, DECIMAL writes only
spi -w 0x0008 2
spi -w 0x1910 13                # 13 = 0x0D = 1.70 Vpp, channel B only
spi -r 0x1910                   # confirm 0x0D
python tools/dither_response_test.py --uart COM5 --frames 300 \
    --out calibration_out/response/F_amp30000_ifcD
python tools/dither_ladder.py \
  --state off=calibration_out/response/F_amp30000/raw \
  --state on=calibration_out/response/F_amp30000_ifcD/raw \
  --state-json off=waveforms/pulse_ladder/w06adc_e04_t04_amp30000.json \
  --state-json on=waveforms/pulse_ladder/w06adc_e04_t04_amp30000.json
spi -w 0x0008 2
spi -w 0x1910 12                # restore 0x0C before anything else
spi -r 0x1910
```

**How to read the tool's table.** `rep A` / `rep B` are the *folded* replica peak-to-peak — the
amplitude to quote, because folding averages the noise over the ~8 visible events before the range
is taken; `mag` (per-event range) is reported too but carries a noise bias at small amplitudes.
The linearity block compares each step with the commanded ratio (×1.5 and ×1.875 from amp16000).
`dt phase` is the timing route the calibration loop now uses (each channel's replica fitted
against the template at a fractional sampling phase); `dt fold` is the older derivative
projection, kept because every earlier ladder number in this document was measured with it, and it
is nonlinear beyond ~0.1 sample — that is why its scale looks short at the top of the ladder
(2026-09-20, model: 86.6 ps where the phase route reads 103.6).

**Limits worth stating before the run.** 30000 LSB is 91.6 % of the DAC's full scale, so the DAC is
the element closest to a limit and the generator refuses a waveform that would clip
(`clipping: false` in the JSON is the check). The ADC side is not near its limit at all — a
735-code replica is 9 % of ±8192 — so a compression seen here is *not* the converter.

### C — known skew (results in, 2026-09-19; steps must stay inside 0..48)

**The actuator spans codes 0..48 only** — `SkewActuator.CODE_MIN = 0`, `CODE_MAX = 48`,
`HOST_STEP_PS = 7.4`, i.e. a 355 ps full span. That matches the ~363 ps on-chip delay range
stated elsewhere in the repo, so 48 is the register's physical limit, not a software clamp.
From the neutral code 24 the reachable relative steps are therefore **−24..+24**.

**The firmware answers `result=OK` to a relative step it cannot take.** Measured 2026-09-19: a
`--steps +32` request from base 24 walked to 48 and then answered `OK 48 -> 48` forever. The
old loop in `tools/dither_response_test.py` had no "did it move" check, so it spun, and **each
iteration issued another transaction — each of which resets the JESD link**. The tool now
bounds-checks every target before moving and aborts on a non-advancing transaction, but the
step list still has to be chosen inside the range.

```
# ⓪ first, after any link recovery: is channel B healthy again?  (do not skip this)
python tools/dither_response_test.py --uart COM5 --frames 100 \
    --out calibration_out/response/check_b
python tools/dither_raw_evidence.py --state ck=calibration_out/response/check_b/raw \
    --waveform-json waveforms/pulse_ladder/w32adc_e16_t32_amp2000.json
#    go on only if rep B is back near 46 codes and B/A mag near 0.96.
#    A rep B near 95 codes or a magnitude ratio near 20 means B's data path is still broken,
#    and then gain and offset readings are meaningless whatever the timing says.

# ① the ladder: 24 -> 32 -> 25.  One run, so the baseline sits inside the same session as the
#    steps and cross-session drift cannot enter the slope.  Only 9 actuator transactions.
#    NOTE: --steps is relative to the STARTING code, not cumulative -- "+8,+1" means
#    base+8 and base+1, i.e. 24 -> 32 -> 25, and the tags are code24 / code32 / code25.
#    1000 frames per state, not 500: the +1 state is the resolution test and needs the precision
#    (sigma ~= 45 ps per frame, so 1000 frames give ~1.4 ps per state and ~2.0 ps on the
#    difference).
python tools/dither_response_test.py --uart COM5 --frames 1000 --steps +8,+1 \
    --allow-skew-writes --out calibration_out/response/ladder
```

**Result (measured 2026-09-19, `w32adc_e16_t32_amp2000`, one session):**

| state | code | dt mean | per-frame σ | SE |
|---|---|---|---|---|
| base | 24 | +62.9 ps | 44.3 ps | 1.40 ps |
| +8 | 32 | +24.4 ps | 44.0 ps | 1.39 ps |
| +1 | 25 | +53.6 ps | 47.8 ps | 1.51 ps |

- **8 codes → −39.5 ± 2.0 ps (20σ), i.e. 4.94 ps/code.** That independently reproduces the
  actuator's own characterization (4.8–4.9 ps/code, measured a different way), so the readout
  tracks a known change with the right scale.
- **1 code → −10.0 ± 2.1 ps (4.9σ): one actuator code is resolved.** It is larger than the
  8-code average because the step is not uniform — the repo's single-step characterization found
  7.4 ps/code near neutral against 4.8 ps/code end to end, and 10.0 agrees with 7.4 within 1.3σ.
- **This contradicts the model-based claim that skew has no usable route without the tone.** That
  claim came from the estimator's tone-phase and centroid routes, which do fail here
  (±1370 ps and ±290 ps per frame); the slope route is ±44 ps per frame.

**Two axis cross-couplings showed up and must be stated, not smoothed over:**

| quantity | code 24 | code 32 | code 25 | dependence |
|---|---|---|---|---|
| `B/A mag` | 0.9709 | 0.9880 | 0.9772 | **+0.21 %/code** (+1.7 % over 8 codes, 16σ) |
| `DC(B−A)` | −2.05 | −3.00 | −1.82 codes | **−0.12 codes/code** (24σ) |

The raw gain and offset readouts drift with the actuator code, so the three axes are not exactly
orthogonal at the raw layer. The staged design already isolates this (stages A/D/E hold the code
fixed), but it has to be reported: a gain or offset number is only comparable at the same code.

**Keep the transaction count down.** Every move resets the JESD link, and an earlier plan of
`+24,-23` (47 transactions) ran on top of ~400 from a railed-actuator episode; the link then died
outright — captures stopped, `raw_frame_probe.py` failed every frame, and the state in between was
already corrupt (channel B's replica went 45.8 → 94.8 codes while channel A stayed at 47.9). Use
the smallest step that is large against the readout's ≈ −57 ps systematic: **8 codes ≈ 40–59 ps,
which 500 frames resolve to ~2 ps, i.e. 20–30σ.** Extend the lever arm in a *separate* run later,
once the states are clean and the link is stable — never walk far out and back in one run.

The `--steps +24,-23` variant (targets 24, 48, 25) remains valid when the link is healthy and the
longer lever arm is wanted; everything below applies to either step list.

```
python tools/dither_raw_evidence.py \
  --state base=calibration_out/response/ladder/raw/code24_frame_*.bin \
  --state c32=calibration_out/response/ladder/raw/code32_frame_*.bin \
  --state b1=calibration_out/response/ladder/raw/code25_frame_*.bin \
  --state c00a=calibration_out/response/c00_a/raw \
  --waveform-json waveforms/pulse_ladder/w32adc_e16_t32_amp2000.json
```

`c00a` is an independent third-party check rather than the baseline: three separate sessions have
now measured code 24 at **+80.4 / +80.1 / +80.3 ps** with `B/A mag` = 0.9640, so the state
reproduces across reboots — but the slope itself is computed from the states inside one run.
(replacing steps ② and ③ of the earlier plan; `c00_b` was discarded because channel B was
corrupt, so it never answered the code-mapping question — the three-session agreement did.)

**Always pass `--verify-frames 0` on these waveforms.** `skew_park.py` verifies the parked state
with the **tone-phase** estimator, and a dither-only capture has no tone, so the check always
fails: measured 2026-09-19 it reported `+nan +- nan ps over 0/20 frames {'tone out of range': 16,
'profile-fallback source': 4}`. The *parking* is sound — every move was ACK-verified — only that
line is meaningless here, and it costs 20 captures.

**Check the code before trusting a baseline, because the reported code can lose its meaning.**
Measured 2026-09-19: after the railed-actuator episode (hundreds of transactions, each resetting
the JESD link and the fine-delay mode) the actuator read **code 1**, not the 48 it had railed at,
with nobody moving it down. So the code is a firmware belief that can drift from the physical
delay. The protection is structural: **put the baseline inside the same run as the steps** (which
the tool does — it captures at the starting code first), so the slope never depends on a state
captured in an earlier session. A separate code-24 baseline then serves as an independent check,
and three sessions have now agreed on it to 0.3 ps (+80.4 / +80.1 / +80.3 ps), so the mapping has
held across reboots in practice.

**Use a separate `--out` per run.** `dither_response_test.py` names frames
`<tag>_frame_<iii>.bin` inside one `raw/` directory, and the tag is just `code<NN>`, so a run
that revisits a code **overwrites the earlier state's frames** — that is how an earlier 20-frame
baseline was lost. `--out` gives each stage its own `raw/`, and it also means a run that stops
early can be re-run without clobbering the frames already captured.

The leading `+` matters: argparse rejects a bare `-1,-2,+3` as an option
(use `--steps=-1,-2,+3` if a step list ever has to start with a minus).

States must be selected with a **glob or a per-run directory**: pointing `--state` at a shared
`raw/` that holds several tags averages them together (the tool now warns when it sees that).

Judge on: slope of dt against codes (ps/code, should match the actuator's own characterization),
and the +1-code state against the run's own baseline, where one code is `4.8 ps / (σ/√N)` sigma.

### D — known gain, 500 frames per state

Only if the IFC gate passed. Fix A at 1.70 and step B through 1.70 / 1.59 / 1.47, 500 frames
each. Expected B/A: **1.000 / +6.9 % / +15.6 %**. Confirm both channels' setting with `status`
after every change and write both numbers into the log's `IFC(A/B)` column.

Two honest limits of this stage:

- **IFC's smallest step is about 6 %** (1.70 → 1.81 is −6.1 %). The mismatches a calibration
  actually has to see are sub-1 %, so this stage proves the readout is *proportional* (two
  points, 6.9 % and 15.6 %) and gives the precision σ; the sub-1 % claim is then σ/√N, and
  must be phrased as an extrapolation, never as "we demonstrated 1 % detection".
- a third point (2.04 or 1.36) only helps if the first two do not fall on a line.

### E — offset: the known step, measured 2026-09-19

**Result.** Two independent toggles of channel B's internal DC-offset calibration, 500 frames per
state, actuator and IFC held fixed:

| pair | `DC(B−A)` OFF | `DC(B−A)` ON | step |
|---|---|---|---|
| `A_base` → `E_on` | −1.95 ± 0.046 codes | +5.97 ± 0.042 codes | **+7.93 codes** |
| `E_on` → `E_off` | −2.19 ± 0.046 codes | (same ON state) | **−8.16 codes** |

The two toggles agree to 0.23 codes, and the two OFF states agree to 0.24 codes (a one-hour drift,
3 % of the step). Per-frame scatter is ~1.0 codes, so 500 frames give 0.045 codes and the step is
**~130σ**. The independent prediction from the raw channel means — `mean(B) = −7.77` codes being
nulled while A stays put — was **+7.8 codes**, which the measurement matches.

**The other two axes did not move**, which is what makes this a clean single-axis manipulation:
`dt` changed by +3.0 ps (1σ), `B/A mag` by −0.2 % (1.3σ), `rep A` by 0.7 %.

Reproduce with:

```
python tools/dither_raw_evidence.py \
  --state off=calibration_out/response/A_base/raw \
  --state on=calibration_out/response/E_on/raw \
  --state off2=calibration_out/response/E_off/raw \
  --waveform-json waveforms/pulse_ladder/w32adc_e16_t32_amp2000.json
```

**Three-axis status after this stage — every axis now has a known-change measurement:**

| axis | known change | readout change | significance | cross-check |
|---|---|---|---|---|
| skew | actuator +8 codes | −39.5 ± 2.0 ps = 4.94 ps/code | **20σ** | actuator's own characterization: 4.8–4.9 ps/code |
| skew | actuator +1 code | −10.0 ± 2.1 ps | **4.9σ** | 7.4 ps/code near neutral, within 1.3σ |
| gain | IFC B 0x0C → 0x0D | `B/A mag` −6.08 % | **15.7σ** | nominal −6.47 %, 1.03σ |
| offset | B DC-offset cal on/off | `DC(B−A)` +7.93 / −8.16 codes | **~130σ** | raw means predict +7.8 codes |

That answers the "first confirm that you can reliably detect the gain, offset and skew" request
with known changes on all three axes rather than extrapolation — with one caveat that must be
stated: **gain below ~1 % is still an extrapolation** (IFC's smallest step is ~6 %), so the
sub-1 % claim is σ/√N, not a demonstrated step.

*Procedure as run (kept for repeatability).*

There is no settable offset register, but `0x0701` bit 7 is the chip's own DC-offset
calibration enable, and that is a *fixed setting* that can be flipped per channel. It is the only
remaining axis of the professor's "confirm you can reliably detect gain, offset and skew": gain is
done (§A) and skew is done (§C), so offset is what this stage settles either way.

**Preconditions — the pair is worthless without them:**

1. **Restore channel B's IFC to 0x0C first.** Stage A left B at 0x0D, and IFC itself moves the DC
   by −3.8 codes (measured), so an un-restored IFC contaminates the very quantity being measured.
2. **Do not touch the actuator.** The offset readout carries the same +0.21 %/code code dependence
   measured in §C; record the code and hold it.
3. **Judge only `DC(B−A)`** — not `B/A mag`, which is a function of IFC by §A.
4. `adc -offset status` reads 0x0701 / 0x073B **once**, for whichever channel is currently
   selected, so both channels must be read through the `spi` route.

```
# ⓪ restore B's IFC and confirm
spi -w 0x0008 2
spi -w 0x1910 12         # 12 = 0x0C = 1.59 Vpp
spi -r 0x1910

# ① current state of both channels
adc -offset              # NOTE: this is a MENU (offset-cmd$), not a one-shot command.
  status                 #   prints Status + 0x0701 + 0x073B for the SELECTED channel
  back                   #   `spi` is rejected while inside the menu, so leave it first
spi -w 0x0008 1
spi -r 0x0701            # A's enable register (measured 2026-09-19: 0x02 = OFF)
spi -w 0x0008 2
spi -r 0x0701            # B's

# ② baseline, 500 frames, its own output directory
python tools/dither_response_test.py --uart COM5 --frames 500 \
    --out calibration_out/response/E_base

# ③ enable on channel B only -- pre-select B, then let the firmware do the two-register write
spi -w 0x0008 2
adc -offset
  on                     # writes 0x0701 = 0x80 AND 0x073B = 0x00 (see the note below)
  status                 # expect Status: ON, 0x0701 = 0x80, 0x073B = 0x00
  back
spi -w 0x0008 2
spi -r 0x0701            # confirm 0x80
spi -w 0x0008 1
spi -r 0x0701            # A must still read 0x02  <-- the per-channel test

# ④ does the write disturb the link?
python tools/raw_frame_probe.py --uart COM5
python tools/frame_consistency.py --uart COM5 --frames 6

# ⑤ stepped capture
python tools/dither_response_test.py --uart COM5 --frames 500 \
    --out calibration_out/response/E_step

# ⑥ compare
python tools/dither_raw_evidence.py \
  --state base=calibration_out/response/E_base/raw \
  --state step=calibration_out/response/E_step/raw \
  --waveform-json waveforms/pulse_ladder/w32adc_e16_t32_amp2000.json

# ⑦ back off, and re-enable once to show the step reproduces
spi -w 0x0008 2
adc -offset
  off
  back
```

**Why the menu's `on`, not a raw `spi -w 0x0701 128`.** The firmware's
`ad9695_adc_set_dc_offset_filt_en(1)` is a **two-register** write — `0x0701 = 0x80` *and*
`0x073B = 0x00` — and its own status test is
`(0x0701 & 0x80) && !(0x073B & 0x80)`. Measured 2026-09-19 the chip sat at `0x0701 = 0x02`,
`0x073B = 0xBF`, so writing only the enable bit would leave bit 7 of 0x073B set and the status
would still read OFF. `on`/`off` also write without touching 0x0008, so the channel must be
pre-selected — exactly like IFC.

**Predicted step.** The menu documents the feature as "removes the average DC bias from the ADC
output, correction range ≈ ±512 codes", and the raw means in this state are `mean(A) = −5.81`,
`mean(B) = −7.77` codes. Nulling B alone should therefore move `DC(B−A)` from ≈ −1.95 to ≈ +5.9
codes: **a step of ≈ +7.8 codes against a 0.045-code precision at 500 frames, i.e. ~170σ.** If no
step appears, the enable does not move this board's DC — then report offset as repeatability only
(±1.0 codes per frame, 0.045 codes at 500 frames).

**Reading the result.** A repeatable step of several codes is a usable known change — then toggle
it back off and on once more to show it reproduces, and the three-axis claim is complete. A step
at or below the 0.045-code precision means the chip's internal calibration does not move this
board's DC: report offset as **repeatability only** (±1.0 codes per frame, 0.045 codes at 500
frames) and say so plainly rather than implying a three-axis known-change test. Either outcome is
a result; the failure mode is quoting a step that is not there.

### Sizing the recordings before spending the time

The 500/1000-frame numbers above come from a *synthetic* per-frame scatter (150–270 ps at
2000 LSB with a 6-sample pulse), and the whole table assumes the replica amplitude scales
linearly with `dither_scale_lsb` — which is what stage B measures, not something to assume.

So: capture **100 frames** at the first state, read the per-frame scatter out of
`dither_raw_evidence.py`, and size the long runs from the measured σ (`N = (k·σ/4.8 ps)²` for
`k` sigma on one actuator code) instead of from the table.

Known offsets are never measured by the raw route: the slope projection carries a systematic
(≈ −57 ps in the instrument check), so read **differences between fixed states**.

### What each stage is for

| stage | answers |
|---|---|
| G, A, D, E | "confirm you can reliably detect gain and offset" — A/D proportional recovery + precision, E the offset step |
| C | "confirm you can reliably detect skew" — known actuator codes, slope, and whether one code is resolvable |
| B | "the dither should be shorter" — where the analog path stops passing a shorter pulse |

If bench time runs short, C is the one to keep: skew is the axis with no fallback route once the
tone is gone, and it is the axis a calibration loop cannot do without.

## 4. Recording log — one line per state

```
state | code | IFC(A/B) | waveform file | frames | start-end | raw dir | notes
```

Per state, record: frames captured, frames aligned, replica pk-pk (A and B), B/A, DC(B−A),
timing via the slope route with its per-frame scatter, and alignment margin. The tool writes
`calibration_out/dither_raw/dither_raw_states.csv` plus a figure per run.

## 5. Traps already paid for

- **Every sample-clock delay write resets the JESD link** (~2 ms plus re-alignment). A capture
  inside that window returns dead or half-synced data: `dma -w` times out, the buffer keeps
  the *previous* frame and every metric repeats exactly. One earlier bench run "converged" on
  a single frozen frame that way, 296 iterations long. Check for repeated frames.
- **The actuator step is not uniform** — 7.4 ps/code measured near neutral, 4.8–4.9 ps/code
  end to end. Characterize the code you are actually using; never extrapolate.
- **About 20 % of captures are torn UDP frames.** They are logged with reasons and retried;
  do not read a rejection rate as instability.
- **Do not compare a state recorded before a delay write with one recorded after** without
  noting the JESD re-sync in between.
- The dither-only loop must run with `--gain-observable dither`; the default tone observable
  becomes a noise integrator when the tone is gone. This session measures rather than loops,
  but any loop run on these waveforms needs that flag.
- **The actuator rails at code 48 and the firmware still answers `OK`.** Measured 2026-09-19: a
  `+32` step from base 24 answered `OK 48 -> 48` indefinitely. `dither_response_test.py` used to
  spin on that, issuing one JESD-resetting transaction per iteration; it now bounds-checks
  targets and aborts on a non-advancing move. **Check every step list against 0..48 before
  running it** — the host's own `SkewActuator.CODE_MIN/CODE_MAX` know the limit, the transaction
  path does not enforce it.
- **Frame files overwrite within one `--out`.** The tag is only `code<NN>`, so a run that
  revisits a code replaces the earlier state's frames in the same `raw/` — that is how a 20-frame
  baseline was lost. One `--out` per stage, and select states with a glob or a per-run directory.
- **`spi -w` truncates its data token to three characters**, so a hex value such as `0x0D` is
  written as `0x00` while the console still prints `Command Success`. Write data values in
  decimal (see the IFC gate).
- **`dither_response_test.py`'s `phase` and `tone gain` lines are meaningless on these
  waveforms.** There is no tone, so those two routes report noise (`phase ±1160 ps`,
  `tone gain 1.23 ± 0.70`, `centroid ±334 ps`). Only `dither gain` — and per 1a, `B/A mag` from
  `dither_raw_evidence.py` — carry information.
- **The 325 MHz = fs/4 clock line is exactly why `raw_frame_probe`'s absolute numbers look
  strange.** It is coherent (does not average away), it sits on the group granularity, and it
  makes the probe's 32-bit-word view show a 1.4× channel asymmetry that the correct 16-bit view
  does not (11.34 vs 11.12 codes; replica ratio 0.992). Do not chase that asymmetry in hardware.
- **Two timing conventions differ by a sign, and the wrong one closes the loop backwards.** A delay
  on channel B shifts its *sampling instant* later (positive in the loop's convention, and what
  `CalibrationState` turns into "command less delay") and shifts its *index* sequence earlier. The
  tone-free route was first written in the index convention; in the model the commanded delay then
  ran 0 → +390 ps while the residual grew to −371 ps, every transaction ACKed `OK`. Any new
  timing route must be checked against a known delay — `calibration_out/_tone_free_loop_test.py`
  does that, and it is the only check that catches this class of bug (a converging-looking trace
  does not).
- **A route that is dead on the waveform in use must not be the one the gates judge.** On a
  tone-free capture the estimator's tone-phase route still returns *finite* numbers — the
  least-squares fit latches onto a dither comb line — so it passes its own validity bound while
  reporting hundreds of ps of noise. Gating on it rejected 81 % of captures in the first live run
  and 45 % of good frames in the model while the loop's own route was fine.
- **A per-event peak-to-peak is noise-biased at low amplitude.** The range of ten noisy samples is
  a few sigma wide whatever is inside them, so at 2000 LSB the per-event magnitude reads high and
  an amplitude ladder built on it shows compression that is not there (model, 2026-09-20: 48.8
  codes measured for a 25-code replica). `tools/dither_ladder.py` therefore quotes the *folded*
  replica peak-to-peak, where the noise is averaged over the events before the range is taken.

## 6. What this session can and cannot support afterwards

Can: gain ratio B/A and offset DC(B−A) from raw folded replicas, per fixed setting, with
error bars from the frame count; timing differences between settings via the slope route;
the pulse-width response of the analog path.

Can: gain ratio `B/A mag` and offset `DC(B−A)` from the raw impulse windows, per fixed setting,
with error bars from the frame count; timing differences between settings via the slope route,
against the actuator's known codes; the pulse-width response of the analog path; and — measured
2026-09-19 — **known-change detection on all three axes** (skew 20σ / 4.9σ, gain 15.7σ, offset
~130σ, §E).

Cannot: absolute timing per frame (the slope route carries a ≈ −57 ps systematic, so only
*differences* are meaningful); any statement about interleaving, since the two converters sample
the same instant; a known gain step finer than IFC's ~6 %, so sub-1 % gain detection is an
extrapolation from σ/√N and must be worded as one. Also, a one-code resolution claim is only
valid **within a session** — cross-session drift at a fixed code is ~20 ps, larger than one
actuator code.

The **cross-couplings** measured here are part of what the session supports, not noise to hide:
the actuator code shifts the gain readout by +0.21 %/code and the DC readout by −0.12 codes/code,
and an IFC change shifts the DC readout by −3.8 codes. Each stage therefore varies exactly one
thing and holds the other two fixed, which is why the single-axis steps above come out clean.

Judge the timing readout on the *difference* between two states against the known actuator
codes — that is the only place a ground truth exists without extra hardware.

## 7. The tone-free closed loop on the bench (step 1)

Everything above measures. This is the run that *closes* the loop with no reference tone at all —
all three axes off the impulses, which is what the 19 September direction asks for and what the
host code only became able to do on 2026-09-20.

### Result — first dither-only closed loop, 2026-09-20

`run4_tonefree` (`calibration_out/closed_loop/`), 300 qualified samples from **300 captures, 0
rejected**, routes `gain dither_mag` / `skew dither_fold` / cancellation off, alignment margin
16.5 throughout (floor 6.0):

| axis | start | end (last 60 samples) | how it got there |
|---|---|---|---|
| skew | **−71.4 ps** (fold route at code 24) | **−7.5 ps** (inside the 10 ps deadband) | 10 single-code moves, code 24 → 34, **every one `move+1:OK`**, batch standard error 0.44–0.93 ps, no direction latch (`skew_abort: null`) |
| offset | — | **−0.023 codes** signed B−A (scatter 1.28) | per-frame, `dither_mag`-independent |
| gain | native B/A magnitude mismatch ~1.5 % | **magnitude ratio 0.9997** | per-frame, correction ratio 1.0149 |
| A−B coherence | **−14.6 dBc** (native, tone-free) | **−32.7 dBc** (residual) | **18.1 dB improvement** |

The per-batch move averaged **+6.4 ps/code**, reproducing the route check's +6.46 — i.e. the
loop's own step agrees with the actuator's known codes, which is what the direction guard checks.

The last row is the independent cross-check that the loop closed the *physical* mismatch rather
than just its own estimator: the coherent A−B dither power is a different functional of the same
captures (folded-replica difference power, not a derivative projection), and 71.4/7.5 = 9.5× in
mismatch predicts 19.5 dB against the 18.1 dB measured — consistent to 1.4 dB.

**And it attributes.** The coherent A−B level is the power sum of a skew term (linear in the
delay) and a gain term (the native mismatch, which no hardware write removes), so both ends of the
run can be predicted from two numbers that were measured independently:

| | skew term | gain term | predicted | measured |
|---|---|---|---|---|
| native (code 24, 71.4 ps, 1.47 %) | −14.63 dBc | −36.66 dBc | **−14.60 dBc** | **−14.60 dBc** |
| residual (code 34, 7.5 ps) | −34.24 dBc | −36.66 dBc | **−32.27 dBc** | **−32.65 dBc** |

0.00 dB and 0.38 dB, so nothing in this metric is unexplained — the same attribution the tone-mode
run reached to 0.25 dB, now without a tone anywhere. (The gain term is the *raw* mismatch: the gain
correction is host-side, so it does not enter a metric computed on the raw capture. That is why
`dbc_ab_coherent_cal` now exists — logged from the corrected streams, it is what the next run
should quote for the residual.)

**The gain axis is where a tone-free session is weaker.** The controlled observable
(`dither_mag`) scatters **0.60 % per frame** here (the tone route is 0.25 %), and `mu_gain = 0.35`
integrates that noise, so the applied correction ratio is a slow random walk — the implied *raw*
magnitude ratio drifted 0.9995 → 0.9877 over the run while the *measured* (corrected) ratio sat at
1.0000 ± 0.0003 throughout, exactly as the controller is designed to do. Read the applied ratio
from `gain_corr_b/gain_corr_a`, and treat any tone-free corrected A−B figure as floored near
−38 dBc by that ~1 % correction uncertainty (AGENTS.md's applied-ratio term).

**Two things this run does not settle.** (1) The parked code is where the *fold route* reads zero;
that route's absolute zero point carries a template-shape systematic, so the run shows the skew
fell ~10× and the residual is inside the deadband *by the controlled route*, not that the true
residual is 7.5 ps.

> **Settled the same afternoon, and the answer was not the one the noise figures suggested.**  A
> tone-mode run (`tonezero_check`, 80 qualified samples, 20 rejected, no latch) was started at that
> parked code 34, with the tone+dither waveform loaded:
>
> | batch | measured at code | tone-route reading | action |
> |---|---|---|---|
> | 1 | 34 | **−16.90 ± 0.07 ps** | `move+1:OK` → 35 |
> | 2 | 35 | **+2.63 ± 0.05 ps** | inside deadband |
> | 3, 4 | 35 | +2.59 / +2.64 ps | no move |
>
> So the **zero points are fine and the *scale* was not.**  The tone loop needed exactly one code
> from where the dither-only loop stopped, which puts the three zero crossings within 0.3 code of
> each other (fold 35.2, phase 35.2, tone 34.9).  But at code 34 the fold route had reported
> −7.5 ps — inside its 10 ps deadband, so it stopped — while the tone route measured **−16.9 ps**,
> i.e. outside that deadband, and an independent functional on the *same frames* agrees: the raw
> A−B spur was −31.5 dBc there, and removing the known 1.5 % native gain term (−36.7 dBc) leaves a
> skew term of **17.7 ps** (1 ps from the tone reading).  Then one code moved the tone reading by
> **19.5 ps**, against the fold route's 6.5 — a 2.8-3× scale disagreement, with the tone and phase
> routes agreeing to 7 % (19.5 vs 18.2 ps/code).
>
> Two consequences, both now in the code and in §7's preconditions: a dither-only loop must drive
> `dither_phase` (fold is quieter but stops with a real residual ~2.8× its deadband), and **the
> 4.85 / 7.4 ps/code characterizations are not an independent anchor** — `skew_step_characterize.py`
> measures with the tone route too, and today's tone reading of 19.5 ps/code at the working point
> disagrees with them, so the step should be re-measured at the code in use.
>
> The same run's end state is also the best this bench has produced: corrected A−B spur
> **−48.4 dBc** (raw −37.0 dBc, which is now *gain*-limited: the 1.47 % native gain mismatch alone
> is −36.7 dBc), corrected DC mismatch −0.011 codes, SNDR 36.9 dB, SFDR 41.0 dB.

> **And the same afternoon settled which route to drive it with** — the dither-only loop re-run on
> `dither_phase` (`run5_phase`, 300 qualified samples from 300 captures, 0 rejected, no latch),
> from the same starting code 24:
>
> | | `run4` (`dither_fold`) | `run5` (`dither_phase`) | tone route |
> |---|---|---|---|
> | moves | 10 (24 → 34) | **11 (24 → 35)** | 1 (34 → 35) |
> | mean step the loop saw | +6.37 ps/code | **+17.87 ps/code** (sd 2.39) | +19.5 ps/code |
> | parked residual, its own route | −7.47 ps | **−0.06 ps** | +2.6 ps at code 35 |
> | per-frame scatter | 2.38 ps | 8.14 ps (batch se 1.8 ps) | — |
> | raw coherent A−B at the end | −32.65 dBc | **−37.19 dBc** | — |
>
> The phase route agrees with the tone route at *both* codes it can be compared at — code 34:
> −18.4 against −16.9 ps; code 35: −0.27 against +2.63 ps — while the fold route read −7.5 ps at
> code 34, i.e. **2.4× low**, and stopped one code short.  So the fold route's under-read is
> confirmed by three independent measurements, the phase route is the one to drive, and this is
> what ``--tone-free`` now selects.
>
> Two consequences worth carrying.  (1) **The native skew at code 24 is ~−212 ps, not −71 ps**:
> the phase route read −196.8 ps there, and its scale is 7 % below the tone route's, so the fold
> route's −71.4 ps was a 0.34× view of it.  (2) The end state's *raw* coherent A−B (−37.19 dBc)
> now sits at the native gain-mismatch floor (1.216 % → −38.30 dBc predicted, 1.1 dB difference),
> so the timing axis is no longer the limiter and further improvement on a raw metric has to come
> from the gain axis.  The corrected column reads −38.74 dBc, i.e. only 1.5 dB below raw, which
> says the residual is dominated by a common term rather than by the correction's mismatch.

(2) The native B/A magnitude mismatch here (~1.5 %) is much smaller
than the archived 2026-09-19 states measured at the same waveform (5.4 %), and this session's
alignment margin is 16.5 against 8.06 — so the setup itself changed between sessions; do not mix
numbers across them.

### What the host code now does differently

| change | why |
|---|---|
| **Timing comes from the folded replicas**, on a selectable route: `dither_phase` fits each channel against the template at a *fractional* sampling phase and differences the two phases; `dither_fold` projects the folded A-B difference onto the pulse slope. | The two disagree about *scale* on this bench, and the phase route is the one that matches the shape-independent tone route. **Measured live 2026-09-20** (`w06` 16000 LSB, 200 frames per state, margin 8.1): phase **+18.2 ps/code at 4.8 ps/frame**, fold +6.5 at 3.1, and the tone route **+19.5 ps/code** measured the next hour at the same working point. The fold projection is quieter and 2.8× low, and that is not academic: a dither-only run driving it parked reading −7.5 ps while the tone route measured **−16.9 ps** at that same code (an independent functional, the raw A−B spur minus the known 1.5 % gain term, implies 17.7 ps), i.e. a scale-biased route stops with a real residual ~2.8× its own deadband. So `--tone-free` selects `dither_phase`, and `dither_fold` is the cross-check. The *zero points* agreed to 0.3 code (fold 35.2, phase 35.2, tone 34.9), so the fault is scale, not offset. |
| **The sign is the sampling-instant convention** (positive = channel B samples later), matching what `CalibrationState` integrates. | The first version measured the *index*-domain shift, which has the opposite sign. In the model that closed the loop backwards: the commanded delay ran 0 → +390 ps while the true residual grew to −371 ps, with 81 % of captures rejected on the way. `calibration_out/_tone_free_loop_test.py` now checks both routes against known delays, so a sign flip cannot come back silently. |
| **The acceptance filter judges the route in use**, and the log records it (`skew_used_ps`, `skew_observable`, `gain_source`). | The old filter gated on the estimator's *tone* fields, which on a tone-free capture are a fit of noise — it would have refused 45 % of good frames in the model and 81 % in the first live run, while the loop's own route was fine. A gain-observable fallback is now itself a rejection: on a tone-free bench, falling back means falling back onto noise. |
| **`--tone-free`** sets `--gain-observable dither_mag`, `--skew-observable dither_phase` and `--no-cancellation` together, and an explicitly passed route still wins. | There is no tone to cancel, and the tone observable would integrate noise. The gain loop still controls the *broadband* (pulse) mismatch; it deliberately does not null f_in, and with no tone there is no f_in to null. |

Offline evidence, all green before the bench:

```
python calibration_out/_tone_free_loop_test.py     # routes vs truth, both closed loops, gate
python tools/loop_direction_check.py               # batch decision + direction latch, all routes
```

The first prints, among others: the phase route reading +5.3 / +27.9 / +54.2 / +104.0 / +202.2 ps
for true +3.6 / +28.6 / +53.6 / +103.6 / +203.6 ps (≤ 1.7 ps of error, 3–7 ps per frame at a
bench-like 200-code replica); **both** tone-free routes converging over the model (gain magnitude
1.0000, offset 0.08 codes, skew +0.18 ps on `dither_fold` and −0.59 ps on `dither_phase`, **0 of
40 captures rejected** in each); and acquisition from a +200 ps mismatch driving the command to
−199 ps instead of running away.

### Preconditions — the same gates as any bench run, and two that are tone-free specific

```
python tools/raw_frame_probe.py --uart COM5        # link health (first UDP after boot often times out)
python tools/geometry_id.py --uart COM5 --frames 4 # period + pulse width against the JSON you pass
adc -cal diagnose skewprep fullprep                # neutral preparation
adc -cal skew step 0                               # require neutral_initialized=YES, note the code
python tools/skew_park.py --uart COM5 --code 24 --verify-frames 0
```

1. **Load a tone-free waveform and pass its geometry on the command line.** The loop builds its
   config from the CLI, not from the JSON, so the flags must match the TXT on the DPG:
   `--amp-dbfs -120 --dither-edge 4 --dither-top 4 --dither-scale 16000` is
   `w06adc_e04_t04_amp16000`. Verify before spending time:
   `python -m calibration_loop.run_calibration check --waveform-json waveforms/pulse_ladder/w06adc_e04_t04_amp16000.json --adc-rate 1.3e9`
   (6/6), and confirm the replica is where it should be with
   `python tools/dither_ladder.py --state ck=<a short capture dir> --state-json ck=<that JSON>`
   — 391 ± 10 codes pk-pk for amp16000. A mismatch shows up as a collapsing alignment margin and
   the run stopping after 20 consecutive rejects.
2. **`--verify-frames 0` on `skew_park.py`**: its verification uses the tone-phase estimator, so on
   a tone-free capture it always reports `+nan`. The parking itself is ACK-verified and sound; only
   that line is meaningless.
3. **Run the route check once per waveform, before the loop.** The two tone-free timing routes
   disagree about *scale* on this bench (2.8× on 2026-09-20), and which one to trust depends on the
   waveform — so measure it on the waveform you loaded (read-only, no register writes):

   ```
   # from the two states of any known code pair, e.g. park 24, capture, step +8, capture
   python tools/timing_route_check.py \
     --state 24=calibration_out/response/R24/raw \
     --state 32=calibration_out/response/R32/raw \
     --waveform-json waveforms/pulse_ladder/<the file you loaded>.json
   ```

   It fits each route's ps/code against the known codes, checks the sign, and — when the captures
   hold a tone — measures the tone-phase route as the scale anchor (refusing the anchor if the
   fitted tone amplitude says the frames have no tone). **Live result, 2026-09-20, `w06`
   16000 LSB:** `dither_phase` **+18.2 ps/code at 4.8 ps/frame** (a 20-frame batch → 1.1 ps of
   standard error against the 10 ps deadband), `dither_fold` +6.5 at 3.1 ps/frame, and the tone
   route +19.5 ps/code — so that session ran on `dither_phase`, which is also the `--tone-free`
   default. The tool now fails a waveform whose routes disagree by more than 30 % and tells you to
   get a tone reference rather than picking the quieter one, because picking the quieter one is
   exactly how the fold route came to drive the first dither-only run.

   **The frames must be healthy before any of this means anything.** The first attempt at this
   session captured two states with **no dither in them at all** — folded replica 1–2 codes against
   the ~391 expected, alignment margin **2.7** against the 8.1 healthy value, and a nonsense
   `dither_gain` of 1.6–2.6 against ~1.0 — and its ps/code was meaningless however it was read. The
   quick health check is the capture tool's own `response_states.csv` (margin ≈ 8, `dither_gain`
   ≈ 1.0) or
   `python tools/dither_raw_evidence.py --state ck=<dir> --waveform-json <that JSON>` (expect
   `rep A ≈ 391` for amp16000), and if it fails, re-upload the TXT and re-probe before spending
   time on the routes.

### The run

```
python -m calibration_loop.run_calibration bench --uart COM5 --tone-free \
    --allow-skew-writes --iterations 300 \
    --amp-dbfs -120 --dither-edge 4 --dither-top 4 --dither-scale 16000 \
    --out calibration_out/closed_loop --stem run4_tonefree
```

300 qualified samples ≈ 370 captures ≈ 30 min unattended. `--allow-skew-writes` is required or the
run measures skew and never moves the actuator. `--tone-free` picks the fold route; add
`--skew-observable dither_phase` if precondition 3 above says the phase route is quieter on the
waveform you loaded.

### What to watch, and what each reading means

| reading | expected | if it is not |
|---|---|---|
| meta JSON `samples_qualified` / `captures_attempted` | ~300 from ~370 (>20 % rejections is normal: torn UDP frames) | rejections near 100 % with reasons mentioning the tone-free routes → the loaded TXT does not match the CLI geometry, or the link is half-synced (reset, re-probe) |
| `skew_observable` on every row | `dither_fold` (or `dither_phase` if the route check said so) | a row with `phase` means the tone-free option did not take effect |
| `gain_source` on every row | `dither_mag` | anything else is now a *rejection*, so a fallback cannot hide inside a converged-looking run |
| `skew_used_ps` (the learning curve) | falls to the deadband in a handful of batches | a *rising* magnitude is the sign fault this section exists to prevent: stop, and re-run `_tone_free_loop_test.py` before trusting the bench |
| `skew_action` per batch | at most one code per 20-frame batch, `move±1:OK` | `low-yield` means the batch filter is discarding frames; `abort:direction` latches skew actuation — that is the guard working, and it means the bench moved differently from the calibrated 4.8–7.4 ps/code |
| `skew_batch_mean_ps` vs `skew_batch_se_ps` | a mean several times its own standard error | a batch mean inside its error bar must not move the actuator — check `skew_min_yield` and the frame count |
| meta JSON `skew_abort` | `null` | a string is the reason actuation stopped; the digital loop keeps running |
| `gain_mag_ratio`, `offset_b−offset_a` | gain → 1.0000 (broadband), offset → ~0.1 codes signed | remember `|dOffset|` rows are scatter, never a bias (AGENTS.md) |
| `skew_fold_ps` vs `skew_slope_ps` (both logged) | the controlled one moves to zero; the other tracks it within its scatter | a large *disagreement between the routes* is a frame worth looking at — they measure the same delay by different arithmetic |

**What is *not* readable from this run.** With no tone, `tone_ratio`, `raw_difference_dbc`,
`cal_difference_dbc`, `raw_*_sndr_db` and the SFDR/ENOB columns are scored at f_in and describe the
noise floor. The tone-free stand-ins are `gain_mag_ratio`, `offset_*_codes`, `skew_used_ps`,
`dbc_ab_coherent` (coherent A-B power relative to the channel) and `snr_dither_db` (coherent dither
power over what remains after subtracting it). The loop's own summary prints exactly those for a
tone-free run.

**Expected precision.** The bench measured 15.2 ps per frame on the timing route at amp16000 with
the 6-sample pulse (45 ps on the long one, 2026-09-19). A 20-frame batch then has ~3.4 ps of
standard error against a 10 ps deadband, and one actuator code (4.8 ps) is resolved at ~1.4σ per
batch, so a handful of batches is what converges. Raising the amplitude per stage F is what would
tighten that, and its own result says by how much.

