# Dither-only bench session — checklist and results

Run on **2026-09-19** on the ZCU102/AD9695 bench, per the 19 September direction: **no tone-based
calibration**, **a shorter dither**, **different fixed settings recorded long enough**, and first
confirm that gain, offset and skew can be detected reliably. Each section keeps its procedure next
to its measured result; §5 records the traps this session paid for.

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
