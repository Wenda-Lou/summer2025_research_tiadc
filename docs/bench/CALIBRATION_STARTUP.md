# Calibration start-up procedure (RF combiner setup)

Each time you come to the lab to run the on-board closed-loop calibration, work through this checklist from top to bottom. **Confirm each step is OK before moving on to the next.**
Applies to: `firmware/thesis_v3_loop_appl` (reference-free, dither-only on-board closed loop), on the PC whose console is COM7.

```
Clocks (Holzworth HSX9002A)
  ch2  325 MHz    ──► J202 (FPGA transceiver reference clock)
  ch4  1.300 GHz  ──► P202 (ADC sample clock)

Signal
  AD9164 DAC (DPG plays the dither) ───────────────┐
                                                   ├─► RF combiner ──► splitter ──┬──► ADC channel A
  DG922 Pro CH1 (tone) ──► low-pass filter (opt.) ─┘                              └──► ADC channel B
```

> **Do not touch** the two cables from the splitter to the ADC. Moving them shifts the skew zero point.

---

## 1. Holzworth HSX9002A (clocks)

1. Open **Holzworth HSX/HSY Control Software** → **Devices** and select `HSX9002A-618`.
2. Settings:

   | Channel | Frequency field | Frequency | Power | Phase | Connect to |
   |---|---|---|---|---|---|
   | ch2 | `00 325 000 000 000` | 325 MHz | `+000.00` dBm | `+000.000` | J202 |
   | ch4 | `01 300 000 000 000` | 1.300 GHz | `+000.00` dBm | `+000.000` | P202 |
   | ch1, ch3 | leave unchanged | — | — | — | not connected, leave on |

3. **Power On/Off Master** must be green and **Enable** must be on for ch2 and ch4; press **Set**.
4. **The clocks must be set before the ZCU102 boots / is Run.** After any clock change the board must be Run again (step 4).

---

## 2. ACE (configure the AD9164 DAC)

1. Open **ACE** and double-click **AD9164-FMCB-EBZ** under Attached Hardware.
   - Board not found: open DPG Lite first and check that **FMC Power** is on (the button reads *Turn Off* when powered); unplug and replug the mini USB on the DAC board; then press **Refresh Attached Hardware**.
   - Plugin error pop-up: close ACE and open it again.
2. **AD916x Startup Wizard**:

   | Item | Setting |
   |---|---|
   | Operation Mode | SERDES Mode |
   | DAC Clock Source | ADF4355+ADCLK914 |
   | Reference Clock | **100 MHz** |
   | FDAC | **2.6 GHz** |
   | Interpolation | 1 |
   | Serdes Lanes | 8 |

3. Press **Apply**; **JESD PLL Locked** (green) must appear on the right.

---

## 3. DPG Lite (DAC plays the dither)

1. **Add Data File** and choose a dither-only file (`waveforms\dither_only\`):

   | File | Dither repetition rate | Use |
   |---|---|---|
   | `w06adc_e04_t04_amp32767_dither_only.txt` | 10 MHz | Standard; least interference |
   | `w06adc_e04_t04_amp32767_dither_only_p130.txt` | 20 MHz | Tolerates a larger tone |
   | `w06adc_e04_t04_amp32767_dither_only_p26.txt` | 100 MHz | Limit testing only; strong interference with the normal signal |

2. **Data Format: Signed** (not unsigned), Resolution 16 bits.
3. Press **Download** (blue arrow), wait for the progress bar to finish, then press **Play** (green triangle).
4. **SYNC Status must show a green check.** Back in ACE press **Update SERDES STATUS**; every lane indicator must be lit.
   - Still a red ✕: press **Init PLL**, then download and play again.

---

## 4. Vitis (load the firmware) + COM7 terminal

1. Open Vitis → `thesis_v3_loop_appl` → **Run**.
2. Open a serial terminal on **COM7** (115200 baud). The boot messages **must include**:
   ```
   ad9695 PLL LOCKED
   JESDPHY: RX reset complete and QPLL locked.
   ```
3. Check the clocks once more:
   ```
   adc -c
   ```
4. **Initialize the skew actuator** (required after every Run):
   ```
   adc -cal skew step 0
   ```
   Expect `RESET_REG = 0x00000000`, `result=OK`, `neutral_initialized=YES`, and code 24.

> **Do not use the arrow keys** to recall a previous command: the terminal sends them as characters (`[A[B`).

---

## 5. ADC full scale (IFC, optional)

In the COM7 terminal:
```
adc -gain
IFC
set 1.36
status
quit
```
- Allowed values: 1.36, 1.47, 1.59 (default), 1.70, 1.81, 1.93, 2.04 Vpp. **1.36 is the minimum** and gives the most codes for a given signal (about 17 % more).
- Use `status` to confirm `Register 0x1910` and `Current full-scale`.
- **Always type `quit` to leave the menu**; otherwise later `adc -cal loop` commands are parsed as IFC commands (`Invalid IFC command`).
- After a re-Run the IFC returns to the default 1.59 and must be set again.
- A smaller full scale only gives the same signal more codes; **the ADC noise is scaled up with it**, so the SNR barely changes.

---

## 5.1 DG922 Pro (tone, optional)

| Item | Setting |
|---|---|
| Waveform | **Sine** |
| Frequency | **199.375 MHz** (**not exactly 200 MHz**: every dither pulse would see the same tone phase) |
| Load (Output → Load) | **50 Ω** (with High-Z the actual output is half the panel value) |
| Amplitude | Start small; see the table below |
| Reference clock (optional) | External 10 MHz (Holzworth 10 MHz Ref Out → DG922 Ref In) |
| Output | Off when testing dither only |

Measured today (2026-11-04): when the tone is too large the estimator rejects every frame (`st=events`):

| Dither | Approximate usable tone limit (DG922 setting) |
|---|---|
| 10 MHz | ~280 mVpp (all frames rejected at 340 mVpp) |
| 20 MHz | ~400 mVpp |
| 100 MHz | ~800 mVpp |

---

## 6. Before each run

1. Press Enter in the terminal; you must see `uart-cmd$:` (confirms the board is idle and not still running the previous run).
2. **Return the code to 24** (skip this if the previous run used `--park-after 24` and printed `every transaction result=OK`), either:
   - in the terminal: `adc -cal loop test park 24`
   - or, **after disconnecting the terminal**, in PowerShell at the repo root:
     ```powershell
     python tools/skew/skew_park.py --uart COM7 --code 24 --verify-frames 0
     ```
3. **Disconnect the COM7 terminal** (Disconnect in Vitis / VS Code Serial Monitor). The tool in the next step needs exclusive access to COM7.

---

## 7. Run the calibration

All commands run in PowerShell at the repo root, with the COM7 terminal disconnected. The tool is `tools/board_loop/board_loop_log.py`; it sends one `adc -cal loop ...` command, streams every record to the PC, and saves the results.

### 7.1 Standard run (300 frames, closed loop)

```powershell
python tools/board_loop/board_loop_log.py --uart COM7 --out calibration_out/combiner_tone400mV_dither20Mhz_run300.log
```

Name the run `combiner_tone<amplitude>mV_dither<rate>Mhz_run300.log`. With no tone, use `combiner_notone_dither<rate>Mhz_run300.log`. If the run did not start at code 24, add the start code, e.g. `..._start12_run300.log`.

### 7.2 Other commands

| Goal | Command (add `--uart COM7 --out calibration_out/<name>.log`) |
|---|---|
| Standard closed loop, 300 frames | *(no extra option)*; the default is `--cmd "adc -cal loop 300"` |
| More frames | `--cmd "adc -cal loop 600"` |
| Measure only; the actuator does not move | `--cmd "adc -cal loop open 60"` |
| Return the actuator to a code when the run ends | `--park-after 12` (any code 0..48; see 7.5) |
| Download the last run again, without recalibrating | `--cmd "adc -cal loop export"` |
| Do not open the browser at the end | `--no-viewer` |
| Old flat layout (files directly in `calibration_out/`) | `--flat` |
| Longer wait for a slow run (default 1800 s) | `--timeout 3600` |

`--wave` is legacy and not needed: the firmware is reference-free and does not need to know which file DPG is playing.

### 7.3 Where the results go

`--out calibration_out/<name>.log` saves everything in its own folder:

```
calibration_out/<name>/
  <name>.log    compact UART console log
  <name>.csv    every frame record (accepted and rejected)
  <name>.json   summary: last-20-frame means, corrections, final code
  <name>.png    curves of skew, gain, offset, corrections and code
  <name>.svg    the same figure as a vector image
  <name>.html   interactive viewer, opened in the browser automatically
```

If `calibration_out/<name>/` already holds a run, the new run goes to `<name>_<date>_<time>/`, so nothing is overwritten. If the run stops early, the `.log` and the rows received so far in the `.csv` are kept.

To reopen or rebuild a viewer later (VS Code has no HTML preview; this opens it in the browser):
```powershell
python tools/board_loop/loop_viewer_export.py calibration_out/<name>/<name>.csv --open
```
It also takes a single-axis sweep folder, e.g. `calibration_out/axis_20261028_203116`.

### 7.4 How to confirm the run succeeded

At the end the console prints `RESULT qualified=... rejected=... code=...` and the last-20-frame means. Check:
- `LOOP-START` shows `code=24` (or your chosen start code) and `reference_free=1`
- `LOOP-END qualified=300/300`, `skew_abort=0`
- Every `LOOP-SKEW-TXN` is `result=OK`
- The last few batches are `inside-deadband`, and the last-20-frame means of gain, offset and skew are all close to 0
- The command exits without `stopped:`, `error:` or `WARNING`

### 7.5 Returning the actuator automatically (`--park-after`)

```powershell
python tools/board_loop/board_loop_log.py --uart COM7 --out calibration_out/<name>.log --park-after 24
```

- After the run, it sends `adc -cal loop test park 24` and walks one code at a time through the ACK-checked transaction.
- The park is sent only if the run ended cleanly. After a timeout, `Command Error` or `RECOVERY_REQUIRED` it is skipped and the log says `park-after: skipped`.
- Success prints `park-after 24: code=24 after N moves, every transaction result=OK`. Anything else prints a `WARNING` and the command exits with 1. The run's data is still saved. Check the `LOOP-SKEW-TXN` lines in the log before the next run.
- `--park-after 24` resets for the next matched run, which replaces step 6.2. Parking at the null (e.g. `--park-after 12`) suits steady-state or tone-limit runs, but 12 is the null of the current bring-up only. After a re-Run, run once from 24 and use the code it stops at.

---

## Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| `DMA capture timeout`, `RESET_REG = 0x00000081` | JESD link down, usually a clock problem | **Do not issue more skew commands.** Check with `adc -c`: `QPLL not locked` points to J202 (ch2), `PLL UNLOCKED` to P202 (ch4). Fix the clock, then Run again in Vitis |
| `RECOVERY_REQUIRED` | Skew transaction failed | Do not retry; check the clocks first, then Run again |
| `Invalid IFC command` | Stuck in the `adc -gain` menu | Type `quit` in the terminal |
| `could not open port 'COM7'` | The terminal is still connected to COM7 | Disconnect the terminal |
| `data arrived without its export header` | The previous run is still going (Ctrl+C only stops the PC side; the board keeps running) | Wait for the board to finish and show `uart-cmd$:`, then run again |
| Every frame is `st=events` | Tone too large, or the DPG is not playing a dither-only file | Lower the DG922 amplitude; check the DPG file and the green SYNC |
| `clipping` rejects | ADC clipping | Lower the DG922 amplitude |
| `code=-1` | Actuator not initialized after a re-Run | `adc -cal skew step 0` |
| ACE cannot find the DAC board | DAC board unpowered or USB not connected | DPG Lite FMC Power, DAC board USB, restart ACE |

---

## Shutdown

1. Turn the DG922 **Output off**.
2. Stop playback in DPG Lite.
3. Power down the rest of the equipment in the usual order.
