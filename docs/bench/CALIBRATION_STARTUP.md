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
2. **Return the code to 24**, either:
   - in the terminal: `adc -cal loop test park 24`
   - or, **after disconnecting the terminal**, in PowerShell at the repo root:
     ```powershell
     python tools/skew/skew_park.py --uart COM7 --code 24 --verify-frames 0
     ```
3. **Disconnect the COM7 terminal** (Disconnect in Vitis / VS Code Serial Monitor). The tool in the next step needs exclusive access to COM7.

---

## 7. Run the calibration

In PowerShell at the repo root:
```powershell
python tools/board_loop/board_loop_log.py --uart COM7 --out calibration_out/combiner_tone400mV_dither20Mhz_run300.log
```

- Suggested file name format: `combiner_tone<amplitude>mV_dither<rate>Mhz_run300.log`; with no tone, use `combiner_notone_...`.
- Results are saved in the `calibration_out/<name>/` folder, containing `.log`, `.csv`, `.json`, `.png`, `.svg` and `.html`. A reused name gets a timestamp appended automatically.
- When the run finishes, the `.html` viewer opens in the browser automatically.

**How to confirm the run succeeded**:
- `LOOP-START` shows `code=24` and `reference_free=1`
- `LOOP-END qualified=300/300`, `skew_abort=0`
- Every `LOOP-SKEW-TXN` is `result=OK`
- The last few batches are `inside-deadband`, and the last-20-frame means of gain, offset and skew are all close to 0

Measure only, without moving the actuator: `--cmd "adc -cal loop open 60"`.

Return the actuator to a chosen code automatically when the run ends: add `--park-after 12` (any code 0..48).
- The park is sent only if the run ended cleanly; after a timeout, error or `RECOVERY_REQUIRED` it is skipped.
- The console prints `park-after 12: code=12 after N moves, every transaction result=OK`. Anything else prints a `WARNING` and the command exits with 1; the run's data is still saved.
- 12 is the null of the current bring-up only. After a re-Run, run once from 24 and use the code it stops at.
- Put the start code in the next run's name, e.g. `..._start12_run300.log`, since a run started at the null is not comparable with one started at 24.

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
