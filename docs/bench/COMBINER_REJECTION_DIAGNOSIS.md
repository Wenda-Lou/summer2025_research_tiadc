# Combined tone/dither rejection diagnosis

## Fresh board cancellation confirmed, 2026-11-19

The final run `dither_cancel_verify_v2_20261119_124959` completed 600/600
captures, zero rejects, code 13 and skew_abort=0. Its separate frozen output
reports revision 2, `status=cancelled reason=validated`, 65-sample spacing,
15 events, four-sample core and 31-sample support. Independent-event pulse
energy reductions are 22.468 dB on A and 22.274 dB on B. The signal sidecar
confirms `dither_cancelled=true`, source `frozen_board_signal_snapshot`.
This is fresh on-board output validation, rather than offline reprocessing.
It does not provide independent tone metrology or an independent HOLD score:
the calibration tail is explicitly feedback=active/training_residuals_only.

The immediately preceding `dither_cancel_verify_v2` run also completed 600/600,
but output was `no-tone reason=tone-dominance`; it is retained as failed
admission. Use the final timestamped run's page when reviewing cancelled data.

## Fresh-output ringing boundary, revision 2

The new `dither_cancel_verify` closed run completed 300/300 frames at code 12
with no rejection or abort, but output remained `ambiguous`. Replaying its own
saved signal (rather than the earlier skew snapshot) reproduced the failure:
sixteen candidates spaced by 65 samples, with core width four and learned
support width 33. The initial half-period patch allowed only floor(65/2)=32.
It had not covered this new frame's weak tail extent.

Revision 2 separates strong-core sparsity from weaker ringing extent. The
outlier core must remain <= period/3; the learned, independently validated
support can extend to 3*period/4, leaving at least a quarter-period of quiet
samples. Event-spacing, group counts, independent improvement/residual checks
and non-pulse residual checks all remain mandatory. Persistent wide cores and
unsupported data still pass through without a cancellation claim.

The fresh recorded frame is now a second fixture,
`calibration_sim/fixtures/output_ringing_boundary.csv`. It passes offline
production-C replay at 25.86/26.51 dB pulse-region reduction. A separate tone
check using the untouched samples finds amplitude changes 0.085%/0.006% and
phase changes 0.025/-0.017 degrees; residual RMS drops from about 66 codes to
5.05/4.89 codes. These are output-only diagnostics, never calibration feedback.

The board's test status, LOOP-OUTPUT and LOOP-SIGNAL-BEGIN now advertise
`output_revision=2`. Output/signal rows include `reason`, `core_samples` and
`support_samples`, including on non-cancelled frames. Host sidecars preserve
these diagnostics. Load the new ELF and check the revision before a fresh
verification run; existing snapshots and HOLD results are not rewritten.

## Baseline direction fault, 2026-11-19

`axis_suite_20261119_104908` accepted all 100 attempted baseline TRAIN frames,
then latched `abort:direction` at attempt 99, code 20. Subsequent suite attempts
105237 and 105257 saw `fault=1` in their first status command and sent no test
writes. This was neither a CLI argument error nor a clipping/DMA failure.

The code 21 batch mean was 67.113 ps (SE 2.493); the next code 20 batch mean
was 67.223 ps (SE 1.963). The old check saw a positive 0.110 ps change versus
the expected negative 19.1 ps and latched, despite the combined SE of 3.173 ps.
The direction guard now requires an opposite-signed change exceeding three
combined SEs, in addition to the existing expected-step discrepancy test.
Unknown uncertainty keeps the previous fail-closed check; failed actuator ACKs
still stop unconditionally. Regression cases cover the noisy near-zero shift,
a clearly reversed shift, unknown uncertainty and the existing lying-bench
one-move stop. This guards against noise-triggered direction claims; it is not
evidence that the physical code 21->20 step has the nominal size or even a
measurable response. Calibration/HOLD precision gates remain unchanged.

The host's latched-fault error now explicitly requests an ELF reload, bring-up
and confirmation of `fault=0`. A fault cannot be cleared by rerunning Python,
changing acquisition thresholds or issuing test restore. Restart the firmware
with the rebuilt ELF; inspect clocks, initialize via `adc -cal skew step 0`,
then read `adc -cal loop test status` before starting another suite.

## Output cancellation follow-up, 2026-11-19

Replaying the exact production output stage on the final skew32 corrected
snapshot reproduced `ambiguous`: sixteen pulse candidates have spacing 65
samples and zero spacing-fit error; fifteen complete events form groups of
eight and seven. The learned support is 30 samples wide, exceeding the old
`period/3` cap (21). No subtraction was performed on the board's saved frame.

The output-only cap now permits `period/2`, retaining at least half a period
outside the subtraction region. Opposite-event validation, minimum event
counts, phase/spacing checks and the quiet-region criterion remain unchanged.
No calibration estimator, correction coefficient or HOLD score is modified.

All five final skew snapshots pass offline production-C replay. Their A/B
pulse-region validation reductions are 25.56/26.93, 23.80/23.80, 25.68/26.25,
24.26/24.10, and 23.13/23.31 dB for starts 16,20,24,28,32. For skew32, the
full-frame residual RMS about the main-tone fit drops from approximately
65 codes to 6.23/5.91 codes. An output-only tone diagnostic (before fit on
unchanged samples, after fit on the complete output) finds amplitude changes
below 0.12% and phase changes below 0.044 degrees across all five snapshots.
These diagnostics are not calibration feedback or independent tone metrology.

The archived 1020-sample skew32 frame is retained as
`calibration_sim/fixtures/output_wide_impulse.csv`, with provenance in its JSON.
The CMake target `adc_signal_replay` writes new cancelled CSV/JSON; regenerated
pages explicitly use these new files. Original non-cancelled records are
retained. New firmware is built, but fresh on-board output still needs loading
that ELF and completing a new run; reading an old snapshot does not reprocess it.

## Final single-axis results, 2026-11-18

The three-axis tests show substantial error correction, with full precision
admission demonstrated only for the five tested positive skew starting points.
Each completed point has 100/100 independent HOLD frames and frozen coefficients
and clock code. Apply the unchanged score `abs(HOLD mean) + 1.96*SE <= tolerance`:

| Axis | Injection / starting points | HOLD mean range | Passing points |
|---|---|---|---|
| Offset | Software post-ADC +32 down to -32, step -1 | -0.66646 to +0.73582 codes | 47/65 at 0.5 code |
| Gain | Hardware IFC 1.36--2.04 Vpp, all seven settings | -0.096593% to +0.135131% | 0/7 at 0.1% |
| Skew | Absolute hardware codes 16,20,24,28,32 | -2.7861 to -0.9475 ps | 5/5 at 10 ps |

Offset/gain source records: [first sweep](../../calibration_out/axis_suite_20261118_155214/summary.csv)
and [IFC=0 completion](../../calibration_out/axis_suite_20261118_172541/summary.csv).
Exclude the first sweep's aborted IFC=0 row (no HOLD); use its completed repeat
from the second run. Gain initial mean errors span -22.17559% to +17.31309%.
Six settings have mean residual magnitude below 0.1%, but every setting fails
the full score (bounds 0.13131%--0.21870%). Do not present this as a precision pass.

The [final skew sweep](../../calibration_out/axis_suite_20261118_180755/summary.csv)
completed all five points and restored the saved baseline. All PRE 40/40,
TRAIN 600/600 and HOLD 100/100 frames qualified. PRE/TRAIN used the explicit
750-mrad phase gate; all HOLD phases returned to the strict 150-mrad gate.
Initial estimated skew ranged from +64.4085 to +288.35775 ps; all five runs
reached code 12, with the worst HOLD score 3.86893 ps. Code 32 needed twenty
single-code moves, arriving at code 12 on TRAIN attempt 399 (400 frames).
All 132 transactions in that run's console log were OK, with no DMA timeout
or RECOVERY_REQUIRED. Output snapshots remain `ambiguous`; cancellation is
not established by the successful skew calibration.

The committed run records include summary/meta, per-frame CSV, console and
baseline logs/JSON. Generated plots and HTML are retained locally. These are
separate single-axis runs, one repeat per point, and board-estimator results
without independent timing metrology. The measurements do not establish joint
three-axis convergence, negative-direction skew acquisition, or full-range /
long-term repeatability. Offset injection is software, not analog ADC offset.

## 2026-11-18 skew sweep follow-up

The completed `axis_suite_20261118_172541` run restored its baseline and had no
DMA timeout. Skew code 16 had 40/40 PRE rejects, all `phase-spread`: spectral
phase RMS 0.320--0.680 rad against the 0.150-rad gate, with de-framing margin
20.16--56.90. This is acquisition rejection, not a failed 10-ps HOLD score.
Code 0 had margin 1.46--1.66 and phase RMS 1.548--2.048 rad; code 8 had margin
3.87--13.01 and phase RMS 0.597--1.384 rad. Relaxing code 16's phase gate does
not establish identifiability at codes 0 or 8.

Code 32 accepted all 300 TRAIN frames but could make only 15 moves: batch=20
accepted frames and at most one ACK-checked hardware code per decision. It
stopped at code 17, still about +84 ps in independent HOLD. The 300-frame
budget, rather than a failed transaction, prevented reaching baseline code 12.
Use TRAIN=600 for this wider scan; rejected attempts consume the same budget.

An explicit experiment option is now implemented (limited board results above):
`adc -cal loop test quality PHASE_MRAD`, supported values 150--750, or suite
`--acquire-phase-mrad 750`. It changes only manual PRE/TRAIN. Baseline, HOLD,
step probes, output and ordinary closed runs keep the 0.150-rad default.
The independent de-framing margin=6, energy/identifiability checks, hardware
ACK/direction checks and HOLD precision are unchanged. The host refuses an
explicit relaxed request on an older ELF before any test writes. Metadata and
phase headers record the requested/applied gates. This is an experimental
acquisition setting; conclusions must stay within the measured scope above.

After loading the updated ELF and completing normal bring-up, retest:

```powershell
python tools/board_loop/board_axis_test.py suite --uart COM5 --axes skew --skew-points=16,32 --train 600 --acquire-phase-mrad 750 --execute
```

The original investigation below began with source analysis and offline
reproduction; subsequent spectrum diagnostics and board results are above.

## Implemented correction

The subsequent `combiner_baseline_range500` board run accepted all 600 TRAIN
and 100 HOLD frames and moved code 24 to 12 through twelve successful one-code
transactions. No abort occurred. HOLD offset (-0.03294 ADC codes) and skew
(-1.9044 ps) passed; gain (+2836.39 ppm) failed the unchanged 2000-ppm baseline
limit. The baseline remained invalid. These recorded results are retained.

The gain freeze strategy now uses the same training-tail window as offset:
the last `min(100, TRAIN/2)` attempts, at least 20 valid observations at one
actuator code. For each accepted training frame, undo its PRE-update digital
gain with `log(native_ratio) = log(measured_ratio) + log(cA) - log(cB)`.
Average this input-domain log ratio and freeze the corresponding relative
correction while preserving common geometric gain. A code move clears the
accumulator. A failed/rejected last attempt, insufficient data or fault prevents
the refinement. The final TRAIN row records the committed coefficients; fresh
HOLD starts from exactly that state. This applies to baseline training and
single-axis gain training. HOLD samples never enter the estimator and neither
HOLD limits nor LMS learning rates are changed.

An offline counterfactual from this run's TRAIN-only tail gives native ratio
1.0090130 and would place its recorded HOLD gain at about +651.66 ppm. This
uses HOLD only for evaluation, not coefficient choice. It is not a new board
run or a raw ADC replay; the original failed assessment is unchanged. Details
are in `calibration_sim/output/gain_training_freeze_replay/analysis.json`.

All 9048 C checks, 18 scenarios and 14 pipeline scenarios pass after the gain
freeze correction. Tests include a last-frame 3% ratio outlier, independent
HOLD rejection of a real gain drift, and final-TRAIN/HOLD coefficient continuity.
Download the new ELF to enable this policy; `test config` on the previous ELF
cannot enable it. Confirm `gain_freeze=training_mean` in `test status` and
`source=dither_gain_training_mean` after TRAIN. Use a new output stem such as
`combiner_baseline_gainmean.log` for the next unchanged-input baseline run.

The first post-fix board run, `calibration_out/combiner_baseline_fixed`, passed
the spectral gates on all 700 attempts. It then rejected 695 as `skew_range`:
the initial skew averaged 208.59 ps, while the manual test's old default
`skew_milli=250` allowed only 192.31 ps at 1.3 GS/s. TRAIN accepted 5/600 and
HOLD accepted 0/100; no skew move occurred and the code stayed at 24. The
ordinary reference-free loop and the automatic axis tool already use 500.

The manual test's boot default now also uses `skew_milli=500` (384.62 ps).
This is the estimator's initial acquisition range; frozen HOLD precision,
deadband and write/direction guards are unchanged. Explicit user configuration
still overrides the default. The existing ELF can select the same supported
range without a download: on the serial console run

```text
adc -cal loop test config 200000 500 1000 1910 20
```

This command keeps this session's gain gate, deadband, step and batch settings
and changes only the skew acquisition range. Wait for `result=OK`, close the
serial terminal, and run the existing baseline command below with a new stem.
The regression reproduces a valid 210 ps spectrum rejected by the old
0.25-sample gate, then converges from the identical capture sequence with the
0.5-sample acquisition range, before independent frozen HOLD.
All 3295 native loop/board-wrapper checks pass after this acquisition-range
correction, and the Vitis ELF compiles and links. Results are in
`calibration_sim/output/combiner_acquisition_gate_fix/`. The next real-board
baseline under `skew_milli=500` remains to be measured.

The reference-free estimator now detects a dominant narrowband peak from the
ADC cross spectrum. When its peak weight exceeds 10% of the passband weight,
it recomputes the spectra using a four-term Blackman-Harris window, excludes
the peak and six neighbouring bins on either side, and derives the broadband
threshold and gain/delay estimates from the remaining bins. No sinusoid is
fitted, no supplied frequency or waveform is used, and the output-only tone
estimator does not enter calibration feedback.

The low-sidelobe window prevents an off-bin main tone's Hann sidelobes from
being mistaken for dither after the main lobe is removed. Spectral powers are
normalised back to the original Hann energy scale. The existing minimum bin
count (8), effective bin count (4), phase-error and absolute-energy gates still
apply to the broadband remainder. Baseline/HOLD thresholds are unchanged.

The original four-level reproduction now accepts 8/8 captures at every level.
Additional tests cover a main tone with gain/phase different from the dither,
tone-only inputs with independent noise across seven frequencies and five
record lengths, and strong-main closed-loop training with independent frozen
HOLD. The negative tone-only cases must produce no usable gain/delay.

Validation of the correction: 9003 C checks, 18 scenarios and 14 pipeline
scenarios pass; the PC decoders for actual C export and signal packets pass.
The Vitis application compiles and links successfully. Four strong-main tests
use 300 qualified training frames, ACK-style one-code skew moves, then 40
fresh frozen HOLD frames. HOLD means span -179 to +293 ppm gain, +0.015 to
+0.082 ADC codes offset, and +5.18 to +5.48 ps skew (the actuator deadband is
10 ps). A separate calibration-then-output test reports `cancelled` and keeps
the frozen state unchanged. This is offline validation; the real board still
needs the new combined-input baseline run.

The firmware status route line includes `spectral_line_rejection=1`. Load the
newly built `firmware/thesis_v3_loop_appl/build/thesis_v3_loop_appl.elf`, repeat
the usual bring-up and `adc -cal skew step 0`, then establish a fresh baseline
under the unchanged combined input:

```powershell
python tools/board_loop/board_loop_log.py --uart COM5 --cmd "adc -cal loop test baseline 600 100" --out calibration_out/combiner_baseline_range500.log
```

The following sections describe the original failure and diagnostic evidence.
The old board captures have not been reclassified as passing.

## Board evidence

`calibration_out/combiner_baseline/combiner_baseline.log` and
`calibration_out/combiner_baseline_20261118_141110/combiner_baseline_20261118_141110.log`
each contain 600 rejected training attempts, all `st=events`. No baseline HOLD
started. The latter run stayed at code 12 with unity digital gain and zero
digital offsets. Its CSV reports zero clipped samples and alignment margin
well above the threshold. No raw ADC frames or spectral diagnostics were
saved, so these records cannot identify the exact failed spectral gate.

The intervening run at `combiner_baseline_20261118_140702` was identified by the
user as dither-only. It accepted 571/600 TRAIN and 100/100 HOLD attempts, then
failed baseline admission on offset (+0.580 codes, limit 0.500) and gain
(-3439 ppm, limit 2000 ppm). Skew passed. Its `restore` failure follows from
the invalid baseline and is separate from the combined-input rejection.

## Source path

The original `calibration_loop_c/adc_calibration_loop_blind.h` computed a Hann-windowed
A/B FFT, takes the largest cross-spectrum magnitude as `peak`, and keeps only
bins with weight at least `0.01 * peak`. A strong narrowband main tone can
set this peak even when broadband dither is present.

The retained bins must satisfy all of these existing gates:

| Gate | Requirement | New diagnostic reason |
|---|---|---|
| Nonzero spectrum | `peak > 0` | `no-energy` |
| Bin count | at least 8 bins, positive slope weight | `few-bins` |
| Spectral spread | `(sum(w))^2 / sum(w^2) >= 4` | `concentrated` |
| A/B phase consistency | weighted phase residual RMS <= 0.15 rad | `phase-spread` |
| Windowed spectral energy | each channel's retained FFT power / nfft^2 >= 4 | `low-energy` |

Any failed gate leaves `ch_a/b.n_events_used` at zero. `reject_reason()` then
returns `ADC_CAL_LOOP_REJECT_TOO_FEW_EVENTS`, displayed as `events`. In this
branch that label does not count physical impulses: the final `n_events_used`
would instead contain the retained FFT-bin count. It also hides phase/energy
failures that happen after the bin count has passed.

The input contains tone plus dither, but the original calibration branch neither
separates narrowband main-tone bins nor computes spread relative to a broadband
background. Raising the main tone can therefore remove the broadband bins or
make the effective bin count fall below four. A/B high correlation does not
prevent this rejection.

## Production C reproduction

`test_blind_combiner_rejection()` in `calibration_sim/sim_loop_tests.c` uses the
production DMA reconstruction, estimator and rejection gates. Only the
simulated physical input knows its waveform. The estimator receives the ADC
sample rate, with `reference_free=1`, `tone_free=1`, and all feedback frozen.
The dither is fixed at about 600 ADC peak codes, with 65-sample spacing and
6-sample pulse duration. Each level uses eight captures, 0.4-code noise and the
same seed. No samples clip. Tone-fit/purity calls remain zero.

| Simulated tone level | Tone peak (ADC codes, approximately) | Result |
|---|---:|---|
| -120 dBFS | 0 | 8/8 pass; about 114-202 retained bins |
| -30 dBFS | 259 | 6/8 pass; two `concentrated` failures |
| -18 dBFS | 1031 | 0/8 pass; `few-bins`, only 3-4 retained bins |
| -6 dBFS | 4105 | 0/8 pass; `few-bins`, only 3-4 retained bins |

For example, one -30 dBFS capture retained 29 bins but had effective bin count
3.341388, below four. A -18 dBFS capture retained three bins with effective
count 2.005200. This reproduces rejection with dither still physically present.
These amplitudes describe synthetic ADC samples, not a signal-generator panel
voltage or a measured threshold for this bench.

Full results are written by the simulator to
`calibration_sim/output/combiner_reject_diagnostic_all/test_summary.txt`.

At the earlier diagnostic-only stage, the desktop C build completed without
warnings; `--run-all` passed
6939 checks, 18 scenarios and 14 pipeline scenarios. The PC decoder checks for
the actual production C export and signal fixtures both passed. These checks
did not constitute a Vitis application build or a board validation. Current
correction validation is listed above.

## Diagnostics added for the next board run

The estimator now retains a status plus FFT length, peak bin, retained bin
count, effective bin count, phase residual and windowed spectral powers.
Rejected `loop test` attempts print an additional line, for example:

```text
LOOP-TEST-SPECTRUM phase=baseline_train attempt=0 reason=concentrated nfft=512 peak_bin=79 bins=17 excluded_bins=13 neff_milli=2125 phase_mrad=nan power_a=400 power_b=420
```

This line is a protocol example, not the missing board measurement. `neff_milli`
is effective bins times 1000; its gate is 4000. `phase_mrad` is radians times
1000; its gate is 150. `power_a/b` are rounded windowed FFT-power sums / nfft^2,
in ADC codes squared; their gate is four. `nan` means the earlier gate stopped
evaluation. `peak_bin * 1.3e9 / nfft` gives the peak frequency in Hz for this
board, without supplying a tone frequency to the estimator.

The diagnostic fields record the gate outcomes. With the correction above,
`excluded_bins` additionally reports the number of bins removed around the
dominant spectral line. Powers use the Hann-equivalent energy scale. The
existing PC logger retains these additional diagnostic lines.

Download the new ELF before collecting these diagnostics. Keep the same
combined-input condition and use the existing manual baseline command on COM5.
The first rejected attempts will distinguish `few-bins`, `concentrated`,
`phase-spread` and `low-energy`. The old UART/CSV records cannot reconstruct
these missing values. The correction has offline validation and an actual
Vitis application build; hardware acceptance still requires a new bench run.
