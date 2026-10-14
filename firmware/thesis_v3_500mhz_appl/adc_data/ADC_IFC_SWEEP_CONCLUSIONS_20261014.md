# ADC IFC Sweep Conclusions — 2026-10-14

## Test conditions

- Input: 250 MHz sine wave from the waveform generator.
- Generator setting: 1.000 Vpp into 50 ohm, with zero DC offset.
- ADC sample rate: 1.3 GSPS.
- IFC settings: 2.04, 1.93, 1.81, 1.70, 1.59, 1.47, and
  1.36 Vpp differential full scale.
- Direct test: the same generator cable was connected to Channel A and Channel
  B sequentially, without changing the generator settings.
- Splitter test: both ADC inputs were driven through the splitter. Its
  independently measured insertion loss was approximately 9.6 dB.

The voltages below are equivalent differential ADC-input Vpp inferred from the
coherent tone amplitude and the selected IFC range. They are not direct voltage
measurements at the ADC die.

## Valid-sample rule

The UDP transport sends the complete 4096-byte DDR region, but production
`adc_frame.c` uses only the first 2032 decoded ADC words. This gives 1016 valid
samples per channel. The GUI process used for these three captures had been
started before the host-side tail-trimming fix, so its original CSV files contain
1024 rows per channel. The last eight rows must not be analysed.

The authoritative results are the `ifc_sweep_summary_valid_words.csv` files,
which were recomputed from the first 1016 samples per channel. The receiver now
applies the 2032-word limit before de-framing, fitting, clipping checks, and CSV
export, so future runs will exclude the invalid tail automatically.

## Direct single-channel results

| Measurement | Result |
|---|---:|
| Channel A equivalent input | 384.340 +/- 0.803 mVpp |
| Channel B equivalent input | 383.532 +/- 0.690 mVpp |
| Mean of both channels | 383.936 mVpp |
| Mean B/A ratio | 0.997900 |
| B relative to A | -0.210% (-0.018 dB) |
| IFC linearity, Channel A | R^2 = 0.999833 |
| IFC linearity, Channel B | R^2 = 0.999821 |
| Maximum IFC product error | 0.40% |
| Clipping | None |

At IFC = 1.59 Vpp, the fitted peaks were 1984.843 codes on Channel A and
1977.817 codes on Channel B. The direct test therefore shows that the two ADC
board input paths differ by only about 0.21% when measured sequentially with the
same source and cable.

## Splitter results

| Measurement | Result |
|---|---:|
| Channel A equivalent input | 122.818 +/- 0.250 mVpp |
| Channel B equivalent input | 124.550 +/- 0.240 mVpp |
| Mean of both channels | 123.684 mVpp |
| Mean B/A ratio | 1.014104 |
| B relative to A | +1.410% (+0.122 dB) |
| IFC linearity, Channel A | R^2 = 0.999844 |
| IFC linearity, Channel B | R^2 = 0.999781 |
| Maximum IFC product error | 0.39% |
| Clipping | None |

The direct result predicts 127.133 mVpp after the independently measured
9.6 dB splitter loss. The observed 123.684 mVpp is 2.71% lower, a difference of
only 0.239 dB. Equivalently, the direct-versus-splitter measurements give an
insertion loss of 9.839 dB. This is reasonable closure with the approximately
9.6 dB bench measurement.

After removing the direct A/B input-path ratio, the splitter branches and their
associated connections contribute a relative B/A factor of 1.016237, or about
+1.624% (+0.140 dB).

## Comparison with the earlier sweeps

The 2026-10-06 250 MHz, 1 Vpp generator-plus-splitter sweep reported an average
63.538 mVpp and B/A = 1.014051. Its relative channel result is reproduced almost
exactly by the new splitter result, B/A = 1.014104, so the relative IFC and
channel-ratio conclusion was sound.

Its absolute amplitude should not be used, however. It is 5.79 dB below the new
splitter result and approximately 6 dB below what the direct measurement plus
the known splitter loss predicts. A different generator 50-ohm/High-Z display
setting is the most likely explanation: a 1 Vpp High-Z setting delivers about
0.5 Vpp into 50 ohm. This explanation should remain labelled as an inference
unless the old generator configuration is independently confirmed.

The earlier full-scale DAC-through-splitter runs measured:

| Frequency | Equivalent ADC input | B/A |
|---|---:|---:|
| 250 MHz | 130.799 mVpp | 1.05411 |
| 300 MHz | 127.236 mVpp | 1.04902 |
| 400 MHz | 123.854 mVpp | 1.06896 |

Those 4.9% to 6.9% ratios describe the complete DAC-to-ADC path used in those
runs. They must not be presented as intrinsic ADC channel mismatch because they
include the DAC path, splitter branches, cables, connectors, and ADC input
networks.

## Final conclusions

1. The programmed ADC IFC scaling is verified: tone amplitude follows `1/IFC`
   across all seven settings, with less than 0.4% maximum product error and no
   clipping in the new direct or splitter tests.
2. The sequential direct test measures only about 0.21% A/B mismatch in the two
   ADC board input paths.
3. With the splitter installed, the end-to-end A/B mismatch is about 1.41%.
   This is the relevant gain-calibration target when the splitter is part of the
   operational signal path.
4. The direct-versus-splitter amplitude comparison measures 9.839 dB insertion
   loss, agreeing with the independent approximately 9.6 dB measurement.
5. The old 63.538 mVpp absolute result should be excluded from final reporting;
   its relative B/A result remains repeatable.

## Data files

- `ifc_single_a_verification_20261014_174553/`
- `ifc_single_b_verification_20261014_173546/`
- `ifc_verification_20261014_175536/`
- `ifc_single_ab_comparison_20261014.csv`
- `ifc_direct_vs_splitter_comparison_20261014.csv`
