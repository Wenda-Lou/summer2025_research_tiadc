# Independent Impulse-Coupling Feasibility Note

Internal planning note | 27 August 2026

## Purpose and Boundary

This note outlines possible future ways to inject an impulse independently of the primary mission-signal generator. It is a feasibility comparison only. None of these paths has been built or validated on the current bench, and all loss/distortion values below are design-dependent estimates rather than measurements.

| Option | Expected insertion loss | Impedance matching | Bandwidth and pulse distortion | Implementation complexity | Effect on mission-signal source |
|---|---|---|---|---|---|
| Resistive summing / injection resistor | Can be kept below roughly 1 dB for the mission path with a weak injection branch, but injection efficiency then falls; a symmetric passive summer can incur several dB. | Requires a 50-ohm small-signal model of both sources and the ADC input. A simple resistor can change source/load impedance and isolation. | Resistors are intrinsically broadband, but PCB/package parasitics and the ADC input network can round edges or cause reflections. Pulse amplitude depends strongly on the resistor ratio. | Low component count, but careful calculation and a small RF test board are required. Source-to-source isolation may require extra attenuation or buffering. | Usually requires access to, or an inline modification near, the mission-signal summing node. The primary generator electronics need not be redesigned, but its load can change. |
| Capacitive coupling | Near-zero ideal through-path DC loss, with small RF loading if the coupling capacitance is small; injection amplitude is strongly frequency-dependent. | The capacitor, trace inductance, source impedance, and 50-ohm line form a high-pass network. Bias and common-mode constraints at the ADC input must be preserved. | Potentially wideband, but sharp impulses are especially vulnerable to droop, bipolar tails, ringing, and amplitude uncertainty. A return path and controlled layout are essential. | Low part count but moderate RF design risk. Simulation and oscilloscope/VNA characterization should precede ADC testing. | Can leave the primary source unchanged while adding a local injection tap, although the signal path or PCB still needs a physical coupling point. |
| 2-way RF combiner | A conventional passive matched combiner normally costs about 3 dB per input path before connector/cable excess loss. Transformer or reactive combiners vary with frequency. | Best-defined 50-ohm interface and useful source isolation when operated in band. Both sources must tolerate the combiner isolation and any reflected power. | Commercial bandwidth is explicit. Operation near band edges can broaden or ring pulses; a very wideband unit is needed to preserve fast edges. | Lowest bench risk: off-the-shelf parts, cables, and attenuators. The 3 dB signal penalty may require more mission-source headroom. | No modification inside the primary source, but the combiner is inserted inline and therefore changes mission-path amplitude. |
| Directional coupler | Main-line through loss can be well below 1 dB in band; the injected path is deliberately attenuated by the coupling factor, commonly on the order of 6-30 dB. | Provides a controlled 50-ohm main line. Directivity and termination quality matter; the auxiliary port must be correctly terminated and driven. | Band-limited like the combiner. Coupling flatness, group delay, and limited low-frequency response can reshape an impulse. Very wideband couplers are more specialized. | Moderate: easy to insert mechanically, but selecting coupling, bandwidth, and available impulse-source amplitude requires care. | Leaves the primary generator unmodified and minimally loads the through path, but adds an inline RF component and requires more injection-source amplitude. |

## Practical Screening Sequence

1. Characterize each candidate with a 50-ohm source/load using through loss, coupled amplitude, and time-domain pulse shape as the primary criteria.
2. Use the existing board sweep's low-disturbance region as an initial target, but re-measure the actual ADC-domain amplitude because every coupling network will reshape and attenuate the pulse.
3. Verify tone-path amplitude, RMSE, and spectral spurs with the injection source disabled and enabled before attempting event detection.
4. Treat the RF combiner as the simplest independent bench demonstration; evaluate a directional coupler if preserving main-line amplitude is more important and sufficient impulse-source headroom is available.

## Decision Needed

The key choice is whether the publication needs only an independent bench demonstration or a compact integration concept. The former favors an off-the-shelf wideband combiner/coupler; the latter justifies a purpose-built resistive or capacitive injection board with measured S-parameters and pulse response.
