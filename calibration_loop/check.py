"""
Cross-check a generated DAC waveform against the rate the ADC actually ran at.

This exists because of a specific failure. The 2026-08-10 run produced six
stages of clean-looking CSV, a "CONVERGED" skew loop and a "PASS" gain stage,
and none of it was driven by the dither: the waveform generator had
``ADC_SAMPLE_RATE_HZ = 1.45e9`` hardcoded while the converter ran at 1.30e9.
The dither period was laid out as 145 ADC samples and arrived every 130, so
after the first event the analysis window never contained a pulse again. Nothing
in the pipeline compared the two numbers, so nothing complained.

The checks here are the ones that would have caught it, and they run off the
metadata JSON that sits next to the waveform, so they work against any
generator that writes one.

    python -m calibration_loop.run_calibration check \\
        --waveform-json waveforms/impulse_dither.json --adc-rate 1.3e9
"""

from __future__ import annotations

import json
from fractions import Fraction
from pathlib import Path

import numpy as np


def _get(meta: dict, *names, default=None):
    """Look a key up across the nesting styles the two generators use."""
    pools = [meta, meta.get("config", {}) or {}, meta.get("derived", {}) or {}]
    for pool in pools:
        for name in names:
            if name in pool and pool[name] is not None:
                return pool[name]
    return default


def check_waveform(json_path: str | Path, adc_rate_hz: float) -> list[dict]:
    """Return one result dict per check: name, ok, detail."""
    meta = json.loads(Path(json_path).read_text(encoding="utf-8"))
    results: list[dict] = []

    def add(name, ok, detail):
        results.append({"name": name, "ok": bool(ok), "detail": detail})

    fs_dac = float(_get(meta, "dac_sample_rate_hz", "fs_dac"))
    n_dac = int(_get(meta, "num_samples", "n_dac_points", "n_dac_samples"))
    period_dac = int(_get(meta, "dither_period_dac"))
    edge_dac = int(_get(meta, "dither_edge_dac"))
    top_dac = int(_get(meta, "dither_top_dac"))

    # 1. The rate the waveform was designed for versus the real one.
    assumed = _get(meta, "adc_sample_rate_hz", "fs_adc_hz")
    if assumed is None:
        ratio = _get(meta, "adc_ratio")
        assumed = fs_dac / float(ratio) if ratio else None
    if assumed is None:
        add("waveform records an ADC rate", False,
            "no ADC rate in the metadata - cannot cross-check")
    else:
        assumed = float(assumed)
        err_ppm = 1e6 * (assumed - adc_rate_hz) / adc_rate_hz
        add("waveform ADC rate matches the hardware",
            abs(err_ppm) < 100,
            f"waveform built for {assumed / 1e6:.3f} MS/s, hardware ran at "
            f"{adc_rate_hz / 1e6:.3f} MS/s ({err_ppm:+.0f} ppm)")

    # 2. The dither period has to be a whole number of ADC samples at the
    #    *real* rate, otherwise successive pulses walk off the analysis window.
    ratio = Fraction(int(round(fs_dac)), int(round(adc_rate_hz)))
    num, den = ratio.numerator, ratio.denominator
    period_adc = Fraction(period_dac * den, num)
    add("dither period is a whole number of ADC samples",
        period_adc.denominator == 1,
        f"fs_dac/fs_adc = {ratio} -> {period_dac} DAC samples = {float(period_adc):g} "
        f"ADC samples")

    if assumed is not None and period_adc.denominator == 1:
        r_assumed = Fraction(int(round(fs_dac)), int(round(float(assumed))))
        period_assumed = Fraction(period_dac * r_assumed.denominator, r_assumed.numerator)
        drift = float(period_assumed - period_adc)
        pulse_len = Fraction((2 * edge_dac + top_dac) * den, num)
        add("analysis and hardware agree on where pulses land",
            abs(drift) < 0.5,
            f"analysis steps {float(period_assumed):g} samples per event, pulses arrive "
            f"every {float(period_adc):g}: {drift:+g} samples of drift per event "
            f"against a {float(pulse_len):g}-sample pulse")

    # 3. The loop has to close on an ADC sample boundary.
    loop_adc = Fraction(n_dac * den, num)
    add("DPG loop closes on an ADC sample",
        loop_adc.denominator == 1,
        f"{n_dac} DAC samples = {float(loop_adc):g} ADC samples")

    # 4. The tone must be coherent over the loop.
    tone_hz = float(_get(meta, "actual_tone_hz", "main_tone_hz", default=0.0))
    cycles = tone_hz * n_dac / fs_dac if fs_dac else 0.0
    add("tone is coherent over the loop",
        abs(cycles - round(cycles)) < 1e-6,
        f"{tone_hz / 1e6:.4f} MHz -> {cycles:.4f} cycles per loop")

    # 5. The tone phase must spread across the events inside one capture,
    #    or it survives the averaging and lands in the estimates.
    if loop_adc.denominator == 1 and period_adc.denominator == 1:
        step = (cycles * float(period_adc) / float(loop_adc)) % 1.0
        k = np.arange(15)
        coherence = float(abs(np.mean(np.exp(2j * np.pi * k * step))))
        add("tone phase spreads across dither events",
            coherence < 0.3,
            f"{step:.4f} cycles per event, coherence {coherence:.3f} over 15 events "
            f"(want < 0.3)")

    return results


def suggest_tone(
    fs_dac: float, adc_rate_hz: float, n_dac: int, period_dac: int,
    target_hz: float, count: int = 5,
) -> list[tuple[int, float, float, float]]:
    """Tone bins near ``target_hz`` that satisfy coherence and phase spread."""
    ratio = Fraction(int(round(fs_dac)), int(round(adc_rate_hz)))
    loop_adc = Fraction(n_dac * ratio.denominator, ratio.numerator)
    period_adc = Fraction(period_dac * ratio.denominator, ratio.numerator)
    if loop_adc.denominator != 1 or period_adc.denominator != 1:
        return []

    target_bin = target_hz * n_dac / fs_dac
    k = np.arange(15)
    out = []
    for b in range(max(1, int(target_bin) - 200), int(target_bin) + 200):
        step = (b * float(period_adc) / float(loop_adc)) % 1.0
        coh = float(abs(np.mean(np.exp(2j * np.pi * k * step))))
        if coh < 0.1:
            out.append((abs(b - target_bin), b, b * fs_dac / n_dac, step, coh))
    out.sort()
    return [(b, f, s, c) for _, b, f, s, c in out[:count]]
