"""
Impulse-dither background calibration of gain, offset and timing skew for the
ZCU102 + AD9164 + AD9695 time-interleaved ADC test bench.

Entry points::

    from calibration_loop import DitherConfig, write_dac_files
    from calibration_loop import CalibrationLoop, CalibrationState, BenchModel

See ``README.md`` in this directory for the theory and the bring-up order.
"""

from .dither import (
    DitherConfig,
    DAC_FULL_SCALE,
    adc_templates,
    build_dac_waveform,
    dither_only_loop,
    polarity_sequence,
    pulse,
    pulse_derivative,
    reference_loop,
    write_dac_files,
)
from .estimator import (
    BlockEstimate,
    CalibrationState,
    ChannelEstimate,
    align_to_loop,
    deframe,
    estimate_block,
    estimate_channel,
    fit_tone,
    interleave,
    prepare_capture,
    unpack_words,
)
from .metrics import analyse, channel_difference_dbc, mismatch_spurs, spectrum
from .loop import CalibrationLoop, LoopOptions
from .simulate import BenchModel

__all__ = [
    "DitherConfig",
    "DAC_FULL_SCALE",
    "adc_templates",
    "build_dac_waveform",
    "dither_only_loop",
    "polarity_sequence",
    "pulse",
    "pulse_derivative",
    "reference_loop",
    "write_dac_files",
    "BlockEstimate",
    "CalibrationState",
    "ChannelEstimate",
    "align_to_loop",
    "deframe",
    "estimate_block",
    "estimate_channel",
    "fit_tone",
    "interleave",
    "prepare_capture",
    "unpack_words",
    "analyse",
    "channel_difference_dbc",
    "mismatch_spurs",
    "spectrum",
    "CalibrationLoop",
    "LoopOptions",
    "BenchModel",
]
