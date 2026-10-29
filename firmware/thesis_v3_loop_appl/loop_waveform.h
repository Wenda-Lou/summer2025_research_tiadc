#ifndef LOOP_WAVEFORM_H
#define LOOP_WAVEFORM_H

/* Compatibility helper names only: no stored waveform or polarity table. */
#include <string.h>
#include "adc_calibration_loop.h"

static inline void loop_waveform_config(unsigned index, adc_cal_loop_config_t *cfg)
{
    /* Hardware sample clock only. No waveform information enters calibration. */
    (void)index;
    memset(cfg, 0, sizeof(*cfg));
    cfg->fs_dac = 1300e6;
    cfg->adc_ratio = 1;
}

static inline void loop_waveform_options(unsigned index, adc_cal_loop_options_t *opt)
{
    (void)index;
    opt->reference_free = 1;
    opt->tone_free = 1;
    {
        opt->cancel_signal = 0;
        opt->require_tone = 0;
        opt->reject_clipping = 1;
        opt->gain_observable = ADC_CAL_LOOP_GAIN_DITHER_MAG;
        opt->max_skew_samples = 0.5;
    }
}

#endif /* LOOP_WAVEFORM_H */
