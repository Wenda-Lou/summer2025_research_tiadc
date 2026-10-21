#ifndef LOOP_WAVEFORM_H
#define LOOP_WAVEFORM_H

/*
 * The DPG vectors the on-board loop knows.  "adc -cal loop wave N" selects
 * which one is assumed to be loaded (default 0, reset at boot).
 *
 * The selection must describe the vector actually on the DPG.  A wrong pulse
 * shape shows up as a lower alignment margin and noisy estimates; a wrong
 * sig_cycles does NOT collapse the margin (the dither slots still line up) but
 * fits the tone at the wrong frequency, so gain and skew are silently wrong.
 * For another vector, add a row with "config" from its .json.
 *
 * All rows: fs_dac 2600e6, adc_ratio 2, n_dac_points 16640, amp_dbfs -1.5,
 * dither_period_dac 260, dither_position_dac 96, seed 20260725 -- the host
 * DitherConfig() defaults, which adc_cal_loop_default_config() carries.
 *
 *   0  waveforms/tone_dither/w06adc_e04_t04_amp5190_tone_m1p5            199.375 MHz
 *   1  waveforms/tone_dither/w06adc_e04_t04_amp5190_tone_m1p5_f399p375   399.375 MHz
 */

#include "adc_calibration_loop.h"

typedef struct {
    const char *name;
    int sig_cycles;
    int dither_edge_dac;
    int dither_top_dac;
    double dither_scale_lsb;
    const char *tone_mhz;   /* for printing only: xil_printf has no %f */
} loop_waveform_t;

static const loop_waveform_t k_loop_waveforms[] = {
    {"w06adc_e04_t04_amp5190_tone_m1p5",          1276, 4, 4, 5190.0, "199.375"},
    {"w06adc_e04_t04_amp5190_tone_m1p5_f399p375", 2556, 4, 4, 5190.0, "399.375"},
};

#define LOOP_WAVEFORM_COUNT (sizeof(k_loop_waveforms) / sizeof(k_loop_waveforms[0]))

static inline void loop_waveform_config(unsigned index, adc_cal_loop_config_t *cfg)
{
    const loop_waveform_t *w = &k_loop_waveforms[index < LOOP_WAVEFORM_COUNT ? index : 0U];

    adc_cal_loop_default_config(cfg);
    cfg->sig_cycles = w->sig_cycles;
    cfg->dither_edge_dac = w->dither_edge_dac;
    cfg->dither_top_dac = w->dither_top_dac;
    cfg->dither_scale_lsb = w->dither_scale_lsb;
}

#endif /* LOOP_WAVEFORM_H */
