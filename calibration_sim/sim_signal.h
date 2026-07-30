#ifndef SIM_SIGNAL_H
#define SIM_SIGNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sim_config.h"

typedef enum {
    SIM_BAD_FRAME_NONE = 0,
    SIM_BAD_FRAME_DMA_FAILURE,
    SIM_BAD_FRAME_ALL_ZERO,
    SIM_BAD_FRAME_CLIPPED,
    SIM_BAD_FRAME_LOW_CORRELATION,
    SIM_BAD_FRAME_CORRUPT_DITHER,
    SIM_BAD_FRAME_EXCESSIVE_DELAY,
    SIM_BAD_FRAME_WRONG_TONE
} sim_bad_frame_mode_t;

typedef struct {
    double adc_sample_rate_hz;
    double dac_sample_rate_hz;
    double tone_frequency_hz;
    double tone_amplitude_codes;
    double reference_dc_codes;
    double gain_a;
    double gain_b;
    double offset_a_codes;
    double offset_b_codes;
    double delay_a_samples;
    double delay_b_samples;
    double noise_stddev_codes;
    double second_harmonic_dbc;
    double third_harmonic_dbc;
    double spur_frequency_hz;
    double spur_amplitude_codes;
    double dither_amplitude_codes;
    uint32_t dither_period_samples;
    uint32_t dither_width_samples;
    uint32_t integer_shift_per_frame;
    double offset_drift_per_frame;
    double gain_drift_per_frame;
    bool enable_dither;
    bool enable_noise;
    bool enable_clipping;
    bool enable_harmonics;
    bool coherent_tone;
    uint32_t random_seed;
    sim_bad_frame_mode_t bad_frame_mode;
    uint32_t bad_frame_period;
} sim_signal_config_t;

typedef struct {
    sim_signal_config_t config;
    uint32_t frame_index;
    uint32_t rng_state;
    double reference_adc[SIM_ADC_CHANNEL_SAMPLES];
    int16_t reference_adc_i16[SIM_ADC_CHANNEL_SAMPLES];
    int16_t reference_dac_i16[SIM_REFERENCE_CAPACITY];
    size_t reference_dac_count;
} sim_signal_state_t;

void sim_signal_default_config(sim_signal_config_t *config);
int sim_signal_configure_scenario(
    const char *scenario,
    sim_signal_config_t *config);
void sim_signal_init(sim_signal_state_t *state, const sim_signal_config_t *config);
void sim_signal_generate_reference(sim_signal_state_t *state);
int sim_signal_generate_frame(
    sim_signal_state_t *state,
    int16_t *channel_a,
    int16_t *channel_b,
    size_t sample_count,
    const char **rejection_reason);
double sim_signal_interpolate_circular(
    const double *samples,
    size_t count,
    double position);
const char *sim_bad_frame_mode_name(sim_bad_frame_mode_t mode);

#endif /* SIM_SIGNAL_H */
