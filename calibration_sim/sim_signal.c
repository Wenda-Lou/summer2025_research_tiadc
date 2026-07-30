#include "sim_signal.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double clamp_double(double value, double minimum, double maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int16_t quantize_adc_code(double value, bool clipping)
{
    if (clipping) {
        value = clamp_double(value, (double)SIM_ADC_MIN_CODE, (double)SIM_ADC_MAX_CODE);
    }
    if (value < (double)SIM_ADC_MIN_CODE) value = (double)SIM_ADC_MIN_CODE;
    if (value > (double)SIM_ADC_MAX_CODE) value = (double)SIM_ADC_MAX_CODE;
    return (int16_t)lrint(value);
}

static uint32_t lcg_next(uint32_t *state)
{
    *state = (*state * 1664525U) + 1013904223U;
    return *state;
}

static double uniform_01(uint32_t *state)
{
    return ((double)(lcg_next(state) >> 8U) + 0.5) / 16777216.0;
}

static double gaussian(uint32_t *state)
{
    const double u1 = fmax(uniform_01(state), 1.0e-12);
    const double u2 = uniform_01(state);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static double dbc_to_linear(double dbc)
{
    return pow(10.0, dbc / 20.0);
}

static double dither_value(const sim_signal_config_t *config, uint32_t index)
{
    const uint32_t period = config->dither_period_samples;
    const uint32_t width = config->dither_width_samples;
    const uint32_t phase = period > 0U ? index % period : 0U;

    if (!config->enable_dither || period == 0U || width == 0U) return 0.0;
    if (phase < width) return config->dither_amplitude_codes;
    if (phase >= period / 2U && phase < (period / 2U) + width) {
        return -config->dither_amplitude_codes;
    }
    return 0.0;
}

static double reference_value_at(const sim_signal_config_t *config, double adc_index)
{
    const double sample_rate = config->adc_sample_rate_hz;
    const double t = adc_index / sample_rate;
    double value = config->reference_dc_codes;
    double tone_hz = config->tone_frequency_hz;

    if (config->coherent_tone) {
        const double cycles = round(config->tone_frequency_hz *
                                    (double)SIM_ADC_CHANNEL_SAMPLES /
                                    config->adc_sample_rate_hz);
        tone_hz = cycles * config->adc_sample_rate_hz / (double)SIM_ADC_CHANNEL_SAMPLES;
    }

    value += config->tone_amplitude_codes * sin(2.0 * M_PI * tone_hz * t);
    if (config->enable_harmonics) {
        value += config->tone_amplitude_codes *
                 dbc_to_linear(config->second_harmonic_dbc) *
                 sin(4.0 * M_PI * tone_hz * t);
        value += config->tone_amplitude_codes *
                 dbc_to_linear(config->third_harmonic_dbc) *
                 sin(6.0 * M_PI * tone_hz * t);
    }
    if (config->spur_amplitude_codes > 0.0 && config->spur_frequency_hz > 0.0) {
        value += config->spur_amplitude_codes *
                 sin(2.0 * M_PI * config->spur_frequency_hz * t);
    }
    value += dither_value(config, (uint32_t)floor(adc_index));
    return value;
}

void sim_signal_default_config(sim_signal_config_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->adc_sample_rate_hz = SIM_DEFAULT_ADC_SAMPLE_RATE_HZ;
    config->dac_sample_rate_hz = SIM_DEFAULT_DAC_SAMPLE_RATE_HZ;
    config->tone_frequency_hz = SIM_DEFAULT_TONE_HZ;
    config->tone_amplitude_codes = 1800.0;
    config->reference_dc_codes = 0.0;
    config->gain_a = 1.0;
    config->gain_b = 1.0;
    config->offset_a_codes = 0.0;
    config->offset_b_codes = 0.0;
    config->delay_a_samples = 0.0;
    config->delay_b_samples = 0.0;
    config->noise_stddev_codes = 0.0;
    config->second_harmonic_dbc = -90.0;
    config->third_harmonic_dbc = -95.0;
    config->spur_frequency_hz = 0.0;
    config->spur_amplitude_codes = 0.0;
    config->dither_amplitude_codes = 35.0;
    config->dither_period_samples = 64U;
    config->dither_width_samples = 4U;
    config->enable_dither = true;
    config->enable_noise = false;
    config->enable_clipping = true;
    config->enable_harmonics = false;
    config->coherent_tone = true;
    config->random_seed = 1U;
    config->bad_frame_mode = SIM_BAD_FRAME_NONE;
    config->bad_frame_period = 0U;
}

int sim_signal_configure_scenario(const char *scenario, sim_signal_config_t *config)
{
    if (scenario == NULL || config == NULL) return -1;
    sim_signal_default_config(config);

    if (strcmp(scenario, "nominal") == 0) {
        config->offset_a_codes = 4.0;
        config->offset_b_codes = 4.5;
        config->gain_a = 1.01;
        config->gain_b = 1.01;
        config->noise_stddev_codes = 1.2;
        config->enable_noise = true;
    } else if (strcmp(scenario, "timing_shift") == 0) {
        config->integer_shift_per_frame = 5U;
        config->delay_a_samples = 0.20;
        config->delay_b_samples = 0.20;
    } else if (strcmp(scenario, "noisy") == 0) {
        config->offset_a_codes = 12.0;
        config->gain_a = 1.0;
        config->noise_stddev_codes = 12.0;
        config->enable_noise = true;
        config->bad_frame_mode = SIM_BAD_FRAME_LOW_CORRELATION;
        config->bad_frame_period = 9U;
    } else if (strcmp(scenario, "bad_reference") == 0) {
        config->tone_frequency_hz = SIM_DEFAULT_TONE_HZ * 1.23;
    } else if (strcmp(scenario, "offset_positive") == 0) {
        config->offset_a_codes = 20.0;
        config->offset_b_codes = 20.0;
    } else if (strcmp(scenario, "offset_negative") == 0) {
        config->offset_a_codes = -20.0;
        config->offset_b_codes = -20.0;
    } else if (strcmp(scenario, "gain_low") == 0) {
        config->gain_a = 0.80;
        config->gain_b = 0.80;
    } else if (strcmp(scenario, "gain_high") == 0) {
        config->gain_a = 1.20;
        config->gain_b = 1.20;
    } else if (strcmp(scenario, "gain_saturation") == 0) {
        config->gain_a = 0.05;
        config->gain_b = 0.05;
    } else if (strcmp(scenario, "skew_positive") == 0) {
        config->delay_b_samples = 0.18;
    } else if (strcmp(scenario, "skew_negative") == 0) {
        config->delay_b_samples = -0.18;
    } else if (strcmp(scenario, "skew_outside_range") == 0) {
        config->delay_b_samples = 1.25;
    } else if (strcmp(scenario, "insufficient_dither") == 0) {
        config->dither_period_samples = 2000U;
    } else if (strcmp(scenario, "clipped_input") == 0) {
        config->tone_amplitude_codes = 9000.0;
        config->enable_clipping = true;
    } else if (strcmp(scenario, "performance_noise") == 0) {
        config->noise_stddev_codes = 80.0;
        config->enable_noise = true;
    } else if (strcmp(scenario, "performance_harmonic") == 0) {
        config->enable_harmonics = true;
        config->second_harmonic_dbc = -30.0;
    } else if (strcmp(scenario, "performance_spur") == 0) {
        config->spur_frequency_hz = 130000000.0;
        config->spur_amplitude_codes = 240.0;
    } else if (strcmp(scenario, "invalidation") == 0) {
        config->offset_a_codes = 5.0;
        config->gain_a = 1.02;
    } else {
        return -2;
    }
    return 0;
}

void sim_signal_init(sim_signal_state_t *state, const sim_signal_config_t *config)
{
    if (state == NULL || config == NULL) return;
    memset(state, 0, sizeof(*state));
    state->config = *config;
    state->rng_state = config->random_seed == 0U ? 1U : config->random_seed;
    sim_signal_generate_reference(state);
}

void sim_signal_generate_reference(sim_signal_state_t *state)
{
    const double ratio = state->config.dac_sample_rate_hz /
                         state->config.adc_sample_rate_hz;
    size_t dac_count = (size_t)llround((double)SIM_ADC_CHANNEL_SAMPLES * ratio);

    if (dac_count == 0U || dac_count > SIM_REFERENCE_CAPACITY) {
        dac_count = SIM_REFERENCE_CAPACITY;
    }
    state->reference_dac_count = dac_count;

    for (size_t i = 0U; i < SIM_ADC_CHANNEL_SAMPLES; ++i) {
        state->reference_adc[i] = reference_value_at(&state->config, (double)i);
        state->reference_adc_i16[i] =
            quantize_adc_code(state->reference_adc[i], state->config.enable_clipping);
    }

    for (size_t i = 0U; i < dac_count; ++i) {
        const double adc_index = (double)i / ratio;
        state->reference_dac_i16[i] =
            quantize_adc_code(reference_value_at(&state->config, adc_index),
                              state->config.enable_clipping);
    }
}

double sim_signal_interpolate_circular(const double *samples, size_t count, double position)
{
    double wrapped;
    size_t i0;
    size_t i1;
    double fraction;

    if (samples == NULL || count == 0U) return 0.0;
    wrapped = fmod(position, (double)count);
    if (wrapped < 0.0) wrapped += (double)count;
    i0 = (size_t)floor(wrapped);
    i1 = i0 + 1U;
    if (i1 >= count) i1 = 0U;
    fraction = wrapped - (double)i0;
    return samples[i0] + fraction * (samples[i1] - samples[i0]);
}

int sim_signal_generate_frame(
    sim_signal_state_t *state,
    int16_t *channel_a,
    int16_t *channel_b,
    size_t sample_count,
    const char **rejection_reason)
{
    const uint32_t frame = state != NULL ? state->frame_index : 0U;
    const bool inject_bad = state != NULL &&
        state->config.bad_frame_period > 0U &&
        frame > 0U &&
        (frame % state->config.bad_frame_period) == 0U;
    double frame_gain_scale;
    double frame_offset;
    uint32_t shift;

    if (rejection_reason != NULL) *rejection_reason = "none";
    if (state == NULL || channel_a == NULL || channel_b == NULL ||
        sample_count > SIM_ADC_CHANNEL_SAMPLES) {
        if (rejection_reason != NULL) *rejection_reason = "invalid frame request";
        return -1;
    }

    if (inject_bad && state->config.bad_frame_mode == SIM_BAD_FRAME_DMA_FAILURE) {
        if (rejection_reason != NULL) *rejection_reason = "injected DMA failure";
        ++state->frame_index;
        return -2;
    }

    frame_gain_scale = 1.0 + state->config.gain_drift_per_frame * (double)frame;
    frame_offset = state->config.offset_drift_per_frame * (double)frame;
    shift = state->config.integer_shift_per_frame * frame;

    for (size_t i = 0U; i < sample_count; ++i) {
        double index = (double)i + (double)shift;
        double ref_a;
        double ref_b;
        double a;
        double b;

        if (inject_bad && state->config.bad_frame_mode == SIM_BAD_FRAME_LOW_CORRELATION) {
            index += (double)(SIM_ADC_CHANNEL_SAMPLES / 3U);
        }
        if (inject_bad && state->config.bad_frame_mode == SIM_BAD_FRAME_EXCESSIVE_DELAY) {
            index += 64.0;
        }

        ref_a = sim_signal_interpolate_circular(
            state->reference_adc, SIM_ADC_CHANNEL_SAMPLES,
            index - state->config.delay_a_samples);
        ref_b = sim_signal_interpolate_circular(
            state->reference_adc, SIM_ADC_CHANNEL_SAMPLES,
            index - state->config.delay_b_samples);

        if (inject_bad && state->config.bad_frame_mode == SIM_BAD_FRAME_WRONG_TONE) {
            ref_a = reference_value_at(&state->config, index * 1.07);
            ref_b = ref_a;
        }
        if (inject_bad && state->config.bad_frame_mode == SIM_BAD_FRAME_CORRUPT_DITHER) {
            ref_a -= dither_value(&state->config, (uint32_t)floor(index));
            ref_b -= dither_value(&state->config, (uint32_t)floor(index));
        }

        a = state->config.gain_a * frame_gain_scale * ref_a +
            state->config.offset_a_codes + frame_offset;
        b = state->config.gain_b * frame_gain_scale * ref_b +
            state->config.offset_b_codes + frame_offset;

        if (state->config.enable_noise) {
            a += gaussian(&state->rng_state) * state->config.noise_stddev_codes;
            b += gaussian(&state->rng_state) * state->config.noise_stddev_codes;
        }
        if (inject_bad && state->config.bad_frame_mode == SIM_BAD_FRAME_ALL_ZERO) {
            a = 0.0;
            b = 0.0;
        }
        if (inject_bad && state->config.bad_frame_mode == SIM_BAD_FRAME_CLIPPED) {
            a = (i & 1U) != 0U ? 20000.0 : -20000.0;
            b = a;
        }

        channel_a[i] = quantize_adc_code(a, state->config.enable_clipping);
        channel_b[i] = quantize_adc_code(b, state->config.enable_clipping);
    }

    ++state->frame_index;
    return 0;
}

const char *sim_bad_frame_mode_name(sim_bad_frame_mode_t mode)
{
    switch (mode) {
    case SIM_BAD_FRAME_NONE: return "none";
    case SIM_BAD_FRAME_DMA_FAILURE: return "dma_failure";
    case SIM_BAD_FRAME_ALL_ZERO: return "all_zero";
    case SIM_BAD_FRAME_CLIPPED: return "clipped";
    case SIM_BAD_FRAME_LOW_CORRELATION: return "low_correlation";
    case SIM_BAD_FRAME_CORRUPT_DITHER: return "corrupt_dither";
    case SIM_BAD_FRAME_EXCESSIVE_DELAY: return "excessive_delay";
    case SIM_BAD_FRAME_WRONG_TONE: return "wrong_tone";
    default: return "unknown";
    }
}
