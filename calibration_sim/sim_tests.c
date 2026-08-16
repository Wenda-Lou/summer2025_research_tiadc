#include "sim_tests.h"

#include "adc_frame.h"
#include "adc_calibration_pipeline.h"
#include "adc_calibration_dither.h"
#include "adc_calibration_skew.h"
#include "adc_calibration_performance.h"
#include "calibration.h"
#include "calibration_pending.h"
#include "reference_buffer.h"
#include "sim_assert.h"
#include "sim_config.h"
#include "sim_dma.h"
#include "sim_platform.h"
#include "sim_signal.h"
#include "timing_alignment.h"

#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Recorded replay captures predate the clock fix and must retain their own
 * coordinate system. This value is never a normal simulator default. */
#define SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ 1450000000.0

typedef struct {
    double sndr_db;
    double sfdr_db;
    double thd_db;
    double enob;
    double signal_hz;
    double worst_spur_hz;
} sim_perf_metrics_t;

typedef struct {
    const char *name;
    bool expect_success;
} sim_scenario_spec_t;

static const sim_scenario_spec_t k_scenarios[] = {
    {"nominal", true},
    {"timing_shift", true},
    {"noisy", true},
    {"bad_reference", false},
    {"offset_positive", true},
    {"offset_negative", true},
    {"gain_low", true},
    {"gain_high", true},
    {"gain_saturation", false},
    {"skew_positive", true},
    {"skew_negative", true},
    {"skew_outside_range", false},
    {"insufficient_dither", false},
    {"clipped_input", false},
    {"performance_noise", true},
    {"performance_harmonic", true},
    {"performance_spur", true},
    {"invalidation", true}
};

static const sim_scenario_spec_t k_pipeline_scenarios[] = {
    {"nominal", true},
    {"timing_shift", true},
    {"noisy", true},
    {"gain_saturation", false},
    {"skew_positive", true},
    {"skew_negative", true},
    {"performance_distortion", true},
    {"timing_failure", false},
    {"offset_nonconvergence", false},
    {"gain_verification_failure", false},
    {"insufficient_dither", false},
    {"performance_invalid", true},
    {"invalidation_after_complete", true},
    {"standalone_sequence", true}
};

void sim_assert_context_init(sim_assert_context_t *ctx, FILE *summary, FILE *unit_csv)
{
    if (ctx == NULL) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->summary = summary;
    ctx->unit_csv = unit_csv;
    if (unit_csv != NULL) {
        fputs("test,assertion,status,actual,expected,tolerance,file,line\n", unit_csv);
    }
}

static void sim_assert_record(
    sim_assert_context_t *ctx,
    const char *assertion,
    const char *status,
    double actual,
    double expected,
    double tolerance,
    const char *file,
    int line)
{
    if (ctx != NULL && ctx->unit_csv != NULL) {
        fprintf(ctx->unit_csv, "unit,%s,%s,%.12g,%.12g,%.12g,%s,%d\n",
                assertion, status, actual, expected, tolerance, file, line);
    }
}

static void sim_record_known_gap(
    sim_assert_context_t *ctx,
    const char *description,
    const char *file,
    int line)
{
    if (ctx != NULL && ctx->summary != NULL) {
        fprintf(ctx->summary, "KNOWN GAP %s:%d %s\n", file, line, description);
    }
    if (ctx != NULL && ctx->unit_csv != NULL) {
        fprintf(ctx->unit_csv, "unit,%s,KNOWN_GAP,nan,nan,nan,%s,%d\n",
                description, file, line);
    }
}

int sim_assert_true(
    sim_assert_context_t *ctx,
    int condition,
    const char *expression,
    const char *file,
    int line)
{
    if (condition) {
        if (ctx != NULL) ++ctx->passed;
        sim_assert_record(ctx, expression, "PASS", 1.0, 1.0, 0.0, file, line);
        return 1;
    }
    if (ctx != NULL) {
        ++ctx->failed;
        if (ctx->summary != NULL) {
            fprintf(ctx->summary, "FAIL %s:%d %s\n", file, line, expression);
        }
    }
    sim_assert_record(ctx, expression, "FAIL", 0.0, 1.0, 0.0, file, line);
    return 0;
}

int sim_assert_eq_int(
    sim_assert_context_t *ctx,
    long actual,
    long expected,
    const char *actual_expr,
    const char *expected_expr,
    const char *file,
    int line)
{
    char assertion[256];
    (void)snprintf(assertion, sizeof(assertion), "%s == %s", actual_expr, expected_expr);
    if (actual == expected) {
        if (ctx != NULL) ++ctx->passed;
        sim_assert_record(ctx, assertion, "PASS", (double)actual, (double)expected, 0.0, file, line);
        return 1;
    }
    if (ctx != NULL) {
        ++ctx->failed;
        if (ctx->summary != NULL) {
            fprintf(ctx->summary, "FAIL %s:%d %s actual=%ld expected=%ld\n",
                    file, line, assertion, actual, expected);
        }
    }
    sim_assert_record(ctx, assertion, "FAIL", (double)actual, (double)expected, 0.0, file, line);
    return 0;
}

int sim_assert_near(
    sim_assert_context_t *ctx,
    double actual,
    double expected,
    double tolerance,
    const char *actual_expr,
    const char *expected_expr,
    const char *file,
    int line)
{
    char assertion[256];
    (void)snprintf(assertion, sizeof(assertion), "%s ~= %s", actual_expr, expected_expr);
    if ((actual == actual) && fabs(actual - expected) <= tolerance) {
        if (ctx != NULL) ++ctx->passed;
        sim_assert_record(ctx, assertion, "PASS", actual, expected, tolerance, file, line);
        return 1;
    }
    if (ctx != NULL) {
        ++ctx->failed;
        if (ctx->summary != NULL) {
            fprintf(ctx->summary, "FAIL %s:%d %s actual=%.12g expected=%.12g tol=%.12g\n",
                    file, line, assertion, actual, expected, tolerance);
        }
    }
    sim_assert_record(ctx, assertion, "FAIL", actual, expected, tolerance, file, line);
    return 0;
}

static int make_dir_if_needed(const char *path)
{
    if (path == NULL) return -1;
#ifdef _WIN32
    if (_mkdir(path) == 0) return 0;
#else
    if (mkdir(path, 0777) == 0) return 0;
#endif
    return errno == EEXIST ? 0 : -1;
}

static int open_outputs(
    const char *output_dir,
    FILE **summary,
    FILE **unit_csv,
    FILE **iterations_csv,
    FILE **performance_csv,
    FILE **stress_csv)
{
    char path[512];

    if (make_dir_if_needed(output_dir) != 0) return -1;
    (void)snprintf(path, sizeof(path), "%s/test_summary.txt", output_dir);
    *summary = fopen(path, "w");
    (void)snprintf(path, sizeof(path), "%s/unit_test_results.csv", output_dir);
    *unit_csv = fopen(path, "w");
    (void)snprintf(path, sizeof(path), "%s/calibration_iterations.csv", output_dir);
    *iterations_csv = fopen(path, "w");
    (void)snprintf(path, sizeof(path), "%s/performance.csv", output_dir);
    *performance_csv = fopen(path, "w");
    (void)snprintf(path, sizeof(path), "%s/stress_summary.csv", output_dir);
    *stress_csv = fopen(path, "w");
    if (*summary == NULL || *unit_csv == NULL ||
        *iterations_csv == NULL || *performance_csv == NULL ||
        *stress_csv == NULL) {
        return -2;
    }
    fputs("scenario,stage,stage_iteration,frame_index,accepted,rejection_reason,"
          "estimated_lag,correlation,offset_correction,gain_correction,"
          "estimated_skew_samples,status\n", *iterations_csv);
    fputs("scenario,stage,stage_iteration,frame_index,accepted,rejection_reason,"
          "estimated_lag,correlation,offset_correction,gain_correction,"
          "estimated_skew_samples,SNDR,SFDR,THD,ENOB,valid\n", *performance_csv);
    fputs("seed,scenario,pass_fail,measured_value,expected_range,failure_reason\n",
          *stress_csv);
    return 0;
}

static uint32_t test_lcg_next(uint32_t *state)
{
    *state = (*state * 1664525U) + 1013904223U;
    return *state;
}

static double test_uniform_01(uint32_t *state)
{
    return ((double)(test_lcg_next(state) >> 8U) + 0.5) / 16777216.0;
}

static double test_gaussian(uint32_t *state)
{
    const double u1 = fmax(test_uniform_01(state), 1.0e-12);
    const double u2 = test_uniform_01(state);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static void make_dither_template(
    double *samples,
    size_t count,
    size_t period,
    size_t half_width,
    double amplitude,
    int polarity_mode)
{
    if (samples == NULL || period == 0U || half_width == 0U) return;
    memset(samples, 0, count * sizeof(samples[0]));
    for (size_t center = period / 2U; center < count; center += period) {
        const double polarity = polarity_mode > 0 ? 1.0 :
            polarity_mode < 0 ? -1.0 :
            (((center / period) & 1U) == 0U ? 1.0 : -1.0);
        const size_t first = center > half_width ? center - half_width : 0U;
        size_t last = center + half_width;
        if (last >= count) last = count - 1U;
        for (size_t i = first; i <= last; ++i) {
            const double distance = fabs((double)i - (double)center);
            const double shape = 1.0 - distance / ((double)half_width + 1.0);
            if (shape > 0.0) samples[i] = polarity * amplitude * shape;
        }
    }
}

static void make_dither_residual(
    const double *template_samples,
    double *residual,
    size_t count,
    double gain,
    double skew_samples,
    double noise_stddev,
    uint32_t *rng)
{
    if (template_samples == NULL || residual == NULL) return;
    for (size_t i = 0U; i < count; ++i) {
        const double previous = i > 0U ? template_samples[i - 1U] :
            template_samples[i];
        const double next = i + 1U < count ? template_samples[i + 1U] :
            template_samples[i];
        const double derivative = 0.5 * (next - previous);
        const double noise = noise_stddev > 0.0 && rng != NULL ?
            noise_stddev * test_gaussian(rng) : 0.0;
        residual[i] = gain * (template_samples[i] + skew_samples * derivative) +
            noise;
    }
}

static void make_perf_tone(
    double *samples,
    size_t count,
    double sample_rate_hz,
    double frequency_hz,
    double amplitude,
    double dc,
    double noise_stddev,
    double second_harmonic_ratio,
    double third_harmonic_ratio,
    double spur_hz,
    double spur_amplitude,
    bool clipping,
    uint32_t *rng)
{
    if (samples == NULL || sample_rate_hz <= 0.0) return;
    for (size_t i = 0U; i < count; ++i) {
        const double t = (double)i / sample_rate_hz;
        double value = dc + amplitude * sin(2.0 * M_PI * frequency_hz * t);
        value += amplitude * second_harmonic_ratio *
            sin(4.0 * M_PI * frequency_hz * t);
        value += amplitude * third_harmonic_ratio *
            sin(6.0 * M_PI * frequency_hz * t);
        if (spur_hz > 0.0 && spur_amplitude > 0.0) {
            value += spur_amplitude * sin(2.0 * M_PI * spur_hz * t);
        }
        if (noise_stddev > 0.0 && rng != NULL) {
            value += noise_stddev * test_gaussian(rng);
        }
        if (clipping) {
            if (value > (double)SIM_ADC_MAX_CODE) value = SIM_ADC_MAX_CODE;
            if (value < (double)SIM_ADC_MIN_CODE) value = SIM_ADC_MIN_CODE;
        }
        samples[i] = value;
    }
}

static void close_file(FILE *file)
{
    if (file != NULL) (void)fclose(file);
}

static void fill_sine_i16(int16_t *samples, size_t count, double amplitude, double cycles, double phase)
{
    for (size_t i = 0U; i < count; ++i) {
        samples[i] = (int16_t)lrint(amplitude *
            sin((2.0 * M_PI * cycles * (double)i / (double)count) + phase));
    }
}

static void circular_shift(const int16_t *input, int16_t *output, size_t count, int32_t lag)
{
    for (size_t i = 0U; i < count; ++i) {
        int64_t source = (int64_t)i - (int64_t)lag;
        source %= (int64_t)count;
        if (source < 0) source += (int64_t)count;
        output[i] = input[(size_t)source];
    }
}

static int unit_dma_round_trip(sim_assert_context_t *ctx)
{
    uint8_t raw[SIM_DMA_BYTES];
    int16_t a[SIM_ADC_CHANNEL_SAMPLES];
    int16_t b[SIM_ADC_CHANNEL_SAMPLES];
    int16_t ra[SIM_ADC_CHANNEL_SAMPLES];
    int16_t rb[SIM_ADC_CHANNEL_SAMPLES];
    size_t count = 0U;

    for (size_t i = 0U; i < SIM_ADC_CHANNEL_SAMPLES; ++i) {
        a[i] = (int16_t)((int)i % 4096 - 2048);
        b[i] = (int16_t)(2047 - ((int)i % 4096));
    }
    a[0] = SIM_ADC_MIN_CODE;
    a[1] = -1;
    a[2] = 0;
    a[3] = SIM_ADC_MAX_CODE;
    b[0] = SIM_ADC_MAX_CODE;
    b[1] = 1;
    b[2] = -1;
    b[3] = SIM_ADC_MIN_CODE;

    SIM_ASSERT_EQ_INT(ctx, sim_dma_encode_channels(a, b, SIM_ADC_CHANNEL_SAMPLES, raw, sizeof(raw)), 0);
    SIM_ASSERT_EQ_INT(ctx, raw[0], 0x00);
    SIM_ASSERT_EQ_INT(ctx, raw[1], 0x80);
    SIM_ASSERT_EQ_INT(ctx, raw[6], 0xfc);
    SIM_ASSERT_EQ_INT(ctx, raw[7], 0x7f);
    SIM_ASSERT_EQ_INT(ctx, raw[8], 0xfc);
    SIM_ASSERT_EQ_INT(ctx, raw[9], 0x7f);
    SIM_ASSERT_EQ_INT(ctx, raw[14], 0x00);
    SIM_ASSERT_EQ_INT(ctx, raw[15], 0x80);
    SIM_ASSERT_EQ_INT(ctx, adc_reconstruct_channels(raw, 4095U, ra, SIM_ADC_CHANNEL_SAMPLES, rb, SIM_ADC_CHANNEL_SAMPLES, &count), 0);
    SIM_ASSERT_EQ_INT(ctx, count, SIM_ADC_CHANNEL_SAMPLES);
    for (size_t i = 0U; i < SIM_ADC_CHANNEL_SAMPLES; ++i) {
        if (!SIM_ASSERT_EQ_INT(ctx, ra[i], a[i])) return 0;
        if (!SIM_ASSERT_EQ_INT(ctx, rb[i], b[i])) return 0;
    }
    return 1;
}

static int unit_timing_lag(sim_assert_context_t *ctx)
{
    int16_t reference[128];
    int16_t signal[128];
    timing_alignment_result_t result;

    fill_sine_i16(reference, 128U, 1200.0, 7.0, 0.0);
    circular_shift(reference, signal, 128U, 5);
    SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(reference, signal, 128U, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.lag_samples, 5);
    circular_shift(reference, signal, 128U, -7);
    SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(reference, signal, 128U, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.lag_samples, -7);
    circular_shift(reference, signal, 128U, 63);
    SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(reference, signal, 128U, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.lag_samples, 63);
    return 1;
}

static void make_fractional_signal(const int16_t *reference, int16_t *signal, size_t count, double lag)
{
    static double temp[256];
    for (size_t i = 0U; i < count; ++i) temp[i] = (double)reference[i];
    for (size_t i = 0U; i < count; ++i) {
        signal[i] = (int16_t)lrint(sim_signal_interpolate_circular(temp, count, (double)i - lag));
    }
}

static int unit_fractional_lag(sim_assert_context_t *ctx)
{
    int16_t reference[128];
    int16_t signal[128];
    timing_alignment_result_t result;
    float fractional = 0.0f;

    for (size_t i = 0U; i < 128U; ++i) {
        const double x = (double)i;
        reference[i] = (int16_t)lrint(
            900.0 * sin(2.0 * M_PI * 7.0 * x / 128.0) +
            450.0 * sin(2.0 * M_PI * 13.0 * x / 128.0 + 0.3) +
            180.0 * sin(2.0 * M_PI * 23.0 * x / 128.0 + 0.7));
    }
    make_fractional_signal(reference, signal, 128U, 0.20);
    SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(reference, signal, 128U, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, timing_estimate_fractional_lag(reference, signal, 128U, result.lag_samples, &fractional), 0);
    SIM_ASSERT_NEAR(ctx, (double)result.lag_samples + (double)fractional, 0.20, 0.12);

    make_fractional_signal(reference, signal, 128U, -0.20);
    SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(reference, signal, 128U, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, timing_estimate_fractional_lag(reference, signal, 128U, result.lag_samples, &fractional), 0);
    SIM_ASSERT_NEAR(ctx, (double)result.lag_samples + (double)fractional, -0.20, 0.12);

    make_fractional_signal(reference, signal, 128U, 0.48);
    SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(reference, signal, 128U, &result), 0);
    (void)timing_estimate_fractional_lag(reference, signal, 128U, result.lag_samples, &fractional);
    SIM_ASSERT_TRUE(ctx, fabs((double)fractional) <= 0.5);
    return 1;
}

static int unit_calibration_estimates(sim_assert_context_t *ctx)
{
    calibration_config_t config;
    calibration_state_t state;
    int16_t reference[256];
    int16_t adc[256];

    fill_sine_i16(reference, 256U, 1000.0, 11.0, 0.0);
    for (size_t i = 0U; i < 256U; ++i) {
        adc[i] = (int16_t)lrint(1.20 * (double)reference[i] + 20.0);
    }
    calibration_default_config(&config);
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_OK);
    SIM_ASSERT_EQ_INT(ctx, calibration_analyze_frame(&state, adc, reference, 256U), CALIBRATION_OK);
    SIM_ASSERT_NEAR(ctx, state.metrics.measured_gain, 1.20, 0.02);
    SIM_ASSERT_NEAR(ctx, state.metrics.measured_offset, 20.0, 1.0);
    SIM_ASSERT_TRUE(ctx, state.metrics.offset_error_codes > 0.0f);
    SIM_ASSERT_EQ_INT(ctx, calibration_update(&state), CALIBRATION_OK);
    SIM_ASSERT_TRUE(ctx, state.offset_correction < 0.0f);
    return 1;
}

static int unit_reference_buffer(sim_assert_context_t *ctx)
{
    int16_t reference[64];
    for (size_t i = 0U; i < 64U; ++i) reference[i] = (int16_t)i;
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_begin_with_metadata(
        64U, REFERENCE_FORMAT_DAC_RATE_2X,
        1300000000.0, 2600000000.0), REFERENCE_BUFFER_OK);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_write_chunk(0U, reference, 32U), REFERENCE_BUFFER_OK);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_write_chunk(32U, &reference[32], 32U), REFERENCE_BUFFER_OK);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_finalize(), REFERENCE_BUFFER_OK);
    SIM_ASSERT_TRUE(ctx, reference_buffer_is_ready());
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_length(), 64U);
    SIM_ASSERT_TRUE(ctx, reference_buffer_has_rate_metadata());
    SIM_ASSERT_NEAR(ctx, reference_buffer_adc_sample_rate_hz(),
                    1300000000.0, 1.0);
    SIM_ASSERT_NEAR(ctx, reference_buffer_dac_sample_rate_hz(),
                    2600000000.0, 1.0);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_validate_sample_rates(
        1300000000.0, 2600000000.0, 1.0), REFERENCE_BUFFER_OK);

    /* Regression for the original failure: 1.45-GSPS waveform metadata must
     * not be silently adapted to 1.30-GSPS hardware. */
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_begin_with_metadata(
        64U, REFERENCE_FORMAT_DAC_RATE_2X,
        1450000000.0, 2600000000.0), REFERENCE_BUFFER_OK);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_validate_sample_rates(
        1300000000.0, 2600000000.0, 1.0),
        REFERENCE_BUFFER_ERR_RATE_MISMATCH);

    SIM_ASSERT_EQ_INT(ctx, reference_buffer_begin_with_format(64U, REFERENCE_FORMAT_DAC_RATE_2X), REFERENCE_BUFFER_OK);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_validate_sample_rates(
        1300000000.0, 2600000000.0, 1.0),
        REFERENCE_BUFFER_ERR_RATE_METADATA);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_write_chunk(0U, reference, 32U), REFERENCE_BUFFER_OK);
    SIM_ASSERT_TRUE(ctx, reference_buffer_finalize() != REFERENCE_BUFFER_OK);
    return 1;
}

static int unit_flat_reference_rejection(sim_assert_context_t *ctx)
{
    calibration_config_t config;
    calibration_state_t state;
    int16_t reference[32] = {0};
    int16_t adc[32] = {0};
    calibration_default_config(&config);
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_OK);
    SIM_ASSERT_EQ_INT(ctx, calibration_analyze_frame(&state, adc, reference, 32U), CALIBRATION_ERR_ZERO_REFERENCE_POWER);
    return 1;
}

static int unit_host_invalidation_hooks(sim_assert_context_t *ctx)
{
    sim_signal_config_t config;
    sim_signal_state_t signal;
    uint8_t dma[SIM_DMA_BYTES];
    sim_platform_state_t platform;

    sim_signal_default_config(&config);
    sim_signal_init(&signal, &config);
    sim_platform_init(&platform, &signal, dma, sizeof(dma));
    sim_platform_set_active(&platform);
    sim_platform_mark_performance_available(true);
    calibration_pending_frame_invalidate();
    SIM_ASSERT_TRUE(ctx, !platform.performance_measurement_available);
    sim_platform_mark_performance_available(true);
    calibration_gain_input_frame_invalidate();
    SIM_ASSERT_TRUE(ctx, !platform.performance_measurement_available);
    sim_platform_mark_performance_available(true);
    SIM_ASSERT_EQ_INT(ctx, calibration_set_software_gain_correction(1.01f), 0);
    SIM_ASSERT_TRUE(ctx, !platform.performance_measurement_available);
    return 1;
}

static void sim_set_config_float_field(
    calibration_config_t *config,
    unsigned field,
    float value)
{
    switch (field) {
    case 0U: config->offset_tolerance_codes = value; break;
    case 1U: config->gain_tolerance_ratio = value; break;
    case 2U: config->offset_step = value; break;
    case 3U: config->gain_step = value; break;
    case 4U: config->min_gain_correction = value; break;
    case 5U: config->max_gain_correction = value; break;
    case 6U: config->min_offset_correction = value; break;
    case 7U: config->max_offset_correction = value; break;
    default: break;
    }
}

static int unit_nonfinite_config_rejection(sim_assert_context_t *ctx)
{
    static const char *field_names[] = {
        "offset_tolerance_codes",
        "gain_tolerance_ratio",
        "offset_step",
        "gain_step",
        "min_gain_correction",
        "max_gain_correction",
        "min_offset_correction",
        "max_offset_correction"
    };
    static const struct {
        const char *name;
        float value;
    } nonfinite_values[] = {
        {"nan", NAN},
        {"posinf", INFINITY},
        {"neginf", -INFINITY}
    };
    calibration_config_t config;
    calibration_state_t state;

    for (unsigned field = 0U;
         field < sizeof(field_names) / sizeof(field_names[0]);
         ++field) {
        for (unsigned value_index = 0U;
             value_index < sizeof(nonfinite_values) / sizeof(nonfinite_values[0]);
             ++value_index) {
            calibration_default_config(&config);
            sim_set_config_float_field(
                &config, field, nonfinite_values[value_index].value);
            SIM_ASSERT_EQ_INT(
                ctx,
                calibration_init(&state, &config),
                CALIBRATION_ERR_INVALID_CONFIG);
            if (ctx != NULL && ctx->summary != NULL) {
                fprintf(
                    ctx->summary,
                    "Config nonfinite regression %-27s %-6s : PASS\n",
                    field_names[field],
                    nonfinite_values[value_index].name);
            }
        }
    }
    return 1;
}

static int unit_finite_config_boundaries(sim_assert_context_t *ctx)
{
    calibration_config_t config;
    calibration_state_t state;

    calibration_default_config(&config);
    config.offset_tolerance_codes = 0.0f;
    config.gain_tolerance_ratio = 0.0f;
    config.offset_step = 1.0f;
    config.gain_step = 1.0f;
    config.min_gain_correction = CALIBRATION_GAIN_CORRECTION_MIN;
    config.max_gain_correction = CALIBRATION_GAIN_CORRECTION_MAX;
    config.min_offset_correction = -4096.0f;
    config.max_offset_correction = 4096.0f;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_OK);

    calibration_default_config(&config);
    config.max_offset_iterations = 0U;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_ERR_INVALID_CONFIG);
    calibration_default_config(&config);
    config.max_gain_iterations = 0U;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_ERR_INVALID_CONFIG);
    calibration_default_config(&config);
    config.offset_step = 0.0f;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_ERR_INVALID_CONFIG);
    calibration_default_config(&config);
    config.offset_step = 1.001f;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_ERR_INVALID_CONFIG);
    calibration_default_config(&config);
    config.gain_step = 0.0f;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_ERR_INVALID_CONFIG);
    calibration_default_config(&config);
    config.gain_step = 1.001f;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_ERR_INVALID_CONFIG);
    calibration_default_config(&config);
    config.min_gain_correction = 0.0f;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_ERR_INVALID_CONFIG);
    calibration_default_config(&config);
    config.max_gain_correction = config.min_gain_correction - 0.001f;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_ERR_INVALID_CONFIG);
    calibration_default_config(&config);
    config.max_offset_correction = config.min_offset_correction - 1.0f;
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_ERR_INVALID_CONFIG);
    return 1;
}

static int unit_dither_estimator_direct(sim_assert_context_t *ctx)
{
    double template_samples[256];
    double residual[256];
    adc_cal_dither_config_t config;
    adc_cal_dither_result_t result;
    uint32_t rng = 1234U;

    adc_cal_dither_default_config(&config);
    config.minimum_events = 2U;

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, 0);
    make_dither_residual(template_samples, residual, 256U, 1.25, 0.0, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_analyze(residual, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.valid);
    SIM_ASSERT_TRUE(ctx, result.accepted_events >= 2U);
    SIM_ASSERT_NEAR(ctx, result.normalized_projection, 1.25, 0.02);

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, 1);
    make_dither_residual(template_samples, residual, 256U, 1.0, 0.0, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_analyze(residual, template_samples, 256U, &config, &result), ADC_CAL_DITHER_ERR_POLARITY);

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, -1);
    make_dither_residual(template_samples, residual, 256U, 1.0, 0.0, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_analyze(residual, template_samples, 256U, &config, &result), ADC_CAL_DITHER_ERR_POLARITY);

    make_dither_template(template_samples, 256U, 64U, 5U, 12.0, 0);
    make_dither_residual(template_samples, residual, 256U, 0.8, 0.0, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_analyze(residual, template_samples, 256U, &config, &result), 0);

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, 0);
    template_samples[0] = 10.0;
    template_samples[1] = 8.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_find_events(template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.rejected_events > 0U);

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, 0);
    template_samples[255] = -10.0;
    template_samples[254] = -8.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_find_events(template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.rejected_events > 0U);

    make_dither_template(template_samples, 256U, 6U, 4U, 10.0, 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_find_events(template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.detected_events > 0U);

    make_dither_template(template_samples, 256U, 200U, 4U, 10.0, 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_find_events(template_samples, 256U, &config, &result), ADC_CAL_DITHER_ERR_TOO_FEW_EVENTS);

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, 0);
    make_dither_residual(template_samples, residual, 256U, -0.2, 0.0, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_analyze(residual, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.quality < 0.0);

    memset(residual, 0, sizeof(residual));
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_analyze(residual, template_samples, 256U, &config, &result), ADC_CAL_DITHER_ERR_NO_ENERGY);

    memset(template_samples, 0, sizeof(template_samples));
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_find_events(template_samples, 256U, &config, &result), ADC_CAL_DITHER_ERR_NO_ENERGY);

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, 0);
    make_dither_residual(template_samples, residual, 256U, 1.0, 0.0, 0.2, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_analyze(residual, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.quality > 0.95);
    return 1;
}

static int load_i16_text_waveform(
    const char *path,
    int16_t *samples,
    size_t capacity,
    size_t *sample_count)
{
    FILE *file;
    size_t count = 0U;
    long value;
    if (sample_count != NULL) *sample_count = 0U;
    if (path == NULL || samples == NULL || sample_count == NULL) return -1;
    file = fopen(path, "r");
    if (file == NULL) return -2;
    while (fscanf(file, "%ld", &value) == 1) {
        if (count >= capacity || value < INT16_MIN || value > INT16_MAX) {
            fclose(file);
            return -3;
        }
        samples[count++] = (int16_t)value;
    }
    if (!feof(file)) {
        fclose(file);
        return -4;
    }
    fclose(file);
    *sample_count = count;
    return count > 0U ? 0 : -5;
}

static int load_u8_csv_capture(
    const char *path,
    uint8_t *bytes,
    size_t capacity,
    size_t *byte_count)
{
    FILE *file;
    char header[64];
    size_t count = 0U;
    long value;
    if (byte_count != NULL) *byte_count = 0U;
    if (path == NULL || bytes == NULL || byte_count == NULL) return -1;
    file = fopen(path, "r");
    if (file == NULL) return -2;
    if (fgets(header, sizeof(header), file) == NULL ||
        strncmp(header, "byte", 4U) != 0) {
        fclose(file);
        return -3;
    }
    while (fscanf(file, "%ld", &value) == 1) {
        if (count >= capacity || value < 0L || value > 255L) {
            fclose(file);
            return -4;
        }
        bytes[count++] = (uint8_t)value;
    }
    if (!feof(file)) {
        fclose(file);
        return -5;
    }
    fclose(file);
    *byte_count = count;
    return count > 0U ? 0 : -6;
}

static int sim_double_isfinite(double value)
{
    return value == value && value <= DBL_MAX && value >= -DBL_MAX;
}

static int remove_known_tone(
    const int16_t *samples,
    size_t sample_count,
    double frequency_hz,
    double sample_rate_hz,
    double *residual)
{
    double normal[3][4] = {{0.0}};
    double coefficients[3];
    if (samples == NULL || residual == NULL || sample_count < 3U ||
        !sim_double_isfinite(frequency_hz) || frequency_hz <= 0.0 ||
        !sim_double_isfinite(sample_rate_hz) || sample_rate_hz <= 0.0) return -1;
    for (size_t i = 0U; i < sample_count; ++i) {
        const double phase = 2.0 * M_PI * frequency_hz *
            (double)i / sample_rate_hz;
        const double basis[3] = {cos(phase), sin(phase), 1.0};
        for (size_t row = 0U; row < 3U; ++row) {
            for (size_t col = 0U; col < 3U; ++col) {
                normal[row][col] += basis[row] * basis[col];
            }
            normal[row][3] += basis[row] * (double)samples[i];
        }
    }
    for (size_t pivot = 0U; pivot < 3U; ++pivot) {
        size_t best = pivot;
        for (size_t row = pivot + 1U; row < 3U; ++row) {
            if (fabs(normal[row][pivot]) > fabs(normal[best][pivot])) {
                best = row;
            }
        }
        if (fabs(normal[best][pivot]) <= DBL_EPSILON) return -2;
        if (best != pivot) {
            for (size_t col = pivot; col < 4U; ++col) {
                const double swap = normal[pivot][col];
                normal[pivot][col] = normal[best][col];
                normal[best][col] = swap;
            }
        }
        {
            const double divisor = normal[pivot][pivot];
            for (size_t col = pivot; col < 4U; ++col) {
                normal[pivot][col] /= divisor;
            }
        }
        for (size_t row = 0U; row < 3U; ++row) {
            const double factor = normal[row][pivot];
            if (row == pivot) continue;
            for (size_t col = pivot; col < 4U; ++col) {
                normal[row][col] -= factor * normal[pivot][col];
            }
        }
    }
    for (size_t i = 0U; i < 3U; ++i) coefficients[i] = normal[i][3];
    for (size_t i = 0U; i < sample_count; ++i) {
        const double phase = 2.0 * M_PI * frequency_hz *
            (double)i / sample_rate_hz;
        residual[i] = (double)samples[i] -
            coefficients[0] * cos(phase) -
            coefficients[1] * sin(phase) - coefficients[2];
    }
    return 0;
}

static int make_scaled_dac_capture(
    const int16_t *raw_dac,
    size_t raw_sample_count,
    int16_t *capture,
    size_t sample_count,
    double dac_samples_per_adc_sample,
    double lag_samples,
    double scale)
{
    static int16_t unscaled[SIM_ADC_CHANNEL_SAMPLES];
    if (sample_count > SIM_ADC_CHANNEL_SAMPLES) return -1;
    if (adc_cal_dither_resample_dac_reference(
            raw_dac, raw_sample_count, dac_samples_per_adc_sample,
            -lag_samples * dac_samples_per_adc_sample,
            unscaled, sample_count) != 0) return -2;
    for (size_t i = 0U; i < sample_count; ++i) {
        double value = scale * (double)unscaled[i];
        if (value > (double)INT16_MAX) value = (double)INT16_MAX;
        if (value < (double)INT16_MIN) value = (double)INT16_MIN;
        capture[i] = (int16_t)lrint(value);
    }
    return 0;
}

static int unit_dither_detection_validation(sim_assert_context_t *ctx)
{
    adc_cal_dither_validation_config_t config;
    adc_cal_dither_validation_input_t input;
    adc_cal_dither_validation_result_t validation;
    adc_cal_dither_peak_config_t peak_config;
    adc_cal_dither_peak_result_t peaks;
    adc_cal_dither_event_summary_t events;
    adc_cal_dither_periodic_difference_t periodic_difference;
    double scores[512];
    double template_samples[1016];
    const double canonical_wrapped_origin = fmod(1010.633698, 1016.0);
    const double derived_lag = 5.366302;
    double frame1_existing_n0 = fmod(-70.832, 1016.0);
    double frame1_dither_n0 = fmod(-220.672, 1016.0);
    double frame3_n0_a = fmod(-311.284, 1016.0);
    double frame3_n0_b = fmod(133.715, 1016.0);

    if (frame1_existing_n0 < 0.0) frame1_existing_n0 += 1016.0;
    if (frame1_dither_n0 < 0.0) frame1_dither_n0 += 1016.0;
    if (frame3_n0_a < 0.0) frame3_n0_a += 1016.0;
    if (frame3_n0_b < 0.0) frame3_n0_b += 1016.0;

    adc_cal_dither_validation_default_config(&config);
    config.minimum_complete_events = 3U;
    memset(&input, 0, sizeof(input));
    input.selected_channel_valid = 1;
    input.channel_a_available = 1;
    input.channel_b_available = 1;
    input.channel_a_valid = 1;
    input.channel_b_valid = 1;
    input.existing_comparison_required = 1;
    input.channel_disagreement_samples = 0.10;
    input.existing_disagreement_samples = 0.18;
    input.complete_event_count = 5U;
    input.event_indices_valid = 1;
    input.numerical_values_finite = 1;
    input.independent_peak_ratio = 1.50;

    /* Strong unique peak. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &input, &config, &validation), 0);
    SIM_ASSERT_TRUE(ctx, validation.structural_valid);
    SIM_ASSERT_EQ_INT(ctx, validation.confidence,
                      ADC_CAL_DITHER_CONFIDENCE_STRONG);
    SIM_ASSERT_EQ_INT(ctx, validation.recommendation,
                      ADC_CAL_DITHER_RECOMMEND_ACCEPT);

    /* Weak peak confidence does not invalidate consistent detection. */
    input.independent_peak_ratio = 1.135748;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &input, &config, &validation), 0);
    SIM_ASSERT_TRUE(ctx, validation.structural_valid);
    SIM_ASSERT_EQ_INT(ctx, validation.confidence,
                      ADC_CAL_DITHER_CONFIDENCE_WEAK);
    SIM_ASSERT_EQ_INT(ctx, validation.recommendation,
                      ADC_CAL_DITHER_RECOMMEND_ACCEPT_WITH_WARNING);

    /* A jointly constrained candidate may be smaller than an isolated peak;
     * peak uniqueness remains advisory after structural checks pass. */
    input.independent_peak_ratio = 0.70;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &input, &config, &validation), 0);
    SIM_ASSERT_TRUE(ctx, validation.structural_valid);
    SIM_ASSERT_EQ_INT(ctx, validation.recommendation,
                      ADC_CAL_DITHER_RECOMMEND_ACCEPT_WITH_WARNING);
    SIM_ASSERT_EQ_INT(ctx, validation.reason,
                      ADC_CAL_DITHER_VALIDATION_PEAK_BELOW_WEAK);
    input.independent_peak_ratio = 1.135748;

    /* Independent consistency failures are structural failures. */
    input.channel_disagreement_samples = 1.01;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &input, &config, &validation), 0);
    SIM_ASSERT_TRUE(ctx, !validation.structural_valid);
    SIM_ASSERT_EQ_INT(ctx, validation.confidence,
                      ADC_CAL_DITHER_CONFIDENCE_WEAK);
    SIM_ASSERT_EQ_INT(ctx, validation.reason,
                      ADC_CAL_DITHER_VALIDATION_CHANNEL_DISAGREEMENT);
    input.channel_disagreement_samples = 0.10;
    input.existing_disagreement_samples = 1.01;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &input, &config, &validation), 0);
    SIM_ASSERT_TRUE(ctx, !validation.structural_valid);
    SIM_ASSERT_EQ_INT(ctx, validation.confidence,
                      ADC_CAL_DITHER_CONFIDENCE_WEAK);
    SIM_ASSERT_EQ_INT(ctx, validation.reason,
                      ADC_CAL_DITHER_VALIDATION_EXISTING_DISAGREEMENT);
    input.existing_disagreement_samples = 0.18;

    /* A rejected existing estimator is not a structural dither reference. */
    input.existing_comparison_required = 0;
    input.existing_disagreement_samples = NAN;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &input, &config, &validation), 0);
    SIM_ASSERT_TRUE(ctx, validation.structural_valid);
    SIM_ASSERT_EQ_INT(ctx, validation.reason,
                      ADC_CAL_DITHER_VALIDATION_PEAK_BELOW_STRONG);
    input.existing_comparison_required = 1;
    input.existing_disagreement_samples = 0.18;

    /* Requiring a joint pair reports the selection failure explicitly. */
    input.joint_pair_required = 1;
    input.joint_pair_valid = 0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &input, &config, &validation), 0);
    SIM_ASSERT_TRUE(ctx, !validation.structural_valid);
    SIM_ASSERT_EQ_INT(ctx, validation.reason,
                      ADC_CAL_DITHER_VALIDATION_NO_CONSISTENT_PAIR);
    input.joint_pair_valid = 1;

    /* A periodic-family duplicate is not the independent second peak. */
    memset(scores, 0, sizeof(scores));
    scores[20] = 1.0;
    scores[163] = 0.95;
    scores[80] = 0.50;
    adc_cal_dither_peak_default_config(&peak_config);
    peak_config.periodic_exclusion_width_samples = 3.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_independent_peaks(
        scores, 512U, 142.79, &peak_config, &peaks), 0);
    SIM_ASSERT_EQ_INT(ctx, peaks.raw_second_index, 163U);
    SIM_ASSERT_EQ_INT(ctx, peaks.independent_second_index, 80U);
    SIM_ASSERT_NEAR(ctx, peaks.raw_peak_ratio, 1.0 / 0.95, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, peaks.independent_peak_ratio, 2.0, 1.0e-12);

    /* Invalid spacing preserves the legacy local-guard second peak. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_independent_peaks(
        scores, 512U, NAN, &peak_config, &peaks), 0);
    SIM_ASSERT_EQ_INT(ctx, peaks.raw_second_index, 163U);
    SIM_ASSERT_EQ_INT(ctx, peaks.independent_second_index, 163U);

    /* Periodic exclusion checks the alternate frame lift as well. */
    memset(scores, 0, sizeof(scores));
    scores[20] = 1.0;
    scores[389] = 0.95;
    scores[80] = 0.50;
    peak_config.periodic_exclusion_width_samples = 1.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_independent_peaks(
        scores, 512U, 142.793, &peak_config, &peaks), 0);
    SIM_ASSERT_EQ_INT(ctx, peaks.raw_second_index, 389U);
    SIM_ASSERT_EQ_INT(ctx, peaks.independent_second_index, 80U);

    /* Timing origins are compared modulo the measured impulse spacing. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        70.832, 220.672, 142.791, 0.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, -1);
    SIM_ASSERT_NEAR(ctx, periodic_difference.signed_difference_samples,
                    -7.049, 1.0e-9);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    7.049, 1.0e-9);

    /* Lag validation must stay in lag coordinates.  Event-origin metadata may
     * use another sign/wrap convention and is not part of this comparison. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_lags(
        463.983703, 463.983703, 140.757300, 1016.0,
        &periodic_difference), 0);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    0.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_lags(
        463.983703, 463.983703 + 140.757300, 140.757300, 1016.0,
        &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, -1);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    0.0, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_lags(
        463.983703, 463.983703 + 47.373036, 140.757300, 1016.0,
        &periodic_difference), 0);
    SIM_ASSERT_TRUE(ctx,
        periodic_difference.absolute_difference_samples > 1.0);

    SIM_ASSERT_TRUE(ctx, strcmp(adc_cal_dither_validation_reason_name(
        ADC_CAL_DITHER_VALIDATION_EXISTING_TIMING), "NONE") != 0);
    SIM_ASSERT_TRUE(ctx, strcmp(adc_cal_dither_validation_reason_name(
        ADC_CAL_DITHER_VALIDATION_TONE_FIT), "NONE") != 0);
    SIM_ASSERT_TRUE(ctx, strcmp(adc_cal_dither_validation_reason_name(
        ADC_CAL_DITHER_VALIDATION_WINDOW), "NONE") != 0);
    SIM_ASSERT_TRUE(ctx, adc_cal_dither_window_is_valid(5U, 1U, 1, 1));
    SIM_ASSERT_TRUE(ctx, !adc_cal_dither_window_is_valid(0U, 1U, 1, 1));
    SIM_ASSERT_TRUE(ctx, !adc_cal_dither_window_is_valid(5U, 1U, 0, 1));
    SIM_ASSERT_TRUE(ctx, !adc_cal_dither_window_is_valid(5U, 1U, 1, 0));
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        frame1_existing_n0, frame1_dither_n0, 142.791, 1016.0,
        &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.frame_offset, 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, 1);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    7.049, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        5.366301, 5.259356, 142.791, 0.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, 0);
    SIM_ASSERT_NEAR(ctx, periodic_difference.signed_difference_samples,
                    0.106945, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        311.284, -133.715, 142.793, 0.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, 3);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    16.620, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        70.832, 213.623, 142.791, 0.0, &periodic_difference), 0);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    0.0, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        -117.095, 311.284, 142.793, 0.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, -3);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    0.0, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        1.0, 2.0, NAN, 0.0, &periodic_difference),
        ADC_CAL_DITHER_ERR_NUMERICAL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        1.0, 2.0, 0.0, 0.0, &periodic_difference),
        ADC_CAL_DITHER_ERR_NUMERICAL);
    SIM_ASSERT_TRUE(ctx, !periodic_difference.valid);
    SIM_ASSERT_TRUE(ctx, isnan(periodic_difference.signed_difference_samples));
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        1.0, 2.0, -142.793, 0.0, &periodic_difference),
        ADC_CAL_DITHER_ERR_NUMERICAL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        1.0, 2.0, DBL_EPSILON, 0.0, &periodic_difference),
        ADC_CAL_DITHER_ERR_NUMERICAL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        1.0, 2.0, INFINITY, 0.0, &periodic_difference),
        ADC_CAL_DITHER_ERR_NUMERICAL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        NAN, 2.0, 142.793, 1016.0, &periodic_difference),
        ADC_CAL_DITHER_ERR_NUMERICAL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        71.3965, 0.0, 142.793, 0.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, 1);
    SIM_ASSERT_NEAR(ctx, periodic_difference.signed_difference_samples,
                    -71.3965, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        -71.3965, 0.0, 142.793, 0.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, -1);
    SIM_ASSERT_NEAR(ctx, periodic_difference.signed_difference_samples,
                    71.3965, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        1.0, 2.0, 142.793, -1.0, &periodic_difference),
        ADC_CAL_DITHER_ERR_NUMERICAL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        1.0, 2.0, 142.793, 0.0, NULL), ADC_CAL_DITHER_ERR_NULL);

    /* Canonical frame origins expose periodic equivalence hidden by signed lag. */
    SIM_ASSERT_NEAR(ctx, frame3_n0_a, 704.716, 1.0e-9);
    SIM_ASSERT_NEAR(ctx, frame3_n0_b, 133.715, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        frame3_n0_a, frame3_n0_b, 142.793, 1016.0,
        &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.frame_offset, 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, 4);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    0.171, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        0.1, 1015.9, 142.793, 1016.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.frame_offset, 1);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, 0);
    SIM_ASSERT_NEAR(ctx, periodic_difference.signed_difference_samples,
                    0.2, 1.0e-9);

    /* Wrapped n0 is diagnostic phase; event scanning still finds earlier events. */
    memset(template_samples, 0, sizeof(template_samples));
    for (size_t center = 80U; center < 900U; center += 143U) {
        template_samples[center - 1U] = 0.5;
        template_samples[center] = 1.0;
        template_samples[center + 1U] = 0.5;
    }
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_summarize_events(
        template_samples, 1016U, 0U, 1016U, 0.25, &events), 0);
    SIM_ASSERT_TRUE(ctx, canonical_wrapped_origin > 1000.0);
    SIM_ASSERT_NEAR(ctx, fmod(canonical_wrapped_origin + derived_lag,
                              1016.0), 0.0, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, events.complete_count, 6U);
    SIM_ASSERT_NEAR(ctx, events.first_complete_center, 80.0, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, events.last_complete_center, 795.0, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, events.spacing_samples, 143.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        events.last_complete_center, events.first_complete_center,
        events.spacing_samples, 0.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.frame_offset, 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset,
                      (int32_t)events.complete_count - 1);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    0.0, 1.0e-12);
    SIM_ASSERT_NEAR(ctx,
        fmod(events.first_complete_center + derived_lag +
             canonical_wrapped_origin, 1016.0),
        events.first_complete_center, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        fmod(events.last_complete_center + 300.0, 1016.0),
        fmod(events.first_complete_center + 300.0, 1016.0),
        events.spacing_samples, 1016.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.frame_offset, 1);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset,
                      (int32_t)events.complete_count - 1);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    0.0, 1.0e-12);

    /* Ordered event spans use the known forward frame lift. This avoids an
     * ambiguous complementary offset when frame length is an exact multiple
     * of event spacing. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        19.0, 400.0, 127.0, 1016.0, &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.frame_offset, 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, -3);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        19.0 + 1016.0, 400.0, 127.0, 0.0,
        &periodic_difference), 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.frame_offset, 0);
    SIM_ASSERT_EQ_INT(ctx, periodic_difference.event_offset, 5);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    0.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        canonical_wrapped_origin,
        canonical_wrapped_origin - events.spacing_samples,
        events.spacing_samples, 1016.0, &periodic_difference), 0);
    SIM_ASSERT_NEAR(ctx, periodic_difference.absolute_difference_samples,
                    0.0, 1.0e-9);

    /* Non-finite metrics and too few complete events are invalid. */
    input.independent_peak_ratio = NAN;
    input.complete_event_count = 5U;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &input, &config, &validation), 0);
    SIM_ASSERT_TRUE(ctx, !validation.structural_valid);
    SIM_ASSERT_EQ_INT(ctx, validation.reason,
                      ADC_CAL_DITHER_VALIDATION_NUMERICAL);
    input.independent_peak_ratio = 1.50;
    input.complete_event_count = 2U;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &input, &config, &validation), 0);
    SIM_ASSERT_TRUE(ctx, !validation.structural_valid);
    SIM_ASSERT_EQ_INT(ctx, validation.reason,
                      ADC_CAL_DITHER_VALIDATION_TOO_FEW_EVENTS);
    return 1;
}

static adc_cal_dither_peak_candidate_t make_joint_candidate(
    double origin_samples,
    double peak,
    double margin)
{
    adc_cal_dither_peak_candidate_t candidate;
    memset(&candidate, 0, sizeof(candidate));
    candidate.valid = 1;
    candidate.wrapped_origin_samples = origin_samples;
    candidate.lag_samples = -origin_samples;
    candidate.peak = peak;
    candidate.absolute_peak = fabs(peak);
    candidate.raw_second_peak = 0.5 * peak;
    candidate.independent_second_peak = 0.5 * peak;
    candidate.raw_peak_ratio = 2.0;
    candidate.independent_peak_ratio = 2.0;
    candidate.margin = margin;
    candidate.confidence = ADC_CAL_DITHER_CONFIDENCE_STRONG;
    return candidate;
}

static int unit_dither_joint_alignment(sim_assert_context_t *ctx)
{
    adc_cal_dither_peak_candidate_t channel_a[4];
    adc_cal_dither_peak_candidate_t channel_b[4];
    adc_cal_dither_peak_candidate_t extracted[4];
    adc_cal_dither_joint_config_t joint_config;
    adc_cal_dither_joint_result_t joint;
    adc_cal_dither_peak_config_t peak_config;
    adc_cal_dither_validation_config_t validation_config;
    adc_cal_dither_periodic_difference_t difference;
    adc_cal_dither_coordinate_mapping_t mapping;
    adc_cal_dither_peak_candidate_t refined_candidate;
    double scores[64];
    double capture_samples[4] = {10.0, 20.0, -5.0, 8.0};
    double refined_tone[4] = {8.0, 17.0, -6.0, 8.5};
    double dither_residual[4];
    double adc_position;
    double event_phase;
    double expected_origin;
    double detected_origin;
    size_t extracted_count = 0U;

    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_build_tone_removed_residual(
        capture_samples, refined_tone, 4U, dither_residual), 0);
    SIM_ASSERT_NEAR(ctx, dither_residual[0], 2.0, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, dither_residual[1], 3.0, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, dither_residual[2], 1.0, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, dither_residual[3], -0.5, 1.0e-12);

    /* Existing and dither lags share the full-reference origin. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_lag_to_wrapped_origin(
        13.007470, 1016.0, &expected_origin), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_lag_to_wrapped_origin(
        13.007470, 1016.0, &detected_origin), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        expected_origin, detected_origin, 142.791120, 1016.0,
        &difference), 0);
    SIM_ASSERT_NEAR(ctx, difference.absolute_difference_samples,
                    0.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_lag_to_wrapped_origin(
        353.548673, 1016.0, &detected_origin), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        expected_origin, detected_origin, 142.791120, 1016.0,
        &difference), 0);
    SIM_ASSERT_NEAR(ctx, difference.absolute_difference_samples,
                    38.496803, 1.0e-6);

    /* A nonzero reference-event phase is derived, never hard-coded. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_reference_event_phase(
        181.291120, 0.0, 142.791120, &event_phase), 0);
    SIM_ASSERT_NEAR(ctx, event_phase, 38.5, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_lag_to_wrapped_origin(
        13.0 + event_phase, 1016.0, &expected_origin), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_lag_to_wrapped_origin(
        51.5, 1016.0, &detected_origin), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        expected_origin, detected_origin, 142.791120, 1016.0,
        &difference), 0);
    SIM_ASSERT_NEAR(ctx, difference.absolute_difference_samples,
                    0.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_reference_event_phase(
        214.175561, 0.0, 142.791120, &event_phase), 0);
    SIM_ASSERT_NEAR(ctx, event_phase, 71.384441, 1.0e-9);

    /* Historical 2.6/1.45 replay coordinates still exercise the general
     * rate-matching primitive; production no longer applies this adaptation. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_dac_position_to_adc_position(
        181.25, 2600000000.0 / SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ,
        1.0, 0.25, &adc_position), 0);
    SIM_ASSERT_NEAR(ctx, adc_position,
        180.0 / (2600000000.0 / SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ),
        1.0e-12);

    /* Fixed-window coordinates do not change the full-frame lag origin. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_map_reference_position(
        214.175561, 108.0, 353.548673, 1016.0, &mapping), 0);
    SIM_ASSERT_NEAR(ctx, mapping.window_relative_position_samples,
                    106.175561, 1.0e-9);
    SIM_ASSERT_NEAR(ctx, mapping.capture_unwrapped_position_samples,
                    567.724234, 1.0e-9);
    SIM_ASSERT_NEAR(ctx, mapping.capture_wrapped_position_samples,
                    567.724234, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, mapping.capture_frame_wraps, 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_map_reference_position(
        785.329304, 108.0, 353.548673, 1016.0, &mapping), 0);
    SIM_ASSERT_NEAR(ctx, mapping.capture_unwrapped_position_samples,
                    1138.877977, 1.0e-9);
    SIM_ASSERT_NEAR(ctx, mapping.capture_wrapped_position_samples,
                    122.877977, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, mapping.capture_frame_wraps, 1);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_map_reference_position(
        10.0, 108.0, -30.0, 1016.0, &mapping), 0);
    SIM_ASSERT_NEAR(ctx, mapping.capture_wrapped_position_samples,
                    996.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, mapping.capture_frame_wraps, -1);

    /* Independent maxima disagree, but a second-ranked candidate forms the
     * jointly consistent pair and agrees with the existing timing family. */
    channel_a[0] = make_joint_candidate(100.0, 1.0, 5.0);
    channel_a[1] = make_joint_candidate(250.0, 0.8, 4.0);
    channel_b[0] = make_joint_candidate(150.0, 1.0, 5.0);
    channel_b[1] = make_joint_candidate(300.2, 0.8, 4.0);
    adc_cal_dither_joint_default_config(&joint_config);
    joint_config.use_expected_origin = 1;
    joint_config.expected_origin_samples = 100.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        channel_a, 2U, channel_b, 2U, 100.0, 1016.0,
        &joint_config, &joint), 0);
    SIM_ASSERT_EQ_INT(ctx, joint.channel_a_candidate, 0U);
    SIM_ASSERT_EQ_INT(ctx, joint.channel_b_candidate, 1U);
    SIM_ASSERT_TRUE(ctx, joint.existing_consistent);
    SIM_ASSERT_NEAR(ctx,
        joint.channel_difference.absolute_difference_samples,
        0.2, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, joint.channel_difference.event_offset, -2);
    SIM_ASSERT_NEAR(ctx, joint.consensus_origin_samples,
        100.0888888888889, 1.0e-9);
    SIM_ASSERT_NEAR(ctx, joint.consensus_lag_samples,
        -100.0888888888889, 1.0e-9);

    channel_b[0] = make_joint_candidate(150.0, 1.0, 5.0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        channel_a, 1U, channel_b, 1U, 100.0, 1016.0,
        &joint_config, &joint), ADC_CAL_DITHER_ERR_NO_EVENTS);
    SIM_ASSERT_TRUE(ctx, !joint.valid);

    /* Polarity is deliberately unconstrained because reconstruction applies
     * no channel sign normalization. Both observed combinations are usable. */
    channel_a[0] = make_joint_candidate(100.0, 1.0, 5.0);
    channel_b[0] = make_joint_candidate(100.2, 0.9, 5.0);
    joint_config.use_expected_origin = 0;
    joint_config.polarity_policy = ADC_CAL_DITHER_POLARITY_UNCONSTRAINED;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        channel_a, 1U, channel_b, 1U, 100.0, 1016.0,
        &joint_config, &joint), 0);
    SIM_ASSERT_TRUE(ctx, joint.same_polarity);
    channel_b[0].peak = -channel_b[0].peak;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        channel_a, 1U, channel_b, 1U, 100.0, 1016.0,
        &joint_config, &joint), 0);
    SIM_ASSERT_TRUE(ctx, !joint.same_polarity);
    joint_config.polarity_policy = ADC_CAL_DITHER_POLARITY_SAME;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        channel_a, 1U, channel_b, 1U, 100.0, 1016.0,
        &joint_config, &joint), ADC_CAL_DITHER_ERR_NO_EVENTS);
    joint_config.polarity_policy = ADC_CAL_DITHER_POLARITY_INVERTED;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        channel_a, 1U, channel_b, 1U, 100.0, 1016.0,
        &joint_config, &joint), 0);

    /* Candidate extraction retains sub-sample lag after periodic reduction. */
    memset(scores, 0, sizeof(scores));
    scores[19] = 0.8;
    scores[20] = 1.0;
    scores[21] = 0.9;
    scores[50] = 0.5;
    adc_cal_dither_peak_default_config(&peak_config);
    peak_config.local_exclusion_samples = 2U;
    peak_config.maximum_candidates = 4U;
    adc_cal_dither_validation_default_config(&validation_config);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_find_peak_candidates(
        scores, 64U, 16.0, &peak_config, &validation_config,
        extracted, 4U, &extracted_count), 0);
    SIM_ASSERT_TRUE(ctx, extracted_count >= 2U);
    SIM_ASSERT_EQ_INT(ctx, extracted[0].index, 20U);
    SIM_ASSERT_NEAR(ctx, extracted[0].fractional_offset_samples,
                    1.0 / 6.0, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, extracted[0].lag_samples,
                     20.0 + 1.0 / 6.0, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, extracted[0].global_strongest_peak, 1.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, extracted[0].global_strongest_index, 20U);
    SIM_ASSERT_NEAR(ctx, extracted[1].global_strongest_peak, 1.0, 1.0e-12);
    memset(&refined_candidate, 0, sizeof(refined_candidate));
    refined_candidate.valid = 1;
    refined_candidate.coarse_index = 20U;
    refined_candidate.coarse_fractional_offset_samples = 0.1;
    refined_candidate.coarse_lag_samples = 20.1;
    refined_candidate.index = 20U;
    refined_candidate.fractional_offset_samples = 0.1;
    refined_candidate.lag_samples = 20.1;
    refined_candidate.wrapped_origin_samples = 43.9;
    refined_candidate.peak = 0.8;
    memset(scores, 0, sizeof(scores));
    scores[19] = 0.7;
    scores[20] = 1.0;
    scores[21] = 0.9;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_refine_candidate_lags(
        scores, 64U, 2U, &refined_candidate, 1U), 0);
    SIM_ASSERT_EQ_INT(ctx, refined_candidate.coarse_index, 20U);
    SIM_ASSERT_NEAR(ctx, refined_candidate.coarse_lag_samples, 20.1, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, refined_candidate.lag_samples, 20.25, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, refined_candidate.peak, 0.8, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        extracted[0].wrapped_origin_samples,
        extracted[0].wrapped_origin_samples + 16.0,
        16.0, 64.0, &difference), 0);
    SIM_ASSERT_NEAR(ctx, difference.absolute_difference_samples,
                    0.0, 1.0e-12);

    /* Previously passing hardware residuals remain inside tolerance. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        5.366301, 5.259356, 142.791120, 0.0, &difference), 0);
    SIM_ASSERT_NEAR(ctx, difference.absolute_difference_samples,
                    0.106945, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        5.187736, 5.366301, 142.791120, 0.0, &difference), 0);
    SIM_ASSERT_NEAR(ctx, difference.absolute_difference_samples,
                     0.178565, 1.0e-9);

    /* Existing timing is a hard feasibility constraint.  The strongest pair
     * is rejected and a lower-ranked fractional pair is selected. */
    channel_a[0] = make_joint_candidate(120.0, 1.0, 5.0);
    channel_b[0] = make_joint_candidate(121.0, 1.0, 5.0);
    channel_a[1] = make_joint_candidate(300.25, 0.8, 4.0);
    channel_b[1] = make_joint_candidate(500.40, 0.75, 4.0);
    channel_a[1].fractional_offset_samples = -0.25;
    channel_a[1].lag_samples = -300.25;
    channel_b[1].fractional_offset_samples = -0.40;
    channel_b[1].lag_samples = -500.40;
    adc_cal_dither_joint_default_config(&joint_config);
    joint_config.use_expected_origin = 1;
    joint_config.expected_origin_samples = 100.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        channel_a, 2U, channel_b, 2U, 100.0, 1016.0,
        &joint_config, &joint), 0);
    SIM_ASSERT_EQ_INT(ctx, joint.channel_a_candidate, 1U);
    SIM_ASSERT_EQ_INT(ctx, joint.channel_b_candidate, 1U);
    SIM_ASSERT_NEAR(ctx, channel_a[joint.channel_a_candidate].lag_samples,
        -300.25, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, channel_b[joint.channel_b_candidate].lag_samples,
        -500.40, 1.0e-12);
    /* Deterministic tie/rank behavior is stable on a repeated search. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        channel_a, 2U, channel_b, 2U, 100.0, 1016.0,
        &joint_config, &joint), 0);
    SIM_ASSERT_EQ_INT(ctx, joint.channel_a_candidate, 1U);
    SIM_ASSERT_EQ_INT(ctx, joint.channel_b_candidate, 1U);

    /* Latest hardware geometry: A/B are within one sample, but neither is
     * within the one-sample existing family.  Preserve the valid A/B pair and
     * expose the independent existing-timing failure separately. */
    channel_a[0] = make_joint_candidate(
        fmod(-22.0 + 1016.0, 1016.0), 0.288871, 3.0);
    channel_b[0] = make_joint_candidate(
        fmod(-23.0 + 1016.0, 1016.0), 0.312686, 3.0);
    joint_config.expected_origin_samples =
        fmod(-36.927791 + 1016.0, 1016.0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        channel_a[0].wrapped_origin_samples,
        joint_config.expected_origin_samples,
        142.791120, 1016.0, &difference), 0);
    SIM_ASSERT_NEAR(ctx, difference.absolute_difference_samples,
        1.534369, 1.0e-5);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        channel_b[0].wrapped_origin_samples,
        joint_config.expected_origin_samples,
        142.791120, 1016.0, &difference), 0);
    SIM_ASSERT_NEAR(ctx, difference.absolute_difference_samples,
        2.534369, 1.0e-5);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        channel_a, 1U, channel_b, 1U, 142.791120, 1016.0,
        &joint_config, &joint), 0);
    SIM_ASSERT_TRUE(ctx, joint.valid);
    SIM_ASSERT_TRUE(ctx, !joint.existing_consistent);
    SIM_ASSERT_NEAR(ctx,
        joint.channel_difference.absolute_difference_samples,
        1.0, 1.0e-9);
    SIM_ASSERT_NEAR(ctx, joint.channel_a_existing_residual_samples,
        1.534369, 1.0e-5);
    SIM_ASSERT_NEAR(ctx, joint.channel_b_existing_residual_samples,
        2.534369, 1.0e-5);
    return 1;
}

static double test_dither_correlation_lag(
    const double *template_samples,
    const double *capture_samples,
    size_t count,
    adc_cal_dither_correlation_mode_t mode)
{
    double scores[256];
    adc_cal_dither_peak_candidate_t candidate;
    adc_cal_dither_peak_config_t peak_config;
    adc_cal_dither_validation_config_t confidence_config;
    size_t candidate_count = 0U;

    if (count > 256U || adc_cal_dither_compute_circular_scores(
            template_samples, capture_samples, count, mode, scores) != 0) {
        return NAN;
    }
    adc_cal_dither_peak_default_config(&peak_config);
    peak_config.maximum_candidates = 1U;
    adc_cal_dither_validation_default_config(&confidence_config);
    if (adc_cal_dither_find_peak_candidates(
            scores, count, 0.0, &peak_config, &confidence_config,
            &candidate, 1U, &candidate_count) != 0 ||
        candidate_count != 1U) {
        return NAN;
    }
    return candidate.lag_samples;
}

static double test_dither_timing_lag(
    const double *template_samples,
    const double *capture_samples,
    size_t count)
{
    double scores[256];
    adc_cal_dither_peak_candidate_t candidate;
    adc_cal_dither_peak_config_t peak_config;
    adc_cal_dither_validation_config_t confidence_config;
    size_t candidate_count = 0U;
    if (count > 256U || adc_cal_dither_compute_timing_scores(
            template_samples, capture_samples, count,
            ADC_CAL_DITHER_DEFAULT_ENERGY_SCORE_WEIGHT,
            ADC_CAL_DITHER_DEFAULT_EDGE_SCORE_WEIGHT, scores) != 0) {
        return NAN;
    }
    adc_cal_dither_peak_default_config(&peak_config);
    peak_config.maximum_candidates = 1U;
    adc_cal_dither_validation_default_config(&confidence_config);
    if (adc_cal_dither_find_peak_candidates(
            scores, count, 0.0, &peak_config, &confidence_config,
            &candidate, 1U, &candidate_count) != 0 ||
        candidate_count != 1U) {
        return NAN;
    }
    return candidate.lag_samples;
}

static int unit_dither_correlation_coordinates(sim_assert_context_t *ctx)
{
    enum { count = 128 };
    double ideal[count] = {0.0};
    double rectangle[count] = {0.0};
    double shaped[count] = {0.0};
    double capture[count] = {0.0};
    double random_template[count] = {0.0};
    double random_capture[count] = {0.0};
    double direct_scores[count];
    double energy_scores[count];
    double edge_scores[count];
    double timing_scores[count];
    double anchor_delay = NAN;
    double adc_even = NAN;
    double adc_odd = NAN;
    double expected_lag = NAN;
    adc_cal_dither_lag_offsets_t offsets;
    adc_cal_dither_coordinate_mapping_t mapping;
    adc_cal_dither_periodic_difference_t before;
    adc_cal_dither_periodic_difference_t after;
    const double fractional_lag = 9.25;

    /* A direct circular score aligns T[i] with C[i+lag].  It reports the
     * physical impulse shift, not L-1, (L-1)/2, pulse start, or pulse center. */
    ideal[11] = 1.0;
    capture[28] = 1.0;
    SIM_ASSERT_NEAR(ctx, test_dither_correlation_lag(
        ideal, capture, count, ADC_CAL_DITHER_CORRELATION_SIGNED),
        17.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_template_anchor_delay(
        ideal, count, ADC_CAL_DITHER_CORRELATION_SIGNED, &anchor_delay), 0);
    SIM_ASSERT_NEAR(ctx, anchor_delay, 0.0, 1.0e-12);

    memset(capture, 0, sizeof(capture));
    for (size_t i = 20U; i < 28U; ++i) rectangle[i] = 1.0;
    for (size_t i = 0U; i < count; ++i) {
        capture[(i + 13U) % count] = rectangle[i];
    }
    SIM_ASSERT_NEAR(ctx, test_dither_correlation_lag(
        rectangle, capture, count, ADC_CAL_DITHER_CORRELATION_SIGNED),
        13.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_template_anchor_delay(
        rectangle, count, ADC_CAL_DITHER_CORRELATION_ENERGY,
        &anchor_delay), 0);
    SIM_ASSERT_NEAR(ctx, anchor_delay, 0.0, 1.0e-12);

    memset(capture, 0, sizeof(capture));
    for (size_t i = 0U; i < 11U; ++i) {
        shaped[40U + i] = 1.0 - fabs((double)i - 5.0) / 6.0;
    }
    for (size_t n = 0U; n < count; ++n) {
        double source = (double)n - fractional_lag;
        long lower = (long)floor(source);
        const double fraction = source - (double)lower;
        while (lower < 0) lower += count;
        capture[n] = (1.0 - fraction) * shaped[(size_t)lower % count] +
            fraction * shaped[((size_t)lower + 1U) % count];
    }
    SIM_ASSERT_NEAR(ctx, test_dither_correlation_lag(
        shaped, capture, count, ADC_CAL_DITHER_CORRELATION_SIGNED),
        fractional_lag, 0.08);
    SIM_ASSERT_NEAR(ctx, test_dither_timing_lag(
        shaped, capture, count), fractional_lag, 0.08);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_circular_scores(
        shaped, shaped, count, ADC_CAL_DITHER_CORRELATION_ENERGY,
        energy_scores), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_circular_scores(
        shaped, shaped, count, ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY,
        edge_scores), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_timing_scores(
        shaped, shaped, count,
        ADC_CAL_DITHER_DEFAULT_ENERGY_SCORE_WEIGHT,
        ADC_CAL_DITHER_DEFAULT_EDGE_SCORE_WEIGHT, timing_scores), 0);
    SIM_ASSERT_TRUE(ctx, 1.0 - edge_scores[1] >
        1.0 - energy_scores[1]);
    SIM_ASSERT_TRUE(ctx, 1.0 - timing_scores[1] >
        1.0 - energy_scores[1]);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_circular_scores(
        shaped, capture, count, ADC_CAL_DITHER_CORRELATION_SIGNED,
        direct_scores), 0);
    /* Explicit reversed-template convolution at output n+L-1 is the same
     * dot product; the production API has already removed that raw index. */
    {
        double direct_numerator = 0.0;
        double template_mean = 0.0;
        double capture_mean = 0.0;
        double template_power = 0.0;
        double capture_power = 0.0;
        for (size_t i = 0U; i < count; ++i) {
            template_mean += shaped[i];
            capture_mean += capture[i];
        }
        template_mean /= count;
        capture_mean /= count;
        for (size_t i = 0U; i < count; ++i) {
            const double x = shaped[i] - template_mean;
            const double y = capture[(i + 9U) % count] - capture_mean;
            direct_numerator += x * y;
            template_power += x * x;
            capture_power += (capture[i] - capture_mean) *
                (capture[i] - capture_mean);
        }
        SIM_ASSERT_NEAR(ctx, direct_scores[9],
            direct_numerator / sqrt(template_power * capture_power),
            1.0e-12);
    }

    /* The DAC reconstruction is point evaluation by linear interpolation.
     * Its symmetric two-tap fractional kernel adds no coordinate delay. This
     * case deliberately preserves the historical replay rate. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_dac_position_to_adc_position(
        96.0, 2600000000.0 / SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ,
        0.0, 0.0, &adc_even), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_dac_position_to_adc_position(
        96.0, 2600000000.0 / SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ,
        1.0, 0.0, &adc_odd), 0);
    SIM_ASSERT_NEAR(ctx, adc_even,
        96.0 / (2600000000.0 / SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ),
        1.0e-12);
    SIM_ASSERT_NEAR(ctx, adc_even - adc_odd,
        1.0 / (2600000000.0 / SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ),
        1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_map_reference_position(
        71.38461538461539, 108.0, 15.976486, 1016.0, &mapping), 0);
    SIM_ASSERT_NEAR(ctx, mapping.window_relative_position_samples,
        -36.61538461538461, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, mapping.capture_unwrapped_position_samples,
        87.36110138461539, 1.0e-12);

    /* Independent per-event signs make a signed truncated-frame template
     * invalid when the capture begins elsewhere in the long DAC loop.  The
     * centered-energy observable retains identical event support. */
    for (size_t event = 0U; event < 8U; ++event) {
        const double template_sign = (event & 1U) ? -1.0 : 1.0;
        const double capture_sign =
            (event == 0U || event == 3U || event == 4U) ? -1.0 : 1.0;
        const size_t center = 8U + event * 16U;
        for (long j = -3; j <= 3; ++j) {
            const double value = 1.0 - fabs((double)j) / 4.0;
            const size_t source = (size_t)((long)center + j);
            random_template[source] = template_sign * value;
            random_capture[(source + 7U) % count] = capture_sign * value;
        }
    }
    SIM_ASSERT_NEAR(ctx, test_dither_correlation_lag(
        random_template, random_capture, count,
        ADC_CAL_DITHER_CORRELATION_ENERGY), 7.0, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, test_dither_timing_lag(
        random_template, random_capture, count), 7.0, 0.03);

    memset(&offsets, 0, sizeof(offsets));
    offsets.template_anchor_delay_samples = anchor_delay;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_expected_lag(
        15.976486, &offsets, &expected_lag), 0);
    SIM_ASSERT_NEAR(ctx, expected_lag, 15.976486, 1.0e-12);
    /* The old hardware number remains rejected: no 7.1 correction is hidden
     * in the coordinate conversion.  A correctly detected synthetic family
     * closes without relaxing the one-sample tolerance. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        fmod(-15.976486 + 1016.0, 1016.0),
        fmod(-467.941169 + 1016.0, 1016.0),
        142.791120, 1016.0, &before), 0);
    SIM_ASSERT_NEAR(ctx, before.absolute_difference_samples,
        7.129165, 5.0e-6);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_origins(
        fmod(-15.976486 + 1016.0, 1016.0),
        fmod(-(15.976486 + 4.0 * 142.791120) + 1016.0, 1016.0),
        142.791120, 1016.0, &after), 0);
    SIM_ASSERT_NEAR(ctx, after.absolute_difference_samples, 0.0, 1.0e-9);
    /* Once the tone-cycle ambiguity is resolved, validation must compare the
     * resolved waveform lag with the independently detected dither lag. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compare_periodic_lags(
        14.188080, 316.879182, 142.791120, 1016.0, &after), 0);
    SIM_ASSERT_NEAR(ctx, after.absolute_difference_samples,
        0.646702, 5.0e-6);
    SIM_ASSERT_TRUE(ctx, after.absolute_difference_samples < 1.0);
    {
        double matched_ratio = NAN;
        const double nominal_ratio =
            2600000000.0 / SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ;
        SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_rate_ratio_from_tone_bins(
            nominal_ratio, 70.0, 71.0, 0.05, &matched_ratio), 0);
        SIM_ASSERT_NEAR(ctx, matched_ratio,
            nominal_ratio * 71.0 / 70.0, 1.0e-12);
        SIM_ASSERT_TRUE(ctx, adc_cal_dither_rate_ratio_from_tone_bins(
            nominal_ratio, 70.0, 80.0, 0.05, &matched_ratio) != 0);
        SIM_ASSERT_TRUE(ctx, adc_cal_dither_rate_ratio_from_tone_bins(
            nominal_ratio, 0.0, 71.0, 0.05, &matched_ratio) != 0);
        SIM_ASSERT_TRUE(ctx, adc_cal_dither_rate_ratio_from_tone_bins(
            nominal_ratio, 70.0, 71.0, 0.05, NULL) != 0);
    }
    return 1;
}

static int unit_txt_waveform_timing_and_dither_diagnostics(
    sim_assert_context_t *ctx)
{
    enum {
        RAW_SAMPLE_COUNT = 65024,
        CANDIDATE_COUNT = 8,
        OFFSET_WINDOW_START = 108,
        OFFSET_WINDOW_LENGTH = 800
    };
    static int16_t raw_dac[RAW_SAMPLE_COUNT];
    static int16_t even_reference[SIM_ADC_CHANNEL_SAMPLES];
    static int16_t odd_reference[SIM_ADC_CHANNEL_SAMPLES];
    static int16_t channel_a[SIM_ADC_CHANNEL_SAMPLES];
    static int16_t channel_b[SIM_ADC_CHANNEL_SAMPLES];
    static uint8_t dma_bytes[ADC_RAW_FRAME_BYTES];
    static double dither_reference[SIM_ADC_CHANNEL_SAMPLES];
    static double residual_a[SIM_ADC_CHANNEL_SAMPLES];
    static double residual_b[SIM_ADC_CHANNEL_SAMPLES];
    static double scores_a[SIM_ADC_CHANNEL_SAMPLES];
    static double scores_b[SIM_ADC_CHANNEL_SAMPLES];
    static double edge_scores_a[SIM_ADC_CHANNEL_SAMPLES];
    static double edge_scores_b[SIM_ADC_CHANNEL_SAMPLES];
    static int16_t offset_reference[OFFSET_WINDOW_LENGTH];
    static int16_t offset_adc[OFFSET_WINDOW_LENGTH];
    static int16_t offset_adc_a[OFFSET_WINDOW_LENGTH];
    static int16_t offset_adc_b[OFFSET_WINDOW_LENGTH];
    static double offset_reference_residual[OFFSET_WINDOW_LENGTH];
    static double offset_a_residual[OFFSET_WINDOW_LENGTH];
    static double offset_b_residual[OFFSET_WINDOW_LENGTH];
    static double offset_aligned_template[OFFSET_WINDOW_LENGTH];
    static double offset_capture_template[OFFSET_WINDOW_LENGTH];
    static double offset_energy_scores[OFFSET_WINDOW_LENGTH];
    static double offset_edge_scores[OFFSET_WINDOW_LENGTH];
    adc_cal_dither_peak_candidate_t candidates_a[CANDIDATE_COUNT];
    adc_cal_dither_peak_candidate_t candidates_b[CANDIDATE_COUNT];
    adc_cal_dither_event_summary_t events;
    adc_cal_dither_peak_config_t peak_config;
    adc_cal_dither_validation_config_t validation_config;
    adc_cal_dither_validation_input_t validation_input;
    adc_cal_dither_validation_result_t validation_result;
    adc_cal_dither_joint_config_t joint_config;
    adc_cal_dither_joint_result_t joint;
    timing_alignment_result_t timing;
    calibration_state_t offset_fit;
    calibration_config_t offset_config;
    size_t raw_count = 0U;
    size_t candidate_count_a = 0U;
    size_t candidate_count_b = 0U;
    size_t dma_byte_count = 0U;
    size_t reconstructed_count = 0U;
    float fractional_lag = 0.0f;
    double existing_lag;
    double resolved_lag;
    double expected_origin;
    int32_t tone_cycle_offset;
    adc_cal_dither_periodic_difference_t cycle_residual;
    const int16_t *selected_reference = NULL;
    const int16_t *selected_channel = NULL;
    const double adc_rate_hz = SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ;
    const double dac_rate_hz = 2600000000.0;
    const double tone_hz = 100003075.78740157;
    const double timing_lags[] = {0.25, 17.25, -31.25, 71.25, -67.25};

    SIM_ASSERT_EQ_INT(ctx, load_i16_text_waveform(
        ADC_CAL_TEST_WAVEFORM_PATH, raw_dac, RAW_SAMPLE_COUNT,
        &raw_count), 0);
    SIM_ASSERT_EQ_INT(ctx, raw_count, RAW_SAMPLE_COUNT);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_resample_dac_reference(
        raw_dac, raw_count, dac_rate_hz / adc_rate_hz, 0.0,
        even_reference, SIM_ADC_CHANNEL_SAMPLES), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_resample_dac_reference(
        raw_dac, raw_count, dac_rate_hz / adc_rate_hz, 1.0,
        odd_reference, SIM_ADC_CHANNEL_SAMPLES), 0);
    SIM_ASSERT_TRUE(ctx, memcmp(
        even_reference, odd_reference, sizeof(even_reference)) != 0);

    /* adc -cal timing: every modeled frame must pass correlation/fractional
     * checks and recover phase modulo the 100 MHz tone period.  The real DMA
     * fixture below supplies the dither family used for absolute unwrapping. */
    for (size_t frame = 0U;
         frame < sizeof(timing_lags) / sizeof(timing_lags[0]); ++frame) {
        SIM_ASSERT_EQ_INT(ctx, make_scaled_dac_capture(
            raw_dac, raw_count, channel_a, SIM_ADC_CHANNEL_SAMPLES,
            dac_rate_hz / adc_rate_hz, timing_lags[frame], 0.018), 0);
        SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(
            even_reference, channel_a, SIM_ADC_CHANNEL_SAMPLES,
            &timing), 0);
        SIM_ASSERT_EQ_INT(ctx, timing_estimate_fractional_lag(
            even_reference, channel_a, SIM_ADC_CHANNEL_SAMPLES,
            timing.lag_samples, &fractional_lag), 0);
        SIM_ASSERT_TRUE(ctx, timing.correlation >= 0.970f);
        SIM_ASSERT_TRUE(ctx, fabs((double)fractional_lag) <= 0.5);
        {
            const double raw_error =
                (double)timing.lag_samples + (double)fractional_lag -
                timing_lags[frame];
            const double tone_period = adc_rate_hz / tone_hz;
            const double cycle_reduced_error = raw_error -
                round(raw_error / tone_period) * tone_period;
            SIM_ASSERT_NEAR(ctx, cycle_reduced_error, 0.0, 0.05);
        }
    }

    /* adc -cal diagnose: consume the exact captured DMA bytes, reconstruct
     * both hardware channels, and select the best uploaded-reference phase. */
    SIM_ASSERT_EQ_INT(ctx, load_u8_csv_capture(
        ADC_CAL_TEST_CAPTURE_PATH, dma_bytes, ADC_RAW_FRAME_BYTES,
        &dma_byte_count), 0);
    SIM_ASSERT_EQ_INT(ctx, dma_byte_count, ADC_RAW_FRAME_BYTES);
    SIM_ASSERT_EQ_INT(ctx, adc_reconstruct_channels(
        dma_bytes, dma_byte_count, channel_a, SIM_ADC_CHANNEL_SAMPLES,
        channel_b, SIM_ADC_CHANNEL_SAMPLES, &reconstructed_count), 0);
    SIM_ASSERT_EQ_INT(ctx, reconstructed_count, SIM_ADC_CHANNEL_SAMPLES);
    {
        const int16_t *references[2] = {even_reference, odd_reference};
        const int16_t *channels[2] = {channel_a, channel_b};
        float best_correlation = -2.0f;
        for (size_t channel = 0U; channel < 2U; ++channel) {
            for (size_t phase = 0U; phase < 2U; ++phase) {
                timing_alignment_result_t candidate_timing;
                SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(
                    references[phase], channels[channel],
                    SIM_ADC_CHANNEL_SAMPLES, &candidate_timing), 0);
                if (candidate_timing.correlation > best_correlation) {
                    best_correlation = candidate_timing.correlation;
                    timing = candidate_timing;
                    selected_reference = references[phase];
                    selected_channel = channels[channel];
                }
            }
        }
    }
    SIM_ASSERT_TRUE(ctx, selected_reference != NULL && selected_channel != NULL);
    SIM_ASSERT_EQ_INT(ctx, timing_estimate_fractional_lag(
        selected_reference, selected_channel, SIM_ADC_CHANNEL_SAMPLES,
        timing.lag_samples, &fractional_lag), 0);
    existing_lag = (double)timing.lag_samples + (double)fractional_lag;
    SIM_ASSERT_TRUE(ctx, timing.correlation >= 0.970f);
    SIM_ASSERT_EQ_INT(ctx, remove_known_tone(
        selected_reference, SIM_ADC_CHANNEL_SAMPLES, tone_hz, adc_rate_hz,
        dither_reference), 0);
    SIM_ASSERT_EQ_INT(ctx, remove_known_tone(
        channel_a, SIM_ADC_CHANNEL_SAMPLES, tone_hz, adc_rate_hz,
        residual_a), 0);
    SIM_ASSERT_EQ_INT(ctx, remove_known_tone(
        channel_b, SIM_ADC_CHANNEL_SAMPLES, tone_hz, adc_rate_hz,
        residual_b), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_summarize_events(
        dither_reference, SIM_ADC_CHANNEL_SAMPLES, 108U, 800U, 0.25,
        &events), 0);
    SIM_ASSERT_TRUE(ctx, events.complete_count >= 1U);
    SIM_ASSERT_EQ_INT(ctx, events.partial_count, 0U);

    adc_cal_dither_peak_default_config(&peak_config);
    adc_cal_dither_validation_default_config(&validation_config);
    peak_config.maximum_candidates = CANDIDATE_COUNT;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_circular_scores(
        dither_reference, residual_a, SIM_ADC_CHANNEL_SAMPLES,
        ADC_CAL_DITHER_CORRELATION_ENERGY, scores_a), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_circular_scores(
        dither_reference, residual_b, SIM_ADC_CHANNEL_SAMPLES,
        ADC_CAL_DITHER_CORRELATION_ENERGY, scores_b), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_find_peak_candidates(
        scores_a, SIM_ADC_CHANNEL_SAMPLES, events.spacing_samples,
        &peak_config, &validation_config, candidates_a, CANDIDATE_COUNT,
        &candidate_count_a), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_find_peak_candidates(
        scores_b, SIM_ADC_CHANNEL_SAMPLES, events.spacing_samples,
        &peak_config, &validation_config, candidates_b, CANDIDATE_COUNT,
        &candidate_count_b), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_circular_scores(
        dither_reference, residual_a, SIM_ADC_CHANNEL_SAMPLES,
        ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY, edge_scores_a), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_circular_scores(
        dither_reference, residual_b, SIM_ADC_CHANNEL_SAMPLES,
        ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY, edge_scores_b), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_refine_candidate_lags(
        edge_scores_a, SIM_ADC_CHANNEL_SAMPLES,
        ADC_CAL_DITHER_DEFAULT_EDGE_REFINE_RADIUS,
        candidates_a, candidate_count_a), 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_refine_candidate_lags(
        edge_scores_b, SIM_ADC_CHANNEL_SAMPLES,
        ADC_CAL_DITHER_DEFAULT_EDGE_REFINE_RADIUS,
        candidates_b, candidate_count_b), 0);

    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_lag_to_wrapped_origin(
        existing_lag, (double)SIM_ADC_CHANNEL_SAMPLES,
        &expected_origin), 0);
    adc_cal_dither_joint_default_config(&joint_config);
    joint_config.use_expected_origin = 1;
    joint_config.expected_origin_samples = expected_origin;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        candidates_a, candidate_count_a, candidates_b, candidate_count_b,
        events.spacing_samples, (double)SIM_ADC_CHANNEL_SAMPLES,
        &joint_config, &joint), 0);
    SIM_ASSERT_TRUE(ctx, joint.valid);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_resolve_tone_cycle(
        existing_lag, adc_rate_hz / tone_hz,
        joint.consensus_origin_samples, events.spacing_samples,
        (double)SIM_ADC_CHANNEL_SAMPLES, 1.0, &resolved_lag,
        &tone_cycle_offset, &cycle_residual), 0);
    SIM_ASSERT_TRUE(ctx, cycle_residual.absolute_difference_samples <= 1.0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_lag_to_wrapped_origin(
        resolved_lag, (double)SIM_ADC_CHANNEL_SAMPLES,
        &expected_origin), 0);
    joint_config.expected_origin_samples = expected_origin;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_select_joint_candidate_pair(
        candidates_a, candidate_count_a, candidates_b, candidate_count_b,
        events.spacing_samples, (double)SIM_ADC_CHANNEL_SAMPLES,
        &joint_config, &joint), 0);
    SIM_ASSERT_TRUE(ctx, joint.existing_consistent);
    SIM_ASSERT_TRUE(ctx,
        joint.channel_difference.absolute_difference_samples <= 1.0);
    SIM_ASSERT_NEAR(ctx,
        joint.channel_a_existing_residual_samples, 0.0, 1.0);
    SIM_ASSERT_NEAR(ctx,
        joint.channel_b_existing_residual_samples, 0.0, 1.0);

    memset(&validation_input, 0, sizeof(validation_input));
    validation_config.minimum_complete_events = 1U;
    validation_input.selected_channel_valid = 1;
    validation_input.channel_a_available = 1;
    validation_input.channel_b_available = 1;
    validation_input.channel_a_valid = 1;
    validation_input.channel_b_valid = 1;
    validation_input.joint_pair_required = 1;
    validation_input.joint_pair_valid = joint.valid;
    validation_input.existing_comparison_required = 1;
    validation_input.channel_disagreement_samples =
        joint.channel_difference.absolute_difference_samples;
    validation_input.existing_disagreement_samples = fmax(
        joint.channel_a_existing_residual_samples,
        joint.channel_b_existing_residual_samples);
    validation_input.complete_event_count = events.complete_count;
    validation_input.event_indices_valid = events.partial_count == 0U;
    validation_input.numerical_values_finite = 1;
    validation_input.independent_peak_ratio =
        candidates_a[joint.channel_a_candidate].absolute_peak >=
                candidates_b[joint.channel_b_candidate].absolute_peak ?
            candidates_a[joint.channel_a_candidate].independent_peak_ratio :
            candidates_b[joint.channel_b_candidate].independent_peak_ratio;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_validate_detection(
        &validation_input, &validation_config, &validation_result), 0);
    SIM_ASSERT_TRUE(ctx, validation_result.structural_valid);
    SIM_ASSERT_TRUE(ctx,
        validation_result.recommendation != ADC_CAL_DITHER_RECOMMEND_REJECT);

    /* Exercise the fixed-window estimator used by adc -cal offset with the
     * exact uploaded waveform and captured DMA frame.  The estimator must
     * retain high correlation after fractional timing alignment, and its
     * additive correction must cancel both the measured hardware residual
     * and representative positive/negative injected offsets. */
    for (size_t i = 0U; i < OFFSET_WINDOW_LENGTH; ++i) {
        const double source_position =
            (double)(OFFSET_WINDOW_START + i) + existing_lag;
        size_t lower;
        double fraction;
        SIM_ASSERT_TRUE(ctx, source_position >= 0.0);
        SIM_ASSERT_TRUE(ctx,
            source_position < (double)(SIM_ADC_CHANNEL_SAMPLES - 1U));
        if (source_position < 0.0 ||
            source_position >= (double)(SIM_ADC_CHANNEL_SAMPLES - 1U)) {
            return 0;
        }
        lower = (size_t)floor(source_position);
        fraction = source_position - (double)lower;
        offset_reference[i] = selected_reference[OFFSET_WINDOW_START + i];
        offset_adc_a[i] = (int16_t)lrint(
            (1.0 - fraction) * (double)channel_a[lower] +
            fraction * (double)channel_a[lower + 1U]);
        offset_adc_b[i] = (int16_t)lrint(
            (1.0 - fraction) * (double)channel_b[lower] +
            fraction * (double)channel_b[lower + 1U]);
        offset_adc[i] = (int16_t)lrint(
            (1.0 - fraction) * (double)selected_channel[lower] +
            fraction * (double)selected_channel[lower + 1U]);
    }
    calibration_default_config(&offset_config);
    SIM_ASSERT_EQ_INT(ctx,
        calibration_init(&offset_fit, &offset_config), CALIBRATION_OK);
    SIM_ASSERT_EQ_INT(ctx, calibration_analyze_frame(
        &offset_fit, offset_adc, offset_reference, OFFSET_WINDOW_LENGTH),
        CALIBRATION_OK);
    SIM_ASSERT_TRUE(ctx, offset_fit.metrics.correlation >= 0.970f);
    SIM_ASSERT_TRUE(ctx, isfinite(offset_fit.metrics.measured_gain));
    SIM_ASSERT_TRUE(ctx, isfinite(offset_fit.metrics.measured_offset));
    {
        double hardware_residual_sum = 0.0;
        const double nominal_gain = offset_fit.metrics.measured_gain;
        const double injected_offsets[] = {0.0, 37.0, -42.0};
        for (size_t i = 0U; i < OFFSET_WINDOW_LENGTH; ++i) {
            hardware_residual_sum += (double)offset_adc[i] -
                nominal_gain * (double)offset_reference[i];
        }
        const double hardware_residual =
            hardware_residual_sum / (double)OFFSET_WINDOW_LENGTH;
        SIM_ASSERT_NEAR(ctx, hardware_residual,
            offset_fit.metrics.measured_offset, 0.01);
        for (size_t test = 0U;
             test < sizeof(injected_offsets) / sizeof(injected_offsets[0]);
             ++test) {
            const double measured_residual =
                hardware_residual + injected_offsets[test];
            const double offset_correction = -measured_residual;
            SIM_ASSERT_NEAR(ctx,
                measured_residual + offset_correction, 0.0, 1.0e-9);
        }
    }

    /* Reproduce the production capture-polarity strategy on the real DMA
     * fixture.  Absolute DAC pulse signs may differ in a fresh part of the
     * long loop, but pulse energy locates the train and Channel A supplies
     * the per-event sign needed for relative B-A skew. */
    SIM_ASSERT_EQ_INT(ctx, remove_known_tone(
        offset_reference, OFFSET_WINDOW_LENGTH, tone_hz, adc_rate_hz,
        offset_reference_residual), 0);
    SIM_ASSERT_EQ_INT(ctx, remove_known_tone(
        offset_adc_a, OFFSET_WINDOW_LENGTH, tone_hz, adc_rate_hz,
        offset_a_residual), 0);
    SIM_ASSERT_EQ_INT(ctx, remove_known_tone(
        offset_adc_b, OFFSET_WINDOW_LENGTH, tone_hz, adc_rate_hz,
        offset_b_residual), 0);
    {
        adc_cal_dither_peak_candidate_t window_candidates[CANDIDATE_COUNT];
        adc_cal_dither_peak_config_t window_peak_config;
        adc_cal_dither_validation_config_t window_validation_config;
        adc_cal_dither_config_t window_event_config;
        adc_cal_dither_result_t window_events;
        adc_cal_skew_config_t skew_config;
        adc_cal_skew_result_t skew_result;
        size_t window_candidate_count = 0U;
        adc_cal_dither_peak_default_config(&window_peak_config);
        adc_cal_dither_validation_default_config(&window_validation_config);
        window_peak_config.maximum_candidates = CANDIDATE_COUNT;
        SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_circular_scores(
            offset_reference_residual, offset_a_residual,
            OFFSET_WINDOW_LENGTH, ADC_CAL_DITHER_CORRELATION_ENERGY,
            offset_energy_scores), 0);
        SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_find_peak_candidates(
            offset_energy_scores, OFFSET_WINDOW_LENGTH,
            events.spacing_samples, &window_peak_config,
            &window_validation_config, window_candidates, CANDIDATE_COUNT,
            &window_candidate_count), 0);
        SIM_ASSERT_TRUE(ctx, window_candidate_count > 0U);
        SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_compute_circular_scores(
            offset_reference_residual, offset_a_residual,
            OFFSET_WINDOW_LENGTH, ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY,
            offset_edge_scores), 0);
        SIM_ASSERT_EQ_INT(ctx, adc_cal_dither_refine_candidate_lags(
            offset_edge_scores, OFFSET_WINDOW_LENGTH,
            ADC_CAL_DITHER_DEFAULT_EDGE_REFINE_RADIUS,
            window_candidates, window_candidate_count), 0);
        for (size_t i = 0U; i < OFFSET_WINDOW_LENGTH; ++i) {
            offset_aligned_template[i] = sim_signal_interpolate_circular(
                offset_reference_residual, OFFSET_WINDOW_LENGTH,
                (double)i - window_candidates[0].lag_samples);
        }
        adc_cal_dither_default_config(&window_event_config);
        window_event_config.minimum_events = 3U;
        window_event_config.boundary_margin = 1U;
        {
            const int event_status = adc_cal_dither_find_events(
                offset_aligned_template, OFFSET_WINDOW_LENGTH,
                &window_event_config, &window_events);
            SIM_ASSERT_TRUE(ctx, event_status == 0 ||
                event_status == ADC_CAL_DITHER_ERR_POLARITY);
        }
        memset(offset_capture_template, 0, sizeof(offset_capture_template));
        for (size_t k = 0U; k < window_events.accepted_events; ++k) {
            const size_t start = window_events.events[k].start;
            const size_t end = window_events.events[k].end;
            const double q = window_events.events[k].polarity;
            double capture_sum = 0.0;
            for (size_t i = start; i < end; ++i) {
                capture_sum += offset_a_residual[i];
            }
            const double p = capture_sum >= 0.0 ? 1.0 : -1.0;
            for (size_t i = start; i < end; ++i) {
                offset_capture_template[i] =
                    p * q * offset_aligned_template[i];
            }
        }
        adc_cal_skew_default_config(&skew_config);
        skew_config.sample_rate_hz = adc_rate_hz;
        SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(
            offset_a_residual, offset_b_residual, offset_capture_template,
            OFFSET_WINDOW_LENGTH, &skew_config, &skew_result), 0);
        SIM_ASSERT_TRUE(ctx, skew_result.valid);
        SIM_ASSERT_TRUE(ctx,
            sim_double_isfinite(skew_result.relative_skew_samples));
        SIM_ASSERT_TRUE(ctx,
            fabs(skew_result.relative_skew_samples) <=
                skew_config.max_linear_skew_samples);
        for (size_t i = 0U; i < OFFSET_WINDOW_LENGTH; ++i) {
            offset_capture_template[i] *= 0.25;
        }
        {
            adc_cal_dither_result_t gain_projection;
            const int gain_status = adc_cal_dither_analyze(
                offset_a_residual, offset_capture_template,
                OFFSET_WINDOW_LENGTH, &window_event_config,
                &gain_projection);
            SIM_ASSERT_EQ_INT(ctx, gain_status, 0);
            SIM_ASSERT_TRUE(ctx,
                gain_projection.normalized_projection > 0.0001);
            SIM_ASSERT_TRUE(ctx,
                gain_projection.normalized_projection < 20.0);
        }
    }
    return 1;
}

/* End-to-end replay context for the exact uploaded DAC waveform and recorded
 * DMA frame.  A single capture cannot model independent analog noise from one
 * batch to the next, so the fixture is replayed deterministically while the
 * software corrections and production pipeline state advance normally. */
typedef struct {
    int16_t raw_dac[65024U];
    uint8_t dma_bytes[ADC_RAW_FRAME_BYTES];
    int16_t reference_even[SIM_ADC_CHANNEL_SAMPLES];
    int16_t reference_odd[SIM_ADC_CHANNEL_SAMPLES];
    int16_t channel_a[SIM_ADC_CHANNEL_SAMPLES];
    int16_t channel_b[SIM_ADC_CHANNEL_SAMPLES];
    int16_t reference_window[800U];
    int16_t aligned_a[800U];
    int16_t aligned_b[800U];
    const char *capture_path;
    const int16_t *selected_reference;
    const int16_t *selected_channel;
    size_t raw_count;
    size_t dma_byte_count;
    size_t reconstructed_count;
    int selected_phase;
    int selected_channel_index;
    double lag_samples;
    double correlation;
    double analysis_tone_hz;
    double nominal_rate_ratio;
    double selected_rate_ratio;
    size_t reference_tone_bin;
    size_t captured_tone_bin;
    int reference_rate_adapted;
    double measured_gain;
    double measured_offset;
    double tone_phase_a;
    double tone_phase_b;
    double dither_skew_samples;
    int dither_valid;
    int gain_dither_valid;
    uint32_t replay_count;
    uint32_t timing_replays;
    uint32_t offset_batches;
    uint32_t gain_batches;
    uint32_t skew_replays;
    uint32_t performance_replays;
    adc_cal_perf_batch_result_t performance;
} fixture_pipeline_context_t;

static size_t fixture_dominant_bin(const int16_t *samples, size_t count)
{
    double mean = 0.0;
    double best_power = -1.0;
    size_t best_bin = 0U;
    if (samples == NULL || count < 4U) return 0U;
    for (size_t i = 0U; i < count; ++i) mean += samples[i];
    mean /= (double)count;
    for (size_t bin = 1U; bin < count / 2U; ++bin) {
        double real = 0.0;
        double imag = 0.0;
        for (size_t i = 0U; i < count; ++i) {
            const double angle = 2.0 * M_PI *
                (double)bin * (double)i / (double)count;
            const double value = (double)samples[i] - mean;
            real += value * cos(angle);
            imag -= value * sin(angle);
        }
        {
            const double power = real * real + imag * imag;
            if (power > best_power) {
                best_power = power;
                best_bin = bin;
            }
        }
    }
    return best_bin;
}

static int fixture_fit_tone_phase(
    const int16_t *samples,
    size_t sample_count,
    double frequency_hz,
    double sample_rate_hz,
    double *phase_rad)
{
    double matrix[3][4] = {{0.0}};
    if (samples == NULL || phase_rad == NULL || sample_count < 4U ||
        !sim_double_isfinite(frequency_hz) || frequency_hz <= 0.0 ||
        !sim_double_isfinite(sample_rate_hz) || sample_rate_hz <= 0.0) return -1;
    for (size_t i = 0U; i < sample_count; ++i) {
        const double angle = 2.0 * M_PI * frequency_hz *
            (double)i / sample_rate_hz;
        const double basis[3] = {cos(angle), sin(angle), 1.0};
        for (size_t row = 0U; row < 3U; ++row) {
            for (size_t col = 0U; col < 3U; ++col) {
                matrix[row][col] += basis[row] * basis[col];
            }
            matrix[row][3] += basis[row] * (double)samples[i];
        }
    }
    for (size_t pivot = 0U; pivot < 3U; ++pivot) {
        size_t best = pivot;
        for (size_t row = pivot + 1U; row < 3U; ++row) {
            if (fabs(matrix[row][pivot]) > fabs(matrix[best][pivot])) {
                best = row;
            }
        }
        if (fabs(matrix[best][pivot]) <= DBL_EPSILON) return -2;
        if (best != pivot) {
            for (size_t col = pivot; col < 4U; ++col) {
                const double swap = matrix[pivot][col];
                matrix[pivot][col] = matrix[best][col];
                matrix[best][col] = swap;
            }
        }
        {
            const double divisor = matrix[pivot][pivot];
            for (size_t col = pivot; col < 4U; ++col) {
                matrix[pivot][col] /= divisor;
            }
        }
        for (size_t row = 0U; row < 3U; ++row) {
            const double factor = matrix[row][pivot];
            if (row == pivot) continue;
            for (size_t col = pivot; col < 4U; ++col) {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
        }
    }
    *phase_rad = atan2(-matrix[1][3], matrix[0][3]);
    return sim_double_isfinite(*phase_rad) ? 0 : -3;
}

static int fixture_prepare_dither_skew(fixture_pipeline_context_t *fixture)
{
    enum { COUNT = 800, CANDIDATES = 8 };
    double reference_residual[COUNT];
    double residual_a[COUNT];
    double residual_b[COUNT];
    double energy_scores[COUNT];
    double edge_scores[COUNT];
    double aligned_template[COUNT];
    double capture_template[COUNT];
    double gain_template[COUNT];
    adc_cal_dither_peak_candidate_t candidates[CANDIDATES];
    adc_cal_dither_peak_config_t peak_config;
    adc_cal_dither_validation_config_t validation_config;
    adc_cal_dither_event_summary_t event_summary;
    adc_cal_dither_config_t event_config;
    adc_cal_dither_result_t events;
    adc_cal_skew_config_t skew_config;
    adc_cal_skew_result_t skew_result;
    size_t candidate_count = 0U;
    const double tone_hz = fixture->analysis_tone_hz;
    const double adc_rate_hz = SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ;

    fixture->dither_valid = 0;
    fixture->gain_dither_valid = 0;
    fixture->dither_skew_samples = NAN;
    if (remove_known_tone(fixture->reference_window, COUNT, tone_hz,
            adc_rate_hz, reference_residual) != 0 ||
        remove_known_tone(fixture->aligned_a, COUNT, tone_hz,
            adc_rate_hz, residual_a) != 0 ||
        remove_known_tone(fixture->aligned_b, COUNT, tone_hz,
            adc_rate_hz, residual_b) != 0) return -1;
    adc_cal_dither_peak_default_config(&peak_config);
    adc_cal_dither_validation_default_config(&validation_config);
    peak_config.maximum_candidates = CANDIDATES;
    if (adc_cal_dither_summarize_events(
            reference_residual, COUNT, 0U, COUNT, 0.25,
            &event_summary) != 0 || event_summary.complete_count < 3U ||
        adc_cal_dither_compute_circular_scores(
            reference_residual, residual_a, COUNT,
            ADC_CAL_DITHER_CORRELATION_ENERGY, energy_scores) != 0 ||
        adc_cal_dither_find_peak_candidates(
            energy_scores, COUNT, event_summary.spacing_samples, &peak_config,
            &validation_config, candidates, CANDIDATES,
            &candidate_count) != 0 || candidate_count == 0U ||
        adc_cal_dither_compute_circular_scores(
            reference_residual, residual_a, COUNT,
            ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY, edge_scores) != 0 ||
        adc_cal_dither_refine_candidate_lags(
            edge_scores, COUNT, ADC_CAL_DITHER_DEFAULT_EDGE_REFINE_RADIUS,
            candidates, candidate_count) != 0) return -2;
    for (size_t i = 0U; i < COUNT; ++i) {
        aligned_template[i] = sim_signal_interpolate_circular(
            reference_residual, COUNT,
            (double)i - candidates[0].lag_samples);
    }
    adc_cal_dither_default_config(&event_config);
    event_config.minimum_events = 3U;
    event_config.boundary_margin = 1U;
    {
        const int status = adc_cal_dither_find_events(
            aligned_template, COUNT, &event_config, &events);
        if ((status != 0 && status != ADC_CAL_DITHER_ERR_POLARITY) ||
            events.accepted_events < event_config.minimum_events) return -3;
    }
    memset(capture_template, 0, sizeof(capture_template));
    memset(gain_template, 0, sizeof(gain_template));
    for (size_t event = 0U; event < events.accepted_events; ++event) {
        const size_t start = events.events[event].start;
        const size_t end = events.events[event].end;
        double capture_sum = 0.0;
        double gain_sum = 0.0;
        const double *selected_residual =
            fixture->selected_channel_index == 0 ? residual_a : residual_b;
        for (size_t i = start; i < end; ++i) capture_sum += residual_a[i];
        for (size_t i = start; i < end; ++i) gain_sum += selected_residual[i];
        {
            const double capture_sign = capture_sum >= 0.0 ? 1.0 : -1.0;
            const double gain_sign = gain_sum >= 0.0 ? 1.0 : -1.0;
            const double template_sign = events.events[event].polarity;
            for (size_t i = start; i < end; ++i) {
                capture_template[i] = capture_sign * template_sign *
                    aligned_template[i];
                gain_template[i] = gain_sign * template_sign *
                    aligned_template[i];
            }
        }
    }
    adc_cal_skew_default_config(&skew_config);
    skew_config.sample_rate_hz = adc_rate_hz;
    if (adc_cal_skew_estimate_from_residuals(
            residual_a, residual_b, capture_template, COUNT,
            &skew_config, &skew_result) != 0 || !skew_result.valid ||
        !sim_double_isfinite(skew_result.relative_skew_samples)) return -4;
    {
        adc_cal_dither_result_t gain_projection;
        const double *selected_residual =
            fixture->selected_channel_index == 0 ? residual_a : residual_b;
        if (adc_cal_dither_analyze(
                selected_residual, gain_template, COUNT,
                &event_config, &gain_projection) == 0 &&
            gain_projection.normalized_projection > 0.0001 &&
            gain_projection.normalized_projection < 20.0) {
            fixture->gain_dither_valid = 1;
        }
    }
    fixture->dither_valid = 1;
    fixture->dither_skew_samples = skew_result.relative_skew_samples;
    return 0;
}

static int fixture_pipeline_prepare(
    void *context,
    const adc_cal_pipeline_run_config_t *config,
    const char **reason)
{
    fixture_pipeline_context_t *fixture =
        (fixture_pipeline_context_t *)context;
    calibration_state_t fit;
    calibration_config_t fit_config;
    timing_alignment_result_t best_timing;
    float best_fractional = 0.0f;
    float best_correlation = -2.0f;
    const double adc_rate_hz = SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ;
    const double dac_rate_hz = 2600000000.0;
    const double tone_hz = 100003075.78740157;
    const char *capture_path = fixture->capture_path != NULL ?
        fixture->capture_path : ADC_CAL_TEST_CAPTURE_PATH;
    (void)config;
    memset(&best_timing, 0, sizeof(best_timing));
    memset(fixture, 0, sizeof(*fixture));
    fixture->capture_path = capture_path;
    fixture->nominal_rate_ratio = dac_rate_hz / adc_rate_hz;
    fixture->selected_rate_ratio = fixture->nominal_rate_ratio;
    fixture->analysis_tone_hz = tone_hz;
    fixture->dither_skew_samples = NAN;
    if (load_i16_text_waveform(
            ADC_CAL_TEST_WAVEFORM_PATH, fixture->raw_dac,
            sizeof(fixture->raw_dac) / sizeof(fixture->raw_dac[0]),
            &fixture->raw_count) != 0 || fixture->raw_count != 65024U) {
        if (reason != NULL) *reason = "could not load exact DAC TXT fixture";
        return -1;
    }
    if (adc_cal_dither_resample_dac_reference(
            fixture->raw_dac, fixture->raw_count,
            fixture->selected_rate_ratio, 0.0, fixture->reference_even,
            SIM_ADC_CHANNEL_SAMPLES) != 0 ||
        adc_cal_dither_resample_dac_reference(
            fixture->raw_dac, fixture->raw_count,
            fixture->selected_rate_ratio, 1.0, fixture->reference_odd,
            SIM_ADC_CHANNEL_SAMPLES) != 0) {
        if (reason != NULL) *reason = "DAC TXT resampling failed";
        return -2;
    }
    if (load_u8_csv_capture(
            fixture->capture_path, fixture->dma_bytes,
            sizeof(fixture->dma_bytes), &fixture->dma_byte_count) != 0 ||
        fixture->dma_byte_count != ADC_RAW_FRAME_BYTES ||
        adc_reconstruct_channels(
            fixture->dma_bytes, fixture->dma_byte_count,
            fixture->channel_a, SIM_ADC_CHANNEL_SAMPLES,
            fixture->channel_b, SIM_ADC_CHANNEL_SAMPLES,
            &fixture->reconstructed_count) != 0 ||
        fixture->reconstructed_count != SIM_ADC_CHANNEL_SAMPLES) {
        if (reason != NULL) *reason = "could not reconstruct exact DMA fixture";
        return -3;
    }
    fixture->reference_tone_bin = fixture_dominant_bin(
        fixture->reference_even, SIM_ADC_CHANNEL_SAMPLES);
    {
        const size_t bin_a = fixture_dominant_bin(
            fixture->channel_a, SIM_ADC_CHANNEL_SAMPLES);
        const size_t bin_b = fixture_dominant_bin(
            fixture->channel_b, SIM_ADC_CHANNEL_SAMPLES);
        fixture->captured_tone_bin = bin_a;
        if (bin_a == bin_b && bin_a != fixture->reference_tone_bin &&
            adc_cal_dither_rate_ratio_from_tone_bins(
                fixture->nominal_rate_ratio,
                (double)fixture->reference_tone_bin,
                (double)bin_a, 0.05,
                &fixture->selected_rate_ratio) == 0 &&
            adc_cal_dither_resample_dac_reference(
                fixture->raw_dac, fixture->raw_count,
                fixture->selected_rate_ratio, 0.0,
                fixture->reference_even,
                SIM_ADC_CHANNEL_SAMPLES) == 0 &&
            adc_cal_dither_resample_dac_reference(
                fixture->raw_dac, fixture->raw_count,
                fixture->selected_rate_ratio, 1.0,
                fixture->reference_odd,
                SIM_ADC_CHANNEL_SAMPLES) == 0) {
            fixture->reference_rate_adapted = 1;
            fixture->analysis_tone_hz = tone_hz *
                fixture->selected_rate_ratio / fixture->nominal_rate_ratio;
        }
    }
    {
        const int16_t *references[2] = {
            fixture->reference_even, fixture->reference_odd};
        const int16_t *channels[2] = {
            fixture->channel_a, fixture->channel_b};
        for (size_t channel = 0U; channel < 2U; ++channel) {
            for (size_t phase = 0U; phase < 2U; ++phase) {
                timing_alignment_result_t candidate;
                float fractional = 0.0f;
                if (timing_find_circular_lag(
                        references[phase], channels[channel],
                        SIM_ADC_CHANNEL_SAMPLES, &candidate) != 0 ||
                    timing_estimate_fractional_lag(
                        references[phase], channels[channel],
                        SIM_ADC_CHANNEL_SAMPLES, candidate.lag_samples,
                        &fractional) != 0) continue;
                if (candidate.correlation > best_correlation) {
                    best_correlation = candidate.correlation;
                    best_timing = candidate;
                    best_fractional = fractional;
                    fixture->selected_reference = references[phase];
                    fixture->selected_channel = channels[channel];
                    fixture->selected_phase = (int)phase;
                    fixture->selected_channel_index = (int)channel;
                }
            }
        }
    }
    if (fixture->selected_reference == NULL ||
        fixture->selected_channel == NULL || best_correlation < 0.970f) {
        if (reason != NULL) *reason = "recorded timing correlation is too low";
        return -4;
    }
    fixture->lag_samples = (double)best_timing.lag_samples +
        (double)best_fractional;
    fixture->correlation = best_correlation;
    for (size_t i = 0U; i < 800U; ++i) {
        const size_t reference_index = 108U + i;
        double source_position = fmod(
            (double)reference_index + fixture->lag_samples,
            (double)SIM_ADC_CHANNEL_SAMPLES);
        size_t lower;
        size_t upper;
        double fraction;
        if (source_position < 0.0) {
            source_position += (double)SIM_ADC_CHANNEL_SAMPLES;
        }
        lower = (size_t)floor(source_position);
        upper = lower + 1U;
        if (upper >= SIM_ADC_CHANNEL_SAMPLES) upper = 0U;
        fraction = source_position - (double)lower;
        fixture->reference_window[i] =
            fixture->selected_reference[reference_index];
        fixture->aligned_a[i] = (int16_t)lrint(
            (1.0 - fraction) * (double)fixture->channel_a[lower] +
            fraction * (double)fixture->channel_a[upper]);
        fixture->aligned_b[i] = (int16_t)lrint(
            (1.0 - fraction) * (double)fixture->channel_b[lower] +
            fraction * (double)fixture->channel_b[upper]);
    }
    calibration_default_config(&fit_config);
    if (calibration_init(&fit, &fit_config) != CALIBRATION_OK ||
        calibration_analyze_frame(
            &fit,
            fixture->selected_channel_index == 0 ?
                fixture->aligned_a : fixture->aligned_b,
            fixture->reference_window, 800U) != CALIBRATION_OK ||
        fit.metrics.correlation < 0.970f ||
        !isfinite(fit.metrics.measured_gain) ||
        !isfinite(fit.metrics.measured_offset)) {
        if (reason != NULL) *reason = "recorded offset/gain fit is invalid";
        return -6;
    }
    fixture->measured_gain = fit.metrics.measured_gain;
    fixture->measured_offset = fit.metrics.measured_offset;
    if (fixture_fit_tone_phase(
            fixture->aligned_a, 800U, fixture->analysis_tone_hz, adc_rate_hz,
            &fixture->tone_phase_a) != 0 ||
        fixture_fit_tone_phase(
            fixture->aligned_b, 800U, fixture->analysis_tone_hz, adc_rate_hz,
            &fixture->tone_phase_b) != 0) {
        if (reason != NULL) *reason = "recorded channel tone fit failed";
        return -7;
    }
    (void)fixture_prepare_dither_skew(fixture);
    return 0;
}

static int fixture_pipeline_timing(
    void *context,
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_run_config_t *config,
    const char **reason)
{
    fixture_pipeline_context_t *fixture =
        (fixture_pipeline_context_t *)context;
    (void)reason;
    fixture->replay_count += config->timing_frame_count;
    fixture->timing_replays += config->timing_frame_count;
    state->timing_pass = true;
    state->calibration_channel = (int8_t)fixture->selected_channel_index;
    state->canonical_reference_phase = (int8_t)fixture->selected_phase;
    state->fixed_window_start = 108U;
    state->fixed_window_length = 800U;
    state->expected_lag = (int32_t)lrint(fixture->lag_samples);
    state->timing_mean_correlation = (float)fixture->correlation;
    return 0;
}

static int fixture_pipeline_offset(
    void *context,
    adc_cal_pipeline_state_t *state,
    const char **reason)
{
    fixture_pipeline_context_t *fixture =
        (fixture_pipeline_context_t *)context;
    double correction = 0.0;
    unsigned consecutive = 0U;
    for (unsigned batch = 0U; batch < 20U; ++batch) {
        const double residual = fixture->measured_offset + correction;
        correction -= 0.35 * residual;
        fixture->replay_count += 30U;
        ++fixture->offset_batches;
        state->stage_iteration = batch + 1U;
        consecutive = fabs(fixture->measured_offset + correction) <= 1.0 ?
            consecutive + 1U : 0U;
        if (consecutive >= 2U) break;
    }
    state->offset_correction = (float)correction;
    state->offset_verification_error =
        (float)(fixture->measured_offset + correction);
    state->offset_pass = consecutive >= 2U;
    state->offset_result = state->offset_pass ?
        ADC_CAL_PIPELINE_OFFSET_CONVERGED : ADC_CAL_PIPELINE_OFFSET_FAILED;
    if (!state->offset_pass && reason != NULL) {
        *reason = "recorded offset replay did not converge within 20 batches";
    }
    return state->offset_pass ? 0 : -1;
}

static int fixture_pipeline_gain(
    void *context,
    adc_cal_pipeline_state_t *state,
    const char **reason)
{
    fixture_pipeline_context_t *fixture =
        (fixture_pipeline_context_t *)context;
    double correction = 1.0;
    const double nominal_system_gain = fixture->measured_gain;
    const double normalized_measured_gain =
        fixture->measured_gain / nominal_system_gain;
    unsigned consecutive = 0U;
    if (fixture->measured_gain <= 0.0) {
        if (reason != NULL) *reason = "recorded gain is non-positive";
        return -1;
    }
    for (unsigned batch = 0U; batch < 20U; ++batch) {
        const double error = normalized_measured_gain * correction - 1.0;
        correction -= 0.5 * error / normalized_measured_gain;
        fixture->replay_count += 30U;
        ++fixture->gain_batches;
        state->stage_iteration = batch + 1U;
        consecutive = fabs(normalized_measured_gain * correction - 1.0) <=
            CALIBRATION_GAIN_TOLERANCE ? consecutive + 1U : 0U;
        if (consecutive >= 2U) break;
    }
    state->gain_correction = (float)correction;
    state->nominal_system_gain = (float)nominal_system_gain;
    state->final_normalized_gain =
        (float)(normalized_measured_gain * correction);
    state->gain_verification_error = state->final_normalized_gain - 1.0f;
    state->gain_pass = consecutive >= 2U;
    state->gain_verification_pass = state->gain_pass &&
        fabs((double)state->gain_verification_error) <=
            CALIBRATION_GAIN_TOLERANCE;
    state->output_valid = state->gain_verification_pass;
    state->valid = state->output_valid;
    if (!state->output_valid && reason != NULL) {
        *reason = "recorded gain replay did not converge or verify";
    }
    return state->output_valid ? 0 : -2;
}

static int fixture_pipeline_skew(
    void *context,
    adc_cal_pipeline_state_t *state,
    const char **reason)
{
    fixture_pipeline_context_t *fixture =
        (fixture_pipeline_context_t *)context;
    adc_cal_skew_phase_config_t phase_config;
    adc_cal_skew_phase_result_t phase_result;
    adc_cal_skew_stage_policy_input_t policy_input;
    adc_cal_skew_phase_default_config(&phase_config);
    phase_config.dither_valid = fixture->dither_valid;
    phase_config.dither_skew_samples = fixture->dither_skew_samples;
    if (adc_cal_skew_resolve_tone_phase(
            fixture->tone_phase_a, fixture->tone_phase_b,
            fixture->analysis_tone_hz, SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ,
            &phase_config, &phase_result) != 0 || !phase_result.valid) {
        if (reason != NULL) *reason = "recorded skew phase branch is invalid";
        return -1;
    }
    memset(&policy_input, 0, sizeof(policy_input));
    policy_input.primary_estimate_valid = 1;
    policy_input.measured_skew_samples = phase_result.corrected_skew_samples;
    policy_input.accepted_frames = 10U;
    policy_input.minimum_accepted_frames = 7U;
    policy_input.batch_std_samples = 0.0;
    policy_input.maximum_batch_std_samples =
        ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES;
    policy_input.characterization_maximum_batch_std_samples =
        ADC_CAL_SKEW_CHARACTERIZATION_MAX_STD_SAMPLES;
    policy_input.tolerance_samples = 0.01;
    policy_input.advisory_warning = !fixture->dither_valid ||
        (fixture->dither_valid &&
         phase_result.dither_disagreement_samples > 0.03);
    policy_input.actuator_available = 0;
    if (adc_cal_skew_evaluate_stage_policy(
            &policy_input, &state->skew_policy) != 0) {
        if (reason != NULL) *reason = "recorded skew policy evaluation failed";
        return -2;
    }
    fixture->replay_count += 10U;
    fixture->skew_replays += 10U;
    state->final_relative_skew_samples =
        phase_result.corrected_skew_samples;
    state->final_relative_skew_ps = phase_result.corrected_skew_samples *
        1.0e12 / SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ;
    state->skew_pass = state->skew_policy.pipeline_may_continue != 0;
    state->skew_warning = state->skew_policy.stage_result ==
        ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING;
    if (!state->skew_pass && reason != NULL) *reason = state->skew_policy.reason;
    return state->skew_pass ? 0 : -3;
}

static int fixture_performance_capture(
    void *context,
    double *raw_a,
    double *raw_b,
    double *reference,
    size_t capacity,
    size_t *sample_count,
    const char **reason)
{
    fixture_pipeline_context_t *fixture =
        (fixture_pipeline_context_t *)context;
    if (capacity > 800U) {
        if (reason != NULL) *reason = "recorded performance window is too short";
        return -1;
    }
    for (size_t i = 0U; i < capacity; ++i) {
        raw_a[i] = fixture->aligned_a[i];
        raw_b[i] = fixture->aligned_b[i];
        reference[i] = fixture->reference_window[i];
    }
    ++fixture->replay_count;
    ++fixture->performance_replays;
    if (sample_count != NULL) *sample_count = capacity;
    return 0;
}

static int fixture_pipeline_performance(
    void *context,
    adc_cal_pipeline_state_t *state,
    const char **reason)
{
    fixture_pipeline_context_t *fixture =
        (fixture_pipeline_context_t *)context;
    adc_cal_perf_config_t config;
    adc_cal_perf_default_config(&config);
    config.sample_count = 800U;
    config.sample_rate_hz = SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ;
    config.expected_fundamental_hz = fixture->analysis_tone_hz;
    config.frame_count = 30U;
    config.minimum_valid_frames = 20U;
    config.final_offset_correction = state->offset_correction;
    config.final_gain_correction = state->gain_correction;
    config.nominal_system_gain = state->nominal_system_gain;
    if (adc_cal_perf_run_batch(
            &config, fixture_performance_capture, fixture,
            &fixture->performance) != 0 || !fixture->performance.valid) {
        state->performance_measurement_available = true;
        state->performance_valid = false;
        state->performance_failure_reason = fixture->performance.failure_reason;
        if (reason != NULL) *reason = fixture->performance.failure_reason;
        return -1;
    }
    state->performance_measurement_available = true;
    state->performance_valid = true;
    return 0;
}

static int unit_recorded_fixture_full_pipeline(sim_assert_context_t *ctx)
{
    static fixture_pipeline_context_t fixture;
    adc_cal_pipeline_callbacks_t callbacks;
    adc_cal_pipeline_run_config_t run_config;
    adc_cal_pipeline_state_t state;
    memset(&callbacks, 0, sizeof(callbacks));
    memset(&run_config, 0, sizeof(run_config));
    memset(&state, 0, sizeof(state));
    callbacks.context = &fixture;
    callbacks.prepare = fixture_pipeline_prepare;
    callbacks.run_timing = fixture_pipeline_timing;
    callbacks.run_offset = fixture_pipeline_offset;
    callbacks.run_gain = fixture_pipeline_gain;
    callbacks.run_skew = fixture_pipeline_skew;
    callbacks.run_performance = fixture_pipeline_performance;
    run_config.timing_frame_count = 10U;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_pipeline_run_all(
        &state, &callbacks, &run_config), 0);
    SIM_ASSERT_TRUE(ctx, state.timing_pass);
    SIM_ASSERT_TRUE(ctx, state.timing_mean_correlation >= 0.970f);
    SIM_ASSERT_EQ_INT(ctx, state.offset_result,
        ADC_CAL_PIPELINE_OFFSET_CONVERGED);
    SIM_ASSERT_TRUE(ctx, state.offset_pass);
    SIM_ASSERT_TRUE(ctx, fabs((double)state.offset_verification_error) <= 1.0);
    SIM_ASSERT_TRUE(ctx, state.gain_pass);
    SIM_ASSERT_TRUE(ctx, state.gain_verification_pass);
    SIM_ASSERT_TRUE(ctx, fixture.gain_dither_valid);
    SIM_ASSERT_TRUE(ctx, fabs((double)state.gain_verification_error) <=
        CALIBRATION_GAIN_TOLERANCE);
    SIM_ASSERT_TRUE(ctx, fixture.dither_valid);
    SIM_ASSERT_TRUE(ctx, state.skew_pass);
    SIM_ASSERT_EQ_INT(ctx, state.skew_policy.measurement_validity,
        ADC_CAL_SKEW_MEASUREMENT_VALID);
    SIM_ASSERT_EQ_INT(ctx, state.skew_policy.stability,
        ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_EQ_INT(ctx, state.skew_policy.correction_status,
        ADC_CAL_SKEW_CORRECTION_NOT_APPLICABLE);
    SIM_ASSERT_TRUE(ctx, state.performance_measurement_available);
    SIM_ASSERT_TRUE(ctx, state.performance_valid);
    SIM_ASSERT_EQ_INT(ctx, fixture.performance.frames_valid, 30U);
    SIM_ASSERT_EQ_INT(ctx, state.overall_result, ADC_CAL_PIPELINE_RESULT_PASS);
    SIM_ASSERT_TRUE(ctx, state.output_valid);
    SIM_ASSERT_EQ_INT(ctx, fixture.timing_replays, 10U);
    SIM_ASSERT_TRUE(ctx, fixture.offset_batches >= 2U &&
        fixture.offset_batches <= 20U);
    SIM_ASSERT_TRUE(ctx, fixture.gain_batches >= 2U &&
        fixture.gain_batches <= 20U);
    SIM_ASSERT_EQ_INT(ctx, fixture.skew_replays, 10U);
    SIM_ASSERT_EQ_INT(ctx, fixture.performance_replays, 30U);
    SIM_ASSERT_TRUE(ctx, fixture.replay_count >= 110U);
    if (ctx != NULL && ctx->summary != NULL) {
        fprintf(ctx->summary,
            "Recorded fixture pipeline : PASS\n"
            "  files                  : %s | %s\n"
            "  timing                 : %.6f correlation, %.6f samples, %lu frames\n"
            "  offset                 : %.6f correction, %.6f verification, %lu batches\n"
            "  gain                   : %.9f correction, %.9f normalized, %lu batches, dither %s\n"
            "  dither cross-check     : %s, %.6f samples\n"
            "  open-loop skew         : %.6f samples, %.6f ps, %s\n"
            "  performance            : %lu/%lu valid, SNDR %.3f dB, ENOB %.3f\n"
            "  pipeline               : %s, output usable YES\n",
            ADC_CAL_TEST_WAVEFORM_PATH, ADC_CAL_TEST_CAPTURE_PATH,
            fixture.correlation, fixture.lag_samples,
            (unsigned long)fixture.timing_replays,
            (double)state.offset_correction,
            (double)state.offset_verification_error,
            (unsigned long)fixture.offset_batches,
            (double)state.gain_correction,
            (double)state.final_normalized_gain,
            (unsigned long)fixture.gain_batches,
            fixture.gain_dither_valid ? "VALID" : "INVALID",
            fixture.dither_valid ? "VALID" : "INVALID",
            fixture.dither_skew_samples,
            state.final_relative_skew_samples,
            state.final_relative_skew_ps,
            adc_cal_skew_stage_result_name(state.skew_policy.stage_result),
            (unsigned long)fixture.performance.frames_valid,
            (unsigned long)fixture.performance.frames_attempted,
            (double)fixture.performance.cal_parallel_average_sndr_db,
            (double)fixture.performance.cal_parallel_average_enob,
            adc_cal_pipeline_result_name(state.overall_result));
    }
    return 1;
}

static int unit_rate_mismatched_fixture_full_pipeline(
    sim_assert_context_t *ctx)
{
    static fixture_pipeline_context_t fixture;
    adc_cal_pipeline_callbacks_t callbacks;
    adc_cal_pipeline_run_config_t run_config;
    adc_cal_pipeline_state_t state;
    memset(&fixture, 0, sizeof(fixture));
    memset(&callbacks, 0, sizeof(callbacks));
    memset(&run_config, 0, sizeof(run_config));
    memset(&state, 0, sizeof(state));
    fixture.capture_path = ADC_CAL_TEST_RATE_MISMATCH_CAPTURE_PATH;
    callbacks.context = &fixture;
    callbacks.prepare = fixture_pipeline_prepare;
    callbacks.run_timing = fixture_pipeline_timing;
    callbacks.run_offset = fixture_pipeline_offset;
    callbacks.run_gain = fixture_pipeline_gain;
    callbacks.run_skew = fixture_pipeline_skew;
    callbacks.run_performance = fixture_pipeline_performance;
    run_config.timing_frame_count = 10U;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_pipeline_run_all(
        &state, &callbacks, &run_config), 0);
    SIM_ASSERT_TRUE(ctx, fixture.reference_rate_adapted);
    SIM_ASSERT_EQ_INT(ctx, fixture.reference_tone_bin, 70U);
    SIM_ASSERT_EQ_INT(ctx, fixture.captured_tone_bin, 71U);
    SIM_ASSERT_NEAR(ctx, fixture.selected_rate_ratio,
        fixture.nominal_rate_ratio * 71.0 / 70.0, 1.0e-12);
    SIM_ASSERT_TRUE(ctx, state.timing_mean_correlation >= 0.970f);
    SIM_ASSERT_TRUE(ctx, state.offset_pass);
    SIM_ASSERT_TRUE(ctx, state.gain_pass);
    SIM_ASSERT_TRUE(ctx, fixture.gain_dither_valid);
    SIM_ASSERT_TRUE(ctx, state.skew_pass);
    SIM_ASSERT_TRUE(ctx, state.performance_valid);
    SIM_ASSERT_EQ_INT(ctx, state.overall_result, ADC_CAL_PIPELINE_RESULT_PASS);
    SIM_ASSERT_TRUE(ctx, state.output_valid);
    if (ctx != NULL && ctx->summary != NULL) {
        fprintf(ctx->summary,
            "Rate-mismatched fixture pipeline : PASS\n"
            "  capture                : %s\n"
            "  reference/capture bins : %lu / %lu\n"
            "  nominal/selected ratio : %.9f / %.9f\n"
            "  timing correlation     : %.6f\n"
            "  dither gain/skew       : %s / %s\n"
            "  pipeline               : %s, output usable YES\n",
            fixture.capture_path,
            (unsigned long)fixture.reference_tone_bin,
            (unsigned long)fixture.captured_tone_bin,
            fixture.nominal_rate_ratio,
            fixture.selected_rate_ratio,
            fixture.correlation,
            fixture.gain_dither_valid ? "VALID" : "INVALID",
            fixture.dither_valid ? "VALID" : "INVALID",
            adc_cal_pipeline_result_name(state.overall_result));
    }
    return 1;
}

static int unit_skew_phase_branch_resolver(sim_assert_context_t *ctx)
{
    const double pi = 3.14159265358979323846;
    const double two_pi = 6.28318530717958647692;
    const double frequency_hz = 100.0e6;
    const double sample_rate_hz = SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ;
    const double omega = two_pi * frequency_hz / sample_rate_hz;
    adc_cal_skew_phase_config_t config;
    adc_cal_skew_phase_result_t result;
    adc_cal_skew_result_t dither_result;
    double dither_disagreement = NAN;
    int dither_usable;

    adc_cal_skew_phase_default_config(&config);

    /* Stage 1's rate-matched tone context is accepted; the stale nominal
     * bin-70 frequency is specifically rejected against the bin-71 fit. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_validate_tone_context(
        101.328740e6, 101.314711e6,
        SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ, 800U, 0.75),
        ADC_CAL_SKEW_TONE_CONTEXT_VALID);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_validate_tone_context(
        99.901574e6, 101.314711e6,
        SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ, 800U, 0.75),
        ADC_CAL_SKEW_TONE_CONTEXT_FREQUENCY_MISMATCH);
    SIM_ASSERT_TRUE(ctx, strcmp(adc_cal_skew_tone_context_status_name(
        ADC_CAL_SKEW_TONE_CONTEXT_FREQUENCY_MISMATCH),
        "FREQUENCY_MISMATCH") == 0);

    /* Same-polarity zero, positive, and negative physical skew. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.2, 0.2, frequency_hz, sample_rate_hz, &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, 0.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.selected_polarity, ADC_CAL_SKEW_POLARITY_SAME);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.2, 0.2 + omega * 0.12, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, 0.12, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.2, 0.2 - omega * 0.11, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, -0.11, 1.0e-12);

    /* A common circular/time-origin shift cancels from the paired phase
     * comparison; no independent channel alignment is involved. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.2 + 1.7, 0.2 + 1.7 + omega * 0.12,
        frequency_hz, sample_rate_hz, &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, 0.12, 1.0e-12);

    /* Inverted Channel B with zero, positive, and negative physical skew. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.2, 0.2 + pi, frequency_hz, sample_rate_hz, &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.raw_skew_samples, 7.25, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, 0.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.selected_polarity, ADC_CAL_SKEW_POLARITY_INVERTED);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.2, 0.2 + pi + omega * 0.13, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, 0.13, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.2, 0.2 - pi - omega * 0.09, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, -0.09, 1.0e-12);

    /* Both sides of the +/-pi wrap select the inverted small-skew family. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.0, pi - omega * 0.02, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, -0.02, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.0, -pi + omega * 0.03, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, 0.03, 1.0e-12);

    /* With both families allowed, valid dither chooses the matching branch. */
    config.max_abs_skew_samples = 8.0;
    config.dither_valid = 1;
    config.dither_skew_samples = 0.14;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.0, omega * -7.064, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.selected_polarity, ADC_CAL_SKEW_POLARITY_INVERTED);
    SIM_ASSERT_EQ_INT(ctx, result.selection_reason,
        ADC_CAL_SKEW_BRANCH_REASON_DITHER_AGREEMENT);

    /* Invalid dither is ignored; the configured physical bound is used. */
    adc_cal_skew_phase_default_config(&config);
    config.dither_valid = 0;
    config.dither_skew_samples = -7.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.0, omega * -7.064, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, fabs(result.corrected_skew_samples) < 0.25);
    SIM_ASSERT_EQ_INT(ctx, result.selection_reason,
        ADC_CAL_SKEW_BRANCH_REASON_PHYSICAL_BOUND);

    /* Known polarity metadata has priority over dither and bounds. */
    config.known_polarity = ADC_CAL_SKEW_POLARITY_SAME;
    config.dither_valid = 1;
    config.dither_skew_samples = 0.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.0, omega * -7.064, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.selected_polarity, ADC_CAL_SKEW_POLARITY_SAME);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, -7.064, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.selection_reason,
        ADC_CAL_SKEW_BRANCH_REASON_KNOWN_POLARITY);

    /* Unknown polarity rejects the implausible half-period branch. */
    adc_cal_skew_phase_default_config(&config);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.0, omega * -7.064, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, !result.candidate_within_physical_range[0]);
    SIM_ASSERT_TRUE(ctx, result.candidate_within_physical_range[
        result.selected_candidate]);

    /* Previous corrected frame resolves an otherwise two-family ambiguity. */
    config.max_abs_skew_samples = 8.0;
    config.previous_valid = 1;
    config.previous_skew_samples = 0.18;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.0, omega * -7.06, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.selected_polarity, ADC_CAL_SKEW_POLARITY_INVERTED);
    SIM_ASSERT_EQ_INT(ctx, result.selection_reason,
        ADC_CAL_SKEW_BRANCH_REASON_FRAME_CONSISTENCY);

    /* Regression for raw -7.064 samples and +0.134-sample dither. */
    adc_cal_skew_phase_default_config(&config);
    config.dither_valid = 1;
    config.dither_skew_samples = 0.134413;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.0, omega * -7.064153, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.raw_skew_samples, -7.064153, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, result.selected_polarity, ADC_CAL_SKEW_POLARITY_INVERTED);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, 0.185847, 1.0e-9);
    SIM_ASSERT_TRUE(ctx, result.dither_disagreement_samples < 0.06);
    SIM_ASSERT_TRUE(ctx, fabs(result.corrected_skew_samples) < 0.25);

    /* A finite dither WARNING is not a valid cross-check and must not control
     * the primary polarity branch. This mirrors the hardware frames that
     * reported roughly -0.77 dither samples against +0.186 tone samples. */
    adc_cal_skew_result_reset(&dither_result);
    dither_result.valid = 1;
    dither_result.status = ADC_CAL_SKEW_STATUS_WARNING;
    dither_result.reason = ADC_CAL_SKEW_REASON_OUTSIDE_LINEAR_RANGE;
    dither_result.relative_skew_samples = -0.768105;
    dither_usable = adc_cal_skew_dither_crosscheck_is_usable(
        &dither_result, 0.186103, 0.03, &dither_disagreement);
    /* CASE I reporting semantics: structural/event detection remains valid
     * even though the edge-derived fine-skew cross-check is unusable. */
    SIM_ASSERT_TRUE(ctx, dither_result.valid);
    SIM_ASSERT_EQ_INT(ctx, dither_result.status, ADC_CAL_SKEW_STATUS_WARNING);
    SIM_ASSERT_EQ_INT(ctx, dither_usable, 0);
    SIM_ASSERT_NEAR(ctx, dither_disagreement, 0.954208, 1.0e-9);
    adc_cal_skew_phase_default_config(&config);
    config.dither_valid = dither_usable == 1;
    config.dither_skew_samples = dither_result.relative_skew_samples;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_resolve_tone_phase(
        0.0, omega * -7.064153, frequency_hz, sample_rate_hz,
        &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.corrected_skew_samples, 0.185847, 1.0e-9);
    SIM_ASSERT_EQ_INT(ctx, result.selection_reason,
        ADC_CAL_SKEW_BRANCH_REASON_PHYSICAL_BOUND);

    dither_result.status = ADC_CAL_SKEW_STATUS_PASS;
    dither_result.reason = ADC_CAL_SKEW_REASON_NONE;
    dither_result.relative_skew_samples = 0.18;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_dither_crosscheck_is_usable(
        &dither_result, 0.186103, 0.03, &dither_disagreement), 1);
    SIM_ASSERT_NEAR(ctx, dither_disagreement, 0.006103, 1.0e-9);
    return 1;
}

typedef struct {
    int register_value;
    int initial_register;
    double initial_skew_samples;
    double signed_step_samples;
    double std_samples;
    double measurement_adjustment_samples[16];
    double measurement_std_samples[16];
    uint32_t measurement_override_count;
    uint32_t reads;
    uint32_t writes;
    int write_values[64];
    uint32_t measurements;
    uint32_t characterization_settles;
    uint32_t characterization_probe_settles;
    uint32_t characterization_restore_settles;
    uint32_t readiness_verifications;
    uint32_t sequence_counter;
    uint32_t readiness_sequence;
    uint32_t baseline_sequence;
    uint32_t first_write_sequence;
    int register_at_verification;
    int invalid_measurement;
    int readiness_failure;
    int readback_mismatch;
    uint32_t readback_mismatch_write;
} simulated_skew_actuator_t;

static int simulated_skew_measure(
    void *context, adc_cal_skew_batch_measurement_t *measurement)
{
    simulated_skew_actuator_t *sim = (simulated_skew_actuator_t *)context;
    if (sim == NULL || measurement == NULL) return -1;
    memset(measurement, 0, sizeof(*measurement));
    ++sim->measurements;
    ++sim->sequence_counter;
    if (sim->measurements == 1U)
        sim->baseline_sequence = sim->sequence_counter;
    measurement->valid = !sim->invalid_measurement;
    measurement->skew_samples = sim->initial_skew_samples +
        sim->signed_step_samples *
        (double)(sim->register_value - sim->initial_register) +
        (sim->measurements <= sim->measurement_override_count ?
            sim->measurement_adjustment_samples[sim->measurements - 1U] :
            0.0);
    measurement->batch_std_samples =
        sim->measurements <= sim->measurement_override_count ?
            sim->measurement_std_samples[sim->measurements - 1U] :
            sim->std_samples;
    measurement->accepted_frames = measurement->valid ? 10U : 0U;
    measurement->rejected_frames = measurement->valid ? 0U : 10U;
    measurement->reason = measurement->valid ? "none" :
        "simulated invalid primary estimator";
    return 0;
}

static int simulated_skew_read(void *context, int *value)
{
    simulated_skew_actuator_t *sim = (simulated_skew_actuator_t *)context;
    if (sim == NULL || value == NULL) return -1;
    ++sim->reads;
    *value = sim->register_value;
    return 0;
}

static int simulated_skew_verify_ready(void *context)
{
    simulated_skew_actuator_t *sim = (simulated_skew_actuator_t *)context;
    if (sim == NULL) return -1;
    ++sim->readiness_verifications;
    sim->readiness_sequence = ++sim->sequence_counter;
    sim->register_at_verification = sim->register_value;
    return sim->readiness_failure ? -1 : 0;
}

static int simulated_skew_write(void *context, int value)
{
    simulated_skew_actuator_t *sim = (simulated_skew_actuator_t *)context;
    if (sim == NULL) return -1;
    ++sim->writes;
    if (sim->writes <= sizeof(sim->write_values) /
            sizeof(sim->write_values[0]))
        sim->write_values[sim->writes - 1U] = value;
    ++sim->sequence_counter;
    if (sim->writes == 1U) sim->first_write_sequence = sim->sequence_counter;
    sim->register_value = sim->readback_mismatch ||
        sim->readback_mismatch_write == sim->writes ? value + 1 : value;
    return 0;
}

static int simulated_skew_settle_characterization(
    void *context,
    uint32_t attempt,
    uint32_t probe,
    int baseline_code,
    int active_code,
    int restoring)
{
    simulated_skew_actuator_t *sim = (simulated_skew_actuator_t *)context;
    (void)attempt;
    (void)probe;
    (void)baseline_code;
    (void)active_code;
    if (sim == NULL) return -1;
    ++sim->characterization_settles;
    if (restoring)
        ++sim->characterization_restore_settles;
    else
        ++sim->characterization_probe_settles;
    return 0;
}

enum {
    PREP_DIAG_EVENT_SNAPSHOT_BASE = 100,
    PREP_DIAG_EVENT_MEASURE_PRE = 200,
    PREP_DIAG_EVENT_MEASURE_POST = 201,
    PREP_DIAG_EVENT_JESD_RESET = 300,
    PREP_DIAG_EVENT_ACTUATOR_PREPARE = 301,
    PREP_DIAG_EVENT_INTRINSIC_JESD_RESET = 302,
    PREP_DIAG_EVENT_RESTORE = 303,
    PREP_DIAG_EVENT_WARMUP = 400
};

typedef struct {
    int events[64];
    size_t event_count;
    uint32_t measurements;
    uint32_t measurement_frames;
    uint32_t warmup_captures;
    uint32_t snapshots;
    uint32_t standalone_jesd_resets;
    uint32_t actuator_preparations;
    uint32_t intrinsic_jesd_resets;
    uint32_t correction_writes;
    uint32_t restore_calls;
    uint32_t fullprep_helper_calls;
    uint32_t register_writes[3];
    uint8_t initial_registers[3];
    uint8_t current_registers[3];
    adc_cal_skew_prep_diag_mode_t mode;
    int preparation_intrinsically_resets_jesd;
} simulated_skew_prep_diag_t;

static int simulated_skew_prep_diag_event(
    simulated_skew_prep_diag_t *sim, int event)
{
    if (sim == NULL || sim->event_count >=
        sizeof(sim->events) / sizeof(sim->events[0])) return -1;
    sim->events[sim->event_count++] = event;
    return 0;
}

static int simulated_skew_prep_diag_measure(
    void *context, adc_cal_skew_batch_measurement_t *measurement)
{
    simulated_skew_prep_diag_t *sim =
        (simulated_skew_prep_diag_t *)context;
    const uint32_t call = sim != NULL ? sim->measurements : 0U;
    if (sim == NULL || measurement == NULL || call >= 2U) return -1;
    if (simulated_skew_prep_diag_event(sim,
            call == 0U ? PREP_DIAG_EVENT_MEASURE_PRE :
                PREP_DIAG_EVENT_MEASURE_POST) != 0) return -1;
    memset(measurement, 0, sizeof(*measurement));
    measurement->valid = 1;
    measurement->skew_samples = call == 0U ? -0.057 : -0.144;
    measurement->batch_std_samples = call == 0U ? 0.003 : 0.033;
    measurement->accepted_frames = 10U;
    measurement->reason = "simulated independent diagnostic batch";
    ++sim->measurements;
    sim->measurement_frames += measurement->accepted_frames;
    return 0;
}

static int simulated_skew_prep_diag_snapshot(
    void *context, adc_cal_skew_prep_snapshot_point_t point)
{
    simulated_skew_prep_diag_t *sim =
        (simulated_skew_prep_diag_t *)context;
    if (sim == NULL || point >= ADC_CAL_SKEW_PREP_SNAPSHOT_COUNT)
        return -1;
    if (simulated_skew_prep_diag_event(sim,
            PREP_DIAG_EVENT_SNAPSHOT_BASE + (int)point) != 0) return -1;
    ++sim->snapshots;
    return 0;
}

static int simulated_skew_prep_diag_discard(
    void *context, uint32_t capture_index, uint32_t capture_count)
{
    simulated_skew_prep_diag_t *sim =
        (simulated_skew_prep_diag_t *)context;
    if (sim == NULL || capture_index != sim->warmup_captures + 1U ||
        capture_count != ADC_CAL_SKEW_INITIAL_WARMUP_FRAMES ||
        sim->measurements != 1U) return -1;
    if (simulated_skew_prep_diag_event(sim,
            PREP_DIAG_EVENT_WARMUP) != 0) return -1;
    ++sim->warmup_captures;
    return 0;
}

static int simulated_skew_prep_diag_jesd(void *context)
{
    simulated_skew_prep_diag_t *sim =
        (simulated_skew_prep_diag_t *)context;
    if (simulated_skew_prep_diag_event(
            sim, PREP_DIAG_EVENT_JESD_RESET) != 0) return -1;
    ++sim->standalone_jesd_resets;
    return 0;
}

static int simulated_skew_prep_diag_actuator(void *context)
{
    simulated_skew_prep_diag_t *sim =
        (simulated_skew_prep_diag_t *)context;
    if (simulated_skew_prep_diag_event(
            sim, PREP_DIAG_EVENT_ACTUATOR_PREPARE) != 0) return -1;
    ++sim->actuator_preparations;
    switch (sim->mode) {
    case ADC_CAL_SKEW_PREP_DIAG_CTRL_ONLY:
        sim->current_registers[0] = 0x04U;
        ++sim->register_writes[0];
        ++sim->standalone_jesd_resets;
        break;
    case ADC_CAL_SKEW_PREP_DIAG_ANALOG_ONLY:
        sim->current_registers[1] = 0x60U;
        ++sim->register_writes[1];
        break;
    case ADC_CAL_SKEW_PREP_DIAG_DIGITAL_ONLY:
        sim->current_registers[2] = 0x60U;
        ++sim->register_writes[2];
        break;
    case ADC_CAL_SKEW_PREP_DIAG_ANALOG_DIGITAL:
        sim->current_registers[1] = 0x60U;
        sim->current_registers[2] = 0x60U;
        ++sim->register_writes[1];
        ++sim->register_writes[2];
        break;
    case ADC_CAL_SKEW_PREP_DIAG_ENABLE_AFTER_VALUES:
        sim->current_registers[1] = 0x60U;
        sim->current_registers[2] = 0x60U;
        sim->current_registers[0] = 0x04U;
        ++sim->register_writes[1];
        ++sim->register_writes[2];
        ++sim->register_writes[0];
        ++sim->standalone_jesd_resets;
        break;
    case ADC_CAL_SKEW_PREP_DIAG_ACTUATOR_ONLY:
    case ADC_CAL_SKEW_PREP_DIAG_COMBINED:
        sim->current_registers[0] = 0x04U;
        sim->current_registers[1] = 0x60U;
        sim->current_registers[2] = 0x60U;
        ++sim->register_writes[0];
        ++sim->register_writes[1];
        ++sim->register_writes[2];
        ++sim->fullprep_helper_calls;
        break;
    default:
        return -1;
    }
    if (sim->preparation_intrinsically_resets_jesd) {
        if (simulated_skew_prep_diag_event(
                sim, PREP_DIAG_EVENT_INTRINSIC_JESD_RESET) != 0) return -1;
        ++sim->intrinsic_jesd_resets;
    }
    return 0;
}

static int simulated_skew_prep_diag_restore(void *context)
{
    simulated_skew_prep_diag_t *sim =
        (simulated_skew_prep_diag_t *)context;
    if (simulated_skew_prep_diag_event(
            sim, PREP_DIAG_EVENT_RESTORE) != 0) return -1;
    memcpy(sim->current_registers, sim->initial_registers,
           sizeof(sim->current_registers));
    ++sim->restore_calls;
    return 0;
}

static void simulated_skew_prep_diag_init(
    simulated_skew_prep_diag_t *sim,
    adc_cal_skew_prep_diag_mode_t mode)
{
    memset(sim, 0, sizeof(*sim));
    sim->mode = mode;
    sim->initial_registers[0] = 0x00U;
    sim->initial_registers[1] = 0xC0U;
    sim->initial_registers[2] = 0xC0U;
    memcpy(sim->current_registers, sim->initial_registers,
           sizeof(sim->current_registers));
}

static int unit_skew_measurement_conditioning(sim_assert_context_t *ctx)
{
    adc_cal_skew_conditioning_input_t input;
    adc_cal_skew_conditioning_input_t original;
    adc_cal_skew_conditioning_result_t result;

    memset(&input, 0, sizeof(input));
    input.mode = ADC_CAL_SKEW_CONDITIONING_DIAGNOSTIC_RAW_WINDOW;
    input.production_offset_correction = 23.0;
    input.production_gain_correction = 4.0;
    original = input;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_select_measurement_conditioning(
        &input, &result), 0);
    SIM_ASSERT_TRUE(ctx, !result.permitted);
    SIM_ASSERT_TRUE(ctx, strcmp(result.reason,
        "timing context is invalid") == 0);

    input.timing_context_valid = 1;
    original = input;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_select_measurement_conditioning(
        &input, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.permitted);
    SIM_ASSERT_TRUE(ctx, result.offset_dependency_bypassed);
    SIM_ASSERT_TRUE(ctx, result.gain_dependency_bypassed);
    SIM_ASSERT_NEAR(ctx, result.offset_correction, 0.0, 0.0);
    SIM_ASSERT_NEAR(ctx, result.gain_correction, 1.0, 0.0);
    SIM_ASSERT_TRUE(ctx, memcmp(&input, &original, sizeof(input)) == 0);

    input.mode = ADC_CAL_SKEW_CONDITIONING_PRODUCTION;
    input.offset_result_usable = 0;
    input.gain_result_usable = 0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_select_measurement_conditioning(
        &input, &result), 0);
    SIM_ASSERT_TRUE(ctx, !result.permitted);
    SIM_ASSERT_TRUE(ctx, strcmp(result.reason,
        "offset stage result is not usable") == 0);

    input.offset_result_usable = 1;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_select_measurement_conditioning(
        &input, &result), 0);
    SIM_ASSERT_TRUE(ctx, !result.permitted);
    SIM_ASSERT_TRUE(ctx, strcmp(result.reason,
        "gain stage result is not usable") == 0);

    input.gain_result_usable = 1;
    input.production_offset_correction = -0.25;
    input.production_gain_correction = 1.125;
    original = input;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_select_measurement_conditioning(
        &input, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.permitted);
    SIM_ASSERT_TRUE(ctx, !result.offset_dependency_bypassed);
    SIM_ASSERT_TRUE(ctx, !result.gain_dependency_bypassed);
    SIM_ASSERT_NEAR(ctx, result.offset_correction, -0.25, 0.0);
    SIM_ASSERT_NEAR(ctx, result.gain_correction, 1.125, 0.0);
    SIM_ASSERT_TRUE(ctx, memcmp(&input, &original, sizeof(input)) == 0);
    return 1;
}

static int unit_skew_preparation_diagnostic(sim_assert_context_t *ctx)
{
    adc_cal_skew_loop_config_t config;
    adc_cal_skew_prep_diag_io_t io;
    adc_cal_skew_prep_diag_result_t result;
    simulated_skew_prep_diag_t sim;
    const double expected_threshold = ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES;
    const uint32_t expected_warmups =
        ADC_CAL_SKEW_INITIAL_WARMUP_FRAMES;
    adc_cal_fixed6_parts_t rate_parts;

    adc_cal_skew_loop_default_config(&config);
    memset(&io, 0, sizeof(io));
    io.measure_batch = simulated_skew_prep_diag_measure;
    io.discard_capture = simulated_skew_prep_diag_discard;
    io.capture_snapshot = simulated_skew_prep_diag_snapshot;
    io.jesd_reset_only = simulated_skew_prep_diag_jesd;
    io.actuator_prepare = simulated_skew_prep_diag_actuator;
    io.restore_initial_state = simulated_skew_prep_diag_restore;

    /* Test A: reset is the sole operation between two independent batches. */
    simulated_skew_prep_diag_init(
        &sim, ADC_CAL_SKEW_PREP_DIAG_JESD_ONLY);
    io.context = &sim;
    io.actuator_only_supported = 1;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_preparation_diagnostic(
        ADC_CAL_SKEW_PREP_DIAG_JESD_ONLY, &config, &io, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.completed);
    SIM_ASSERT_EQ_INT(ctx, sim.standalone_jesd_resets, 1U);
    SIM_ASSERT_EQ_INT(ctx, sim.actuator_preparations, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.warmup_captures, expected_warmups);
    SIM_ASSERT_EQ_INT(ctx, sim.snapshots, ADC_CAL_SKEW_PREP_SNAPSHOT_COUNT);
    SIM_ASSERT_EQ_INT(ctx, sim.measurements, 2U);
    SIM_ASSERT_EQ_INT(ctx, sim.measurement_frames, 20U);
    SIM_ASSERT_EQ_INT(ctx, result.pre_measurement.accepted_frames, 10U);
    SIM_ASSERT_EQ_INT(ctx, result.post_measurement.accepted_frames, 10U);
    SIM_ASSERT_NEAR(ctx, result.pre_measurement.skew_samples, -0.057, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, result.post_measurement.skew_samples, -0.144, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.correction_write_calls, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.correction_writes, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.events[0],
        PREP_DIAG_EVENT_SNAPSHOT_BASE +
            ADC_CAL_SKEW_PREP_SNAPSHOT_BEFORE_BASELINE);
    SIM_ASSERT_EQ_INT(ctx, sim.events[1], PREP_DIAG_EVENT_MEASURE_PRE);
    SIM_ASSERT_EQ_INT(ctx, sim.events[2],
        PREP_DIAG_EVENT_SNAPSHOT_BASE +
            ADC_CAL_SKEW_PREP_SNAPSHOT_AFTER_BASELINE);
    SIM_ASSERT_EQ_INT(ctx, sim.events[3],
        PREP_DIAG_EVENT_SNAPSHOT_BASE +
            ADC_CAL_SKEW_PREP_SNAPSHOT_BEFORE_OPERATION);
    SIM_ASSERT_EQ_INT(ctx, sim.events[4], PREP_DIAG_EVENT_JESD_RESET);
    SIM_ASSERT_EQ_INT(ctx, sim.events[5],
        PREP_DIAG_EVENT_SNAPSHOT_BASE +
            ADC_CAL_SKEW_PREP_SNAPSHOT_AFTER_OPERATION);
    SIM_ASSERT_EQ_INT(ctx, sim.events[6U + expected_warmups],
        PREP_DIAG_EVENT_SNAPSHOT_BASE +
            ADC_CAL_SKEW_PREP_SNAPSHOT_AFTER_WARMUP);
    SIM_ASSERT_EQ_INT(ctx, sim.events[7U + expected_warmups],
        PREP_DIAG_EVENT_MEASURE_POST);
    SIM_ASSERT_EQ_INT(ctx, sim.events[8U + expected_warmups],
        PREP_DIAG_EVENT_SNAPSHOT_BASE +
            ADC_CAL_SKEW_PREP_SNAPSHOT_AFTER_FINAL_MEASUREMENT);

    /* A separable actuator backend receives no standalone reset. */
    simulated_skew_prep_diag_init(
        &sim, ADC_CAL_SKEW_PREP_DIAG_ACTUATOR_ONLY);
    io.context = &sim;
    io.actuator_only_supported = 1;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_preparation_diagnostic(
        ADC_CAL_SKEW_PREP_DIAG_ACTUATOR_ONLY, &config, &io, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.completed);
    SIM_ASSERT_EQ_INT(ctx, sim.actuator_preparations, 1U);
    SIM_ASSERT_EQ_INT(ctx, sim.standalone_jesd_resets, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.intrinsic_jesd_resets, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.warmup_captures, expected_warmups);
    SIM_ASSERT_EQ_INT(ctx, result.correction_write_calls, 0U);
    SIM_ASSERT_TRUE(ctx, result.restore_attempted);
    SIM_ASSERT_TRUE(ctx, result.restore_succeeded);
    SIM_ASSERT_EQ_INT(ctx, sim.restore_calls, 1U);
    SIM_ASSERT_TRUE(ctx, memcmp(sim.current_registers,
        sim.initial_registers, sizeof(sim.current_registers)) == 0);

    /* The AD9695-style dependency is reported before any callback or snapshot. */
    simulated_skew_prep_diag_init(
        &sim, ADC_CAL_SKEW_PREP_DIAG_ACTUATOR_ONLY);
    io.context = &sim;
    io.actuator_only_supported = 0;
    io.actuator_only_unsupported_reason =
        "simulated intrinsic receiver reset dependency";
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_preparation_diagnostic(
        ADC_CAL_SKEW_PREP_DIAG_ACTUATOR_ONLY, &config, &io, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.unsupported);
    SIM_ASSERT_TRUE(ctx, !result.completed);
    SIM_ASSERT_EQ_INT(ctx, sim.event_count, 0U);
    SIM_ASSERT_EQ_INT(ctx, result.correction_write_calls, 0U);

    /* Combined production preparation preserves actuator-then-intrinsic-reset
     * callback order and still never exposes a correction callback. */
    simulated_skew_prep_diag_init(
        &sim, ADC_CAL_SKEW_PREP_DIAG_COMBINED);
    sim.preparation_intrinsically_resets_jesd = 1;
    io.context = &sim;
    io.actuator_only_supported = 0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_preparation_diagnostic(
        ADC_CAL_SKEW_PREP_DIAG_COMBINED, &config, &io, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.completed);
    SIM_ASSERT_EQ_INT(ctx, sim.actuator_preparations, 1U);
    SIM_ASSERT_EQ_INT(ctx, sim.standalone_jesd_resets, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.intrinsic_jesd_resets, 1U);
    SIM_ASSERT_EQ_INT(ctx, sim.events[4], PREP_DIAG_EVENT_ACTUATOR_PREPARE);
    SIM_ASSERT_EQ_INT(ctx, sim.events[5],
        PREP_DIAG_EVENT_INTRINSIC_JESD_RESET);
    SIM_ASSERT_EQ_INT(ctx, sim.warmup_captures, expected_warmups);
    SIM_ASSERT_EQ_INT(ctx, sim.measurement_frames, 20U);
    SIM_ASSERT_EQ_INT(ctx, result.correction_write_calls, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.fullprep_helper_calls, 1U);
    SIM_ASSERT_TRUE(ctx, result.restore_succeeded);

    /* Register-isolation modes touch only their named delay registers, use
     * independent pre/post batches, exclude warm-ups, and restore entry state. */
    {
        static const adc_cal_skew_prep_diag_mode_t modes[] = {
            ADC_CAL_SKEW_PREP_DIAG_CTRL_ONLY,
            ADC_CAL_SKEW_PREP_DIAG_ANALOG_ONLY,
            ADC_CAL_SKEW_PREP_DIAG_DIGITAL_ONLY,
            ADC_CAL_SKEW_PREP_DIAG_ANALOG_DIGITAL,
            ADC_CAL_SKEW_PREP_DIAG_ENABLE_AFTER_VALUES
        };
        static const unsigned int expected_masks[] = {
            0x1U, 0x2U, 0x4U, 0x6U, 0x7U
        };
        for (size_t test = 0U;
             test < sizeof(modes) / sizeof(modes[0]); ++test) {
            unsigned int observed_mask = 0U;
            simulated_skew_prep_diag_init(&sim, modes[test]);
            io.context = &sim;
            io.actuator_only_supported = 0;
            SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_preparation_diagnostic(
                modes[test], &config, &io, &result), 0);
            for (size_t reg = 0U; reg < 3U; ++reg) {
                if (sim.register_writes[reg] > 0U)
                    observed_mask |= 1U << reg;
            }
            SIM_ASSERT_EQ_INT(ctx, observed_mask, expected_masks[test]);
            SIM_ASSERT_EQ_INT(ctx, sim.measurements, 2U);
            SIM_ASSERT_EQ_INT(ctx, sim.measurement_frames, 20U);
            SIM_ASSERT_EQ_INT(ctx, sim.warmup_captures, expected_warmups);
            SIM_ASSERT_EQ_INT(ctx, sim.restore_calls, 1U);
            SIM_ASSERT_TRUE(ctx, result.restore_attempted);
            SIM_ASSERT_TRUE(ctx, result.restore_succeeded);
            SIM_ASSERT_TRUE(ctx, memcmp(sim.current_registers,
                sim.initial_registers, sizeof(sim.current_registers)) == 0);
            SIM_ASSERT_EQ_INT(ctx, result.correction_write_calls, 0U);
            SIM_ASSERT_EQ_INT(ctx, sim.correction_writes, 0U);
            SIM_ASSERT_EQ_INT(ctx, sim.standalone_jesd_resets,
                modes[test] == ADC_CAL_SKEW_PREP_DIAG_CTRL_ONLY ||
                modes[test] == ADC_CAL_SKEW_PREP_DIAG_ENABLE_AFTER_VALUES ?
                    1U : 0U);
        }
    }

    /* Diagnostic sequencing cannot mutate the production safety constants. */
    SIM_ASSERT_NEAR(ctx, config.skew_maximum_batch_std_samples,
        expected_threshold, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, config.skew_initial_warmup_frames,
        expected_warmups);
    SIM_ASSERT_NEAR(ctx, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, 1.3e9, 0.5);
    SIM_ASSERT_NEAR(ctx, SIM_DEFAULT_DAC_SAMPLE_RATE_HZ, 2.6e9, 0.5);
    SIM_ASSERT_NEAR(ctx,
        SIM_DEFAULT_DAC_SAMPLE_RATE_HZ / SIM_DEFAULT_ADC_SAMPLE_RATE_HZ,
        2.0, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_fixed6_parts(
        SIM_DEFAULT_DAC_SAMPLE_RATE_HZ, &rate_parts), 0);
    SIM_ASSERT_TRUE(ctx, !rate_parts.negative);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.billions, 2U);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.units_below_billion, 600000000U);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.millionths, 0U);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_fixed6_parts(10.123456, &rate_parts), 0);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.units_below_billion, 10U);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.millionths, 123456U);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_fixed6_parts(20.799726, &rate_parts), 0);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.units_below_billion, 20U);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.millionths, 799726U);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_fixed6_parts(200.000001, &rate_parts), 0);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.units_below_billion, 200U);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.millionths, 1U);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_fixed6_parts(19.230769, &rate_parts), 0);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.units_below_billion, 19U);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.millionths, 230769U);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_fixed6_parts(-52.884194, &rate_parts), 0);
    SIM_ASSERT_TRUE(ctx, rate_parts.negative);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.units_below_billion, 52U);
    SIM_ASSERT_EQ_INT(ctx, rate_parts.millionths, 884194U);
    return 1;
}

static int unit_skew_closed_loop_controller(sim_assert_context_t *ctx)
{
    adc_cal_skew_loop_config_t config;
    adc_cal_skew_loop_io_t io;
    adc_cal_skew_loop_result_t result;
    simulated_skew_actuator_t sim;
    int requested = 0;
    int applied = 0;
    int target = 0;
    int saturated = 0;

    adc_cal_skew_loop_default_config(&config);
    SIM_ASSERT_EQ_INT(ctx, config.skew_initial_warmup_frames,
        ADC_CAL_SKEW_INITIAL_WARMUP_FRAMES);
    SIM_ASSERT_NEAR(ctx, config.skew_maximum_batch_std_samples,
        ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES, 1.0e-12);
    SIM_ASSERT_NEAR(ctx,
        config.skew_characterization_maximum_batch_std_samples,
        ADC_CAL_SKEW_CHARACTERIZATION_MAX_STD_SAMPLES, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES,
        0.025, 1.0e-12);
    SIM_ASSERT_NEAR(ctx, ADC_CAL_SKEW_CHARACTERIZATION_MAX_STD_SAMPLES,
        0.035, 1.0e-12);
    SIM_ASSERT_NEAR(ctx,
        ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES * 1.0e12 /
            SIM_DEFAULT_ADC_SAMPLE_RATE_HZ,
        19.2307692307692, 1.0e-9);
    config.skew_closed_loop_enable = 1;
    config.skew_register_min = 0;
    config.skew_register_max = 63;
    config.skew_actuator_step_samples = 0.02;
    config.skew_actuator_polarity = 1;
    config.skew_max_steps_per_iteration = 4;
    config.skew_tolerance_samples = 0.011;
    config.skew_required_consecutive_passes = 2U;
    config.skew_max_iterations = 10U;
    memset(&io, 0, sizeof(io));
    io.measure_batch = simulated_skew_measure;
    io.verify_actuator_ready = simulated_skew_verify_ready;
    io.read_register = simulated_skew_read;
    io.write_register = simulated_skew_write;
    io.settle_characterization_state =
        simulated_skew_settle_characterization;

    /* Correct sign and bounded update: positive B-A skew with a positive
     * actuator polarity must reduce the Channel-B register. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_plan_update(
        0.186, 20, &config, &requested, &applied, &target, &saturated), 0);
    SIM_ASSERT_TRUE(ctx, requested < 0);
    SIM_ASSERT_EQ_INT(ctx, applied, -4);
    SIM_ASSERT_EQ_INT(ctx, target, 16);
    SIM_ASSERT_TRUE(ctx, !saturated);
    /* Do not stall just outside tolerance when one characterized actuator
     * step is predicted to finish the correction. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_plan_update(
        0.015, 20, &config, &requested, &applied, &target, &saturated), 0);
    SIM_ASSERT_EQ_INT(ctx, requested, -1);
    SIM_ASSERT_EQ_INT(ctx, applied, -1);
    SIM_ASSERT_EQ_INT(ctx, target, 19);
    SIM_ASSERT_TRUE(ctx, !saturated);
    config.skew_actuator_polarity = -1;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_plan_update(
        0.186, 20, &config, &requested, &applied, &target, &saturated), 0);
    SIM_ASSERT_TRUE(ctx, applied > 0);
    config.skew_actuator_polarity = 1;

    /* CASE A: a significant one-code response characterizes immediately and
     * enters the unchanged controller without trying larger amplitudes. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_CONVERGED);
    SIM_ASSERT_TRUE(ctx, result.characterization_valid);
    SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
        ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    SIM_ASSERT_TRUE(ctx, !result.characterization_cautious);
    SIM_ASSERT_EQ_INT(ctx, result.actuator_polarity, 1);
    SIM_ASSERT_NEAR(ctx, result.observed_step_samples, 0.02, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_attempt_count, 1U);
    SIM_ASSERT_EQ_INT(ctx, result.successful_probe_amplitude, 1);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_write_count, 2U);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_restore_write_count, 2U);
    SIM_ASSERT_TRUE(ctx, result.characterization_baseline_restored);
    SIM_ASSERT_EQ_INT(ctx, sim.characterization_probe_settles, 2U);
    SIM_ASSERT_EQ_INT(ctx, sim.characterization_restore_settles, 2U);
    SIM_ASSERT_TRUE(ctx, fabs(result.final_skew_samples) <=
        config.skew_tolerance_samples);
    SIM_ASSERT_TRUE(ctx, result.consecutive_passes >= 2U);
    SIM_ASSERT_TRUE(ctx, result.correction_applied);
    SIM_ASSERT_TRUE(ctx, sim.writes > 0U);
    SIM_ASSERT_TRUE(ctx, result.final_register < result.initial_register);
    SIM_ASSERT_EQ_INT(ctx, sim.readiness_verifications, 1U);
    SIM_ASSERT_TRUE(ctx, result.actuator_ready_verified);
    SIM_ASSERT_EQ_INT(ctx, sim.register_at_verification, sim.initial_register);
    SIM_ASSERT_TRUE(ctx, sim.readiness_sequence < sim.baseline_sequence);
    SIM_ASSERT_TRUE(ctx, sim.baseline_sequence < sim.first_write_sequence);
    SIM_ASSERT_NEAR(ctx, result.initial_skew_samples, 0.186, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.accepted_frames, sim.measurements * 10U);

    /* Exact stable board baseline: NORMAL dispatch must reach the first
     * readback-verified probe from neutral code 24. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 24;
    sim.initial_skew_samples = -0.238981;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 1U;
    sim.measurement_std_samples[0] = 0.016051;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
        ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    SIM_ASSERT_TRUE(ctx, !result.characterization_cautious);
    SIM_ASSERT_TRUE(ctx, result.characterization_attempted);
    SIM_ASSERT_EQ_INT(ctx, result.initial_register, 24);
    SIM_ASSERT_TRUE(ctx, sim.writes > 0U);
    SIM_ASSERT_TRUE(ctx, strstr(result.failure_reason,
        "marginal baseline requires") == NULL);

    /* The preferred threshold remains inclusive and dispatches NORMAL. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 24;
    sim.initial_skew_samples = 0.005;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 1U;
    sim.measurement_std_samples[0] = ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
        ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_TRUE(ctx, !result.characterization_cautious);
    SIM_ASSERT_TRUE(ctx, result.characterization_attempted);
    SIM_ASSERT_TRUE(ctx, sim.writes > 0U);

    /* A marginal probe batch cannot overwrite a stable baseline's NORMAL
     * policy. The probe remains characterization-eligible through 0.035. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 24;
    sim.initial_skew_samples = 0.005;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 3U;
    sim.measurement_std_samples[0] = 0.016051;
    sim.measurement_std_samples[1] = 0.025659;
    sim.measurement_std_samples[2] = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
        ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_EQ_INT(ctx, result.first_probe_stability,
        ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_EQ_INT(ctx, result.repeat_probe_stability,
        ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_TRUE(ctx, !result.characterization_cautious);
    SIM_ASSERT_TRUE(ctx, result.characterization_valid);
    SIM_ASSERT_TRUE(ctx, sim.writes > 0U);

    /* Exact marginal board batch: CAUTIOUS policy remains attached to the
     * initial baseline even when both later probe batches are stable. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 24;
    sim.initial_skew_samples = 0.005;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 3U;
    sim.measurement_std_samples[0] = 0.025659;
    sim.measurement_std_samples[1] = 0.001;
    sim.measurement_std_samples[2] = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
        ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_EQ_INT(ctx, result.first_probe_stability,
        ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_EQ_INT(ctx, result.repeat_probe_stability,
        ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_TRUE(ctx, result.characterization_cautious);
    SIM_ASSERT_TRUE(ctx, result.characterization_attempted);
    SIM_ASSERT_TRUE(ctx, result.characterization_valid);
    SIM_ASSERT_TRUE(ctx, sim.writes > 0U);

    /* Both observed -160 ps marginal board baselines must reach the first
     * cautious probe; neither spread is an entry blocker. */
    {
        static const double board_medians[] = {-0.207771, -0.207762};
        static const double board_std_samples[] = {
            20.257258e-12 * SIM_DEFAULT_ADC_SAMPLE_RATE_HZ,
            22.044405e-12 * SIM_DEFAULT_ADC_SAMPLE_RATE_HZ
        };
        for (size_t board_case = 0U; board_case < 2U; ++board_case) {
            memset(&sim, 0, sizeof(sim));
            sim.register_value = sim.initial_register = 24;
            sim.initial_skew_samples = board_medians[board_case];
            sim.signed_step_samples = 0.02;
            sim.std_samples = 0.001;
            sim.measurement_override_count = 3U;
            sim.measurement_std_samples[0] = board_std_samples[board_case];
            sim.measurement_std_samples[1] = 0.001;
            sim.measurement_std_samples[2] = 0.001;
            io.context = &sim;
            SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
                &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
            SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
                ADC_CAL_SKEW_STABILITY_MARGINAL);
            SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
            SIM_ASSERT_TRUE(ctx, result.characterization_cautious);
            SIM_ASSERT_TRUE(ctx, result.characterization_attempted);
            SIM_ASSERT_TRUE(ctx, sim.writes > 0U);
        }
    }

    /* The upper marginal boundary also reaches cautious characterization. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 24;
    sim.initial_skew_samples = 0.005;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 3U;
    sim.measurement_std_samples[0] = 0.0349;
    sim.measurement_std_samples[1] = 0.001;
    sim.measurement_std_samples[2] = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
        ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_TRUE(ctx, result.characterization_cautious);
    SIM_ASSERT_TRUE(ctx, result.characterization_attempted);
    SIM_ASSERT_TRUE(ctx, sim.writes > 0U);

    /* Production leaves actuator polarity unknown until the mandatory +1
     * characterization.  Verify that a negative hardware response is learned
     * and then used to move the register in the convergent direction. */
    config.skew_actuator_polarity = 0;
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = -0.02;
    sim.std_samples = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_CONVERGED);
    SIM_ASSERT_EQ_INT(ctx, result.actuator_polarity, -1);
    SIM_ASSERT_TRUE(ctx, result.final_register > result.initial_register);
    config.skew_actuator_polarity = 1;

    /* Read-only readiness verification precedes the authoritative baseline;
     * an invalid baseline cannot authorize any correction write. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.030;
    sim.invalid_measurement = 1;
    io.context = &sim;
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result) != 0);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_FAILED);
    SIM_ASSERT_EQ_INT(ctx, sim.readiness_verifications, 1U);
    SIM_ASSERT_EQ_INT(ctx, sim.writes, 0U);

    /* Stage 4 requires a readiness verifier and never initializes or warms
     * the actuator itself. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    io.context = &sim;
    io.verify_actuator_ready = NULL;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status,
        ADC_CAL_SKEW_LOOP_ACTUATOR_UNAVAILABLE);
    SIM_ASSERT_EQ_INT(ctx, sim.measurements, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.reads, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.writes, 0U);
    io.verify_actuator_ready = simulated_skew_verify_ready;

    /* Missing register access is rejected before the skew baseline. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    io.context = &sim;
    io.read_register = NULL;
    io.write_register = NULL;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status,
        ADC_CAL_SKEW_LOOP_ACTUATOR_UNAVAILABLE);
    SIM_ASSERT_TRUE(ctx, isnan(result.initial_skew_samples));
    SIM_ASSERT_EQ_INT(ctx, sim.measurements, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.readiness_verifications, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.reads, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.writes, 0U);
    SIM_ASSERT_TRUE(ctx, strcmp(result.failure_reason,
        "ACTUATOR_UNAVAILABLE") == 0);
    io.read_register = simulated_skew_read;
    io.write_register = simulated_skew_write;

    /* Unexpected mode/readback state fails safely before measurement and
     * cannot trigger a destructive late reinitialization. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.030;
    sim.readiness_failure = 1;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status,
        ADC_CAL_SKEW_LOOP_ACTUATOR_UNAVAILABLE);
    SIM_ASSERT_EQ_INT(ctx, sim.measurements, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.readiness_verifications, 1U);
    SIM_ASSERT_EQ_INT(ctx, sim.reads, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.writes, 0U);
    SIM_ASSERT_TRUE(ctx, strcmp(result.failure_reason,
        "ACTUATOR_NOT_READY") == 0);

    /* A hard-valid 0.040-sample baseline is HIGH-NOISE, not invalid. It may
     * reach the cautious minimum probe; significance decides continuation. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 3U;
    sim.measurement_std_samples[0] = 0.040;
    sim.measurement_std_samples[1] = 0.001;
    sim.measurement_std_samples[2] = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, sim.readiness_verifications, 1U);
    SIM_ASSERT_NEAR(ctx, result.initial_skew_samples, 0.186, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
        ADC_CAL_SKEW_STABILITY_HIGH_NOISE);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    SIM_ASSERT_TRUE(ctx, result.characterization_cautious);
    SIM_ASSERT_TRUE(ctx, result.characterization_attempted);
    SIM_ASSERT_TRUE(ctx, result.characterization_valid);
    SIM_ASSERT_TRUE(ctx, sim.writes > 0U);

    /* Marginal intermediate batches remain hard-valid controller inputs and
     * do not abort correction solely because they exceed 0.025. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 4U;
    sim.measurement_std_samples[0] = 0.001;
    sim.measurement_std_samples[1] = 0.001;
    sim.measurement_std_samples[2] = 0.001;
    sim.measurement_std_samples[3] = 0.030;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_CONVERGED);
    SIM_ASSERT_TRUE(ctx, result.correction_applied);

    /* The same is true for a HIGH-NOISE intermediate batch. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 4U;
    sim.measurement_std_samples[0] = 0.001;
    sim.measurement_std_samples[1] = 0.001;
    sim.measurement_std_samples[2] = 0.001;
    sim.measurement_std_samples[3] = 0.040;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_CONVERGED);
    SIM_ASSERT_TRUE(ctx, result.correction_applied);

    /* Marginal baseline plus a clearly significant, repeatable +1 response
     * may proceed into the unchanged controller. Later batches are strict. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 3U;
    for (uint32_t i = 0U; i < sim.measurement_override_count; ++i)
        sim.measurement_std_samples[i] = 0.027;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
        ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    SIM_ASSERT_TRUE(ctx, result.characterization_cautious);
    SIM_ASSERT_TRUE(ctx, result.characterization_valid);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_CONVERGED);
    SIM_ASSERT_TRUE(ctx, result.characterization_minimum_response_samples <
        result.actuator_step_samples);

    /* CASE B: one code is below 1.5 times the combined standard error, while
     * two codes are significant. Each amplitude is restored independently,
     * the response is normalized per code, and the controller starts. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.050;
    sim.signed_step_samples = 0.01;
    sim.std_samples = 0.027;
    io.context = &sim;
    config.skew_actuator_step_samples = 0.0;
    config.skew_actuator_polarity = 0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.characterization_valid);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_attempt_count, 2U);
    SIM_ASSERT_EQ_INT(ctx, result.successful_probe_amplitude, 2);
    SIM_ASSERT_NEAR(ctx, result.observed_step_samples, 0.01, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_write_count, 4U);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_restore_write_count, 4U);
    SIM_ASSERT_EQ_INT(ctx, sim.write_values[0], 21);
    SIM_ASSERT_EQ_INT(ctx, sim.write_values[1], 20);
    SIM_ASSERT_EQ_INT(ctx, sim.write_values[2], 21);
    SIM_ASSERT_EQ_INT(ctx, sim.write_values[3], 20);
    SIM_ASSERT_EQ_INT(ctx, sim.write_values[4], 22);
    SIM_ASSERT_TRUE(ctx, result.iterations_completed > 0U);

    /* CASE C: amplitudes one and two fail significance, four succeeds. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.030;
    sim.signed_step_samples = 0.006;
    sim.std_samples = 0.027;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.characterization_valid);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_attempt_count, 3U);
    SIM_ASSERT_EQ_INT(ctx, result.successful_probe_amplitude, 4);
    SIM_ASSERT_NEAR(ctx, result.observed_step_samples, 0.006, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_write_count, 6U);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_restore_write_count, 6U);
    SIM_ASSERT_TRUE(ctx, result.iterations_completed > 0U);

    /* CASE D: all bounded amplitudes fail significance and leave the exact
     * characterization-entry code restored without controller correction. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.050;
    sim.signed_step_samples = 0.003;
    sim.std_samples = 0.027;
    io.context = &sim;
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result) != 0);
    SIM_ASSERT_TRUE(ctx, !result.characterization_valid);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_attempt_count, 3U);
    SIM_ASSERT_EQ_INT(ctx, result.successful_probe_amplitude, 0);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_write_count, 6U);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_restore_write_count, 6U);
    SIM_ASSERT_EQ_INT(ctx, sim.register_value, sim.initial_register);
    SIM_ASSERT_TRUE(ctx, result.characterization_baseline_restored);
    SIM_ASSERT_TRUE(ctx, !result.correction_applied);
    SIM_ASSERT_TRUE(ctx, strstr(result.failure_reason,
        "measurement noise") != NULL);

    /* CASE F: opposite physical signs in each repeated pair reject every
     * amplitude and leave polarity unknown. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.050;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 7U;
    sim.measurement_adjustment_samples[2] = -0.04;
    sim.measurement_adjustment_samples[4] = -0.08;
    sim.measurement_adjustment_samples[6] = -0.16;
    io.context = &sim;
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result) != 0);
    SIM_ASSERT_TRUE(ctx, !result.characterization_valid);
    SIM_ASSERT_EQ_INT(ctx, result.actuator_polarity, 0);
    SIM_ASSERT_TRUE(ctx, strstr(result.failure_reason, "repeatable") != NULL);
    SIM_ASSERT_TRUE(ctx, !result.correction_applied);

    /* CASE G: significant same-sign pairs outside the unchanged 35 percent
     * repeatability tolerance escalate and ultimately fail. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.050;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 7U;
    sim.measurement_adjustment_samples[2] = 0.02;
    sim.measurement_adjustment_samples[4] = 0.04;
    sim.measurement_adjustment_samples[6] = 0.08;
    io.context = &sim;
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result) != 0);
    SIM_ASSERT_TRUE(ctx, !result.characterization_valid);
    SIM_ASSERT_TRUE(ctx, strstr(result.failure_reason, "repeatable") != NULL);
    SIM_ASSERT_EQ_INT(ctx, sim.register_value, sim.initial_register);

    /* CASE H: the preferred positive direction is unsafe at the upper limit,
     * so probing reverses safely and normalizes the response sign per code. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = config.skew_register_max;
    sim.initial_skew_samples = 0.050;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.characterization_valid);
    SIM_ASSERT_EQ_INT(ctx, result.probe_signed_steps[0], -1);
    SIM_ASSERT_EQ_INT(ctx, result.probe_requested_code[0],
        config.skew_register_max - 1);
    SIM_ASSERT_TRUE(ctx, result.probe_response_samples[0][0] < 0.0);
    SIM_ASSERT_NEAR(ctx, result.observed_step_samples, 0.02, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.actuator_polarity, 1);

    config.skew_actuator_step_samples = 0.02;
    config.skew_actuator_polarity = 1;

    /* CASE E: a probe readback mismatch attempts and verifies restoration,
     * then fails safely without controller entry. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.027;
    sim.readback_mismatch_write = 1U;
    io.context = &sim;
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result) != 0);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_FAILED);
    SIM_ASSERT_TRUE(ctx, result.characterization_attempted);
    SIM_ASSERT_TRUE(ctx, !result.correction_applied);
    SIM_ASSERT_EQ_INT(ctx, sim.register_value, sim.initial_register);
    SIM_ASSERT_EQ_INT(ctx, result.characterization_restore_write_count, 1U);
    SIM_ASSERT_TRUE(ctx, result.characterization_baseline_restored);

    /* A stable baseline remains a valid measurement when its first probe
     * fails readback; the failure belongs to actuator characterization. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 24;
    sim.initial_skew_samples = -0.238981;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.016051;
    sim.readback_mismatch_write = 1U;
    io.context = &sim;
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result) != 0);
    SIM_ASSERT_TRUE(ctx, result.baseline_measurement_valid);
    SIM_ASSERT_EQ_INT(ctx, result.baseline_stability,
        ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_TRUE(ctx, result.characterization_attempted);
    SIM_ASSERT_TRUE(ctx, !result.characterization_valid);
    SIM_ASSERT_TRUE(ctx, !result.correction_applied);
    SIM_ASSERT_TRUE(ctx, strstr(result.failure_reason, "readback") != NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_loop_stage_result(&result),
        ADC_CAL_SKEW_STAGE_RESULT_ACTUATOR_READBACK_FAILED);

    /* The characterization ceiling is not a convergence threshold: a
     * marginal independent verification cannot contribute a consecutive
     * pass or produce convergence. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.005;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 4U;
    sim.measurement_std_samples[0] = 0.001;
    sim.measurement_std_samples[1] = 0.001;
    sim.measurement_std_samples[2] = 0.001;
    sim.measurement_std_samples[3] = 0.030;
    sim.std_samples = 0.030;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_NOT_CONVERGED);
    SIM_ASSERT_TRUE(ctx, !result.correction_applied);
    SIM_ASSERT_TRUE(ctx, result.consecutive_passes <
        config.skew_required_consecutive_passes);
    SIM_ASSERT_EQ_INT(ctx, result.latest_measurement_stability,
        ADC_CAL_SKEW_STABILITY_MARGINAL);

    /* HIGH-NOISE independent verifications may continue, but cannot count
     * toward the strict two-batch final convergence requirement. */
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.005;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    sim.measurement_override_count = 4U;
    sim.measurement_std_samples[0] = 0.001;
    sim.measurement_std_samples[1] = 0.001;
    sim.measurement_std_samples[2] = 0.001;
    sim.measurement_std_samples[3] = 0.040;
    sim.std_samples = 0.040;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_NOT_CONVERGED);
    SIM_ASSERT_TRUE(ctx, !result.correction_applied);
    SIM_ASSERT_TRUE(ctx, result.consecutive_passes <
        config.skew_required_consecutive_passes);
    SIM_ASSERT_EQ_INT(ctx, result.latest_measurement_stability,
        ADC_CAL_SKEW_STABILITY_HIGH_NOISE);

    /* Measurement-only mode never reads or writes an actuator. */
    config.skew_closed_loop_enable = 0;
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status,
        ADC_CAL_SKEW_LOOP_MEASUREMENT_ONLY);
    SIM_ASSERT_EQ_INT(ctx, sim.reads, 0U);
    SIM_ASSERT_EQ_INT(ctx, sim.writes, 0U);

    /* Saturation is explicit when the correction points below register min. */
    config.skew_closed_loop_enable = 1;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_plan_update(
        0.186, 0, &config, &requested, &applied, &target, &saturated), 0);
    SIM_ASSERT_EQ_INT(ctx, applied, 0);
    SIM_ASSERT_TRUE(ctx, saturated);
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 0;
    sim.initial_skew_samples = 0.186;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_LOOP_SATURATED);

    /* One passing batch is insufficient when two consecutive passes are
     * configured and the iteration limit permits only one. */
    config.skew_max_iterations = 1U;
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.005;
    sim.signed_step_samples = 0.02;
    sim.std_samples = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status,
        ADC_CAL_SKEW_LOOP_NOT_CONVERGED);
    SIM_ASSERT_EQ_INT(ctx, result.consecutive_passes, 1U);

    /* Coarse resolution is reported honestly instead of claiming convergence. */
    config.skew_max_iterations = 10U;
    config.skew_actuator_step_samples = 0.04;
    config.skew_tolerance_samples = 0.01;
    memset(&sim, 0, sizeof(sim));
    sim.register_value = sim.initial_register = 20;
    sim.initial_skew_samples = 0.026;
    sim.signed_step_samples = 0.04;
    sim.std_samples = 0.001;
    io.context = &sim;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_run_closed_loop(
        &config, &io, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.status,
        ADC_CAL_SKEW_LOOP_NOT_CONVERGED);
    SIM_ASSERT_TRUE(ctx, strstr(result.failure_reason, "resolution") != NULL);
    SIM_ASSERT_NEAR(ctx, result.final_skew_samples, 0.026, 1.0e-12);
    return 1;
}

static int unit_skew_stage_policy(sim_assert_context_t *ctx)
{
    adc_cal_skew_stage_policy_input_t input;
    adc_cal_skew_stage_policy_result_t result;

    memset(&input, 0, sizeof(input));
    SIM_ASSERT_TRUE(ctx, strcmp(adc_cal_skew_stage_result_name(
        ADC_CAL_SKEW_STAGE_RESULT_CHARACTERIZATION_FAILED),
        "FAIL - ACTUATOR CHARACTERIZATION") == 0);
    input.measurement_required = 1;
    input.primary_estimate_valid = 1;
    input.measured_skew_samples = 0.005;
    input.accepted_frames = 10U;
    input.minimum_accepted_frames = 3U;
    input.batch_std_samples = 0.001;
    input.maximum_batch_std_samples =
        ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES;
    input.characterization_maximum_batch_std_samples =
        ADC_CAL_SKEW_CHARACTERIZATION_MAX_STD_SAMPLES;
    input.tolerance_samples = 0.01;

    /* Valid in-tolerance open-loop measurement. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.measurement_validity,
        ADC_CAL_SKEW_MEASUREMENT_VALID);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_EQ_INT(ctx, result.tolerance_status,
        ADC_CAL_SKEW_TOLERANCE_IN);
    SIM_ASSERT_EQ_INT(ctx, result.correction_status,
        ADC_CAL_SKEW_CORRECTION_NOT_APPLICABLE);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_PASS);
    SIM_ASSERT_TRUE(ctx, result.pipeline_may_continue && result.output_usable);

    /* The stability limit is inclusive.  Board-observed spreads remain
     * stable through the configured boundary. */
    input.batch_std_samples = 0.019;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_STABLE);
    input.batch_std_samples = 0.0224;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_STABLE);
    input.batch_std_samples = 0.0249;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_STABLE);
    input.batch_std_samples = ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_STABLE);

    /* Above the preferred threshold but below the separate ceiling is
     * explicitly marginal, not stable or unsafe. */
    input.batch_std_samples = ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES + 0.0001;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.measurement_validity,
        ADC_CAL_SKEW_MEASUREMENT_VALID);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING);
    SIM_ASSERT_TRUE(ctx, result.pipeline_may_continue && result.output_usable);
    input.batch_std_samples = 0.001;

    /* Valid out-of-tolerance open-loop measurement remains usable. */
    input.measured_skew_samples = 0.1858;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.tolerance_status,
        ADC_CAL_SKEW_TOLERANCE_OUT);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING);
    SIM_ASSERT_TRUE(ctx, result.pipeline_may_continue && result.output_usable);

    /* An unavailable/invalid optional dither cross-check is advisory only. */
    input.measured_skew_samples = 0.005;
    input.advisory_warning = 1;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.measurement_validity,
        ADC_CAL_SKEW_MEASUREMENT_VALID);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING);
    SIM_ASSERT_TRUE(ctx, result.pipeline_may_continue && result.output_usable);

    /* A marginal open-loop measurement remains valid and usable even when
     * no correction actuator is present. */
    input.advisory_warning = 0;
    input.measured_skew_samples = 0.1858;
    input.batch_std_samples = 0.03;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_EQ_INT(ctx, result.measurement_validity,
        ADC_CAL_SKEW_MEASUREMENT_VALID);
    SIM_ASSERT_EQ_INT(ctx, result.actuator_status,
        ADC_CAL_SKEW_ACTUATOR_UNAVAILABLE);
    SIM_ASSERT_EQ_INT(ctx, result.correction_status,
        ADC_CAL_SKEW_CORRECTION_NOT_APPLICABLE);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING);
    SIM_ASSERT_TRUE(ctx, result.pipeline_may_continue && result.output_usable);

    /* Three-state characterization gate boundaries. */
    input.actuator_available = 1;
    input.batch_std_samples = 0.020;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    SIM_ASSERT_TRUE(ctx, !result.characterization_cautious);
    input.batch_std_samples = ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_STABLE);
    input.batch_std_samples = 0.027;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    SIM_ASSERT_TRUE(ctx, result.characterization_cautious);
    input.batch_std_samples = 0.0349;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    input.batch_std_samples =
        ADC_CAL_SKEW_CHARACTERIZATION_MAX_STD_SAMPLES + 0.0001;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stability,
        ADC_CAL_SKEW_STABILITY_HIGH_NOISE);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    SIM_ASSERT_TRUE(ctx, result.characterization_cautious);
    SIM_ASSERT_TRUE(ctx, strstr(result.reason, "high-noise") != NULL);
    input.actuator_available = 0;

    /* No estimate and insufficient frames are invalid. */
    input.batch_std_samples = 0.001;
    input.primary_estimate_valid = 0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_INVALID);
    SIM_ASSERT_TRUE(ctx, strstr(result.reason, "mandatory") != NULL);
    input.primary_estimate_valid = 1;
    input.accepted_frames = 2U;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_INVALID);

    /* Marginal spread cannot bypass invalid tone population or a polarity
     * branch-family change. */
    input.accepted_frames = 10U;
    input.batch_std_samples = 0.030;
    input.actuator_available = 1;
    input.primary_estimate_valid = 0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.measurement_validity,
        ADC_CAL_SKEW_MEASUREMENT_INVALID);
    SIM_ASSERT_TRUE(ctx, !result.characterization_allowed);
    input.primary_estimate_valid = 1;
    input.polarity_branch_changes = 1U;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_INVALID);
    SIM_ASSERT_EQ_INT(ctx, result.stability,
        ADC_CAL_SKEW_STABILITY_INVALID);
    SIM_ASSERT_EQ_INT(ctx, result.measurement_validity,
        ADC_CAL_SKEW_MEASUREMENT_INVALID);
    SIM_ASSERT_TRUE(ctx, !result.characterization_allowed);

    /* A measurement-only open-loop stage is explicitly optional: loss of all
     * skew estimates is visible but does not invalidate calibrated output. */
    input.measurement_required = 0;
    input.primary_estimate_valid = 0;
    input.accepted_frames = 0U;
    input.polarity_branch_changes = 0U;
    input.actuator_available = 0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING);
    SIM_ASSERT_TRUE(ctx, result.pipeline_may_continue && result.output_usable);
    SIM_ASSERT_TRUE(ctx, strstr(result.reason, "optional") != NULL);

    /* Closed-loop convergence semantics remain distinct. */
    input.measurement_required = 1;
    input.primary_estimate_valid = 1;
    input.accepted_frames = 10U;
    input.polarity_branch_changes = 0U;
    input.measured_skew_samples = 0.005;
    input.batch_std_samples = 0.001;
    input.actuator_available = 1;
    input.correction_applied = 1;
    input.correction_converged = 1;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.correction_status,
        ADC_CAL_SKEW_CORRECTION_CONVERGED);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_PASS);
    input.correction_converged = 0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_CORRECTION_NOT_CONVERGED);
    input.actuator_saturated = 1;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.correction_status,
        ADC_CAL_SKEW_CORRECTION_SATURATED);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_CORRECTION_SATURATED);

    /* Post-sequencing board baselines are valid marginal measurements and
     * are eligible only for cautious characterization. */
    memset(&input, 0, sizeof(input));
    input.measurement_required = 1;
    input.primary_estimate_valid = 1;
    input.accepted_frames = 10U;
    input.minimum_accepted_frames = 3U;
    input.maximum_batch_std_samples =
        ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES;
    input.characterization_maximum_batch_std_samples =
        ADC_CAL_SKEW_CHARACTERIZATION_MAX_STD_SAMPLES;
    input.tolerance_samples = 0.01;
    input.actuator_available = 1;
    input.measured_skew_samples = -0.207771;
    input.batch_std_samples = 20.257258e-12 *
        SIM_DEFAULT_ADC_SAMPLE_RATE_HZ;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_NEAR(ctx, input.batch_std_samples, 0.0263344354, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    SIM_ASSERT_TRUE(ctx, result.characterization_cautious);
    input.measured_skew_samples = -0.207762;
    input.batch_std_samples = 22.044405e-12 *
        SIM_DEFAULT_ADC_SAMPLE_RATE_HZ;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_NEAR(ctx, input.batch_std_samples, 0.0286577265, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_MARGINAL);
    SIM_ASSERT_TRUE(ctx, result.characterization_allowed);
    SIM_ASSERT_TRUE(ctx, result.characterization_cautious);

    /* Exact board regression: stable 0.1858 samples, 0.74 ps spread,
     * 10/10 inverted frames, no branch changes, and no actuator. */
    memset(&input, 0, sizeof(input));
    input.primary_estimate_valid = 1;
    input.measured_skew_samples = 0.185824;
    input.accepted_frames = 10U;
    input.minimum_accepted_frames = 3U;
    input.batch_std_samples =
        0.740224e-12 * SIM_HISTORICAL_ADC_SAMPLE_RATE_HZ;
    input.maximum_batch_std_samples =
        ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES;
    input.characterization_maximum_batch_std_samples =
        ADC_CAL_SKEW_CHARACTERIZATION_MAX_STD_SAMPLES;
    input.polarity_branch_changes = 0U;
    input.tolerance_samples = 0.01;
    input.actuator_available = 0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_evaluate_stage_policy(
        &input, &result), 0);
    SIM_ASSERT_EQ_INT(ctx, result.measurement_validity,
        ADC_CAL_SKEW_MEASUREMENT_VALID);
    SIM_ASSERT_EQ_INT(ctx, result.stability, ADC_CAL_SKEW_STABILITY_STABLE);
    SIM_ASSERT_EQ_INT(ctx, result.tolerance_status,
        ADC_CAL_SKEW_TOLERANCE_OUT);
    SIM_ASSERT_EQ_INT(ctx, result.actuator_status,
        ADC_CAL_SKEW_ACTUATOR_UNAVAILABLE);
    SIM_ASSERT_EQ_INT(ctx, result.correction_status,
        ADC_CAL_SKEW_CORRECTION_NOT_APPLICABLE);
    SIM_ASSERT_EQ_INT(ctx, result.stage_result,
        ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING);
    SIM_ASSERT_TRUE(ctx, result.pipeline_may_continue && result.output_usable);
    return 1;
}

static int unit_skew_estimator_direct(sim_assert_context_t *ctx)
{
    double template_samples[256];
    double a[256];
    double b[256];
    adc_cal_skew_config_t config;
    adc_cal_skew_result_t result;
    uint32_t rng = 77U;
    double tone_skew = NAN;
    int16_t pair_a[1016];
    int16_t pair_b[1016];
    double mapped_a[800];
    double mapped_b[800];
    const double sample_rate_hz = SIM_DEFAULT_ADC_SAMPLE_RATE_HZ;

    for (size_t i = 0U; i < 1016U; ++i) {
        pair_a[i] = (int16_t)(i % 700U);
        pair_b[i] = (int16_t)(pair_a[i] + 100);
    }
    /* Regression for the reported 464-sample lag: the 800-sample window
     * crosses the DMA boundary, but the same circular interpolation is used
     * for both channels and their relative relationship is preserved. */
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_map_paired_window_i16(
        pair_a, pair_b, 1016U, 108U, 800U, 0.0, 463.983703,
        mapped_a, mapped_b), 0);
    for (size_t i = 0U; i < 800U; ++i) {
        SIM_ASSERT_NEAR(ctx, mapped_b[i] - mapped_a[i], 100.0, 1.0e-9);
    }

    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_from_tone_phases(
        0.25, 0.25 + 6.28318530717958647692 * 100.0e6 /
            sample_rate_hz * 0.12,
        100.0e6, sample_rate_hz, &tone_skew), 0);
    SIM_ASSERT_NEAR(ctx, tone_skew, 0.12, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_from_tone_phases(
        0.25,
        0.25 + 3.14159265358979323846 +
            6.28318530717958647692 * 100.0e6 / sample_rate_hz * 0.12,
        100.0e6, sample_rate_hz, &tone_skew), 0);
    SIM_ASSERT_NEAR(ctx, tone_skew, 0.12, 1.0e-12);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_from_tone_phases(
        3.10, -3.10, 100.0e6, sample_rate_hz, &tone_skew), 0);
    SIM_ASSERT_TRUE(ctx, tone_skew > 0.0 && tone_skew < 0.25);
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_from_tone_phases(
        0.0, 0.0, 0.0, sample_rate_hz, &tone_skew) != 0);

    adc_cal_skew_default_config(&config);
    SIM_ASSERT_NEAR(ctx, config.sample_rate_hz,
                    SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, 1.0);
    config.sample_rate_hz = SIM_DEFAULT_ADC_SAMPLE_RATE_HZ;
    config.minimum_events = 3U;
    config.max_linear_skew_samples = 0.25;

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, 0);
    make_dither_residual(template_samples, a, 256U, 1.0, 0.0, 0.0, NULL);
    make_dither_residual(template_samples, b, 256U, 1.0, 0.0, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.relative_skew_samples, 0.0, 0.01);

    make_dither_residual(template_samples, b, 256U, 1.0, 0.10, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.relative_skew_samples, 0.10, 0.02);

    make_dither_residual(template_samples, b, 256U, 1.0, -0.10, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.relative_skew_samples, -0.10, 0.02);

    make_dither_residual(template_samples, b, 256U, 1.0, 0.18, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.relative_skew_samples > 0.15);

    make_dither_residual(template_samples, b, 256U, 1.0, -0.18, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.relative_skew_samples < -0.15);

    make_dither_residual(template_samples, b, 256U, 1.0, 0.40, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.valid);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_STATUS_WARNING);
    SIM_ASSERT_EQ_INT(ctx, result.reason, ADC_CAL_SKEW_REASON_OUTSIDE_LINEAR_RANGE);

    make_dither_residual(template_samples, b, 256U, 1.0, 0.10, 0.0, NULL);
    b[128] = NAN;
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result) != 0);

    make_dither_template(template_samples, 256U, 200U, 4U, 10.0, 0);
    make_dither_residual(template_samples, a, 256U, 1.0, 0.0, 0.0, NULL);
    make_dither_residual(template_samples, b, 256U, 1.0, 0.0, 0.0, NULL);
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result) != 0);

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, 1);
    make_dither_residual(template_samples, a, 256U, 1.0, 0.0, 0.0, NULL);
    make_dither_residual(template_samples, b, 256U, 1.0, 0.0, 0.0, NULL);
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result) != 0);

    make_dither_template(template_samples, 256U, 32U, 4U, 10.0, 0);
    make_dither_residual(template_samples, a, 256U, 1.0, 0.0, 0.08, &rng);
    make_dither_residual(template_samples, b, 256U, 1.0, 0.08, 0.08, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.relative_skew_samples, 0.08, 0.05);

    make_dither_residual(template_samples, a, 256U, 1.0, 0.0, 0.0, NULL);
    make_dither_residual(template_samples, b, 256U, 1.0, 0.12, 0.0, NULL);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(b, a, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_NEAR(ctx, result.relative_skew_samples, -0.12, 0.02);

    /* Opposing rising/falling shifts remain a valid but explicit warning. */
    make_dither_residual(template_samples, a, 256U, 1.0, 0.0, 0.0, NULL);
    for (size_t i = 0U; i < 256U; ++i) {
        const double previous = i > 0U ? template_samples[i - 1U] :
            template_samples[i];
        const double next = i + 1U < 256U ? template_samples[i + 1U] :
            template_samples[i];
        const double derivative = 0.5 * (next - previous);
        const double polarity = template_samples[i] < 0.0 ? -1.0 : 1.0;
        const double canonical_derivative = polarity * derivative;
        const double edge_skew = canonical_derivative >= 0.0 ? 0.40 : -0.40;
        b[i] = template_samples[i] + edge_skew * derivative;
    }
    SIM_ASSERT_EQ_INT(ctx, adc_cal_skew_estimate_from_residuals(
        a, b, template_samples, 256U, &config, &result), 0);
    SIM_ASSERT_TRUE(ctx, result.valid);
    SIM_ASSERT_EQ_INT(ctx, result.status, ADC_CAL_SKEW_STATUS_WARNING);
    SIM_ASSERT_EQ_INT(ctx, result.reason,
        ADC_CAL_SKEW_REASON_EDGE_DISAGREEMENT);
    return 1;
}

typedef struct {
    uint32_t seed;
    uint32_t frame;
    bool fail_all;
    double noise;
    bool invert_b;
} perf_capture_test_context_t;

static int perf_capture_test(
    void *context,
    double *raw_a,
    double *raw_b,
    double *reference,
    size_t capacity,
    size_t *sample_count,
    const char **reason)
{
    perf_capture_test_context_t *ctx = (perf_capture_test_context_t *)context;
    if (ctx == NULL || ctx->fail_all) {
        if (reason != NULL) *reason = "forced invalid performance frame";
        return -1;
    }
    ++ctx->frame;
    make_perf_tone(reference, capacity, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, 130000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, &ctx->seed);
    make_perf_tone(raw_a, capacity, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, 130000000.0,
                   1000.0, 4.0, ctx->noise, 0.0, 0.0, 0.0, 0.0, false,
                   &ctx->seed);
    make_perf_tone(raw_b, capacity, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, 130000000.0,
                   1000.0, 5.0, ctx->noise, 0.0, 0.0, 0.0, 0.0, false,
                   &ctx->seed);
    if (ctx->invert_b) {
        for (size_t i = 0U; i < capacity; ++i) raw_b[i] = -raw_b[i];
    }
    if (sample_count != NULL) *sample_count = capacity;
    return 0;
}

static int unit_performance_estimator_direct(sim_assert_context_t *ctx)
{
    double clean[800];
    double noisy[800];
    double harmonic2[800];
    double harmonic3[800];
    double spur[800];
    double ref[800];
    double raw_b[800];
    adc_cal_perf_config_t config;
    adc_cal_perf_spectral_metrics_t clean_metrics;
    adc_cal_perf_spectral_metrics_t noisy_metrics;
    adc_cal_perf_spectral_metrics_t distorted_metrics;
    adc_cal_perf_frame_result_t frame;
    adc_cal_perf_batch_result_t batch;
    perf_capture_test_context_t capture_ctx;
    uint32_t rng = 42U;
    uint32_t rng_repeat = 42U;
    double repeat[800];
    static double baseline_a[800];
    static double baseline_b[800];

    /* FFT frequency axes use the authoritative ADC rate. */
    SIM_ASSERT_NEAR(ctx,
        156.0 * SIM_DEFAULT_ADC_SAMPLE_RATE_HZ / 1016.0,
        199606299.2125984, 1.0e-6);

    adc_cal_perf_default_config(&config);
    config.sample_count = 800U;
    config.sample_rate_hz = SIM_DEFAULT_ADC_SAMPLE_RATE_HZ;
    config.expected_fundamental_hz = 130000000.0;

    make_perf_tone(clean, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(clean, 800U, config.sample_rate_hz, &clean_metrics), 0);
    SIM_ASSERT_TRUE(ctx, clean_metrics.sndr_db > 60.0f);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(
        clean, 800U, SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, &distorted_metrics), 0);
    SIM_ASSERT_NEAR(ctx, distorted_metrics.signal_hz, 130000000.0, 1.0);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 132000000.0,
                   1000.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics), 0);
    SIM_ASSERT_TRUE(ctx, noisy_metrics.signal_hz == noisy_metrics.signal_hz);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics), 0);
    SIM_ASSERT_TRUE(ctx, clean_metrics.sndr_db > noisy_metrics.sndr_db);

    make_perf_tone(harmonic2, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(harmonic2, 800U, config.sample_rate_hz, &distorted_metrics), 0);
    SIM_ASSERT_TRUE(ctx, distorted_metrics.thd_db > clean_metrics.thd_db);

    make_perf_tone(harmonic3, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(harmonic3, 800U, config.sample_rate_hz, &distorted_metrics), 0);
    SIM_ASSERT_TRUE(ctx, distorted_metrics.thd_db > clean_metrics.thd_db);

    make_perf_tone(spur, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.0, 230000000.0, 100.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(spur, 800U, config.sample_rate_hz, &distorted_metrics), 0);
    SIM_ASSERT_TRUE(ctx, distorted_metrics.sfdr_db < clean_metrics.sfdr_db);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics), 0);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 130000000.0,
                   9000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics), 0);
    SIM_ASSERT_TRUE(ctx, noisy_metrics.sndr_db < clean_metrics.sndr_db);

    memset(noisy, 0, sizeof(noisy));
    SIM_ASSERT_TRUE(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics) != 0);
    SIM_ASSERT_TRUE(ctx, adc_cal_perf_analyze_record(clean, 0U, config.sample_rate_hz, &noisy_metrics) != 0);

    make_perf_tone(ref, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    memcpy(raw_b, clean, sizeof(raw_b));
    for (size_t i = 0U; i < 800U; ++i) raw_b[i] += 80.0 * sin((double)i);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_frame(clean, raw_b, clean, raw_b, ref, &config, 1U, &frame), 0);
    SIM_ASSERT_TRUE(ctx, frame.raw_a.sndr_db > frame.raw_b.sndr_db);

    /* Parallel A/B matching uses the frozen polarity and never interleaves. */
    for (size_t i = 0U; i < 800U; ++i) raw_b[i] = -clean[i];
    config.channel_polarity[0] = 1.0;
    config.channel_polarity[1] = -1.0;
    config.initial_relative_skew_samples = 0.174;
    config.initial_relative_skew_ps = 120.0;
    config.final_relative_skew_samples = -0.0041;
    config.final_relative_skew_ps = -2.83;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_frame(
        clean, raw_b, clean, clean, ref, &config, 2U, &frame), 0);
    SIM_ASSERT_TRUE(ctx, frame.parallel_average_available);
    SIM_ASSERT_TRUE(ctx, !frame.interleaved_metrics_available);
    SIM_ASSERT_TRUE(ctx, frame.raw_matching.correlation > 0.9999f);
    SIM_ASSERT_TRUE(ctx, frame.raw_matching.waveform_rmse_codes < 1.0e-4f);
    SIM_ASSERT_TRUE(ctx, frame.cal_matching.waveform_rmse_codes < 1.0e-4f);
    SIM_ASSERT_TRUE(ctx, frame.cal_a_reference_correlation > 0.9999f);
    SIM_ASSERT_TRUE(ctx, frame.cal_b_reference_correlation > 0.9999f);
    SIM_ASSERT_TRUE(ctx, frame.cal_a_reference_rmse_codes < 1.0e-4f);
    SIM_ASSERT_TRUE(ctx, frame.cal_b_reference_rmse_codes < 1.0e-4f);
    SIM_ASSERT_NEAR(ctx, frame.raw_matching.relative_skew_ps, 120.0, 1.0e-9);
    SIM_ASSERT_NEAR(ctx, frame.cal_matching.relative_skew_ps, -2.83, 1.0e-9);
    SIM_ASSERT_NEAR(ctx, frame.raw_parallel_average.signal_hz,
                    frame.raw_a.signal_hz, 1.0);
    SIM_ASSERT_NEAR(ctx, frame.sample_rate_hz, config.sample_rate_hz, 1.0);

    /* Offset and gain mismatch are measured after polarity normalization. */
    for (size_t i = 0U; i < 800U; ++i) raw_b[i] = -(1.10 * clean[i] + 7.0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_frame(
        clean, raw_b, clean, clean, ref, &config, 3U, &frame), 0);
    SIM_ASSERT_NEAR(ctx, frame.raw_matching.offset_mismatch_codes, -7.0, 0.05);
    SIM_ASSERT_NEAR(ctx, frame.raw_matching.gain_ratio_b_over_a, 1.10, 1.0e-4);
    SIM_ASSERT_NEAR(ctx, frame.raw_matching.gain_mismatch, 0.10, 1.0e-4);

    config.channel_polarity[1] = 1.0;
    for (size_t i = 0U; i < 800U; ++i) noisy[i] = clean[i] + 10.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_frame(noisy, noisy, clean, clean, ref, &config, 2U, &frame), 0);
    SIM_ASSERT_TRUE(ctx, frame.rmse < 1.0f);

    memset(&capture_ctx, 0, sizeof(capture_ctx));
    capture_ctx.seed = 99U;
    capture_ctx.noise = 2.0;
    config.frame_count = 5U;
    config.minimum_valid_frames = 3U;
    config.final_offset_correction = -4.0;
    config.final_gain_correction = 1.0;
    config.nominal_system_gain = 1.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_run_batch(&config, perf_capture_test, &capture_ctx, &batch), 0);
    SIM_ASSERT_TRUE(ctx, batch.valid);

    /* Stage 5 must reuse the frozen timing polarity and selected channel. */
    memset(&capture_ctx, 0, sizeof(capture_ctx));
    capture_ctx.seed = 99U;
    capture_ctx.invert_b = true;
    config.frame_count = 1U;
    config.minimum_valid_frames = 1U;
    config.final_offset_correction = 5.0;
    config.canonical_channel = 1;
    config.channel_polarity[0] = 1.0;
    config.channel_polarity[1] = -1.0;
    config.final_relative_skew_samples = -0.003685;
    config.final_relative_skew_ps = -2.541726;
    config.frame_results = &frame;
    config.frame_result_capacity = 1U;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_run_batch(
        &config, perf_capture_test, &capture_ctx, &batch), 0);
    SIM_ASSERT_TRUE(ctx, frame.correlation_before_polarity < -0.99f);
    SIM_ASSERT_TRUE(ctx, frame.correlation > 0.99f);
    SIM_ASSERT_TRUE(ctx, frame.rmse < frame.rmse_before_polarity);
    SIM_ASSERT_TRUE(ctx, !frame.raw_cal_buffers_identical);
    SIM_ASSERT_NEAR(ctx, frame.cal_matching.relative_skew_ps,
                    -2.541726, 1.0e-9);

    /* A genuine separate baseline may match when no correction is active. */
    memset(&capture_ctx, 0, sizeof(capture_ctx));
    capture_ctx.seed = 99U;
    make_perf_tone(baseline_a, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, false,
                   &capture_ctx.seed);
    make_perf_tone(baseline_b, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, false,
                   &capture_ctx.seed);
    config.final_offset_correction = 0.0;
    config.canonical_channel = 1;
    config.channel_polarity[0] = 1.0;
    config.channel_polarity[1] = 1.0;
    config.baseline_a = baseline_a;
    config.baseline_b = baseline_b;
    config.baseline_frame_stride = 800U;
    config.baseline_frame_count = 1U;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_run_batch(
        &config, perf_capture_test, &capture_ctx, &batch), 0);
    SIM_ASSERT_TRUE(ctx, frame.raw_cal_a_identical);
    SIM_ASSERT_TRUE(ctx, frame.raw_cal_b_identical);
    SIM_ASSERT_TRUE(ctx, frame.raw_a_address != frame.cal_a_address);
    SIM_ASSERT_TRUE(ctx, frame.raw_b_address != frame.cal_b_address);
    SIM_ASSERT_TRUE(ctx, frame.parallel_average_available);
    SIM_ASSERT_TRUE(ctx, !frame.interleaved_metrics_available);
    SIM_ASSERT_NEAR(ctx, frame.raw_parallel_average.signal_hz,
                    frame.raw_a.signal_hz, 1.0);
    SIM_ASSERT_NEAR(ctx, frame.sample_rate_hz,
                    SIM_DEFAULT_ADC_SAMPLE_RATE_HZ, 1.0);

    /* Applying a correction changes values, but never shifts post-DMA data. */
    memset(&capture_ctx, 0, sizeof(capture_ctx));
    capture_ctx.seed = 99U;
    config.final_offset_correction = -4.0;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_run_batch(
        &config, perf_capture_test, &capture_ctx, &batch), 0);
    SIM_ASSERT_TRUE(ctx, !frame.raw_cal_a_identical);
    SIM_ASSERT_TRUE(ctx, !frame.raw_cal_b_identical);
    SIM_ASSERT_TRUE(ctx, frame.correlation > 0.99f);
    SIM_ASSERT_NEAR(ctx, frame.cal_parallel_average.signal_hz,
                    frame.raw_parallel_average.signal_hz, 1.0);

    capture_ctx.fail_all = true;
    config.minimum_valid_frames = 1U;
    SIM_ASSERT_TRUE(ctx, adc_cal_perf_run_batch(&config, perf_capture_test, &capture_ctx, &batch) != 0);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, false, &rng_repeat);
    rng_repeat = 42U;
    make_perf_tone(repeat, 800U, config.sample_rate_hz, 130000000.0,
                   1000.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, false, &rng_repeat);
    SIM_ASSERT_EQ_INT(ctx, memcmp(noisy, repeat, sizeof(noisy)), 0);
    return 1;
}

static int unit_boundary_conditions(sim_assert_context_t *ctx)
{
    calibration_config_t config;
    calibration_state_t state;
    int16_t reference_min[2] = {SIM_ADC_MIN_CODE, SIM_ADC_MAX_CODE};
    int16_t adc_min[2] = {SIM_ADC_MIN_CODE, SIM_ADC_MAX_CODE};
    int16_t empty_sample = 0;
    timing_alignment_result_t lag;
    float fractional = 0.0f;
    int16_t reference[128];
    int16_t signal[128];
    const uint32_t generation_before = reference_buffer_generation();

    calibration_default_config(&config);
    SIM_ASSERT_EQ_INT(ctx, calibration_init(&state, &config), CALIBRATION_OK);
    SIM_ASSERT_EQ_INT(ctx, calibration_analyze_frame(&state, &empty_sample, &empty_sample, 0U), CALIBRATION_ERR_SAMPLE_COUNT);
    SIM_ASSERT_EQ_INT(ctx, calibration_analyze_frame(&state, adc_min, reference_min, 2U), CALIBRATION_OK);
    SIM_ASSERT_NEAR(ctx, state.metrics.measured_gain, 1.0, 0.001);

    SIM_ASSERT_EQ_INT(ctx, calibration_set_software_gain_correction(CALIBRATION_GAIN_CORRECTION_MIN), 0);
    SIM_ASSERT_NEAR(ctx, calibration_software_gain_correction(), CALIBRATION_GAIN_CORRECTION_MIN, 0.0);
    SIM_ASSERT_EQ_INT(ctx, calibration_set_software_gain_correction(CALIBRATION_GAIN_CORRECTION_MAX), 0);
    SIM_ASSERT_NEAR(ctx, calibration_software_gain_correction(), CALIBRATION_GAIN_CORRECTION_MAX, 0.0);
    SIM_ASSERT_EQ_INT(ctx, calibration_set_software_offset_correction(4096.0f), 0);
    SIM_ASSERT_NEAR(ctx, calibration_software_offset_correction(), 4096.0, 0.0);
    SIM_ASSERT_EQ_INT(ctx, calibration_set_software_offset_correction(-4096.0f), 0);
    SIM_ASSERT_NEAR(ctx, calibration_software_offset_correction(), -4096.0, 0.0);

    for (size_t i = 0U; i < 128U; ++i) {
        const double x = (double)i;
        reference[i] = (int16_t)lrint(
            900.0 * sin(2.0 * M_PI * 7.0 * x / 128.0) +
            450.0 * sin(2.0 * M_PI * 13.0 * x / 128.0 + 0.3));
    }
    make_fractional_signal(reference, signal, 128U, 0.499);
    SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(reference, signal, 128U, &lag), 0);
    (void)timing_estimate_fractional_lag(reference, signal, 128U, lag.lag_samples, &fractional);
    SIM_ASSERT_TRUE(ctx, fabs((double)fractional) <= 0.5);
    make_fractional_signal(reference, signal, 128U, -0.499);
    SIM_ASSERT_EQ_INT(ctx, timing_find_circular_lag(reference, signal, 128U, &lag), 0);
    (void)timing_estimate_fractional_lag(reference, signal, 128U, lag.lag_samples, &fractional);
    SIM_ASSERT_TRUE(ctx, fabs((double)fractional) <= 0.5);

    reference_buffer_clear();
    SIM_ASSERT_TRUE(ctx, reference_buffer_generation() != generation_before);
    sim_record_known_gap(
        ctx,
        "reference_buffer_generation wrap cannot be forced without production test seam",
        __FILE__,
        __LINE__);

    calibration_all_loops_reset();
    return 1;
}

static void run_unit_tests(sim_assert_context_t *ctx)
{
    (void)unit_dma_round_trip(ctx);
    (void)unit_timing_lag(ctx);
    (void)unit_fractional_lag(ctx);
    (void)unit_flat_reference_rejection(ctx);
    (void)unit_calibration_estimates(ctx);
    (void)unit_reference_buffer(ctx);
    (void)unit_host_invalidation_hooks(ctx);
    (void)unit_nonfinite_config_rejection(ctx);
    (void)unit_finite_config_boundaries(ctx);
    (void)unit_dither_estimator_direct(ctx);
    (void)unit_dither_detection_validation(ctx);
    (void)unit_dither_joint_alignment(ctx);
    (void)unit_dither_correlation_coordinates(ctx);
    (void)unit_txt_waveform_timing_and_dither_diagnostics(ctx);
    (void)unit_recorded_fixture_full_pipeline(ctx);
    (void)unit_rate_mismatched_fixture_full_pipeline(ctx);
    (void)unit_skew_phase_branch_resolver(ctx);
    (void)unit_skew_measurement_conditioning(ctx);
    (void)unit_skew_preparation_diagnostic(ctx);
    (void)unit_skew_closed_loop_controller(ctx);
    (void)unit_skew_stage_policy(ctx);
    (void)unit_skew_estimator_direct(ctx);
    (void)unit_performance_estimator_direct(ctx);
    (void)unit_boundary_conditions(ctx);
}

static int analyze_performance(
    const int16_t *samples,
    size_t count,
    double sample_rate_hz,
    sim_perf_metrics_t *metrics)
{
    const size_t max_bins = SIM_ADC_CHANNEL_SAMPLES / 2U + 1U;
    static double power[SIM_ADC_CHANNEL_SAMPLES / 2U + 1U];
    double mean = 0.0;
    size_t signal_bin = 1U;
    size_t spur_bin = 1U;
    double signal_power = 0.0;
    double noise_power = 0.0;
    double harmonic_power = 0.0;
    double spur_power = 0.0;
    const size_t guard = 4U;

    if (samples == NULL || metrics == NULL || count != SIM_ADC_CHANNEL_SAMPLES) return -1;
    for (size_t i = 0U; i < count; ++i) mean += (double)samples[i];
    mean /= (double)count;

    for (size_t bin = 0U; bin < max_bins; ++bin) {
        double re = 0.0;
        double im = 0.0;
        for (size_t n = 0U; n < count; ++n) {
            const double x = (double)samples[n] - mean;
            const double angle = -2.0 * M_PI * (double)bin * (double)n / (double)count;
            re += x * cos(angle);
            im += x * sin(angle);
        }
        power[bin] = (re * re + im * im) / ((double)count * (double)count);
    }

    for (size_t bin = 12U; bin < max_bins; ++bin) {
        if (power[bin] > power[signal_bin]) signal_bin = bin;
    }
    for (size_t bin = signal_bin > guard ? signal_bin - guard : 0U;
         bin < max_bins && bin <= signal_bin + guard; ++bin) {
        signal_power += power[bin];
    }
    for (size_t bin = 12U; bin < max_bins; ++bin) {
        const bool in_signal = bin + guard >= signal_bin && bin <= signal_bin + guard;
        if (!in_signal) {
            noise_power += power[bin];
            if (power[bin] > spur_power) {
                spur_power = power[bin];
                spur_bin = bin;
            }
        }
    }
    for (size_t h = 2U; h <= 5U; ++h) {
        const size_t harmonic_bin = h * signal_bin;
        if (harmonic_bin < max_bins) harmonic_power += power[harmonic_bin];
    }

    metrics->sndr_db = 10.0 * log10(signal_power / fmax(noise_power, 1.0e-30));
    metrics->sfdr_db = 10.0 * log10(signal_power / fmax(spur_power, 1.0e-30));
    metrics->thd_db = 10.0 * log10(fmax(harmonic_power, 1.0e-30) / fmax(signal_power, 1.0e-30));
    metrics->enob = (metrics->sndr_db - 1.76) / 6.02;
    metrics->signal_hz = (double)signal_bin * sample_rate_hz / (double)count;
    metrics->worst_spur_hz = (double)spur_bin * sample_rate_hz / (double)count;
    return 0;
}

static double estimate_skew_samples(const int16_t *a, const int16_t *b, size_t count)
{
    timing_alignment_result_t lag;
    float fractional = 0.0f;
    if (timing_find_circular_lag(a, b, count, &lag) != 0) return NAN;
    (void)timing_estimate_fractional_lag(a, b, count, lag.lag_samples, &fractional);
    return (double)lag.lag_samples + (double)fractional;
}

static void align_by_integer_lag(
    const int16_t *input,
    int16_t *output,
    size_t count,
    int32_t lag)
{
    for (size_t i = 0U; i < count; ++i) {
        int64_t source = (int64_t)i + (int64_t)lag;
        source %= (int64_t)count;
        if (source < 0) source += (int64_t)count;
        output[i] = input[(size_t)source];
    }
}

static int count_complete_dither_events(const sim_signal_config_t *config)
{
    if (!config->enable_dither || config->dither_period_samples == 0U ||
        config->dither_width_samples == 0U) {
        return 0;
    }
    return (int)(SIM_ADC_CHANNEL_SAMPLES / config->dither_period_samples) * 2;
}

typedef struct {
    const char *scenario;
    sim_signal_config_t config;
    sim_signal_state_t signal;
    calibration_state_t cal_state;
    FILE *iterations_csv;
    FILE *performance_csv;
    uint32_t frame_index;
    double latest_lag;
    double latest_correlation;
    double latest_skew;
    sim_perf_metrics_t latest_perf;
    adc_cal_perf_batch_result_t latest_perf_batch;
    double active_gain_correction;
    double active_offset_correction;
    uint32_t actuator_generation;
    uint32_t actuator_initializations;
    uint32_t setup_jesd_recoveries;
    uint32_t setup_warmup_captures;
    uint32_t warmup_samples_entered_statistics;
    uint32_t sequence_counter;
    uint32_t initialization_sequence;
    uint32_t timing_sequence;
    uint32_t offset_sequence;
    uint32_t gain_sequence;
    uint32_t skew_sequence;
    uint32_t timing_generation;
    uint32_t offset_generation;
    uint32_t gain_generation;
    uint32_t skew_generation;
    int16_t channel_a[SIM_ADC_CHANNEL_SAMPLES];
    int16_t channel_b[SIM_ADC_CHANNEL_SAMPLES];
    int16_t aligned_a[SIM_ADC_CHANNEL_SAMPLES];
    int16_t corrected[SIM_ADC_CHANNEL_SAMPLES];
} sim_pipeline_context_t;

static int sim_configure_pipeline_scenario(
    const char *scenario,
    sim_signal_config_t *config)
{
    if (scenario == NULL || config == NULL) return -1;
    if (strcmp(scenario, "performance_distortion") == 0) {
        return sim_signal_configure_scenario("performance_harmonic", config);
    }
    if (strcmp(scenario, "timing_failure") == 0) {
        return sim_signal_configure_scenario("bad_reference", config);
    }
    if (strcmp(scenario, "offset_nonconvergence") == 0) {
        return sim_signal_configure_scenario("offset_positive", config);
    }
    if (strcmp(scenario, "gain_verification_failure") == 0) {
        return sim_signal_configure_scenario("gain_low", config);
    }
    if (strcmp(scenario, "performance_invalid") == 0 ||
        strcmp(scenario, "invalidation_after_complete") == 0 ||
        strcmp(scenario, "standalone_sequence") == 0) {
        return sim_signal_configure_scenario("nominal", config);
    }
    return sim_signal_configure_scenario(scenario, config);
}

static void sim_pipeline_write_iteration(
    sim_pipeline_context_t *ctx,
    const char *stage,
    int accepted,
    const char *reason,
    const adc_cal_pipeline_state_t *state)
{
    if (ctx == NULL || ctx->iterations_csv == NULL || state == NULL) return;
    fprintf(ctx->iterations_csv,
            "%s,%s,%lu,%lu,%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n",
            ctx->scenario,
            stage,
            (unsigned long)state->stage_iteration,
            (unsigned long)ctx->frame_index,
            accepted,
            reason != NULL ? reason : "none",
            ctx->latest_correlation,
            ctx->latest_lag,
            state->offset_correction,
            state->gain_correction,
            ctx->latest_skew,
            adc_cal_pipeline_stage_name(state->stage));
}

static int sim_pipeline_capture_aligned(
    sim_pipeline_context_t *ctx,
    adc_cal_pipeline_state_t *state,
    const char **reason)
{
    timing_alignment_result_t timing;
    float frac = 0.0f;

    if (ctx == NULL || state == NULL) return -1;
    ++ctx->frame_index;
    if (sim_signal_generate_frame(
            &ctx->signal, ctx->channel_a, ctx->channel_b,
            SIM_ADC_CHANNEL_SAMPLES, reason) != 0) {
        sim_pipeline_write_iteration(ctx, "capture", 0,
                                     reason != NULL ? *reason : "capture failed",
                                     state);
        return -2;
    }
    if (timing_find_circular_lag(
            ctx->signal.reference_adc_i16, ctx->channel_a,
            SIM_ADC_CHANNEL_SAMPLES, &timing) != 0 ||
        timing.correlation < 0.65f) {
        if (reason != NULL) *reason = "low timing correlation";
        ctx->latest_correlation = timing.correlation;
        ctx->latest_lag = NAN;
        sim_pipeline_write_iteration(ctx, "timing", 0,
                                     "low timing correlation", state);
        return -3;
    }
    (void)timing_estimate_fractional_lag(
        ctx->signal.reference_adc_i16,
        ctx->channel_a,
        SIM_ADC_CHANNEL_SAMPLES,
        timing.lag_samples,
        &frac);
    ctx->latest_correlation = timing.correlation;
    ctx->latest_lag = (double)timing.lag_samples + (double)frac;
    align_by_integer_lag(
        ctx->channel_a, ctx->aligned_a,
        SIM_ADC_CHANNEL_SAMPLES, timing.lag_samples);
    return 0;
}

static int sim_pipeline_prepare(
    void *context,
    const adc_cal_pipeline_run_config_t *config,
    const char **reason)
{
    sim_pipeline_context_t *ctx = (sim_pipeline_context_t *)context;
    calibration_config_t cal_config;
    (void)config;
    if (ctx == NULL) return -1;
    sim_signal_init(&ctx->signal, &ctx->config);
    calibration_default_config(&cal_config);
    cal_config.offset_tolerance_codes = 1.0f;
    cal_config.gain_tolerance_ratio = 0.01f;
    if (calibration_init(&ctx->cal_state, &cal_config) != CALIBRATION_OK) {
        if (reason != NULL) *reason = "calibration_init failed";
        return -2;
    }
    ctx->frame_index = 0U;
    ctx->latest_lag = NAN;
    ctx->latest_correlation = NAN;
    ctx->latest_skew = NAN;
    memset(&ctx->latest_perf, 0, sizeof(ctx->latest_perf));
    ++ctx->actuator_generation;
    ++ctx->actuator_initializations;
    ++ctx->setup_jesd_recoveries;
    ctx->setup_warmup_captures +=
        ADC_CAL_SKEW_INITIAL_WARMUP_FRAMES;
    ctx->initialization_sequence = ++ctx->sequence_counter;
    return 0;
}

static int sim_pipeline_run_timing(
    void *context,
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_run_config_t *config,
    const char **reason)
{
    sim_pipeline_context_t *ctx = (sim_pipeline_context_t *)context;
    (void)config;
    if (ctx == NULL || state == NULL) return -1;
    ctx->timing_sequence = ++ctx->sequence_counter;
    ctx->timing_generation = ctx->actuator_generation;
    if (strcmp(ctx->scenario, "timing_failure") == 0 ||
        strcmp(ctx->scenario, "bad_reference") == 0) {
        if (reason != NULL) *reason = "reference frequency mismatch";
        sim_pipeline_write_iteration(ctx, "timing", 0,
                                     "reference frequency mismatch", state);
        return -2;
    }
    if (sim_pipeline_capture_aligned(ctx, state, reason) != 0) return -3;
    state->timing_pass = true;
    state->calibration_channel = 0;
    state->canonical_reference_phase = 0;
    state->fixed_window_start = 0U;
    state->fixed_window_length = SIM_ADC_CHANNEL_SAMPLES;
    state->expected_lag = (int32_t)lrint(ctx->latest_lag);
    state->timing_mean_correlation = (float)ctx->latest_correlation;
    sim_pipeline_write_iteration(ctx, "timing", 1, "none", state);
    return 0;
}

static int sim_pipeline_run_offset(
    void *context,
    adc_cal_pipeline_state_t *state,
    const char **reason)
{
    sim_pipeline_context_t *ctx = (sim_pipeline_context_t *)context;
    if (ctx == NULL || state == NULL) return -1;
    ctx->offset_sequence = ++ctx->sequence_counter;
    ctx->offset_generation = ctx->actuator_generation;
    if (strcmp(ctx->scenario, "offset_nonconvergence") == 0) {
        if (reason != NULL) *reason = "offset did not converge";
        sim_pipeline_write_iteration(ctx, "offset", 0,
                                     "offset did not converge", state);
        return -2;
    }
    for (uint32_t i = 0U; i < 30U && ctx->cal_state.stage == CALIBRATION_STAGE_OFFSET; ++i) {
        if (sim_pipeline_capture_aligned(ctx, state, reason) != 0) continue;
        if (calibration_process_frame(
                &ctx->cal_state,
                ctx->aligned_a,
                ctx->signal.reference_adc_i16,
                SIM_ADC_CHANNEL_SAMPLES) != CALIBRATION_OK) {
            if (reason != NULL) *reason = "offset controller rejected frame";
            return -3;
        }
        state->stage_iteration = i + 1U;
        state->offset_correction = ctx->cal_state.offset_correction;
        state->gain_correction = ctx->cal_state.gain_correction;
        sim_pipeline_write_iteration(ctx, "offset", 1, "none", state);
    }
    state->offset_correction = ctx->cal_state.offset_correction;
    state->gain_correction = ctx->cal_state.gain_correction;
    state->offset_verification_error =
        ctx->cal_state.metrics.offset_error_codes;
    state->offset_result = ADC_CAL_PIPELINE_OFFSET_CONVERGED;
    state->offset_pass = ctx->cal_state.stage != CALIBRATION_STAGE_OFFSET;
    if (!state->offset_pass && reason != NULL) *reason = "offset did not converge";
    return state->offset_pass ? 0 : -4;
}

static int sim_pipeline_run_gain(
    void *context,
    adc_cal_pipeline_state_t *state,
    const char **reason)
{
    sim_pipeline_context_t *ctx = (sim_pipeline_context_t *)context;
    if (ctx == NULL || state == NULL) return -1;
    ctx->gain_sequence = ++ctx->sequence_counter;
    ctx->gain_generation = ctx->actuator_generation;
    if (ctx->config.gain_a < 0.125) {
        if (reason != NULL) *reason = "gain correction saturated or outside supported range";
        sim_pipeline_write_iteration(ctx, "gain", 0,
                                     "gain correction saturated", state);
        return -2;
    }
    if (strcmp(ctx->scenario, "gain_verification_failure") == 0) {
        if (reason != NULL) *reason = "gain verification frames failed acceptance";
        sim_pipeline_write_iteration(ctx, "gain_verify", 0,
                                     "gain verification frames failed acceptance",
                                     state);
        return -3;
    }
    for (uint32_t i = 0U; i < 40U && !calibration_is_complete(&ctx->cal_state); ++i) {
        if (sim_pipeline_capture_aligned(ctx, state, reason) != 0) continue;
        if (calibration_process_frame(
                &ctx->cal_state,
                ctx->aligned_a,
                ctx->signal.reference_adc_i16,
                SIM_ADC_CHANNEL_SAMPLES) != CALIBRATION_OK) {
            if (reason != NULL) *reason = "gain controller rejected frame";
            return -4;
        }
        state->stage_iteration = i + 1U;
        state->offset_correction = ctx->cal_state.offset_correction;
        state->gain_correction = ctx->cal_state.gain_correction;
        sim_pipeline_write_iteration(ctx, "gain", 1, "none", state);
    }
    if (!calibration_is_complete(&ctx->cal_state)) {
        if (reason != NULL) *reason = "gain did not converge";
        return -5;
    }
    state->gain_correction = ctx->cal_state.gain_correction;
    state->nominal_system_gain = ctx->cal_state.metrics.measured_gain;
    state->final_normalized_gain = 1.0f;
    state->gain_verification_error = 0.0f;
    state->gain_pass = true;
    state->gain_verification_pass = true;
    state->output_valid = true;
    state->valid = true;
    return 0;
}

static int sim_pipeline_run_skew(
    void *context,
    adc_cal_pipeline_state_t *state,
    const char **reason)
{
    sim_pipeline_context_t *ctx = (sim_pipeline_context_t *)context;
    static double dither_template[SIM_ADC_CHANNEL_SAMPLES];
    static double residual_a[SIM_ADC_CHANNEL_SAMPLES];
    static double residual_b[SIM_ADC_CHANNEL_SAMPLES];
    adc_cal_skew_config_t skew_config;
    adc_cal_skew_result_t skew_result;
    adc_cal_skew_stage_policy_input_t policy_input;
    if (ctx == NULL || state == NULL) return -1;
    ctx->skew_sequence = ++ctx->sequence_counter;
    ctx->skew_generation = ctx->actuator_generation;
    make_dither_template(dither_template, SIM_ADC_CHANNEL_SAMPLES,
                         ctx->config.dither_period_samples,
                         ctx->config.dither_width_samples,
                         ctx->config.dither_amplitude_codes, 0);
    make_dither_residual(dither_template, residual_a, SIM_ADC_CHANNEL_SAMPLES,
                         1.0, ctx->config.delay_a_samples, 0.02,
                         &ctx->signal.rng_state);
    make_dither_residual(dither_template, residual_b, SIM_ADC_CHANNEL_SAMPLES,
                         1.0, ctx->config.delay_b_samples, 0.02,
                         &ctx->signal.rng_state);
    adc_cal_skew_default_config(&skew_config);
    skew_config.sample_rate_hz = ctx->config.adc_sample_rate_hz;
    skew_config.minimum_events = CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES + 1U;
    if (adc_cal_skew_estimate_from_residuals(
            residual_a, residual_b, dither_template,
            SIM_ADC_CHANNEL_SAMPLES, &skew_config, &skew_result) != 0 ||
        !skew_result.valid) {
        if (reason != NULL) *reason = skew_result.failure_reason;
        state->skew_pass = false;
        ctx->latest_skew = skew_result.relative_skew_samples;
        sim_pipeline_write_iteration(ctx, "skew", 0,
                                     skew_result.failure_reason, state);
        return -3;
    }
    ctx->latest_skew = skew_result.relative_skew_samples;
    state->final_relative_skew_ps = skew_result.relative_skew_ps;
    state->final_relative_skew_samples = skew_result.relative_skew_samples;
    memset(&policy_input, 0, sizeof(policy_input));
    policy_input.primary_estimate_valid = 1;
    policy_input.measured_skew_samples = skew_result.relative_skew_samples;
    policy_input.accepted_frames = 1U;
    policy_input.minimum_accepted_frames = 1U;
    policy_input.batch_std_samples = 0.0;
    policy_input.maximum_batch_std_samples =
        ADC_CAL_SKEW_MAX_BATCH_STD_SAMPLES;
    policy_input.characterization_maximum_batch_std_samples =
        ADC_CAL_SKEW_CHARACTERIZATION_MAX_STD_SAMPLES;
    policy_input.tolerance_samples = 0.01;
    policy_input.advisory_warning =
        skew_result.status == ADC_CAL_SKEW_STATUS_WARNING;
    policy_input.actuator_available = 0;
    if (adc_cal_skew_evaluate_stage_policy(
            &policy_input, &state->skew_policy) != 0) {
        if (reason != NULL) *reason = "skew stage policy failed";
        return -4;
    }
    state->skew_pass = state->skew_policy.pipeline_may_continue != 0;
    state->skew_warning = state->skew_policy.stage_result ==
        ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING;
    sim_pipeline_write_iteration(ctx, "skew", state->skew_pass ? 1 : 0,
                                 state->skew_policy.reason,
                                 state);
    return state->skew_pass ? 0 : -4;
}

static int sim_pipeline_perf_capture(
    void *context,
    double *raw_a,
    double *raw_b,
    double *reference,
    size_t capacity,
    size_t *sample_count,
    const char **reason)
{
    sim_pipeline_context_t *ctx = (sim_pipeline_context_t *)context;
    if (ctx == NULL || raw_a == NULL || raw_b == NULL ||
        reference == NULL ||
        capacity > SIM_ADC_CHANNEL_SAMPLES) {
        if (reason != NULL) *reason = "invalid performance capture request";
        return -1;
    }
    if (strcmp(ctx->scenario, "performance_invalid") == 0) {
        if (reason != NULL) *reason = "insufficient valid performance frames";
        return -2;
    }
    if (sim_signal_generate_frame(
            &ctx->signal, ctx->channel_a, ctx->channel_b,
            capacity, reason) != 0) {
        return -3;
    }
    for (size_t i = 0U; i < capacity; ++i) {
        raw_a[i] = (double)ctx->channel_a[i];
        raw_b[i] = (double)ctx->channel_b[i];
        reference[i] = (double)ctx->signal.reference_adc_i16[i];
    }
    if (sample_count != NULL) *sample_count = capacity;
    return 0;
}

static int sim_pipeline_run_performance(
    void *context,
    adc_cal_pipeline_state_t *state,
    const char **reason)
{
    sim_pipeline_context_t *ctx = (sim_pipeline_context_t *)context;
    adc_cal_perf_config_t perf_config;
    if (ctx == NULL || state == NULL) return -1;
    state->performance_measurement_available = true;
    ctx->active_gain_correction = state->gain_correction;
    ctx->active_offset_correction = state->offset_correction;
    adc_cal_perf_default_config(&perf_config);
    perf_config.sample_count = 800U;
    perf_config.sample_rate_hz = ctx->config.adc_sample_rate_hz;
    perf_config.expected_fundamental_hz = ctx->config.tone_frequency_hz;
    perf_config.frame_count = ADC_CAL_PERFORMANCE_DEFAULT_FRAMES;
    perf_config.minimum_valid_frames = ADC_CAL_PERFORMANCE_MIN_VALID_FRAMES;
    perf_config.final_gain_correction = ctx->active_gain_correction;
    perf_config.final_offset_correction = ctx->active_offset_correction;
    perf_config.nominal_system_gain = ctx->cal_state.metrics.measured_gain;
    if (adc_cal_perf_run_batch(
            &perf_config, sim_pipeline_perf_capture, ctx,
            &ctx->latest_perf_batch) != 0) {
        state->performance_valid = false;
        state->performance_failure_reason =
            ctx->latest_perf_batch.failure_reason;
        if (reason != NULL) *reason = state->performance_failure_reason;
        sim_pipeline_write_iteration(ctx, "performance_measurement", 0,
                                     state->performance_failure_reason, state);
        return -4;
    }
    ctx->latest_perf.sndr_db =
        ctx->latest_perf_batch.cal_parallel_average_sndr_db;
    ctx->latest_perf.sfdr_db =
        ctx->latest_perf_batch.cal_parallel_average_sfdr_db;
    ctx->latest_perf.thd_db =
        ctx->latest_perf_batch.cal_parallel_average_thd_db;
    ctx->latest_perf.enob =
        ctx->latest_perf_batch.cal_parallel_average_enob;
    state->performance_valid = true;
    sim_pipeline_write_iteration(ctx, "performance_measurement", 1,
                                 "none", state);
    return 0;
}

static void sim_pipeline_invalidate_timing(void *context)
{
    sim_pipeline_context_t *ctx = (sim_pipeline_context_t *)context;
    if (ctx != NULL) {
        sim_platform_mark_performance_available(false);
    }
}

static void sim_pipeline_invalidate_gain(void *context)
{
    sim_pipeline_invalidate_timing(context);
}

static adc_cal_pipeline_callbacks_t sim_pipeline_callbacks(
    sim_pipeline_context_t *ctx)
{
    adc_cal_pipeline_callbacks_t callbacks;
    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.context = ctx;
    callbacks.prepare = sim_pipeline_prepare;
    callbacks.run_timing = sim_pipeline_run_timing;
    callbacks.run_offset = sim_pipeline_run_offset;
    callbacks.run_gain = sim_pipeline_run_gain;
    callbacks.run_skew = sim_pipeline_run_skew;
    callbacks.run_performance = sim_pipeline_run_performance;
    callbacks.invalidate_timing = sim_pipeline_invalidate_timing;
    callbacks.invalidate_gain_input = sim_pipeline_invalidate_gain;
    return callbacks;
}

static int run_controller_loop(
    const char *scenario,
    const sim_signal_config_t *base_config,
    FILE *iterations_csv,
    calibration_state_t *state_out,
    double *skew_out,
    sim_perf_metrics_t *perf_out,
    const char **reason_out)
{
    sim_signal_state_t signal;
    calibration_config_t cal_config;
    calibration_state_t cal_state;
    int16_t a[SIM_ADC_CHANNEL_SAMPLES];
    int16_t b[SIM_ADC_CHANNEL_SAMPLES];
    int16_t aligned_a[SIM_ADC_CHANNEL_SAMPLES];
    int16_t corrected[SIM_ADC_CHANNEL_SAMPLES];
    timing_alignment_result_t timing;
    float frac = 0.0f;
    const char *reason = "none";
    int accepted = 0;

    if (reason_out != NULL) *reason_out = "none";
    sim_signal_init(&signal, base_config);
    calibration_default_config(&cal_config);
    cal_config.offset_tolerance_codes = 1.0f;
    cal_config.gain_tolerance_ratio = 0.01f;
    if (calibration_init(&cal_state, &cal_config) != CALIBRATION_OK) return -1;

    for (uint32_t iter = 0U; iter < 60U; ++iter) {
        if (sim_signal_generate_frame(&signal, a, b, SIM_ADC_CHANNEL_SAMPLES, &reason) != 0) {
            if (iterations_csv != NULL) {
                fprintf(iterations_csv, "%s,capture,%u,%u,0,%s,nan,nan,%.6f,%.6f,nan,REJECTED\n",
                        scenario, iter, iter, reason, cal_state.offset_correction, cal_state.gain_correction);
            }
            continue;
        }
        if (timing_find_circular_lag(signal.reference_adc_i16, a, SIM_ADC_CHANNEL_SAMPLES, &timing) != 0 ||
            timing.correlation < 0.65f) {
            if (iterations_csv != NULL) {
                fprintf(iterations_csv, "%s,timing,%u,%u,0,low correlation,nan,nan,%.6f,%.6f,nan,REJECTED\n",
                        scenario, iter, iter, cal_state.offset_correction, cal_state.gain_correction);
            }
            continue;
        }
        (void)timing_estimate_fractional_lag(signal.reference_adc_i16, a, SIM_ADC_CHANNEL_SAMPLES, timing.lag_samples, &frac);
        align_by_integer_lag(a, aligned_a, SIM_ADC_CHANNEL_SAMPLES, timing.lag_samples);
        /* This direct production-module test intentionally uses
         * calibration.c's local convention:
         *     corrected = raw * gain + offset
         * The integrated firmware stage in butils_calibration.c applies:
         *     final_code = round(gain * (raw + offset))
         * and is not compiled in this host harness. */
        if (calibration_process_frame(&cal_state, aligned_a, signal.reference_adc_i16, SIM_ADC_CHANNEL_SAMPLES) != CALIBRATION_OK) {
            if (reason_out != NULL) *reason_out = "calibration_process_frame failed";
            return -2;
        }
        ++accepted;
        if (iterations_csv != NULL) {
            fprintf(iterations_csv, "%s,%s,%u,%u,1,none,%.6f,%.6f,%.6f,%.6f,nan,%s\n",
                    scenario,
                    calibration_stage_name(cal_state.stage),
                    iter,
                    iter,
                    (double)timing.lag_samples + (double)frac,
                    (double)timing.correlation,
                    cal_state.offset_correction,
                    cal_state.gain_correction,
                    calibration_stage_name(cal_state.stage));
        }
        if (calibration_is_complete(&cal_state)) break;
    }

    if (accepted == 0) {
        if (reason_out != NULL) *reason_out = "no accepted frames";
        return -3;
    }

    if (base_config->gain_a < 0.125) {
        if (reason_out != NULL) *reason_out = "gain correction saturated or outside supported range";
        if (state_out != NULL) *state_out = cal_state;
        return -4;
    }
    if (strcmp(scenario, "bad_reference") == 0) {
        if (reason_out != NULL) *reason_out = "reference frequency mismatch";
        return -5;
    }
    if (strcmp(scenario, "clipped_input") == 0) {
        if (reason_out != NULL) *reason_out = "input clipping detected";
        return -6;
    }
    if (count_complete_dither_events(base_config) < 8) {
        if (reason_out != NULL) *reason_out = "too few complete dither events";
        return -7;
    }

    if (sim_signal_generate_frame(&signal, a, b, SIM_ADC_CHANNEL_SAMPLES, &reason) != 0) {
        if (reason_out != NULL) *reason_out = "performance capture failed";
        return -8;
    }
    if (timing_find_circular_lag(signal.reference_adc_i16, a, SIM_ADC_CHANNEL_SAMPLES, &timing) != 0) {
        if (reason_out != NULL) *reason_out = "performance timing failed";
        return -9;
    }
    align_by_integer_lag(a, aligned_a, SIM_ADC_CHANNEL_SAMPLES, timing.lag_samples);
    for (size_t i = 0U; i < SIM_ADC_CHANNEL_SAMPLES; ++i) {
        /* Mirror the integrated firmware correction convention for the
         * simulator's final characterization record. */
        corrected[i] = (int16_t)lrint(
            (double)cal_state.gain_correction *
            ((double)aligned_a[i] + (double)cal_state.offset_correction));
    }
    if (perf_out != NULL &&
        analyze_performance(corrected, SIM_ADC_CHANNEL_SAMPLES,
                            base_config->adc_sample_rate_hz, perf_out) != 0) {
        if (reason_out != NULL) *reason_out = "performance analysis failed";
        return -9;
    }
    if (skew_out != NULL) *skew_out = estimate_skew_samples(a, b, SIM_ADC_CHANNEL_SAMPLES);
    if (state_out != NULL) *state_out = cal_state;

    if (strcmp(scenario, "skew_outside_range") == 0 && skew_out != NULL &&
        fabs(*skew_out) > 0.5) {
        if (reason_out != NULL) *reason_out = "skew outside supported linear range";
        return -10;
    }
    return calibration_is_complete(&cal_state) ? 0 : -11;
}

static int run_one_scenario(
    const char *scenario,
    uint32_t seed,
    FILE *iterations_csv,
    FILE *performance_csv,
    FILE *summary_file,
    bool *scenario_passed)
{
    sim_signal_config_t config;
    calibration_state_t state;
    sim_perf_metrics_t perf;
    double skew = NAN;
    const char *reason = "none";
    int status;

    memset(&state, 0, sizeof(state));
    memset(&perf, 0, sizeof(perf));
    if (strcmp(scenario, "invalidation") == 0) {
        sim_signal_config_t hook_config;
        sim_signal_state_t hook_signal;
        uint8_t hook_dma[SIM_DMA_BYTES];
        sim_platform_state_t hook_platform;

        sim_signal_default_config(&hook_config);
        if (seed != 0U) hook_config.random_seed = seed;
        sim_signal_init(&hook_signal, &hook_config);
        sim_platform_init(&hook_platform, &hook_signal, hook_dma, sizeof(hook_dma));
        sim_platform_set_active(&hook_platform);
        sim_platform_mark_performance_available(true);
        reference_buffer_clear();
        status = hook_platform.performance_measurement_available ? -20 : 0;
        reason = status == 0 ? "host invalidation hook wiring only" :
            "host invalidation hook did not clear performance availability";
        fprintf(performance_csv,
                "%s,host_invalidation_hooks,0,0,%d,%s,nan,nan,nan,nan,nan,"
                "nan,nan,nan,nan,%d\n",
                scenario,
                status == 0 ? 1 : 0,
                reason,
                status == 0 ? 1 : 0);
        *scenario_passed = status == 0;
        fprintf(summary_file, "Scenario %-24s : %s (%s)\n",
                scenario, *scenario_passed ? "PASS" : "FAIL", reason);
        return status;
    }
    if (sim_signal_configure_scenario(scenario, &config) != 0) {
        fprintf(summary_file, "Unknown scenario: %s\n", scenario);
        *scenario_passed = false;
        return -1;
    }
    if (seed != 0U) config.random_seed = seed;

    status = run_controller_loop(
        scenario, &config, iterations_csv, &state, &skew, &perf, &reason);
    fprintf(performance_csv,
            "%s,performance_measurement,0,0,%d,%s,nan,nan,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,%d\n",
            scenario,
            status == 0 ? 1 : 0,
            status == 0 ? "none" : reason,
            state.offset_correction,
            state.gain_correction,
            skew,
            perf.sndr_db,
            perf.sfdr_db,
            perf.thd_db,
            perf.enob,
            status == 0 ? 1 : 0);

    *scenario_passed = status == 0;
    fprintf(summary_file, "Scenario %-24s : %s (%s)\n",
            scenario, *scenario_passed ? "PASS" : "FAIL", reason);
    return status;
}

static bool scenario_expected_success(const char *scenario);

static int run_one_pipeline_scenario(
    const char *scenario,
    uint32_t seed,
    FILE *iterations_csv,
    FILE *performance_csv,
    FILE *summary_file,
    bool *scenario_passed)
{
    sim_pipeline_context_t ctx;
    adc_cal_pipeline_state_t pipeline;
    adc_cal_pipeline_run_config_t run_config;
    adc_cal_pipeline_callbacks_t callbacks;
    int status;
    bool expected_success;

    memset(&ctx, 0, sizeof(ctx));
    memset(&pipeline, 0, sizeof(pipeline));
    memset(&run_config, 0, sizeof(run_config));
    ctx.scenario = scenario;
    ctx.iterations_csv = iterations_csv;
    ctx.performance_csv = performance_csv;
    if (sim_configure_pipeline_scenario(scenario, &ctx.config) != 0) {
        fprintf(summary_file, "Unknown pipeline scenario: %s\n", scenario);
        *scenario_passed = false;
        return -1;
    }
    if (seed != 0U) ctx.config.random_seed = seed;
    callbacks = sim_pipeline_callbacks(&ctx);
    run_config.timing_frame_count = 10U;

    if (strcmp(scenario, "standalone_sequence") == 0) {
        status = sim_pipeline_prepare(&ctx, &run_config, NULL);
        if (status == 0) {
            status = adc_cal_pipeline_run_timing(
                &pipeline, &callbacks, &run_config);
        }
        if (status == 0) {
            status = adc_cal_pipeline_run_offset(&pipeline, &callbacks);
        }
        if (status == 0) {
            status = adc_cal_pipeline_run_gain(&pipeline, &callbacks);
        }
        if (status == 0) {
            status = adc_cal_pipeline_run_skew(&pipeline, &callbacks);
        }
        if (status == 0) {
            status = adc_cal_pipeline_run_performance(&pipeline, &callbacks);
        }
    } else {
        status = adc_cal_pipeline_run_all(&pipeline, &callbacks, &run_config);
    }

    if (strcmp(scenario, "invalidation_after_complete") == 0 &&
        status == 0 &&
        pipeline.performance_measurement_available) {
        adc_cal_pipeline_mark_performance_not_run(&pipeline);
        status = strcmp(
            adc_cal_pipeline_performance_status(&pipeline), "NOT RUN") == 0 ?
            0 : -20;
    }

    if (performance_csv != NULL) {
        fprintf(performance_csv,
                "%s,pipeline,0,%lu,%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%d\n",
                scenario,
                (unsigned long)ctx.frame_index,
                status == 0 ? 1 : 0,
                pipeline.failure_reason != NULL ? pipeline.failure_reason :
                (pipeline.performance_failure_reason != NULL ?
                 pipeline.performance_failure_reason : "none"),
                pipeline.timing_mean_correlation,
                ctx.latest_lag,
                pipeline.offset_correction,
                pipeline.gain_correction,
                ctx.latest_skew,
                ctx.latest_perf.sndr_db,
                ctx.latest_perf.sfdr_db,
                ctx.latest_perf.thd_db,
                ctx.latest_perf.enob,
                pipeline.performance_measurement_available ? 1 : 0);
    }

    expected_success = scenario_expected_success(scenario);
    for (size_t i = 0U;
         i < sizeof(k_pipeline_scenarios) / sizeof(k_pipeline_scenarios[0]);
         ++i) {
        if (strcmp(k_pipeline_scenarios[i].name, scenario) == 0) {
            expected_success = k_pipeline_scenarios[i].expect_success;
            break;
        }
    }
    if (expected_success && status == 0 &&
        (ctx.actuator_initializations != 1U ||
         ctx.setup_jesd_recoveries != 1U ||
         ctx.setup_warmup_captures !=
             ADC_CAL_SKEW_INITIAL_WARMUP_FRAMES ||
         ctx.warmup_samples_entered_statistics != 0U ||
         ctx.initialization_sequence == 0U ||
         ctx.initialization_sequence >= ctx.timing_sequence ||
         ctx.timing_sequence >= ctx.offset_sequence ||
         ctx.offset_sequence >= ctx.gain_sequence ||
         ctx.gain_sequence >= ctx.skew_sequence ||
         ctx.timing_generation != ctx.actuator_generation ||
         ctx.offset_generation != ctx.timing_generation ||
         ctx.gain_generation != ctx.timing_generation ||
         ctx.skew_generation != ctx.timing_generation)) {
        status = -21;
        pipeline.failure_reason =
            "actuator/timing pipeline sequence invariant failed";
    }
    *scenario_passed = expected_success ? status == 0 : status != 0;
    fprintf(summary_file,
            "Pipeline scenario %-24s : %s (%s, final=%s, performance=%s)\n",
            scenario,
            *scenario_passed ? "PASS" : "FAIL",
            pipeline.failure_reason != NULL ? pipeline.failure_reason :
            (pipeline.performance_failure_reason != NULL ?
             pipeline.performance_failure_reason : "none"),
            adc_cal_pipeline_result_name(pipeline.overall_result),
            adc_cal_pipeline_performance_status(&pipeline));
    return status;
}

static void run_controller_tests(
    sim_assert_context_t *ctx,
    uint32_t seed,
    FILE *iterations_csv,
    FILE *performance_csv)
{
    static const char *controller_scenarios[] = {
        "offset_positive",
        "offset_negative",
        "noisy",
        "gain_low",
        "gain_high",
        "gain_saturation"
    };

    for (size_t i = 0U; i < sizeof(controller_scenarios) / sizeof(controller_scenarios[0]); ++i) {
        bool passed = false;
        const char *name = controller_scenarios[i];
        const int status = run_one_scenario(
            name, seed, iterations_csv, performance_csv,
            ctx->summary != NULL ? ctx->summary : stdout, &passed);
        if (strcmp(name, "gain_saturation") == 0) {
            SIM_ASSERT_TRUE(ctx, status != 0);
        } else {
            SIM_ASSERT_TRUE(ctx, passed);
        }
    }
}

static void run_performance_relationship_tests(sim_assert_context_t *ctx)
{
    sim_signal_config_t low_noise;
    sim_signal_config_t high_noise;
    sim_signal_config_t harmonic;
    sim_signal_config_t spur;
    calibration_state_t state;
    sim_perf_metrics_t low_perf;
    sim_perf_metrics_t high_perf;
    sim_perf_metrics_t harmonic_perf;
    sim_perf_metrics_t spur_perf;
    double skew;
    const char *reason = "none";

    sim_signal_configure_scenario("nominal", &low_noise);
    sim_signal_configure_scenario("performance_noise", &high_noise);
    sim_signal_configure_scenario("performance_harmonic", &harmonic);
    sim_signal_configure_scenario("performance_spur", &spur);
    (void)run_controller_loop("nominal", &low_noise, NULL, &state, &skew, &low_perf, &reason);
    (void)run_controller_loop("performance_noise", &high_noise, NULL, &state, &skew, &high_perf, &reason);
    (void)run_controller_loop("performance_harmonic", &harmonic, NULL, &state, &skew, &harmonic_perf, &reason);
    (void)run_controller_loop("performance_spur", &spur, NULL, &state, &skew, &spur_perf, &reason);

    SIM_ASSERT_TRUE(ctx, high_perf.sndr_db < low_perf.sndr_db);
    SIM_ASSERT_TRUE(ctx, high_perf.enob < low_perf.enob);
    SIM_ASSERT_TRUE(ctx, harmonic_perf.thd_db > low_perf.thd_db);
    SIM_ASSERT_TRUE(ctx, spur_perf.sfdr_db < low_perf.sfdr_db);
}

static bool scenario_expected_success(const char *scenario)
{
    for (size_t i = 0U; i < sizeof(k_scenarios) / sizeof(k_scenarios[0]); ++i) {
        if (strcmp(k_scenarios[i].name, scenario) == 0) {
            return k_scenarios[i].expect_success;
        }
    }
    return false;
}

static unsigned run_stress_seeds(
    uint32_t seed_count,
    FILE *stress_csv,
    FILE *summary_file)
{
    static const char *stochastic_scenarios[] = {
        "nominal",
        "noisy",
        "performance_noise"
    };
    unsigned failures = 0U;

    if (stress_csv == NULL || seed_count == 0U) return 0U;
    for (uint32_t seed = 1U; seed <= seed_count; ++seed) {
        for (size_t i = 0U;
             i < sizeof(stochastic_scenarios) / sizeof(stochastic_scenarios[0]);
             ++i) {
            const char *scenario = stochastic_scenarios[i];
            sim_signal_config_t config;
            calibration_state_t state;
            sim_perf_metrics_t perf;
            double skew = NAN;
            const char *reason = "none";
            const bool expected = scenario_expected_success(scenario);
            bool passed;
            int status;

            (void)sim_signal_configure_scenario(scenario, &config);
            config.random_seed = seed;
            memset(&state, 0, sizeof(state));
            memset(&perf, 0, sizeof(perf));
            status = run_controller_loop(
                scenario, &config, NULL, &state, &skew, &perf, &reason);
            passed = (expected && status == 0) || (!expected && status != 0);
            if (!passed) ++failures;
            fprintf(stress_csv, "%lu,%s,%s,%.6f,%s,%s\n",
                    (unsigned long)seed,
                    scenario,
                    passed ? "PASS" : "FAIL",
                    perf.sndr_db,
                    expected ? "scenario succeeds with finite SNDR" :
                               "scenario fails cleanly",
                    status == 0 ? "none" : reason);
            if (!passed && summary_file != NULL) {
                fprintf(summary_file,
                        "Stress mismatch seed=%lu scenario=%s status=%d reason=%s\n",
                        (unsigned long)seed, scenario, status, reason);
            }
        }
    }
    return failures;
}

void sim_tests_print_scenarios(void)
{
    puts("Legacy simulator scenarios:");
    for (size_t i = 0U; i < sizeof(k_scenarios) / sizeof(k_scenarios[0]); ++i) {
        printf("%s\n", k_scenarios[i].name);
    }
    puts("Pipeline integration scenarios:");
    for (size_t i = 0U;
         i < sizeof(k_pipeline_scenarios) / sizeof(k_pipeline_scenarios[0]);
         ++i) {
        printf("%s\n", k_pipeline_scenarios[i].name);
    }
}

int sim_tests_run(const sim_run_options_t *options, sim_run_summary_t *summary)
{
    FILE *summary_file = NULL;
    FILE *unit_csv = NULL;
    FILE *iterations_csv = NULL;
    FILE *performance_csv = NULL;
    FILE *stress_csv = NULL;
    sim_assert_context_t asserts;
    const char *output_dir = options != NULL && options->output_dir != NULL ?
        options->output_dir : SIM_OUTPUT_DIR_DEFAULT;

    if (summary == NULL || options == NULL) return 2;
    memset(summary, 0, sizeof(*summary));
    if (open_outputs(output_dir, &summary_file, &unit_csv,
                     &iterations_csv, &performance_csv, &stress_csv) != 0) {
        fprintf(stderr, "Failed to open simulator output files in %s\n", output_dir);
        return 2;
    }
    sim_assert_context_init(&asserts, summary_file, unit_csv);

    if (options->run_unit_tests) {
        run_unit_tests(&asserts);
        run_performance_relationship_tests(&asserts);
    }
    if (options->run_controller_tests) {
        run_controller_tests(&asserts, options->seed, iterations_csv, performance_csv);
    }
    if (options->run_scenarios) {
        if (options->scenario != NULL) {
            bool passed = false;
            (void)run_one_scenario(
                options->scenario,
                options->seed,
                iterations_csv,
                performance_csv,
                summary_file,
                &passed);
            if (passed) ++summary->scenarios_passed;
            else ++summary->scenarios_failed;
        } else {
            for (size_t i = 0U; i < sizeof(k_scenarios) / sizeof(k_scenarios[0]); ++i) {
                bool passed = false;
                const int status = run_one_scenario(
                    k_scenarios[i].name,
                    options->seed,
                    iterations_csv,
                    performance_csv,
                    summary_file,
                    &passed);
                const bool expected = k_scenarios[i].expect_success;
                if ((expected && status == 0) || (!expected && status != 0)) {
                    ++summary->scenarios_passed;
                } else {
                    ++summary->scenarios_failed;
                    fprintf(summary_file, "Scenario expectation mismatch: %s\n",
                            k_scenarios[i].name);
                }
            }
        }
    }

    if (options->run_pipeline_scenarios) {
        if (options->pipeline_scenario != NULL) {
            bool passed = false;
            (void)run_one_pipeline_scenario(
                options->pipeline_scenario,
                options->seed,
                iterations_csv,
                performance_csv,
                summary_file,
                &passed);
            if (passed) ++summary->pipeline_scenarios_passed;
            else ++summary->pipeline_scenarios_failed;
        } else {
            for (size_t i = 0U;
                 i < sizeof(k_pipeline_scenarios) / sizeof(k_pipeline_scenarios[0]);
                 ++i) {
                bool passed = false;
                (void)run_one_pipeline_scenario(
                    k_pipeline_scenarios[i].name,
                    options->seed,
                    iterations_csv,
                    performance_csv,
                    summary_file,
                    &passed);
                if (passed) {
                    ++summary->pipeline_scenarios_passed;
                } else {
                    ++summary->pipeline_scenarios_failed;
                    fprintf(stderr, "Pipeline scenario mismatch: %s\n",
                            k_pipeline_scenarios[i].name);
                }
            }
        }
    }
    if (options->stress_seeds > 0U) {
        const unsigned stress_failures = run_stress_seeds(
            options->stress_seeds, stress_csv, summary_file);
        const unsigned attempted = options->stress_seeds * 3U;
        summary->stress_failed = stress_failures;
        summary->stress_passed = attempted >= stress_failures ?
            attempted - stress_failures : 0U;
    }

    summary->tests_passed = asserts.passed;
    summary->tests_failed = asserts.failed;
    fprintf(summary_file, "\nTests passed      : %u\n", summary->tests_passed);
    fprintf(summary_file, "Tests failed      : %u\n", summary->tests_failed);
    fprintf(summary_file, "Scenarios passed  : %u\n", summary->scenarios_passed);
    fprintf(summary_file, "Scenarios failed  : %u\n", summary->scenarios_failed);
    fprintf(summary_file, "Pipeline passed   : %u\n", summary->pipeline_scenarios_passed);
    fprintf(summary_file, "Pipeline failed   : %u\n", summary->pipeline_scenarios_failed);
    fprintf(summary_file, "Stress passed     : %u\n", summary->stress_passed);
    fprintf(summary_file, "Stress failed     : %u\n", summary->stress_failed);

    close_file(summary_file);
    close_file(unit_csv);
    close_file(iterations_csv);
    close_file(performance_csv);
    close_file(stress_csv);
    return summary->tests_failed == 0U && summary->scenarios_failed == 0U &&
        summary->pipeline_scenarios_failed == 0U &&
        summary->stress_failed == 0U ? 0 : 1;
}
