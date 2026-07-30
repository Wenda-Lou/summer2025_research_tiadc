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
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_begin_with_format(64U, REFERENCE_FORMAT_DAC_RATE_2X), REFERENCE_BUFFER_OK);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_write_chunk(0U, reference, 32U), REFERENCE_BUFFER_OK);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_write_chunk(32U, &reference[32], 32U), REFERENCE_BUFFER_OK);
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_finalize(), REFERENCE_BUFFER_OK);
    SIM_ASSERT_TRUE(ctx, reference_buffer_is_ready());
    SIM_ASSERT_EQ_INT(ctx, reference_buffer_length(), 64U);

    SIM_ASSERT_EQ_INT(ctx, reference_buffer_begin_with_format(64U, REFERENCE_FORMAT_DAC_RATE_2X), REFERENCE_BUFFER_OK);
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

static int unit_skew_estimator_direct(sim_assert_context_t *ctx)
{
    double template_samples[256];
    double a[256];
    double b[256];
    adc_cal_skew_config_t config;
    adc_cal_skew_result_t result;
    uint32_t rng = 77U;

    adc_cal_skew_default_config(&config);
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
    SIM_ASSERT_TRUE(ctx, adc_cal_skew_estimate_from_residuals(a, b, template_samples, 256U, &config, &result) != 0);
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
    return 1;
}

typedef struct {
    uint32_t seed;
    uint32_t frame;
    bool fail_all;
    double noise;
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
    make_perf_tone(reference, capacity, 1450000000.0, 145000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, &ctx->seed);
    make_perf_tone(raw_a, capacity, 1450000000.0, 145000000.0,
                   1000.0, 4.0, ctx->noise, 0.0, 0.0, 0.0, 0.0, false,
                   &ctx->seed);
    make_perf_tone(raw_b, capacity, 1450000000.0, 145000000.0,
                   1000.0, 5.0, ctx->noise, 0.0, 0.0, 0.0, 0.0, false,
                   &ctx->seed);
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

    adc_cal_perf_default_config(&config);
    config.sample_count = 800U;
    config.sample_rate_hz = 1450000000.0;
    config.expected_fundamental_hz = 145000000.0;

    make_perf_tone(clean, 800U, config.sample_rate_hz, 145000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(clean, 800U, config.sample_rate_hz, &clean_metrics), 0);
    SIM_ASSERT_TRUE(ctx, clean_metrics.sndr_db > 60.0f);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 147000000.0,
                   1000.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics), 0);
    SIM_ASSERT_TRUE(ctx, noisy_metrics.signal_hz == noisy_metrics.signal_hz);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 145000000.0,
                   1000.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics), 0);
    SIM_ASSERT_TRUE(ctx, clean_metrics.sndr_db > noisy_metrics.sndr_db);

    make_perf_tone(harmonic2, 800U, config.sample_rate_hz, 145000000.0,
                   1000.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(harmonic2, 800U, config.sample_rate_hz, &distorted_metrics), 0);
    SIM_ASSERT_TRUE(ctx, distorted_metrics.thd_db > clean_metrics.thd_db);

    make_perf_tone(harmonic3, 800U, config.sample_rate_hz, 145000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(harmonic3, 800U, config.sample_rate_hz, &distorted_metrics), 0);
    SIM_ASSERT_TRUE(ctx, distorted_metrics.thd_db > clean_metrics.thd_db);

    make_perf_tone(spur, 800U, config.sample_rate_hz, 145000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.0, 230000000.0, 100.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(spur, 800U, config.sample_rate_hz, &distorted_metrics), 0);
    SIM_ASSERT_TRUE(ctx, distorted_metrics.sfdr_db < clean_metrics.sfdr_db);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 145000000.0,
                   1000.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics), 0);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 145000000.0,
                   9000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, &rng);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics), 0);
    SIM_ASSERT_TRUE(ctx, noisy_metrics.sndr_db < clean_metrics.sndr_db);

    memset(noisy, 0, sizeof(noisy));
    SIM_ASSERT_TRUE(ctx, adc_cal_perf_analyze_record(noisy, 800U, config.sample_rate_hz, &noisy_metrics) != 0);
    SIM_ASSERT_TRUE(ctx, adc_cal_perf_analyze_record(clean, 0U, config.sample_rate_hz, &noisy_metrics) != 0);

    make_perf_tone(ref, 800U, config.sample_rate_hz, 145000000.0,
                   1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, &rng);
    memcpy(raw_b, clean, sizeof(raw_b));
    for (size_t i = 0U; i < 800U; ++i) raw_b[i] += 80.0 * sin((double)i);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_perf_analyze_frame(clean, raw_b, clean, raw_b, ref, &config, 1U, &frame), 0);
    SIM_ASSERT_TRUE(ctx, frame.raw_a.sndr_db > frame.raw_b.sndr_db);

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

    capture_ctx.fail_all = true;
    config.minimum_valid_frames = 1U;
    SIM_ASSERT_TRUE(ctx, adc_cal_perf_run_batch(&config, perf_capture_test, &capture_ctx, &batch) != 0);

    make_perf_tone(noisy, 800U, config.sample_rate_hz, 145000000.0,
                   1000.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, false, &rng_repeat);
    rng_repeat = 42U;
    make_perf_tone(repeat, 800U, config.sample_rate_hz, 145000000.0,
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
    if (ctx == NULL || state == NULL) return -1;
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
    state->skew_pass = true;
    sim_pipeline_write_iteration(ctx, "skew", state->skew_pass ? 1 : 0,
                                 state->skew_pass ? "none" :
                                 "skew outside supported linear range",
                                 state);
    return 0;
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
    ctx->latest_perf.sndr_db = ctx->latest_perf_batch.sndr_db;
    ctx->latest_perf.sfdr_db = ctx->latest_perf_batch.sfdr_db;
    ctx->latest_perf.thd_db = ctx->latest_perf_batch.thd_db;
    ctx->latest_perf.enob = ctx->latest_perf_batch.enob;
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
