/* butils.c
 * Concise, table-driven UART command handler.
 */

#include "butils.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "xil_printf.h"
#include "xparameters.h"
#include "peripherals.h"
#include "bjesdphy.h"
#include "bjesdlink.h"
#include "ad9695_api.h"
#include "xspips.h"
#include "xaxidma.h"
#include "baxidma.h"
#include "sleep.h"
#include "ad9695_registers.h"
#include "ethernet.h"
#include "calibration.h"
#include "reference_buffer.h"
#include "adc_frame.h"
#include "timing_alignment.h"
#include "adc_test_config.h"

#include "xil_cache.h"
#include "xuartps.h"

extern XUartPs uart_inst;
extern XSpiPs spi_inst;
extern XAxiDma dma_inst;
extern u8 *RxBufferPtr;

extern uint8_t uart_send_flag; //Send flag enabled by the uart
volatile uint8_t adc_sweep_active = 0;

typedef struct {
    double configured_rate_hz;
    double analysis_rate_hz;
    double correction_factor;
    bool measured_rate_valid;
} adc_sample_rate_state_t;

static adc_sample_rate_state_t g_adc_sample_rate = {
    ADC_CONFIGURED_SAMPLE_RATE_HZ,
    ADC_CONFIGURED_SAMPLE_RATE_HZ,
    1.0,
    false
};

double adc_get_configured_sample_rate_hz(void)
{
    return g_adc_sample_rate.configured_rate_hz;
}

double adc_get_effective_sample_rate_hz(void)
{
    return g_adc_sample_rate.analysis_rate_hz;
}

double adc_get_sample_rate_correction_factor(void)
{
    return g_adc_sample_rate.correction_factor;
}

bool adc_effective_sample_rate_is_valid(void)
{
    return g_adc_sample_rate.measured_rate_valid;
}

bool adc_set_effective_sample_rate_hz(double rate_hz)
{
    const double configured_hz = g_adc_sample_rate.configured_rate_hz;
    if (!isfinite(rate_hz) || rate_hz < 0.8 * configured_hz ||
        rate_hz > 1.2 * configured_hz)
        return false;
    g_adc_sample_rate.analysis_rate_hz = rate_hz;
    g_adc_sample_rate.correction_factor = rate_hz / configured_hz;
    g_adc_sample_rate.measured_rate_valid = true;
    return true;
}

#define ERR(fmt, ...) xil_printf("Command Error: " fmt "\r\n", ##__VA_ARGS__)

#define CAL_ALIGNMENT_GUARD_SAMPLES 64U
#define CAL_MIN_ANALYSIS_SAMPLES    512U
#define ADC_CAL_DEFAULT_FRAMES              10U
#define ADC_CAL_MIN_FRAMES                  2U
#define ADC_CAL_MAX_FRAMES                  100U
#define CAL_TIMING_MIN_CORRELATION           0.970f
#define CAL_TIMING_MIN_ANALYSIS_SAMPLES       800U
#define CAL_TIMING_MAX_ABS_FRAC_LAG          0.5f
#define CAL_TIMING_MIN_ACCEPTED_FRAMES       5U
#define CAL_TIMING_MIN_ACCEPTANCE_RATE       0.70f
#define CAL_DAC_REF_MIN_CORRELATION           0.970f
#define CAL_REF_FREQ_TOLERANCE_HZ               2000000.0
#define CAL_REF_VARIANCE_EPSILON                1.0e-6
#define CAL_REF_PHASE_EQUIVALENT_CORR_DELTA     0.001f
#define CAL_REF_SPECTRAL_PEAK_COUNT              10U
#define CAL_ADC_FULL_SCALE_CODES                 8192.0f
#define CAL_DAC_FULL_SCALE_CODES                32768.0f
#define CAL_REF_ADC_DEBUG_SAMPLE_COUNT           32U

_Static_assert(ADC_VALID_SAMPLE_COUNT == ADC_TEST_CAPTURE_SAMPLES,
               "ADC test configuration sample count mismatch");

typedef struct {
    size_t reference_start;
    size_t measurement_start;
    size_t overlap_count;
    size_t analysis_count;
    size_t applied_guard;
} adc_cal_overlap_t;

typedef enum {
    CAL_TIMING_REJECT_NONE = 0,
    CAL_TIMING_REJECT_DMA,
    CAL_TIMING_REJECT_RECONSTRUCTION,
    CAL_TIMING_REJECT_INTEGER_ALIGNMENT,
    CAL_TIMING_REJECT_FRACTIONAL_ALIGNMENT,
    CAL_TIMING_REJECT_INVALID_OVERLAP,
    CAL_TIMING_REJECT_LOW_CORRELATION,
    CAL_TIMING_REJECT_TOO_FEW_SAMPLES,
    CAL_TIMING_REJECT_INVALID_METRIC
} calibration_timing_reject_reason_t;

typedef struct {
    uint8_t capture_success;
    uint8_t alignment_success;
    uint8_t accepted;
    int32_t integer_lag;
    float fractional_lag;
    float total_lag;
    uint32_t valid_overlap_samples;
    uint32_t analysis_samples;
    float correlation;
    float raw_rmse;
    float fitted_rmse;
    calibration_timing_reject_reason_t reject_reason;
} calibration_timing_frame_result_t;

static int16_t calibration_convert_reference_to_adc_units(int16_t dac_code)
{
    /*
     * No verified AD9164-code to AD9695-code transfer scale is available.
     * Preserve the uploaded numeric scale and report a relative gain.  Any
     * future physical conversion belongs in this single function.
     */
    return dac_code;
}

int adc_capture_frame(void);
static void adc_ifc_sweep(void);
static void calibration_run_adc_reference_diagnostic(void);
static void calibration_run_raw_mapping_diagnostic(
    const int16_t *even_reference, const int16_t *odd_reference);
static void handle_adc_offset_calibration_loop_cmd(void);
static void handle_adc_offset_calibration_status_cmd(void);
static void handle_adc_gain_calibration_loop_cmd(void);
static void handle_adc_gain_calibration_status_cmd(void);
void adc_timing_capture(uint32_t frame_count);

static void print_float_value(const char *label, float value, const char *unit)
{
    int32_t whole;
    int32_t fraction;
    float absolute_value;

    absolute_value = fabsf(value);
    whole = (int32_t)absolute_value;
    fraction = (int32_t)(
        (absolute_value - (float)whole) * 1000000.0f
    );

    xil_printf("%-22s: ", label);

    if (value < 0.0f) {
        xil_printf("-");
    }

    xil_printf(
        "%ld.%06ld%s\r\n",
        (long)whole,
        (long)fraction,
        unit != NULL ? unit : ""
    );
}

static void print_double_value(const char *label, double value, const char *unit);
static void print_double_inline(double value);

static void print_adc_sample_rate_state(void)
{
    print_double_value("Configured sample rate",
        adc_get_configured_sample_rate_hz() / 1.0e6, " MSPS");
    print_double_value("Analysis sample rate",
        adc_get_effective_sample_rate_hz() / 1.0e6, " MSPS");
    xil_printf("Rate source            : %s\r\n",
        adc_effective_sample_rate_is_valid() ?
        "known-tone measurement" : "configured value");
    xil_printf("Measured rate valid    : %s\r\n",
        adc_effective_sample_rate_is_valid() ? "YES" : "NO");
    print_double_value("Correction factor",
        adc_get_sample_rate_correction_factor(), "");
}

static void print_adc_analysis_rate_header(void)
{
    print_double_value("Configured sample rate",
        adc_get_configured_sample_rate_hz() / 1.0e6, " MSPS");
    print_double_value("Analysis sample rate",
        adc_get_effective_sample_rate_hz() / 1.0e6, " MSPS");
    xil_printf("Rate source            : %s\r\n",
        adc_effective_sample_rate_is_valid() ?
        "known-tone measurement" : "configured value");
    print_double_value("DAC/ADC rate ratio",
        DAC_SAMPLE_RATE_HZ / adc_get_effective_sample_rate_hz(), "");
}

static void print_double_value(
    const char *label,
    double value,
    const char *unit
)
{
    int32_t whole;
    int32_t fraction;
    double absolute_value = fabs(value);

    if (!isfinite(value)) {
        xil_printf("%-22s: invalid\r\n", label);
        return;
    }

    whole = (int32_t)absolute_value;
    fraction = (int32_t)((absolute_value - (double)whole) * 1000000.0);
    xil_printf("%-22s: %s%ld.%06ld%s\r\n",
               label,
               value < 0.0 ? "-" : "",
               (long)whole,
               (long)fraction,
               unit != NULL ? unit : "");
}

static void print_double_inline(double value)
{
    const double absolute = fabs(value);
    const int32_t whole = (int32_t)absolute;
    const int32_t fraction = (int32_t)((absolute - (double)whole) * 1000000.0);
    xil_printf("%s%ld.%06ld", value < 0.0 ? "-" : "",
               (long)whole, (long)fraction);
}

static int adc_analyze_guarded_overlap(
    const int16_t *reference,
    const int16_t *measurement,
    size_t sample_count,
    int32_t lag_samples,
    calibration_state_t *state,
    adc_cal_overlap_t *overlap
)
{
    calibration_config_t config;
    calibration_status_t status;

    if ((reference == NULL) || (measurement == NULL) ||
        (state == NULL) || (overlap == NULL) || (sample_count == 0U))
    {
        return -1;
    }

    memset(overlap, 0, sizeof(*overlap));

    /* timing_apply_circular_lag() uses aligned[i] = signal[i + lag]. */
    if (lag_samples >= 0)
    {
        overlap->measurement_start = (size_t)lag_samples;
        if (overlap->measurement_start >= sample_count) {
            return -2;
        }
        overlap->overlap_count =
            sample_count - overlap->measurement_start;
    }
    else
    {
        overlap->reference_start = (size_t)(-(int64_t)lag_samples);
        if (overlap->reference_start >= sample_count) {
            return -2;
        }
        overlap->overlap_count = sample_count - overlap->reference_start;
    }

    overlap->analysis_count = overlap->overlap_count;

    if (overlap->analysis_count >=
        (CAL_MIN_ANALYSIS_SAMPLES + (2U * CAL_ALIGNMENT_GUARD_SAMPLES)))
    {
        overlap->reference_start += CAL_ALIGNMENT_GUARD_SAMPLES;
        overlap->measurement_start += CAL_ALIGNMENT_GUARD_SAMPLES;
        overlap->analysis_count -= 2U * CAL_ALIGNMENT_GUARD_SAMPLES;
        overlap->applied_guard = CAL_ALIGNMENT_GUARD_SAMPLES;
    }

    if (overlap->analysis_count < CAL_MIN_ANALYSIS_SAMPLES) {
        return -3;
    }

    calibration_default_config(&config);
    status = calibration_init(state, &config);
    if (status != CALIBRATION_OK) {
        return -10 + (int)status;
    }

    status = calibration_analyze_frame(
        state,
        &measurement[overlap->measurement_start],
        &reference[overlap->reference_start],
        overlap->analysis_count
    );

    if (status != CALIBRATION_OK) {
        return -20 + (int)status;
    }

    return 0;
}

static void print_overlap_measurements(
    const calibration_state_t *state,
    const adc_cal_overlap_t *overlap
)
{
    xil_printf("Valid overlap samples  : %lu\r\n",
               (unsigned long)overlap->overlap_count);
    xil_printf("Guard samples per edge : %lu\r\n",
               (unsigned long)overlap->applied_guard);
    xil_printf("Analysis samples       : %lu\r\n",
               (unsigned long)overlap->analysis_count);
    print_float_value("Overlap correlation",
                      state->metrics.correlation, "");
    print_float_value("Reference mean",
                      state->metrics.reference_mean, " codes");
    print_float_value("Measurement mean",
                      state->metrics.adc_mean, " codes");
    print_float_value("Offset difference",
                      state->metrics.offset_error_codes, " codes");
    print_float_value("Fitted offset",
                      state->metrics.measured_offset, " codes");
    print_float_value("Relative gain",
                      state->metrics.measured_gain, "");
    print_float_value("Relative gain error",
                      state->metrics.gain_error_ratio, "");
    print_float_value("Raw aligned RMSE",
                      state->metrics.rmse_codes, " codes");
    print_float_value("Gain/offset fit RMSE",
                      state->metrics.fitted_rmse_codes, " codes");
    print_float_value("Fitted MAE",
                      state->metrics.fitted_mae_codes, " codes");
}

static void print_adc_test_configuration(size_t sample_count)
{
    const double ratio = DAC_SAMPLE_RATE_HZ / adc_get_effective_sample_rate_hz();

    print_double_value("ADC sample rate",
                       adc_get_effective_sample_rate_hz() / 1.0e6, " MHz");
    print_adc_sample_rate_state();
    print_double_value("DAC sample rate",
                       DAC_SAMPLE_RATE_HZ / 1.0e6, " MHz");
    print_double_value("DAC/ADC rate ratio", ratio, "");
    xil_printf("Capture samples       : %lu\r\n",
               (unsigned long)sample_count);
}

static void print_reference_coherence(double reference_frequency_hz,
                                      size_t sample_count)
{
    double cycles;
    double coherence_error;

    if (!isfinite(reference_frequency_hz) || reference_frequency_hz <= 0.0 ||
        !isfinite(adc_get_effective_sample_rate_hz()) ||
        adc_get_effective_sample_rate_hz() <= 0.0 || sample_count == 0U) {
        xil_printf("Reference coherence   : unavailable\r\n");
        return;
    }
    cycles = reference_frequency_hz * (double)sample_count /
             adc_get_effective_sample_rate_hz();
    coherence_error = fabs(cycles - round(cycles));
    print_double_value("Reference cycles/frame", cycles, "");
    print_double_value("Coherence error", coherence_error, " cycles");
    xil_printf("Coherence status      : %s\r\n",
        coherence_error <= CAL_COHERENCE_TOLERANCE ? "PASS" : "WARNING");
    if (coherence_error > CAL_COHERENCE_TOLERANCE)
        xil_printf("WARNING: Non-coherent capture may increase frame-to-frame metric variation.\r\n");
}

static int estimate_tone_frequency(
    const int16_t *samples,
    size_t sample_count,
    double *frequency_hz,
    double *tone_bin
)
{
    double mean = 0.0;
    double first_crossing = 0.0;
    double last_crossing = 0.0;
    size_t crossing_count = 0U;

    if ((samples == NULL) || (frequency_hz == NULL) ||
        (tone_bin == NULL) || (sample_count < 3U)) {
        return -1;
    }

    for (size_t i = 0U; i < sample_count; ++i) {
        mean += (double)samples[i];
    }
    mean /= (double)sample_count;

    for (size_t i = 0U; i + 1U < sample_count; ++i) {
        const double y0 = (double)samples[i] - mean;
        const double y1 = (double)samples[i + 1U] - mean;

        if ((y0 <= 0.0) && (y1 > 0.0)) {
            const double difference = y1 - y0;
            double crossing;

            if (fabs(difference) <= CAL_FRACTIONAL_EPSILON) {
                continue;
            }
            crossing = (double)i - (y0 / difference);
            if (crossing_count == 0U) {
                first_crossing = crossing;
            }
            last_crossing = crossing;
            ++crossing_count;
        }
    }

    if ((crossing_count < 2U) ||
        (last_crossing <= first_crossing)) {
        return -2;
    }

    *frequency_hz =
        ((double)(crossing_count - 1U) * adc_get_effective_sample_rate_hz()) /
        (last_crossing - first_crossing);
    *tone_bin = *frequency_hz * (double)sample_count /
                adc_get_effective_sample_rate_hz();

    if (!isfinite(*frequency_hz) || !isfinite(*tone_bin)) {
        return -3;
    }
    return 0;
}

static void print_frequency_validation(
    const int16_t *samples,
    size_t sample_count
)
{
    double measured_hz;
    double measured_bin;
    double measured_coherence_error;
    int status = estimate_tone_frequency(
        samples, sample_count, &measured_hz, &measured_bin
    );

    if (status != 0) {
        xil_printf("Measured frequency    : unavailable (status %d)\r\n",
                   status);
        xil_printf("Frequency validation  : WARNING\r\n");
        return;
    }

    measured_coherence_error = fabs(measured_bin - round(measured_bin));
    print_double_value("Measured tone bin", measured_bin, "");
    print_double_value("Measured tone freq", measured_hz / 1.0e6, " MHz");
    print_double_value("Measured coherence err",
                       measured_coherence_error, " cycles");
    xil_printf("Frequency measurement : PASS\r\n");
    if (measured_coherence_error > CAL_COHERENCE_TOLERANCE) {
        xil_printf("WARNING: The measured tone is not coherent with the "
                   "%lu-sample ADC window.\r\n",
                   (unsigned long)sample_count);
    }
}

static int adc_analyze_fractional_overlap(
    const int16_t *reference,
    const int16_t *measurement,
    size_t sample_count,
    double total_lag,
    int16_t *reference_work,
    int16_t *measurement_work,
    calibration_state_t *state,
    adc_cal_overlap_t *overlap
)
{
    size_t valid_count = 0U;

    if ((reference == NULL) || (measurement == NULL) ||
        (reference_work == NULL) || (measurement_work == NULL) ||
        !isfinite(total_lag)) {
        return -1;
    }

    for (size_t i = 0U; i < sample_count; ++i) {
        const double source_position = (double)i + total_lag;
        size_t lower;
        double fraction;
        double interpolated;
        long rounded;

        if ((source_position < 0.0) ||
            (source_position >= (double)(sample_count - 1U))) {
            continue;
        }

        lower = (size_t)floor(source_position);
        fraction = source_position - (double)lower;
        interpolated =
            (1.0 - fraction) * (double)measurement[lower] +
            fraction * (double)measurement[lower + 1U];

        if (!isfinite(interpolated)) {
            return -2;
        }
        rounded = lround(interpolated);
        if (rounded > INT16_MAX) rounded = INT16_MAX;
        if (rounded < INT16_MIN) rounded = INT16_MIN;

        reference_work[valid_count] = reference[i];
        measurement_work[valid_count] = (int16_t)rounded;
        ++valid_count;
    }

    if (valid_count < CAL_MIN_ANALYSIS_SAMPLES) {
        return -3;
    }

    return adc_analyze_guarded_overlap(
        reference_work,
        measurement_work,
        valid_count,
        0,
        state,
        overlap
    );
}

static const char *cal_timing_reject_reason_text(
    calibration_timing_reject_reason_t reason
)
{
    switch (reason) {
    case CAL_TIMING_REJECT_DMA:
        return "DMA capture failed";
    case CAL_TIMING_REJECT_RECONSTRUCTION:
        return "sample reconstruction failed";
    case CAL_TIMING_REJECT_INTEGER_ALIGNMENT:
        return "integer alignment failed";
    case CAL_TIMING_REJECT_FRACTIONAL_ALIGNMENT:
        return "fractional alignment failed";
    case CAL_TIMING_REJECT_INVALID_OVERLAP:
        return "invalid overlap";
    case CAL_TIMING_REJECT_LOW_CORRELATION:
        return "correlation below 0.970000";
    case CAL_TIMING_REJECT_TOO_FEW_SAMPLES:
        return "insufficient guarded overlap";
    case CAL_TIMING_REJECT_INVALID_METRIC:
        return "invalid timing metric";
    case CAL_TIMING_REJECT_NONE:
    default:
        return "none";
    }
}

static int adc_measure_timing_frame(
    const int16_t *reference,
    const int16_t *measurement,
    size_t sample_count,
    int16_t *fractional_reference_work,
    int16_t *fractional_measurement_work,
    calibration_timing_frame_result_t *result
)
{
    timing_alignment_result_t integer_alignment;
    calibration_state_t fractional_state;
    adc_cal_overlap_t fractional_overlap;
    int status;

    if (result == NULL) {
        return -1;
    }
    memset(result, 0, sizeof(*result));
    result->capture_success = 1U;

    status = timing_find_circular_lag(
        reference, measurement, sample_count, &integer_alignment
    );
    if (status != 0) {
        result->reject_reason = CAL_TIMING_REJECT_INTEGER_ALIGNMENT;
        return status;
    }
    result->integer_lag = integer_alignment.lag_samples;

    status = timing_estimate_fractional_lag(
        reference, measurement, sample_count,
        result->integer_lag, &result->fractional_lag
    );
    if (status != 0) {
        result->reject_reason = CAL_TIMING_REJECT_FRACTIONAL_ALIGNMENT;
        return status;
    }

    result->total_lag =
        (float)result->integer_lag + result->fractional_lag;
    if (!isfinite(result->fractional_lag) ||
        !isfinite(result->total_lag)) {
        result->reject_reason = CAL_TIMING_REJECT_INVALID_METRIC;
        return -2;
    }

    status = adc_analyze_fractional_overlap(
        reference, measurement, sample_count, (double)result->total_lag,
        fractional_reference_work, fractional_measurement_work,
        &fractional_state, &fractional_overlap
    );
    if (status != 0) {
        result->reject_reason =
            (status == -3) ? CAL_TIMING_REJECT_TOO_FEW_SAMPLES :
                             CAL_TIMING_REJECT_INVALID_OVERLAP;
        return status;
    }

    result->alignment_success = 1U;
    result->valid_overlap_samples =
        (uint32_t)fractional_overlap.overlap_count;
    result->analysis_samples =
        (uint32_t)fractional_overlap.analysis_count;
    result->correlation = fractional_state.metrics.correlation;
    result->raw_rmse = fractional_state.metrics.rmse_codes;
    result->fitted_rmse = fractional_state.metrics.fitted_rmse_codes;

    if (!isfinite(result->correlation) || !isfinite(result->raw_rmse)) {
        result->reject_reason = CAL_TIMING_REJECT_INVALID_METRIC;
    } else if (result->analysis_samples < CAL_TIMING_MIN_ANALYSIS_SAMPLES) {
        result->reject_reason = CAL_TIMING_REJECT_TOO_FEW_SAMPLES;
    } else if (fabsf(result->fractional_lag) >
               CAL_TIMING_MAX_ABS_FRAC_LAG) {
        result->reject_reason = CAL_TIMING_REJECT_FRACTIONAL_ALIGNMENT;
    } else if (result->correlation < CAL_TIMING_MIN_CORRELATION) {
        result->reject_reason = CAL_TIMING_REJECT_LOW_CORRELATION;
    } else {
        result->accepted = 1U;
        result->reject_reason = CAL_TIMING_REJECT_NONE;
    }

    return 0;
}

static float median_float(float *values, size_t count)
{
    if ((values == NULL) || (count == 0U)) {
        return 0.0f;
    }

    for (size_t i = 1U; i < count; ++i) {
        const float value = values[i];
        size_t j = i;
        while ((j > 0U) && (values[j - 1U] > value)) {
            values[j] = values[j - 1U];
            --j;
        }
        values[j] = value;
    }

    if ((count & 1U) != 0U) {
        return values[count / 2U];
    }
    return 0.5f * (values[(count / 2U) - 1U] + values[count / 2U]);
}

static int print_alignment_measurements(
    const int16_t *reference,
    const int16_t *measurement,
    size_t sample_count,
    int32_t integer_lag,
    int16_t *fractional_reference_work,
    int16_t *fractional_measurement_work,
    calibration_metrics_t *fractional_metrics_out,
    float *total_lag_out
)
{
    calibration_state_t integer_state;
    calibration_state_t fractional_state;
    adc_cal_overlap_t integer_overlap;
    adc_cal_overlap_t fractional_overlap;
    float fractional_lag = 0.0f;
    double total_lag;
    int status;

    status = adc_analyze_guarded_overlap(
        reference, measurement, sample_count, integer_lag,
        &integer_state, &integer_overlap
    );
    if (status != 0) {
        return status;
    }

    status = timing_estimate_fractional_lag(
        reference, measurement, sample_count, integer_lag, &fractional_lag
    );
    if (status != 0) {
        xil_printf("Fractional lag refinement warning: %d; using 0.\r\n",
                   status);
        fractional_lag = 0.0f;
    }

    total_lag = (double)integer_lag + (double)fractional_lag;
    if (!isfinite(total_lag)) {
        return -30;
    }

    status = adc_analyze_fractional_overlap(
        reference, measurement, sample_count, total_lag,
        fractional_reference_work, fractional_measurement_work,
        &fractional_state, &fractional_overlap
    );
    if (status != 0) {
        return -40 + status;
    }

    xil_printf("Integer lag samples    : %ld\r\n", (long)integer_lag);
    print_float_value("Fractional lag samples", fractional_lag, "");
    print_float_value("Total estimated lag", (float)total_lag, " samples");

    xil_printf("-- Integer-aligned overlap --\r\n");
    print_overlap_measurements(&integer_state, &integer_overlap);
    xil_printf("-- Fractional-aligned overlap --\r\n");
    print_overlap_measurements(&fractional_state, &fractional_overlap);

    if (fractional_metrics_out != NULL) {
        *fractional_metrics_out = fractional_state.metrics;
    }
    if (total_lag_out != NULL) {
        *total_lag_out = (float)total_lag;
    }

    return 0;
}

static int next_tok(char **ctx, char *out, size_t len) {
    char *t = strtok(NULL, " ");
    (void)ctx;
    if (!t) return 0;
    strncpy(out, t, len - 1);
    out[len - 1] = '\0';
    return 1;
}

static void parse_cmd_args(char *line, char *option, size_t opt_len, char *addr_str, size_t addr_len, char *data_str, size_t data_len, const char *cmd_name) {
    char *ctx = line;
    (void)cmd_name;
    strtok(ctx, " "); // skip command name
    if (!next_tok(&ctx, option, opt_len)) { ERR("Missing option (-r / -w)"); return; }
    if (!next_tok(&ctx, addr_str, addr_len)) { ERR("Missing address"); return; }
    if (!strcmp(option, "-w") && !next_tok(&ctx, data_str, data_len)) { ERR("Missing write data"); return; }
}

// Handler for SPI commands
void handle_spi_cmd(char *line) {
    char option[4], addr_str[8], data_str[4];
    uint16_t addr;
    uint8_t data;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "spi");
    addr = (uint16_t)strtol(addr_str, NULL, 0);

    if (!strcmp(option, "-r")) {
        ad9695_read_register(&spi_inst, addr, &data);
        xil_printf("Value at 0x%04X = 0x%02X\r\n", addr, data);
    } else if (!strcmp(option, "-w")) {
        data = (uint8_t)strtol(data_str, NULL, 0);
        ad9695_write_register(&spi_inst, addr, data);
        xil_printf("Command Success: Wrote 0x%02X to 0x%04X\r\n", data, addr);
    } else ERR("Invalid option '%s' (use -r or -w)", option);
}

// Handler for JESD204 PHY commands
void handle_phy_cmd(char *line) {
    char option[4], addr_str[12], data_str[12];
    uint32_t addr, data, tmp_reg;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "phy");
    addr = (uint32_t)strtoul(addr_str, NULL, 0);

    if (!strcmp(option, "-r")) {
        jesdphy_read(addr, &tmp_reg);
        xil_printf("Value at 0x%08X = 0x%08X\r\n", XPAR_JESD204_PHY_0_BASEADDR + addr, tmp_reg);
    } else if (!strcmp(option, "-w")) {
        data = (uint32_t)strtoul(data_str, NULL, 0);
        jesdphy_write(addr, data);
        xil_printf("Wrote 0x%08X to 0x%08X\r\n", data, XPAR_JESD204_PHY_0_BASEADDR + addr);
    } else ERR("Invalid option '%s' (use -r or -w)", option);
}

// Handler for JESD204 Link-layer commands
void handle_link_cmd(char *line) {
    char option[4], addr_str[12], data_str[12];
    uint32_t addr, data, tmp_reg;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "link");
    addr = (uint32_t)strtoul(addr_str, NULL, 0);

    if (!strcmp(option, "-r")) {
        jesdlink_read(addr, &tmp_reg);
        xil_printf("Value at 0x%08X = 0x%08X\r\n", XPAR_JESD204C_0_BASEADDR + addr, tmp_reg);
    } else if (!strcmp(option, "-w")) {
        data = (uint32_t)strtoul(data_str, NULL, 0);
        jesdlink_write(addr, data);
        xil_printf("Wrote 0x%08X to 0x%08X\r\n", data, XPAR_JESD204C_0_BASEADDR + addr);
    } else ERR("Invalid option '%s' (use -r or -w)", option);
}

void handle_dma_cmd(char* line) {
    char copy[MAX_UART_LINE_LENGTH];
    char option[4];

    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char* token = strtok(copy, " ");
    if (!token || strcmp(token, "dma") != 0) { ERR("Expected \"dma\""); return; }

    token = strtok(NULL, " ");
    if (!token) { ERR("Missing option (-r or -w)"); return; }
    strncpy(option, token, sizeof(option) - 1);
    option[sizeof(option) - 1] = '\0';

    if (strcmp(option, "-w") == 0) {
        xil_printf("Starting DMA capture of %d bytes...\r\n", DMA_CMD_BUF_SIZE);
        if (adc_sweep_active)
        {
            ERR("DMA commands are disabled while an ADC sweep is in progress.");
            return;
        }
        Xil_DCacheFlushRange((UINTPTR)RxBufferPtr, DMA_CMD_BUF_SIZE);
        int res =XAxiDma_SimpleTransfer(&dma_inst, (UINTPTR) RxBufferPtr,
                        DMA_CMD_BUF_SIZE, XAXIDMA_DEVICE_TO_DMA);

        if (res != XST_SUCCESS) { ERR("XAxiDma_SimpleTransfer failed. Error Code: %d.", res); return; }
        u32 timeout = 100000;
        int busy;
        do {
            busy = XAxiDma_Busy(&dma_inst, XAXIDMA_DEVICE_TO_DMA);
            if(!busy){ break; }
            timeout --;
            usleep(1);
        }while(timeout > 0);
        if (busy) { xil_printf("DMA was still busy and timed out.\r\n"); }
        else { 
            xil_printf("DMA Finished Successfully.\r\n"); 
            Xil_DCacheInvalidateRange((UINTPTR)RxBufferPtr, DMA_CMD_BUF_SIZE);
        }
        xil_printf("dma -w complete.\r\n");
    } else if (strcmp(option, "-r") == 0) {
            Xil_DCacheInvalidateRange((UINTPTR)RxBufferPtr, DMA_CMD_BUF_SIZE);
        if (adc_sweep_active)
        {
            ERR("DMA commands are disabled while an ADC sweep is in progress.");
            return;
        }
        xil_printf("Reading back %d bytes:\r\n", DMA_CMD_BUF_SIZE);
        for (uint32_t i = 0; i < DMA_CMD_BUF_SIZE; i+=16) {
            xil_printf("@0x%02X = 0x%02X ", i, RxBufferPtr[i]);
            xil_printf("\r\n");
        }
        xil_printf("\r\n");
    } else if (strcmp(option, "-d") == 0) {
        if (adc_sweep_active)
        {
            ERR("DMA commands are disabled while an ADC sweep is in progress.");
            return;
        }
        XAxiDma_Reset(&dma_inst);
        xil_printf("reset completed!\r\n");
    } else if (strcmp(option, "-c") == 0) {
        if (adc_sweep_active)
        {
            ERR("DMA commands are disabled while an ADC sweep is in progress.");
            return;
        }
        XAxiDma_Resume(&dma_inst);
        xil_printf("resume completed!\r\n");
    } else { ERR("Invalid option \"%s\" (use -r or -w or -d)", option); }
}

void handle_mem_cmd(char* line) {
    char option[4], addr_str[12], data_str[12];
    uint32_t addr, data, tmp_reg;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "mem");
    addr = (uint32_t)strtoul(addr_str, NULL, 0);

    if (strcmp(option, "-r") == 0) {
        tmp_reg = Xil_In32(addr);
        xil_printf("Command Success: Value at 0x%08X = 0x%08X\r\n", addr, tmp_reg);
    } else if (strcmp(option, "-w") == 0) {
        data = (uint32_t)strtoul(data_str, NULL, 0);
        Xil_Out32(addr, data);
        xil_printf("Command Success: Wrote 0x%08X to 0x%08X\r\n", data, addr);
    } else { ERR("Invalid option \"%s\" (use -r or -w)", option); }
}

#define DMA_CTRL_BASE XPAR_AXI_DMA_0_BASEADDR // Unused macro

void handle_dma_dbg_cmd(char* line) {
    char option[4], addr_str[12], data_str[12];
    uint32_t offset, data, reg_val;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "dbg");
    offset = (uint32_t)strtoul(addr_str, NULL, 0);
    UINTPTR addr = (UINTPTR)(RxBufferPtr + offset);

    if (strcmp(option, "-r") == 0) {
        reg_val = Xil_In32(addr);
        xil_printf("Command Success: DMA[0x%08X] = 0x%08X\r\n", offset, reg_val);
    } else if (strcmp(option, "-w") == 0) {
        data = (uint32_t)strtoul(data_str, NULL, 0);
        Xil_Out32(addr, data);
        xil_printf("Command Success: Wrote 0x%08X to DMA[0x%08X]\r\n", data, offset);
    } else { ERR("Invalid option \"%s\" (use -r or -w)", option); }
} 

void handle_udp_cmd(char *line)
{
    (void)line;

    if (adc_sweep_active)
    {
        ERR("UDP transmission is disabled while an ADC sweep is in progress.");
        return;
    }

    uart_send_flag = 1;
}

void handle_adc_cmd(char* line)
{
    char copy[MAX_UART_LINE_LENGTH];
    char option[16];

    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char* token = strtok(copy, " ");
    if (!token || strcmp(token, "adc") != 0) { ERR("Expected \"adc\""); return; }

    token = strtok(NULL, " ");
    if (!token) { ERR("Missing option"); return; }
    strncpy(option, token, sizeof(option) - 1);
    option[sizeof(option) - 1] = '\0';

    if(strcmp(option, "-c") == 0){
        int timeout = 10;
        uint8_t pll_stat;
        struct jesdphy_pll_status phy_pll_status;

        do {
            usleep(10000);
            ad9695_jesd_get_pll_status(&pll_stat);
        } while (!(pll_stat & AD9695_JESD_PLL_LOCK_STAT) && timeout--);

        xil_printf("ad9695 PLL %s\r\n", (pll_stat & AD9695_JESD_PLL_LOCK_STAT) ? "LOCKED" : "UNLOCKED");
        jesdphy_get_pll_status(&phy_pll_status);
        jesdphy_check_pll_status(&phy_pll_status);
    } else if (strcmp(token, "?") == 0 || strcmp(token, "status") == 0){
        uint8_t r701, r73b;

        ad9695_read_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL, &r701);
        ad9695_read_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL2, &r73b);

        if ((r701 & AD9695_DC_OFFSET_CAL_EN) && ((r73b & AD9695_DC_OFFSET_CAL_EN) == 0))
        {
            xil_printf("DC offset calibration: ON\r\n");
        }
        else
        {
            xil_printf("DC offset calibration: OFF\r\n");
        }

        xil_printf("0x0701 = 0x%02X\r\n", r701);
        xil_printf("0x073B = 0x%02X\r\n", r73b);
    } else if (strcmp(option, "-timing") == 0) {
        uint32_t frame_count = ADC_TIMING_DEFAULT_FRAMES;

        token = strtok(NULL, " ");
        if (token != NULL)
        {
            char *endptr = NULL;
            unsigned long parsed = strtoul(token, &endptr, 0);

            if ((endptr == token) || (*endptr != '\0') ||
                (parsed == 0) || (parsed > ADC_TIMING_MAX_FRAMES))
            {
                ERR("Invalid timing frame count. Use 1 to %u.",
                    ADC_TIMING_MAX_FRAMES);
                return;
            }

            frame_count = (uint32_t)parsed;
        }

        adc_timing_capture(frame_count);

    } else if (strcmp(option, "-gain") == 0) {
        
        handle_adc_gain_cmd();

    } else if (strcmp(option, "-offset") == 0)
    {
        handle_adc_offset_cmd();
    } else if (strcmp(option, "-cal") == 0) {
        uint32_t frame_count = ADC_CAL_DEFAULT_FRAMES;
        token = strtok(NULL, " ");
        if (token == NULL) {
            handle_adc_calibration_cmd(frame_count);
            return;
        }
        if (strcmp(token, "offset") == 0) {
            if (strtok(NULL, " ") != NULL) {
                ERR("Use adc -cal offset.");
                return;
            }
            handle_adc_offset_calibration_loop_cmd();
            return;
        }
        if (strcmp(token, "gain") == 0) {
            if (strtok(NULL, " ") != NULL) {
                ERR("Use adc -cal gain.");
                return;
            }
            handle_adc_gain_calibration_loop_cmd();
            return;
        }
        if (strcmp(token, "status") == 0) {
            if (strtok(NULL, " ") != NULL) {
                ERR("Use adc -cal status.");
                return;
            }
            handle_adc_offset_calibration_status_cmd();
            handle_adc_gain_calibration_status_cmd();
            return;
        }
        if (strcmp(token, "reset") == 0) {
            if (strtok(NULL, " ") != NULL) {
                ERR("Use adc -cal reset.");
                return;
            }
            calibration_all_loops_reset();
            xil_printf("ADC software gain and offset calibration states reset.\r\n");
            handle_adc_offset_calibration_status_cmd();
            handle_adc_gain_calibration_status_cmd();
            return;
        }

        if (token != NULL)
        {
            char *endptr = NULL;
            unsigned long parsed = strtoul(token, &endptr, 0);
            if ((endptr == token) || (*endptr != '\0') ||
                (parsed < ADC_CAL_MIN_FRAMES) ||
                (parsed > ADC_CAL_MAX_FRAMES))
            {
                ERR("Invalid calibration frame count. Use %u to %u.",
                    ADC_CAL_MIN_FRAMES, ADC_CAL_MAX_FRAMES);
                return;
            }
            frame_count = (uint32_t)parsed;
        }
        if (strtok(NULL, " ") != NULL) {
            ERR("Too many arguments for adc -cal.");
            return;
        }
        handle_adc_calibration_cmd(frame_count);
    } else if (strcmp(option, "-ref") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL) {
            handle_adc_reference_status_cmd();
        } else if ((strcmp(token, "diagnose") == 0) &&
                   (strtok(NULL, " ") == NULL)) {
            calibration_run_adc_reference_diagnostic();
        } else {
            ERR("Use adc -ref or adc -ref diagnose.");
        }
    }else {
        ERR("Invalid option \"%s\" (use -c, status, -timing [frames], -gain, -offset, -cal [frames|gain|offset|status|reset], -ref, or -ref diagnose)", option);
    }
}

typedef void (*cmd_fn)(char *line);
static const struct { const char *name; cmd_fn fn; } cmd_table[] = {
    { "spi",  handle_spi_cmd  },
    { "phy",  handle_phy_cmd  },
    { "link", handle_link_cmd },
    { "dma",  handle_dma_cmd  },
    { "dbg",  handle_dma_dbg_cmd  },
    { "mem",  handle_mem_cmd  },
    { "udp",  handle_udp_cmd  },
    { "adc",  handle_adc_cmd  }
};

void handle_cmd(char *line) {
    if (!line || !*line) { ERR("empty command"); return; }

    char cmd[8];
    strncpy(cmd, line, sizeof cmd - 1);
    cmd[sizeof cmd - 1] = '\0';
    char *space = strchr(cmd, ' ');
    if (space) *space = '\0';

    for (size_t i = 0; i < sizeof cmd_table / sizeof cmd_table[0]; ++i) {
        if (!strcmp(cmd, cmd_table[i].name)) { cmd_table[i].fn(line); return; }
    }
    xil_printf("Invalid command type: %s\r\n", cmd);
}

void handle_adc_gain_cmd(void)
{
    char line[MAX_UART_LINE_LENGTH];
    char copy[MAX_UART_LINE_LENGTH];
    char *token;

    xil_printf("\r\nEntering ADC Gain setting menu\r\n");
    xil_printf("Available commands:\r\n");
    xil_printf("  IFC   Input full-scale mode\r\n");
    xil_printf("  help  Print this menu\r\n");
    xil_printf("  back  Quit gain setting menu\r\n");

    while (1)
    {
        xil_printf("gain-cmd$: ");
        uart_get_line(line);

        strncpy(copy, line, sizeof(copy) - 1);
        copy[sizeof(copy) - 1] = '\0';

        token = strtok(copy, " ");

        if (!token)
            continue;

        if (strcmp(token, "back") == 0 || strcmp(token, "quit") == 0 || strcmp(token, "exit") == 0)
        {
            xil_printf("Leaving ADC Gain setting menu.\r\n");
            return;
        }

        else if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0)
        {
            xil_printf("\r\nADC Gain setting menu\r\n");
            xil_printf("Available commands:\r\n");
            xil_printf("  IFC   Input full-scale mode\r\n");
            xil_printf("  back  Quit gain setting menu\r\n");
        }

        else if (strcmp(token, "IFC") == 0 || strcmp(token, "ifc") == 0)
        {
            xil_printf("\r\nGain setting IFC mode\r\n");
            xil_printf("Input full-scale control changes ADC sensitivity using register 0x1910.\r\n");
            xil_printf("Smaller full-scale voltage gives larger digital sample amplitude.\r\n");
            xil_printf("\r\nAvailable commands:\r\n");
            xil_printf("  set <num>   Set input full-scale value\r\n");
            xil_printf("              Range: 1.36 largest amplitude  -->  2.04 smallest amplitude\r\n");
            xil_printf("              Valid values: 1.36, 1.47, 1.59, 1.70, 1.81, 1.93, 2.04\r\n");
            xil_printf("  status      Check current input full-scale status\r\n");
            xil_printf("  back        Back to gain mode selection\r\n");
            xil_printf("  quit        Quit gain setting menu\r\n");
            xil_printf("  sweep       Run sweep test across the IFC range\r\n");

            while (1)
            {
                xil_printf("gain-ifc$: ");
                uart_get_line(line);

                strncpy(copy, line, sizeof(copy) - 1);
                copy[sizeof(copy) - 1] = '\0';

                token = strtok(copy, " ");

                if (!token)
                    continue;

                if (strcmp(token, "back") == 0 || strcmp(token, "exit") == 0)
                {
                    xil_printf("Back to gain mode selection.\r\n");
                    break;
                }

                else if (strcmp(token, "quit") == 0)
                {
                    xil_printf("Leaving ADC Gain setting menu.\r\n");
                    return;
                }

                else if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0)
                {
                    xil_printf("\r\nGain setting IFC mode\r\n");
                    xil_printf("Available commands:\r\n");
                    xil_printf("  set <num>   Set input full-scale value\r\n");
                    xil_printf("              Range: 1.36 largest amplitude  -->  2.04 smallest amplitude\r\n");
                    xil_printf("              Valid values: 1.36, 1.47, 1.59, 1.70, 1.81, 1.93, 2.04\r\n");
                    xil_printf("  status      Check current input full-scale status\r\n");
                    xil_printf("  back        Back to gain mode selection\r\n");
                    xil_printf("  quit        Quit gain setting menu\r\n");
                    xil_printf("  sweep       Sweep all supported IFC values\r\n");
                }

                else if (strcmp(token, "status") == 0)
                {
                    ad9695_print_input_full_scale_status();
                }

                else if (strcmp(token, "set") == 0)
                {
                    token = strtok(NULL, " ");

                    if (!token)
                    {
                        xil_printf("Missing value. Example: set 1.59\r\n");
                    }
                    else
                    {
                        ad9695_set_input_full_scale(token);
                    }
                } 
                
                else if (strcmp(token, "sweep") == 0)
                {
                    adc_ifc_sweep();
                }

                else
                {
                    xil_printf("Invalid IFC command. Use set <num>, status, back, quit, or help.\r\n");
                }
            }
        }

        else
        {
            xil_printf("Invalid gain command. Use IFC, back, or help.\r\n");
        }
    }
}

void handle_adc_offset_cmd(void)
{
    char line[MAX_UART_LINE_LENGTH];
    char copy[MAX_UART_LINE_LENGTH];
    char *token;

    xil_printf("\r\nEntering ADC DC Offset Calibration menu\r\n");
    xil_printf("DC offset calibration removes the average DC bias from the ADC output.\r\n");
    xil_printf("Correction range is approximately +/-512 ADC codes.\r\n");
    xil_printf("\r\nAvailable commands:\r\n");
    xil_printf("  on        Enable DC offset calibration\r\n");
    xil_printf("  off       Disable DC offset calibration\r\n");
    xil_printf("  status    Check current calibration status\r\n");
    xil_printf("  help      Print this menu\r\n");
    xil_printf("  back      Return to UART command prompt\r\n");

    while (1)
    {
        xil_printf("offset-cmd$: ");
        uart_get_line(line);

        strncpy(copy, line, sizeof(copy) - 1);
        copy[sizeof(copy) - 1] = '\0';

        token = strtok(copy, " ");

        if (!token)
            continue;

        if (strcmp(token, "back") == 0 || strcmp(token, "exit") == 0)
        {
            xil_printf("Leaving ADC DC Offset Calibration menu.\r\n");
            return;
        }

        else if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0)
        {
            xil_printf("\r\nADC DC Offset Calibration menu\r\n");
            xil_printf("Available commands:\r\n");
            xil_printf("  on        Enable DC offset calibration\r\n");
            xil_printf("  off       Disable DC offset calibration\r\n");
            xil_printf("  status    Check current calibration status\r\n");
            xil_printf("  help      Print this menu\r\n");
            xil_printf("  back      Return to UART command prompt\r\n");
        }

        else if (strcmp(token, "on") == 0)
        {
            ad9695_adc_set_dc_offset_filt_en(1);
            xil_printf("DC offset calibration enabled.\r\n");
        }

        else if (strcmp(token, "off") == 0)
        {
            ad9695_adc_set_dc_offset_filt_en(0);
            xil_printf("DC offset calibration disabled.\r\n");
        }

        else if (strcmp(token, "status") == 0)
        {
            uint8_t r701;
            uint8_t r73b;

            ad9695_read_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL, &r701);
            ad9695_read_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL2, &r73b);

            xil_printf("\r\nDC Offset Calibration Status\r\n");

            if ((r701 & AD9695_DC_OFFSET_CAL_EN) &&
                ((r73b & AD9695_DC_OFFSET_CAL_EN) == 0))
            {
                xil_printf("Status           : ON\r\n");
            }
            else
            {
                xil_printf("Status           : OFF\r\n");
            }

            xil_printf("Register 0x0701  : 0x%02X\r\n", r701);
            xil_printf("Register 0x073B  : 0x%02X\r\n", r73b);
        }

        else
        {
            xil_printf("Invalid offset command. Use on, off, status, help, or back.\r\n");
        }
    }
}

int adc_capture_frame(void)
{
    int res;
    u32 timeout;
    u32 status;

    /*
     * Match the working manual sequence:
     *   dma -d
     *   wait
     *   dma -w
     */
    xil_printf("Resetting DMA...\r\n");

    XAxiDma_Reset(&dma_inst);

    timeout = 1000000;

    while (!XAxiDma_ResetIsDone(&dma_inst))
    {
        if (--timeout == 0)
        {
            ERR("DMA reset timeout.");
            return XST_FAILURE;
        }

        usleep(1);
    }

    /*
     * The manual command has a natural delay before dma -w.
     * Give the DMA hardware time to settle after reset.
     */
    usleep(100000);  /* 100 ms */

    status = XAxiDma_ReadReg(
        dma_inst.RegBase,
        XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET
    );

    xil_printf("DMA status after reset: 0x%08X\r\n", status);

    /*
     * Prepare destination buffer.
     */
    Xil_DCacheFlushRange(
        (UINTPTR)RxBufferPtr,
        DMA_CMD_BUF_SIZE
    );

    xil_printf("Starting DMA capture of %d bytes...\r\n",
               DMA_CMD_BUF_SIZE);

    res = XAxiDma_SimpleTransfer(
        &dma_inst,
        (UINTPTR)RxBufferPtr,
        DMA_CMD_BUF_SIZE,
        XAXIDMA_DEVICE_TO_DMA
    );

    if (res != XST_SUCCESS)
    {
        status = XAxiDma_ReadReg(
            dma_inst.RegBase,
            XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET
        );

        ERR(
            "XAxiDma_SimpleTransfer failed. Error code: %d, "
            "S2MM status: 0x%08X",
            res,
            status
        );

        return XST_FAILURE;
    }

    /*
     * Wait for the capture to finish.
     */
    timeout = 1000000;

    while (XAxiDma_Busy(&dma_inst, XAXIDMA_DEVICE_TO_DMA))
    {
        if (--timeout == 0)
        {
            status = XAxiDma_ReadReg(
                dma_inst.RegBase,
                XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET
            );

            ERR("DMA capture timeout. S2MM status: 0x%08X", status);
            return XST_FAILURE;
        }

        usleep(1);
    }

    Xil_DCacheInvalidateRange(
        (UINTPTR)RxBufferPtr,
        DMA_CMD_BUF_SIZE
    );

    xil_printf("DMA capture complete.\r\n");

    return XST_SUCCESS;
}

void adc_timing_capture(uint32_t frame_count)
{
    static int16_t timing_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t reconstructed_samples[ADC_VALID_SAMPLE_COUNT];
    static int16_t aligned_samples[ADC_VALID_SAMPLE_COUNT];
    static int16_t fractional_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t fractional_measurement[ADC_VALID_SAMPLE_COUNT];

    uint32_t successful_captures = 0U;
    size_t reference_count = 0U;
    size_t reconstructed_count = 0U;
    timing_analysis_result_t result;
    int reconstruction_status;
    int timing_status;
    int overlap_status;

    if ((frame_count == 0U) || (frame_count > ADC_TIMING_MAX_FRAMES))
    {
        ERR("Timing frame count must be between 1 and %u.",
            ADC_TIMING_MAX_FRAMES);
        return;
    }

    if (adc_sweep_active)
    {
        ERR("Another automatic ADC capture is already in progress.");
        return;
    }

    adc_sweep_active = 1;

    xil_printf("\r\n");
    xil_printf("===================================\r\n");
    xil_printf("Starting On-Board ADC Timing Test\r\n");
    xil_printf("Requested frames : %lu\r\n",
               (unsigned long)frame_count);
    xil_printf("Sample count     : %u\r\n",
               ADC_VALID_SAMPLE_COUNT);
    xil_printf("Analysis         : circular lag + fitted RMSE\r\n");
    xil_printf("===================================\r\n");
    print_adc_test_configuration(ADC_VALID_SAMPLE_COUNT);

    /*
     * Frame 1 becomes the fixed on-board timing reference.
     */
    if (adc_capture_frame() != XST_SUCCESS)
    {
        xil_printf("Timing reference capture failed.\r\n");
        adc_sweep_active = 0;
        return;
    }

    reconstruction_status = adc_reconstruct_frame(
        RxBufferPtr,
        DMA_CMD_BUF_SIZE,
        timing_reference,
        ADC_VALID_SAMPLE_COUNT,
        &reference_count
    );

    if (reconstruction_status != 0)
    {
        xil_printf(
            "Timing reference reconstruction failed: %d\r\n",
            reconstruction_status
        );
        adc_sweep_active = 0;
        return;
    }

    if (reference_count != ADC_TEST_CAPTURE_SAMPLES)
    {
        xil_printf("Timing reference rejected: expected %u samples, got %lu.\r\n",
                   (unsigned int)ADC_TEST_CAPTURE_SAMPLES,
                   (unsigned long)reference_count);
        adc_sweep_active = 0;
        return;
    }

    successful_captures = 1U;
    print_frequency_validation(timing_reference, reference_count);

    /*
     * Analyze frame 1 against itself to include the reference frame in the
     * same on-board timing summary format.
     */
    timing_status = timing_analyze_frame(
        timing_reference,
        timing_reference,
        reference_count,
        aligned_samples,
        &result
    );

    if (timing_status != 0)
    {
        xil_printf(
            "Timing reference analysis failed: %d\r\n",
            timing_status
        );
        adc_sweep_active = 0;
        return;
    }

    xil_printf("\r\n[TIMING_RESULT 1/%lu]\r\n",
               (unsigned long)frame_count);
    xil_printf("Sample count           : %lu\r\n",
               (unsigned long)result.sample_count);
    xil_printf("Lag samples            : %ld\r\n",
               (long)result.lag_samples);
    print_float_value("Correlation", result.correlation, "");
    print_float_value("Fitted scale", result.fitted_scale, "");
    print_float_value("Fitted offset",
                      result.fitted_offset, " codes");
    print_float_value("Aligned fitted RMSE",
                      result.aligned_rmse_codes, " codes");
    print_float_value("Raw mean", result.signal_mean, " codes");
    print_float_value("Raw RMS", result.signal_rms, " codes");
    xil_printf("Raw minimum            : %d codes\r\n",
               (int)result.signal_min);
    xil_printf("Raw maximum            : %d codes\r\n",
               (int)result.signal_max);
    overlap_status = print_alignment_measurements(
        timing_reference,
        timing_reference,
        reference_count,
        result.lag_samples,
        fractional_reference,
        fractional_measurement,
        NULL,
        NULL
    );
    if (overlap_status != 0) {
        xil_printf("Guarded overlap analysis failed: %d\r\n",
                   overlap_status);
    }
    xil_printf("Timing status          : PASS\r\n");

    for (uint32_t frame = 2U; frame <= frame_count; frame++)
    {
        xil_printf("\r\n[TIMING_FRAME_BEGIN %lu/%lu]\r\n",
                   (unsigned long)frame,
                   (unsigned long)frame_count);

        if (adc_capture_frame() != XST_SUCCESS)
        {
            xil_printf("[TIMING_FRAME_FAILED %lu]\r\n",
                       (unsigned long)frame);
            continue;
        }

        reconstruction_status = adc_reconstruct_frame(
            RxBufferPtr,
            DMA_CMD_BUF_SIZE,
            reconstructed_samples,
            ADC_VALID_SAMPLE_COUNT,
            &reconstructed_count
        );

        if (reconstruction_status != 0)
        {
            xil_printf(
                "Frame %lu reconstruction failed: %d\r\n",
                (unsigned long)frame,
                reconstruction_status
            );
            continue;
        }

        if (reconstructed_count != reference_count)
        {
            xil_printf(
                "Frame %lu rejected: sample count mismatch "
                "(%lu versus %lu).\r\n",
                (unsigned long)frame,
                (unsigned long)reconstructed_count,
                (unsigned long)reference_count
            );
            continue;
        }

        timing_status = timing_analyze_frame(
            timing_reference,
            reconstructed_samples,
            reconstructed_count,
            aligned_samples,
            &result
        );

        if (timing_status != 0)
        {
            xil_printf(
                "Frame %lu timing analysis failed: %d\r\n",
                (unsigned long)frame,
                timing_status
            );
            continue;
        }

        successful_captures++;

        xil_printf("[TIMING_RESULT %lu/%lu]\r\n",
                   (unsigned long)frame,
                   (unsigned long)frame_count);
        xil_printf("Sample count           : %lu\r\n",
                   (unsigned long)result.sample_count);
        xil_printf("Lag samples            : %ld\r\n",
                   (long)result.lag_samples);
        print_float_value("Correlation", result.correlation, "");
        print_float_value("Fitted scale", result.fitted_scale, "");
        print_float_value("Fitted offset",
                          result.fitted_offset, " codes");
        print_float_value("Aligned fitted RMSE",
                          result.aligned_rmse_codes, " codes");
        print_float_value("Raw mean", result.signal_mean, " codes");
        print_float_value("Raw RMS", result.signal_rms, " codes");
        xil_printf("Raw minimum            : %d codes\r\n",
                   (int)result.signal_min);
        xil_printf("Raw maximum            : %d codes\r\n",
                   (int)result.signal_max);

        overlap_status = print_alignment_measurements(
            timing_reference,
            reconstructed_samples,
            reconstructed_count,
            result.lag_samples,
            fractional_reference,
            fractional_measurement,
            NULL,
            NULL
        );
        if (overlap_status != 0) {
            xil_printf("Guarded overlap analysis failed: %d\r\n",
                       overlap_status);
        }

        if (result.correlation < ADC_TIMING_MIN_CORRELATION)
        {
            xil_printf(
                "Timing status          : REJECTED "
                "(correlation below threshold)\r\n"
            );
        }
        else
        {
            xil_printf("Timing status          : PASS\r\n");
        }

        xil_printf("[TIMING_FRAME_END %lu]\r\n",
                   (unsigned long)frame);

        if (frame < frame_count)
        {
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
        }
    }

    adc_sweep_active = 0;

    xil_printf("\r\n");
    xil_printf("===================================\r\n");
    xil_printf("On-board timing test finished.\r\n");
    xil_printf("Analyzed captures : %lu/%lu\r\n",
               (unsigned long)successful_captures,
               (unsigned long)frame_count);
    xil_printf("No UDP receiver was required.\r\n");
    xil_printf("===================================\r\n");
}

static void adc_ifc_sweep(void)
{
    static const char *ifc_values[] =
    {
        "2.04",
        "1.93",
        "1.81",
        "1.70",
        "1.59",
        "1.47",
        "1.36"
    };

    const int number_of_steps =
        sizeof(ifc_values) / sizeof(ifc_values[0]);

    int successful_captures = 0;
    int transmitted_frames = 0;

    /*
     * Prevent manual DMA or UDP commands from interfering with
     * the automatic sweep.
     */
    adc_sweep_active = 1;

    xil_printf("\r\n");
    xil_printf("===============================\r\n");
    xil_printf("Starting Input Full-Scale Sweep\r\n");
    xil_printf("===============================\r\n");

    for (int i = 0; i < number_of_steps; i++)
    {
        xil_printf("----------------------------------\r\n");
        xil_printf(
            "Step %d of %d\r\n",
            i + 1,
            number_of_steps
        );

        xil_printf(
            "Input Full-Scale : %s Vpp\r\n",
            ifc_values[i]
        );

        /*
         * Program the AD9695 input full-scale register.
         */
        ad9695_set_input_full_scale(ifc_values[i]);

        /*
         * Allow the ADC analog and digital datapaths to settle.
         */
        usleep(200000);  /* 200 ms */

        /*
         * Reset DMA, capture one frame, wait until complete,
         * and invalidate the data cache.
         */
        if (adc_capture_frame() != XST_SUCCESS)
        {
            xil_printf(
                "Capture failed for %s Vpp.\r\n",
                ifc_values[i]
            );

            continue;
        }

        successful_captures++;

        /*
         * adc_capture_frame() already invalidates the cache after
         * DMA completion. Repeating it here is harmless and ensures
         * udp_send_mem() reads the newest samples from DDR.
         */
        Xil_DCacheInvalidateRange(
            (UINTPTR)RxBufferPtr,
            DMA_CMD_BUF_SIZE
        );

        xil_printf("Transmitting frame...\r\n");

        /*
         * Use the exact UDP function already proven to work.
         *
         * Do not set uart_send_flag and wait for it. The flag is
         * normally processed by udp_update() in the main loop, but
         * the sweep blocks that main loop until it returns.
         */
        udp_send_mem();

        transmitted_frames++;

        xil_printf(
            "Transmission complete for %s Vpp.\r\n",
            ifc_values[i]
        );

        /*
         * Small separation between complete frames so the host receiver can
         * finish storing the current frame.
         */
        usleep(100000);  /* 100 ms */
    }

    adc_sweep_active = 0;

    xil_printf("\r\n");
    xil_printf("=================================\r\n");
    xil_printf("IFC sweep finished.\r\n");

    xil_printf(
        "Successful captures     : %d/%d\r\n",
        successful_captures,
        number_of_steps
    );

    xil_printf(
        "Transmitted frames      : %d/%d\r\n",
        transmitted_frames,
        number_of_steps
    );

    xil_printf("=================================\r\n");
}

static double calibration_reference_variance(
    const int16_t *samples,
    size_t sample_count
)
{
    double mean = 0.0;
    double sum = 0.0;

    if ((samples == NULL) || (sample_count == 0U)) return 0.0;
    for (size_t i = 0U; i < sample_count; ++i) mean += samples[i];
    mean /= (double)sample_count;
    for (size_t i = 0U; i < sample_count; ++i) {
        const double centered = (double)samples[i] - mean;
        sum += centered * centered;
    }
    return sum / (double)sample_count;
}

static int calibration_build_adc_reference_from_raw_dac(
    const int16_t *raw_dac,
    size_t raw_count,
    int16_t *even_reference,
    int16_t *odd_reference,
    size_t output_capacity,
    size_t *reconstructed_count
)
{
    const double source_step = DAC_SAMPLE_RATE_HZ /
                               adc_get_effective_sample_rate_hz();

    if ((raw_dac == NULL) || (even_reference == NULL) ||
        (odd_reference == NULL) || (reconstructed_count == NULL) ||
        (output_capacity == 0U)) return -1;

    if (!(source_step > 0.0) || !isfinite(source_step)) return -2;
    for (size_t i = 0U; i < output_capacity; ++i) {
        const double base_position = (double)i * source_step;
        const double positions[2] = {base_position, base_position + 1.0};
        int16_t *outputs[2] = {even_reference, odd_reference};
        for (size_t phase = 0U; phase < 2U; ++phase) {
            const double position = fmod(positions[phase], (double)raw_count);
            const size_t index0 = (size_t)floor(position);
            const size_t index1 = (index0 + 1U) % raw_count;
            const double fraction = position - (double)index0;
            long interpolated;
            if (index0 >= raw_count) return -3;
            interpolated = lround((1.0 - fraction) * raw_dac[index0] +
                                  fraction * raw_dac[index1]);
            if (interpolated > INT16_MAX) interpolated = INT16_MAX;
            if (interpolated < INT16_MIN) interpolated = INT16_MIN;
            outputs[phase][i] = calibration_convert_reference_to_adc_units(
                (int16_t)interpolated);
        }
    }
    *reconstructed_count = output_capacity;
    return 0;
}

typedef struct {
    size_t bin;
    double magnitude;
} calibration_spectral_peak_t;

typedef struct {
    size_t dominant_bin;
    double dominant_frequency_hz;
    double bin245_magnitude;
    calibration_spectral_peak_t peaks[CAL_REF_SPECTRAL_PEAK_COUNT];
} calibration_spectrum_t;

static double calibration_goertzel_magnitude(
    const int16_t *samples,
    size_t sample_count,
    size_t bin,
    double mean
)
{
    const double omega = 6.28318530717958647692 * (double)bin /
                         (double)sample_count;
    const double coefficient = 2.0 * cos(omega);
    double previous = 0.0;
    double previous2 = 0.0;

    for (size_t i = 0U; i < sample_count; ++i) {
        const double current = ((double)samples[i] - mean) +
                               coefficient * previous - previous2;
        previous2 = previous;
        previous = current;
    }
    {
        double power = previous2 * previous2 + previous * previous -
                       coefficient * previous * previous2;
        if (power < 0.0 && power > -1.0e-6) power = 0.0;
        return power > 0.0 ? sqrt(power) / (double)sample_count : 0.0;
    }
}

static int calibration_calculate_full_spectrum(
    const int16_t *samples,
    size_t sample_count,
    double sample_rate_hz,
    calibration_spectrum_t *spectrum
)
{
    double mean = 0.0;

    if ((samples == NULL) || (spectrum == NULL) || (sample_count < 4U))
        return -1;
    memset(spectrum, 0, sizeof(*spectrum));
    for (size_t i = 0U; i < sample_count; ++i) mean += samples[i];
    mean /= (double)sample_count;

    for (size_t bin = 1U; bin < sample_count / 2U; ++bin) {
        const double magnitude = calibration_goertzel_magnitude(
            samples, sample_count, bin, mean);
        if (bin == 245U) spectrum->bin245_magnitude = magnitude;
        for (size_t rank = 0U; rank < CAL_REF_SPECTRAL_PEAK_COUNT; ++rank) {
            if (magnitude > spectrum->peaks[rank].magnitude) {
                for (size_t move = CAL_REF_SPECTRAL_PEAK_COUNT - 1U;
                     move > rank; --move)
                    spectrum->peaks[move] = spectrum->peaks[move - 1U];
                spectrum->peaks[rank].bin = bin;
                spectrum->peaks[rank].magnitude = magnitude;
                break;
            }
        }
    }
    spectrum->dominant_bin = spectrum->peaks[0].bin;
    spectrum->dominant_frequency_hz =
        (double)spectrum->dominant_bin * sample_rate_hz /
        (double)sample_count;
    return spectrum->dominant_bin != 0U ? 0 : -2;
}

static void calibration_print_spectrum(
    const char *name,
    const int16_t *samples,
    size_t sample_count,
    double sample_rate_hz,
    calibration_spectrum_t *spectrum
)
{
    xil_printf("\r\n---------- %s Spectrum ----------\r\n", name);
    if (calibration_calculate_full_spectrum(
            samples, sample_count, sample_rate_hz, spectrum) != 0) {
        xil_printf("Spectrum status         : FAIL\r\n");
        return;
    }
    xil_printf("Dominant bin            : %lu\r\n",
               (unsigned long)spectrum->dominant_bin);
    print_double_value("Dominant frequency",
                       spectrum->dominant_frequency_hz / 1.0e6, " MHz");
    xil_printf("Rank   Bin   Frequency MHz   Magnitude\r\n");
    for (size_t rank = 0U; rank < CAL_REF_SPECTRAL_PEAK_COUNT; ++rank) {
        const calibration_spectral_peak_t *peak = &spectrum->peaks[rank];
        xil_printf("%-6lu %-5lu ", (unsigned long)(rank + 1U),
                   (unsigned long)peak->bin);
        print_double_inline((double)peak->bin * sample_rate_hz /
                            (double)sample_count / 1.0e6);
        xil_printf("          ");
        print_double_inline(peak->magnitude);
        xil_printf("\r\n");
    }
}

static float calibration_best_reference_correlation(
    const int16_t *candidate,
    size_t candidate_count,
    const int16_t *even_reference,
    const int16_t *odd_reference,
    size_t reference_stride
)
{
    static int16_t reference_work[ADC_VALID_SAMPLE_COUNT];
    timing_alignment_result_t result;
    float best = -1.0f;

    if ((candidate == NULL) || (candidate_count == 0U) ||
        (reference_stride == 0U)) return 0.0f;
    for (unsigned int phase = 0U; phase < 2U; ++phase) {
        const int16_t *source = phase == 0U ? even_reference : odd_reference;
        for (size_t i = 0U; i < candidate_count; ++i)
            reference_work[i] = source[i * reference_stride];
        if (timing_find_circular_lag(reference_work, candidate,
                candidate_count, &result) == 0 && result.correlation > best)
            best = result.correlation;
    }
    return best;
}

static void calibration_print_order_candidate(
    const char *name,
    const int16_t *candidate,
    size_t candidate_count,
    double sample_rate_hz,
    const int16_t *even_reference,
    const int16_t *odd_reference,
    size_t reference_stride,
    float *correlation_out,
    size_t *dominant_bin_out
)
{
    calibration_spectrum_t spectrum;
    const float correlation = calibration_best_reference_correlation(
        candidate, candidate_count, even_reference, odd_reference,
        reference_stride);
    const int spectral_status = calibration_calculate_full_spectrum(
        candidate, candidate_count, sample_rate_hz, &spectrum);

    xil_printf("\r\nCandidate name          : %s\r\n", name);
    xil_printf("Sample count            : %lu\r\n", (unsigned long)candidate_count);
    print_double_value("Effective sample rate", sample_rate_hz / 1.0e6, " MHz");
    xil_printf("Dominant bin            : %lu\r\n",
               (unsigned long)(spectral_status == 0 ? spectrum.dominant_bin : 0U));
    print_double_value("Dominant frequency", spectral_status == 0 ?
        spectrum.dominant_frequency_hz / 1.0e6 : 0.0, " MHz");
    print_float_value("DAC-reference correlation", correlation, "");
    if (correlation_out != NULL) *correlation_out = correlation;
    if (dominant_bin_out != NULL) *dominant_bin_out =
        spectral_status == 0 ? spectrum.dominant_bin : 0U;
}

static void calibration_decode_raw_mapping(
    const uint8_t *raw, size_t beats, unsigned int mapping,
    int16_t *channel_a, int16_t *channel_b)
{
    static const uint8_t indices[4][8] = {
        {0,1,2,3,4,5,6,7}, {4,5,6,7,0,1,2,3},
        {3,2,1,0,7,6,5,4}, {7,6,5,4,3,2,1,0}
    };
    for (size_t beat = 0U; beat < beats; ++beat) {
        uint16_t words[8];
        for (size_t word = 0U; word < 8U; ++word) {
            const size_t offset = (beat * 16U) + (word * 2U);
            words[word] = (uint16_t)raw[offset] |
                          ((uint16_t)raw[offset + 1U] << 8U);
        }
        for (size_t i = 0U; i < 4U; ++i) {
            channel_a[beat * 4U + i] =
                ((int16_t)words[indices[mapping][i]]) >> 2;
            channel_b[beat * 4U + i] =
                ((int16_t)words[indices[mapping][i + 4U]]) >> 2;
        }
    }
}

static void calibration_print_raw_beats(const uint8_t *raw, const char *title)
{
    xil_printf("\r\n%s\r\n", title);
    for (size_t beat = 0U; beat < 16U; ++beat) {
        xil_printf("Beat %lu hex   :", (unsigned long)beat);
        for (size_t word = 0U; word < 8U; ++word) {
            const size_t offset = beat * 16U + word * 2U;
            const uint16_t value = (uint16_t)raw[offset] |
                                   ((uint16_t)raw[offset + 1U] << 8U);
            xil_printf(" %04X", value);
        }
        xil_printf("\r\nBeat %lu signed:", (unsigned long)beat);
        for (size_t word = 0U; word < 8U; ++word) {
            const size_t offset = beat * 16U + word * 2U;
            const int16_t value = (int16_t)((uint16_t)raw[offset] |
                ((uint16_t)raw[offset + 1U] << 8U));
            xil_printf(" %d", (int)value);
        }
        xil_printf("\r\n");
    }
}

static float calibration_ramp_order_score(const int16_t *samples, size_t count)
{
    size_t matches = 0U;
    if (count < 2U) return 0.0f;
    for (size_t i = 1U; i < count; ++i) {
        const int16_t previous = samples[i - 1U];
        const int16_t current = samples[i];
        if ((current == previous + 1) ||
            ((previous >= 8190) && (current <= -8191))) ++matches;
    }
    return (float)matches / (float)(count - 1U);
}

static void calibration_print_raw_channel_metrics(
    const char *name, const int16_t *samples, size_t count,
    const int16_t *even_reference, const int16_t *odd_reference)
{
    calibration_spectrum_t spectrum;
    calibration_spectrum_t reference_spectrum;
    double mean = 0.0, rms = 0.0;
    int16_t minimum, maximum;
    minimum = maximum = samples[0];
    for (size_t i = 0U; i < count; ++i) {
        mean += samples[i]; rms += (double)samples[i] * samples[i];
        if (samples[i] < minimum) minimum = samples[i];
        if (samples[i] > maximum) maximum = samples[i];
    }
    mean /= count; rms = sqrt(rms / count);
    (void)calibration_calculate_full_spectrum(
        samples, count, adc_get_effective_sample_rate_hz(), &spectrum);
    (void)calibration_calculate_full_spectrum(even_reference, count,
        adc_get_effective_sample_rate_hz(), &reference_spectrum);
    xil_printf("\r\n%s\r\n", name);
    xil_printf("Sample count            : %lu\r\n", (unsigned long)count);
    print_double_value("Mean", mean, " codes");
    print_double_value("RMS", rms, " codes");
    xil_printf("Minimum                 : %d\r\n", (int)minimum);
    xil_printf("Maximum                 : %d\r\n", (int)maximum);
    xil_printf("Dominant spectral bin   : %lu\r\n",
               (unsigned long)spectrum.dominant_bin);
    print_double_value("Dominant frequency",
        spectrum.dominant_frequency_hz / 1.0e6, " MHz");
    print_double_value("Reference tone frequency",
        reference_spectrum.dominant_frequency_hz / 1.0e6, " MHz");
    print_double_value("Magnitude at reference bin",
        calibration_goertzel_magnitude(samples, count,
            reference_spectrum.dominant_bin, mean), "");
    print_double_value("Magnitude at dominant bin",
        calibration_goertzel_magnitude(samples, count,
            spectrum.dominant_bin, mean), "");
    print_float_value("Uploaded-DAC correlation",
        calibration_best_reference_correlation(samples, count,
            even_reference, odd_reference, 1U), "");
    xil_printf("First 32 samples        :\r\n");
    for (size_t i = 0U; i < 32U && i < count; ++i)
        xil_printf("%d%s", (int)samples[i],
                   ((i + 1U) % 8U) == 0U ? "\r\n" : " ");
}

static calibration_timing_frame_result_t calibration_print_channel_alignment(
    const char *label, const int16_t *reference, const int16_t *measurement,
    size_t count)
{
    static int16_t work_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t work_measurement[ADC_VALID_SAMPLE_COUNT];
    calibration_timing_frame_result_t result;
    memset(&result, 0, sizeof(result));
    (void)adc_measure_timing_frame(reference, measurement, count,
        work_reference, work_measurement, &result);
    xil_printf("\r\n%s\r\n", label);
    print_float_value("Correlation", result.correlation, "");
    xil_printf("Lag                     : %ld samples\r\n",
               (long)result.integer_lag);
    print_float_value("Fractional lag", result.fractional_lag, " samples");
    print_float_value("RMSE", result.fitted_rmse, " codes");
    return result;
}

static int calibration_prepare_uploaded_dac_reference(
    int16_t *even_reference,
    int16_t *odd_reference,
    size_t *reconstructed_count,
    double *even_variance,
    double *odd_variance,
    int print_errors
)
{
    const int16_t *raw_reference;
    size_t raw_count;
    int status;

    if (!reference_buffer_is_ready()) {
        if (print_errors) xil_printf("ERROR: No valid DAC reference uploaded.\r\n");
        return -1;
    }
    if (!isfinite(adc_get_effective_sample_rate_hz()) ||
        adc_get_effective_sample_rate_hz() <= 0.0) {
        if (print_errors) xil_printf("ERROR: Invalid effective ADC sample-rate value.\r\n");
        return -7;
    }
    raw_reference = reference_buffer_data();
    raw_count = reference_buffer_length();
    if (raw_reference == NULL) {
        if (print_errors) xil_printf("ERROR: Uploaded DAC reference pointer is NULL.\r\n");
        return -2;
    }
    if (reference_buffer_format() != REFERENCE_FORMAT_DAC_RATE_2X) {
        if (print_errors) xil_printf("ERROR: Uploaded reference is not tagged as raw DAC-rate data.\r\n");
        return -3;
    }
    if (raw_count < (2U * ADC_CHANNEL_SAMPLE_COUNT)) {
        if (print_errors) {
            xil_printf("ERROR: Uploaded DAC reference is too short.\r\n");
            xil_printf("Raw DAC samples       : %lu\r\n", (unsigned long)raw_count);
            xil_printf("Required DAC samples  : %lu\r\n",
                       (unsigned long)(2U * ADC_CHANNEL_SAMPLE_COUNT));
        }
        return -4;
    }

    /* Analyze exactly one ADC DMA frame; extra uploaded samples stay intact. */
    status = calibration_build_adc_reference_from_raw_dac(
        raw_reference, raw_count,
        even_reference, odd_reference, ADC_CHANNEL_SAMPLE_COUNT,
        reconstructed_count);
    if ((status != 0) || (*reconstructed_count != ADC_CHANNEL_SAMPLE_COUNT)) {
        if (print_errors) xil_printf("ERROR: DAC reference reconstruction bounds check failed.\r\n");
        return -5;
    }
    *even_variance = calibration_reference_variance(
        even_reference, *reconstructed_count);
    *odd_variance = calibration_reference_variance(
        odd_reference, *reconstructed_count);
    if (!isfinite(*even_variance) || !isfinite(*odd_variance) ||
        (*even_variance <= CAL_REF_VARIANCE_EPSILON) ||
        (*odd_variance <= CAL_REF_VARIANCE_EPSILON)) {
        if (print_errors) xil_printf("ERROR: Reconstructed DAC reference has near-zero variance.\r\n");
        return -6;
    }
    return 0;
}

static const calibration_timing_frame_result_t *calibration_select_phase(
    const calibration_timing_frame_result_t *even_result,
    const calibration_timing_frame_result_t *odd_result
)
{
    if (!even_result->alignment_success) return odd_result;
    if (!odd_result->alignment_success) return even_result;
    if (odd_result->correlation > even_result->correlation) return odd_result;
    if ((fabsf(odd_result->correlation - even_result->correlation) <= 1.0e-6f) &&
        (odd_result->fitted_rmse < even_result->fitted_rmse)) return odd_result;
    return even_result;
}

typedef struct {
    const int16_t *selected_reference;
    const int16_t *selected_adc;
    const char *selected_channel_name;
    const char *selected_phase_name;
    size_t sample_count;
    calibration_timing_frame_result_t timing;
    calibration_state_t fit_state;
    adc_cal_overlap_t fit_overlap;
    double reference_frequency_hz;
    double adc_frequency_hz;
    const char *failure_reason;
    int status;
} adc_reference_analysis_t;

/* Analyze one already-captured two-channel frame against both DAC phases. */
static int calibration_analyze_reference_frame(
    const int16_t *even_reference,
    const int16_t *odd_reference,
    const int16_t *channel_a,
    const int16_t *channel_b,
    size_t sample_count,
    int16_t *fractional_reference,
    int16_t *fractional_measurement,
    int locked_channel,
    adc_reference_analysis_t *analysis)
{
    calibration_timing_frame_result_t results[2][2];
    const int16_t *channels[2] = { channel_a, channel_b };
    const int16_t *references[2] = { even_reference, odd_reference };
    const char *channel_names[2] = { "Channel A", "Channel B" };
    const char *phase_names[2] = { "EVEN", "ODD" };
    calibration_spectrum_t reference_spectrum;
    calibration_spectrum_t adc_spectrum;
    size_t best_channel = 0U;
    size_t best_phase = 0U;
    int have_alignment = 0;

    if (analysis == NULL) return -1;
    memset(analysis, 0, sizeof(*analysis));
    analysis->failure_reason = "invalid analysis input";
    if (sample_count == 0U) return -1;
    if (!isfinite(adc_get_effective_sample_rate_hz()) ||
        adc_get_effective_sample_rate_hz() <= 0.0) {
        analysis->failure_reason = "invalid sample-rate value";
        return -1;
    }

    for (size_t channel = 0U; channel < 2U; ++channel) {
        if (locked_channel >= 0 && channel != (size_t)locked_channel)
            continue;
        for (size_t phase = 0U; phase < 2U; ++phase) {
            (void)adc_measure_timing_frame(references[phase], channels[channel],
                sample_count, fractional_reference, fractional_measurement,
                &results[channel][phase]);
            if (results[channel][phase].alignment_success &&
                (!have_alignment ||
                 results[channel][phase].correlation >
                     results[best_channel][best_phase].correlation ||
                 (fabsf(results[channel][phase].correlation -
                        results[best_channel][best_phase].correlation) <= 1.0e-6f &&
                  results[channel][phase].fitted_rmse <
                      results[best_channel][best_phase].fitted_rmse))) {
                best_channel = channel;
                best_phase = phase;
                have_alignment = 1;
            }
        }
    }
    if (!have_alignment) {
        analysis->failure_reason = "no valid reference phase";
        return -2;
    }

    analysis->selected_reference = references[best_phase];
    analysis->selected_adc = channels[best_channel];
    analysis->selected_channel_name = channel_names[best_channel];
    analysis->selected_phase_name = phase_names[best_phase];
    analysis->sample_count = sample_count;
    analysis->timing = results[best_channel][best_phase];

    if (adc_analyze_fractional_overlap(analysis->selected_reference,
            analysis->selected_adc, sample_count,
            (double)analysis->timing.total_lag, fractional_reference,
            fractional_measurement, &analysis->fit_state,
            &analysis->fit_overlap) != 0)
    {
        analysis->failure_reason = "insufficient overlap";
        return -3;
    }
    if (calibration_calculate_full_spectrum(analysis->selected_reference,
            sample_count, adc_get_effective_sample_rate_hz(),
            &reference_spectrum) != 0 ||
        calibration_calculate_full_spectrum(analysis->selected_adc,
            sample_count, adc_get_effective_sample_rate_hz(),
            &adc_spectrum) != 0)
    {
        analysis->failure_reason = "reference frequency measurement failed";
        return -4;
    }

    analysis->reference_frequency_hz = reference_spectrum.dominant_frequency_hz;
    analysis->adc_frequency_hz = adc_spectrum.dominant_frequency_hz;
    if (!isfinite(analysis->reference_frequency_hz) ||
        !isfinite(analysis->adc_frequency_hz)) {
        analysis->failure_reason = "invalid numerical result";
        return -5;
    }
    if (analysis->fit_overlap.analysis_count < CAL_MIN_ANALYSIS_SAMPLES ||
        analysis->timing.reject_reason == CAL_TIMING_REJECT_TOO_FEW_SAMPLES) {
        analysis->failure_reason = "insufficient overlap";
        return -5;
    }
    if (analysis->timing.correlation < CAL_DAC_REF_MIN_CORRELATION ||
        analysis->timing.reject_reason == CAL_TIMING_REJECT_LOW_CORRELATION) {
        analysis->failure_reason = "aligned correlation below threshold";
        return -5;
    }
    if (!analysis->timing.accepted) {
        analysis->failure_reason =
            cal_timing_reject_reason_text(analysis->timing.reject_reason);
        return -5;
    }
    if (fabs(analysis->reference_frequency_hz - analysis->adc_frequency_hz) >
        fmax(CAL_REF_FREQ_TOLERANCE_HZ,
             2.0 * adc_get_effective_sample_rate_hz() / (double)sample_count)) {
        analysis->failure_reason = "reference frequency mismatch";
        return -5;
    }

    analysis->failure_reason = "none";
    analysis->status = 0;
    return 0;
}

static void calibration_run_adc_reference_diagnostic(void)
{
    static int16_t even_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t odd_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t adc_samples[ADC_VALID_SAMPLE_COUNT];
    static int16_t candidate[ADC_VALID_SAMPLE_COUNT];
    calibration_spectrum_t even_spectrum;
    calibration_spectrum_t odd_spectrum;
    calibration_spectrum_t adc_spectrum;
    calibration_timing_frame_result_t even_alignment;
    calibration_timing_frame_result_t odd_alignment;
    const calibration_timing_frame_result_t *selected_alignment;
    size_t reference_count = 0U;
    size_t adc_count = 0U;
    double even_variance, odd_variance;
    int16_t minimum, maximum;
    double mean = 0.0, rms_sum = 0.0;
    float best_candidate_correlation = -1.0f;
    const char *best_candidate_name = "none";
    int capture_status;
    int diagnostic_pass = 0;

    print_adc_sample_rate_state();
    if (calibration_prepare_uploaded_dac_reference(
            even_reference, odd_reference, &reference_count,
            &even_variance, &odd_variance, 1) != 0) return;
    if (adc_sweep_active) {
        ERR("Another automatic ADC capture is already in progress.");
        return;
    }
    adc_sweep_active = 1U;
    xil_printf("\r\nCapturing one 4095-byte DMA frame for diagnosis.\r\n");
    capture_status = adc_capture_frame();
    if (capture_status != XST_SUCCESS) {
        xil_printf("ERROR: Diagnostic DMA capture failed.\r\n");
        goto diagnostic_done;
    }
    if (adc_reconstruct_frame(RxBufferPtr, DMA_CMD_BUF_SIZE, adc_samples,
            ADC_VALID_SAMPLE_COUNT, &adc_count) != 0 ||
        adc_count != ADC_CHANNEL_SAMPLE_COUNT) {
        xil_printf("ERROR: Diagnostic ADC reconstruction failed.\r\n");
        goto diagnostic_done;
    }

    minimum = maximum = adc_samples[0];
    for (size_t i = 0U; i < adc_count; ++i) {
        const double value = adc_samples[i];
        if (adc_samples[i] < minimum) minimum = adc_samples[i];
        if (adc_samples[i] > maximum) maximum = adc_samples[i];
        mean += value;
        rms_sum += value * value;
    }
    mean /= (double)adc_count;
    rms_sum = sqrt(rms_sum / (double)adc_count);

    xil_printf("\r\n========== DMA-Reconstructed ADC Frame ==========\r\n");
    xil_printf("ADC sample count        : %lu\r\n", (unsigned long)adc_count);
    xil_printf("ADC minimum             : %d\r\n", (int)minimum);
    xil_printf("ADC maximum             : %d\r\n", (int)maximum);
    print_double_value("ADC mean", mean, " codes");
    print_double_value("ADC RMS", rms_sum, " codes");
    xil_printf("First 32 reconstructed ADC samples:\r\n");
    for (size_t i = 0U; i < CAL_REF_ADC_DEBUG_SAMPLE_COUNT; ++i) {
        xil_printf("%d%s", (int)adc_samples[i],
                   ((i + 1U) % 8U) == 0U ? "\r\n" : " ");
    }

    xil_printf("\r\nReconstruction assumptions under test:\r\n");
    xil_printf("Raw words              : little-endian signed int16\r\n");
    xil_printf("ADC width/alignment    : signed left-aligned 14-bit, arithmetic >> 2\r\n");
    xil_printf("Eight-word ordering    : ADC0 S0..S3, ADC1 S0..S3\r\n");
    xil_printf("JESD configuration     : M=2, L=4, N=16, NP=16; no I/Q remap in this function\r\n");
    xil_printf("Lane/beat ordering     : assumed to be resolved by FPGA DMA producer\r\n");

    calibration_print_spectrum("DAC EVEN reference", even_reference,
        reference_count, adc_get_effective_sample_rate_hz(), &even_spectrum);
    calibration_print_spectrum("DAC ODD reference", odd_reference,
        reference_count, adc_get_effective_sample_rate_hz(), &odd_spectrum);
    calibration_print_spectrum("Real ADC capture", adc_samples,
        adc_count, adc_get_effective_sample_rate_hz(), &adc_spectrum);

    /* Inspect the untouched DMA words and both independent converters. */
    calibration_run_raw_mapping_diagnostic(even_reference, odd_reference);

    xil_printf("\r\n========== Sample-Order Experiments ==========\r\n");
#define RUN_CANDIDATE(candidate_name, data, count, rate, stride) do { \
        float candidate_correlation; size_t candidate_bin; \
        calibration_print_order_candidate(candidate_name, data, count, rate, \
            even_reference, odd_reference, stride, \
            &candidate_correlation, &candidate_bin); \
        if (candidate_correlation > best_candidate_correlation) { \
            best_candidate_correlation = candidate_correlation; \
            best_candidate_name = candidate_name; \
        } \
    } while (0)

    RUN_CANDIDATE("A. Current reconstructed sequence", adc_samples,
                  adc_count, adc_get_effective_sample_rate_hz(), 1U);
    for (size_t i = 0U; i < adc_count; ++i) {
        const uint16_t value = (uint16_t)adc_samples[i];
        candidate[i] = (int16_t)((value << 8U) | (value >> 8U));
    }
    RUN_CANDIDATE("B. Byte-swapped 16-bit samples", candidate,
                  adc_count, adc_get_effective_sample_rate_hz(), 1U);
    for (size_t i = 0U; i < adc_count; i += 2U) {
        candidate[i] = adc_samples[i + 1U];
        candidate[i + 1U] = adc_samples[i];
    }
    RUN_CANDIDATE("C. Adjacent sample pairs swapped", candidate,
                  adc_count, adc_get_effective_sample_rate_hz(), 1U);
    for (size_t phase = 0U; phase < 2U; ++phase) {
        const size_t count = adc_count / 2U;
        for (size_t i = 0U; i < count; ++i)
            candidate[i] = adc_samples[(2U * i) + phase];
        RUN_CANDIDATE(phase == 0U ? "D. Even-index samples only" :
                      "E. Odd-index samples only", candidate, count,
                      adc_get_effective_sample_rate_hz() / 2.0, 2U);
    }
    for (size_t phase = 0U; phase < 4U; ++phase) {
        const char *names[4] = {
            "F0. Every-fourth phase 0", "F1. Every-fourth phase 1",
            "F2. Every-fourth phase 2", "F3. Every-fourth phase 3"};
        const size_t count = adc_count / 4U;
        for (size_t i = 0U; i < count; ++i)
            candidate[i] = adc_samples[(4U * i) + phase];
        RUN_CANDIDATE(names[phase], candidate, count,
                      adc_get_effective_sample_rate_hz() / 4.0, 4U);
    }
#undef RUN_CANDIDATE

    (void)adc_measure_timing_frame(even_reference, adc_samples, adc_count,
        candidate, candidate + adc_count, &even_alignment);
    (void)adc_measure_timing_frame(odd_reference, adc_samples, adc_count,
        candidate, candidate + adc_count, &odd_alignment);
    selected_alignment = calibration_select_phase(&even_alignment,
        &odd_alignment);
    diagnostic_pass = selected_alignment->accepted &&
        selected_alignment->correlation >= CAL_DAC_REF_MIN_CORRELATION &&
        selected_alignment->analysis_samples >= CAL_MIN_ANALYSIS_SAMPLES &&
        fabs(adc_spectrum.dominant_frequency_hz -
             (selected_alignment == &odd_alignment ?
              odd_spectrum.dominant_frequency_hz :
              even_spectrum.dominant_frequency_hz)) <=
            fmax(CAL_REF_FREQ_TOLERANCE_HZ,
                 2.0 * adc_get_effective_sample_rate_hz() / (double)adc_count);

    xil_printf("\r\n========== ADC Reference Diagnostic ==========\r\n");
    xil_printf("DAC EVEN dominant bin       : %lu\r\n",
               (unsigned long)even_spectrum.dominant_bin);
    xil_printf("DAC ODD dominant bin        : %lu\r\n",
               (unsigned long)odd_spectrum.dominant_bin);
    xil_printf("ADC dominant bin            : %lu\r\n",
               (unsigned long)adc_spectrum.dominant_bin);
    print_float_value("Aligned reference correlation",
                      selected_alignment->correlation, "");
    xil_printf("Aligned overlap samples      : %lu\r\n",
               (unsigned long)selected_alignment->analysis_samples);
    xil_printf("Best sample-order candidate : %s\r\n", best_candidate_name);
    print_float_value("Best candidate correlation",
                      best_candidate_correlation, "");
    xil_printf("Diagnostic status           : %s\r\n",
               diagnostic_pass ? "PASS" : "FAIL");
    xil_printf("==============================================\r\n");

diagnostic_done:
    xil_printf("No correction coefficients were updated.\r\n");
    adc_sweep_active = 0U;
}

static void calibration_run_raw_mapping_diagnostic(
    const int16_t *even_reference, const int16_t *odd_reference)
{
    static int16_t channel_a[ADC_VALID_SAMPLE_COUNT];
    static int16_t channel_b[ADC_VALID_SAMPLE_COUNT];
    static int16_t current_reconstruction[ADC_VALID_SAMPLE_COUNT];
    const size_t beat_count = ADC_VALID_SAMPLE_COUNT / 8U;
    const size_t channel_count = beat_count * 4U;
    uint8_t saved_mode_a = AD9695_TESTMODE_OFF;
    uint8_t saved_mode_b = AD9695_TESTMODE_OFF;
    float best_ramp_score = 0.0f;
    float best_dac_correlation = -1.0f;
    size_t best_dominant_bin = 0U;
    size_t selected_a_bin = 0U, selected_b_bin = 0U;
    size_t normal_bins[4][2] = {{0U}};
    calibration_spectrum_t reference_spectrum, legacy_spectrum;
    calibration_spectrum_t channel_a_spectrum, channel_b_spectrum;
    calibration_timing_frame_result_t ab_alignment;
    float legacy_dac_correlation = 0.0f;
    float channel_a_dac_correlation = 0.0f;
    float channel_b_dac_correlation = 0.0f;
    unsigned int best_mapping = 0U;
    char best_channel = 'A';
    int ramp_capture_ok = 0;
    size_t current_count = 0U;
    struct jesd_param_t jesd_readback;

    memset(&jesd_readback, 0, sizeof(jesd_readback));
    ad9695_jesd_get_cfg_param(&jesd_readback);
    xil_printf("\r\nJESD SPI readback: M=%u L=%u F=%u S=%u N=%u NP=%u CS=%u HD=%u\r\n",
        jesd_readback.jesd_M, jesd_readback.jesd_L,
        jesd_readback.jesd_F, jesd_readback.jesd_S,
        jesd_readback.jesd_N, jesd_readback.jesd_NP,
        jesd_readback.jesd_CS, jesd_readback.jesd_HD);

    calibration_print_raw_beats(RxBufferPtr,
        "========== First 16 Normal Raw DMA Beats ==========");
    calibration_decode_raw_mapping(RxBufferPtr, beat_count, 0U,
        channel_a, channel_b);
    (void)adc_reconstruct_frame(RxBufferPtr, DMA_CMD_BUF_SIZE,
        current_reconstruction, ADC_VALID_SAMPLE_COUNT, &current_count);
    xil_printf("\r\nFirst 32 reconstruction comparison:\r\n");
    xil_printf("Index   Compatibility A    Channel A direct Channel B direct\r\n");
    for (size_t i = 0U; i < 32U; ++i) {
        xil_printf("%-7lu %-18d %-16d %d\r\n", (unsigned long)i,
            i < current_count ? (int)current_reconstruction[i] : 0,
            (int)channel_a[i], (int)channel_b[i]);
    }
    for (unsigned int mapping = 0U; mapping < 1U; ++mapping) {
        char label_a[40], label_b[40];
        calibration_decode_raw_mapping(RxBufferPtr, beat_count, mapping,
            channel_a, channel_b);
        snprintf(label_a, sizeof(label_a), "Mapping %u Channel A", mapping + 1U);
        snprintf(label_b, sizeof(label_b), "Mapping %u Channel B", mapping + 1U);
        calibration_print_raw_channel_metrics(label_a, channel_a,
            channel_count, even_reference, odd_reference);
        calibration_print_raw_channel_metrics(label_b, channel_b,
            channel_count, even_reference, odd_reference);
        {
            calibration_spectrum_t spectrum;
            float correlation = calibration_best_reference_correlation(
                channel_a, channel_count, even_reference, odd_reference, 1U);
            (void)calibration_calculate_full_spectrum(channel_a, channel_count,
                adc_get_effective_sample_rate_hz(), &spectrum);
            normal_bins[mapping][0] = spectrum.dominant_bin;
            if (correlation > best_dac_correlation) {
                best_dac_correlation = correlation;
                best_dominant_bin = spectrum.dominant_bin;
                best_mapping = mapping; best_channel = 'A';
            }
            correlation = calibration_best_reference_correlation(
                channel_b, channel_count, even_reference, odd_reference, 1U);
            (void)calibration_calculate_full_spectrum(channel_b, channel_count,
                adc_get_effective_sample_rate_hz(), &spectrum);
            normal_bins[mapping][1] = spectrum.dominant_bin;
            if (correlation > best_dac_correlation) {
                best_dac_correlation = correlation;
                best_dominant_bin = spectrum.dominant_bin;
                best_mapping = mapping; best_channel = 'B';
            }
        }
    }

    /* Mapping 1 is proven chronological by the ramp: analyze only it. */
    calibration_decode_raw_mapping(RxBufferPtr, beat_count, 0U,
        channel_a, channel_b);
    (void)calibration_calculate_full_spectrum(current_reconstruction,
        current_count, adc_get_effective_sample_rate_hz(), &legacy_spectrum);
    (void)calibration_calculate_full_spectrum(channel_a, channel_count,
        adc_get_effective_sample_rate_hz(), &channel_a_spectrum);
    (void)calibration_calculate_full_spectrum(channel_b, channel_count,
        adc_get_effective_sample_rate_hz(), &channel_b_spectrum);
    (void)calibration_calculate_full_spectrum(even_reference, channel_count,
        adc_get_effective_sample_rate_hz(), &reference_spectrum);
    legacy_dac_correlation = calibration_best_reference_correlation(
        current_reconstruction, current_count, even_reference, odd_reference, 1U);
    channel_a_dac_correlation = calibration_best_reference_correlation(
        channel_a, channel_count, even_reference, odd_reference, 1U);
    channel_b_dac_correlation = calibration_best_reference_correlation(
        channel_b, channel_count, even_reference, odd_reference, 1U);
    {
        timing_alignment_result_t even_result, odd_result;
        const int16_t *dac_a = even_reference;
        const int16_t *dac_b = even_reference;
        if (timing_find_circular_lag(even_reference, channel_a,
                channel_count, &even_result) == 0 &&
            timing_find_circular_lag(odd_reference, channel_a,
                channel_count, &odd_result) == 0 &&
            odd_result.correlation > even_result.correlation) dac_a = odd_reference;
        if (timing_find_circular_lag(even_reference, channel_b,
                channel_count, &even_result) == 0 &&
            timing_find_circular_lag(odd_reference, channel_b,
                channel_count, &odd_result) == 0 &&
            odd_result.correlation > even_result.correlation) dac_b = odd_reference;
        (void)calibration_print_channel_alignment(
            "DAC reference vs Channel A", dac_a, channel_a, channel_count);
        (void)calibration_print_channel_alignment(
            "DAC reference vs Channel B", dac_b, channel_b, channel_count);
    }
    ab_alignment = calibration_print_channel_alignment(
        "Channel A vs Channel B", channel_a, channel_b, channel_count);

    xil_printf("\r\n========== AD9695 Ramp Transport Test ==========\r\n");
    ad9695_adc_set_channel_select(0U);
    ad9695_read_register(&spi_inst, AD9695_REG_TEST_MODE, &saved_mode_a);
    ad9695_adc_set_channel_select(1U);
    ad9695_read_register(&spi_inst, AD9695_REG_TEST_MODE, &saved_mode_b);
    ad9695_adc_set_channel_select(0U);
    ad9695_write_register(&spi_inst, AD9695_REG_TEST_MODE, AD9695_TESTMODE_RAMP);
    ad9695_adc_set_channel_select(1U);
    ad9695_write_register(&spi_inst, AD9695_REG_TEST_MODE, AD9695_TESTMODE_RAMP);
    ad9695_adc_set_channel_select(2U);
    usleep(1000U);
    if (adc_capture_frame() == XST_SUCCESS) {
        ramp_capture_ok = 1;
        calibration_print_raw_beats(RxBufferPtr,
            "First 16 ramp-mode raw DMA beats:");
        for (unsigned int mapping = 0U; mapping < 1U; ++mapping) {
            float score_a, score_b;
            calibration_decode_raw_mapping(RxBufferPtr, beat_count, mapping,
                channel_a, channel_b);
            score_a = calibration_ramp_order_score(channel_a, channel_count);
            score_b = calibration_ramp_order_score(channel_b, channel_count);
            xil_printf("Mapping %u ramp score A/B: ", mapping + 1U);
            print_double_inline(score_a); xil_printf(" / ");
            print_double_inline(score_b); xil_printf("\r\n");
            if ((mapping == best_mapping) && (best_channel == 'A'))
                best_ramp_score = score_a;
            if ((mapping == best_mapping) && (best_channel == 'B'))
                best_ramp_score = score_b;
        }
    } else {
        xil_printf("ERROR: Ramp-mode DMA capture failed.\r\n");
    }

    /* Mandatory cleanup: restore both channel registers on every path. */
    ad9695_adc_set_channel_select(0U);
    ad9695_write_register(&spi_inst, AD9695_REG_TEST_MODE, saved_mode_a);
    ad9695_adc_set_channel_select(1U);
    ad9695_write_register(&spi_inst, AD9695_REG_TEST_MODE, saved_mode_b);
    ad9695_adc_set_channel_select(2U);

    selected_a_bin = normal_bins[best_mapping][0];
    selected_b_bin = normal_bins[best_mapping][1];

    xil_printf("\r\n========== Raw JESD/DMA Mapping Summary ==========\r\n");
    xil_printf("ADC channel mapping      : Mapping %u, Channel %c\r\n",
               best_mapping + 1U, best_channel);
    xil_printf("Words per beat           : 8\r\n");
    xil_printf("Samples/channel/beat     : 4 (M=2 assumption)\r\n");
    xil_printf("Configured JESD          : M=2 L=4 F=1 S=derived/readback N=16 NP=16\r\n");
    xil_printf("DMA bus width            : 128 bits (8 x 16-bit words)\r\n");
    xil_printf("Bit alignment            : unresolved; compare raw and ramp hex\r\n");
    xil_printf("Required right shift     : %s\r\n",
               best_ramp_score > 0.90f ? "2 indicated by ramp" : "UNCONFIRMED");
    xil_printf("Selected calibration channel: %c (diagnostic only)\r\n", best_channel);
    xil_printf("Selected dominant bin    : %lu\r\n",
               (unsigned long)best_dominant_bin);
    xil_printf("Channel A dominant bin   : %lu\r\n", (unsigned long)selected_a_bin);
    xil_printf("Channel B dominant bin   : %lu\r\n", (unsigned long)selected_b_bin);
    print_float_value("Selected DAC correlation", best_dac_correlation, "");
    print_float_value("Best ramp order score", best_ramp_score, "");
    {
        const double selected_frequency = (double)best_dominant_bin *
            adc_get_effective_sample_rate_hz() / (double)channel_count;
        const int hypothesis_pass = ramp_capture_ok &&
            best_ramp_score > 0.90f && best_dac_correlation > 0.98f &&
            fabs(selected_frequency - reference_spectrum.dominant_frequency_hz) <=
                CAL_REF_FREQ_TOLERANCE_HZ;
        print_double_value("Selected dominant freq",
                           selected_frequency / 1.0e6, " MHz");
        if (hypothesis_pass)
            xil_printf("Likely calibration channel: %c (diagnostic only)\r\n",
                       best_channel);
        else
            xil_printf("Two-channel hypothesis not confirmed.\r\n");
        xil_printf("Diagnostic status        : %s\r\n",
                   hypothesis_pass ? "PASS" : "FAIL");
    }
    xil_printf("No calibration mapping or coefficients were changed.\r\n");
    xil_printf("==================================================\r\n");

    {
        const double legacy_frequency = legacy_spectrum.dominant_frequency_hz;
        const double channel_a_frequency = channel_a_spectrum.dominant_frequency_hz;
        const double channel_b_frequency = channel_b_spectrum.dominant_frequency_hz;
        const int both_tones =
            fabs(channel_a_frequency - reference_spectrum.dominant_frequency_hz) <=
                CAL_REF_FREQ_TOLERANCE_HZ &&
            fabs(channel_b_frequency - reference_spectrum.dominant_frequency_hz) <=
                CAL_REF_FREQ_TOLERANCE_HZ;
        const char *recommended = "Neither";
        if (both_tones && ab_alignment.correlation > 0.98f) {
            recommended = channel_a_dac_correlation >=
                          channel_b_dac_correlation ? "A" : "B";
        }
        xil_printf("\r\n========================================\r\n");
        xil_printf("Compatibility Channel A reconstruction:\r\n");
        print_double_value("Dominant frequency", legacy_frequency / 1.0e6, " MHz");
        print_float_value("Correlation to DAC", legacy_dac_correlation, "");
        xil_printf("Channel A:\r\n");
        print_double_value("Dominant frequency", channel_a_frequency / 1.0e6, " MHz");
        print_float_value("Correlation to DAC", channel_a_dac_correlation, "");
        xil_printf("Channel B:\r\n");
        print_double_value("Dominant frequency", channel_b_frequency / 1.0e6, " MHz");
        print_float_value("Correlation to DAC", channel_b_dac_correlation, "");
        print_float_value("A-B correlation", ab_alignment.correlation, "");
        xil_printf("Recommended calibration channel: %s\r\n", recommended);
        if (both_tones && ab_alignment.correlation > 0.98f)
            xil_printf("Conclusion: independent ADC data paths are consistent.\r\n");
        xil_printf("========================================\r\n");
    }
}

void handle_adc_reference_status_cmd(void)
{
    static int16_t even_reference[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t odd_reference[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t channel_a[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t channel_b[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t fractional_reference[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t fractional_measurement[ADC_CHANNEL_SAMPLE_COUNT];
    adc_reference_analysis_t analysis;
    size_t reference_count = 0U;
    size_t adc_count = 0U;
    double even_variance = 0.0;
    double odd_variance = 0.0;
    int status = -1;

    if (!reference_buffer_is_ready() || reference_buffer_length() == 0U) {
        xil_printf("ADC reference analysis stopped: uploaded DAC reference is not available.\r\n");
        return;
    }
    if (adc_sweep_active) {
        ERR("Another automatic ADC capture is already in progress.");
        return;
    }
    print_adc_analysis_rate_header();
    if (calibration_prepare_uploaded_dac_reference(even_reference,
            odd_reference, &reference_count, &even_variance,
            &odd_variance, 1) != 0)
        return;

    adc_sweep_active = 1U;
    if (adc_capture_frame() != XST_SUCCESS) {
        xil_printf("ADC reference analysis stopped: DMA capture failed.\r\n");
        goto reference_done;
    }
    if (adc_reconstruct_channels(RxBufferPtr, DMA_CMD_BUF_SIZE,
            channel_a, ADC_CHANNEL_SAMPLE_COUNT, channel_b,
            ADC_CHANNEL_SAMPLE_COUNT, &adc_count) != 0 ||
        adc_count != reference_count) {
        xil_printf("ADC reference analysis stopped: sample reconstruction failed.\r\n");
        goto reference_done;
    }
    status = calibration_analyze_reference_frame(even_reference,
        odd_reference, channel_a, channel_b, adc_count,
        fractional_reference, fractional_measurement, -1, &analysis);

    xil_printf("\r\n========== ADC Reference Analysis ==========\r\n");
    xil_printf("ADC samples             : %lu\r\n", (unsigned long)adc_count);
    print_double_value("Configured sample rate",
        adc_get_configured_sample_rate_hz() / 1.0e6, " MSPS");
    print_double_value("Analysis sample rate",
        adc_get_effective_sample_rate_hz() / 1.0e6, " MSPS");
    if (status == 0) {
        print_double_value("Reference dominant freq",
            analysis.reference_frequency_hz / 1.0e6, " MHz");
        print_double_value("ADC dominant freq",
            analysis.adc_frequency_hz / 1.0e6, " MHz");
        xil_printf("Selected ADC channel    : %s\r\n",
            analysis.selected_channel_name);
        xil_printf("Selected reference phase: %s\r\n",
            analysis.selected_phase_name);
        xil_printf("Integer lag             : %ld samples\r\n",
            (long)analysis.timing.integer_lag);
        print_float_value("Fractional lag", analysis.timing.fractional_lag,
            " samples");
        print_float_value("Alignment correlation",
            analysis.timing.correlation, "");
        print_float_value("Aligned RMSE",
            analysis.fit_state.metrics.fitted_rmse_codes, " codes");
        print_reference_coherence(analysis.reference_frequency_hz, adc_count);
    } else {
        xil_printf("Failure reason          : %s\r\n",
            analysis.failure_reason != NULL ? analysis.failure_reason :
            "unknown analysis error");
    }
    xil_printf("Reference status        : %s\r\n",
        status == 0 ? "PASS" : "FAIL");
    xil_printf("============================================\r\n");
    xil_printf("No correction coefficients were updated.\r\n");

reference_done:
    adc_sweep_active = 0U;
}

static const char *calibration_channel_name(int channel)
{
    switch (channel) {
    case 0:
        return "Channel A";
    case 1:
        return "Channel B";
    case -1:
        return "auto";
    default:
        return "invalid";
    }
}

static void calibration_offset_loop_begin_run(
    calibration_offset_loop_state_t *state
)
{
    const int8_t calibration_channel = state->calibration_channel;

    memset(state, 0, sizeof(*state));
    state->offset_correction = calibration_software_offset_correction();
    state->gain_correction = calibration_software_gain_correction();
    state->calibration_channel = calibration_channel;
    state->final_status = CALIBRATION_OFFSET_LOOP_RUNNING;
}

static float calibration_mean_i16(const int16_t *samples, size_t sample_count)
{
    double sum = 0.0;

    if ((samples == NULL) || (sample_count == 0U)) {
        return 0.0f;
    }

    for (size_t i = 0U; i < sample_count; ++i) {
        sum += (double)samples[i];
    }

    return (float)(sum / (double)sample_count);
}

static int calibration_samples_are_clipped(
    const int16_t *samples,
    size_t sample_count
)
{
    if ((samples == NULL) || (sample_count == 0U)) {
        return 1;
    }

    for (size_t i = 0U; i < sample_count; ++i) {
        if ((samples[i] <= CALIBRATION_ADC_MIN_CODE) ||
            (samples[i] >= CALIBRATION_ADC_MAX_CODE)) {
            return 1;
        }
    }

    return 0;
}

static int calibration_fit_metrics_are_valid(
    const calibration_state_t *fit_state
)
{
    const calibration_metrics_t *metrics;

    if (fit_state == NULL) {
        return 0;
    }

    metrics = &fit_state->metrics;
    return isfinite(metrics->adc_mean) &&
           isfinite(metrics->reference_mean) &&
           isfinite(metrics->measured_gain) &&
           isfinite(metrics->measured_offset) &&
           isfinite(metrics->fitted_rmse_codes) &&
           isfinite(metrics->correlation) &&
           (fabsf(metrics->measured_gain) > FLT_EPSILON) &&
           (metrics->fitted_rmse_codes >= 0.0f);
}

static void calibration_offset_loop_print_summary(
    const calibration_offset_loop_state_t *state
)
{
    xil_printf("\r\n========== Offset Calibration Summary ==========\r\n");
    xil_printf("Accepted frames          : %lu\r\n",
               (unsigned long)state->accepted_frame_count);
    xil_printf("Rejected frames          : %lu\r\n",
               (unsigned long)state->rejected_frame_count);
    print_float_value("Final offset correction",
                      state->offset_correction, " codes");
    print_float_value("Final fitted offset",
                      state->latest_fitted_offset, " codes");
    print_float_value("Final correlation",
                      state->latest_correlation, "");
    print_float_value("Final fitted RMSE",
                      state->latest_rmse, " codes");
    xil_printf("Consecutive passes       : %lu\r\n",
               (unsigned long)state->convergence_count);
    xil_printf("Calibration status       : %s\r\n",
               calibration_offset_loop_status_name(state->final_status));
    xil_printf("===============================================\r\n");
}

static void calibration_offset_loop_reject_frame(
    calibration_offset_loop_state_t *state,
    const char *reason
)
{
    ++state->rejected_frame_count;
    state->convergence_count = 0U;
    xil_printf("Status                  : REJECTED\r\n");
    xil_printf("Rejection reason        : %s\r\n",
               reason != NULL ? reason : "unknown");
}

static int calibration_offset_loop_analyze_fit(
    calibration_offset_loop_state_t *loop_state,
    const adc_reference_analysis_t *analysis,
    const int16_t *fractional_reference,
    const int16_t *fractional_measurement,
    calibration_state_t *fit_state,
    float *raw_mean
)
{
    calibration_config_t config;
    calibration_status_t status;
    const size_t reference_start = analysis->fit_overlap.reference_start;
    const size_t measurement_start = analysis->fit_overlap.measurement_start;
    const size_t analysis_count = analysis->fit_overlap.analysis_count;

    if ((loop_state == NULL) || (analysis == NULL) ||
        (fractional_reference == NULL) || (fractional_measurement == NULL) ||
        (fit_state == NULL) || (raw_mean == NULL) ||
        (analysis_count < CAL_MIN_ANALYSIS_SAMPLES)) {
        return -1;
    }

    calibration_default_config(&config);
    config.offset_tolerance_codes = CALIBRATION_OFFSET_TOLERANCE_CODES;
    config.offset_step = CALIBRATION_OFFSET_UPDATE_STEP;
    config.min_offset_correction =
        -CALIBRATION_OFFSET_MAX_ABS_CORRECTION_CODES;
    config.max_offset_correction =
        CALIBRATION_OFFSET_MAX_ABS_CORRECTION_CODES;

    status = calibration_init(fit_state, &config);
    if (status != CALIBRATION_OK) {
        return -2;
    }

    fit_state->offset_correction = loop_state->offset_correction;
    fit_state->gain_correction = loop_state->gain_correction;

    *raw_mean = calibration_mean_i16(
        &fractional_measurement[measurement_start],
        analysis_count
    );

    status = calibration_analyze_frame(
        fit_state,
        &fractional_measurement[measurement_start],
        &fractional_reference[reference_start],
        analysis_count
    );

    if (status != CALIBRATION_OK) {
        return -3;
    }

    if (!calibration_fit_metrics_are_valid(fit_state)) {
        return -4;
    }

    return 0;
}

static void handle_adc_offset_calibration_status_cmd(void)
{
    const calibration_offset_loop_state_t *state =
        calibration_offset_loop_state();

    xil_printf("\r\n========== ADC Offset Calibration Status ==========\r\n");
    print_float_value("Offset correction",
                      state->offset_correction, " codes");
    print_float_value("Gain correction",
                      calibration_software_gain_correction(), "");
    xil_printf("Accepted frames        : %lu\r\n",
               (unsigned long)state->accepted_frame_count);
    xil_printf("Rejected frames        : %lu\r\n",
               (unsigned long)state->rejected_frame_count);
    xil_printf("Consecutive passes     : %lu\r\n",
               (unsigned long)state->convergence_count);
    xil_printf("Calibration channel    : %s\r\n",
               calibration_channel_name(state->calibration_channel));
    print_float_value("Latest correlation",
                      state->latest_correlation, "");
    print_float_value("Latest fitted gain",
                      state->latest_fitted_gain, "");
    print_float_value("Latest fitted offset",
                      state->latest_fitted_offset, " codes");
    print_float_value("Latest fitted RMSE",
                      state->latest_rmse, " codes");
    xil_printf("Calibration status     : %s\r\n",
               calibration_offset_loop_status_name(state->final_status));
    xil_printf("==================================================\r\n");
}

static void handle_adc_offset_calibration_loop_cmd(void)
{
    static int16_t even_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t odd_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t channel_a[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t channel_b[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t fractional_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t fractional_measurement[ADC_VALID_SAMPLE_COUNT];

    calibration_offset_loop_state_t *state =
        calibration_offset_loop_state();
    size_t reconstructed_count = 0U;
    double even_variance;
    double odd_variance;
    int reconstruction_status;

    if (adc_sweep_active) {
        ERR("Another automatic ADC capture is already in progress.");
        state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
        return;
    }

    calibration_offset_loop_begin_run(state);

    if ((state->calibration_channel < -1) ||
        (state->calibration_channel > 1)) {
        ERR("Invalid calibration channel in offset calibration state.");
        state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
        calibration_offset_loop_print_summary(state);
        return;
    }

    if (RxBufferPtr == NULL) {
        ERR("DMA receive buffer is not available.");
        state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
        calibration_offset_loop_print_summary(state);
        return;
    }

    print_adc_analysis_rate_header();
    if (calibration_prepare_uploaded_dac_reference(
            even_reference, odd_reference, &reconstructed_count,
            &even_variance, &odd_variance, 1) != 0) {
        state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
        calibration_offset_loop_print_summary(state);
        return;
    }

    adc_sweep_active = 1U;

    xil_printf("\r\n========== ADC Offset Calibration ==========\r\n");
    xil_printf("Calibration channel     : %s\r\n",
               calibration_channel_name(state->calibration_channel));
    print_float_value("Gain correction",
                      state->gain_correction, "");
    print_float_value("Initial offset correction",
                      state->offset_correction, " codes");
    print_float_value("Offset tolerance",
                      CALIBRATION_OFFSET_TOLERANCE_CODES, " codes");
    xil_printf("Maximum accepted frames : %u\r\n",
               CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS);
    xil_printf("Maximum rejected frames : %u\r\n",
               CALIBRATION_OFFSET_MAX_REJECTED_FRAMES);
    print_float_value("Offset update step",
                      CALIBRATION_OFFSET_UPDATE_STEP, "");

    while ((state->accepted_frame_count <
            CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS) &&
           (state->rejected_frame_count <
            CALIBRATION_OFFSET_MAX_REJECTED_FRAMES) &&
           (state->convergence_count <
            CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES)) {
        adc_reference_analysis_t analysis;
        calibration_state_t fit_state;
        calibration_timing_frame_result_t *selected = &analysis.timing;
        float raw_mean = 0.0f;
        float coefficient_delta = 0.0f;
        float next_offset_correction;
        int fit_status;

        xil_printf("\r\nIteration %lu\r\n",
                   (unsigned long)(state->accepted_frame_count +
                                   state->rejected_frame_count + 1U));

        if (adc_capture_frame() != XST_SUCCESS) {
            calibration_offset_loop_reject_frame(
                state, "DMA capture failed");
            continue;
        }

        reconstruction_status = adc_reconstruct_channels(
            RxBufferPtr, DMA_CMD_BUF_SIZE, channel_a,
            ADC_CHANNEL_SAMPLE_COUNT, channel_b,
            ADC_CHANNEL_SAMPLE_COUNT, &reconstructed_count
        );
        if ((reconstruction_status != 0) ||
            (reconstructed_count != ADC_CHANNEL_SAMPLE_COUNT)) {
            calibration_offset_loop_reject_frame(
                state, "sample reconstruction failed");
            continue;
        }

        fit_status = calibration_analyze_reference_frame(even_reference,
            odd_reference, channel_a, channel_b, reconstructed_count,
            fractional_reference, fractional_measurement,
            state->calibration_channel, &analysis);

        xil_printf("Channel                 : %s\r\n",
                   analysis.selected_channel_name != NULL ?
                   analysis.selected_channel_name : "none");
        xil_printf("Reference phase         : %s\r\n",
                   analysis.selected_phase_name != NULL ?
                   analysis.selected_phase_name : "none");

        if (fit_status != 0) {
            calibration_offset_loop_reject_frame(
                state,
                analysis.failure_reason != NULL ?
                analysis.failure_reason : "invalid alignment");
            continue;
        }

        print_float_value("Correlation",
                          selected->correlation, "");
        if (!selected->accepted ||
            (selected->correlation < CAL_DAC_REF_MIN_CORRELATION)) {
            calibration_offset_loop_reject_frame(
                state,
                selected->correlation < CAL_DAC_REF_MIN_CORRELATION ?
                "DAC-reference correlation below 0.970000" :
                cal_timing_reject_reason_text(selected->reject_reason));
            continue;
        }

        if (calibration_samples_are_clipped(
                analysis.selected_adc, analysis.sample_count)) {
            calibration_offset_loop_reject_frame(
                state, "ADC clipping detected");
            continue;
        }

        fit_status = calibration_offset_loop_analyze_fit(
            state, &analysis, fractional_reference, fractional_measurement,
            &fit_state, &raw_mean
        );
        if (fit_status != 0) {
            calibration_offset_loop_reject_frame(
                state, "invalid corrected fit");
            continue;
        }

        state->latest_correlation = selected->correlation;
        state->latest_fitted_offset =
            fit_state.metrics.measured_offset;
        state->latest_fitted_gain =
            fit_state.metrics.measured_gain;
        state->latest_rmse =
            fit_state.metrics.fitted_rmse_codes;
        state->latest_raw_mean = raw_mean;
        state->latest_corrected_mean =
            fit_state.metrics.adc_mean;

        if (state->calibration_channel < 0) {
            state->calibration_channel =
                analysis.selected_adc == channel_b ? 1 : 0;
        }

        print_float_value("Raw mean",
                          state->latest_raw_mean, " codes");
        print_float_value("Corrected mean",
                          state->latest_corrected_mean, " codes");
        print_float_value("Fitted gain",
                          state->latest_fitted_gain, "");
        print_float_value("Fitted offset before",
                          state->latest_fitted_offset, " codes");

        if (fabsf(state->latest_fitted_offset) <=
            CALIBRATION_OFFSET_TOLERANCE_CODES) {
            ++state->convergence_count;
        } else {
            state->convergence_count = 0U;
            coefficient_delta =
                -CALIBRATION_OFFSET_UPDATE_STEP *
                state->latest_fitted_offset;
        }

        next_offset_correction =
            state->offset_correction + coefficient_delta;

        if (!isfinite(next_offset_correction) ||
            (fabsf(next_offset_correction) >
             CALIBRATION_OFFSET_MAX_ABS_CORRECTION_CODES)) {
            ++state->accepted_frame_count;
            print_float_value("Offset update",
                              coefficient_delta, " codes");
            print_float_value("Offset correction",
                              state->offset_correction, " codes");
            print_float_value("RMSE",
                              state->latest_rmse, " codes");
            xil_printf("Status                  : FAILED\r\n");
            xil_printf("Failure reason          : coefficient limit reached\r\n");
            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
            break;
        }

        state->offset_correction = next_offset_correction;
        (void)calibration_set_software_offset_correction(
            state->offset_correction);
        ++state->accepted_frame_count;

        print_float_value("Offset update",
                          coefficient_delta, " codes");
        print_float_value("Offset correction",
                          state->offset_correction, " codes");
        print_float_value("RMSE",
                          state->latest_rmse, " codes");
        xil_printf("Consecutive passes      : %lu\r\n",
                   (unsigned long)state->convergence_count);
        xil_printf("Status                  : ACCEPTED\r\n");

        if (state->accepted_frame_count <
            CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS) {
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
        }
    }

    if (state->final_status == CALIBRATION_OFFSET_LOOP_RUNNING) {
        if (state->convergence_count >=
            CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES) {
            state->final_status = CALIBRATION_OFFSET_LOOP_PASS;
        } else if (state->rejected_frame_count >=
                   CALIBRATION_OFFSET_MAX_REJECTED_FRAMES) {
            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
        } else {
            state->final_status = CALIBRATION_OFFSET_LOOP_NOT_CONVERGED;
        }
    }

    calibration_offset_loop_print_summary(state);
    adc_sweep_active = 0U;
}

static void calibration_gain_loop_begin_run(
    calibration_gain_loop_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->gain_correction = calibration_software_gain_correction();
    state->fixed_offset_correction =
        calibration_software_offset_correction();
    state->calibration_channel = -1;
    state->final_status = CALIBRATION_GAIN_LOOP_RUNNING;
}

static void calibration_gain_loop_reject_frame(
    calibration_gain_loop_state_t *state, const char *reason)
{
    ++state->rejected_frame_count;
    state->convergence_count = 0U;
    xil_printf("Status                  : REJECTED\r\n");
    xil_printf("Rejection reason        : %s\r\n",
               reason != NULL ? reason : "unknown");
}

static int calibration_apply_gain_loop_correction(
    const int16_t *raw, int16_t *corrected, size_t count,
    float gain_correction, float offset_correction)
{
    if (raw == NULL || corrected == NULL || count == 0U ||
        !isfinite(gain_correction) || !isfinite(offset_correction))
        return -1;
    for (size_t i = 0U; i < count; ++i) {
        const double value = (double)raw[i] * gain_correction +
                             offset_correction;
        if (!isfinite(value) || value < CALIBRATION_ADC_MIN_CODE ||
            value > CALIBRATION_ADC_MAX_CODE)
            return -2;
        corrected[i] = (int16_t)lround(value);
    }
    return 0;
}

static void calibration_gain_loop_print_summary(
    const calibration_gain_loop_state_t *state)
{
    xil_printf("\r\n========== Gain Calibration Summary ==========\r\n");
    xil_printf("Accepted frames          : %lu\r\n",
               (unsigned long)state->accepted_frame_count);
    xil_printf("Rejected frames          : %lu\r\n",
               (unsigned long)state->rejected_frame_count);
    xil_printf("Calibration channel      : %s\r\n",
               calibration_channel_name(state->calibration_channel));
    print_float_value("Fixed offset correction",
                      state->fixed_offset_correction, " codes");
    print_float_value("Final gain correction",
                      state->gain_correction, "");
    print_float_value("Final fitted gain",
                      state->latest_fitted_gain, "");
    print_float_value("Final gain error",
                      state->latest_gain_error, "");
    print_float_value("Final fitted offset",
                      state->latest_fitted_offset, " codes");
    print_float_value("Final correlation",
                      state->latest_correlation, "");
    print_float_value("Final fitted RMSE",
                      state->latest_rmse, " codes");
    xil_printf("Consecutive passes       : %lu\r\n",
               (unsigned long)state->convergence_count);
    xil_printf("Calibration status       : %s\r\n",
               calibration_gain_loop_status_name(state->final_status));
    xil_printf("=============================================\r\n");
}

static void handle_adc_gain_calibration_status_cmd(void)
{
    const calibration_gain_loop_state_t *state =
        calibration_gain_loop_state();
    xil_printf("\r\n========== ADC Gain Calibration Status ==========\r\n");
    print_float_value("Gain correction", state->gain_correction, "");
    print_float_value("Fixed offset correction",
                      state->fixed_offset_correction, " codes");
    xil_printf("Accepted frames        : %lu\r\n",
               (unsigned long)state->accepted_frame_count);
    xil_printf("Rejected frames        : %lu\r\n",
               (unsigned long)state->rejected_frame_count);
    xil_printf("Consecutive passes     : %lu\r\n",
               (unsigned long)state->convergence_count);
    xil_printf("Calibration channel    : %s\r\n",
               calibration_channel_name(state->calibration_channel));
    print_float_value("Latest fitted gain", state->latest_fitted_gain, "");
    print_float_value("Latest gain error", state->latest_gain_error, "");
    print_float_value("Latest fitted offset",
                      state->latest_fitted_offset, " codes");
    print_float_value("Latest correlation", state->latest_correlation, "");
    print_float_value("Latest fitted RMSE", state->latest_rmse, " codes");
    xil_printf("Calibration status     : %s\r\n",
               calibration_gain_loop_status_name(state->final_status));
    xil_printf("================================================\r\n");
}

static void handle_adc_gain_calibration_loop_cmd(void)
{
    static int16_t even_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t odd_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t even_gain_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t odd_gain_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t raw_channel_a[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t raw_channel_b[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t corrected_channel_a[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t corrected_channel_b[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t fractional_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t fractional_measurement[ADC_VALID_SAMPLE_COUNT];
    calibration_gain_loop_state_t *state = calibration_gain_loop_state();
    size_t reconstructed_count = 0U;
    double even_variance, odd_variance;

    if (adc_sweep_active || RxBufferPtr == NULL) {
        ERR("ADC capture is unavailable for gain calibration.");
        state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
        return;
    }
    calibration_gain_loop_begin_run(state);
    print_adc_analysis_rate_header();
    if (calibration_prepare_uploaded_dac_reference(even_reference,
            odd_reference, &reconstructed_count, &even_variance,
            &odd_variance, 1) != 0) {
        state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
        calibration_gain_loop_print_summary(state);
        return;
    }
    /* Put DAC and ADC samples in common signed full-scale code units. */
    for (size_t i = 0U; i < reconstructed_count; ++i) {
        even_gain_reference[i] = (int16_t)lround(
            (double)even_reference[i] * CAL_ADC_FULL_SCALE_CODES /
            CAL_DAC_FULL_SCALE_CODES);
        odd_gain_reference[i] = (int16_t)lround(
            (double)odd_reference[i] * CAL_ADC_FULL_SCALE_CODES /
            CAL_DAC_FULL_SCALE_CODES);
    }

    adc_sweep_active = 1U;
    xil_printf("\r\n========== ADC Gain Calibration ==========\r\n");
    xil_printf("Calibration channel     : auto (lock after first accepted frame)\r\n");
    print_float_value("Fixed offset correction",
                      state->fixed_offset_correction, " codes");
    print_float_value("Initial gain correction",
                      state->gain_correction, "");
    print_float_value("Gain tolerance", CALIBRATION_GAIN_TOLERANCE, "");
    print_float_value("Gain update step", CALIBRATION_GAIN_UPDATE_STEP, "");
    xil_printf("Maximum accepted frames : %u\r\n",
               CALIBRATION_GAIN_MAX_ACCEPTED_ITERATIONS);
    xil_printf("Maximum rejected frames : %u\r\n",
               CALIBRATION_GAIN_MAX_REJECTED_FRAMES);

    while (state->accepted_frame_count <
               CALIBRATION_GAIN_MAX_ACCEPTED_ITERATIONS &&
           state->rejected_frame_count <
               CALIBRATION_GAIN_MAX_REJECTED_FRAMES &&
           state->convergence_count <
               CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES) {
        adc_reference_analysis_t analysis;
        float update_factor = 1.0f;
        float next_gain;
        int status;

        xil_printf("\r\nIteration %lu\r\n",
            (unsigned long)(state->accepted_frame_count +
                            state->rejected_frame_count + 1U));
        if (adc_capture_frame() != XST_SUCCESS) {
            calibration_gain_loop_reject_frame(state, "DMA capture failure");
            continue;
        }
        status = adc_reconstruct_channels(RxBufferPtr, DMA_CMD_BUF_SIZE,
            raw_channel_a, ADC_CHANNEL_SAMPLE_COUNT, raw_channel_b,
            ADC_CHANNEL_SAMPLE_COUNT, &reconstructed_count);
        if (status != 0 || reconstructed_count != ADC_CHANNEL_SAMPLE_COUNT) {
            calibration_gain_loop_reject_frame(
                state, "sample reconstruction failed");
            continue;
        }
        if (calibration_samples_are_clipped(raw_channel_a,
                reconstructed_count) ||
            calibration_samples_are_clipped(raw_channel_b,
                reconstructed_count)) {
            calibration_gain_loop_reject_frame(state, "clipping");
            continue;
        }
        if (calibration_apply_gain_loop_correction(raw_channel_a,
                corrected_channel_a, reconstructed_count,
                state->gain_correction, state->fixed_offset_correction) != 0 ||
            calibration_apply_gain_loop_correction(raw_channel_b,
                corrected_channel_b, reconstructed_count,
                state->gain_correction, state->fixed_offset_correction) != 0) {
            calibration_gain_loop_reject_frame(
                state, "invalid or clipped corrected samples");
            continue;
        }

        status = calibration_analyze_reference_frame(even_gain_reference,
            odd_gain_reference, corrected_channel_a, corrected_channel_b,
            reconstructed_count, fractional_reference,
            fractional_measurement, state->calibration_channel, &analysis);
        if (status != 0) {
            calibration_gain_loop_reject_frame(state,
                analysis.failure_reason != NULL ? analysis.failure_reason :
                "invalid alignment or regression");
            continue;
        }
        if (!calibration_fit_metrics_are_valid(&analysis.fit_state)) {
            calibration_gain_loop_reject_frame(
                state, "nonfinite fitted result");
            continue;
        }

        state->latest_fitted_gain =
            analysis.fit_state.metrics.measured_gain;
        state->latest_gain_error = state->latest_fitted_gain - 1.0f;
        state->latest_fitted_offset =
            analysis.fit_state.metrics.measured_offset;
        state->latest_correlation = analysis.timing.correlation;
        state->latest_rmse =
            analysis.fit_state.metrics.fitted_rmse_codes;
        if (!isfinite(state->latest_gain_error) ||
            state->latest_fitted_gain < CALIBRATION_GAIN_FITTED_MIN) {
            calibration_gain_loop_reject_frame(
                state, "fitted gain outside valid range");
            state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
            break;
        }
        if (state->calibration_channel < 0)
            state->calibration_channel =
                analysis.selected_adc == corrected_channel_b ? 1 : 0;

        if (fabsf(state->latest_gain_error) <=
            CALIBRATION_GAIN_TOLERANCE) {
            ++state->convergence_count;
        } else {
            state->convergence_count = 0U;
            update_factor = 1.0f - CALIBRATION_GAIN_UPDATE_STEP *
                                      state->latest_gain_error;
        }
        next_gain = state->gain_correction * update_factor;

        print_float_value("Correlation", state->latest_correlation, "");
        print_float_value("Fitted gain before",
                          state->latest_fitted_gain, "");
        print_float_value("Gain error", state->latest_gain_error, "");
        print_float_value("Gain update factor", update_factor, "");
        print_float_value("Fitted offset",
                          state->latest_fitted_offset, " codes");
        print_float_value("RMSE", state->latest_rmse, " codes");

        if (!isfinite(update_factor) || !isfinite(next_gain) ||
            next_gain < CALIBRATION_GAIN_CORRECTION_MIN ||
            next_gain > CALIBRATION_GAIN_CORRECTION_MAX ||
            calibration_set_software_gain_correction(next_gain) != 0) {
            state->convergence_count = 0U;
            ++state->accepted_frame_count;
            state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
            xil_printf("Status                  : FAILED\r\n");
            xil_printf("Failure reason          : gain coefficient limit reached\r\n");
            break;
        }

        state->gain_correction = next_gain;
        ++state->accepted_frame_count;
        print_float_value("Gain correction", state->gain_correction, "");
        xil_printf("Consecutive passes      : %lu\r\n",
                   (unsigned long)state->convergence_count);
        xil_printf("Status                  : ACCEPTED\r\n");
        if (state->convergence_count <
            CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES)
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
    }

    if (state->final_status == CALIBRATION_GAIN_LOOP_RUNNING) {
        if (state->convergence_count >=
            CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES)
            state->final_status = CALIBRATION_GAIN_LOOP_PASS;
        else if (state->rejected_frame_count >=
                 CALIBRATION_GAIN_MAX_REJECTED_FRAMES)
            state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
        else
            state->final_status = CALIBRATION_GAIN_LOOP_NOT_CONVERGED;
    }
    calibration_gain_loop_print_summary(state);
    adc_sweep_active = 0U;
}

void handle_adc_calibration_cmd(uint32_t frame_count)
{
    static int16_t even_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t odd_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t channel_a[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t channel_b[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t fractional_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t fractional_measurement[ADC_VALID_SAMPLE_COUNT];
    static float correlations[ADC_CAL_MAX_FRAMES];
    static float gains[ADC_CAL_MAX_FRAMES];
    static float offsets[ADC_CAL_MAX_FRAMES];
    static float fitted_rmse[ADC_CAL_MAX_FRAMES];

    const int16_t *uploaded_reference;
    size_t uploaded_count;
    size_t reconstructed_count = 0U;
    uint32_t accepted_frames = 0U;
    int calibration_channel = -1;
    double sum_correlation = 0.0;
    double even_variance;
    double odd_variance;
    calibration_spectrum_t header_reference_spectrum;
    int reconstruction_status;

    if ((frame_count < ADC_CAL_MIN_FRAMES) ||
        (frame_count > ADC_CAL_MAX_FRAMES)) {
        ERR("Calibration frame count must be between %u and %u.",
            ADC_CAL_MIN_FRAMES, ADC_CAL_MAX_FRAMES);
        return;
    }

    print_adc_analysis_rate_header();
    uploaded_reference = reference_buffer_data();
    uploaded_count = reference_buffer_length();
    (void)uploaded_reference;
    (void)uploaded_count;
    if (calibration_prepare_uploaded_dac_reference(
            even_reference, odd_reference, &reconstructed_count,
            &even_variance, &odd_variance, 1) != 0) return;

    if (adc_sweep_active)
    {
        ERR("Another automatic ADC capture is already in progress.");
        return;
    }

    adc_sweep_active = 1;

    xil_printf("\r\n");
    xil_printf("ADC DAC-Referenced Calibration Measurement\r\n");
    xil_printf("Reference source: uploaded DAC TXT\r\n");
    xil_printf("Requested frames: %lu\r\n", (unsigned long)frame_count);
    xil_printf("ADC samples per channel: %u\r\n", ADC_CHANNEL_SAMPLE_COUNT);
    if (calibration_calculate_full_spectrum(even_reference,
            reconstructed_count, adc_get_effective_sample_rate_hz(),
            &header_reference_spectrum) == 0) {
        print_double_value("Reference frequency",
            header_reference_spectrum.dominant_frequency_hz / 1.0e6,
            " MHz");
        print_reference_coherence(
            header_reference_spectrum.dominant_frequency_hz,
            reconstructed_count);
    }

    for (uint32_t frame = 1U; frame <= frame_count; ++frame)
    {
        adc_reference_analysis_t analysis;
        calibration_timing_frame_result_t *selected = &analysis.timing;
        calibration_state_t *fit_state = &analysis.fit_state;
        int fit_status;

        xil_printf("\r\n---------- Calibration Frame %lu ----------\r\n",
                   (unsigned long)frame);

        if (adc_capture_frame() != XST_SUCCESS) {
            xil_printf("Frame status           : REJECTED\r\n");
            xil_printf("Rejection reason       : DMA capture failed\r\n");
            continue;
        }

        reconstruction_status = adc_reconstruct_channels(
            RxBufferPtr, DMA_CMD_BUF_SIZE, channel_a,
            ADC_CHANNEL_SAMPLE_COUNT, channel_b,
            ADC_CHANNEL_SAMPLE_COUNT, &reconstructed_count
        );
        if ((reconstruction_status != 0) ||
            (reconstructed_count != ADC_CHANNEL_SAMPLE_COUNT)) {
            xil_printf("Frame status           : REJECTED\r\n");
            xil_printf("Rejection reason       : sample reconstruction failed\r\n");
            continue;
        }

        fit_status = calibration_analyze_reference_frame(even_reference,
            odd_reference, channel_a, channel_b, reconstructed_count,
            fractional_reference, fractional_measurement,
            calibration_channel, &analysis);

        xil_printf("Channel                : %s\r\n",
                   analysis.selected_channel_name != NULL ?
                   analysis.selected_channel_name : "none");
        xil_printf("Reference phase        : %s\r\n",
                   analysis.selected_phase_name != NULL ?
                   analysis.selected_phase_name : "none");
        if (fit_status != 0) {
            xil_printf("Frame status           : REJECTED\r\n");
            xil_printf("Rejection reason       : %s\r\n",
                analysis.failure_reason != NULL ? analysis.failure_reason :
                "unknown analysis error");
            continue;
        }
        xil_printf("Integer lag            : %ld samples\r\n",
                   (long)selected->integer_lag);
        print_float_value("Fractional lag", selected->fractional_lag, " samples");
        xil_printf("Analysis samples       : %lu\r\n",
                   (unsigned long)selected->analysis_samples);
        print_float_value("Correlation", selected->correlation, "");

        if (!selected->accepted ||
            (selected->correlation < CAL_DAC_REF_MIN_CORRELATION)) {
            xil_printf("Frame status           : REJECTED\r\n");
            xil_printf("Rejection reason       : %s\r\n",
                       selected->correlation < CAL_DAC_REF_MIN_CORRELATION ?
                       "DAC-reference correlation below 0.970000" :
                       cal_timing_reject_reason_text(selected->reject_reason));
            continue;
        }

        {
            const float normalized_gain = fit_state->metrics.measured_gain *
                (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
            const float normalized_offset = fit_state->metrics.measured_offset /
                CAL_ADC_FULL_SCALE_CODES;
            print_float_value("Normalized gain", normalized_gain, "");
            print_float_value("Scale deviation from nominal",
                normalized_gain - 1.0f, "");
            print_float_value("Fitted DC offset",
                fit_state->metrics.measured_offset, " ADC codes");
            print_float_value("Normalized DC offset", normalized_offset,
                " full-scale");
        }
        print_float_value("Fitted MAE", fit_state->metrics.fitted_mae_codes,
                          " codes");
        print_float_value("Fitted RMSE", fit_state->metrics.fitted_rmse_codes,
                          " codes");
        xil_printf("Frame status           : ACCEPTED\r\n");
        correlations[accepted_frames] = selected->correlation;
        gains[accepted_frames] = fit_state->metrics.measured_gain *
            (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
        offsets[accepted_frames] = fit_state->metrics.measured_offset;
        fitted_rmse[accepted_frames] = fit_state->metrics.fitted_rmse_codes;
        sum_correlation += (double)selected->correlation;
        if (calibration_channel < 0)
            calibration_channel = analysis.selected_adc == channel_b ? 1 : 0;
        ++accepted_frames;

        if (frame < frame_count) usleep(ADC_TIMING_INTERFRAME_DELAY_US);
    }

    xil_printf("\r\n========== DAC-Referenced Calibration Summary ==========\r\n");
    xil_printf("Requested frames          : %lu\r\n", (unsigned long)frame_count);
    xil_printf("Accepted frames           : %lu\r\n", (unsigned long)accepted_frames);
    xil_printf("Rejected frames           : %lu\r\n",
               (unsigned long)(frame_count - accepted_frames));
    xil_printf("Calibration channel       : %s\r\n",
               calibration_channel == 0 ? "Channel A" :
               (calibration_channel == 1 ? "Channel B" : "none"));
    if (accepted_frames > 0U) {
        float min_corr = correlations[0];
        for (uint32_t i = 1U; i < accepted_frames; ++i)
            if (correlations[i] < min_corr) min_corr = correlations[i];
        print_double_value("Mean correlation",
                           sum_correlation / (double)accepted_frames, "");
        print_float_value("Minimum correlation", min_corr, "");
        print_float_value("Median normalized gain",
                          median_float(gains, accepted_frames), "");
        print_float_value("Median fitted offset",
                          median_float(offsets, accepted_frames), " codes");
        print_float_value("Median fitted RMSE",
                          median_float(fitted_rmse, accepted_frames), " codes");
    }
    {
        const float acceptance_rate =
            (float)accepted_frames / (float)frame_count;
        const uint32_t required =
            frame_count < CAL_TIMING_MIN_ACCEPTED_FRAMES ?
            frame_count : CAL_TIMING_MIN_ACCEPTED_FRAMES;
        xil_printf("Calibration status        : %s\r\n",
                   ((accepted_frames >= required) &&
                    (acceptance_rate >= CAL_TIMING_MIN_ACCEPTANCE_RATE)) ?
                   "PASS" : "FAIL");
    }
    xil_printf("========================================================\r\n");

    xil_printf("No correction coefficients were updated.\r\n");
    adc_sweep_active = 0;
}
