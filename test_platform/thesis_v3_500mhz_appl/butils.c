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
#include "calibration_pending.h"
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

extern uint8_t uart_send_flag; // Send flag enabled by the uart
volatile uint8_t adc_sweep_active = 0;

typedef struct {
    double configured_rate_hz;
    double analysis_rate_hz;
    double correction_factor;
    bool measured_rate_valid;
    uint32_t generation;
} adc_sample_rate_state_t;

static adc_sample_rate_state_t g_adc_sample_rate = {
    ADC_CONFIGURED_SAMPLE_RATE_HZ,
    ADC_CONFIGURED_SAMPLE_RATE_HZ,
    1.0,
    false,
    0U
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
    if (rate_hz != g_adc_sample_rate.analysis_rate_hz) {
        calibration_pending_frame_invalidate();
        ++g_adc_sample_rate.generation;
    }
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
#define CAL_UPDATE_FRAME_BATCH_SIZE         ADC_CAL_DEFAULT_FRAMES
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
#define CAL_REPRESENTATIVE_TIE_EPSILON            1.0e-6f

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

typedef struct {
    int locked_channel;
    float adc_gain_correction;
    float adc_offset_correction;
    float reference_scale;
    bool reject_clipped_input;
} calibration_frame_config_t;

typedef struct {
    int16_t channel_a[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t channel_b[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t fractional_reference[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t fractional_measurement[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t aligned_reference[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t aligned_raw_adc[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t aligned_corrected_adc[ADC_CHANNEL_SAMPLE_COUNT];
} calibration_frame_workspace_t;

typedef struct {
    uint32_t capture_sequence;
    uint32_t retained_frame_number;
    bool capture_succeeded;
    bool reconstruction_succeeded;
    bool frame_valid;
    int8_t selected_channel;
    int8_t selected_reference_phase;
    const char *selected_channel_name;
    const char *selected_phase_name;
    int32_t integer_lag;
    float fractional_lag;
    float total_lag;
    const int16_t *aligned_reference_samples;
    const int16_t *aligned_raw_adc_samples;
    const int16_t *aligned_corrected_adc_samples;
    size_t valid_analysis_sample_count;
    float raw_aligned_adc_mean;
    float correlation;
    float even_candidate_correlation;
    float odd_candidate_correlation;
    int32_t even_candidate_lag;
    int32_t odd_candidate_lag;
    uint8_t even_candidate_valid;
    uint8_t odd_candidate_valid;
    calibration_metrics_t metrics;
    calibration_timing_frame_result_t timing;
    adc_cal_overlap_t overlap;
    double reference_frequency_hz;
    double adc_frequency_hz;
    const char *rejection_reason;
} calibration_aligned_frame_t;

typedef struct {
    bool valid;
    bool consumed;
    uint32_t capture_sequence;
    uint32_t retained_frame_number;
    int8_t selected_channel;
    int8_t selected_reference_phase;
    int32_t integer_lag;
    float fractional_lag;
    float total_lag;
    size_t analysis_sample_count;
    float raw_aligned_adc_mean;
    float correlation;
    calibration_metrics_t metrics;
    calibration_timing_frame_result_t timing;
    adc_cal_overlap_t overlap;
    double reference_frequency_hz;
    double adc_frequency_hz;
    int16_t aligned_reference[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t aligned_raw_adc[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t aligned_corrected_adc[ADC_CHANNEL_SAMPLE_COUNT];
    uint32_t reference_generation;
    size_t reference_length;
    reference_buffer_format_t reference_format;
    uint32_t sample_rate_generation;
    double configured_sample_rate_hz;
    double effective_sample_rate_hz;
    double dac_adc_rate_ratio;
    int8_t channel_configuration;
    float software_gain_correction;
    float software_offset_correction;
    uint8_t source_offset_status;
} calibration_pending_frame_t;

typedef struct {
    float median_fitted_offset;
    float median_normalized_gain;
    float median_fitted_rmse;
} calibration_selection_medians_t;

typedef struct {
    float mean;
    float stddev;
    float minimum;
    float maximum;
    float rmse;
    float mean_correlation;
    float mean_fitted_rmse;
    uint32_t accepted;
    uint32_t rejected;
} calibration_offset_batch_result_t;

typedef struct {
    float mean_gain;
    float mean_error;
    float stddev;
    float standard_error;
    float mean_offset;
    float mean_rmse;
    float mean_correlation;
    uint32_t accepted;
    uint32_t rejected;
} calibration_gain_batch_result_t;

typedef enum {
    CAL_OFFSET_TERMINATION_NONE = 0,
    CAL_OFFSET_TERMINATION_CONVERGED,
    CAL_OFFSET_TERMINATION_NO_IMPROVEMENT,
    CAL_OFFSET_TERMINATION_ITERATION_LIMIT,
    CAL_OFFSET_TERMINATION_VERIFICATION_FAILED,
    CAL_OFFSET_TERMINATION_ERROR
} calibration_offset_termination_t;

typedef enum {
    CAL_OFFSET_VERIFICATION_NONE = 0,
    CAL_OFFSET_VERIFICATION_PASS,
    CAL_OFFSET_VERIFICATION_MARGINAL,
    CAL_OFFSET_VERIFICATION_FAIL
} calibration_offset_verification_status_t;

/* Immutable reference selected by the most recent successful adc -cal. */
static calibration_pending_frame_t g_stored_offset_reference;
/* One-shot output handed from a successful offset stage to the gain stage. */
static calibration_pending_frame_t g_pending_calibration_frame;
static bool g_quiet_calibration_capture;

void calibration_pending_frame_invalidate(void)
{
    memset(&g_stored_offset_reference, 0,
           sizeof(g_stored_offset_reference));
    g_stored_offset_reference.selected_channel = -1;
    g_stored_offset_reference.selected_reference_phase = -1;
    memset(&g_pending_calibration_frame, 0,
           sizeof(g_pending_calibration_frame));
    g_pending_calibration_frame.selected_channel = -1;
    g_pending_calibration_frame.selected_reference_phase = -1;
}

void calibration_gain_input_frame_invalidate(void)
{
    memset(&g_pending_calibration_frame, 0,
           sizeof(g_pending_calibration_frame));
    g_pending_calibration_frame.selected_channel = -1;
    g_pending_calibration_frame.selected_reference_phase = -1;
}

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
static int calibration_prepare_uploaded_dac_reference(
    int16_t *even_reference,
    int16_t *odd_reference,
    size_t *reconstructed_count,
    double *even_variance,
    double *odd_variance,
    int print_errors
);
static int calibration_analyze_reference_frame(
    const int16_t *even_reference,
    const int16_t *odd_reference,
    const int16_t *channel_a,
    const int16_t *channel_b,
    size_t sample_count,
    int16_t *fractional_reference,
    int16_t *fractional_measurement,
    int locked_channel,
    adc_reference_analysis_t *analysis
);
static int calibration_capture_and_align(
    const int16_t *even_reference,
    const int16_t *odd_reference,
    size_t reference_count,
    const calibration_frame_config_t *config,
    calibration_frame_workspace_t *workspace,
    calibration_aligned_frame_t *frame
);
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

    /* Project lag convention: aligned[i] = signal[i + lag]. */
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
    const calibration_metrics_t *metrics,
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
                      metrics->correlation, "");
    print_float_value("Reference mean",
                      metrics->reference_mean, " codes");
    print_float_value("Measurement mean",
                      metrics->adc_mean, " codes");
    print_float_value("Offset difference",
                      metrics->offset_error_codes, " codes");
    print_float_value("Fitted offset",
                      metrics->measured_offset, " codes");
    print_float_value("Relative gain",
                      metrics->measured_gain, "");
    print_float_value("Relative gain error",
                      metrics->gain_error_ratio, "");
    print_float_value("Raw aligned RMSE",
                      metrics->rmse_codes, " codes");
    print_float_value("Gain/offset fit RMSE",
                      metrics->fitted_rmse_codes, " codes");
    print_float_value("Fitted MAE",
                      metrics->fitted_mae_codes, " codes");
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
    if (!g_quiet_calibration_capture) xil_printf("Resetting DMA...\r\n");

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

    if (!g_quiet_calibration_capture)
        xil_printf("DMA status after reset: 0x%08X\r\n", status);

    /*
     * Prepare destination buffer.
     */
    Xil_DCacheFlushRange(
        (UINTPTR)RxBufferPtr,
        DMA_CMD_BUF_SIZE
    );

    if (!g_quiet_calibration_capture)
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

    if (!g_quiet_calibration_capture) xil_printf("DMA capture complete.\r\n");

    return XST_SUCCESS;
}

void adc_timing_capture(uint32_t frame_count)
{
    static int16_t even_reference[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t odd_reference[ADC_CHANNEL_SAMPLE_COUNT];
    static calibration_frame_workspace_t frame_workspace;

    uint32_t captured_frames = 0U;
    uint32_t accepted_frames = 0U;
    size_t reference_count = 0U;
    double even_variance;
    double odd_variance;
    int timing_channel = -1;

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

    print_adc_analysis_rate_header();
    if (calibration_prepare_uploaded_dac_reference(
            even_reference, odd_reference, &reference_count,
            &even_variance, &odd_variance, 1) != 0) {
        return;
    }

    adc_sweep_active = 1;
    g_quiet_calibration_capture = true;

    xil_printf("\r\n");
    xil_printf("========================================\r\n");
    xil_printf("Starting DAC-Referenced ADC Timing Test\r\n");
    xil_printf("Reference source : uploaded DAC TXT\r\n");
    xil_printf("Requested frames : %lu\r\n",
               (unsigned long)frame_count);
    xil_printf("Samples/channel  : %u\r\n",
               ADC_CHANNEL_SAMPLE_COUNT);
    xil_printf("Analysis         : uploaded reference + circular/fractional alignment\r\n");
    xil_printf("========================================\r\n");
    print_adc_test_configuration(ADC_CHANNEL_SAMPLE_COUNT);

    for (uint32_t frame = 1U; frame <= frame_count; ++frame)
    {
        calibration_frame_config_t frame_config;
        calibration_aligned_frame_t aligned_frame;
        int analysis_status;

        xil_printf("\r\n[TIMING_FRAME_BEGIN %lu/%lu]\r\n",
                   (unsigned long)frame,
                   (unsigned long)frame_count);

        frame_config.locked_channel = timing_channel;
        frame_config.adc_gain_correction = 1.0f;
        frame_config.adc_offset_correction = 0.0f;
        frame_config.reference_scale = 1.0f;
        frame_config.reject_clipped_input = false;
        analysis_status = calibration_capture_and_align(
            even_reference, odd_reference, reference_count,
            &frame_config, &frame_workspace, &aligned_frame
        );
        if (aligned_frame.capture_succeeded) {
            ++captured_frames;
        }

        xil_printf("[TIMING_RESULT %lu/%lu]\r\n",
                   (unsigned long)frame,
                   (unsigned long)frame_count);
        xil_printf("Sample count           : %lu\r\n",
                   (unsigned long)reference_count);
        xil_printf("Channel                : %s\r\n",
                   aligned_frame.selected_channel_name);
        xil_printf("Reference phase        : %s\r\n",
                   aligned_frame.selected_phase_name);

        if ((analysis_status != 0) || !aligned_frame.frame_valid) {
            if (!aligned_frame.capture_succeeded) {
                xil_printf("[TIMING_FRAME_FAILED %lu]\r\n",
                           (unsigned long)frame);
            }
            xil_printf("Timing status          : REJECTED\r\n");
            xil_printf("Rejection reason       : %s\r\n",
                aligned_frame.rejection_reason);
            goto timing_frame_end;
        }

        xil_printf("Integer lag            : %ld samples\r\n",
                   (long)aligned_frame.integer_lag);
        print_float_value("Fractional lag", aligned_frame.fractional_lag,
                          " samples");
        print_float_value("Total estimated lag", aligned_frame.total_lag,
                          " samples");
        print_float_value("Correlation", aligned_frame.correlation, "");
        print_overlap_measurements(
            &aligned_frame.metrics, &aligned_frame.overlap);
        {
            const float normalized_gain =
                aligned_frame.metrics.measured_gain *
                (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
            const float normalized_offset =
                aligned_frame.metrics.measured_offset /
                CAL_ADC_FULL_SCALE_CODES;
            print_float_value("Normalized gain", normalized_gain, "");
            print_float_value("Scale deviation from nominal",
                              normalized_gain - 1.0f, "");
            print_float_value("Normalized DC offset", normalized_offset,
                              " full-scale");
        }

        if (timing_channel < 0)
            timing_channel = aligned_frame.selected_channel;
        ++accepted_frames;
        xil_printf("Timing status          : PASS\r\n");

timing_frame_end:
        xil_printf("[TIMING_FRAME_END %lu]\r\n",
                   (unsigned long)frame);

        if (frame < frame_count)
        {
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
        }
    }

    g_quiet_calibration_capture = false;
    adc_sweep_active = 0;

    xil_printf("\r\n");
    xil_printf("========================================\r\n");
    xil_printf("DAC-referenced timing test finished.\r\n");
    xil_printf("Captured frames    : %lu/%lu\r\n",
               (unsigned long)captured_frames,
               (unsigned long)frame_count);
    xil_printf("Accepted frames    : %lu/%lu\r\n",
               (unsigned long)accepted_frames,
               (unsigned long)frame_count);
    xil_printf("Timing channel     : %s\r\n",
               timing_channel == 0 ? "Channel A" :
               (timing_channel == 1 ? "Channel B" : "none"));
    xil_printf("Reference source   : uploaded DAC TXT\r\n");
    xil_printf("No UDP receiver was required.\r\n");
    xil_printf("========================================\r\n");
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
#define RUN_CANDIDATE(candidate_name, data, count, rate, stride) \
    do \
    { \
        float candidate_correlation; \
        size_t candidate_bin; \
        calibration_print_order_candidate(candidate_name, data, count, rate, \
            even_reference, odd_reference, stride, \
            &candidate_correlation, &candidate_bin); \
        if (candidate_correlation > best_candidate_correlation) \
        { \
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

    memset(state, 0, sizeof(*state));
    state->offset_correction = calibration_software_offset_correction();
    state->gain_correction = calibration_software_gain_correction();
    state->calibration_channel = calibration_channel_selection();
    state->final_status = CALIBRATION_OFFSET_LOOP_RUNNING;
    state->latest_correlation = 0.0f;
    state->latest_mean_residual = 0.0f;
    state->latest_fitted_offset = 0.0f;
    state->latest_fitted_gain = 0.0f;
    state->latest_rmse = 0.0f;
    state->latest_raw_mean = 0.0f;
    state->latest_corrected_mean = 0.0f;
    state->best_abs_residual = FLT_MAX;
    state->best_score = FLT_MAX;
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
           isfinite(metrics->fitted_mae_codes) &&
           isfinite(metrics->rmse_codes) &&
           isfinite(metrics->mae_codes) &&
           isfinite(metrics->correlation) &&
           isfinite(metrics->offset_error_codes) &&
           isfinite(metrics->gain_error_ratio) &&
           isfinite(metrics->adc_rms_ac) &&
           isfinite(metrics->reference_rms_ac) &&
           (fabsf(metrics->measured_gain) > FLT_EPSILON) &&
           (metrics->fitted_rmse_codes >= 0.0f) &&
           (metrics->rmse_codes >= 0.0f);
}

/*
 * Capture exactly one DMA frame and carry that same frame through channel
 * reconstruction, uploaded-reference alignment, validation, correction, and
 * regression.  The returned sample pointers remain valid until workspace is
 * reused by the next iteration.
 */
static int calibration_capture_and_align(
    const int16_t *even_reference,
    const int16_t *odd_reference,
    size_t reference_count,
    const calibration_frame_config_t *config,
    calibration_frame_workspace_t *workspace,
    calibration_aligned_frame_t *frame
)
{
    static uint32_t capture_sequence;
    adc_reference_analysis_t analysis;
    calibration_state_t fit_state;
    calibration_config_t fit_config;
    const int16_t *selected_raw_adc;
    size_t reconstructed_count = 0U;
    size_t reference_start;
    size_t measurement_start;
    size_t analysis_count;
    double raw_sum = 0.0;
    int status;

    if (frame == NULL) {
        return -1;
    }
    memset(frame, 0, sizeof(*frame));
    frame->capture_sequence = ++capture_sequence;
    frame->selected_channel = -1;
    frame->selected_reference_phase = -1;
    frame->selected_channel_name = "none";
    frame->selected_phase_name = "none";
    frame->rejection_reason = "invalid shared-frame input";

    if ((even_reference == NULL) || (odd_reference == NULL) ||
        (config == NULL) || (workspace == NULL) ||
        (reference_count != ADC_CHANNEL_SAMPLE_COUNT) ||
        (config->locked_channel < -1) || (config->locked_channel > 1) ||
        !isfinite(config->adc_gain_correction) ||
        !isfinite(config->adc_offset_correction) ||
        !isfinite(config->reference_scale) ||
        (config->adc_gain_correction <= 0.0f) ||
        (config->reference_scale <= 0.0f)) {
        return -1;
    }

    if (adc_capture_frame() != XST_SUCCESS) {
        frame->rejection_reason = "DMA capture failed";
        return -2;
    }
    frame->capture_succeeded = true;

    status = adc_reconstruct_channels(
        RxBufferPtr, DMA_CMD_BUF_SIZE,
        workspace->channel_a, ADC_CHANNEL_SAMPLE_COUNT,
        workspace->channel_b, ADC_CHANNEL_SAMPLE_COUNT,
        &reconstructed_count
    );
    if ((status != 0) || (reconstructed_count != reference_count)) {
        frame->rejection_reason = "sample reconstruction failed";
        return -3;
    }
    frame->reconstruction_succeeded = true;

    status = calibration_analyze_reference_frame(
        even_reference, odd_reference,
        workspace->channel_a, workspace->channel_b,
        reconstructed_count,
        workspace->fractional_reference,
        workspace->fractional_measurement,
        config->locked_channel, &analysis
    );

    frame->selected_channel_name =
        analysis.selected_channel_name != NULL ?
        analysis.selected_channel_name : "none";
    frame->selected_phase_name =
        analysis.selected_phase_name != NULL ?
        analysis.selected_phase_name : "none";
    frame->timing = analysis.timing;
    frame->integer_lag = analysis.timing.integer_lag;
    frame->fractional_lag = analysis.timing.fractional_lag;
    frame->total_lag = analysis.timing.total_lag;
    frame->correlation = analysis.timing.correlation;
    frame->reference_frequency_hz = analysis.reference_frequency_hz;
    frame->adc_frequency_hz = analysis.adc_frequency_hz;

    if (analysis.selected_adc == workspace->channel_a) {
        frame->selected_channel = 0;
    } else if (analysis.selected_adc == workspace->channel_b) {
        frame->selected_channel = 1;
    }
    if (analysis.selected_reference == even_reference) {
        frame->selected_reference_phase = 0;
    } else if (analysis.selected_reference == odd_reference) {
        frame->selected_reference_phase = 1;
    }

    if (status != 0) {
        frame->rejection_reason =
            analysis.failure_reason != NULL ?
            analysis.failure_reason : "invalid alignment";
        return -4;
    }

    selected_raw_adc = frame->selected_channel == 1 ?
        workspace->channel_b : workspace->channel_a;
    if (config->reject_clipped_input &&
        calibration_samples_are_clipped(
            selected_raw_adc, reconstructed_count)) {
        frame->rejection_reason = "ADC clipping detected";
        return -5;
    }

    reference_start = analysis.fit_overlap.reference_start;
    measurement_start = analysis.fit_overlap.measurement_start;
    analysis_count = analysis.fit_overlap.analysis_count;
    if ((analysis_count < CAL_MIN_ANALYSIS_SAMPLES) ||
        (reference_start + analysis_count > reconstructed_count) ||
        (measurement_start + analysis_count > reconstructed_count)) {
        frame->rejection_reason = "insufficient aligned overlap";
        return -6;
    }

    for (size_t i = 0U; i < analysis_count; ++i) {
        const int16_t raw_adc =
            workspace->fractional_measurement[measurement_start + i];
        const double corrected_adc =
            (double)raw_adc * (double)config->adc_gain_correction +
            (double)config->adc_offset_correction;
        const double scaled_reference =
            (double)workspace->fractional_reference[reference_start + i] *
            (double)config->reference_scale;
        long corrected_code;
        long reference_code;

        if (!isfinite(corrected_adc) || !isfinite(scaled_reference)) {
            frame->rejection_reason = "nonfinite corrected aligned sample";
            return -7;
        }
        corrected_code = lround(corrected_adc);
        reference_code = lround(scaled_reference);
        if ((corrected_code < INT16_MIN) || (corrected_code > INT16_MAX) ||
            (reference_code < INT16_MIN) || (reference_code > INT16_MAX) ||
            (config->reject_clipped_input &&
             ((corrected_code < CALIBRATION_ADC_MIN_CODE) ||
              (corrected_code > CALIBRATION_ADC_MAX_CODE)))) {
            frame->rejection_reason = "corrected aligned sample out of range";
            return -8;
        }

        workspace->aligned_corrected_adc[i] = (int16_t)corrected_code;
        workspace->aligned_reference[i] = (int16_t)reference_code;
        workspace->aligned_raw_adc[i] = raw_adc;
        raw_sum += (double)raw_adc;
    }

    calibration_default_config(&fit_config);
    status = calibration_init(&fit_state, &fit_config);
    if (status != CALIBRATION_OK) {
        frame->rejection_reason = "regression initialization failed";
        return -9;
    }
    status = calibration_analyze_frame(
        &fit_state,
        workspace->aligned_corrected_adc,
        workspace->aligned_reference,
        analysis_count
    );
    if ((status != CALIBRATION_OK) ||
        !calibration_fit_metrics_are_valid(&fit_state)) {
        frame->rejection_reason = "invalid aligned regression";
        return -10;
    }

    frame->aligned_reference_samples = workspace->aligned_reference;
    frame->aligned_raw_adc_samples = workspace->aligned_raw_adc;
    frame->aligned_corrected_adc_samples =
        workspace->aligned_corrected_adc;
    frame->valid_analysis_sample_count = analysis_count;
    frame->raw_aligned_adc_mean =
        (float)(raw_sum / (double)analysis_count);
    frame->metrics = fit_state.metrics;
    frame->overlap = analysis.fit_overlap;
    frame->frame_valid = true;
    frame->rejection_reason = "none";
    return 0;
}

static int calibration_pending_frame_is_compatible(const char **reason)
{
    const calibration_pending_frame_t *pending =
        &g_pending_calibration_frame;
    const double current_ratio =
        DAC_SAMPLE_RATE_HZ / adc_get_effective_sample_rate_hz();

    if (reason != NULL) {
        *reason = "No valid pending aligned frame.";
    }
    if (!pending->valid || pending->consumed) {
        return 0;
    }
    if (!reference_buffer_is_ready() ||
        (pending->reference_generation != reference_buffer_generation()) ||
        (pending->reference_length != reference_buffer_length()) ||
        (pending->reference_format != reference_buffer_format())) {
        if (reason != NULL) *reason = "Uploaded DAC reference changed.";
        return 0;
    }
    if ((pending->sample_rate_generation != g_adc_sample_rate.generation) ||
        (pending->configured_sample_rate_hz !=
             adc_get_configured_sample_rate_hz()) ||
        (pending->effective_sample_rate_hz !=
             adc_get_effective_sample_rate_hz()) ||
        (pending->dac_adc_rate_ratio != current_ratio)) {
        if (reason != NULL) *reason = "ADC/DAC sample-rate configuration changed.";
        return 0;
    }
    if (pending->channel_configuration !=
        calibration_channel_selection()) {
        if (reason != NULL) *reason = "Calibration channel setting changed.";
        return 0;
    }
    if ((pending->software_gain_correction !=
             calibration_software_gain_correction()) ||
        (pending->software_offset_correction !=
             calibration_software_offset_correction())) {
        if (reason != NULL) *reason = "Software calibration coefficients changed.";
        return 0;
    }
    if ((pending->analysis_sample_count < CAL_MIN_ANALYSIS_SAMPLES) ||
        (pending->analysis_sample_count > ADC_CHANNEL_SAMPLE_COUNT)) {
        if (reason != NULL) *reason = "Pending aligned sample count is invalid.";
        return 0;
    }
    return 1;
}

static int calibration_stored_reference_is_compatible(const char **reason)
{
    const calibration_pending_frame_t *saved = &g_stored_offset_reference;
    const double current_ratio =
        DAC_SAMPLE_RATE_HZ / adc_get_effective_sample_rate_hz();

    if (reason != NULL) *reason = "No valid stored adc -cal frame.";
    if (!saved->valid || saved->consumed) return 0;
    if (!reference_buffer_is_ready() ||
        saved->reference_generation != reference_buffer_generation() ||
        saved->reference_length != reference_buffer_length() ||
        saved->reference_format != reference_buffer_format()) {
        if (reason != NULL) *reason = "Uploaded DAC reference changed.";
        return 0;
    }
    if (saved->sample_rate_generation != g_adc_sample_rate.generation ||
        saved->configured_sample_rate_hz != adc_get_configured_sample_rate_hz() ||
        saved->effective_sample_rate_hz != adc_get_effective_sample_rate_hz() ||
        saved->dac_adc_rate_ratio != current_ratio) {
        if (reason != NULL) *reason = "ADC/DAC sample-rate configuration changed.";
        return 0;
    }
    if (saved->channel_configuration != calibration_channel_selection() ||
        saved->software_gain_correction != calibration_software_gain_correction()) {
        if (reason != NULL) *reason = "Calibration channel or gain changed.";
        return 0;
    }
    if (saved->analysis_sample_count < CAL_MIN_ANALYSIS_SAMPLES ||
        saved->analysis_sample_count > ADC_CHANNEL_SAMPLE_COUNT) {
        if (reason != NULL) *reason = "Stored reference sample count is invalid.";
        return 0;
    }
    return 1;
}

static int calibration_pending_frame_copy(
    calibration_pending_frame_t *destination,
    const calibration_aligned_frame_t *frame,
    uint32_t frame_number,
    uint32_t reference_generation,
    size_t reference_length,
    reference_buffer_format_t reference_format
)
{
    const size_t sample_count = frame != NULL ?
        frame->valid_analysis_sample_count : 0U;

    if ((destination == NULL) || (frame == NULL) || !frame->frame_valid ||
        (frame->aligned_reference_samples == NULL) ||
        (frame->aligned_raw_adc_samples == NULL) ||
        (frame->aligned_corrected_adc_samples == NULL) ||
        (sample_count < CAL_MIN_ANALYSIS_SAMPLES) ||
        (sample_count > ADC_CHANNEL_SAMPLE_COUNT) ||
        !reference_buffer_is_ready() ||
        (reference_generation != reference_buffer_generation()) ||
        (reference_length != reference_buffer_length()) ||
        (reference_format != reference_buffer_format())) {
        return -1;
    }

    memset(destination, 0, sizeof(*destination));
    destination->selected_channel = -1;
    destination->selected_reference_phase = -1;

    destination->capture_sequence = frame->capture_sequence;
    destination->retained_frame_number = frame_number;
    destination->selected_channel = frame->selected_channel;
    destination->selected_reference_phase = frame->selected_reference_phase;
    destination->integer_lag = frame->integer_lag;
    destination->fractional_lag = frame->fractional_lag;
    destination->total_lag = frame->total_lag;
    destination->analysis_sample_count = sample_count;
    destination->raw_aligned_adc_mean = frame->raw_aligned_adc_mean;
    destination->correlation = frame->correlation;
    destination->metrics = frame->metrics;
    destination->timing = frame->timing;
    destination->overlap = frame->overlap;
    destination->reference_frequency_hz = frame->reference_frequency_hz;
    destination->adc_frequency_hz = frame->adc_frequency_hz;
    memcpy(destination->aligned_reference,
           frame->aligned_reference_samples,
           sample_count * sizeof(destination->aligned_reference[0]));
    memcpy(destination->aligned_raw_adc,
           frame->aligned_raw_adc_samples,
           sample_count * sizeof(destination->aligned_raw_adc[0]));
    memcpy(destination->aligned_corrected_adc,
           frame->aligned_corrected_adc_samples,
           sample_count * sizeof(destination->aligned_corrected_adc[0]));
    destination->reference_generation = reference_generation;
    destination->reference_length = reference_length;
    destination->reference_format = reference_format;
    destination->sample_rate_generation = g_adc_sample_rate.generation;
    destination->configured_sample_rate_hz =
        adc_get_configured_sample_rate_hz();
    destination->effective_sample_rate_hz =
        adc_get_effective_sample_rate_hz();
    destination->dac_adc_rate_ratio =
        DAC_SAMPLE_RATE_HZ / adc_get_effective_sample_rate_hz();
    destination->channel_configuration = calibration_channel_selection();
    destination->software_gain_correction =
        calibration_software_gain_correction();
    destination->software_offset_correction =
        calibration_software_offset_correction();
    destination->consumed = false;
    destination->valid = true;
    return 0;
}

/*
 * Select a typical accepted frame, not the final frame or the frame with the
 * best correlation.  Ranking is lexicographic and therefore deterministic.
 */
static int calibration_select_representative_frame(
    const calibration_pending_frame_t *candidates,
    size_t candidate_count,
    size_t *selected_index,
    calibration_selection_medians_t *medians
)
{
    float offsets[ADC_CAL_MAX_FRAMES];
    float normalized_gains[ADC_CAL_MAX_FRAMES];
    float fitted_rmse[ADC_CAL_MAX_FRAMES];
    size_t best_index = 0U;

    if ((candidates == NULL) || (selected_index == NULL) ||
        (medians == NULL) || (candidate_count == 0U) ||
        (candidate_count > ADC_CAL_MAX_FRAMES)) {
        return -1;
    }

    for (size_t i = 0U; i < candidate_count; ++i) {
        if (!candidates[i].valid || candidates[i].consumed ||
            !isfinite(candidates[i].metrics.measured_offset) ||
            !isfinite(candidates[i].metrics.measured_gain) ||
            !isfinite(candidates[i].metrics.fitted_rmse_codes) ||
            !isfinite(candidates[i].correlation)) {
            return -2;
        }
        offsets[i] = candidates[i].metrics.measured_offset;
        normalized_gains[i] = candidates[i].metrics.measured_gain *
            (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
        fitted_rmse[i] = candidates[i].metrics.fitted_rmse_codes;
    }

    medians->median_fitted_offset =
        median_float(offsets, candidate_count);
    medians->median_normalized_gain =
        median_float(normalized_gains, candidate_count);
    medians->median_fitted_rmse =
        median_float(fitted_rmse, candidate_count);
    if (!isfinite(medians->median_fitted_offset) ||
        !isfinite(medians->median_normalized_gain) ||
        !isfinite(medians->median_fitted_rmse)) {
        return -3;
    }

    for (size_t i = 1U; i < candidate_count; ++i) {
        const float candidate_offset_distance = fabsf(
            candidates[i].metrics.measured_offset -
            medians->median_fitted_offset);
        const float best_offset_distance = fabsf(
            candidates[best_index].metrics.measured_offset -
            medians->median_fitted_offset);
        const float candidate_normalized_gain =
            candidates[i].metrics.measured_gain *
            (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
        const float best_normalized_gain =
            candidates[best_index].metrics.measured_gain *
            (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
        const float candidate_gain_distance = fabsf(
            candidate_normalized_gain - medians->median_normalized_gain);
        const float best_gain_distance = fabsf(
            best_normalized_gain - medians->median_normalized_gain);
        int choose_candidate = 0;

        if (candidate_offset_distance <
            best_offset_distance - CAL_REPRESENTATIVE_TIE_EPSILON) {
            choose_candidate = 1;
        } else if (fabsf(candidate_offset_distance - best_offset_distance) <=
                   CAL_REPRESENTATIVE_TIE_EPSILON) {
            if (candidate_gain_distance <
                best_gain_distance - CAL_REPRESENTATIVE_TIE_EPSILON) {
                choose_candidate = 1;
            } else if (fabsf(candidate_gain_distance - best_gain_distance) <=
                       CAL_REPRESENTATIVE_TIE_EPSILON) {
                if (candidates[i].metrics.fitted_rmse_codes <
                    candidates[best_index].metrics.fitted_rmse_codes -
                        CAL_REPRESENTATIVE_TIE_EPSILON) {
                    choose_candidate = 1;
                } else if (fabsf(
                        candidates[i].metrics.fitted_rmse_codes -
                        candidates[best_index].metrics.fitted_rmse_codes) <=
                           CAL_REPRESENTATIVE_TIE_EPSILON) {
                    if (candidates[i].correlation >
                        candidates[best_index].correlation +
                            CAL_REPRESENTATIVE_TIE_EPSILON) {
                        choose_candidate = 1;
                    } else if (fabsf(candidates[i].correlation -
                                     candidates[best_index].correlation) <=
                                   CAL_REPRESENTATIVE_TIE_EPSILON &&
                               candidates[i].retained_frame_number <
                                   candidates[best_index].retained_frame_number) {
                        choose_candidate = 1;
                    }
                }
            }
        }

        if (choose_candidate) {
            best_index = i;
        }
    }

    *selected_index = best_index;
    return 0;
}

static int calibration_restore_owned_frame(
    const calibration_pending_frame_t *source,
    calibration_frame_workspace_t *workspace,
    calibration_aligned_frame_t *frame
)
{
    const size_t sample_count = source != NULL ?
        source->analysis_sample_count : 0U;

    if ((source == NULL) || !source->valid || source->consumed ||
        (workspace == NULL) || (frame == NULL) ||
        (sample_count < CAL_MIN_ANALYSIS_SAMPLES) ||
        (sample_count > ADC_CHANNEL_SAMPLE_COUNT)) {
        return -1;
    }

    memset(frame, 0, sizeof(*frame));
    memcpy(workspace->aligned_reference,
           source->aligned_reference,
           sample_count * sizeof(workspace->aligned_reference[0]));
    memcpy(workspace->aligned_raw_adc,
           source->aligned_raw_adc,
           sample_count * sizeof(workspace->aligned_raw_adc[0]));
    memcpy(workspace->aligned_corrected_adc,
           source->aligned_corrected_adc,
           sample_count * sizeof(workspace->aligned_corrected_adc[0]));
    frame->capture_sequence = source->capture_sequence;
    frame->retained_frame_number = source->retained_frame_number;
    frame->capture_succeeded = true;
    frame->reconstruction_succeeded = true;
    frame->frame_valid = true;
    frame->selected_channel = source->selected_channel;
    frame->selected_reference_phase = source->selected_reference_phase;
    frame->selected_channel_name = source->selected_channel == 0 ?
        "Channel A" : "Channel B";
    frame->selected_phase_name = source->selected_reference_phase == 0 ?
        "EVEN" : "ODD";
    frame->integer_lag = source->integer_lag;
    frame->fractional_lag = source->fractional_lag;
    frame->total_lag = source->total_lag;
    frame->aligned_reference_samples = workspace->aligned_reference;
    frame->aligned_raw_adc_samples = workspace->aligned_raw_adc;
    frame->aligned_corrected_adc_samples =
        workspace->aligned_corrected_adc;
    frame->valid_analysis_sample_count = sample_count;
    frame->raw_aligned_adc_mean = source->raw_aligned_adc_mean;
    frame->correlation = source->correlation;
    frame->metrics = source->metrics;
    frame->timing = source->timing;
    frame->overlap = source->overlap;
    frame->reference_frequency_hz = source->reference_frequency_hz;
    frame->adc_frequency_hz = source->adc_frequency_hz;
    frame->rejection_reason = "none";
    return 0;
}

/* Capture a fresh batch and apply the same representative selector as adc -cal. */
static int calibration_capture_representative_batch(
    const int16_t *even_reference,
    const int16_t *odd_reference,
    size_t reference_count,
    const calibration_frame_config_t *config,
    calibration_frame_workspace_t *workspace,
    calibration_aligned_frame_t *selected_frame,
    uint32_t *accepted_count,
    const char **reason
)
{
    static calibration_pending_frame_t
        candidates[CAL_UPDATE_FRAME_BATCH_SIZE];
    calibration_selection_medians_t medians;
    size_t candidate_count = 0U;
    size_t selected_index = 0U;
    const uint32_t reference_generation = reference_buffer_generation();
    const size_t reference_length = reference_buffer_length();
    const reference_buffer_format_t reference_format =
        reference_buffer_format();
    const uint32_t required =
        CAL_UPDATE_FRAME_BATCH_SIZE < CAL_TIMING_MIN_ACCEPTED_FRAMES ?
        CAL_UPDATE_FRAME_BATCH_SIZE : CAL_TIMING_MIN_ACCEPTED_FRAMES;

    if (accepted_count != NULL) *accepted_count = 0U;
    if (reason != NULL) *reason = "fresh frame batch is invalid";
    if ((even_reference == NULL) || (odd_reference == NULL) ||
        (config == NULL) || (workspace == NULL) ||
        (selected_frame == NULL)) {
        return -1;
    }

    for (uint32_t frame_number = 1U;
         frame_number <= CAL_UPDATE_FRAME_BATCH_SIZE;
         ++frame_number) {
        calibration_aligned_frame_t captured_frame;
        const int status = calibration_capture_and_align(
            even_reference, odd_reference, reference_count,
            config, workspace, &captured_frame
        );

        if ((status == 0) && captured_frame.frame_valid &&
            (calibration_pending_frame_copy(
                &candidates[candidate_count], &captured_frame,
                frame_number, reference_generation, reference_length,
                reference_format) == 0)) {
            ++candidate_count;
        }
        if (frame_number < CAL_UPDATE_FRAME_BATCH_SIZE) {
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
        }
    }

    if (accepted_count != NULL) {
        *accepted_count = (uint32_t)candidate_count;
    }
    if (!reference_buffer_is_ready() ||
        (reference_generation != reference_buffer_generation()) ||
        (reference_length != reference_buffer_length()) ||
        (reference_format != reference_buffer_format())) {
        if (reason != NULL) *reason =
            "uploaded reference changed during fresh selection batch";
        return -2;
    }
    if ((candidate_count < required) ||
        ((float)candidate_count / (float)CAL_UPDATE_FRAME_BATCH_SIZE <
         CAL_TIMING_MIN_ACCEPTANCE_RATE)) {
        if (reason != NULL) *reason =
            "insufficient accepted frames in fresh selection batch";
        return -3;
    }
    if (calibration_select_representative_frame(
            candidates, candidate_count, &selected_index, &medians) != 0) {
        if (reason != NULL) *reason =
            "representative frame selection failed";
        return -4;
    }
    if (calibration_restore_owned_frame(
            &candidates[selected_index], workspace, selected_frame) != 0) {
        if (reason != NULL) *reason =
            "selected frame restoration failed";
        return -5;
    }

    if (reason != NULL) *reason = "none";
    return 0;
}

static int calibration_pending_frame_consume(
    float adc_gain_correction,
    float adc_offset_correction,
    float reference_scale,
    calibration_frame_workspace_t *workspace,
    calibration_aligned_frame_t *frame,
    const char **reason
)
{
    calibration_pending_frame_t *pending =
        &g_pending_calibration_frame;
    calibration_state_t fit_state;
    calibration_config_t fit_config;
    const size_t sample_count = pending->analysis_sample_count;
    int status;

    if ((workspace == NULL) || (frame == NULL) ||
        !isfinite(adc_gain_correction) ||
        !isfinite(adc_offset_correction) ||
        !isfinite(reference_scale) || (adc_gain_correction <= 0.0f) ||
        (reference_scale <= 0.0f)) {
        if (reason != NULL) *reason = "Invalid pending-frame consumer.";
        return -1;
    }
    if (!calibration_pending_frame_is_compatible(reason)) {
        calibration_gain_input_frame_invalidate();
        return -2;
    }

    /* Claim the pending frame before copying it into the loop workspace. */
    pending->consumed = true;
    pending->valid = false;
    memset(frame, 0, sizeof(*frame));
    frame->capture_sequence = pending->capture_sequence;
    frame->retained_frame_number = pending->retained_frame_number;
    frame->capture_succeeded = true;
    frame->reconstruction_succeeded = true;
    frame->selected_channel = pending->selected_channel;
    frame->selected_reference_phase = pending->selected_reference_phase;
    frame->selected_channel_name = pending->selected_channel == 0 ?
        "Channel A" : "Channel B";
    frame->selected_phase_name = pending->selected_reference_phase == 0 ?
        "EVEN" : "ODD";
    frame->integer_lag = pending->integer_lag;
    frame->fractional_lag = pending->fractional_lag;
    frame->total_lag = pending->total_lag;
    frame->correlation = pending->correlation;
    frame->timing = pending->timing;
    frame->overlap = pending->overlap;
    frame->reference_frequency_hz = pending->reference_frequency_hz;
    frame->adc_frequency_hz = pending->adc_frequency_hz;
    frame->raw_aligned_adc_mean = pending->raw_aligned_adc_mean;

    for (size_t i = 0U; i < sample_count; ++i) {
        const int16_t raw_adc = pending->aligned_raw_adc[i];
        const double corrected_adc =
            (double)raw_adc * (double)adc_gain_correction +
            (double)adc_offset_correction;
        const double scaled_reference =
            (double)pending->aligned_reference[i] *
            (double)reference_scale;
        const long corrected_code = lround(corrected_adc);
        const long reference_code = lround(scaled_reference);
        if (!isfinite(corrected_adc) || !isfinite(scaled_reference) ||
            (corrected_code < CALIBRATION_ADC_MIN_CODE) ||
            (corrected_code > CALIBRATION_ADC_MAX_CODE) ||
            (reference_code < INT16_MIN) ||
            (reference_code > INT16_MAX)) {
            if (reason != NULL) *reason =
                "Pending aligned sample correction failed.";
            return -3;
        }
        workspace->aligned_raw_adc[i] = raw_adc;
        workspace->aligned_reference[i] = (int16_t)reference_code;
        workspace->aligned_corrected_adc[i] = (int16_t)corrected_code;
    }

    calibration_default_config(&fit_config);
    status = calibration_init(&fit_state, &fit_config);
    if (status == CALIBRATION_OK) {
        status = calibration_analyze_frame(
            &fit_state,
            workspace->aligned_corrected_adc,
            workspace->aligned_reference,
            sample_count
        );
    }
    if ((status != CALIBRATION_OK) ||
        !calibration_fit_metrics_are_valid(&fit_state)) {
        if (reason != NULL) *reason = "Pending-frame regression failed.";
        return -4;
    }

    frame->aligned_reference_samples = workspace->aligned_reference;
    frame->aligned_raw_adc_samples = workspace->aligned_raw_adc;
    frame->aligned_corrected_adc_samples =
        workspace->aligned_corrected_adc;
    frame->valid_analysis_sample_count = sample_count;
    frame->metrics = fit_state.metrics;
    frame->frame_valid = true;
    frame->rejection_reason = "none";
    if (reason != NULL) *reason = "none";
    return 0;
}

/* Capture one fresh ADC frame and align its fixed channel to the immutable
 * waveform retained by adc -cal. */
static int calibration_capture_against_owned_reference(
    const calibration_pending_frame_t *saved,
    bool use_saved_calibration_reference,
    float gain_correction,
    float offset_correction,
    float reference_scale,
    calibration_frame_workspace_t *workspace,
    calibration_aligned_frame_t *frame,
    const char **reason)
{
    calibration_timing_frame_result_t phase_results[2];
    calibration_state_t fit_state;
    calibration_config_t fit_config;
    const int16_t *captured_channel;
    const int16_t *phase_candidates[2];
    const int16_t *selected_candidate;
    const calibration_timing_frame_result_t *selected_result = NULL;
    size_t selected_phase = 0U;
    size_t reconstructed_count = 0U;
    size_t reference_start, measurement_start, count;
    double raw_sum = 0.0;
    int status;

    if (reason != NULL) *reason = "fresh ADC capture is invalid";
    if (saved == NULL || !saved->valid || workspace == NULL || frame == NULL ||
        !isfinite(gain_correction) || gain_correction <= 0.0f ||
        !isfinite(offset_correction) || !isfinite(reference_scale) ||
        reference_scale <= 0.0f) return -1;
    memset(frame, 0, sizeof(*frame));
    if (adc_capture_frame() != XST_SUCCESS) return -2;
    if (adc_reconstruct_channels(RxBufferPtr, DMA_CMD_BUF_SIZE,
            workspace->channel_a, ADC_CHANNEL_SAMPLE_COUNT,
            workspace->channel_b, ADC_CHANNEL_SAMPLE_COUNT,
            &reconstructed_count) != 0 ||
        reconstructed_count <= saved->analysis_sample_count) {
        if (reason != NULL) *reason = "ADC reconstruction failed";
        return -3;
    }
    captured_channel = saved->selected_channel == 1 ?
        workspace->channel_b : workspace->channel_a;
    /* DMA may start on either converter sample phase.  Keep the saved
     * waveform fixed, but independently test both possible fresh starts. */
    phase_candidates[0] = captured_channel;
    phase_candidates[1] = captured_channel + 1U;
    for (size_t phase = 0U; phase < 2U; ++phase) {
        (void)adc_measure_timing_frame(saved->aligned_corrected_adc,
            phase_candidates[phase], saved->analysis_sample_count,
            workspace->fractional_reference,
            workspace->fractional_measurement, &phase_results[phase]);
    }
    frame->even_candidate_correlation = phase_results[0].correlation;
    frame->odd_candidate_correlation = phase_results[1].correlation;
    frame->even_candidate_lag = phase_results[0].integer_lag;
    frame->odd_candidate_lag = phase_results[1].integer_lag;
    frame->even_candidate_valid = phase_results[0].alignment_success;
    frame->odd_candidate_valid = phase_results[1].alignment_success;
    if (phase_results[0].alignment_success &&
        phase_results[1].alignment_success) {
        if (phase_results[0].correlation >= phase_results[1].correlation) {
            selected_phase = 0U;
            selected_result = &phase_results[0];
        } else {
            selected_phase = 1U;
            selected_result = &phase_results[1];
        }
    } else if (phase_results[0].alignment_success) {
        selected_phase = 0U;
        selected_result = &phase_results[0];
    } else if (phase_results[1].alignment_success) {
        selected_phase = 1U;
        selected_result = &phase_results[1];
    } else {
        if (reason != NULL) *reason = "neither fresh phase candidate aligned";
        return -4;
    }
    selected_candidate = phase_candidates[selected_phase];
    frame->selected_reference_phase = (int8_t)selected_phase;
    frame->selected_phase_name = selected_phase == 0U ? "EVEN" : "ODD";
    frame->integer_lag = selected_result->integer_lag;
    frame->fractional_lag = selected_result->fractional_lag;
    frame->total_lag = selected_result->total_lag;
    frame->correlation = selected_result->correlation;
    if (!isfinite(selected_result->correlation)) {
        if (reason != NULL) *reason = "selected correlation is not finite";
        return -4;
    }
    if (selected_result->correlation < CAL_DAC_REF_MIN_CORRELATION) {
        if (reason != NULL) *reason = "correlation to stored adc -cal frame below threshold";
        return -4;
    }
    if (adc_analyze_fractional_overlap(
            use_saved_calibration_reference ? saved->aligned_reference :
                                              saved->aligned_corrected_adc,
            selected_candidate, saved->analysis_sample_count,
            (double)selected_result->total_lag,
            workspace->fractional_reference,
            workspace->fractional_measurement, &fit_state,
            &frame->overlap) != 0) {
        if (reason != NULL) *reason = "insufficient overlap with stored reference";
        return -5;
    }
    reference_start = frame->overlap.reference_start;
    measurement_start = frame->overlap.measurement_start;
    count = frame->overlap.analysis_count;
    if (count < CAL_MIN_ANALYSIS_SAMPLES ||
        reference_start + count > saved->analysis_sample_count ||
        measurement_start + count > saved->analysis_sample_count) return -6;

    for (size_t i = 0U; i < count; ++i) {
        const int16_t raw = workspace->fractional_measurement[measurement_start + i];
        const double corrected =
            (double)raw * (double)gain_correction +
            (double)offset_correction;
        const long code = lround(corrected);
        if (!isfinite(corrected) || code < CALIBRATION_ADC_MIN_CODE ||
            code > CALIBRATION_ADC_MAX_CODE) {
            if (reason != NULL) *reason = "corrected ADC sample out of range";
            return -7;
        }
        {
            const long reference_code = lround(
                (double)workspace->fractional_reference[reference_start + i] *
                (double)reference_scale);
            if (reference_code < INT16_MIN || reference_code > INT16_MAX) {
                if (reason != NULL) *reason = "scaled reference out of range";
                return -7;
            }
            workspace->aligned_reference[i] = (int16_t)reference_code;
        }
        workspace->aligned_raw_adc[i] = raw;
        workspace->aligned_corrected_adc[i] = (int16_t)code;
        raw_sum += (double)raw;
    }
    calibration_default_config(&fit_config);
    status = calibration_init(&fit_state, &fit_config);
    if (status == CALIBRATION_OK)
        status = calibration_analyze_frame(&fit_state,
            workspace->aligned_corrected_adc, workspace->aligned_reference,
            count);
    if (status != CALIBRATION_OK || !calibration_fit_metrics_are_valid(&fit_state)) {
        if (reason != NULL) *reason = "stored-reference regression failed";
        return -8;
    }
    frame->capture_succeeded = true;
    frame->reconstruction_succeeded = true;
    frame->frame_valid = true;
    frame->selected_channel = saved->selected_channel;
    frame->selected_channel_name = saved->selected_channel == 0 ? "Channel A" : "Channel B";
    frame->integer_lag = selected_result->integer_lag;
    frame->fractional_lag = selected_result->fractional_lag;
    frame->total_lag = selected_result->total_lag;
    frame->correlation = selected_result->correlation;
    frame->timing = *selected_result;
    frame->aligned_reference_samples = workspace->aligned_reference;
    frame->aligned_raw_adc_samples = workspace->aligned_raw_adc;
    frame->aligned_corrected_adc_samples = workspace->aligned_corrected_adc;
    frame->valid_analysis_sample_count = count;
    frame->raw_aligned_adc_mean = (float)(raw_sum / (double)count);
    frame->metrics = fit_state.metrics;
    frame->rejection_reason = "none";
    if (reason != NULL) *reason = "none";
    return 0;
}

static int calibration_capture_against_stored_reference(
    float offset_correction,
    calibration_frame_workspace_t *workspace,
    calibration_aligned_frame_t *frame,
    const char **reason)
{
    if (!calibration_stored_reference_is_compatible(reason)) return -1;
    return calibration_capture_against_owned_reference(
        &g_stored_offset_reference, false, 1.0f, offset_correction, 1.0f,
        workspace, frame, reason);
}

static int calibration_compute_offset_batch(
    float offset_correction,
    calibration_frame_workspace_t *workspace,
    calibration_offset_loop_state_t *state,
    calibration_offset_batch_result_t *batch)
{
    double sum = 0.0;
    double sum_squares = 0.0;
    double correlation_sum = 0.0;
    double fitted_rmse_sum = 0.0;

    if (workspace == NULL || state == NULL || batch == NULL) return -1;
    memset(batch, 0, sizeof(*batch));
    batch->minimum = FLT_MAX;
    batch->maximum = -FLT_MAX;

    for (uint32_t frame_number = 1U;
         frame_number <= CALIBRATION_OFFSET_BATCH_SIZE;
         ++frame_number) {
        calibration_aligned_frame_t frame;
        const char *reason = NULL;
        float residual;
        int status;

        xil_printf("\r\nFrame %lu/%u\r\n",
                   (unsigned long)frame_number,
                   CALIBRATION_OFFSET_BATCH_SIZE);
        status = calibration_capture_against_stored_reference(
            offset_correction, workspace, &frame, &reason);
        print_float_value("EVEN correlation",
                          frame.even_candidate_correlation, "");
        print_float_value("ODD correlation",
                          frame.odd_candidate_correlation, "");
        if (frame.even_candidate_valid || frame.odd_candidate_valid) {
            xil_printf("Selected phase          : %s\r\n",
                       frame.selected_phase_name);
            print_float_value("Selected correlation", frame.correlation, "");
            xil_printf("Selected integer lag    : %ld samples\r\n",
                       (long)frame.integer_lag);
            print_float_value("Selected fractional lag",
                              frame.fractional_lag, " samples");
        }
        if (status != 0 || !frame.frame_valid ||
            !isfinite(frame.metrics.adc_mean) ||
            !isfinite(frame.metrics.fitted_rmse_codes) ||
            !isfinite(frame.correlation)) {
            ++batch->rejected;
            ++state->rejected_frame_count;
            xil_printf("Status                  : REJECTED\r\n");
            xil_printf("Rejection reason        : %s\r\n",
                       reason != NULL ? reason : "invalid frame metrics");
        } else {
            residual = frame.metrics.adc_mean -
                       g_stored_offset_reference.metrics.adc_mean;
            if (!isfinite(residual)) {
                ++batch->rejected;
                ++state->rejected_frame_count;
                xil_printf("Status                  : REJECTED\r\n");
                xil_printf("Rejection reason        : nonfinite residual\r\n");
            } else {
                ++batch->accepted;
                ++state->accepted_frame_count;
                sum += residual;
                sum_squares += (double)residual * (double)residual;
                correlation_sum += frame.correlation;
                fitted_rmse_sum += frame.metrics.fitted_rmse_codes;
                if (residual < batch->minimum) batch->minimum = residual;
                if (residual > batch->maximum) batch->maximum = residual;
                state->latest_fitted_offset = frame.metrics.measured_offset;
                state->latest_fitted_gain = frame.metrics.measured_gain;
                state->latest_raw_mean = frame.raw_aligned_adc_mean;
                state->latest_corrected_mean = frame.metrics.adc_mean;
                print_float_value("Residual", residual, " codes");
                xil_printf("Status                  : ACCEPTED\r\n");
            }
        }
        if (frame_number < CALIBRATION_OFFSET_BATCH_SIZE)
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
    }

    if (batch->accepted == 0U) return -2;
    batch->mean = (float)(sum / (double)batch->accepted);
    batch->rmse = (float)sqrt(sum_squares / (double)batch->accepted);
    batch->stddev = (float)sqrt(fmax(0.0,
        sum_squares / (double)batch->accepted -
        (double)batch->mean * (double)batch->mean));
    batch->mean_correlation =
        (float)(correlation_sum / (double)batch->accepted);
    batch->mean_fitted_rmse =
        (float)(fitted_rmse_sum / (double)batch->accepted);
    return 0;
}

static int calibration_compute_gain_batch(
    const calibration_pending_frame_t *input,
    float gain_correction,
    float fixed_offset_correction,
    calibration_frame_workspace_t *workspace,
    calibration_gain_loop_state_t *state,
    calibration_gain_batch_result_t *batch)
{
    double gain_sum = 0.0, gain_square_sum = 0.0;
    double offset_sum = 0.0, rmse_sum = 0.0, correlation_sum = 0.0;

    if (input == NULL || workspace == NULL || state == NULL || batch == NULL)
        return -1;
    memset(batch, 0, sizeof(*batch));
    for (uint32_t frame_number = 1U;
         frame_number <= CALIBRATION_GAIN_BATCH_SIZE; ++frame_number) {
        calibration_aligned_frame_t frame;
        const char *reason = NULL;
        int status;

        xil_printf("\r\nGain frame %lu/%u\r\n",
                   (unsigned long)frame_number,
                   CALIBRATION_GAIN_BATCH_SIZE);
        status = calibration_capture_against_owned_reference(
            input, true, gain_correction,
            gain_correction * fixed_offset_correction,
            CAL_ADC_FULL_SCALE_CODES / CAL_DAC_FULL_SCALE_CODES,
            workspace, &frame, &reason);
        if (status != 0 || !frame.frame_valid ||
            !isfinite(frame.metrics.measured_gain) ||
            !isfinite(frame.metrics.measured_offset) ||
            !isfinite(frame.metrics.fitted_rmse_codes) ||
            !isfinite(frame.correlation)) {
            ++batch->rejected;
            ++state->rejected_frame_count;
            xil_printf("Status                  : REJECTED\r\n");
            xil_printf("Reason                  : %s\r\n",
                       reason != NULL ? reason : "invalid gain metrics");
        } else {
            const float measured_gain = frame.metrics.measured_gain;
            ++batch->accepted;
            ++state->accepted_frame_count;
            gain_sum += measured_gain;
            gain_square_sum +=
                (double)measured_gain * (double)measured_gain;
            offset_sum += frame.metrics.measured_offset;
            rmse_sum += frame.metrics.fitted_rmse_codes;
            correlation_sum += frame.correlation;
            print_float_value("Regression gain", measured_gain, "");
            print_float_value("Gain error", measured_gain - 1.0f, "");
            print_float_value("Regression offset",
                              frame.metrics.measured_offset, " codes");
            print_float_value("Regression RMSE",
                              frame.metrics.fitted_rmse_codes, " codes");
            print_float_value("Correlation", frame.correlation, "");
            xil_printf("Status                  : ACCEPTED\r\n");
        }
        if (frame_number < CALIBRATION_GAIN_BATCH_SIZE)
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
    }
    if (batch->accepted == 0U) return -2;
    batch->mean_gain = (float)(gain_sum / (double)batch->accepted);
    batch->mean_error = batch->mean_gain - 1.0f;
    batch->stddev = (float)sqrt(fmax(0.0,
        gain_square_sum / (double)batch->accepted -
        (double)batch->mean_gain * (double)batch->mean_gain));
    batch->standard_error =
        batch->stddev / sqrtf((float)batch->accepted);
    batch->mean_offset = (float)(offset_sum / (double)batch->accepted);
    batch->mean_rmse = (float)(rmse_sum / (double)batch->accepted);
    batch->mean_correlation =
        (float)(correlation_sum / (double)batch->accepted);
    return 0;
}

/*
 * Publish a fresh representative capture measured with the final coefficient.
 * The stored reference remains at its canonical adc -cal scale so a later
 * calibration stage can apply its own stage-specific scaling exactly once.
 */
static int calibration_publish_final_capture(
    const char *stage_name,
    const int16_t *even_reference,
    const int16_t *odd_reference,
    size_t reference_count,
    const calibration_frame_config_t *config,
    calibration_frame_workspace_t *workspace)
{
    calibration_aligned_frame_t selected_frame;
    uint32_t accepted_count = 0U;
    const char *reason = NULL;
    int status;

    calibration_gain_input_frame_invalidate();
    xil_printf("\r\n========== Calibrated Capture Output ==========\r\n");
    xil_printf("Source stage             : %s\r\n",
               stage_name != NULL ? stage_name : "unknown");
    xil_printf("Capture timing           : after final coefficient update\r\n");

    usleep(ADC_TIMING_INTERFRAME_DELAY_US);
    status = calibration_capture_representative_batch(
        even_reference, odd_reference, reference_count, config, workspace,
        &selected_frame, &accepted_count, &reason
    );
    xil_printf("Accepted output frames   : %lu/%u\r\n",
               (unsigned long)accepted_count,
               CAL_UPDATE_FRAME_BATCH_SIZE);
    if ((status != 0) || !selected_frame.frame_valid) {
        xil_printf("Pending input frame      : none\r\n");
        xil_printf("Capture output status    : NOT READY\r\n");
        xil_printf("Reason                   : %s\r\n",
                   reason != NULL ? reason : "final capture failed");
        xil_printf("===============================================\r\n");
        return -1;
    }

    status = calibration_pending_frame_copy(
        &g_pending_calibration_frame, &selected_frame,
        selected_frame.retained_frame_number,
        reference_buffer_generation(), reference_buffer_length(),
        reference_buffer_format()
    );
    if (status != 0) {
        calibration_gain_input_frame_invalidate();
        xil_printf("Pending input frame      : none\r\n");
        xil_printf("Capture output status    : NOT READY\r\n");
        xil_printf("Reason                   : pending-frame copy failed\r\n");
        xil_printf("===============================================\r\n");
        return -2;
    }

    xil_printf("Pending input frame      : Frame %lu\r\n",
               (unsigned long)selected_frame.retained_frame_number);
    xil_printf("Selection reason         : Closest to median calibration metrics\r\n");
    xil_printf("Channel                  : %s\r\n",
               selected_frame.selected_channel_name);
    xil_printf("Reference phase          : %s\r\n",
               selected_frame.selected_phase_name);
    print_float_value("Correlation", selected_frame.correlation, "");
    xil_printf("Integer lag              : %ld samples\r\n",
               (long)selected_frame.integer_lag);
    print_float_value("Fractional lag",
                      selected_frame.fractional_lag, " samples");
    xil_printf("Valid aligned samples    : %lu\r\n",
               (unsigned long)selected_frame.valid_analysis_sample_count);
    xil_printf("Capture output status    : READY\r\n");
    xil_printf("===============================================\r\n");
    return 0;
}

static const char *calibration_offset_termination_name(uint8_t reason)
{
    switch ((calibration_offset_termination_t)reason) {
    case CAL_OFFSET_TERMINATION_CONVERGED:
        return "converged";
    case CAL_OFFSET_TERMINATION_NO_IMPROVEMENT:
        return "no further improvement";
    case CAL_OFFSET_TERMINATION_ITERATION_LIMIT:
        return "iteration limit";
    case CAL_OFFSET_TERMINATION_VERIFICATION_FAILED:
        return "verification outside hard limit";
    case CAL_OFFSET_TERMINATION_ERROR:
        return "controller error";
    default:
        return "not terminated";
    }
}

static const char *calibration_offset_verification_name(uint8_t status)
{
    switch ((calibration_offset_verification_status_t)status) {
    case CAL_OFFSET_VERIFICATION_PASS: return "PASS";
    case CAL_OFFSET_VERIFICATION_MARGINAL: return "MARGINAL";
    case CAL_OFFSET_VERIFICATION_FAIL: return "FAIL";
    default: return "NOT RUN";
    }
}

static void calibration_offset_loop_print_summary(
    const calibration_offset_loop_state_t *state
)
{
    xil_printf("\r\n========== Offset Calibration Summary ==========\r\n");
    xil_printf("Iterations completed     : %lu\r\n",
               (unsigned long)state->batch_iteration_count);
    xil_printf("Accepted frames          : %lu\r\n",
               (unsigned long)state->accepted_frame_count);
    xil_printf("Rejected frames          : %lu\r\n",
               (unsigned long)state->rejected_frame_count);
    xil_printf("Calibration channel      : %s\r\n",
               calibration_channel_name(state->calibration_channel));
    print_float_value("Final applied correction",
                      state->offset_correction, " codes");
    print_float_value("Final batch residual",
                      state->latest_mean_residual, " codes");
    print_float_value("Final filtered residual",
                      state->filtered_residual, " codes");
    print_float_value("Final batch RMSE",
                      state->latest_batch_rmse, " codes");
    print_float_value("Best residual",
                      state->best_filtered_residual, " codes");
    print_float_value("Best raw residual",
                      state->best_raw_residual, " codes");
    print_float_value("Best correction",
                      state->best_offset_correction, " codes");
    print_float_value("Best RMSE", state->best_rmse, " codes");
    print_float_value("Residual standard deviation",
                      state->latest_residual_stddev, " codes");
    print_float_value("Residual standard error",
                      state->latest_standard_error, " codes");
    print_float_value("Controller gain",
                      state->latest_controller_gain, "");
    print_float_value("Filter alpha",
                      CALIBRATION_OFFSET_FILTER_ALPHA, "");
    print_float_value("Verification residual",
                      state->verification_residual, " codes");
    print_float_value("Verification correlation",
                      state->verification_correlation, "");
    print_float_value("Verification std dev",
                      state->verification_stddev, " codes");
    print_float_value("Verification standard error",
                      state->verification_standard_error, " codes");
    xil_printf("Verification frames     : %lu\r\n",
               (unsigned long)state->verification_accepted_frames);
    xil_printf("Verification status     : %s\r\n",
               calibration_offset_verification_name(
                   state->verification_status));
    xil_printf("Controller converged    : %s\r\n",
               state->controller_converged ? "YES" : "NO");
    xil_printf("Consecutive passes      : %lu/%u\r\n",
               (unsigned long)state->convergence_count,
               CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES);
    xil_printf("Termination reason       : %s\r\n",
               calibration_offset_termination_name(state->termination_reason));
    xil_printf("Formal calibration status: %s\r\n",
               state->restored_best_solution ? "BEST AVAILABLE SOLUTION" :
               calibration_offset_loop_status_name(state->final_status));
    xil_printf("Output usable           : %s\r\n",
               state->final_status == CALIBRATION_OFFSET_LOOP_PASS ||
               state->final_status == CALIBRATION_OFFSET_LOOP_BEST_AVAILABLE ?
               "YES" : "NO");
    xil_printf("===============================================\r\n");
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
    xil_printf("Batch iterations       : %lu\r\n",
               (unsigned long)state->batch_iteration_count);
    xil_printf("Calibration channel    : %s\r\n",
               calibration_channel_name(state->calibration_channel));
    print_float_value("Latest correlation",
                      state->latest_correlation, "");
    print_float_value("Latest mean aligned residual",
                      state->latest_mean_residual, " codes");
    print_float_value("Latest residual std dev",
                      state->latest_residual_stddev, " codes");
    print_float_value("Latest batch RMSE",
                      state->latest_batch_rmse, " codes");
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
    static calibration_frame_workspace_t frame_workspace;

    calibration_offset_loop_state_t *state =
        calibration_offset_loop_state();
    size_t reconstructed_count = 0U;
    double even_variance;
    double odd_variance;
    const char *pending_reason = NULL;

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

    if (!calibration_stored_reference_is_compatible(&pending_reason)) {
        xil_printf("No valid stored adc -cal reference frame.\r\n");
        if (pending_reason != NULL) {
            xil_printf("Reason: %s\r\n", pending_reason);
        }
        xil_printf("Run 'adc -cal' immediately before this command.\r\n");
        memset(&g_stored_offset_reference, 0,
               sizeof(g_stored_offset_reference));
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
    g_quiet_calibration_capture = true;

    xil_printf("\r\n========== ADC Offset Calibration ==========\r\n");
    xil_printf("Offset reference source : stored adc -cal frame\r\n");
    xil_printf("Reference channel       : %s\r\n",
               calibration_channel_name(g_stored_offset_reference.selected_channel));
    xil_printf("Reference phase         : %s\r\n",
               g_stored_offset_reference.selected_reference_phase == 0 ? "EVEN" : "ODD");
    xil_printf("Reference samples       : %lu\r\n",
               (unsigned long)g_stored_offset_reference.analysis_sample_count);
    print_float_value("Reference mean",
                      g_stored_offset_reference.metrics.adc_mean, " codes");
    print_float_value("Gain correction",
                      state->gain_correction, "");
    print_float_value("Initial offset correction",
                      state->offset_correction, " codes");
    print_float_value("Offset tolerance",
                      CALIBRATION_OFFSET_TOLERANCE_CODES, " codes");
    xil_printf("Batch size              : %u frames\r\n",
               CALIBRATION_OFFSET_BATCH_SIZE);
    xil_printf("Maximum batch iterations: %u\r\n",
               CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS);
    print_float_value("Far offset update gain",
                      CALIBRATION_OFFSET_UPDATE_STEP, "");
    print_float_value("Near offset update gain",
                      CALIBRATION_OFFSET_NEAR_UPDATE_STEP, "");
    print_float_value("Residual filter alpha",
                      CALIBRATION_OFFSET_FILTER_ALPHA, "");
    print_float_value("Standard error diagnostic guide",
                      CALIBRATION_OFFSET_MAX_STANDARD_ERROR_CODES, " codes");
    print_float_value("Enter tolerance",
                      CALIBRATION_OFFSET_ENTER_TOLERANCE_CODES, " codes");
    print_float_value("Exit tolerance",
                      CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES, " codes");
    print_float_value("Maximum stable update",
                      CALIBRATION_OFFSET_MAX_UPDATE_CODES, " codes");
    print_float_value("Verification limit",
                      CALIBRATION_OFFSET_VERIFICATION_TARGET_CODES, " codes");
    print_float_value("Verification hard limit",
                      CALIBRATION_OFFSET_VERIFICATION_HARD_LIMIT_CODES,
                      " codes");

    while ((state->batch_iteration_count <
            CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS) &&
           (state->convergence_count <
            CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES)) {
        calibration_offset_batch_result_t batch;
        float coefficient_delta = 0.0f;
        float next_offset_correction;
        float standard_error;
        float score;
        bool batch_pass;
        bool inside_convergence_region;
        bool correction_held;

        ++state->batch_iteration_count;
        xil_printf("\r\n========== Offset Batch %lu ==========\r\n",
                   (unsigned long)state->batch_iteration_count);
        if (calibration_compute_offset_batch(state->offset_correction,
                &frame_workspace, state, &batch) != 0) {
            state->convergence_count = 0U;
            xil_printf("Batch status            : REJECTED\r\n");
            xil_printf("Reason                  : no valid aligned frames\r\n");
            continue;
        }

        state->latest_mean_residual = batch.mean;
        state->latest_residual_stddev = batch.stddev;
        state->latest_residual_min = batch.minimum;
        state->latest_residual_max = batch.maximum;
        state->latest_batch_rmse = batch.mean_fitted_rmse;
        state->latest_correlation = batch.mean_correlation;
        state->latest_rmse = batch.mean_fitted_rmse;
        state->calibration_channel = g_stored_offset_reference.selected_channel;
        standard_error = batch.stddev / sqrtf((float)batch.accepted);
        state->latest_standard_error = standard_error;

        if (state->filtered_residual_valid == 0U) {
            state->filtered_residual = batch.mean;
            state->filtered_residual_valid = 1U;
        } else {
            state->filtered_residual =
                CALIBRATION_OFFSET_FILTER_ALPHA * state->filtered_residual +
                (1.0f - CALIBRATION_OFFSET_FILTER_ALPHA) * batch.mean;
        }

        xil_printf("\r\n---------- Batch Statistics ----------\r\n");
        print_float_value("Residual mean", batch.mean, " codes");
        print_float_value("Residual std dev", batch.stddev, " codes");
        print_float_value("Minimum residual", batch.minimum, " codes");
        print_float_value("Maximum residual", batch.maximum, " codes");
        print_float_value("Residual RMS", batch.rmse, " codes");
        print_float_value("Batch fitted RMSE",
                          batch.mean_fitted_rmse, " codes");
        print_float_value("Residual standard error",
                          standard_error, " codes");
        print_float_value("Filtered residual",
                          state->filtered_residual, " codes");
        xil_printf("Accepted frames         : %lu/%u\r\n",
                   (unsigned long)batch.accepted,
                   CALIBRATION_OFFSET_BATCH_SIZE);
        xil_printf("Rejected frames         : %lu/%u\r\n",
                   (unsigned long)batch.rejected,
                   CALIBRATION_OFFSET_BATCH_SIZE);

        score = fabsf(state->filtered_residual) +
                CALIBRATION_OFFSET_SCORE_RMSE_WEIGHT *
                    batch.mean_fitted_rmse +
                CALIBRATION_OFFSET_SCORE_STDERR_WEIGHT * standard_error;
        print_float_value("Batch solution score", score, "");
        if (score < state->best_score) {
            const float improvement = state->best_score < FLT_MAX ?
                state->best_score - score : FLT_MAX;
            state->best_score = score;
            state->best_abs_residual = fabsf(state->filtered_residual);
            state->best_residual = state->filtered_residual;
            state->best_filtered_residual = state->filtered_residual;
            state->best_raw_residual = batch.mean;
            state->best_offset_correction = state->offset_correction;
            state->best_rmse = batch.mean_fitted_rmse;
            if (improvement >= CALIBRATION_OFFSET_MIN_IMPROVEMENT_CODES)
                state->no_improvement_count = 0U;
            else
                ++state->no_improvement_count;
        } else {
            ++state->no_improvement_count;
        }
        state->latest_controller_gain =
            fabsf(state->filtered_residual) <
                CALIBRATION_OFFSET_NEAR_RESIDUAL_CODES ?
            CALIBRATION_OFFSET_NEAR_UPDATE_STEP :
            CALIBRATION_OFFSET_UPDATE_STEP;
        coefficient_delta =
            -state->latest_controller_gain * state->filtered_residual;
        inside_convergence_region =
            fabsf(state->filtered_residual) <=
                (state->convergence_count > 0U ?
                    CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES :
                    CALIBRATION_OFFSET_ENTER_TOLERANCE_CODES);
        batch_pass =
            batch.accepted >= CALIBRATION_OFFSET_MIN_ACCEPTED_FRAMES &&
            inside_convergence_region &&
            fabsf(coefficient_delta) <=
                CALIBRATION_OFFSET_MAX_UPDATE_CODES;
        correction_held = batch_pass;
        if (batch_pass) {
            ++state->convergence_count;
            coefficient_delta = 0.0f;
        } else {
            if (fabsf(state->filtered_residual) >
                    CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES)
                state->convergence_count = 0U;
        }

        if (fabsf(state->filtered_residual) <=
                CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES ||
            state->convergence_count > 0U) {
            state->no_improvement_count = 0U;
        }
        print_float_value("Proposed offset update",
                          -state->latest_controller_gain *
                              state->filtered_residual, " codes");
        xil_printf("Correction held for confirmation: %s\r\n",
                   correction_held ? "YES" : "NO");
        xil_printf("Inside convergence region: %s\r\n",
                   inside_convergence_region ? "YES" : "NO");
        xil_printf("No-improvement batches  : %lu/%u\r\n",
                   (unsigned long)state->no_improvement_count,
                   CALIBRATION_OFFSET_NO_IMPROVEMENT_LIMIT);
        if (state->convergence_count == 0U &&
            state->no_improvement_count >=
                CALIBRATION_OFFSET_NO_IMPROVEMENT_LIMIT) {
            state->termination_reason =
                CAL_OFFSET_TERMINATION_NO_IMPROVEMENT;
            break;
        }

        next_offset_correction =
            state->offset_correction + coefficient_delta;
        if (!isfinite(coefficient_delta) ||
            !isfinite(next_offset_correction)) {
            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
            state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
            xil_printf("Batch status            : FAILED\r\n");
            xil_printf("Reason                  : nonfinite offset update\r\n");
            break;
        }
        next_offset_correction = fmaxf(
            -CALIBRATION_OFFSET_MAX_ABS_CORRECTION_CODES,
            fminf(CALIBRATION_OFFSET_MAX_ABS_CORRECTION_CODES,
                  next_offset_correction));

        if (coefficient_delta != 0.0f) {
            if (calibration_set_software_offset_correction(
                    next_offset_correction) != 0) {
                state->convergence_count = 0U;
                state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                xil_printf("Batch status            : FAILED\r\n");
                xil_printf("Reason                  : offset coefficient rejected\r\n");
                break;
            }
            state->offset_correction = next_offset_correction;
        }

        print_float_value("Offset update",
                          coefficient_delta, " codes");
        print_float_value("Offset correction",
                          state->offset_correction, " codes");
        xil_printf("Consecutive passes      : %lu/%u\r\n",
                   (unsigned long)state->convergence_count,
                   CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES);
        xil_printf("Batch status            : %s\r\n",
                   batch_pass ? "PASS" : "ACCEPTED");
        if (state->convergence_count <
            CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES) {
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
        }
    }

    if (state->final_status == CALIBRATION_OFFSET_LOOP_RUNNING) {
        if (state->convergence_count >=
            CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES) {
            state->controller_converged = 1U;
            state->final_status =
                CALIBRATION_OFFSET_LOOP_CONTROLLER_CONVERGED;
            state->termination_reason = CAL_OFFSET_TERMINATION_CONVERGED;
        } else {
            state->final_status = CALIBRATION_OFFSET_LOOP_NOT_CONVERGED;
            if (state->termination_reason == CAL_OFFSET_TERMINATION_NONE)
                state->termination_reason =
                    CAL_OFFSET_TERMINATION_ITERATION_LIMIT;
        }
    }

    if (state->final_status ==
            CALIBRATION_OFFSET_LOOP_CONTROLLER_CONVERGED) {
        double residual_sum = 0.0, residual_square_sum = 0.0;
        double correlation_sum = 0.0, rmse_sum = 0.0;
        uint32_t total_frames = 0U;

        state->final_status = CALIBRATION_OFFSET_LOOP_VERIFYING;
        for (uint32_t batch_number = 1U;
             batch_number <= CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES;
             ++batch_number) {
            calibration_offset_batch_result_t verification;

            xil_printf("\r\n========== Offset Verification Batch %lu/%u ==========\r\n",
                       (unsigned long)batch_number,
                       CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES);
            ++state->verification_batch_count;
            if (calibration_compute_offset_batch(state->offset_correction,
                    &frame_workspace, state, &verification) == 0) {
                const double n = (double)verification.accepted;
                residual_sum += n * (double)verification.mean;
                residual_square_sum += n *
                    ((double)verification.stddev * verification.stddev +
                     (double)verification.mean * verification.mean);
                correlation_sum += n * verification.mean_correlation;
                rmse_sum += n * verification.mean_fitted_rmse;
                total_frames += verification.accepted;
            }
            if (total_frames > 0U) {
                const double n = (double)total_frames;
                const double mean = residual_sum / n;
                const double variance = fmax(0.0,
                    residual_square_sum / n - mean * mean);
                state->verification_residual = (float)mean;
                state->verification_stddev = (float)sqrt(variance);
                state->verification_standard_error =
                    state->verification_stddev / sqrtf((float)total_frames);
                state->verification_correlation =
                    (float)(correlation_sum / n);
                state->verification_rmse = (float)(rmse_sum / n);
                state->verification_accepted_frames = total_frames;
            }
            print_float_value("Combined verification mean",
                              state->verification_residual, " codes");
            xil_printf("Combined verification frames: %lu\r\n",
                       (unsigned long)total_frames);
            if (total_frames >= CALIBRATION_OFFSET_MIN_ACCEPTED_FRAMES &&
                state->verification_correlation >=
                    CAL_DAC_REF_MIN_CORRELATION &&
                fabsf(state->verification_residual) <=
                    CALIBRATION_OFFSET_VERIFICATION_TARGET_CODES) {
                break;
            }
            if (batch_number <
                    CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES) {
                xil_printf("Verification status     : RETRY REQUIRED\r\n");
            }
        }
        state->verification_valid =
            state->verification_accepted_frames >=
                CALIBRATION_OFFSET_MIN_ACCEPTED_FRAMES &&
            state->verification_correlation >= CAL_DAC_REF_MIN_CORRELATION;
        if (!state->verification_valid) {
            state->verification_status = CAL_OFFSET_VERIFICATION_FAIL;
            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
            state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
        } else if (fabsf(state->verification_residual) <=
                   CALIBRATION_OFFSET_VERIFICATION_TARGET_CODES) {
            state->verification_status = CAL_OFFSET_VERIFICATION_PASS;
            state->final_status = CALIBRATION_OFFSET_LOOP_PASS;
        } else if (fabsf(state->verification_residual) <=
                   CALIBRATION_OFFSET_VERIFICATION_HARD_LIMIT_CODES) {
            state->verification_status = CAL_OFFSET_VERIFICATION_MARGINAL;
            state->final_status = CALIBRATION_OFFSET_LOOP_BEST_AVAILABLE;
        } else {
            state->verification_status = CAL_OFFSET_VERIFICATION_FAIL;
            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
            state->termination_reason =
                CAL_OFFSET_TERMINATION_VERIFICATION_FAILED;
        }
        print_float_value("Verification residual",
                          state->verification_residual, " codes");
        print_float_value("Verification std dev",
                          state->verification_stddev, " codes");
        print_float_value("Verification standard error",
                          state->verification_standard_error, " codes");
        print_float_value("Verification correlation",
                          state->verification_correlation, "");
        xil_printf("Verification frames     : %lu\r\n",
                   (unsigned long)state->verification_accepted_frames);
        xil_printf("Verification status     : %s\r\n",
                   calibration_offset_verification_name(
                       state->verification_status));
    }

    if (state->final_status == CALIBRATION_OFFSET_LOOP_NOT_CONVERGED &&
        state->best_score < FLT_MAX &&
        isfinite(state->best_offset_correction)) {
        if (calibration_set_software_offset_correction(
                state->best_offset_correction) == 0) {
            state->offset_correction = state->best_offset_correction;
            state->restored_best_solution = 1U;
            state->final_status =
                CALIBRATION_OFFSET_LOOP_BEST_AVAILABLE;
        } else {
            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
            state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
        }
    }
    if (state->final_status == CALIBRATION_OFFSET_LOOP_NOT_CONVERGED) {
        state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
    }

    if (state->final_status == CALIBRATION_OFFSET_LOOP_PASS ||
        state->final_status == CALIBRATION_OFFSET_LOOP_BEST_AVAILABLE) {
        calibration_frame_config_t output_config;
        int output_status;
        const char *output_stage =
            state->final_status == CALIBRATION_OFFSET_LOOP_PASS ?
            "offset calibration (converged)" :
            "offset calibration (best available)";

        output_config.locked_channel = state->calibration_channel;
        output_config.adc_gain_correction = 1.0f;
        output_config.adc_offset_correction = state->offset_correction;
        output_config.reference_scale = 1.0f;
        output_config.reject_clipped_input = true;
        output_status = calibration_publish_final_capture(
            output_stage, even_reference, odd_reference,
            reconstructed_count, &output_config, &frame_workspace);
        if (output_status != 0) {
            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
            state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
            state->restored_best_solution = 0U;
        } else {
            g_pending_calibration_frame.source_offset_status =
                state->final_status == CALIBRATION_OFFSET_LOOP_PASS ? 2U : 1U;
        }
    } else {
        calibration_gain_input_frame_invalidate();
        xil_printf("\r\nCalibrated capture output: none "
                   "(offset calibration produced no usable solution).\r\n");
    }
    calibration_offset_loop_print_summary(state);
    g_quiet_calibration_capture = false;
    adc_sweep_active = 0U;
}

static void calibration_gain_loop_begin_run(
    calibration_gain_loop_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->gain_correction = calibration_software_gain_correction();
    state->fixed_offset_correction =
        calibration_software_offset_correction();
    state->calibration_channel = calibration_channel_selection();
    state->final_status = CALIBRATION_GAIN_LOOP_RUNNING;
    state->latest_fitted_gain = 0.0f;
    state->latest_gain_error = 0.0f;
    state->latest_fitted_offset = 0.0f;
    state->latest_correlation = 0.0f;
    state->latest_rmse = 0.0f;
    state->latest_waveform_rmse = 0.0f;
    state->latest_waveform_rmse_improvement = 0.0f;
    state->previous_waveform_rmse = 0.0f;
    state->have_previous_waveform_rmse = 0U;
    state->best_score = FLT_MAX;
    state->best_gain_correction = state->gain_correction;
}

static int calibration_measure_centered_gain_residual(
    const calibration_aligned_frame_t *frame,
    float fixed_offset_correction,
    float gain_correction,
    float *gain_residual,
    float *waveform_rmse)
{
    double offset_corrected_sum = 0.0;
    double reference_sum = 0.0;
    double projection_numerator = 0.0;
    double projection_denominator = 0.0;
    double error_square_sum = 0.0;
    double offset_corrected_mean;
    double reference_mean;
    const size_t sample_count = frame != NULL ?
        frame->valid_analysis_sample_count : 0U;

    if ((frame == NULL) || !frame->frame_valid ||
        (frame->aligned_raw_adc_samples == NULL) ||
        (frame->aligned_reference_samples == NULL) ||
        (gain_residual == NULL) || (waveform_rmse == NULL) ||
        (sample_count < CAL_MIN_ANALYSIS_SAMPLES) ||
        !isfinite(fixed_offset_correction) ||
        !isfinite(gain_correction) || (gain_correction <= 0.0f)) {
        return -1;
    }

    /* Firmware stores C = -O_final, so x_o = raw + C. */
    for (size_t i = 0U; i < sample_count; ++i) {
        offset_corrected_sum +=
            (double)frame->aligned_raw_adc_samples[i] +
            (double)fixed_offset_correction;
        reference_sum += (double)frame->aligned_reference_samples[i];
    }
    offset_corrected_mean = offset_corrected_sum / (double)sample_count;
    reference_mean = reference_sum / (double)sample_count;

    for (size_t i = 0U; i < sample_count; ++i) {
        const double centered_adc =
            ((double)frame->aligned_raw_adc_samples[i] +
             (double)fixed_offset_correction) - offset_corrected_mean;
        const double centered_reference =
            (double)frame->aligned_reference_samples[i] - reference_mean;
        const double error =
            (double)gain_correction * centered_adc - centered_reference;

        projection_numerator += centered_adc * error;
        projection_denominator += centered_adc * centered_adc;
        error_square_sum += error * error;
    }

    if (!isfinite(projection_numerator) ||
        !isfinite(projection_denominator) ||
        !isfinite(error_square_sum) ||
        (projection_denominator <= CAL_REF_VARIANCE_EPSILON)) {
        return -2;
    }

    *gain_residual = (float)(projection_numerator /
                             projection_denominator);
    *waveform_rmse = (float)sqrt(error_square_sum /
                                 (double)sample_count);
    return (isfinite(*gain_residual) && isfinite(*waveform_rmse)) ? 0 : -3;
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

static void calibration_gain_loop_print_summary(
    const calibration_gain_loop_state_t *state)
{
    xil_printf("\r\n========== Gain Calibration Summary ==========\r\n");
    xil_printf("Iterations completed     : %lu\r\n",
               (unsigned long)state->batch_iteration_count);
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
    print_float_value("Best gain correction",
                      state->best_gain_correction, "");
    print_float_value("Best gain error", state->best_gain_error, "");
    print_float_value("Final RMSE", state->latest_rmse, " codes");
    print_float_value("Best RMSE", state->best_rmse, " codes");
    print_float_value("Gain standard deviation",
                      state->latest_gain_stddev, "");
    print_float_value("Gain standard error",
                      state->latest_gain_standard_error, "");
    print_float_value("Controller gain",
                      state->latest_controller_gain, "");
    print_float_value("Filter alpha", CALIBRATION_GAIN_FILTER_ALPHA, "");
    xil_printf("Consecutive passes       : %lu\r\n",
               (unsigned long)state->convergence_count);
    xil_printf("Termination reason       : %s\r\n",
               calibration_offset_termination_name(state->termination_reason));
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
    print_float_value("Latest normalized gain residual",
                      state->latest_gain_error, "");
    print_float_value("Latest centered waveform RMSE",
                      state->latest_waveform_rmse, " codes");
    print_float_value("Latest waveform RMSE improvement",
                      state->latest_waveform_rmse_improvement, " codes");
    print_float_value("Latest fitted offset",
                      state->latest_fitted_offset, " codes");
    print_float_value("Latest correlation", state->latest_correlation, "");
    print_float_value("Latest fitted RMSE", state->latest_rmse, " codes");
    xil_printf("Calibration status     : %s\r\n",
               calibration_gain_loop_status_name(state->final_status));
    xil_printf("================================================\r\n");
}

static int calibration_publish_gain_output(
    const calibration_pending_frame_t *input,
    calibration_frame_workspace_t *workspace,
    float gain_correction,
    float offset_correction)
{
    calibration_pending_frame_t best;
    float best_error = FLT_MAX;
    uint32_t accepted = 0U;

    memset(&best, 0, sizeof(best));
    calibration_gain_input_frame_invalidate();
    xil_printf("\r\n========== ADC Calibrated Output ==========\r\n");
    xil_printf("Source stage            : gain calibration\r\n");
    for (uint32_t i = 1U; i <= CALIBRATION_GAIN_BATCH_SIZE; ++i) {
        calibration_aligned_frame_t frame;
        const char *reason = NULL;
        if (calibration_capture_against_owned_reference(
                input, true, gain_correction,
                gain_correction * offset_correction,
                CAL_ADC_FULL_SCALE_CODES / CAL_DAC_FULL_SCALE_CODES,
                workspace, &frame, &reason) == 0 && frame.frame_valid) {
            const float error = fabsf(frame.metrics.measured_gain - 1.0f);
            ++accepted;
            if (error < best_error &&
                calibration_pending_frame_copy(&best, &frame, i,
                    reference_buffer_generation(), reference_buffer_length(),
                    reference_buffer_format()) == 0) {
                best_error = error;
            }
        }
        if (i < CALIBRATION_GAIN_BATCH_SIZE)
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
    }
    print_float_value("Offset correction", offset_correction, " codes");
    print_float_value("Gain correction", gain_correction, "");
    xil_printf("Verification frames     : %lu/%u\r\n",
               (unsigned long)accepted, CALIBRATION_GAIN_BATCH_SIZE);
    if (!best.valid) {
        xil_printf("Capture status          : NOT READY\r\n");
        xil_printf("Output ready            : NO\r\n");
        xil_printf("===========================================\r\n");
        return -1;
    }
    g_pending_calibration_frame = best;
    g_pending_calibration_frame.valid = true;
    g_pending_calibration_frame.consumed = false;
    xil_printf("Capture status          : READY\r\n");
    print_float_value("Correlation", best.correlation, "");
    xil_printf("Samples                 : %lu\r\n",
               (unsigned long)best.analysis_sample_count);
    xil_printf("Output ready            : YES\r\n");
    xil_printf("===========================================\r\n");
    return 0;
}

static void handle_adc_gain_calibration_loop_cmd(void)
{
    static calibration_pending_frame_t gain_input;
    static calibration_frame_workspace_t workspace;
    calibration_gain_loop_state_t *state = calibration_gain_loop_state();
    const char *reason = NULL;

    if (adc_sweep_active || RxBufferPtr == NULL) {
        ERR("ADC capture is unavailable for gain calibration.");
        state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
        return;
    }
    if (!calibration_pending_frame_is_compatible(&reason)) {
        xil_printf("ERROR\r\n\r\nNo calibrated offset capture available.\r\n");
        xil_printf("Run\r\n\r\nadc -cal\r\n\r\nfollowed by\r\n\r\nadc -cal offset\r\n");
        return;
    }
    gain_input = g_pending_calibration_frame;
    calibration_gain_loop_begin_run(state);
    state->calibration_channel = gain_input.selected_channel;
    adc_sweep_active = 1U;
    g_quiet_calibration_capture = true;

    xil_printf("\r\n========== ADC Gain Calibration ==========\r\n");
    xil_printf("Gain input source       : offset-calibrated capture\r\n");
    xil_printf("Source status           : %s\r\n",
               gain_input.source_offset_status == 2U ?
               "CONVERGED" : "BEST AVAILABLE");
    xil_printf("Input frame             : Frame %lu\r\n",
               (unsigned long)gain_input.retained_frame_number);
    xil_printf("Channel                 : %s\r\n",
               calibration_channel_name(gain_input.selected_channel));
    xil_printf("Reference phase         : %s\r\n",
               gain_input.selected_reference_phase == 0 ? "EVEN" : "ODD");
    xil_printf("Samples                 : %lu\r\n",
               (unsigned long)gain_input.analysis_sample_count);

    while (state->batch_iteration_count <
               CALIBRATION_GAIN_MAX_ACCEPTED_ITERATIONS &&
           state->convergence_count <
               CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES) {
        calibration_gain_batch_result_t batch;
        float gain_update = 0.0f;
        float score;
        float next_gain;
        bool pass;

        ++state->batch_iteration_count;
        xil_printf("\r\n========== Gain Batch %lu ==========\r\n",
                   (unsigned long)state->batch_iteration_count);
        if (calibration_compute_gain_batch(&gain_input,
                state->gain_correction, state->fixed_offset_correction,
                &workspace, state, &batch) != 0) {
            state->convergence_count = 0U;
            ++state->no_improvement_count;
            continue;
        }
        state->latest_fitted_gain = batch.mean_gain;
        state->latest_gain_error = batch.mean_error;
        state->latest_fitted_offset = batch.mean_offset;
        state->latest_rmse = batch.mean_rmse;
        state->latest_correlation = batch.mean_correlation;
        state->latest_gain_stddev = batch.stddev;
        state->latest_gain_standard_error = batch.standard_error;
        if (state->filtered_gain_error_valid == 0U) {
            state->filtered_gain_error = batch.mean_error;
            state->filtered_gain_error_valid = 1U;
        } else {
            state->filtered_gain_error =
                CALIBRATION_GAIN_FILTER_ALPHA * state->filtered_gain_error +
                (1.0f - CALIBRATION_GAIN_FILTER_ALPHA) * batch.mean_error;
        }
        score = fabsf(batch.mean_error) + 0.1f * batch.mean_rmse +
                0.5f * batch.standard_error;
        if (score < state->best_score) {
            const float improvement = state->best_score < FLT_MAX ?
                state->best_score - score : FLT_MAX;
            state->best_score = score;
            state->best_gain_correction = state->gain_correction;
            state->best_gain_error = batch.mean_error;
            state->best_rmse = batch.mean_rmse;
            state->no_improvement_count =
                improvement >= CALIBRATION_GAIN_MIN_IMPROVEMENT ? 0U :
                state->no_improvement_count + 1U;
        } else {
            ++state->no_improvement_count;
        }
        pass = fabsf(state->filtered_gain_error) <
                   CALIBRATION_GAIN_TOLERANCE &&
               batch.standard_error <
                   CALIBRATION_GAIN_MAX_STANDARD_ERROR;
        state->latest_controller_gain =
            fabsf(state->filtered_gain_error) < CALIBRATION_GAIN_NEAR_ERROR ?
            CALIBRATION_GAIN_NEAR_UPDATE_STEP :
            CALIBRATION_GAIN_UPDATE_STEP;
        if (pass) {
            ++state->convergence_count;
        } else {
            state->convergence_count = 0U;
            gain_update = -state->latest_controller_gain *
                          state->filtered_gain_error;
        }
        xil_printf("\r\n---------- Gain Batch Statistics ----------\r\n");
        print_float_value("Batch gain", batch.mean_gain, "");
        print_float_value("Gain error", batch.mean_error, "");
        print_float_value("Filtered gain error",
                          state->filtered_gain_error, "");
        print_float_value("Gain std dev", batch.stddev, "");
        print_float_value("Gain standard error", batch.standard_error, "");
        print_float_value("Regression offset", batch.mean_offset, " codes");
        print_float_value("Regression RMSE", batch.mean_rmse, " codes");
        print_float_value("Correlation", batch.mean_correlation, "");
        xil_printf("Accepted frames         : %lu/%u\r\n",
                   (unsigned long)batch.accepted, CALIBRATION_GAIN_BATCH_SIZE);
        xil_printf("Rejected frames         : %lu/%u\r\n",
                   (unsigned long)batch.rejected, CALIBRATION_GAIN_BATCH_SIZE);
        if (!pass && state->no_improvement_count >=
                CALIBRATION_GAIN_NO_IMPROVEMENT_LIMIT) {
            state->termination_reason =
                CAL_OFFSET_TERMINATION_NO_IMPROVEMENT;
            break;
        }
        next_gain = fmaxf(CALIBRATION_GAIN_CORRECTION_MIN,
            fminf(CALIBRATION_GAIN_CORRECTION_MAX,
                  state->gain_correction + gain_update));
        if (!isfinite(next_gain) ||
            (gain_update != 0.0f &&
             calibration_set_software_gain_correction(next_gain) != 0)) {
            state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
            state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
            break;
        }
        if (gain_update != 0.0f) state->gain_correction = next_gain;
        print_float_value("Gain update", gain_update, "");
        print_float_value("Gain correction", state->gain_correction, "");
        xil_printf("Consecutive passes      : %lu\r\n",
                   (unsigned long)state->convergence_count);
        xil_printf("Batch status            : %s\r\n",
                   pass ? "PASS" : "ACCEPTED");
    }

    if (state->final_status == CALIBRATION_GAIN_LOOP_RUNNING) {
        if (state->convergence_count >=
            CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES) {
            state->final_status = CALIBRATION_GAIN_LOOP_PASS;
            state->termination_reason = CAL_OFFSET_TERMINATION_CONVERGED;
        } else if (state->best_score < FLT_MAX &&
                   calibration_set_software_gain_correction(
                       state->best_gain_correction) == 0) {
            state->gain_correction = state->best_gain_correction;
            state->restored_best_solution = 1U;
            state->final_status = CALIBRATION_GAIN_LOOP_BEST_AVAILABLE;
            if (state->termination_reason == CAL_OFFSET_TERMINATION_NONE)
                state->termination_reason =
                    CAL_OFFSET_TERMINATION_ITERATION_LIMIT;
        } else {
            state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
            state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
        }
    }
    if (state->final_status == CALIBRATION_GAIN_LOOP_PASS ||
        state->final_status == CALIBRATION_GAIN_LOOP_BEST_AVAILABLE) {
        if (calibration_publish_gain_output(&gain_input, &workspace,
                state->gain_correction, state->fixed_offset_correction) != 0) {
            state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
            state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
        }
    } else {
        calibration_gain_input_frame_invalidate();
    }
    calibration_gain_loop_print_summary(state);
    g_quiet_calibration_capture = false;
    adc_sweep_active = 0U;
}

#if 0 /* Superseded single-frame/uploaded-reference gain controller. */
static void handle_adc_gain_calibration_loop_cmd_legacy(void)
{
    static int16_t even_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t odd_reference[ADC_VALID_SAMPLE_COUNT];
    static calibration_frame_workspace_t frame_workspace;
    calibration_gain_loop_state_t *state = calibration_gain_loop_state();
    size_t reconstructed_count = 0U;
    double even_variance, odd_variance;
    const char *pending_reason = NULL;
    bool use_pending_frame = true;

    if (adc_sweep_active || RxBufferPtr == NULL) {
        ERR("ADC capture is unavailable for gain calibration.");
        state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
        return;
    }
    calibration_gain_loop_begin_run(state);
    if (state->calibration_channel < -1 ||
        state->calibration_channel > 1)
    {
        ERR("Invalid shared calibration channel selection.");
        state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
        calibration_gain_loop_print_summary(state);
        return;
    }
    if (!calibration_pending_frame_is_compatible(&pending_reason)) {
        xil_printf("No valid pending aligned frame.\r\n");
        if (pending_reason != NULL) {
            xil_printf("Reason: %s\r\n", pending_reason);
        }
        xil_printf("Run 'adc -cal offset' successfully before this command.\r\n");
        calibration_gain_input_frame_invalidate();
        state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
        calibration_gain_loop_print_summary(state);
        return;
    }
    print_adc_analysis_rate_header();
    if (calibration_prepare_uploaded_dac_reference(even_reference,
            odd_reference, &reconstructed_count, &even_variance,
            &odd_variance, 1) != 0) {
        state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
        calibration_gain_loop_print_summary(state);
        return;
    }
    adc_sweep_active = 1U;
    xil_printf("\r\n========== ADC Gain Calibration ==========\r\n");
    xil_printf("Calibration channel     : %s%s\r\n",
               calibration_channel_name(state->calibration_channel),
               state->calibration_channel < 0 ?
               " (lock after first accepted frame)" : " (locked)");
    print_float_value("Fixed offset correction",
                      state->fixed_offset_correction, " codes");
    print_float_value("Initial gain correction",
                      state->gain_correction, "");
    print_float_value("Gain update tolerance",
                      CALIBRATION_GAIN_TOLERANCE, "");
    print_float_value("Gain update step", CALIBRATION_GAIN_UPDATE_STEP, "");
    print_float_value("Waveform stop abs. tolerance",
                      CALIBRATION_GAIN_RMSE_STOP_ABS_CODES, " codes");
    print_float_value("Waveform stop relative tolerance",
                      CALIBRATION_GAIN_RMSE_STOP_RELATIVE, "");
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
        calibration_frame_config_t frame_config;
        calibration_aligned_frame_t aligned_frame;
        float gain_delta;
        float gain_residual;
        float waveform_rmse;
        float waveform_rmse_improvement = 0.0f;
        float waveform_stop_tolerance = 0.0f;
        bool waveform_stopped_improving = false;
        float next_gain;
        uint32_t batch_accepted_frames = 0U;
        const char *frame_reason = NULL;
        const bool using_pending = use_pending_frame;
        int status;

        xil_printf("\r\nIteration %lu\r\n",
            (unsigned long)(state->accepted_frame_count +
                            state->rejected_frame_count + 1U));
        if (use_pending_frame) {
            use_pending_frame = false;
            status = calibration_pending_frame_consume(
                state->gain_correction,
                state->gain_correction * state->fixed_offset_correction,
                CAL_ADC_FULL_SCALE_CODES / CAL_DAC_FULL_SCALE_CODES,
                &frame_workspace, &aligned_frame, &pending_reason
            );
            if (status != 0) {
                calibration_gain_loop_reject_frame(state, pending_reason);
                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                break;
            }
            xil_printf("Input source            : pending adc -cal frame %lu\r\n",
                       (unsigned long)aligned_frame.retained_frame_number);
        } else {
            frame_config.locked_channel = state->calibration_channel;
            frame_config.adc_gain_correction = state->gain_correction;
            frame_config.adc_offset_correction =
                state->gain_correction * state->fixed_offset_correction;
            frame_config.reference_scale =
                CAL_ADC_FULL_SCALE_CODES / CAL_DAC_FULL_SCALE_CODES;
            frame_config.reject_clipped_input = true;
            status = calibration_capture_representative_batch(
                even_reference, odd_reference, reconstructed_count,
                &frame_config, &frame_workspace, &aligned_frame,
                &batch_accepted_frames, &frame_reason
            );
            xil_printf("Input source            : fresh representative batch\r\n");
            xil_printf("Batch accepted frames   : %lu/%u\r\n",
                       (unsigned long)batch_accepted_frames,
                       CAL_UPDATE_FRAME_BATCH_SIZE);
        }
        if ((status != 0) || !aligned_frame.frame_valid) {
            calibration_gain_loop_reject_frame(state,
                frame_reason != NULL ?
                frame_reason : aligned_frame.rejection_reason);
            continue;
        }
        if (!using_pending) {
            xil_printf("Selected batch frame    : %lu\r\n",
                       (unsigned long)aligned_frame.retained_frame_number);
        }
        xil_printf("Channel                 : %s\r\n",
                   aligned_frame.selected_channel_name);
        xil_printf("Reference phase         : %s\r\n",
                   aligned_frame.selected_phase_name);

        {
            const float fitted_gain =
            aligned_frame.metrics.measured_gain;
            const float fitted_offset =
            aligned_frame.metrics.measured_offset;
            const float correlation = aligned_frame.correlation;
            const float fitted_rmse =
            aligned_frame.metrics.fitted_rmse_codes;
        if (!isfinite(fitted_gain) ||
                !isfinite(fitted_offset) || !isfinite(correlation) ||
                !isfinite(fitted_rmse)) {
            calibration_gain_loop_reject_frame(
                state, "invalid aligned regression result");
            state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
            break;
        }

            state->latest_fitted_gain = fitted_gain;
            state->latest_fitted_offset = fitted_offset;
            state->latest_correlation = correlation;
            state->latest_rmse = fitted_rmse;
        }
        if (state->calibration_channel < 0)
        {
            const int8_t selected_channel = aligned_frame.selected_channel;
            if (calibration_set_channel_selection(selected_channel) != 0)
            {
                calibration_gain_loop_reject_frame(
                    state, "invalid calibration channel selection");
                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                break;
            }
            state->calibration_channel = selected_channel;
        }

        if (calibration_measure_centered_gain_residual(
                &aligned_frame, state->fixed_offset_correction,
                state->gain_correction, &gain_residual,
                &waveform_rmse) != 0) {
            calibration_gain_loop_reject_frame(
                state, "centered gain-residual calculation failed");
            continue;
        }

        gain_delta = -CALIBRATION_GAIN_UPDATE_STEP * gain_residual;
        next_gain = state->gain_correction + gain_delta;
        state->latest_gain_error = gain_residual;
        state->latest_waveform_rmse = waveform_rmse;

        if (state->have_previous_waveform_rmse != 0U) {
            waveform_rmse_improvement =
                state->previous_waveform_rmse - waveform_rmse;
            waveform_stop_tolerance = fmaxf(
                CALIBRATION_GAIN_RMSE_STOP_ABS_CODES,
                fabsf(state->previous_waveform_rmse) *
                    CALIBRATION_GAIN_RMSE_STOP_RELATIVE);
            waveform_stopped_improving =
                fabsf(waveform_rmse_improvement) <= waveform_stop_tolerance;
        }
        state->latest_waveform_rmse_improvement =
            waveform_rmse_improvement;

        if ((fabsf(gain_delta) < CALIBRATION_GAIN_TOLERANCE) &&
            waveform_stopped_improving) {
            ++state->convergence_count;
        } else {
            state->convergence_count = 0U;
        }

        print_float_value("Correlation", state->latest_correlation, "");
        print_float_value("Fitted gain (regression)",
                          state->latest_fitted_gain, "");
        print_float_value("Normalized gain residual",
                          state->latest_gain_error, "");
        print_float_value("Gain coefficient update", gain_delta, "");
        print_float_value("Centered waveform RMSE", waveform_rmse, " codes");
        if (state->have_previous_waveform_rmse != 0U) {
            print_float_value("Waveform RMSE improvement",
                              waveform_rmse_improvement, " codes");
            print_float_value("Waveform stop tolerance",
                              waveform_stop_tolerance, " codes");
            xil_printf("Waveform stopped improving: %s\r\n",
                       waveform_stopped_improving ? "YES" : "NO");
        } else {
            xil_printf("Waveform stopped improving: not yet evaluated\r\n");
        }
        print_float_value("Fixed offset correction",
                          state->fixed_offset_correction, " codes");
        print_float_value("Fitted offset (regression)",
                          state->latest_fitted_offset, " codes");
        print_float_value("Fitted RMSE", state->latest_rmse, " codes");

        if (!isfinite(gain_delta) || !isfinite(next_gain) ||
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
        state->previous_waveform_rmse = waveform_rmse;
        state->have_previous_waveform_rmse = 1U;
        ++state->accepted_frame_count;
        print_float_value("New gain correction", state->gain_correction, "");
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
    if (state->final_status == CALIBRATION_GAIN_LOOP_PASS) {
        calibration_frame_config_t output_config;

        output_config.locked_channel = state->calibration_channel;
        output_config.adc_gain_correction = state->gain_correction;
        output_config.adc_offset_correction =
            state->gain_correction * state->fixed_offset_correction;
        output_config.reference_scale = 1.0f;
        output_config.reject_clipped_input = true;
        (void)calibration_publish_final_capture(
            "gain calibration", even_reference, odd_reference,
            reconstructed_count, &output_config, &frame_workspace);
    } else {
        calibration_gain_input_frame_invalidate();
        xil_printf("\r\nCalibrated capture output: none "
                   "(gain calibration did not pass).\r\n");
    }
    adc_sweep_active = 0U;
}

#endif

void handle_adc_calibration_cmd(uint32_t frame_count)
{
    static int16_t even_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t odd_reference[ADC_VALID_SAMPLE_COUNT];
    static calibration_frame_workspace_t frame_workspace;
    static float correlations[ADC_CAL_MAX_FRAMES];
    static float integer_lags[ADC_CAL_MAX_FRAMES];
    static float fractional_lags[ADC_CAL_MAX_FRAMES];
    static calibration_pending_frame_t
        accepted_candidates[ADC_CAL_MAX_FRAMES];

    size_t reconstructed_count = 0U;
    uint32_t accepted_frames = 0U;
    size_t candidate_count = 0U;
    size_t selected_candidate = 0U;
    int calibration_channel = calibration_channel_selection();
    int alignment_pass = 0;
    int representative_selected = 0;
    double sum_correlation = 0.0;
    double even_variance;
    double odd_variance;
    uint32_t reference_generation;
    size_t uploaded_reference_length;
    reference_buffer_format_t uploaded_reference_format;
    calibration_selection_medians_t selection_medians;

    calibration_pending_frame_invalidate();

    if ((frame_count < ADC_CAL_MIN_FRAMES) ||
        (frame_count > ADC_CAL_MAX_FRAMES)) {
        ERR("Calibration frame count must be between %u and %u.",
            ADC_CAL_MIN_FRAMES, ADC_CAL_MAX_FRAMES);
        return;
    }

    print_adc_analysis_rate_header();
    if (calibration_prepare_uploaded_dac_reference(
            even_reference, odd_reference, &reconstructed_count,
            &even_variance, &odd_variance, 1) != 0) return;
    reference_generation = reference_buffer_generation();
    uploaded_reference_length = reference_buffer_length();
    uploaded_reference_format = reference_buffer_format();

    if (adc_sweep_active)
    {
        ERR("Another automatic ADC capture is already in progress.");
        return;
    }

    adc_sweep_active = 1;
    g_quiet_calibration_capture = true;

    xil_printf("\r\n");
    xil_printf("ADC Timing Alignment Measurement\r\n");
    xil_printf("Reference source : uploaded DAC TXT\r\n");
    xil_printf("Requested frames : %lu\r\n", (unsigned long)frame_count);

    for (uint32_t frame = 1U; frame <= frame_count; ++frame)
    {
        calibration_frame_config_t frame_config;
        calibration_aligned_frame_t aligned_frame;
        int fit_status;

        xil_printf("\r\n---------- Frame %lu ----------\r\n",
                   (unsigned long)frame);

        frame_config.locked_channel = calibration_channel;
        frame_config.adc_gain_correction =
            calibration_software_gain_correction();
        frame_config.adc_offset_correction =
            calibration_software_offset_correction();
        frame_config.reference_scale = 1.0f;
        frame_config.reject_clipped_input = false;
        fit_status = calibration_capture_and_align(
            even_reference, odd_reference, reconstructed_count,
            &frame_config, &frame_workspace, &aligned_frame
        );

        if ((fit_status != 0) || !aligned_frame.frame_valid) {
            xil_printf("Status           : REJECTED\r\n");
            xil_printf("Reason           : %s\r\n",
                       aligned_frame.rejection_reason);
            continue;
        }
        xil_printf("Channel          : %s\r\n",
                   aligned_frame.selected_channel_name);
        xil_printf("Reference phase  : %s\r\n",
                   aligned_frame.selected_phase_name);
        print_float_value("Correlation", aligned_frame.correlation, "");
        xil_printf("Integer lag      : %ld samples\r\n",
                   (long)aligned_frame.integer_lag);
        print_float_value("Fractional lag",
                          aligned_frame.fractional_lag, " samples");
        xil_printf("Status           : ACCEPTED\r\n");
        correlations[accepted_frames] = aligned_frame.correlation;
        integer_lags[accepted_frames] = (float)aligned_frame.integer_lag;
        fractional_lags[accepted_frames] = aligned_frame.fractional_lag;
        sum_correlation += (double)aligned_frame.correlation;
        if (calibration_channel < 0)
            calibration_channel = aligned_frame.selected_channel;
        if (calibration_pending_frame_copy(
                &accepted_candidates[candidate_count],
                &aligned_frame, frame, reference_generation,
                uploaded_reference_length,
                uploaded_reference_format) != 0) {
            memset(&accepted_candidates[candidate_count], 0,
                   sizeof(accepted_candidates[candidate_count]));
        } else {
            ++candidate_count;
        }
        ++accepted_frames;

        if (frame < frame_count) usleep(ADC_TIMING_INTERFRAME_DELAY_US);
    }

    xil_printf("\r\n========== Timing Alignment Summary ==========\r\n");
    xil_printf("Requested frames    : %lu\r\n", (unsigned long)frame_count);
    xil_printf("Accepted frames     : %lu\r\n", (unsigned long)accepted_frames);
    xil_printf("Rejected frames     : %lu\r\n",
               (unsigned long)(frame_count - accepted_frames));
    xil_printf("Calibration channel : %s\r\n",
               calibration_channel == 0 ? "Channel A" :
               (calibration_channel == 1 ? "Channel B" : "none"));
    if (accepted_frames > 0U) {
        float min_corr = correlations[0];
        for (uint32_t i = 1U; i < accepted_frames; ++i)
            if (correlations[i] < min_corr) min_corr = correlations[i];
        xil_printf("\r\n");
        print_double_value("Mean correlation",
                           sum_correlation / (double)accepted_frames, "");
        print_float_value("Minimum correlation", min_corr, "");
        print_float_value("Median lag",
                          median_float(integer_lags, accepted_frames),
                          " samples");
        print_float_value("Median frac. lag",
                          median_float(fractional_lags, accepted_frames),
                          " samples");
    }
    {
        const float acceptance_rate =
            (float)accepted_frames / (float)frame_count;
        const uint32_t required =
            frame_count < CAL_TIMING_MIN_ACCEPTED_FRAMES ?
            frame_count : CAL_TIMING_MIN_ACCEPTED_FRAMES;
        alignment_pass =
            (accepted_frames >= required) &&
            (acceptance_rate >= CAL_TIMING_MIN_ACCEPTANCE_RATE);
        if (alignment_pass && (candidate_count >= required) &&
            (candidate_count == accepted_frames) &&
            (calibration_select_representative_frame(
                accepted_candidates, candidate_count,
                &selected_candidate, &selection_medians) == 0)) {
            g_stored_offset_reference =
                accepted_candidates[selected_candidate];
            g_stored_offset_reference.valid = true;
            g_stored_offset_reference.consumed = false;
            representative_selected = 1;
        } else {
            calibration_pending_frame_invalidate();
        }
        xil_printf("\r\nAlignment status    : %s\r\n",
                   alignment_pass ? "PASS" : "FAIL");
    }
    if (representative_selected && g_stored_offset_reference.valid) {
        xil_printf("Stored offset reference: Frame %lu\r\n",
                   (unsigned long)
                       g_stored_offset_reference.retained_frame_number);
        xil_printf("Selection reason    : Closest to median calibration metrics\r\n");
    } else {
        xil_printf("Stored offset reference: none\r\n");
    }
    xil_printf("==============================================\r\n");
    g_quiet_calibration_capture = false;
    adc_sweep_active = 0;
}
