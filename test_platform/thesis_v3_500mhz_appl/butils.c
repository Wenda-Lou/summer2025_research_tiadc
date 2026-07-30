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
#define CAL_FIXED_WINDOW_LENGTH               CAL_TIMING_MIN_ANALYSIS_SAMPLES
#define CAL_FIXED_LAG_SEARCH_MARGIN            32
#define CAL_FIXED_LAG_CORRELATION_TIE_EPSILON  0.001f
#define CAL_OFFSET_STABILITY_DEFAULT_FRAMES    100U
#define CAL_OFFSET_STABILITY_MAX_FRAMES        100U
#define CAL_OFFSET_STABILITY_BIAS_CORRELATION  0.30f
#ifndef ADC_CAL_DEBUG
#define ADC_CAL_DEBUG                           0
#endif
#ifndef CALIBRATION_ALIGNMENT_DEBUG
#define CALIBRATION_ALIGNMENT_DEBUG            0
#endif
#define ADC_CAL_VERBOSE_DEBUG \
    ((ADC_CAL_DEBUG) || (CALIBRATION_ALIGNMENT_DEBUG))
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
#define ADC_PERFORMANCE_FRAMES                    30U
#define ADC_PERFORMANCE_MIN_VALID_FRAMES          20U
#define ADC_PERFORMANCE_FUNDAMENTAL_SEARCH_BINS   3U
#define ADC_PERFORMANCE_HANN_SIGNAL_HALF_WIDTH    2U
#define CAL_TONE_REFINE_HALF_RANGE_BINS            0.75
#define CAL_TONE_REFINE_MIN_STEP                  1.0e-12
#define CAL_TONE_REFINE_MAX_ITERATIONS              40U
#define CAL_DITHER_PEAK_GUARD_SAMPLES                8U
#define CAL_DITHER_EVENT_THRESHOLD_FRACTION         0.25
#define CAL_DITHER_CONSISTENCY_TOLERANCE_SAMPLES    1.0
#ifndef CAL_TONE_VALIDATION_MAX_RMSE_CODES
#define CAL_TONE_VALIDATION_MAX_RMSE_CODES        512.0
#endif
#ifndef CAL_TONE_VALIDATION_MIN_CORRELATION
#define CAL_TONE_VALIDATION_MIN_CORRELATION       CAL_DAC_REF_MIN_CORRELATION
#endif
#ifndef CAL_DITHER_VALIDATION_MIN_MARGIN
#define CAL_DITHER_VALIDATION_MIN_MARGIN            6.0
#endif
#ifndef CAL_DITHER_VALIDATION_MIN_PEAK_RATIO
#define CAL_DITHER_VALIDATION_MIN_PEAK_RATIO        1.10
#endif
#ifndef CAL_DITHER_VALIDATION_MIN_COMPLETE_EVENTS
#define CAL_DITHER_VALIDATION_MIN_COMPLETE_EVENTS   1U
#endif
#ifndef CAL_DITHER_CHANNEL_N0_TOLERANCE_SAMPLES
#define CAL_DITHER_CHANNEL_N0_TOLERANCE_SAMPLES     1.0
#endif
#ifndef CAL_EXISTING_DITHER_LAG_TOLERANCE_SAMPLES
#define CAL_EXISTING_DITHER_LAG_TOLERANCE_SAMPLES   CAL_DITHER_CONSISTENCY_TOLERANCE_SAMPLES
#endif
#ifndef CAL_DITHER_OFFSET_MIN_COMPLETE_EVENTS
#define CAL_DITHER_OFFSET_MIN_COMPLETE_EVENTS        2U
#endif
#ifndef CAL_DITHER_OFFSET_DENOMINATOR_FLOOR
#define CAL_DITHER_OFFSET_DENOMINATOR_FLOOR          0.05
#endif
#ifndef CAL_DITHER_OFFSET_FLAT_TOP_DERIVATIVE_FRACTION
#define CAL_DITHER_OFFSET_FLAT_TOP_DERIVATIVE_FRACTION 0.15
#endif
#ifndef CAL_DITHER_OFFSET_FLAT_TOP_AMPLITUDE_FRACTION
#define CAL_DITHER_OFFSET_FLAT_TOP_AMPLITUDE_FRACTION 0.70
#endif
#ifndef CAL_DITHER_OFFSET_MIN_FLAT_TOP_SAMPLES
#define CAL_DITHER_OFFSET_MIN_FLAT_TOP_SAMPLES       1U
#endif
#ifndef CAL_DITHER_OFFSET_AGREEMENT_TOLERANCE_CODES
#define CAL_DITHER_OFFSET_AGREEMENT_TOLERANCE_CODES  5.0
#endif
#ifndef CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES
#define CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES  128U
#endif

_Static_assert(ADC_VALID_SAMPLE_COUNT == ADC_TEST_CAPTURE_SAMPLES,
               "ADC test configuration sample count mismatch");

typedef enum {
    CAL_TIMING_VALIDATION_FAIL = 0,
    CAL_TIMING_VALIDATION_PASS,
    CAL_TIMING_VALIDATION_WARNING
} calibration_timing_validation_status_t;

typedef struct {
    uint8_t valid;
    calibration_timing_validation_status_t existing_status;
    calibration_timing_validation_status_t tone_status;
    calibration_timing_validation_status_t dither_status;
    calibration_timing_validation_status_t channel_status;
    calibration_timing_validation_status_t existing_dither_status;
    calibration_timing_validation_status_t window_status;
    calibration_timing_validation_status_t numerical_status;
    calibration_timing_validation_status_t overall_status;
    double channel_a_n0;
    double channel_b_n0;
    double common_selected_n0;
    double channel_n0_disagreement_samples;
    uint32_t window_partial_event_count;
    uint32_t total_dither_event_count;
    uint8_t dither_event_indices_valid;
    const char *numerical_reason;
} calibration_timing_validation_t;

typedef struct {
    uint8_t valid;
    double n0_integer;
    double n0_fractional;
    double sign;
    double peak;
    double second_peak;
    double peak_ratio;
    double align_margin;
    double derived_lag;
} calibration_dither_channel_alignment_t;

typedef struct {
    uint8_t valid;
    double fitted_frequency_hz;
    double initial_frequency_hz;
    double expected_frequency_hz;
    double cosine_coefficient;
    double sine_coefficient;
    double amplitude;
    double phase_rad;
    double dc_offset_codes;
    double rmse;
    double tone_only_correlation;
    double frequency_error_hz;
} calibration_tone_fit_result_t;

typedef enum {
    CAL_DITHER_OFFSET_STATUS_INVALID = 0,
    CAL_DITHER_OFFSET_STATUS_PASS,
    CAL_DITHER_OFFSET_STATUS_WARNING
} calibration_dither_offset_status_t;

typedef enum {
    CAL_DITHER_OFFSET_REASON_NONE = 0,
    CAL_DITHER_OFFSET_REASON_TIMING_CONTEXT,
    CAL_DITHER_OFFSET_REASON_TONE_FIT,
    CAL_DITHER_OFFSET_REASON_TOO_FEW_EVENTS,
    CAL_DITHER_OFFSET_REASON_POLARITY_IMBALANCE,
    CAL_DITHER_OFFSET_REASON_FLAT_TOP,
    CAL_DITHER_OFFSET_REASON_INTERPOLATION,
    CAL_DITHER_OFFSET_REASON_NUMERICAL,
    CAL_DITHER_OFFSET_REASON_NO_DITHER,
    CAL_DITHER_OFFSET_REASON_EVENT_PROFILE
} calibration_dither_offset_reason_t;

typedef struct {
    uint8_t valid;
    calibration_dither_offset_status_t status;
    calibration_dither_offset_reason_t reason;
    uint8_t timing_context_pass;
    uint8_t tone_fit_pass;
    uint8_t event_count_pass;
    uint8_t polarity_balance_pass;
    uint8_t flat_top_pass;
    uint8_t numerical_pass;
    uint8_t estimate_consistency_pass;
    calibration_tone_fit_result_t tone;
    double existing_offset_codes;
    double dither_offset_codes;
    double fitted_tone_dc_codes;
    double existing_vs_dither_codes;
    double dither_vs_fitted_dc_codes;
    double offset_profile_std_codes;
    double offset_profile_min_codes;
    double offset_profile_max_codes;
    uint32_t complete_event_count;
    uint32_t discarded_boundary_event_count;
    double mean_event_polarity;
    double separation_denominator;
    uint32_t flat_top_sample_count;
    double fitted_tone_frequency_hz;
    double fitted_tone_amplitude_codes;
    double fitted_tone_phase_rad;
    double tone_fit_rmse_codes;
    double tone_only_correlation;
    const char *units;
} calibration_dither_offset_diagnostic_t;

typedef struct {
    uint8_t valid;
    calibration_tone_fit_result_t tone;
    calibration_tone_fit_result_t initial_tone;
    calibration_dither_channel_alignment_t channel[2];
    int8_t selected_common_channel;
    int32_t dither_n0_integer;
    double dither_n0_fractional;
    double dither_sign;
    double dither_peak;
    double dither_second_peak;
    double dither_peak_ratio;
    double dither_align_margin;
    double dither_event_spacing_samples;
    uint32_t complete_dither_event_count;
    double dither_derived_lag;
    double alignment_disagreement_samples;
    uint8_t alignment_methods_consistent;
    uint32_t partial_dither_event_count;
    uint32_t total_dither_event_count;
    uint8_t dither_event_indices_valid;
    calibration_timing_validation_t validation;
    const char *status_text;
} calibration_timing_diagnostics_t;

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
    int32_t raw_candidate_lag;
    float raw_candidate_correlation;
    uint8_t raw_candidate_coverage_valid;
    int32_t valid_lag_min;
    int32_t valid_lag_max;
    int32_t local_lag_min;
    int32_t local_lag_max;
    int32_t expected_lag;
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
    int8_t canonical_reference_phase;
    const char *selected_channel_name;
    const char *selected_phase_name;
    int32_t integer_lag;
    float fractional_lag;
    float total_lag;
    const int16_t *aligned_reference_samples;
    const int16_t *aligned_raw_adc_samples;
    const int16_t *aligned_corrected_adc_samples;
    const int16_t *alignment_reference_samples;
    const int16_t *canonical_reference_window;
    size_t alignment_reference_count;
    size_t calibration_window_start;
    size_t calibration_window_length;
    uint32_t canonical_reference_checksum;
    float canonical_reference_mean;
    float analysis_reference_scale;
    float canonical_nominal_system_gain;
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
    calibration_timing_diagnostics_t timing_diagnostics;
    const char *rejection_reason;
} calibration_aligned_frame_t;

typedef struct {
    bool valid;
    bool consumed;
    uint32_t capture_sequence;
    uint32_t retained_frame_number;
    int8_t selected_channel;
    int8_t selected_reference_phase;
    int8_t canonical_reference_phase;
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
    calibration_timing_diagnostics_t timing_diagnostics;
    int16_t aligned_reference[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t aligned_raw_adc[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t aligned_corrected_adc[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t alignment_reference[ADC_CHANNEL_SAMPLE_COUNT];
    int16_t canonical_reference_window[CAL_FIXED_WINDOW_LENGTH];
    size_t alignment_reference_count;
    size_t calibration_window_start;
    size_t calibration_window_length;
    uint32_t canonical_reference_checksum;
    float canonical_reference_mean;
    float analysis_reference_scale;
    float canonical_nominal_system_gain;
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
    calibration_offset_result_t source_offset_result;
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
    uint32_t dither_pass;
    uint32_t dither_warning;
    uint32_t dither_invalid;
    uint32_t dither_valid_estimates;
    double mean_dither_offset;
    double mean_existing_dither_delta;
    calibration_dither_offset_diagnostic_t dither_latest;
} calibration_offset_batch_result_t;

typedef struct {
    float mean_raw_system_gain;
    float mean_gain;
    float mean_error;
    float stddev;
    float standard_error;
    float mean_gain_rmse;
    float mean_corrected_adc;
    float mean_correlation;
    uint32_t accepted;
    uint32_t rejected;
} calibration_gain_batch_result_t;

typedef struct {
    float raw_system_gain;
    float raw_adc_mean;
    float corrected_adc_mean;
    float reference_mean;
    float residual_offset;
    float rmse;
} calibration_fixed_offset_gain_metrics_t;

typedef struct {
    uint32_t frame_number;
    int8_t phase;
    int8_t canonical_phase;
    int32_t integer_lag;
    float fractional_lag;
    float correlation;
    float adc_mean;
    float residual;
} calibration_offset_stability_record_t;

typedef struct {
    uint32_t frame_number;
    float mean_residual;
    float rmse;
    float correlation;
    float normalized_gain;
    float raw_reference_mean;
    float scaled_reference_mean;
    float raw_adc_mean;
    float offset_corrected_adc_mean;
    float gain_corrected_adc_mean;
    float reference_fit_s_error_db;
    float sndr_db;
    float enob;
    bool valid;
    bool coherent_sampling;
    bool fundamental_known;
    size_t sample_count;
    size_t transform_length;
    size_t fundamental_bin;
    size_t signal_bin_first;
    size_t signal_bin_last;
    double sample_rate_hz;
    double expected_fundamental_hz;
    double expected_fundamental_bin;
    double detected_fundamental_hz;
    double cycles_in_window;
    double dc_power;
    double signal_power;
    double total_non_dc_power;
    double noise_distortion_power;
    const char *window_name;
    const char *failure_reason;
} adc_performance_frame_result_t;

typedef struct {
    size_t count;
    double mean;
    double m2;
    double minimum;
    double maximum;
} adc_performance_statistics_t;

typedef struct {
    float mean_residual;
    float rmse;
    float raw_reference_mean;
    float scaled_reference_mean;
    float raw_adc_mean;
    float offset_corrected_adc_mean;
    float gain_corrected_adc_mean;
    float reference_fit_s_error_db;
} adc_final_reference_metrics_t;

typedef struct {
    float mean_residual;
    float residual_stddev;
    float residual_standard_error;
    float residual_minimum;
    float residual_maximum;
    float rmse;
    float rmse_stddev;
    float correlation;
    float minimum_correlation;
    float sndr_db;
    float sndr_stddev;
    float minimum_sndr_db;
    float enob;
    float enob_stddev;
    float minimum_enob;
    float mean_normalized_gain;
    float normalized_gain_stddev;
    float offset_verification_residual;
    float offset_verification_standard_error;
    float offset_residual_difference;
    float combined_offset_standard_error;
    float offset_difference_z_like;
    float post_gain_residual;
    float post_gain_residual_standard_error;
    bool reference_metrics_valid;
    bool spectral_metrics_valid;
    bool valid;
    uint32_t frames_attempted;
    uint32_t frames_valid;
    uint32_t frames_rejected;
    int8_t calibration_channel;
    int8_t canonical_reference_phase;
    size_t fixed_window_start;
    size_t fixed_window_length;
    float final_offset_correction;
    float final_gain_correction;
    const char *failure_reason;
    adc_performance_frame_result_t frames[ADC_PERFORMANCE_FRAMES];
} adc_performance_result_t;

typedef enum {
    ADC_CAL_STAGE_IDLE = 0,
    ADC_CAL_STAGE_TIMING,
    ADC_CAL_STAGE_OFFSET,
    ADC_CAL_STAGE_GAIN,
    ADC_CAL_STAGE_VERIFY,
    ADC_CAL_STAGE_PERFORMANCE,
    ADC_CAL_STAGE_COMPLETE,
    ADC_CAL_STAGE_FAILED
} adc_calibration_stage_t;

typedef enum {
    ADC_CAL_RESULT_NONE = 0,
    ADC_CAL_RESULT_PASS,
    ADC_CAL_RESULT_PROVISIONAL,
    ADC_CAL_RESULT_FAILED
} adc_calibration_result_t;

typedef struct {
    bool active;
    bool valid;
    bool timing_pass;
    bool offset_pass;
    bool gain_pass;
    bool gain_verification_pass;
    bool output_valid;
    adc_calibration_stage_t stage;
    adc_calibration_stage_t failed_stage;
    adc_calibration_result_t overall_result;
    calibration_offset_result_t offset_result;
    uint8_t offset_verification_status;
    bool offset_controller_converged;
    int8_t calibration_channel;
    int8_t canonical_reference_phase;
    size_t fixed_window_start;
    size_t fixed_window_length;
    int32_t expected_lag;
    float timing_mean_correlation;
    float offset_correction;
    float offset_verification_error;
    float gain_correction;
    float nominal_system_gain;
    float final_normalized_gain;
    float gain_verification_error;
    const char *failure_reason;
    adc_performance_result_t performance;
    calibration_pending_frame_t final_output;
} adc_automatic_calibration_state_t;

/* Canonical offset convention used by calibration: the stored correction is
 * additive and equals -O, so applying it is raw + correction. */
static double calibration_apply_offset_correction(
    double raw_sample, float offset_correction)
{
    return raw_sample + (double)offset_correction;
}

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
static adc_automatic_calibration_state_t g_automatic_calibration;
static bool g_quiet_calibration_capture;

void calibration_pending_frame_invalidate(void)
{
    g_automatic_calibration.valid = false;
    g_automatic_calibration.output_valid = false;
    g_automatic_calibration.final_output.valid = false;
    memset(&g_automatic_calibration.performance, 0,
           sizeof(g_automatic_calibration.performance));
    memset(&g_stored_offset_reference, 0,
           sizeof(g_stored_offset_reference));
    g_stored_offset_reference.selected_channel = -1;
    g_stored_offset_reference.selected_reference_phase = -1;
    g_stored_offset_reference.canonical_reference_phase = -1;
    memset(&g_pending_calibration_frame, 0,
           sizeof(g_pending_calibration_frame));
    g_pending_calibration_frame.selected_channel = -1;
    g_pending_calibration_frame.selected_reference_phase = -1;
    g_pending_calibration_frame.canonical_reference_phase = -1;
}

void calibration_gain_input_frame_invalidate(void)
{
    g_automatic_calibration.valid = false;
    g_automatic_calibration.output_valid = false;
    g_automatic_calibration.final_output.valid = false;
    memset(&g_automatic_calibration.performance, 0,
           sizeof(g_automatic_calibration.performance));
    memset(&g_pending_calibration_frame, 0,
           sizeof(g_pending_calibration_frame));
    g_pending_calibration_frame.selected_channel = -1;
    g_pending_calibration_frame.selected_reference_phase = -1;
    g_pending_calibration_frame.canonical_reference_phase = -1;
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
static void calibration_run_timing_alignment_diagnostic(uint32_t frame_count);
static void calibration_run_raw_mapping_diagnostic(
    const int16_t *even_reference, const int16_t *odd_reference);
static void handle_adc_offset_calibration_loop_cmd(void);
static void handle_adc_offset_calibration_status_cmd(void);
static void handle_adc_offset_stability_cmd(uint32_t frame_count);
static void handle_adc_timing_calibration_stage_cmd(uint32_t frame_count);
static void handle_adc_gain_calibration_loop_cmd(void);
static void handle_adc_gain_calibration_status_cmd(void);
static int adc_run_timing_calibration(uint32_t frame_count);
static void calibration_automatic_state_reset(void);
static void calibration_automatic_print_command_help(void);
static void calibration_automatic_print_summary(void);
static const char *calibration_offset_verification_name(uint8_t status);
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
static int calibration_compute_timing_diagnostics(
    const int16_t *alignment_reference,
    const int16_t *channel_a,
    const int16_t *channel_b,
    size_t sample_count,
    int selected_channel,
    double expected_tone_hz,
    double adc_sample_rate_hz,
    double existing_lag,
    size_t fixed_window_start,
    size_t fixed_window_length,
    calibration_timing_diagnostics_t *diagnostics
);
static void calibration_print_timing_diagnostics_compact(
    const calibration_timing_diagnostics_t *diagnostics);
static void calibration_print_timing_diagnostics_detail(
    const calibration_aligned_frame_t *frame,
    const calibration_timing_diagnostics_t *diagnostics);
static void calibration_validate_timing_alignment(
    const calibration_aligned_frame_t *frame,
    calibration_timing_diagnostics_t *diagnostics);
static const char *calibration_dither_offset_status_name(
    calibration_dither_offset_status_t status);
static const char *calibration_dither_offset_reason_name(
    calibration_dither_offset_reason_t reason);
static void calibration_print_dither_offset_diagnostic(
    const calibration_dither_offset_diagnostic_t *diagnostic);
static int calibration_capture_against_owned_reference(
    const calibration_pending_frame_t *saved,
    bool use_saved_calibration_reference,
    float gain_correction,
    float offset_correction,
    float reference_scale,
    calibration_frame_workspace_t *workspace,
    calibration_aligned_frame_t *frame,
    const char **reason);
static const char *calibration_channel_name(int channel);
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
static void print_signed_float_inline(float value);
static void print_float_value_2(const char *label, float value,
                                const char *unit);
static void print_float_value_or_invalid(const char *label, float value,
                                         const char *unit);
static void print_signed_float_value_or_invalid(const char *label,
                                                float value,
                                                const char *unit);

static bool calibration_compact_output_enabled(void)
{
    return g_automatic_calibration.active && !ADC_CAL_VERBOSE_DEBUG;
}

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

static void print_adc_calibration_rate_summary(void)
{
    const double configured_hz = adc_get_configured_sample_rate_hz();
    const double analysis_hz = adc_get_effective_sample_rate_hz();

    if (fabs(configured_hz - analysis_hz) <= 0.5) {
        print_double_value("Sample rate", analysis_hz / 1.0e6, " MSPS");
    } else {
        print_double_value("Configured sample rate",
                           configured_hz / 1.0e6, " MSPS");
        print_double_value("Analysis sample rate",
                           analysis_hz / 1.0e6, " MSPS");
    }
    print_double_value("DAC/ADC rate ratio",
                       DAC_SAMPLE_RATE_HZ / analysis_hz, "");
    if (ADC_CAL_VERBOSE_DEBUG) {
        xil_printf("Rate source            : %s\r\n",
            adc_effective_sample_rate_is_valid() ?
            "known-tone measurement" : "configured value");
        print_double_value("Correction factor",
            adc_get_sample_rate_correction_factor(), "");
    }
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

static void print_signed_float_inline(float value)
{
    if (!isfinite(value)) {
        xil_printf("invalid");
        return;
    }
    if (value >= 0.0f) xil_printf("+");
    print_double_inline((double)value);
}

static void print_float_value_2(
    const char *label, float value, const char *unit)
{
    int32_t scaled;

    if (!isfinite(value)) {
        xil_printf("%-22s: INVALID\r\n", label);
        return;
    }
    scaled = (int32_t)lroundf(fabsf(value) * 100.0f);
    xil_printf("%-22s: %s%ld.%02ld%s\r\n",
               label, value < 0.0f ? "-" : "",
               (long)(scaled / 100), (long)(scaled % 100),
               unit != NULL ? unit : "");
}

static void print_float_value_or_invalid(
    const char *label, float value, const char *unit)
{
    if (isfinite(value)) {
        print_float_value(label, value, unit);
    } else {
        xil_printf("%-22s: INVALID\r\n", label);
    }
}

static void print_signed_float_value_or_invalid(
    const char *label, float value, const char *unit)
{
    if (!isfinite(value)) {
        xil_printf("%-22s: INVALID\r\n", label);
        return;
    }
    xil_printf("%-22s: %s", label, value >= 0.0f ? "+" : "");
    print_double_inline((double)value);
    xil_printf("%s\r\n", unit != NULL ? unit : "");
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

static void calibration_automatic_state_reset(void)
{
    memset(&g_automatic_calibration, 0, sizeof(g_automatic_calibration));
    g_automatic_calibration.stage = ADC_CAL_STAGE_IDLE;
    g_automatic_calibration.failed_stage = ADC_CAL_STAGE_IDLE;
    g_automatic_calibration.calibration_channel = -1;
    g_automatic_calibration.canonical_reference_phase = -1;
    g_automatic_calibration.gain_correction = 1.0f;
}

static void calibration_automatic_print_command_help(void)
{
    xil_printf("\r\nADC calibration commands:\r\n");
    xil_printf("  adc -cal\r\n");
    xil_printf("      Run complete ADC timing, offset, and gain calibration.\r\n");
    xil_printf("\r\nDebug/development stage commands:\r\n");
    xil_printf("  adc -cal timing [frames]  Run timing/reference selection only.\r\n");
    xil_printf("  adc -cal diagnose [frames]\r\n");
    xil_printf("      Run timing tone/dither diagnostics without storing state.\r\n");
    xil_printf("  adc -cal offset           Run offset stage only.\r\n");
    xil_printf("  adc -cal gain             Run gain stage only.\r\n");
    xil_printf("  adc -cal stability [frames]\r\n");
    xil_printf("      Characterize fixed-offset capture stability.\r\n");
    xil_printf("  adc -cal status | reset | help\r\n");
}

static void calibration_print_stage_header(
    uint32_t stage_number, const char *stage_name)
{
    xil_printf("\r\nStage %lu/4: %s\r\n",
               (unsigned long)stage_number, stage_name);
    xil_printf("-----------------------------------------\r\n");
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
        if (strcmp(token, "timing") == 0) {
            uint32_t timing_frames = ADC_CAL_DEFAULT_FRAMES;
            token = strtok(NULL, " ");
            if (token != NULL) {
                char *endptr = NULL;
                const unsigned long parsed = strtoul(token, &endptr, 0);
                if (endptr == token || *endptr != '\0' ||
                    parsed < ADC_CAL_MIN_FRAMES ||
                    parsed > ADC_CAL_MAX_FRAMES ||
                    strtok(NULL, " ") != NULL) {
                    ERR("Use adc -cal timing [%u..%u].",
                        ADC_CAL_MIN_FRAMES, ADC_CAL_MAX_FRAMES);
                    return;
                }
                timing_frames = (uint32_t)parsed;
            }
            handle_adc_timing_calibration_stage_cmd(timing_frames);
            return;
        }
        if (strcmp(token, "diagnose") == 0) {
            uint32_t diagnostic_frames = 1U;
            token = strtok(NULL, " ");
            if (token != NULL) {
                char *endptr = NULL;
                const unsigned long parsed = strtoul(token, &endptr, 0);
                if (endptr == token || *endptr != '\0' ||
                    parsed == 0U || parsed > ADC_CAL_MAX_FRAMES ||
                    strtok(NULL, " ") != NULL) {
                    ERR("Use adc -cal diagnose [1..%u].",
                        ADC_CAL_MAX_FRAMES);
                    return;
                }
                diagnostic_frames = (uint32_t)parsed;
            }
            calibration_run_timing_alignment_diagnostic(diagnostic_frames);
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
        if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0) {
            if (strtok(NULL, " ") != NULL) {
                ERR("Use adc -cal help.");
                return;
            }
            calibration_automatic_print_command_help();
            return;
        }
        if (strcmp(token, "stability") == 0) {
            uint32_t stability_frames =
                CAL_OFFSET_STABILITY_DEFAULT_FRAMES;
            token = strtok(NULL, " ");
            if (token != NULL) {
                char *endptr = NULL;
                const unsigned long parsed = strtoul(token, &endptr, 0);
                if (endptr == token || *endptr != '\0' || parsed == 0U ||
                    parsed > CAL_OFFSET_STABILITY_MAX_FRAMES ||
                    strtok(NULL, " ") != NULL) {
                    ERR("Use adc -cal stability [1..%u].",
                        CAL_OFFSET_STABILITY_MAX_FRAMES);
                    return;
                }
                stability_frames = (uint32_t)parsed;
            }
            handle_adc_offset_stability_cmd(stability_frames);
            return;
        }
        if (strcmp(token, "status") == 0) {
            if (strtok(NULL, " ") != NULL) {
                ERR("Use adc -cal status.");
                return;
            }
            calibration_automatic_print_summary();
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
            calibration_automatic_state_reset();
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
        ERR("Invalid option \"%s\" (use -c, status, -timing [frames], -gain, -offset, -cal [frames|timing|diagnose|gain|offset|stability|status|reset|help], -ref, or -ref diagnose)", option);
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

/*
 * Software ADC calibration and reference diagnostics are implemented in a
 * companion source file to keep the UART command dispatcher readable while
 * preserving the existing file-local helper/state boundaries.
 */
#include "butils_calibration.c"
