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
#include <stdarg.h>
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
#include "adc_calibration_pipeline.h"
#include "adc_calibration_dither.h"
#include "adc_calibration_skew.h"
#include "adc_calibration_performance.h"
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
    double measured_rate_hz;
    bool measured_rate_valid;
    uint32_t generation;
} adc_sample_rate_state_t;

static adc_sample_rate_state_t g_adc_sample_rate = {
    ADC_CONFIGURED_SAMPLE_RATE_HZ,
    0.0,
    false,
    0U
};

double adc_get_configured_sample_rate_hz(void)
{
    return g_adc_sample_rate.configured_rate_hz;
}

double adc_get_effective_sample_rate_hz(void)
{
    return g_adc_sample_rate.configured_rate_hz;
}

double adc_get_sample_rate_correction_factor(void)
{
    /* Retained for CSV schema compatibility. Analysis no longer substitutes
     * an inferred clock for the authoritative hardware configuration. */
    return 1.0;
}

double adc_get_measured_sample_rate_hz(void)
{
    return g_adc_sample_rate.measured_rate_hz;
}

bool adc_measured_sample_rate_is_valid(void)
{
    return g_adc_sample_rate.measured_rate_valid;
}

bool adc_record_measured_sample_rate_hz(double rate_hz)
{
    const double configured_hz = g_adc_sample_rate.configured_rate_hz;
    if (!isfinite(rate_hz) || rate_hz < 0.8 * configured_hz ||
        rate_hz > 1.2 * configured_hz)
        return false;
    g_adc_sample_rate.measured_rate_hz = rate_hz;
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
#define ADC_CAL_EXPORT_BUFFER_SIZE             524288U
#define ADC_CAL_TIMING_HISTORY_CAPACITY \
    ADC_TIMING_MAX_FRAMES
#define ADC_CAL_BASELINE_CAPTURE_HISTORY_CAPACITY \
    ADC_PERFORMANCE_FRAMES
#define ADC_CAL_OFFSET_CAPTURE_HISTORY_CAPACITY \
    ((CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS + \
      CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES) * \
     CALIBRATION_OFFSET_BATCH_SIZE + CAL_UPDATE_FRAME_BATCH_SIZE)
#define ADC_CAL_OFFSET_HISTORY_CAPACITY \
    CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS
#define ADC_CAL_GAIN_CAPTURE_HISTORY_CAPACITY \
    ((CALIBRATION_GAIN_MAX_ACCEPTED_ITERATIONS + 1U) * \
     CALIBRATION_GAIN_BATCH_SIZE)
#define ADC_CAL_GAIN_HISTORY_CAPACITY \
    CALIBRATION_GAIN_MAX_ACCEPTED_ITERATIONS
/* One closed-loop skew run stores up to:
 *   initial baseline (1 batch)
 * + 4 characterization rungs x 4 batches (baseline/probe x 2)
 * + up to CAL_SKEW_MAX_ITERATIONS controller batches.
 * With 10-frame batches and 10 controller iterations that is 270 frames;
 * keep margin so the final repeat probe and every controller batch fit. */
#define ADC_CAL_SKEW_CAPTURE_HISTORY_CAPACITY       320U
#define ADC_CAL_SKEW_HISTORY_CAPACITY              10U
#define ADC_PERFORMANCE_FUNDAMENTAL_SEARCH_BINS   3U
#define ADC_PERFORMANCE_HANN_SIGNAL_HALF_WIDTH    2U
#define CAL_TONE_REFINE_HALF_RANGE_BINS            0.75

static char g_adc_cal_export_buffer[ADC_CAL_EXPORT_BUFFER_SIZE];
static size_t g_adc_cal_export_length = 0U;
static bool g_adc_cal_export_available = false;

typedef struct {
    uint32_t frame_index;
    uint32_t global_capture_index;
    bool accepted;
    int8_t channel;
    int32_t integer_lag_samples;
    float fractional_lag_samples;
    float total_lag_samples;
    float correlation;
    int8_t canonical_phase;
    double expected_tone_frequency_hz;
    double fitted_tone_frequency_hz;
    double tone_fit_error_hz;
    float tone_fit_rmse_codes;
    float tone_correlation;
    bool dither_valid;
    double dither_peak;
    double dither_lag_samples;
    bool selected_reference_frame;
    double physical_adc_rate_hz;
    double configured_dac_rate_hz;
    double reference_rate_compensation;
    double selected_reference_ratio;
    float offset_correction_active_codes;
    float gain_correction_active;
    double delay_register_active;
    double active_polarity;
    const char *status;
    /* Selected-channel spectral SNDR/ENOB of the analysis window (dB/bits). */
    float sndr_db;
    float enob_bits;
} adc_cal_timing_history_record_t;

typedef struct {
    uint32_t capture_group_index;
    uint32_t capture_index;
    uint32_t global_capture_index;
    uint32_t performance_frame_index;
    bool accepted;
    const char *rejection_reason;
    double physical_adc_rate_hz;
    double configured_dac_rate_hz;
    double reference_rate_compensation;
    int8_t channel;
    int8_t canonical_phase;
    float offset_correction_active_codes;
    float gain_correction_active;
    double delay_register_active;
    double active_polarity;
    float correlation;
    float selected_adc_mean_codes;
    double channel_a_mean_codes;
    double channel_b_mean_codes;
    double channel_a_rms_codes;
    double channel_b_rms_codes;
} adc_cal_baseline_capture_record_t;

typedef struct {
    uint32_t iteration;
    uint32_t capture_group_index;
    uint32_t capture_index;
    uint32_t global_capture_index;
    const char *capture_phase;
    bool accepted;
    const char *rejection_reason;
    double physical_adc_rate_hz;
    double configured_dac_rate_hz;
    double reference_rate_compensation;
    int8_t channel;
    int8_t canonical_phase;
    float offset_correction_active_codes;
    float gain_correction_active;
    double delay_register_active;
    double active_polarity;
    float filtered_residual_at_iteration_start;
    float measured_offset_residual_codes;
    float fit_offset_codes;
    float correlation;
    float rmse_codes;
    float tolerance_codes;
    /* Selected-channel spectral SNDR/ENOB of the corrected analysis window. */
    float sndr_db;
    float enob_bits;
} adc_cal_offset_capture_record_t;

typedef struct {
    uint32_t iteration;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    float residual_offset_codes;
    float filtered_residual_codes;
    float correction_before_codes;
    float correction_after_codes;
    float correction_delta_codes;
    float tolerance_codes;
    uint32_t consecutive_passes;
    uint32_t required_passes;
    float residual_std_codes;
    float residual_min_codes;
    float residual_max_codes;
    const char *status;
    /* Mean accepted-frame spectral SNDR/ENOB of the corrected analysis
     * window (dB/bits). */
    float sndr_db;
    float enob_bits;
} adc_cal_offset_history_record_t;

typedef struct {
    uint32_t iteration;
    uint32_t capture_group_index;
    uint32_t capture_index;
    uint32_t global_capture_index;
    const char *capture_phase;
    bool accepted;
    const char *rejection_reason;
    double physical_adc_rate_hz;
    double configured_dac_rate_hz;
    double reference_rate_compensation;
    int8_t channel;
    int8_t canonical_phase;
    float offset_correction_active_codes;
    float gain_correction_active;
    double delay_register_active;
    double active_polarity;
    float measured_gain;
    float gain_error;
    float correlation;
    float rmse_codes;
    double dither_gain;
    bool dither_valid;
    const char *dither_reason;
    /* Selected-channel spectral SNDR/ENOB of the corrected analysis window. */
    float sndr_db;
    float enob_bits;
} adc_cal_gain_capture_record_t;

typedef struct {
    uint32_t iteration;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    float measured_gain;
    float gain_error;
    float correction_before;
    float correction_after;
    float correction_delta;
    float tolerance;
    uint32_t consecutive_passes;
    uint32_t required_passes;
    float verification_correlation;
    double dither_gain;
    const char *dither_gain_status;
    uint32_t dither_event_count;
    const char *dither_warning_reason;
    const char *status;
    /* Mean accepted-frame spectral SNDR/ENOB of the corrected window. */
    float sndr_db;
    float enob_bits;
} adc_cal_gain_history_record_t;

typedef struct {
    uint32_t iteration;
    uint32_t capture_group_index;
    uint32_t capture_index;
    uint32_t global_capture_index;
    const char *capture_phase;
    bool accepted;
    const char *rejection_reason;
    double physical_adc_rate_hz;
    double configured_dac_rate_hz;
    double reference_rate_compensation;
    int8_t channel;
    int8_t canonical_phase;
    int delay_register_active;
    float offset_correction_active_codes;
    float gain_correction_active;
    double active_polarity;
    bool primary_valid;
    const char *primary_rejection_reason;
    double raw_phase_difference_rad;
    int polarity_hypothesis;
    double applied_phase_adjustment_rad;
    double corrected_phase_difference_rad;
    double measured_skew_samples;
    double measured_skew_ps;
    double tone_a_frequency_hz;
    double tone_b_frequency_hz;
    double tone_a_amplitude;
    double tone_b_amplitude;
    double tone_a_correlation;
    double tone_b_correlation;
    double tone_a_rmse;
    double tone_b_rmse;
    bool dither_a_valid;
    bool dither_b_valid;
    bool dither_skew_valid;
    double dither_skew_samples;
    double dither_skew_ps;
    double tone_dither_disagreement_samples;
    double tone_dither_disagreement_ps;
    /* Per-frame tone-fit SNDR/ENOB (A and B, from amplitude/rmse). */
    double tone_a_sndr_db;
    double tone_a_enob_bits;
    double tone_b_sndr_db;
    double tone_b_enob_bits;
    /* Per-frame dither diagnostics; NAN when the estimate is unavailable. */
    double dither_rising_skew_ps;
    double dither_falling_skew_ps;
    double dither_edge_disagreement_ps;
    const char *dither_reason;
} adc_cal_skew_capture_record_t;

typedef struct {
    uint32_t iteration;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    double skew_mean_samples;
    double skew_mean_ps;
    double skew_median_samples;
    double skew_median_ps;
    double skew_std_samples;
    double skew_std_ps;
    double best_skew_samples;
    double best_skew_ps;
    int delay_register_before;
    int delay_register_after;
    int register_change;
    double actuator_resolution_samples_per_step;
    double actuator_resolution_ps_per_step;
    int correction_requested_steps;
    int correction_applied_steps;
    double skew_tolerance_samples;
    double skew_tolerance_ps;
    uint32_t consecutive_passes;
    uint32_t required_passes;
    bool estimator_valid;
    bool estimator_stable;
    bool correction_applied;
    bool saturated;
    double dither_skew_samples;
    double dither_skew_ps;
    double tone_dither_disagreement_ps;
    uint32_t dither_valid_frames;
    uint32_t dither_invalid_frames;
    const char *status;
    /* Mean accepted-frame tone-fit SNDR/ENOB (A and B). */
    double tone_a_sndr_db;
    double tone_a_enob_bits;
    double tone_b_sndr_db;
    double tone_b_enob_bits;
    /* Cross-frame joint dither estimate for this iteration's batch
     * (all accepted frames aggregated into one profile pair). */
    bool dither_joint_valid;
    double dither_joint_skew_ps;
    double dither_joint_edge_disagreement_ps;
    double dither_joint_tone_disagreement_ps;
} adc_cal_skew_history_record_t;

typedef struct {
    adc_cal_timing_history_record_t timing[
        ADC_CAL_TIMING_HISTORY_CAPACITY];
    adc_cal_baseline_capture_record_t baseline_captures[
        ADC_CAL_BASELINE_CAPTURE_HISTORY_CAPACITY];
    adc_cal_offset_capture_record_t offset_captures[
        ADC_CAL_OFFSET_CAPTURE_HISTORY_CAPACITY];
    adc_cal_offset_history_record_t offset[
        ADC_CAL_OFFSET_HISTORY_CAPACITY];
    adc_cal_gain_capture_record_t gain_captures[
        ADC_CAL_GAIN_CAPTURE_HISTORY_CAPACITY];
    adc_cal_gain_history_record_t gain[
        ADC_CAL_GAIN_HISTORY_CAPACITY];
    adc_cal_skew_capture_record_t skew_captures[
        ADC_CAL_SKEW_CAPTURE_HISTORY_CAPACITY];
    adc_cal_skew_history_record_t skew[
        ADC_CAL_SKEW_HISTORY_CAPACITY];
    uint32_t timing_count;
    uint32_t baseline_capture_count;
    uint32_t offset_capture_count;
    uint32_t offset_count;
    uint32_t gain_capture_count;
    uint32_t gain_count;
    uint32_t skew_capture_count;
    uint32_t skew_count;
    uint32_t global_capture_count;
    bool timing_truncated;
    bool baseline_captures_truncated;
    bool offset_captures_truncated;
    bool gain_captures_truncated;
    bool skew_captures_truncated;
} adc_cal_export_history_t;

static adc_cal_export_history_t g_adc_cal_export_history;

typedef struct {
    uint32_t iteration;
    uint32_t capture_group_index;
    int active_delay_register;
    const char *capture_phase;
} adc_cal_skew_capture_context_t;

static adc_cal_skew_capture_context_t g_adc_cal_skew_capture_context = {
    0U, 0U, -1, "baseline"
};
#define CAL_TONE_REFINE_MIN_STEP                  1.0e-12
#define CAL_TONE_REFINE_MAX_ITERATIONS              40U
#define CAL_DITHER_EVENT_THRESHOLD_FRACTION         0.25
#define CAL_DITHER_CONSISTENCY_TOLERANCE_SAMPLES    1.0
#ifndef CAL_TONE_VALIDATION_MAX_RMSE_CODES
#define CAL_TONE_VALIDATION_MAX_RMSE_CODES        512.0
#endif
#ifndef CAL_TONE_VALIDATION_MIN_CORRELATION
#define CAL_TONE_VALIDATION_MIN_CORRELATION       CAL_DAC_REF_MIN_CORRELATION
#endif
#ifndef CAL_DITHER_VALIDATION_WEAK_PEAK_RATIO
#define CAL_DITHER_VALIDATION_WEAK_PEAK_RATIO       ADC_CAL_DITHER_DEFAULT_WEAK_PEAK_RATIO
#endif
#ifndef CAL_DITHER_VALIDATION_STRONG_PEAK_RATIO
#define CAL_DITHER_VALIDATION_STRONG_PEAK_RATIO     ADC_CAL_DITHER_DEFAULT_STRONG_PEAK_RATIO
#endif
#ifndef CAL_DITHER_PERIODIC_EXCLUSION_WIDTH_SAMPLES
#define CAL_DITHER_PERIODIC_EXCLUSION_WIDTH_SAMPLES ADC_CAL_DITHER_DEFAULT_PERIODIC_EXCLUSION_WIDTH
#endif
#ifndef CAL_DITHER_MAX_CHANNEL_CANDIDATES
#define CAL_DITHER_MAX_CHANNEL_CANDIDATES ADC_CAL_DITHER_DEFAULT_CANDIDATE_COUNT
#endif
#ifndef CAL_DITHER_EDGE_REFINE_RADIUS
#define CAL_DITHER_EDGE_REFINE_RADIUS ADC_CAL_DITHER_DEFAULT_EDGE_REFINE_RADIUS
#endif
#ifndef CAL_DITHER_INTERNAL_CONTEXT_TOLERANCE_SAMPLES
#define CAL_DITHER_INTERNAL_CONTEXT_TOLERANCE_SAMPLES 1.0e-6
#endif
#ifndef CAL_DITHER_INTERNAL_EVENT_FAMILY_TOLERANCE_SAMPLES
#define CAL_DITHER_INTERNAL_EVENT_FAMILY_TOLERANCE_SAMPLES 1.0
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
#ifndef CAL_DITHER_GAIN_MIN_COMPLETE_EVENTS
#define CAL_DITHER_GAIN_MIN_COMPLETE_EVENTS          CAL_DITHER_OFFSET_MIN_COMPLETE_EVENTS
#endif
#ifndef CAL_DITHER_GAIN_DENOMINATOR_FLOOR
#define CAL_DITHER_GAIN_DENOMINATOR_FLOOR            CAL_DITHER_OFFSET_DENOMINATOR_FLOOR
#endif
#ifndef CAL_DITHER_GAIN_TEMPLATE_ENERGY_FLOOR
#define CAL_DITHER_GAIN_TEMPLATE_ENERGY_FLOOR        1.0e-6
#endif
#ifndef CAL_DITHER_GAIN_MIN_POSITIVE_GAIN
#define CAL_DITHER_GAIN_MIN_POSITIVE_GAIN            0.0001
#endif
#ifndef CAL_DITHER_GAIN_MIN_PLAUSIBLE_GAIN
#define CAL_DITHER_GAIN_MIN_PLAUSIBLE_GAIN           0.0001
#endif
#ifndef CAL_DITHER_GAIN_MAX_PLAUSIBLE_GAIN
#define CAL_DITHER_GAIN_MAX_PLAUSIBLE_GAIN           20.0
#endif
#ifndef CAL_DITHER_GAIN_MIN_TEMPLATE_CORRELATION
#define CAL_DITHER_GAIN_MIN_TEMPLATE_CORRELATION     0.80
#endif
#ifndef CAL_DITHER_GAIN_MAX_NORMALIZED_FIT_RMSE
#define CAL_DITHER_GAIN_MAX_NORMALIZED_FIT_RMSE      0.50
#endif
#ifndef CAL_DITHER_GAIN_FULL_FLAT_TOLERANCE
#define CAL_DITHER_GAIN_FULL_FLAT_TOLERANCE          0.10
#endif
#ifndef CAL_DITHER_GAIN_AGREEMENT_TOLERANCE
#define CAL_DITHER_GAIN_AGREEMENT_TOLERANCE          0.10
#endif
#ifndef CAL_SKEW_BATCH_SIZE
#define CAL_SKEW_BATCH_SIZE                          10U
#endif
#ifndef CAL_SKEW_MIN_ACCEPTED_FRAMES
#define CAL_SKEW_MIN_ACCEPTED_FRAMES                 3U
#endif
#ifndef CAL_SKEW_REQUIRED_CONVERGED_BATCHES
#define CAL_SKEW_REQUIRED_CONVERGED_BATCHES          2U
#endif
#ifndef CAL_SKEW_CLOSED_LOOP_ENABLE
/* The automatic adc -cal pipeline runs the validated Stage 4 correction
 * loop. Stage-only `adc -cal skew` remains open-loop by default. */
#define CAL_SKEW_CLOSED_LOOP_ENABLE                   1
#endif
#ifndef CAL_SKEW_BATCH_DIAGNOSTICS
/* Per-frame skew diagnostics print ~20 lines per frame and flood the UART
 * (10 frames x many batches x iterations drown the earlier stage output;
 * 2026-08-19 the gain/offset logs became unreachable).  All per-frame
 * detail is exported in the calibration_skew_captures.csv diagnostic
 * columns, so keep the UART summary-only by default. */
#define CAL_SKEW_BATCH_DIAGNOSTICS                    0
#endif
#ifndef CAL_SKEW_MAX_ITERATIONS
#define CAL_SKEW_MAX_ITERATIONS                       10U
#endif
#ifndef CAL_SKEW_CONTROLLER_GAIN
#define CAL_SKEW_CONTROLLER_GAIN                      0.5
#endif
#ifndef CAL_SKEW_MAX_STEPS_PER_ITERATION
#define CAL_SKEW_MAX_STEPS_PER_ITERATION              16
#endif
#ifndef CAL_SKEW_REGISTER_MIN
#define CAL_SKEW_REGISTER_MIN                         0
#endif
#ifndef CAL_SKEW_REGISTER_MAX
#define CAL_SKEW_REGISTER_MAX                          48
#endif
#ifndef CAL_SKEW_ACTUATOR_STEP_SAMPLES
/* Learned from mandatory hardware characterization. */
#define CAL_SKEW_ACTUATOR_STEP_SAMPLES                 0.0
#endif
#ifndef CAL_SKEW_ACTUATOR_POLARITY
#define CAL_SKEW_ACTUATOR_POLARITY                    0
#endif

/* Hardware contract from system.bd:
 *
 *   skew_delay_gpio/S_AXI  = 0xA0030000
 *   GPIO channel 1 DATA    = base + 0x0
 *   GPIO width             = 10 bits, output-only
 *   reset/neutral code     = 256
 *   actuator range         = 0..512 (0..2 samples in Q8 units)
 *
 * Require the BSP-generated instance name.  A block-design address alone is
 * not proof that the currently loaded bitstream contains an AXI slave there;
 * probing an absent slave with Xil_In32 can block the CPU indefinitely.
 * The controller does not assume the measurement sign implied by the HDL.
 * CAL_SKEW_ACTUATOR_POLARITY remains unknown (0) until the mandatory +1-step
 * characterization measures the sign from fresh capture batches. */
#if defined(XPAR_SKEW_DELAY_GPIO_BASEADDR)
#define ADC_SKEW_ACTUATOR_GPIO_BASE_ADDRESS           ((UINTPTR)XPAR_SKEW_DELAY_GPIO_BASEADDR)
#elif defined(XPAR_SKEW_DELAY_GPIO_0_BASEADDR)
#define ADC_SKEW_ACTUATOR_GPIO_BASE_ADDRESS           ((UINTPTR)XPAR_SKEW_DELAY_GPIO_0_BASEADDR)
#endif
#if defined(ADC_SKEW_ACTUATOR_GPIO_BASE_ADDRESS)
#define ADC_SKEW_ACTUATOR_REGISTER_ADDRESS            (ADC_SKEW_ACTUATOR_GPIO_BASE_ADDRESS + (UINTPTR)0x0U)
#define ADC_SKEW_ACTUATOR_REGISTER_MASK               0x000003FFU
#define ADC_SKEW_ACTUATOR_REGISTER_SHIFT              0U
#endif

/* The AD9695 has a real per-channel fine sampling-clock delay.  A logical
 * position applies complementary Channel-A/Channel-B codes whose sum remains
 * 192.  Four raw 1.725 ps taps move each channel in opposite directions, so
 * one characterized relative step is about 13.8 ps. */
#define ADC_SKEW_AD9695_CHANNEL_A_SELECT               0x01U
#define ADC_SKEW_AD9695_CHANNEL_B_SELECT               0x02U
#define ADC_SKEW_AD9695_BROADCAST_SELECT               0x03U
#define ADC_SKEW_AD9695_FINE_DELAY_MODE                AD9695_FINE_DELAY_192
#define ADC_SKEW_AD9695_RAW_STEPS_PER_CONTROL_STEP      4U
#define ADC_SKEW_AD9695_RAW_MIDPOINT                    96U
#define ADC_SKEW_AD9695_INITIAL_CONTROL_CODE           24
#ifndef CAL_SKEW_TOLERANCE_SAMPLES
#define CAL_SKEW_TOLERANCE_SAMPLES                   0.01
#endif
#ifndef CAL_SKEW_MAX_LINEAR_SKEW_SAMPLES
#define CAL_SKEW_MAX_LINEAR_SKEW_SAMPLES             0.25
#endif
#ifndef CAL_SKEW_MAX_EDGE_DISAGREEMENT_SAMPLES
#define CAL_SKEW_MAX_EDGE_DISAGREEMENT_SAMPLES       0.03
#endif
#ifndef CAL_SKEW_DITHER_PROFILE_WINDOW_HALF
/* The bench analog chain disperses the injected impulse into a ~75-125
 * ADC-sample 10-90% blob; widen the dither profile aggregation window so
 * the dispersed pulse edges are not truncated by the template-sized
 * window.  Kept below half the 130-sample dither period to avoid overlap
 * (the doubled-period 260-sample geometry was tested and rejected on
 * 2026-08-19 because the wider pulse pollutes the tone fit; keep this
 * macro matched to the waveform actually loaded on the DPG). */
#define CAL_SKEW_DITHER_PROFILE_WINDOW_HALF           64U
#endif
#ifndef CAL_SKEW_DITHER_PROFILE_MASK_FRACTION
/* With the widened +-64 window, tone-residual and ringing tails outside
 * the pulse body otherwise dominate the rising/falling derivative
 * projection (2026-08-16: edge estimates blew out to -402/+216 ps).
 * Gate the projection to |template| >= 15 % of the profile peak. */
#define CAL_SKEW_DITHER_PROFILE_MASK_FRACTION         0.15
#endif
#ifndef CAL_SKEW_DITHER_PROFILE_WINDOW_DETREND
/* Per-event window linear detrending.  Offline replay showed it helps some
 * frames (100-240 ps -> <23.1 ps) but hurts others (window nearly fills the
 * 130-sample event period, so the linear fit absorbs pulse shoulders), and
 * the 2026-08-19 board run with detrend=1 still reported dither-valid
 * 0/10 on every skew iteration.  Kept OFF: the cross-frame joint estimate
 * (adc_cal_skew_estimate_joint_frames) randomizes the same window bias
 * across frames without this tradeoff. */
#define CAL_SKEW_DITHER_PROFILE_WINDOW_DETREND         0U
#endif
#ifndef CAL_SKEW_WARN_EDGE_DISAGREEMENT_SAMPLES
#define CAL_SKEW_WARN_EDGE_DISAGREEMENT_SAMPLES      0.015
#endif
#ifndef CAL_SKEW_CHANNEL_B_RELATIVE_POLARITY
/* adc_frame.c performs no digital sign inversion.  Leave the effective
 * board/analog polarity unknown unless platform metadata overrides it. */
#define CAL_SKEW_CHANNEL_B_RELATIVE_POLARITY ADC_CAL_SKEW_POLARITY_UNKNOWN
#endif
#ifndef CAL_SKEW_DERIVATIVE_ENERGY_FLOOR
#define CAL_SKEW_DERIVATIVE_ENERGY_FLOOR             1.0e-6
#endif
#ifndef CAL_SKEW_EDGE_DERIVATIVE_FRACTION
#define CAL_SKEW_EDGE_DERIVATIVE_FRACTION            0.35
#endif
#ifndef CAL_SKEW_MIN_EDGE_SAMPLES
#define CAL_SKEW_MIN_EDGE_SAMPLES                    2U
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
    int32_t channel_frame_offset;
    int32_t channel_event_offset;
    double channel_family_disagreement_samples;
    uint32_t window_partial_event_count;
    uint32_t total_dither_event_count;
    uint8_t dither_event_indices_valid;
    const char *numerical_reason;
    adc_cal_dither_confidence_t peak_confidence;
    adc_cal_dither_recommendation_t recommendation;
    adc_cal_dither_validation_reason_t dither_reason;
} calibration_timing_validation_t;

typedef struct {
    uint8_t valid;
    uint32_t candidate_rank;
    uint32_t candidate_count;
    uint32_t coarse_peak_index;
    double coarse_lag_samples;
    uint32_t peak_index;
    double fractional_offset_samples;
    double n0_integer;
    double n0_fractional;
    double sign;
    double peak;
    uint32_t global_strongest_index;
    double global_strongest_peak;
    double raw_second_peak;
    double second_peak;
    double raw_peak_ratio;
    double peak_ratio;
    double align_margin;
    double derived_lag;
    adc_cal_dither_confidence_t peak_confidence;
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

typedef enum {
    CAL_DITHER_GAIN_STATUS_INVALID = 0,
    CAL_DITHER_GAIN_STATUS_PASS,
    CAL_DITHER_GAIN_STATUS_WARNING
} calibration_dither_gain_status_t;

typedef enum {
    CAL_DITHER_GAIN_REASON_NONE = 0,
    CAL_DITHER_GAIN_REASON_CONTEXT,
    CAL_DITHER_GAIN_REASON_TONE_FIT,
    CAL_DITHER_GAIN_REASON_TOO_FEW_EVENTS,
    CAL_DITHER_GAIN_REASON_POLARITY_IMBALANCE,
    CAL_DITHER_GAIN_REASON_TEMPLATE,
    CAL_DITHER_GAIN_REASON_GAIN_VALUE,
    CAL_DITHER_GAIN_REASON_FIT_QUALITY,
    CAL_DITHER_GAIN_REASON_INTERPOLATION,
    CAL_DITHER_GAIN_REASON_NUMERICAL,
    CAL_DITHER_GAIN_REASON_NO_DITHER,
    CAL_DITHER_GAIN_REASON_EVENT_PROFILE
} calibration_dither_gain_reason_t;

typedef struct {
    uint8_t valid;
    calibration_dither_gain_status_t status;
    calibration_dither_gain_reason_t reason;
    uint8_t context_pass;
    uint8_t tone_fit_pass;
    uint8_t event_count_pass;
    uint8_t polarity_pass;
    uint8_t template_pass;
    uint8_t gain_value_pass;
    uint8_t fit_quality_pass;
    uint8_t numerical_pass;
    uint8_t agreement_pass;
    calibration_tone_fit_result_t tone;
    double existing_normalized_gain;
    double dither_full_gain;
    double dither_flat_gain;
    double requested_dither_correction;
    double tone_amplitude_codes;
    double existing_vs_dither_gain;
    double full_vs_flat_gain;
    double template_energy;
    double template_correlation;
    double fit_rmse;
    double normalized_fit_rmse;
    double peak_residual;
    uint32_t complete_event_count;
    uint32_t discarded_boundary_event_count;
    double mean_event_polarity;
    double separation_denominator;
    uint32_t flat_top_sample_count;
    double fitted_tone_frequency_hz;
    double tone_fit_rmse_codes;
    double tone_only_correlation;
    const char *gain_definition;
} calibration_dither_gain_diagnostic_t;

typedef enum {
    CAL_SKEW_ESTIMATOR_INVALID = 0,
    CAL_SKEW_ESTIMATOR_PASS,
    CAL_SKEW_ESTIMATOR_WARNING
} calibration_skew_estimator_status_t;

typedef enum {
    CAL_SKEW_STAGE_PASS = 0,
    CAL_SKEW_STAGE_PASS_WITH_WARNING,
    CAL_SKEW_STAGE_RUNNING,
    CAL_SKEW_STAGE_SATURATED,
    CAL_SKEW_STAGE_NOT_CONVERGED,
    CAL_SKEW_STAGE_FAIL
} calibration_skew_stage_status_t;

typedef enum {
    CAL_SKEW_REASON_NONE = 0,
    CAL_SKEW_REASON_PREREQUISITE,
    CAL_SKEW_REASON_CAPTURE,
    CAL_SKEW_REASON_CONTEXT,
    CAL_SKEW_REASON_TONE_FIT,
    CAL_SKEW_REASON_DITHER_ALIGNMENT,
    CAL_SKEW_REASON_TOO_FEW_EVENTS,
    CAL_SKEW_REASON_POLARITY_IMBALANCE,
    CAL_SKEW_REASON_TEMPLATE,
    CAL_SKEW_REASON_DERIVATIVE,
    CAL_SKEW_REASON_GAIN,
    CAL_SKEW_REASON_EDGE_DISAGREEMENT,
    CAL_SKEW_REASON_OUTSIDE_LINEAR_RANGE,
    CAL_SKEW_REASON_NUMERICAL,
    CAL_SKEW_REASON_ACTUATOR
} calibration_skew_reject_reason_t;

typedef struct {
    uint8_t valid;
    calibration_skew_estimator_status_t status;
    calibration_skew_reject_reason_t reason;
    calibration_tone_fit_result_t tone;
    double pulse_gain;
    double skew_full_samples;
    double skew_rising_samples;
    double skew_falling_samples;
    double skew_full_ps;
    double skew_rising_ps;
    double skew_falling_ps;
    uint32_t rising_edge_samples;
    uint32_t falling_edge_samples;
} calibration_skew_channel_estimate_t;

typedef struct {
    uint8_t valid;
    uint8_t capture_valid;
    uint8_t paired_channels_valid;
    uint8_t phase_difference_valid;
    uint8_t primary_estimator_valid;
    size_t channel_a_sample_count;
    size_t channel_b_sample_count;
    size_t window_start;
    size_t window_length;
    int8_t canonical_phase;
    double sample_rate_hz;
    double tone_frequency_hz;
    double selected_dac_adc_rate_ratio;
    calibration_skew_estimator_status_t estimator_status;
    calibration_skew_stage_status_t stage_status;
    calibration_skew_reject_reason_t reason;
    calibration_skew_channel_estimate_t channel[2];
    double relative_skew_samples;
    double relative_skew_ps;
    double raw_tone_phase_difference_rad;
    double raw_tone_skew_samples;
    double applied_phase_adjustment_rad;
    double corrected_tone_phase_difference_rad;
    double dither_relative_skew_samples;
    double tone_dither_disagreement_samples;
    uint8_t dither_channel_a_valid;
    uint8_t dither_channel_b_valid;
    uint8_t dither_crosscheck_valid;
    const char *dither_crosscheck_reason;
    const char *channel_a_tone_rejection_reason;
    const char *channel_b_tone_rejection_reason;
    adc_cal_skew_polarity_t selected_polarity;
    adc_cal_skew_branch_reason_t branch_selection_reason;
    double relative_rising_skew_samples;
    double relative_falling_skew_samples;
    double edge_disagreement_samples;
    double edge_disagreement_ps;
    double mean_event_polarity;
    double separation_denominator;
    double pulse_energy;
    double derivative_energy;
    double max_abs_derivative;
    uint32_t complete_event_count;
    uint32_t discarded_boundary_event_count;
    size_t profile_count;
    int m_first;
    int m_last;
    const char *rejection_reason;
} calibration_skew_frame_result_t;

typedef struct {
    calibration_skew_stage_status_t stage_status;
    adc_cal_skew_stage_policy_result_t policy;
    calibration_skew_estimator_status_t estimator_status;
    calibration_skew_reject_reason_t reason;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    uint32_t frames_captured;
    uint32_t valid_paired_channel_frames;
    uint32_t channel_a_tone_fits_valid;
    uint32_t channel_b_tone_fits_valid;
    uint32_t paired_tone_fits_valid;
    uint32_t phase_differences_valid;
    uint32_t polarity_branches_valid;
    uint32_t same_polarity_frames;
    uint32_t inverted_polarity_frames;
    uint32_t polarity_branch_changes;
    uint32_t dither_estimate_available_frames;
    uint32_t dither_crosscheck_valid_frames;
    uint32_t dither_crosscheck_invalid_frames;
    uint32_t consecutive_passes;
    double initial_relative_skew_samples;
    double final_relative_skew_samples;
    double best_relative_skew_samples;
    double final_relative_skew_ps;
    double best_relative_skew_ps;
    double median_relative_skew_samples;
    double median_relative_skew_ps;
    double mean_relative_skew_samples;
    double minimum_relative_skew_samples;
    double maximum_relative_skew_samples;
    double primary_skew_sequence_samples[CAL_SKEW_BATCH_SIZE];
    double relative_skew_std_samples;
    double relative_skew_std_ps;
    double rising_skew_ps;
    double falling_skew_ps;
    double edge_disagreement_ps;
    uint8_t saturated;
    int requested_delay_steps;
    int applied_delay_steps;
    int initial_delay_register;
    int final_delay_register;
    bool closed_loop_enabled;
    adc_cal_skew_loop_result_t loop_result;
    const char *failure_reason;
    const char *correction_reason;
    calibration_skew_frame_result_t latest_frame;
} calibration_skew_batch_result_t;

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
    double dither_raw_second_peak;
    double dither_second_peak;
    double dither_raw_peak_ratio;
    double dither_peak_ratio;
    double dither_align_margin;
    double dither_event_spacing_samples;
    double reference_dither_event_phase_samples;
    double lag_comparison_phase_offset_samples;
    double expected_dither_lag_samples;
    uint8_t tone_cycle_resolved;
    int32_t tone_cycle_offset;
    double tone_cycle_residual_samples;
    double predicted_event_origin_from_existing;
    double detected_event_origin;
    double applied_reference_anchor_offset_samples;
    double applied_resampling_delay_samples;
    double applied_template_anchor_delay_samples;
    double applied_reconstruction_offset_samples;
    double applied_window_coordinate_offset_samples;
    double first_complete_dither_event;
    double last_complete_dither_event;
    uint32_t complete_dither_event_count;
    double dither_derived_lag;
    double alignment_disagreement_samples;
    int32_t alignment_frame_offset;
    int32_t alignment_event_offset;
    double alignment_family_disagreement_samples;
    uint8_t alignment_methods_consistent;
    double origin_lag_closure_samples;
    double first_complete_capture_event;
    double last_complete_capture_event;
    double first_complete_capture_event_unwrapped;
    double last_complete_capture_event_unwrapped;
    uint8_t capture_event_train_crosses_frame;
    int32_t complete_event_frame_offset;
    int32_t complete_event_offset;
    double complete_event_family_span_error;
    uint8_t complete_event_count_consistent;
    int32_t capture_event_frame_offset;
    int32_t capture_event_offset;
    double capture_event_family_span_error;
    uint8_t capture_event_count_consistent;
    uint8_t joint_candidate_pair_valid;
    double joint_candidate_score;
    uint8_t joint_same_polarity;
    uint8_t joint_existing_consistent;
    double joint_channel_a_existing_residual_samples;
    double joint_channel_b_existing_residual_samples;
    uint8_t timing_context_internally_consistent;
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
    uint32_t global_capture_index;
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
    uint8_t reference_rate_adapted;
    size_t reference_tone_bin;
    size_t captured_tone_bin;
    double nominal_dac_adc_rate_ratio;
    double selected_dac_adc_rate_ratio;
    float nominal_rate_correlation;
    float selected_rate_correlation;
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
    uint8_t reference_rate_adapted;
    double nominal_dac_adc_rate_ratio;
    double selected_dac_adc_rate_ratio;
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
    uint32_t dither_pass;
    uint32_t dither_warning;
    uint32_t dither_invalid;
    uint32_t dither_valid_estimates;
    double mean_dither_gain;
    double mean_dither_flat_gain;
    double mean_existing_dither_delta;
    calibration_dither_gain_diagnostic_t dither_latest;
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

typedef adc_cal_perf_spectral_metrics_t adc_performance_spectral_metrics_t;
typedef adc_cal_perf_matching_metrics_t adc_performance_matching_metrics_t;

typedef struct {
    uint32_t frame_number;
    uint32_t global_capture_index;
    uint32_t raw_baseline_global_capture_index;
    uint32_t iteration;
    uint32_t cycles;
    int32_t rotation;
    uint32_t events_used;
    int32_t align_n0;
    float offset_a_codes;
    float offset_b_codes;
    float gain_a_codes;
    float gain_b_codes;
    float gain_ratio;
    float skew_a_ps;
    float skew_b_ps;
    float skew_mismatch_ps;
    float mean_residual;
    float rmse;
    float correlation;
    float cal_a_reference_correlation;
    float cal_b_reference_correlation;
    float cal_a_reference_rmse_codes;
    float cal_b_reference_rmse_codes;
    float rmse_before_polarity;
    float correlation_before_polarity;
    float normalized_gain;
    float raw_reference_mean;
    float scaled_reference_mean;
    float raw_adc_mean;
    float offset_corrected_adc_mean;
    float gain_corrected_adc_mean;
    float reference_fit_s_error_db;
    float sndr_db;
    float sfdr_db;
    float thd_db;
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
    adc_performance_spectral_metrics_t raw_a;
    adc_performance_spectral_metrics_t raw_b;
    adc_performance_spectral_metrics_t cal_a;
    adc_performance_spectral_metrics_t cal_b;
    adc_performance_spectral_metrics_t raw_parallel_average;
    adc_performance_spectral_metrics_t cal_parallel_average;
    adc_performance_matching_metrics_t raw_matching;
    adc_performance_matching_metrics_t cal_matching;
    float raw_cal_b_rms_difference;
    float raw_cal_b_max_abs_difference;
    float raw_cal_a_rms_difference;
    float raw_cal_a_max_abs_difference;
    uintptr_t raw_a_address;
    uintptr_t cal_a_address;
    uintptr_t raw_b_address;
    uintptr_t cal_b_address;
    bool raw_cal_a_identical;
    bool raw_cal_b_identical;
    bool raw_cal_buffers_identical;
    float raw_offset_spur_dbc;
    float raw_image_spur_dbc;
    float cal_offset_spur_dbc;
    float cal_image_spur_dbc;
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
    float cal_parallel_average_sndr_db;
    float sndr_stddev;
    float minimum_sndr_db;
    float cal_parallel_average_sfdr_db;
    float cal_parallel_average_thd_db;
    float cal_parallel_average_enob;
    float enob_stddev;
    float minimum_enob;
    float raw_parallel_average_sndr_db;
    float raw_parallel_average_sfdr_db;
    float raw_parallel_average_thd_db;
    float raw_parallel_average_enob;
    float raw_a_sndr_db;
    float raw_a_sfdr_db;
    float raw_a_thd_db;
    float raw_a_enob;
    float cal_a_sndr_db;
    float cal_a_sfdr_db;
    float cal_a_thd_db;
    float cal_a_enob;
    float raw_b_sndr_db;
    float raw_b_sfdr_db;
    float raw_b_thd_db;
    float raw_b_enob;
    float cal_b_sndr_db;
    float cal_b_sfdr_db;
    float cal_b_thd_db;
    float cal_b_enob;
    float rmse_before_polarity;
    float correlation_before_polarity;
    float raw_cal_b_rms_difference;
    float raw_cal_b_max_abs_difference;
    float raw_cal_a_rms_difference;
    float raw_cal_a_max_abs_difference;
    float raw_image_spur_dbc;
    float cal_image_spur_dbc;
    adc_performance_matching_metrics_t raw_matching;
    adc_performance_matching_metrics_t cal_matching;
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
    double channel_a_polarity;
    double channel_b_polarity;
    double initial_relative_skew_samples;
    double initial_relative_skew_ps;
    double final_relative_skew_samples;
    double final_relative_skew_ps;
    bool raw_cal_buffers_identical;
    bool raw_cal_a_identical;
    bool raw_cal_b_identical;
    bool captures_include_final_skew;
    const char *failure_reason;
    adc_performance_frame_result_t frames[ADC_PERFORMANCE_FRAMES];
} adc_performance_result_t;

typedef struct {
    bool valid;
    uint32_t frames_captured;
    size_t sample_count;
    double sample_rate_hz;
    int8_t canonical_reference_phase;
    size_t window_start;
    size_t window_length;
    double channel_a[ADC_PERFORMANCE_FRAMES][CAL_FIXED_WINDOW_LENGTH];
    double channel_b[ADC_PERFORMANCE_FRAMES][CAL_FIXED_WINDOW_LENGTH];
    uint32_t global_capture_index[ADC_PERFORMANCE_FRAMES];
    const char *failure_reason;
} adc_performance_baseline_t;

typedef enum {
    ADC_CAL_STAGE_IDLE = 0,
    ADC_CAL_STAGE_TIMING,
    ADC_CAL_STAGE_OFFSET,
    ADC_CAL_STAGE_GAIN,
    ADC_CAL_STAGE_SKEW,
    ADC_CAL_STAGE_GAIN_VERIFY,
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
    bool skew_pass;
    bool skew_warning;
    bool skew_correction_applied;
    bool gain_verification_pass;
    bool output_valid;
    bool performance_measurement_available;
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
    double initial_relative_skew_samples;
    double initial_relative_skew_ps;
    double final_relative_skew_samples;
    double final_relative_skew_ps;
    adc_cal_skew_polarity_t final_channel_polarity;
    int final_delay_register;
    adc_cal_skew_stage_policy_result_t skew_policy;
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
static adc_cal_pipeline_state_t g_adc_calibration_pipeline;
static adc_performance_baseline_t g_performance_baseline;
static bool g_quiet_calibration_capture;

void calibration_pending_frame_invalidate(void)
{
    g_automatic_calibration.valid = false;
    g_automatic_calibration.output_valid = false;
    g_automatic_calibration.final_output.valid = false;
    g_automatic_calibration.performance_measurement_available = false;
    adc_cal_pipeline_mark_performance_not_run(&g_adc_calibration_pipeline);
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
    g_performance_baseline.valid = false;
    g_performance_baseline.frames_captured = 0U;
    g_performance_baseline.failure_reason = "baseline not captured";
}

void calibration_gain_input_frame_invalidate(void)
{
    g_automatic_calibration.valid = false;
    g_automatic_calibration.output_valid = false;
    g_automatic_calibration.final_output.valid = false;
    g_automatic_calibration.performance_measurement_available = false;
    adc_cal_pipeline_mark_performance_not_run(&g_adc_calibration_pipeline);
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
static void handle_adc_skew_calibration_cmd(
    bool diagnose_mode, bool closed_loop_requested);
static void handle_adc_skew_preparation_diagnostic(
    adc_cal_skew_prep_diag_mode_t mode);
static void handle_adc_skew_step_cmd(int requested_steps);
static int adc_run_timing_calibration(uint32_t frame_count);
static int adc_run_timing_calibration_conditioned(
    uint32_t frame_count,
    float gain_correction,
    float offset_correction,
    calibration_pending_frame_t *timing_output,
    bool publish_production_state);
static void calibration_automatic_state_reset(void);
static void calibration_automatic_print_command_help(void);
static void calibration_automatic_print_summary(void);
static void handle_adc_calibration_export_cmd(void);
static const char *calibration_existing_offset_loop_status_name(
    const calibration_offset_loop_state_t *state);
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
    int existing_timing_valid,
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
static bool calibration_timing_diagnostics_are_storage_eligible(
    const calibration_timing_diagnostics_t *diagnostics);
static const char *calibration_dither_offset_status_name(
    calibration_dither_offset_status_t status);
static const char *calibration_dither_offset_reason_name(
    calibration_dither_offset_reason_t reason);
static void calibration_print_dither_offset_diagnostic(
    const calibration_dither_offset_diagnostic_t *diagnostic);
static const char *calibration_existing_gain_loop_status_name(
    const calibration_gain_loop_state_t *state);
static const char *calibration_dither_gain_status_name(
    calibration_dither_gain_status_t status);
static const char *calibration_dither_gain_reason_name(
    calibration_dither_gain_reason_t reason);
static void calibration_print_dither_gain_diagnostic(
    const calibration_dither_gain_diagnostic_t *diagnostic);
static const char *calibration_skew_estimator_status_name(
    calibration_skew_estimator_status_t status);
static const char *calibration_skew_reason_name(
    calibration_skew_reject_reason_t reason);
static int calibration_run_skew_open_loop(
    calibration_skew_batch_result_t *batch,
    bool diagnose_mode);
static int calibration_run_skew_stage(
    calibration_skew_batch_result_t *batch,
    bool diagnose_mode,
    bool closed_loop_requested);
static void calibration_print_skew_summary(
    const calibration_skew_batch_result_t *batch,
    bool diagnose_mode);
static int calibration_estimate_skew_frame(
    const double *channel_a,
    const double *channel_b,
    const double *reference,
    const calibration_pending_frame_t *timing_context,
    adc_cal_skew_polarity_t known_polarity,
    int previous_valid,
    double previous_skew_samples,
    calibration_skew_frame_result_t *result,
    double *out_residual_a,
    double *out_residual_b,
    double *out_dither_template);
static int calibration_interpolate_i16(
    const int16_t *samples,
    size_t count,
    double position,
    double *value);
static int calibration_fit_tone_refined(
    const double *samples,
    size_t sample_count,
    double expected_frequency_hz,
    double sample_rate_hz,
    calibration_tone_fit_result_t *fit,
    double *fitted_waveform,
    double *residual);
static bool calibration_tone_fit_parameters_are_finite(
    const calibration_tone_fit_result_t *fit);
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
static void print_double_value(const char *label, double value, const char *unit);

static void print_float_value(const char *label, float value, const char *unit)
{
    print_double_value(label, (double)value, unit);
}

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
    xil_printf("Rate source            : configured value\r\n");
    xil_printf("Measured rate valid    : %s\r\n",
        adc_measured_sample_rate_is_valid() ? "YES" : "NO");
    if (adc_measured_sample_rate_is_valid()) {
        print_double_value("Measured diagnostic rate",
            adc_get_measured_sample_rate_hz() / 1.0e6, " MSPS");
    }
}

static void print_adc_analysis_rate_header(void)
{
    print_double_value("Configured sample rate",
        adc_get_configured_sample_rate_hz() / 1.0e6, " MSPS");
    print_double_value("Configured DAC rate",
        DAC_SAMPLE_RATE_HZ / 1.0e6, " MSPS");
    print_double_value("Analysis sample rate",
        adc_get_effective_sample_rate_hz() / 1.0e6, " MSPS");
    xil_printf("Rate source            : configured value\r\n");
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
    print_double_value("Configured DAC rate",
                       DAC_SAMPLE_RATE_HZ / 1.0e6, " MSPS");
    print_double_value("DAC/ADC rate ratio",
                       DAC_SAMPLE_RATE_HZ / analysis_hz, "");
    if (ADC_CAL_VERBOSE_DEBUG) {
        xil_printf("Rate source            : configured value\r\n");
        if (adc_measured_sample_rate_is_valid()) {
            print_double_value("Measured diagnostic rate",
                adc_get_measured_sample_rate_hz() / 1.0e6, " MSPS");
        }
    }
}

static void print_double_value(
    const char *label,
    double value,
    const char *unit
)
{
    adc_cal_fixed6_parts_t parts;

    if (adc_cal_fixed6_parts(value, &parts) != 0) {
        xil_printf("%-22s: invalid\r\n", label);
        return;
    }

    xil_printf("%-22s: %s", label, parts.negative ? "-" : "");
    if (parts.billions > 0U)
        xil_printf("%lu%09lu", (unsigned long)parts.billions,
                   (unsigned long)parts.units_below_billion);
    else
        xil_printf("%lu", (unsigned long)parts.units_below_billion);
    xil_printf(".%06lu%s\r\n", (unsigned long)parts.millionths,
               unit != NULL ? unit : "");
}

static void print_double_inline(double value)
{
    adc_cal_fixed6_parts_t parts;
    if (adc_cal_fixed6_parts(value, &parts) != 0) {
        xil_printf("invalid");
        return;
    }
    xil_printf("%s", parts.negative ? "-" : "");
    if (parts.billions > 0U)
        xil_printf("%lu%09lu", (unsigned long)parts.billions,
                   (unsigned long)parts.units_below_billion);
    else
        xil_printf("%lu", (unsigned long)parts.units_below_billion);
    xil_printf(".%06lu", (unsigned long)parts.millionths);
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
        double source_position = fmod((double)i + total_lag,
                                      (double)sample_count);
        size_t lower;
        size_t upper;
        double fraction;
        double interpolated;
        long rounded;

        if (source_position < 0.0) source_position += (double)sample_count;

        lower = (size_t)floor(source_position);
        upper = lower + 1U;
        if (upper >= sample_count) upper = 0U;
        fraction = source_position - (double)lower;
        interpolated =
            (1.0 - fraction) * (double)measurement[lower] +
            fraction * (double)measurement[upper];

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
        for (uint32_t i = 0; i < DMA_CMD_BUF_SIZE; i += 16U) {
            xil_printf("@0x%04lX :", (unsigned long)i);
            for (uint32_t byte = 0U;
                 byte < 16U && i + byte < DMA_CMD_BUF_SIZE;
                 ++byte) {
                xil_printf(" %02X", RxBufferPtr[i + byte]);
            }
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
    } else if (strcmp(option, "-burst") == 0) {
        token = strtok(NULL, " ");
        if (!token) { ERR("Missing burst frame count (dma -burst N)"); return; }
        int burst_count = atoi(token);
        if (burst_count <= 0 || burst_count > 100) {
            ERR("Burst frame count must be 1..100.");
            return;
        }
        if (adc_sweep_active)
        {
            ERR("DMA commands are disabled while an ADC sweep is in progress.");
            return;
        }
        xil_printf("Starting %d-frame DMA burst (capture + UDP per frame)...\r\n",
                   burst_count);
        for (int i = 1; i <= burst_count; ++i) {
            xil_printf("Burst frame %d/%d: capturing...\r\n", i, burst_count);
            if (adc_capture_frame() != XST_SUCCESS) {
                xil_printf("Burst frame %d failed; aborting burst.\r\n", i);
                break;
            }
            /*
             * adc_capture_frame() already invalidates the cache after DMA
             * completion. Repeating it here is harmless and ensures
             * udp_send_mem() reads the newest samples from DDR.
             */
            Xil_DCacheInvalidateRange((UINTPTR)RxBufferPtr, DMA_CMD_BUF_SIZE);
            udp_send_mem();
            xil_printf("Burst frame %d/%d transmitted\r\n", i, burst_count);
            /* Small separation so the host receiver can finish storing the
             * current frame before the next one arrives. */
            usleep(100000); /* 100 ms */
        }
        xil_printf("DMA burst complete.\r\n");
    } else { ERR("Invalid option \"%s\" (use -r or -w or -d or -burst)", option); }
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
    g_adc_cal_export_length = 0U;
    g_adc_cal_export_available = false;
    g_adc_cal_export_buffer[0] = '\0';
    memset(&g_adc_cal_export_history, 0,
           sizeof(g_adc_cal_export_history));
    memset(&g_automatic_calibration, 0, sizeof(g_automatic_calibration));
    adc_cal_pipeline_reset(&g_adc_calibration_pipeline);
    g_automatic_calibration.stage = ADC_CAL_STAGE_IDLE;
    g_automatic_calibration.failed_stage = ADC_CAL_STAGE_IDLE;
    g_automatic_calibration.calibration_channel = -1;
    g_automatic_calibration.canonical_reference_phase = -1;
    g_automatic_calibration.gain_correction = 1.0f;
    g_automatic_calibration.final_delay_register = -1;
    g_automatic_calibration.initial_relative_skew_ps = NAN;
    g_automatic_calibration.initial_relative_skew_samples = NAN;
    g_automatic_calibration.final_relative_skew_ps = NAN;
    g_automatic_calibration.final_relative_skew_samples = NAN;
    g_automatic_calibration.final_channel_polarity =
        ADC_CAL_SKEW_POLARITY_UNKNOWN;
    g_automatic_calibration.performance_measurement_available = false;
}

static void calibration_automatic_print_command_help(void)
{
    xil_printf("\r\nADC calibration commands:\r\n");
    xil_printf("  adc -cal                  Run all stages, including closed-loop skew correction.\r\n");
    xil_printf("      Run timing, offset, gain, open-loop skew measurement, and final performance characterization.\r\n");
    xil_printf("\r\nDebug/development stage commands:\r\n");
    xil_printf("  adc -cal timing [frames]  Run timing/reference selection only.\r\n");
    xil_printf("  adc -cal diagnose [frames]\r\n");
    xil_printf("      Run timing tone/dither diagnostics without storing state.\r\n");
    xil_printf("  adc -cal diagnose skewprep MODE\r\n");
    xil_printf("      MODE: jesd|ctrl|analog|digital|analogdigital|enableafter|fullprep|combined|actuator\r\n");
    xil_printf("      Isolate delay-register/JESD preparation; restore state; never apply correction.\r\n");
    xil_printf("  adc -cal offset           Run offset stage only.\r\n");
    xil_printf("  adc -cal gain             Run gain stage only.\r\n");
    xil_printf("  adc -cal skew             Run open-loop Channel B-A skew measurement.\r\n");
    xil_printf("  adc -cal skew open        Explicitly select open-loop measurement.\r\n");
    xil_printf("  adc -cal skew diagnose    Run verbose open-loop skew characterization.\r\n");
    xil_printf("  adc -cal skew closed-loop [diagnose]\r\n");
    xil_printf("      Run characterized closed-loop skew correction with readback.\r\n");
    xil_printf("  adc -cal skew step +/-N   Apply a manual verified actuator step.\r\n");
    xil_printf("  adc -cal stability [frames]\r\n");
    xil_printf("      Characterize fixed-offset capture stability.\r\n");
    xil_printf("  adc -cal status          Display automatic calibration state and latest metrics.\r\n");
    xil_printf("  adc -cal export          Send stored per-stage CSV histories over Ethernet.\r\n");
    xil_printf("  adc -cal reset           Reset software coefficients and calibration loop states.\r\n");
    xil_printf("  adc -cal help            Display this help.\r\n");
}

static void calibration_print_stage_header(
    uint32_t stage_number, const char *stage_name)
{
    xil_printf("\r\nStage %lu/5: %s\r\n",
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
            if (token != NULL && strcmp(token, "skewprep") == 0) {
                adc_cal_skew_prep_diag_mode_t mode;

                token = strtok(NULL, " ");
                if (token == NULL || strtok(NULL, " ") != NULL) {
                    ERR("Use adc -cal diagnose skewprep jesd|ctrl|analog|digital|analogdigital|enableafter|fullprep|combined|actuator.");
                    return;
                }
                if (strcmp(token, "jesd") == 0) {
                    mode = ADC_CAL_SKEW_PREP_DIAG_JESD_ONLY;
                } else if (strcmp(token, "actuator") == 0) {
                    mode = ADC_CAL_SKEW_PREP_DIAG_ACTUATOR_ONLY;
                } else if (strcmp(token, "combined") == 0) {
                    mode = ADC_CAL_SKEW_PREP_DIAG_COMBINED;
                } else if (strcmp(token, "fullprep") == 0) {
                    mode = ADC_CAL_SKEW_PREP_DIAG_COMBINED;
                } else if (strcmp(token, "ctrl") == 0) {
                    mode = ADC_CAL_SKEW_PREP_DIAG_CTRL_ONLY;
                } else if (strcmp(token, "analog") == 0) {
                    mode = ADC_CAL_SKEW_PREP_DIAG_ANALOG_ONLY;
                } else if (strcmp(token, "digital") == 0) {
                    mode = ADC_CAL_SKEW_PREP_DIAG_DIGITAL_ONLY;
                } else if (strcmp(token, "analogdigital") == 0) {
                    mode = ADC_CAL_SKEW_PREP_DIAG_ANALOG_DIGITAL;
                } else if (strcmp(token, "enableafter") == 0) {
                    mode = ADC_CAL_SKEW_PREP_DIAG_ENABLE_AFTER_VALUES;
                } else {
                    ERR("Use adc -cal diagnose skewprep jesd|ctrl|analog|digital|analogdigital|enableafter|fullprep|combined|actuator.");
                    return;
                }
                handle_adc_skew_preparation_diagnostic(mode);
                return;
            }
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
        if (strcmp(token, "export") == 0) {
            if (strtok(NULL, " ") != NULL) {
                ERR("Use adc -cal export.");
                return;
            }
            handle_adc_calibration_export_cmd();
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
        if (strcmp(token, "skew") == 0) {
            token = strtok(NULL, " ");
            if (token == NULL) {
                handle_adc_skew_calibration_cmd(false, false);
                return;
            }
            if (strcmp(token, "diagnose") == 0) {
                if (strtok(NULL, " ") != NULL) {
                    ERR("Use adc -cal skew diagnose.");
                    return;
                }
                handle_adc_skew_calibration_cmd(true, false);
                return;
            }
            if (strcmp(token, "open") == 0 ||
                strcmp(token, "open-loop") == 0) {
                if (strtok(NULL, " ") != NULL) {
                    ERR("Use adc -cal skew open.");
                    return;
                }
                handle_adc_skew_calibration_cmd(false, false);
                return;
            }
            if (strcmp(token, "closed") == 0 ||
                strcmp(token, "close-loop") == 0 ||
                strcmp(token, "closed-loop") == 0) {
                bool diagnose_mode = false;
                token = strtok(NULL, " ");
                if (token != NULL) {
                    if (strcmp(token, "diagnose") != 0 ||
                        strtok(NULL, " ") != NULL) {
                        ERR("Use adc -cal skew closed-loop [diagnose].");
                        return;
                    }
                    diagnose_mode = true;
                }
                handle_adc_skew_calibration_cmd(diagnose_mode, true);
                return;
            }
            if (strcmp(token, "step") == 0) {
                char *endptr = NULL;
                long parsed;
                token = strtok(NULL, " ");
                if (token == NULL) {
                    ERR("Use adc -cal skew step +/-N.");
                    return;
                }
                parsed = strtol(token, &endptr, 0);
                if (endptr == token || *endptr != '\0' ||
                    parsed < INT_MIN || parsed > INT_MAX ||
                    strtok(NULL, " ") != NULL) {
                    ERR("Use adc -cal skew step +/-N.");
                    return;
                }
                handle_adc_skew_step_cmd((int)parsed);
                return;
            }
            ERR("Use adc -cal skew [open|diagnose|closed-loop [diagnose]] or adc -cal skew step +/-N.");
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
            ERR("Invalid calibration frame count. Use %u to %u, or use adc -cal help.",
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
        ERR("Invalid option \"%s\" (use -c, status, -timing [frames], -gain, -offset, -cal [frames|timing|diagnose|gain|offset|skew|stability|status|export|reset|help], -ref, or -ref diagnose)", option);
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
    if (adc_sweep_active)
    {
        /* Keep the raw lwIP receive queue drained during long automatic
         * calibration runs.  A single offset run can spend tens of seconds
         * in this settle delay; leaving it as one blocking sleep exhausts
         * the pbuf pool even though recv_callback() correctly frees pbufs. */
        for (unsigned settle_ms = 0U; settle_ms < 100U; ++settle_ms)
        {
            udp_service_calibration();
            usleep(1000);
        }
    }
    else
    {
        usleep(100000);  /* 100 ms */
    }

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
