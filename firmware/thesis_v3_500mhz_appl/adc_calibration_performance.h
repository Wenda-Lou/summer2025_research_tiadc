#ifndef ADC_CALIBRATION_PERFORMANCE_H
#define ADC_CALIBRATION_PERFORMANCE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ADC_CAL_PERFORMANCE_MAX_SAMPLES
#define ADC_CAL_PERFORMANCE_MAX_SAMPLES 1024U
#endif

#ifndef ADC_CAL_PERFORMANCE_DEFAULT_FRAMES
#define ADC_CAL_PERFORMANCE_DEFAULT_FRAMES 30U
#endif

#ifndef ADC_CAL_PERFORMANCE_MIN_VALID_FRAMES
#define ADC_CAL_PERFORMANCE_MIN_VALID_FRAMES 20U
#endif

typedef struct {
    float sndr_db;
    float sfdr_db;
    float thd_db;
    float enob;
    double signal_hz;
    double worst_spur_hz;
    size_t signal_bin;
    size_t worst_spur_bin;
    double signal_power;
    double noise_distortion_power;
    double spur_power;
    double harmonic_power;
} adc_cal_perf_spectral_metrics_t;

typedef struct {
    bool valid;
    float correlation;
    float waveform_rmse_codes;
    float residual_dbc;
    float offset_mismatch_codes;
    float gain_ratio_b_over_a;
    float gain_mismatch;
    double relative_skew_samples;
    double relative_skew_ps;
} adc_cal_perf_matching_metrics_t;

typedef struct {
    uint32_t frame_number;
    bool valid;
    bool parallel_average_available;
    bool interleaved_metrics_available;
    size_t sample_count;
    double sample_rate_hz;
    double expected_fundamental_hz;
    float mean_residual;
    float rmse;
    float correlation;
    float cal_a_reference_correlation;
    float cal_b_reference_correlation;
    float cal_a_reference_rmse_codes;
    float cal_b_reference_rmse_codes;
    float correlation_before_polarity;
    float rmse_before_polarity;
    float normalized_gain;
    float sndr_db;
    float sfdr_db;
    float thd_db;
    float enob;
    adc_cal_perf_matching_metrics_t raw_matching;
    adc_cal_perf_matching_metrics_t cal_matching;
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
    adc_cal_perf_spectral_metrics_t raw_a;
    adc_cal_perf_spectral_metrics_t raw_b;
    adc_cal_perf_spectral_metrics_t cal_a;
    adc_cal_perf_spectral_metrics_t cal_b;
    adc_cal_perf_spectral_metrics_t raw_parallel_average;
    adc_cal_perf_spectral_metrics_t cal_parallel_average;
    const char *failure_reason;
} adc_cal_perf_frame_result_t;

typedef struct {
    uint32_t frames_attempted;
    uint32_t frames_valid;
    uint32_t frames_rejected;
    bool valid;
    bool spectral_metrics_valid;
    float cal_parallel_average_sndr_db;
    float cal_parallel_average_sfdr_db;
    float cal_parallel_average_thd_db;
    float cal_parallel_average_enob;
    float raw_parallel_average_sndr_db;
    float raw_parallel_average_sfdr_db;
    float raw_parallel_average_thd_db;
    float raw_parallel_average_enob;
    float mean_residual;
    float rmse;
    float correlation;
    const char *failure_reason;
} adc_cal_perf_batch_result_t;

typedef struct {
    size_t sample_count;
    double sample_rate_hz;
    double expected_fundamental_hz;
    uint32_t frame_count;
    uint32_t minimum_valid_frames;
    double final_gain_correction;
    double final_offset_correction;
    double nominal_system_gain;
    int canonical_channel;
    /* Per-channel multipliers that put both physical ADC waveforms into the
     * canonical reference polarity.  cal_a/cal_b passed to analyze_frame are
     * corrected physical samples; polarity is applied exactly once inside
     * the analysis. */
    double channel_polarity[2];
    double initial_relative_skew_samples;
    double initial_relative_skew_ps;
    double final_relative_skew_samples;
    double final_relative_skew_ps;
    const double *baseline_a;
    const double *baseline_b;
    size_t baseline_frame_stride;
    uint32_t baseline_frame_count;
    adc_cal_perf_frame_result_t *frame_results;
    size_t frame_result_capacity;
} adc_cal_perf_config_t;

typedef int (*adc_cal_perf_capture_fn)(
    void *context,
    double *raw_a,
    double *raw_b,
    double *reference,
    size_t capacity,
    size_t *sample_count,
    const char **reason);

void adc_cal_perf_default_config(adc_cal_perf_config_t *config);
void adc_cal_perf_spectral_reset(adc_cal_perf_spectral_metrics_t *metrics);

/* Fit-based SNDR/ENOB for per-capture pipeline tracking: the tone fit
 * isolates the fundamental, so SNDR = 10*log10(amp^2/2 / rmse^2) and
 * ENOB = (SNDR - 1.76)/6.02.  The residual includes harmonics, dither
 * pulses and noise, matching the spectral SNDR definition closely enough
 * for trend tracking.  Returns 0 and fills both outputs, or -1/-2 on
 * invalid arguments / non-finite or non-positive inputs (outputs NAN). */
int adc_cal_perf_sndr_enob_from_tone_fit(
    double amplitude_codes,
    double rmse_codes,
    double *sndr_db,
    double *enob_bits);

int adc_cal_perf_resolve_channel_polarity(
    int canonical_channel,
    double canonical_reference_polarity,
    double relative_b_over_a_polarity,
    double channel_polarity[2]);

int adc_cal_perf_analyze_record(
    const double *samples,
    size_t sample_count,
    double sample_rate_hz,
    adc_cal_perf_spectral_metrics_t *metrics);

int adc_cal_perf_analyze_frame(
    const double *raw_a,
    const double *raw_b,
    const double *cal_a,
    const double *cal_b,
    const double *reference,
    const adc_cal_perf_config_t *config,
    uint32_t frame_number,
    adc_cal_perf_frame_result_t *result);

int adc_cal_perf_run_batch(
    const adc_cal_perf_config_t *config,
    adc_cal_perf_capture_fn capture,
    void *context,
    adc_cal_perf_batch_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* ADC_CALIBRATION_PERFORMANCE_H */
