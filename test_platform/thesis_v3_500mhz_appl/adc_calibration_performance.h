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
    uint32_t frame_number;
    bool valid;
    size_t sample_count;
    double sample_rate_hz;
    double expected_fundamental_hz;
    float mean_residual;
    float rmse;
    float correlation;
    float normalized_gain;
    float sndr_db;
    float sfdr_db;
    float thd_db;
    float enob;
    float raw_difference_dbc;
    float cal_difference_dbc;
    float cal_dc_difference_codes;
    adc_cal_perf_spectral_metrics_t raw_a;
    adc_cal_perf_spectral_metrics_t raw_b;
    adc_cal_perf_spectral_metrics_t cal_a;
    adc_cal_perf_spectral_metrics_t cal_b;
    adc_cal_perf_spectral_metrics_t raw_combined;
    adc_cal_perf_spectral_metrics_t cal_combined;
    const char *failure_reason;
} adc_cal_perf_frame_result_t;

typedef struct {
    uint32_t frames_attempted;
    uint32_t frames_valid;
    uint32_t frames_rejected;
    bool valid;
    bool spectral_metrics_valid;
    float sndr_db;
    float sfdr_db;
    float thd_db;
    float enob;
    float raw_sndr_db;
    float raw_sfdr_db;
    float raw_thd_db;
    float raw_enob;
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
    bool combined_uses_channel_a;
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
