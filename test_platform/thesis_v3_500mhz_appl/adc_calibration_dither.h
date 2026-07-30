#ifndef ADC_CALIBRATION_DITHER_H
#define ADC_CALIBRATION_DITHER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ADC_CAL_DITHER_MAX_EVENTS
#define ADC_CAL_DITHER_MAX_EVENTS 512U
#endif

#ifndef ADC_CAL_DITHER_DEFAULT_THRESHOLD_FRACTION
#define ADC_CAL_DITHER_DEFAULT_THRESHOLD_FRACTION 0.25
#endif

#ifndef ADC_CAL_DITHER_DEFAULT_MIN_EVENTS
#define ADC_CAL_DITHER_DEFAULT_MIN_EVENTS 2U
#endif

#ifndef ADC_CAL_DITHER_DENOMINATOR_FLOOR
#define ADC_CAL_DITHER_DENOMINATOR_FLOOR 0.05
#endif

typedef enum {
    ADC_CAL_DITHER_OK = 0,
    ADC_CAL_DITHER_ERR_NULL = -1,
    ADC_CAL_DITHER_ERR_SAMPLE_COUNT = -2,
    ADC_CAL_DITHER_ERR_NO_ENERGY = -3,
    ADC_CAL_DITHER_ERR_NO_EVENTS = -4,
    ADC_CAL_DITHER_ERR_TOO_FEW_EVENTS = -5,
    ADC_CAL_DITHER_ERR_POLARITY = -6,
    ADC_CAL_DITHER_ERR_NUMERICAL = -7
} adc_cal_dither_status_t;

typedef struct {
    size_t start;
    size_t end;
    double center;
    double polarity;
} adc_cal_dither_event_t;

typedef struct {
    double threshold_fraction;
    size_t minimum_events;
    size_t boundary_margin;
} adc_cal_dither_config_t;

typedef struct {
    int valid;
    adc_cal_dither_status_t status;
    size_t detected_events;
    size_t accepted_events;
    size_t rejected_events;
    double mean_polarity;
    double separation_denominator;
    double template_projection;
    double template_energy;
    double derivative_projection;
    double derivative_energy;
    double normalized_projection;
    double quality;
    double peak_abs_template;
    adc_cal_dither_event_t events[ADC_CAL_DITHER_MAX_EVENTS];
} adc_cal_dither_result_t;

void adc_cal_dither_default_config(adc_cal_dither_config_t *config);
const char *adc_cal_dither_status_name(adc_cal_dither_status_t status);
void adc_cal_dither_result_reset(adc_cal_dither_result_t *result);

int adc_cal_dither_find_events(
    const double *template_samples,
    size_t sample_count,
    const adc_cal_dither_config_t *config,
    adc_cal_dither_result_t *result);

int adc_cal_dither_analyze(
    const double *samples,
    const double *template_samples,
    size_t sample_count,
    const adc_cal_dither_config_t *config,
    adc_cal_dither_result_t *result);

int adc_cal_dither_interpolate(
    const double *samples,
    size_t count,
    double position,
    double *value);

#ifdef __cplusplus
}
#endif

#endif /* ADC_CALIBRATION_DITHER_H */
