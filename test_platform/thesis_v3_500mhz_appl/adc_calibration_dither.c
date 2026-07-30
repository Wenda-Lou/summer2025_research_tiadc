#include "adc_calibration_dither.h"

#include <float.h>
#include <math.h>
#include <string.h>

static int adc_cal_double_isfinite(double value)
{
    return value == value && value <= DBL_MAX && value >= -DBL_MAX;
}

static double dither_abs(double value)
{
    return value < 0.0 ? -value : value;
}

void adc_cal_dither_default_config(adc_cal_dither_config_t *config)
{
    if (config == NULL) return;
    config->threshold_fraction = ADC_CAL_DITHER_DEFAULT_THRESHOLD_FRACTION;
    config->minimum_events = ADC_CAL_DITHER_DEFAULT_MIN_EVENTS;
    config->boundary_margin = 1U;
}

const char *adc_cal_dither_status_name(adc_cal_dither_status_t status)
{
    switch (status) {
    case ADC_CAL_DITHER_OK: return "PASS";
    case ADC_CAL_DITHER_ERR_NULL: return "NULL_INPUT";
    case ADC_CAL_DITHER_ERR_SAMPLE_COUNT: return "SAMPLE_COUNT";
    case ADC_CAL_DITHER_ERR_NO_ENERGY: return "NO_ENERGY";
    case ADC_CAL_DITHER_ERR_NO_EVENTS: return "NO_EVENTS";
    case ADC_CAL_DITHER_ERR_TOO_FEW_EVENTS: return "TOO_FEW_EVENTS";
    case ADC_CAL_DITHER_ERR_POLARITY: return "POLARITY";
    case ADC_CAL_DITHER_ERR_NUMERICAL: return "NUMERICAL";
    default: return "UNKNOWN";
    }
}

void adc_cal_dither_result_reset(adc_cal_dither_result_t *result)
{
    if (result == NULL) return;
    memset(result, 0, sizeof(*result));
    result->valid = 0;
    result->status = ADC_CAL_DITHER_ERR_NUMERICAL;
    result->mean_polarity = NAN;
    result->separation_denominator = NAN;
    result->template_projection = NAN;
    result->template_energy = NAN;
    result->derivative_projection = NAN;
    result->derivative_energy = NAN;
    result->normalized_projection = NAN;
    result->quality = NAN;
    result->peak_abs_template = NAN;
}

int adc_cal_dither_interpolate(
    const double *samples,
    size_t count,
    double position,
    double *value)
{
    size_t lower;
    double fraction;
    if (samples == NULL || value == NULL || count == 0U ||
        !adc_cal_double_isfinite(position)) {
        return -1;
    }
    if (position < 0.0 || position > (double)(count - 1U)) {
        return -2;
    }
    lower = (size_t)floor(position);
    if (lower + 1U >= count) {
        *value = samples[count - 1U];
        return adc_cal_double_isfinite(*value) ? 0 : -3;
    }
    fraction = position - (double)lower;
    *value = (1.0 - fraction) * samples[lower] +
             fraction * samples[lower + 1U];
    return adc_cal_double_isfinite(*value) ? 0 : -3;
}

int adc_cal_dither_find_events(
    const double *template_samples,
    size_t sample_count,
    const adc_cal_dither_config_t *config,
    adc_cal_dither_result_t *result)
{
    adc_cal_dither_config_t local_config;
    double peak = 0.0;
    double threshold;
    size_t index = 0U;

    if (result == NULL) return ADC_CAL_DITHER_ERR_NULL;
    adc_cal_dither_result_reset(result);
    if (template_samples == NULL) {
        result->status = ADC_CAL_DITHER_ERR_NULL;
        return result->status;
    }
    if (sample_count == 0U || sample_count > ADC_CAL_DITHER_MAX_EVENTS * 4U) {
        result->status = ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
        return result->status;
    }
    if (config == NULL) {
        adc_cal_dither_default_config(&local_config);
        config = &local_config;
    }
    result->mean_polarity = 0.0;
    if (!adc_cal_double_isfinite(config->threshold_fraction) ||
        config->threshold_fraction <= 0.0 ||
        config->threshold_fraction >= 1.0) {
        result->status = ADC_CAL_DITHER_ERR_NUMERICAL;
        return result->status;
    }
    for (size_t i = 0U; i < sample_count; ++i) {
        if (!adc_cal_double_isfinite(template_samples[i])) {
            result->status = ADC_CAL_DITHER_ERR_NUMERICAL;
            return result->status;
        }
        if (dither_abs(template_samples[i]) > peak) {
            peak = dither_abs(template_samples[i]);
        }
    }
    result->peak_abs_template = peak;
    if (peak <= DBL_EPSILON) {
        result->status = ADC_CAL_DITHER_ERR_NO_ENERGY;
        return result->status;
    }
    threshold = peak * config->threshold_fraction;
    while (index < sample_count) {
        size_t start;
        size_t end;
        double weighted = 0.0;
        double weight = 0.0;
        double signed_sum = 0.0;
        if (dither_abs(template_samples[index]) < threshold) {
            ++index;
            continue;
        }
        start = index;
        while (index < sample_count &&
               dither_abs(template_samples[index]) >= threshold) {
            const double a = dither_abs(template_samples[index]);
            weighted += (double)index * a;
            weight += a;
            signed_sum += template_samples[index];
            ++index;
        }
        end = index;
        ++result->detected_events;
        if (start < config->boundary_margin ||
            end + config->boundary_margin > sample_count ||
            result->accepted_events >= ADC_CAL_DITHER_MAX_EVENTS) {
            ++result->rejected_events;
            continue;
        }
        result->events[result->accepted_events].start = start;
        result->events[result->accepted_events].end = end;
        result->events[result->accepted_events].center =
            weight > DBL_EPSILON ? weighted / weight :
            0.5 * ((double)start + (double)(end - 1U));
        result->events[result->accepted_events].polarity =
            signed_sum >= 0.0 ? 1.0 : -1.0;
        result->mean_polarity +=
            result->events[result->accepted_events].polarity;
        ++result->accepted_events;
    }
    if (result->detected_events == 0U) {
        result->status = ADC_CAL_DITHER_ERR_NO_EVENTS;
        return result->status;
    }
    if (result->accepted_events < config->minimum_events) {
        result->status = ADC_CAL_DITHER_ERR_TOO_FEW_EVENTS;
        return result->status;
    }
    result->mean_polarity /= (double)result->accepted_events;
    result->separation_denominator =
        1.0 - result->mean_polarity * result->mean_polarity;
    if (!adc_cal_double_isfinite(result->separation_denominator) ||
        result->separation_denominator < ADC_CAL_DITHER_DENOMINATOR_FLOOR) {
        result->status = ADC_CAL_DITHER_ERR_POLARITY;
        return result->status;
    }
    result->status = ADC_CAL_DITHER_OK;
    return 0;
}

int adc_cal_dither_analyze(
    const double *samples,
    const double *template_samples,
    size_t sample_count,
    const adc_cal_dither_config_t *config,
    adc_cal_dither_result_t *result)
{
    double sample_mean = 0.0;
    double template_mean = 0.0;
    double sample_energy = 0.0;
    int status;

    if (result == NULL) return ADC_CAL_DITHER_ERR_NULL;
    status = adc_cal_dither_find_events(
        template_samples, sample_count, config, result);
    if (status != 0) return status;
    if (samples == NULL) {
        result->status = ADC_CAL_DITHER_ERR_NULL;
        return result->status;
    }
    result->template_projection = 0.0;
    result->template_energy = 0.0;
    result->derivative_projection = 0.0;
    result->derivative_energy = 0.0;
    result->normalized_projection = NAN;
    result->quality = NAN;
    for (size_t i = 0U; i < sample_count; ++i) {
        if (!adc_cal_double_isfinite(samples[i]) || !adc_cal_double_isfinite(template_samples[i])) {
            result->status = ADC_CAL_DITHER_ERR_NUMERICAL;
            return result->status;
        }
        sample_mean += samples[i];
        template_mean += template_samples[i];
    }
    sample_mean /= (double)sample_count;
    template_mean /= (double)sample_count;
    for (size_t i = 0U; i < sample_count; ++i) {
        const double s = samples[i] - sample_mean;
        const double t = template_samples[i] - template_mean;
        const double previous = i > 0U ? template_samples[i - 1U] :
            template_samples[i];
        const double next = i + 1U < sample_count ? template_samples[i + 1U] :
            template_samples[i];
        const double derivative = 0.5 * (next - previous);
        result->template_projection += s * t;
        result->template_energy += t * t;
        result->derivative_projection += s * derivative;
        result->derivative_energy += derivative * derivative;
        sample_energy += s * s;
    }
    if (result->template_energy <= DBL_EPSILON ||
        sample_energy <= DBL_EPSILON) {
        result->status = ADC_CAL_DITHER_ERR_NO_ENERGY;
        return result->status;
    }
    result->normalized_projection =
        result->template_projection / result->template_energy;
    result->quality = result->template_projection /
        sqrt(result->template_energy * sample_energy);
    result->valid = adc_cal_double_isfinite(result->normalized_projection) &&
        adc_cal_double_isfinite(result->quality);
    result->status = result->valid ? ADC_CAL_DITHER_OK :
        ADC_CAL_DITHER_ERR_NUMERICAL;
    return result->valid ? 0 : result->status;
}
