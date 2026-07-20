/* butils.c
 * Concise, table-driven UART command handler.
 */

#include "butils.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>
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

#define ERR(fmt, ...) xil_printf("Command Error: " fmt "\r\n", ##__VA_ARGS__)

#define CAL_ALIGNMENT_GUARD_SAMPLES 64U
#define CAL_MIN_ANALYSIS_SAMPLES    512U
#define ADC_CAL_DEFAULT_FRAMES              10U
#define ADC_CAL_MIN_FRAMES                  2U
#define ADC_CAL_MAX_FRAMES                  100U
#define CAL_TIMING_MIN_CORRELATION           0.970f
#define CAL_TIMING_MIN_ANALYSIS_SAMPLES      1800U
#define CAL_TIMING_MAX_ABS_FRAC_LAG          0.5f
#define CAL_TIMING_MIN_ACCEPTED_FRAMES       5U
#define CAL_TIMING_MIN_ACCEPTANCE_RATE       0.70f

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
    calibration_timing_reject_reason_t reject_reason;
} calibration_timing_frame_result_t;

int adc_capture_frame(void);
static void adc_ifc_sweep(void);
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
    const double ratio = DAC_SAMPLE_RATE_HZ / ADC_SAMPLE_RATE_HZ;
    const double cycles =
        CAL_EXPECTED_TONE_HZ * (double)sample_count / ADC_SAMPLE_RATE_HZ;
    const double nearest_cycles = round(cycles);
    const double coherence_error = fabs(cycles - nearest_cycles);

    print_double_value("ADC sample rate",
                       ADC_SAMPLE_RATE_HZ / 1.0e6, " MHz");
    print_double_value("DAC sample rate",
                       DAC_SAMPLE_RATE_HZ / 1.0e6, " MHz");
    print_double_value("DAC/ADC rate ratio", ratio, "");
    xil_printf("Capture samples       : %lu\r\n",
               (unsigned long)sample_count);
    xil_printf("Expected tone bin     : %u\r\n",
               (unsigned int)CAL_EXPECTED_TONE_BIN);
    print_double_value("Expected tone freq",
                       CAL_EXPECTED_TONE_HZ / 1.0e6, " MHz");
    print_double_value("Expected cycles/frame", cycles, "");
    print_double_value("Cycles per ADC frame", cycles, "");
    print_double_value("Coherence error", coherence_error, " cycles");

    if ((sample_count == ADC_TEST_CAPTURE_SAMPLES) &&
        isfinite(coherence_error) &&
        (coherence_error <= CAL_COHERENCE_TOLERANCE)) {
        xil_printf("Coherence status      : PASS\r\n");
    } else {
        xil_printf("Coherence status      : WARNING\r\n");
        xil_printf("WARNING: The waveform is not coherent with the "
                   "%lu-sample ADC window.\r\n",
                   (unsigned long)sample_count);
        xil_printf("Gain, offset, and mean estimates may vary with "
                   "capture phase.\r\n");
    }
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
        ((double)(crossing_count - 1U) * ADC_SAMPLE_RATE_HZ) /
        (last_crossing - first_crossing);
    *tone_bin = *frequency_hz * (double)sample_count / ADC_SAMPLE_RATE_HZ;

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
    double difference_hz;
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

    difference_hz = measured_hz - CAL_EXPECTED_TONE_HZ;
    measured_coherence_error = fabs(measured_bin - round(measured_bin));
    print_double_value("Measured tone bin", measured_bin, "");
    print_double_value("Measured tone freq", measured_hz / 1.0e6, " MHz");
    print_double_value("Tone frequency error",
                       difference_hz / 1.0e3, " kHz");
    print_double_value("Measured coherence err",
                       measured_coherence_error, " cycles");
    xil_printf("Frequency validation  : %s\r\n",
               (fabs(difference_hz) <= CAL_FREQUENCY_TOLERANCE_HZ) ?
               "PASS" : "WARNING");
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
    if (!t) return 0;
    strncpy(out, t, len - 1);
    out[len - 1] = '\0';
    return 1;
}

static void parse_cmd_args(char *line, char *option, size_t opt_len, char *addr_str, size_t addr_len, char *data_str, size_t data_len, const char *cmd_name) {
    char *ctx = line;
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
    uint32_t addr = RxBufferPtr + offset;

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

        do {
            usleep(10000);
            ad9695_jesd_get_pll_status(&pll_stat);
        } while (!(pll_stat & AD9695_JESD_PLL_LOCK_STAT) && timeout--);

        xil_printf("ad9695 PLL %s\r\n", (pll_stat & AD9695_JESD_PLL_LOCK_STAT) ? "LOCKED" : "UNLOCKED");
        jesdphy_check_pll_status(&pll_stat);
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
        handle_adc_reference_status_cmd();
    }else {
        ERR("Invalid option \"%s\" (use -c, status, -timing [frames], -gain, -offset, -cal [frames], or -ref)", option);
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
    xil_printf("  DDC   Digital downconverter gain mode, not for full-bandwidth capture\r\n");
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
            xil_printf("  DDC   Digital downconverter gain mode, not used for current full-bandwidth capture\r\n");
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

        else if (strcmp(token, "DDC") == 0 || strcmp(token, "ddc") == 0)
        {
            xil_printf("\r\nGain setting DDC mode\r\n");
            xil_printf("DDC gain controls digital downconverter gain.\r\n");
            xil_printf("It is not used for the current full-bandwidth ADC capture path.\r\n");
            xil_printf("Use IFC mode for now.\r\n");
        }

        else
        {
            xil_printf("Invalid gain command. Use IFC, DDC, back, or help.\r\n");
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
     * Frame 1 becomes the fixed reference, exactly as in receive_data.py.
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
     * Analyze frame 1 against itself so the output matches the Python
     * timing summary.
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
         * Small separation between complete frames so the Python
         * receiver can finish storing the current frame.
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

void handle_adc_reference_status_cmd(void)
{
    const int16_t *reference;
    size_t length;
    size_t expected_length;

    xil_printf("\r\n========== Reference Buffer ==========\r\n");

    xil_printf(
        "Reference ready   : %s\r\n",
        reference_buffer_is_ready() ? "YES" : "NO"
    );

    length = reference_buffer_length();
    expected_length = reference_buffer_expected_length();

    xil_printf(
        "Reference length  : %lu\r\n",
        (unsigned long)length
    );

    xil_printf(
        "Expected length   : %lu\r\n",
        (unsigned long)expected_length
    );

    if (!reference_buffer_is_ready() || length == 0U)
    {
        xil_printf("No reference currently loaded.\r\n");
        return;
    }

    reference = reference_buffer_data();

    xil_printf(
        "First sample      : %d\r\n",
        (int)reference[0]
    );

    if (length > 1U)
    {
        xil_printf(
            "Second sample     : %d\r\n",
            (int)reference[1]
        );
    }

    xil_printf(
        "Middle sample     : %d\r\n",
        (int)reference[length / 2U]
    );

    xil_printf(
        "Last sample       : %d\r\n",
        (int)reference[length - 1U]
    );
}

void handle_adc_calibration_cmd(uint32_t frame_count)
{
    static int16_t captured_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t reconstructed_samples[ADC_VALID_SAMPLE_COUNT];
    static int16_t fractional_reference[ADC_VALID_SAMPLE_COUNT];
    static int16_t fractional_measurement[ADC_VALID_SAMPLE_COUNT];
    static float accepted_correlations[ADC_CAL_MAX_FRAMES - 1U];
    static float accepted_rmse[ADC_CAL_MAX_FRAMES - 1U];

    size_t reference_count = 0U;
    size_t reconstructed_count = 0U;
    uint32_t successful_captures = 0U;
    uint32_t accepted_frames = 0U;
    double sum_correlation = 0.0;
    int32_t min_integer_lag = 0;
    int32_t max_integer_lag = 0;
    float min_fractional_lag = 0.0f;
    float max_fractional_lag = 0.0f;

    int reconstruction_status;

    if ((frame_count < ADC_CAL_MIN_FRAMES) ||
        (frame_count > ADC_CAL_MAX_FRAMES)) {
        ERR("Calibration frame count must be between %u and %u.",
            ADC_CAL_MIN_FRAMES, ADC_CAL_MAX_FRAMES);
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
    xil_printf("ADC Timing-Only Calibration Validation\r\n");
    xil_printf("Reference : first ADC capture\r\n");
    xil_printf("Measurements: frames 2 through %u\r\n",
               (unsigned int)frame_count);
    xil_printf("===================================\r\n");
    print_adc_test_configuration(ADC_VALID_SAMPLE_COUNT);

    /*
     * 1. Capture and reconstruct frame 1 as the fixed reference, matching the
     * tested adc -timing acquisition method.
     */
    xil_printf("\r\nCapturing reference frame (1/%u).\r\n",
               (unsigned int)frame_count);
    if (adc_capture_frame() != XST_SUCCESS)
    {
        xil_printf("Calibration reference capture failed.\r\n");
        goto calibration_done;
    }

    reconstruction_status = adc_reconstruct_frame(
        RxBufferPtr,
        DMA_CMD_BUF_SIZE,
        captured_reference,
        ADC_VALID_SAMPLE_COUNT,
        &reference_count
    );

    if (reconstruction_status != 0)
    {
        xil_printf(
            "Calibration reference reconstruction failed: %d\r\n",
            reconstruction_status
        );
        goto calibration_done;
    }

    xil_printf(
        "Reference samples reconstructed: %lu\r\n",
        (unsigned long)reference_count
    );
    if (reference_count != ADC_TEST_CAPTURE_SAMPLES)
    {
        xil_printf(
            "Calibration reference rejected: expected %u samples, got %lu.\r\n",
            (unsigned int)ADC_TEST_CAPTURE_SAMPLES,
            (unsigned long)reference_count
        );
        goto calibration_done;
    }

    for (uint32_t frame = 2U; frame <= frame_count; ++frame)
    {
        calibration_timing_frame_result_t result;
        int measurement_status;

        memset(&result, 0, sizeof(result));

        xil_printf("\r\n[TIMING_FRAME %lu/%lu]\r\n",
                   (unsigned long)frame,
                   (unsigned long)frame_count);

        if (adc_capture_frame() != XST_SUCCESS) {
            result.reject_reason = CAL_TIMING_REJECT_DMA;
            goto print_frame_result;
        }
        result.capture_success = 1U;
        ++successful_captures;

        reconstruction_status = adc_reconstruct_frame(
            RxBufferPtr, DMA_CMD_BUF_SIZE, reconstructed_samples,
            ADC_VALID_SAMPLE_COUNT, &reconstructed_count
        );
        if ((reconstruction_status != 0) ||
            (reconstructed_count != reference_count)) {
            result.reject_reason = CAL_TIMING_REJECT_RECONSTRUCTION;
            goto print_frame_result;
        }

        measurement_status = adc_measure_timing_frame(
            captured_reference,
            reconstructed_samples,
            reconstructed_count,
            fractional_reference,
            fractional_measurement,
            &result
        );
        (void)measurement_status;

        if (result.accepted)
        {
            const uint32_t index = accepted_frames;
            accepted_correlations[index] = result.correlation;
            accepted_rmse[index] = result.raw_rmse;
            sum_correlation += (double)result.correlation;

            if (accepted_frames == 0U) {
                min_integer_lag = result.integer_lag;
                max_integer_lag = result.integer_lag;
                min_fractional_lag = result.fractional_lag;
                max_fractional_lag = result.fractional_lag;
            } else {
                if (result.integer_lag < min_integer_lag)
                    min_integer_lag = result.integer_lag;
                if (result.integer_lag > max_integer_lag)
                    max_integer_lag = result.integer_lag;
                if (result.fractional_lag < min_fractional_lag)
                    min_fractional_lag = result.fractional_lag;
                if (result.fractional_lag > max_fractional_lag)
                    max_fractional_lag = result.fractional_lag;
            }
            ++accepted_frames;
        }

print_frame_result:
        xil_printf("Integer lag          : %ld samples\r\n",
                   (long)result.integer_lag);
        print_float_value("Fractional lag", result.fractional_lag, " samples");
        print_float_value("Total estimated lag", result.total_lag, " samples");
        xil_printf("Valid overlap        : %lu samples\r\n",
                   (unsigned long)result.valid_overlap_samples);
        xil_printf("Analysis samples     : %lu\r\n",
                   (unsigned long)result.analysis_samples);
        print_float_value("Correlation", result.correlation, "");
        print_float_value("Raw aligned RMSE", result.raw_rmse, " codes");
        xil_printf("Timing result        : %s\r\n",
                   result.accepted ? "PASS" : "REJECT");
        if (!result.accepted) {
            xil_printf("Reject reason        : %s\r\n",
                       cal_timing_reject_reason_text(result.reject_reason));
        }

        if (frame < frame_count) {
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
        }
    }

    {
        const uint32_t measurement_frames = frame_count - 1U;
        const uint32_t rejected_frames = measurement_frames - accepted_frames;
        const float acceptance_rate =
            (float)accepted_frames / (float)measurement_frames;
        const uint32_t required_accepted =
            (measurement_frames < CAL_TIMING_MIN_ACCEPTED_FRAMES) ?
            measurement_frames : CAL_TIMING_MIN_ACCEPTED_FRAMES;
        const int overall_pass =
            (accepted_frames >= required_accepted) &&
            (acceptance_rate >= CAL_TIMING_MIN_ACCEPTANCE_RATE);

        xil_printf("\r\n========== Timing Alignment Summary ==========\r\n");
        xil_printf("Requested frames       : %lu\r\n",
                   (unsigned long)frame_count);
        xil_printf("Reference frames        : 1\r\n");
        xil_printf("Measurement frames      : %lu\r\n",
                   (unsigned long)measurement_frames);
        xil_printf("Successful captures     : %lu\r\n",
                   (unsigned long)successful_captures);
        xil_printf("Accepted frames         : %lu\r\n",
                   (unsigned long)accepted_frames);
        xil_printf("Rejected frames         : %lu\r\n",
                   (unsigned long)rejected_frames);
        print_float_value("Acceptance rate", acceptance_rate * 100.0f, " %");
        print_float_value("Correlation threshold",
                          CAL_TIMING_MIN_CORRELATION, "");
        xil_printf("Minimum analysis samples: %u\r\n",
                   (unsigned int)CAL_TIMING_MIN_ANALYSIS_SAMPLES);

        if (accepted_frames > 0U)
        {
            float min_corr = accepted_correlations[0];
            float min_rmse = accepted_rmse[0];
            float max_rmse = accepted_rmse[0];
            for (uint32_t i = 1U; i < accepted_frames; ++i) {
                if (accepted_correlations[i] < min_corr)
                    min_corr = accepted_correlations[i];
                if (accepted_rmse[i] < min_rmse)
                    min_rmse = accepted_rmse[i];
                if (accepted_rmse[i] > max_rmse)
                    max_rmse = accepted_rmse[i];
            }
            print_double_value("Mean correlation",
                               sum_correlation / (double)accepted_frames, "");
            print_float_value("Median correlation",
                              median_float(accepted_correlations,
                                           accepted_frames), "");
            print_float_value("Minimum accepted corr", min_corr, "");
            print_float_value("Median raw RMSE",
                              median_float(accepted_rmse, accepted_frames),
                              " codes");
            print_float_value("Minimum raw RMSE", min_rmse, " codes");
            print_float_value("Maximum raw RMSE", max_rmse, " codes");
            xil_printf("Integer lag range      : %ld to %ld samples\r\n",
                       (long)min_integer_lag, (long)max_integer_lag);
            print_float_value("Minimum fractional lag",
                              min_fractional_lag, " samples");
            print_float_value("Maximum fractional lag",
                              max_fractional_lag, " samples");
        }
        else
        {
            xil_printf("No accepted frames available for statistics.\r\n");
        }
        xil_printf("Timing status          : %s\r\n",
                   overall_pass ? "PASS" : "FAIL");
        xil_printf("==============================================\r\n");
    }

calibration_done:
    xil_printf("No correction coefficients were updated.\r\n");
    adc_sweep_active = 0;
}
