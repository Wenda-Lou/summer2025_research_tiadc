/* Offline replay of a saved corrected snapshot; production C cancellation.
 * Usage: adc_signal_replay INPUT.csv OUTPUT.csv SAMPLE_RATE_HZ
 * Creates OUTPUT.csv/json exclusively. Never opens the bench or calibrates. */
#include "adc_calibration_loop.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static adc_cal_loop_t loop;
static double a[ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES], b[ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES];
static int replay_finite(double v) { return v == v && v >= -DBL_MAX && v <= DBL_MAX; }

int main(int argc, char **argv)
{
    if (argc != 4) {
        fprintf(stderr, "Usage: %s INPUT.csv OUTPUT.csv SAMPLE_RATE_HZ\n", argv[0]);
        return 2;
    }
    char *end = NULL;
    const double rate = strtod(argv[3], &end);
    if (end == argv[3] || *end || !replay_finite(rate) || rate <= 0.0) return 2;
    char sidecar[4096];
    const size_t length = strlen(argv[2]);
    if (length < 4 || length + 2 >= sizeof(sidecar) || strcmp(argv[2]+length-4, ".csv") != 0)
        return 2;
    memcpy(sidecar, argv[2], length-4);
    strcpy(sidecar+length-4, ".json");
    FILE *input = fopen(argv[1], "r");
    if (!input) { perror(argv[1]); return 2; }
    char row[1024];
    if (!fgets(row, sizeof(row), input)) { fclose(input); return 2; }
    row[strcspn(row, "\r\n")] = '\0';
    if (strcmp(row, "sample,time_s,output_a_codes,output_b_codes") != 0) {
        fclose(input); return 2;
    }
    size_t count = 0;
    while (fgets(row, sizeof(row), input)) {
        unsigned index;
        double time, ca, cb;
        int used = 0;
        if (count == ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES ||
            sscanf(row, "%u,%lf,%lf,%lf %n", &index, &time, &ca, &cb, &used) != 4 ||
            row[used] != '\0' || index != count || !replay_finite(time) ||
            fabs(time-(double)index/rate) > 1e-12 || !replay_finite(ca) || !replay_finite(cb)) {
            fclose(input); fprintf(stderr, "Invalid snapshot row\n"); return 2;
        }
        a[count] = ca; b[count] = cb; count++;
    }
    const int read_error = ferror(input);
    fclose(input);
    if (read_error || count < 512) return 2;
    loop.cfg.fs_dac = rate; loop.cfg.adc_ratio = 1;
    if (adc_cal_loop_cancel_dither(&loop, a, b, count) != 0) return 2;
    FILE *output = fopen(argv[2], "wx");
    if (!output) { perror(argv[2]); return 2; }
    FILE *meta = fopen(sidecar, "wx");
    if (!meta) {
        perror(sidecar); fclose(output); remove(argv[2]); return 2;
    }
    fprintf(output, "sample,time_s,output_a_codes,output_b_codes\n");
    for (size_t i = 0; i < count; ++i)
        fprintf(output, "%zu,%.17g,%.17g,%.17g\n", i, (double)i/rate,
                loop.output_a[i], loop.output_b[i]);
    const int cancelled = loop.output.status == ADC_CAL_LOOP_OUTPUT_CANCELLED;
    fprintf(meta, "{\n  \"version\": \"1\", \"samples\": %zu,\n"
            "  \"sample_rate_hz\": %.17g, \"coefficients\": \"frozen\",\n"
            "  \"status\": \"%s\", \"dither_cancelled\": %s,\n"
            "  \"source\": \"offline_production_c_replay\", \"output_revision\": %u,\n"
            "  \"reason\": \"%s\", \"core_samples\": %u, \"support_samples\": %u,\n"
            "  \"tone_hz\": %.17g, \"period_samples\": %u, \"events\": %u,\n"
            "  \"validation_db_a\": %.17g, \"validation_db_b\": %.17g\n}\n",
            count, rate, adc_cal_loop_output_status_name(loop.output.status),
            cancelled ? "true" : "false", (unsigned)ADC_CAL_LOOP_OUTPUT_REVISION,
            loop.output.reason ? loop.output.reason : "unspecified",
            loop.output.pulse_core_samples, loop.output.pulse_support_samples,
            loop.output.tone_hz, loop.output.period_samples,
            loop.output.events, loop.output.validation_db_a, loop.output.validation_db_b);
    const int csv_error = ferror(output), json_error = ferror(meta);
    const int csv_close = fclose(output), json_close = fclose(meta);
    if (csv_error || json_error || csv_close || json_close) {
        remove(argv[2]); remove(sidecar); return 2;
    }
    printf("status=%s samples=%zu period=%u events=%u validation_db=%.3f/%.3f\n",
           adc_cal_loop_output_status_name(loop.output.status), count, loop.output.period_samples,
           loop.output.events, loop.output.validation_db_a, loop.output.validation_db_b);
    return cancelled ? 0 : 1;
}
