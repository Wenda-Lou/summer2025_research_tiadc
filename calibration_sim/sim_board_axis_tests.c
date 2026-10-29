/* Real board experiment fragment, SPI/UART/DMA spies. The production
 * estimator and feedback physics are tested separately in sim_loop_tests.c. */
#include "sim_loop_tests.h"
#include "adc_calibration_loop.h"
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BOARD_LOOP_MAX_SAMPLES 2000U
static adc_cal_loop_t g_board_loop;
static uint8_t g_board_loop_frame[4096];
static bool g_board_loop_valid, g_quiet_calibration_capture;
static unsigned g_board_loop_run_wave, g_board_loop_wave;
static int spi_inst;
static uint8_t bt_index = 3U, bt_regs[2][3];
static int bt_code = 24, bt_moves, bt_writes, bt_ack_fail, bt_reg_fail, bt_capture_fail;
static double bt_hold_offset_bias, bt_hold_offset_slope;
static unsigned bt_hold_frames;
static unsigned bt_train_frames;
static bool bt_train_last_outlier;
static double bt_exported_train_offset_b;
static char bt_output[65536];
static size_t bt_output_n;
static unsigned bt_stream_rows;

static void xil_printf(const char *fmt, ...)
{
    va_list ap;
    int n;
    if (strncmp(fmt, "LOOP-EXPORT-DATA ", 17U) == 0) bt_stream_rows++;
    va_start(ap, fmt);
    n = vsnprintf(bt_output + bt_output_n, sizeof(bt_output) - bt_output_n, fmt, ap);
    va_end(ap);
    if (n > 0 && (size_t)n < sizeof(bt_output) - bt_output_n) bt_output_n += (size_t)n;
}
static int board_loop_isfinite(double v) { return v == v && fabs(v) <= DBL_MAX; }
static void board_loop_put(const char *key, double v, double scale)
{
    if (board_loop_isfinite(v)) xil_printf(" %s=%ld", key, (long)lround(v * scale));
    else xil_printf(" %s=nan", key);
}
static void board_loop_print_row(const adc_cal_loop_row_t *row)
{
    if (g_board_loop.opt.update_axes == ADC_CAL_LOOP_AXIS_OFFSET)
        bt_exported_train_offset_b = row->offset_b_state;
    xil_printf("LOOP st=%d\n", (int)row->rejected);
}
static int bt_reg_slot(uint16_t reg) { return reg == 0x1910U ? 0 : reg == 0x0701U ? 1 : 2; }
static void ad9695_read_register(int *spi, uint16_t reg, uint8_t *v)
{
    (void)spi;
    *v = reg == 8U ? bt_index : bt_regs[bt_index == 2U ? 1 : 0][bt_reg_slot(reg)];
}
static void ad9695_write_register(int *spi, uint16_t reg, uint8_t v)
{
    (void)spi;
    bt_writes++;
    if (reg == 8U) bt_index = v;
    else if (!bt_reg_fail) bt_regs[bt_index == 2U ? 1 : 0][bt_reg_slot(reg)] = v;
}
static int adc_skew_actuator_read_value(int *v) { *v = bt_code; return 0; }
static int board_loop_request_steps(void *ctx, int delta, adc_cal_loop_skew_ack_t *ack)
{
    (void)ctx;
    bt_moves++;
    ack->ok = !bt_ack_fail;
    if (ack->ok) bt_code += delta;
    ack->code_after = bt_code;
    return 0;
}
#include "../firmware/thesis_v3_loop_appl/loop_waveform.h"
static const char *board_loop_reject_tag(adc_cal_loop_reject_t rejected)
{
    return rejected == ADC_CAL_LOOP_REJECT_NONE ? "ok" :
           rejected == ADC_CAL_LOOP_REJECT_CLIPPING ? "clipping" : "rejected";
}
#define BUTILS_LOOP_FRAGMENT 1
#include "../firmware/thesis_v3_loop_appl/butils_loop_results.c"
#undef BUTILS_LOOP_FRAGMENT
static long board_loop_capture(void *ctx, uint8_t *buf, size_t cap)
{
    (void)ctx; (void)buf; (void)cap;
    return bt_capture_fail ? -1L : 1L;
}
static int bt_step(adc_cal_loop_t *loop, const uint8_t *raw, size_t n, adc_cal_loop_row_t *row)
{
    double mu_o = loop->state.mu_offset, mu_g = loop->state.mu_gain;
    (void)raw; (void)n;
    memset(row, 0, sizeof(*row));
    row->est.ch_a.offset_codes = (3.0 + loop->opt.input_offset[0] - loop->state.offset_a) * loop->state.gain_corr_a;
    row->est.ch_b.offset_codes = (-5.0 + loop->opt.input_offset[1] - loop->state.offset_b) * loop->state.gain_corr_b;
    if ((loop->opt.update_axes & ADC_CAL_LOOP_AXIS_OFFSET) != 0U) {
        if (bt_train_last_outlier && bt_train_frames == 59U)
            row->est.ch_b.offset_codes += 3.0 * loop->state.gain_corr_b;
        bt_train_frames++;
    }
    if (loop->opt.update_axes == 0U) {
        row->est.ch_b.offset_codes += bt_hold_offset_bias + bt_hold_offset_slope * (double)bt_hold_frames;
        bt_hold_frames++;
    }
    row->est.ch_a.tone_amplitude = 100.0 * loop->state.gain_corr_a;
    row->est.ch_b.tone_amplitude = 100.0 * loop->state.gain_corr_b;
    row->est.tone_ratio = row->est.ch_b.tone_amplitude / row->est.ch_a.tone_amplitude;
    row->est.mag_a = row->est.ch_a.tone_amplitude;
    row->est.mag_b = row->est.ch_b.tone_amplitude;
    row->est.gain_mag_ratio = row->est.tone_ratio;
    if (loop->opt.tone_free) {
        /* Deliberately unusable tone; baseline and summaries must use mag. */
        row->est.tone_ratio = NAN;
        row->est.ch_a.tone_amplitude = row->est.ch_b.tone_amplitude = NAN;
    }
    row->est.skew_used_ps = (double)(bt_code - 24) * 19.1;
    if ((loop->opt.update_axes & ADC_CAL_LOOP_AXIS_OFFSET) == 0U) loop->state.mu_offset = 0;
    if ((loop->opt.update_axes & ADC_CAL_LOOP_AXIS_GAIN) == 0U) loop->state.mu_gain = 0;
    (void)adc_cal_loop_state_update(&loop->state, &row->est, 0, loop->opt.gain_observable, 0, row);
    loop->state.mu_offset = mu_o;
    loop->state.mu_gain = mu_g;
    return 0;
}

#define BUTILS_LOOP_FRAGMENT 1
#define adc_cal_loop_step bt_step
#include "../firmware/thesis_v3_loop_appl/butils_loop_test.c"
#undef adc_cal_loop_step
#undef BUTILS_LOOP_FRAGMENT

static int bt_command(const char *args)
{
    char line[256];
    (void)snprintf(line, sizeof(line), "test %s", args);
    bt_output_n = 0U;
    bt_output[0] = '\0';
    bt_train_frames = 0U;
    (void)strtok(line, " ");
    board_loop_test_command();
    return strstr(bt_output, "LOOP-TEST-END result=OK") != NULL;
}

void sim_board_axis_tests_run(sim_assert_context_t *ctx, FILE *summary)
{
    adc_cal_loop_state_t baseline;
    int writes;
    fprintf(summary, "\n== board axis-test command/SPI spies ==\n");
    /* Legacy waveform IDs do not select references in blind mode. */
    SIM_ASSERT_TRUE(ctx, bt_command("baseline 40 10"));
    SIM_ASSERT_TRUE(ctx, g_board_loop.opt.reference_free);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop.ref_nz_count, 0);
    /* The ordinary board loop uses this helper without test_configure(), so
     * full-scale ADC rejection must not depend on the experiment wrapper. */
    for (unsigned wave = 2U; wave < 4U; ++wave) {
        adc_cal_loop_options_t opt;
        adc_cal_loop_default_options(&opt);
        loop_waveform_options(wave, &opt);
        SIM_ASSERT_EQ_INT(ctx, opt.reject_clipping, 1);
    }
    g_board_loop_wave = 2U;
    for (unsigned c = 0; c < 2U; ++c) {
        bt_regs[c][0] = 0x0CU; bt_regs[c][1] = 0xA5U; bt_regs[c][2] = 0xD7U;
    }
    SIM_ASSERT_TRUE(ctx, bt_command("baseline 40 10"));
    SIM_ASSERT_TRUE(ctx, g_loop_test_baseline_valid);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-TEST-ROW ") == NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-TEST-WINDOW phase=baseline_train") != NULL);
    baseline = g_loop_test_baseline;
    SIM_ASSERT_TRUE(ctx, bt_command("baseline 40 10 full"));
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-TEST-ROW phase=baseline_train") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-TEST-ROW phase=baseline_hold") != NULL);
    SIM_ASSERT_TRUE(ctx, !g_loop_test_full_rows);
    SIM_ASSERT_NEAR(ctx, g_loop_test_baseline.offset_a, baseline.offset_a, 0.0);
    SIM_ASSERT_NEAR(ctx, g_loop_test_baseline.gain_corr_b, baseline.gain_corr_b, 0.0);
    SIM_ASSERT_EQ_INT(ctx, g_loop_test_code, 24);
    SIM_ASSERT_EQ_INT(ctx, bt_index, 3);
    baseline = g_loop_test_baseline;
    SIM_ASSERT_TRUE(ctx, bt_command("offset B 32"));
    SIM_ASSERT_TRUE(ctx, bt_command("run offset 4 40 8"));
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.offset_b - baseline.offset_b, 32, 0.01);
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.gain_corr_b, baseline.gain_corr_b, 0);
    SIM_ASSERT_NEAR(ctx, g_loop_test_baseline.offset_b, baseline.offset_b, 0);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop.opt.update_axes, 0);
    SIM_ASSERT_TRUE(ctx, bt_command("restore"));
    SIM_ASSERT_NEAR(ctx, g_loop_test_offset[1], 0, 0);
    SIM_ASSERT_TRUE(ctx, bt_command("ifc B 0x0d"));
    SIM_ASSERT_EQ_INT(ctx, bt_regs[0][0], 12);
    SIM_ASSERT_EQ_INT(ctx, bt_regs[1][0], 13);
    SIM_ASSERT_EQ_INT(ctx, bt_index, 3);
    SIM_ASSERT_TRUE(ctx, bt_command("run gain 4 4 4"));
    SIM_ASSERT_TRUE(ctx, bt_command("ifc A 13"));
    SIM_ASSERT_TRUE(ctx, !bt_command("run gain 4 4 4")); /* two banks changed */
    SIM_ASSERT_TRUE(ctx, !bt_command("run skew 4 4 4"));
    SIM_ASSERT_TRUE(ctx, bt_command("restore"));
    SIM_ASSERT_TRUE(ctx, bt_command("offset A 8"));
    SIM_ASSERT_TRUE(ctx, bt_command("offset B 8"));
    SIM_ASSERT_TRUE(ctx, !bt_command("run offset 4 4 4"));
    SIM_ASSERT_TRUE(ctx, bt_command("restore"));
    SIM_ASSERT_TRUE(ctx, bt_command("dcfilter A off"));
    SIM_ASSERT_EQ_INT(ctx, bt_regs[0][1], 0x25);
    SIM_ASSERT_EQ_INT(ctx, bt_regs[0][2], 0xD7);
    SIM_ASSERT_EQ_INT(ctx, bt_regs[1][1], 0xA5);
    SIM_ASSERT_EQ_INT(ctx, bt_index, 3);
    SIM_ASSERT_TRUE(ctx, bt_command("dcfilter A on"));
    SIM_ASSERT_EQ_INT(ctx, bt_regs[0][1], 0xA5);
    SIM_ASSERT_EQ_INT(ctx, bt_regs[0][2], 0x57);
    SIM_ASSERT_TRUE(ctx, bt_command("restore"));
    writes = bt_writes;
    SIM_ASSERT_TRUE(ctx, !bt_command("ifc B 9"));
    SIM_ASSERT_TRUE(ctx, !bt_command("ifc C 13"));
    SIM_ASSERT_TRUE(ctx, !bt_command("offset B 4097"));
    SIM_ASSERT_TRUE(ctx, !bt_command("park 49"));
    SIM_ASSERT_TRUE(ctx, !bt_command("run skew 0 5 5"));
    SIM_ASSERT_TRUE(ctx, !bt_command("config 1 501 0 1 20"));
    SIM_ASSERT_EQ_INT(ctx, bt_writes, writes);
    SIM_ASSERT_TRUE(ctx, bt_command("step 24 4"));
    SIM_ASSERT_EQ_INT(ctx, bt_code, 24);
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.offset_a, baseline.offset_a, 0);
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.gain_corr_b, baseline.gain_corr_b, 0);
    bt_capture_fail = 1;
    SIM_ASSERT_TRUE(ctx, !bt_command("run offset 8 4 4"));
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "attempts=3 accepted=0") != NULL);
    bt_capture_fail = 0;
    bt_reg_fail = 1;
    SIM_ASSERT_TRUE(ctx, !bt_command("ifc B 13"));
    SIM_ASSERT_TRUE(ctx, g_loop_test_fault);
    SIM_ASSERT_EQ_INT(ctx, bt_index, 3);
    writes = bt_writes;
    SIM_ASSERT_TRUE(ctx, !bt_command("restore"));
    SIM_ASSERT_EQ_INT(ctx, bt_writes, writes);
    g_loop_test_fault = false; /* simulate a new bring-up */
    bt_reg_fail = 0;
    bt_ack_fail = 1;
    writes = bt_moves;
    SIM_ASSERT_TRUE(ctx, !bt_command("park 25"));
    SIM_ASSERT_EQ_INT(ctx, bt_moves, writes + 1);
    SIM_ASSERT_TRUE(ctx, !bt_command("park 25"));
    SIM_ASSERT_EQ_INT(ctx, bt_moves, writes + 1);
    SIM_ASSERT_TRUE(ctx, bt_command("status"));
    /* New waveform selects the tone-free controller and baseline grading;
     * exercise the actual waveform table rather than a stub configuration. */
    g_loop_test_fault = false; bt_ack_fail = 0; bt_code = 24;
    g_board_loop_wave = 0U;
    writes = bt_writes;
    SIM_ASSERT_TRUE(ctx, !bt_command("restore"));
    SIM_ASSERT_TRUE(ctx, bt_command("baseline 40 10"));
    SIM_ASSERT_TRUE(ctx, g_board_loop.opt.reference_free);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop.ref_nz_count, 0);
    g_board_loop_wave = 2U;
    SIM_ASSERT_TRUE(ctx, bt_command("baseline 40 10"));
    SIM_ASSERT_TRUE(ctx, g_board_loop.opt.tone_free);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop.opt.cancel_signal, 0);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop.opt.require_tone, 0);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop.opt.gain_observable, ADC_CAL_LOOP_GAIN_DITHER_MAG);
    SIM_ASSERT_NEAR(ctx, g_board_loop.cfg.amp_dbfs, 0.0, 0.0);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "gain=dither_spectrum skew=dither_cross_spectrum") != NULL);
    baseline = g_loop_test_baseline;
    SIM_ASSERT_TRUE(ctx, bt_command("offset B 32"));
    SIM_ASSERT_TRUE(ctx, bt_command("run offset 4 40 8"));
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.offset_b - baseline.offset_b, 32.0, 0.01);
    SIM_ASSERT_TRUE(ctx, bt_command("restore"));
    SIM_ASSERT_TRUE(ctx, bt_command("ifc B 13"));
    SIM_ASSERT_TRUE(ctx, bt_command("run gain 4 4 4"));
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "gain_ppm=0") != NULL);
    SIM_ASSERT_TRUE(ctx, bt_command("restore"));
    SIM_ASSERT_TRUE(ctx, bt_command("step 24 4"));
    SIM_ASSERT_EQ_INT(ctx, bt_code, 24);
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.gain_corr_b, baseline.gain_corr_b, 0.0);
    g_board_loop_wave = 3U;
    SIM_ASSERT_TRUE(ctx, bt_command("baseline 40 10"));
    SIM_ASSERT_NEAR(ctx, g_board_loop.cfg.dither_scale_lsb, 0.0, 0.0);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop.cfg.sig_cycles, 0);
    SIM_ASSERT_TRUE(ctx, g_board_loop.opt.tone_free);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop.opt.reject_clipping, 1);
    /* A converged TRAIN state cannot qualify a drifting frozen HOLD. Reproduce the
     * reported -1.623-code mean without changing the real feedback update. */
    bt_hold_frames = 0U;
    bt_hold_offset_bias = -0.90503;
    bt_hold_offset_slope = -1.435 / 99.0;
    SIM_ASSERT_TRUE(ctx, !bt_command("baseline 40 100"));
    SIM_ASSERT_TRUE(ctx, !g_loop_test_baseline_valid);
    SIM_ASSERT_TRUE(ctx, !g_loop_test_fault);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "axis=offset pass=0 mean_mc=-1623 limit_mc=500") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "axis=gain pass=1") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "axis=skew pass=1") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "baseline-hold-out-of-spec") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "valid=0 snapshot=1 wave=-1 code=-1 fault=0") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-TEST-CURRENT wave=3 waveform=reference_free_dither code=24") != NULL);
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.offset_a, 3.0, 1e-5);
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.offset_b, -5.0, 1e-5);
    {
        unsigned windows = 0U;
        const char *cursor = bt_output;
        while ((cursor = strstr(cursor, "LOOP-TEST-WINDOW phase=baseline_hold")) != NULL) {
            windows++;
            cursor++;
        }
        SIM_ASSERT_EQ_INT(ctx, windows, 5);
    }
    writes = bt_writes;
    SIM_ASSERT_TRUE(ctx, !bt_command("run offset 4 4 4"));
    SIM_ASSERT_EQ_INT(ctx, bt_writes, writes);
    bt_hold_offset_bias = bt_hold_offset_slope = 0.0;
    /* A last-frame 3-code estimator outlier would freeze a 1.05-code error
     * with mu=0.35. Training-only averaging leaves 3/30=0.1 code; fresh HOLD
     * must pass without adapting on its measurements. */
    bt_train_last_outlier = true;
    SIM_ASSERT_TRUE(ctx, bt_command("baseline 60 100"));
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "source=dither_training_mean samples=30") != NULL);
    SIM_ASSERT_NEAR(ctx, g_loop_test_baseline.offset_b, -4.9, 1e-9);
    SIM_ASSERT_NEAR(ctx, g_loop_test_baseline.mu_offset, 0.35, 0.0);
    bt_train_last_outlier = false;
    SIM_ASSERT_TRUE(ctx, bt_command("baseline 40 10"));
    baseline = g_loop_test_baseline;
    SIM_ASSERT_TRUE(ctx, bt_command("offset B 16"));
    bt_train_last_outlier = true;
    SIM_ASSERT_TRUE(ctx, bt_command("run offset 4 60 100 full"));
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "source=dither_training_mean samples=30 code=24 phase=train") != NULL);
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.offset_b, 11.1, 1e-9);
    SIM_ASSERT_NEAR(ctx, bt_exported_train_offset_b, g_board_loop.state.offset_b, 0.0);
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.gain_corr_b, baseline.gain_corr_b, 0.0);
    SIM_ASSERT_NEAR(ctx, g_loop_test_baseline.offset_b, baseline.offset_b, 0.0);
    bt_train_last_outlier = false;
    SIM_ASSERT_TRUE(ctx, bt_command("restore"));
    /* Exercise the production recorder/exporter, including a rejected frame
     * and a DMA capture with no measurement. Export must not touch hardware. */
    bt_output_n = 0U; bt_output[0] = '\0';
    board_loop_results_export();
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "result=ERROR reason=no-completed-run") != NULL);
    bt_output_n = 0U; bt_output[0] = '\0';
    board_loop_results_begin(3U, 24U, true, true);
    board_loop_results_stream_begin();
    for (unsigned i = 0U; i < 25U; ++i) {
        adc_cal_loop_row_t row = {0};
        row.iteration = i;
        row.rejected = i == 0U ? ADC_CAL_LOOP_REJECT_CLIPPING : ADC_CAL_LOOP_REJECT_NONE;
        row.clipped_samples = i == 0U ? 1U : 0U;
        row.est.gain_mag_ratio = 1.0 + (i % 2U ? 0.001 : -0.001);
        row.est.ch_b.offset_codes = i % 2U ? 1.0 : -1.0;
        row.est.skew_used_ps = -2.09;
        row.gain_corr_a = 1.01; row.gain_corr_b = 0.99;
        row.offset_a_state = 3.0; row.offset_b_state = -5.0;
        row.skew_code = 12; row.prep.align_margin = 17.0;
        board_loop_results_record(&row, i + 2U);
    }
    g_board_loop.qualified = 24U; g_board_loop.captures = 26U;
    g_board_loop.rejected = 1U; g_board_loop.act.code = 12;
    g_board_loop.state.gain_corr_a = 1.01; g_board_loop.state.gain_corr_b = 0.99;
    g_board_loop.state.offset_a = 3.0; g_board_loop.state.offset_b = -5.0;
    board_loop_results_finish(&g_board_loop, 1234UL, 19UL);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop_result.stored, 25);
    SIM_ASSERT_TRUE(ctx, g_board_loop_result.ready);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-PROGRESS accepted=20/24") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-PROGRESS accepted=24/24") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-RESULT window=20 feedback=active gain_ppm=0") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-EXPORT-END result=OK run_id=1 rows=25 total_rows=25 dropped=0") != NULL);
    fprintf(summary, "\n== production C streaming fixture ==\n%s", bt_output);
    /* Later experiment commands reuse g_board_loop, not the saved result. */
    g_board_loop.state.offset_a = 888.0;
    bt_output_n = 0U; bt_output[0] = '\0';
    writes = bt_writes;
    {
        const int moves = bt_moves;
        board_loop_results_export();
        SIM_ASSERT_EQ_INT(ctx, bt_writes, writes);
        SIM_ASSERT_EQ_INT(ctx, bt_moves, moves);
    }
    SIM_ASSERT_NEAR(ctx, g_board_loop.state.offset_a, 888.0, 0.0);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "offa_mc=3000 offb_mc=-5000") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "waveform=reference_free_dither") != NULL);
    SIM_ASSERT_TRUE(ctx, strstr(bt_output, "LOOP-EXPORT-END result=OK run_id=1 rows=25") != NULL);
    /* Streaming keeps every record even when the optional RAM snapshot wraps. */
    bt_stream_rows = 0U;
    board_loop_results_begin(3U, BOARD_LOOP_MAX_SAMPLES, true, true);
    board_loop_results_stream_begin();
    for (unsigned i = 1U; i <= BOARD_LOOP_HISTORY_CAPACITY + 3U; ++i) {
        adc_cal_loop_row_t row = {0};
        row.est.gain_mag_ratio = 1.0;
        board_loop_results_record(&row, i);
    }
    SIM_ASSERT_EQ_INT(ctx, g_board_loop_result.stored, BOARD_LOOP_HISTORY_CAPACITY);
    SIM_ASSERT_EQ_INT(ctx, g_board_loop_result.seen - g_board_loop_result.stored, 3);
    SIM_ASSERT_EQ_INT(ctx, board_loop_result_row(0U)->capture, 4);
    SIM_ASSERT_EQ_INT(ctx, board_loop_result_row(BOARD_LOOP_HISTORY_CAPACITY - 1U)->capture,
                      BOARD_LOOP_HISTORY_CAPACITY + 3U);
    SIM_ASSERT_EQ_INT(ctx, bt_stream_rows, BOARD_LOOP_HISTORY_CAPACITY + 3U);
}
