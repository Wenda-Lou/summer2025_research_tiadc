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
static char bt_output[65536];
static size_t bt_output_n;

static void xil_printf(const char *fmt, ...)
{
    va_list ap;
    int n;
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
static void loop_waveform_config(unsigned wave, adc_cal_loop_config_t *cfg)
{
    (void)wave;
    adc_cal_loop_default_config(cfg);
}
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
    row->est.ch_a.tone_amplitude = 100.0 * loop->state.gain_corr_a;
    row->est.ch_b.tone_amplitude = 100.0 * loop->state.gain_corr_b;
    row->est.tone_ratio = row->est.ch_b.tone_amplitude / row->est.ch_a.tone_amplitude;
    row->est.skew_used_ps = (double)(bt_code - 24) * 19.1;
    if ((loop->opt.update_axes & ADC_CAL_LOOP_AXIS_OFFSET) == 0U) loop->state.mu_offset = 0;
    if ((loop->opt.update_axes & ADC_CAL_LOOP_AXIS_GAIN) == 0U) loop->state.mu_gain = 0;
    (void)adc_cal_loop_state_update(&loop->state, &row->est, 0, ADC_CAL_LOOP_GAIN_TONE, 0, row);
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
    (void)strtok(line, " ");
    board_loop_test_command();
    return strstr(bt_output, "LOOP-TEST-END result=OK") != NULL;
}

void sim_board_axis_tests_run(sim_assert_context_t *ctx, FILE *summary)
{
    adc_cal_loop_state_t baseline;
    int writes;
    fprintf(summary, "\n== board axis-test command/SPI spies ==\n");
    for (unsigned c = 0; c < 2U; ++c) {
        bt_regs[c][0] = 0x0CU; bt_regs[c][1] = 0xA5U; bt_regs[c][2] = 0xD7U;
    }
    SIM_ASSERT_TRUE(ctx, bt_command("baseline 40 10"));
    SIM_ASSERT_TRUE(ctx, g_loop_test_baseline_valid);
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
}
