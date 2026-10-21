/* butils_loop.c
 * "adc -cal loop": the impulse-dither calibration loop run on the A53.
 *
 * #included at the end of butils.c, like butils_calibration.c, so it reaches the
 * file-local capture and skew-transaction helpers.  butils.c defines
 * BUTILS_LOOP_FRAGMENT first; compiled on its own (Vitis adds every .c in the
 * component folder to the source list) this file is empty, not a duplicate.
 *
 * The loop itself is calibration_loop_c/adc_calibration_loop.c (shared with
 * calibration_sim, verified against the host Python loop).  This file is only
 * the board adapter: capture, skew transaction, console parsing and printing.
 */

#ifdef BUTILS_LOOP_FRAGMENT

#include "adc_calibration_loop.h"
#include "loop_waveform.h"
#include "xiltimer.h"

#define BOARD_LOOP_MAX_SAMPLES 2000U

static adc_cal_loop_t g_board_loop;                  /* ~570 KB: .bss, never the stack */
static uint8_t g_board_loop_frame[DMA_CMD_BUF_SIZE + 1U];
static bool g_board_loop_valid;                     /* g_board_loop holds a finished run */
static bool g_board_loop_closed;                    /* that run was allowed to move skew */
static unsigned g_board_loop_wave;                  /* k_loop_waveforms[] row assumed on the DPG */
static unsigned g_board_loop_run_wave;              /* row the last run used */
static XTime g_board_loop_t_capture_end;
static uint64_t g_board_loop_compute_ticks;
static uint64_t g_board_loop_compute_count;

/* ---------------------------------------------------------------------------
 * Printing.  xil_printf has no floating point, so values go out as scaled
 * integers ("nan" when not finite).
 * ------------------------------------------------------------------------ */

static int board_loop_isfinite(double v)
{
    return v == v && v <= DBL_MAX && v >= -DBL_MAX;
}

static void board_loop_put(const char *key, double value, double scale)
{
    if (!board_loop_isfinite(value) || fabs(value * scale) > 9.0e15) {
        xil_printf(" %s=nan", key);
    } else {
        xil_printf(" %s=%ld", key, (long)lround(value * scale));
    }
}

static const char *board_loop_reject_tag(adc_cal_loop_reject_t r)
{
    switch (r) {
    case ADC_CAL_LOOP_REJECT_NONE: return "ok";
    case ADC_CAL_LOOP_REJECT_ALIGN_MARGIN: return "margin";
    case ADC_CAL_LOOP_REJECT_TOO_FEW_EVENTS: return "events";
    case ADC_CAL_LOOP_REJECT_A_NOT_FINITE: return "a_nan";
    case ADC_CAL_LOOP_REJECT_B_NOT_FINITE: return "b_nan";
    case ADC_CAL_LOOP_REJECT_SKEW_RANGE: return "skew_range";
    case ADC_CAL_LOOP_REJECT_GAIN_RANGE: return "gain_range";
    case ADC_CAL_LOOP_REJECT_CAPTURE: return "capture";
    }
    return "?";
}

/* One line per capture:
 *   LOOP it=<n> st=<ok|reason> n0= rot= margin_x100=
 *        tone_ppm=(|B|/|A| tone - 1)e6  gain_ppm=(g_B/g_A dither - 1)e6
 *        doff_mc=(off_B - off_A)e3 codes  skew_cps=skew_used e2 ps
 *        corr_a_ppm= corr_b_ppm= (gain_corr - 1)e6  offa_mc= offb_mc= (state offset)e3
 *        [batch=<used>/<frames> bmean_cps= act=<action> code=<code>]   on a batch decision
 */
static void board_loop_print_row(const adc_cal_loop_row_t *row)
{
    const adc_cal_loop_block_estimate_t *e = &row->est;

    xil_printf("LOOP it=%u st=%s n0=%d rot=%d", row->iteration,
               board_loop_reject_tag(row->rejected), row->prep.n0, row->prep.rotation);
    board_loop_put("margin_x100", row->prep.align_margin, 100.0);
    board_loop_put("tone_ppm", e->tone_ratio - 1.0, 1e6);
    board_loop_put("gain_ppm", e->gain_ratio - 1.0, 1e6);
    board_loop_put("doff_mc", e->ch_b.offset_codes - e->ch_a.offset_codes, 1e3);
    board_loop_put("skew_cps", e->skew_used_ps, 100.0);
    board_loop_put("corr_a_ppm", row->gain_corr_a - 1.0, 1e6);
    board_loop_put("corr_b_ppm", row->gain_corr_b - 1.0, 1e6);
    board_loop_put("offa_mc", row->offset_a_state, 1e3);
    board_loop_put("offb_mc", row->offset_b_state, 1e3);
    if (row->skew_action != ADC_CAL_LOOP_SKEW_ACTION_NONE) {
        xil_printf(" batch=%d/%d", row->skew_batch_used, row->skew_batch_n);
        board_loop_put("bmean_cps", row->skew_batch_mean_ps, 100.0);
        xil_printf(" act=%s", adc_cal_loop_skew_action_name(row->skew_action));
        if (row->skew_step_codes != 0) {
            xil_printf("%s%d", row->skew_step_codes > 0 ? "+" : "", row->skew_step_codes);
        }
        xil_printf(" code=%d", row->skew_code);
    }
    xil_printf("\r\n");
}

/* ---------------------------------------------------------------------------
 * Hooks handed to the loop.
 * ------------------------------------------------------------------------ */

static long board_loop_capture(void *ctx, uint8_t *buf, size_t capacity)
{
    const uint32_t generation_before = dma_capture_generation();
    (void)ctx;

    if (capacity < DMA_CMD_BUF_SIZE) {
        return -1;
    }
    if (adc_capture_frame() != XST_SUCCESS) {
        xil_printf("LOOP-CAPTURE failed\r\n");
        return -1;
    }
    /* adc_capture_frame() publishes a new generation only after the transfer
     * completed and the cache was invalidated: anything else is a stale buffer. */
    if (dma_capture_generation() == generation_before) {
        xil_printf("LOOP-CAPTURE stale buffer\r\n");
        return -1;
    }
    memcpy(buf, RxBufferPtr, DMA_CMD_BUF_SIZE);
    XTime_GetTime(&g_board_loop_t_capture_end);
    return (long)DMA_CMD_BUF_SIZE;
}

/* The sanctioned write path: the same transaction "adc -cal skew step" runs
 * (neutral check, write + readback, one JESD reset, warm-up capture, freshness
 * check).  Only its result updates the loop's belief about the code. */
static int board_loop_request_steps(void *ctx, int delta, adc_cal_loop_skew_ack_t *ack)
{
    adc_skew_txn_result_t r;
    (void)ctx;

    (void)adc_skew_transaction(delta, &r);
    ack->ok = r.ok;
    if (r.code_after >= 0) {
        ack->code_after = r.code_after;
    }
    xil_printf("LOOP-SKEW-TXN steps=%s%d result=%s code_before=%d code_after=%d stage=%s\r\n",
               delta > 0 ? "+" : "", delta, r.ok ? "OK" : "RECOVERY_REQUIRED",
               r.code_before, r.code_after,
               r.stage != NULL ? r.stage : "-");
    return 0;
}

static void board_loop_on_row(void *ctx, const adc_cal_loop_row_t *row)
{
    XTime now;
    (void)ctx;

    XTime_GetTime(&now);
    if (now >= g_board_loop_t_capture_end) {
        g_board_loop_compute_ticks += (uint64_t)(now - g_board_loop_t_capture_end);
        g_board_loop_compute_count++;
    }
    board_loop_print_row(row);
}

/* ---------------------------------------------------------------------------
 * Commands.
 * ------------------------------------------------------------------------ */

static void board_loop_print_waves(void)
{
    for (unsigned i = 0U; i < LOOP_WAVEFORM_COUNT; ++i) {
        xil_printf("      %c %u  %s  (%s MHz)\r\n", i == g_board_loop_wave ? '*' : ' ', i,
                   k_loop_waveforms[i].name, k_loop_waveforms[i].tone_mhz);
    }
}

static void adc_loop_print_command_help(void)
{
    xil_printf("  adc -cal loop N           On-board impulse-dither loop: N qualified samples,\r\n");
    xil_printf("                            gain/offset per frame, skew batch decision (ACK-checked).\r\n");
    xil_printf("  adc -cal loop open N      Same, skew measured only (no actuator writes).\r\n");
    xil_printf("  adc -cal loop status      State of the last loop run.\r\n");
    xil_printf("  adc -cal loop wave [K]    List the known DPG vectors / select K as the one loaded.\r\n");
    xil_printf("      Waveform assumed on the DPG (*):\r\n");
    board_loop_print_waves();
}

static void board_loop_print_status(void)
{
    const adc_cal_loop_t *L = &g_board_loop;

    if (!g_board_loop_valid) {
        xil_printf("LOOP-STATUS no run yet (adc -cal loop N)\r\n");
        return;
    }
    xil_printf("LOOP-STATUS mode=%s waveform=%s qualified=%u captures=%u rejected=%u polarity=%d "
               "code=%d skew_abort=%d iteration=%u",
               g_board_loop_closed ? "closed" : "open", k_loop_waveforms[g_board_loop_run_wave].name,
               L->qualified, L->captures, L->rejected,
               (int)L->polarity, L->act.code, L->skew_abort, L->state.iteration);
    board_loop_put("corr_a_ppm", L->state.gain_corr_a - 1.0, 1e6);
    board_loop_put("corr_b_ppm", L->state.gain_corr_b - 1.0, 1e6);
    board_loop_put("offa_mc", L->state.offset_a, 1e3);
    board_loop_put("offb_mc", L->state.offset_b, 1e3);
    xil_printf("\r\n");
}

static void board_loop_run(unsigned n_qualified, bool closed)
{
    adc_cal_loop_config_t cfg;
    adc_cal_loop_options_t opt;
    adc_cal_loop_actuator_t act;
    const char *why = NULL;
    const bool quiet_before = g_quiet_calibration_capture;
    XTime t_start;
    XTime t_end;
    unsigned got;
    int code = -1;

    loop_waveform_config(g_board_loop_wave, &cfg);
    if (adc_cal_loop_config_validate(&cfg, &why) != 0) {
        ERR("Loop waveform config invalid: %s", why != NULL ? why : "?");
        return;
    }
    adc_cal_loop_default_options(&opt);
    opt.close_skew_loop = closed ? 1 : 0;
    adc_cal_loop_default_actuator(&act);
    act.request_steps = board_loop_request_steps;

    if (closed) {
        /* Learn the actuator position through a zero-step transaction, exactly as
         * the host SkewActuator does before its first move.  A failure here means
         * the bench is not in a known actuator state: stop before writing. */
        adc_skew_txn_result_t r;
        (void)adc_skew_transaction(0, &r);
        xil_printf("LOOP-SKEW-TXN steps=+0 result=%s code_after=%d stage=%s\r\n",
                   r.ok ? "OK" : "RECOVERY_REQUIRED", r.code_after,
                   r.stage != NULL ? r.stage : "-");
        if (!r.ok || r.code_after < 0) {
            ERR("Actuator not in a known state; run adc -cal skew step 0 first.");
            return;
        }
        code = r.code_after;
    } else if (adc_skew_actuator_read_value(&code) != 0) {
        code = -1; /* measure-only: the code is reported, never needed */
    }
    act.code = code;

    if (adc_cal_loop_init(&g_board_loop, &cfg, &opt, NULL, &act) != 0) {
        ERR("adc_cal_loop_init failed.");
        return;
    }
    g_board_loop_valid = false;
    g_board_loop_closed = closed;
    g_board_loop_run_wave = g_board_loop_wave;
    g_board_loop_compute_ticks = 0U;
    g_board_loop_compute_count = 0U;

    xil_printf("LOOP-START mode=%s samples=%u waveform=%s tone_mhz=%s sig_cycles=%d code=%d\r\n",
               closed ? "closed" : "open", n_qualified, k_loop_waveforms[g_board_loop_wave].name,
               k_loop_waveforms[g_board_loop_wave].tone_mhz, cfg.sig_cycles, code);

    g_quiet_calibration_capture = true;
    XTime_GetTime(&t_start);
    got = adc_cal_loop_run(&g_board_loop, n_qualified, board_loop_capture, NULL,
                           board_loop_on_row, NULL,
                           g_board_loop_frame, sizeof(g_board_loop_frame));
    XTime_GetTime(&t_end);
    g_quiet_calibration_capture = quiet_before;
    g_board_loop_valid = true;

    xil_printf("LOOP-END qualified=%u/%u captures=%u rejected=%u code=%d skew_abort=%d "
               "elapsed_ms=%lu compute_ms_avg=%lu\r\n",
               got, n_qualified, g_board_loop.captures, g_board_loop.rejected,
               g_board_loop.act.code, g_board_loop.skew_abort,
               (unsigned long)((t_end - t_start) * 1000ULL / COUNTS_PER_SECOND),
               (unsigned long)(g_board_loop_compute_count
                   ? g_board_loop_compute_ticks * 1000ULL /
                     (g_board_loop_compute_count * (uint64_t)COUNTS_PER_SECOND)
                   : 0ULL));
    board_loop_print_status();
}

/* Called from the "adc -cal" parser with strtok positioned after "loop". */
static void handle_adc_loop_cmd(void)
{
    char *token = strtok(NULL, " ");
    bool closed = true;
    char *endptr = NULL;
    unsigned long parsed;

    if (token == NULL || strcmp(token, "help") == 0 || strcmp(token, "?") == 0) {
        adc_loop_print_command_help();
        return;
    }
    if (strcmp(token, "status") == 0) {
        board_loop_print_status();
        return;
    }
    if (strcmp(token, "wave") == 0) {
        token = strtok(NULL, " ");
        if (token != NULL) {
            parsed = strtoul(token, &endptr, 0);
            if (endptr == token || *endptr != '\0' || parsed >= LOOP_WAVEFORM_COUNT ||
                strtok(NULL, " ") != NULL) {
                ERR("Use adc -cal loop wave K with K = 0..%u.", (unsigned)LOOP_WAVEFORM_COUNT - 1U);
                return;
            }
            g_board_loop_wave = (unsigned)parsed;
        }
        xil_printf("LOOP-WAVE selected=%u name=%s tone_mhz=%s sig_cycles=%d\r\n",
                   g_board_loop_wave, k_loop_waveforms[g_board_loop_wave].name,
                   k_loop_waveforms[g_board_loop_wave].tone_mhz,
                   k_loop_waveforms[g_board_loop_wave].sig_cycles);
        board_loop_print_waves();
        return;
    }
    if (strcmp(token, "open") == 0) {
        closed = false;
        token = strtok(NULL, " ");
        if (token == NULL) {
            ERR("Use adc -cal loop open N.");
            return;
        }
    }
    parsed = strtoul(token, &endptr, 0);
    if (endptr == token || *endptr != '\0' || parsed == 0UL ||
        parsed > BOARD_LOOP_MAX_SAMPLES || strtok(NULL, " ") != NULL) {
        ERR("Use adc -cal loop [open] N with N = 1..%u.", BOARD_LOOP_MAX_SAMPLES);
        return;
    }
    board_loop_run((unsigned)parsed, closed);
}

#endif /* BUTILS_LOOP_FRAGMENT */
