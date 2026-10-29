/* Included by butils_loop.c; standalone Vitis compilation is intentionally empty.
 * Logging/export only. No estimator, feedback or hardware writes live here. */
#ifdef BUTILS_LOOP_FRAGMENT

#define BOARD_LOOP_HISTORY_CAPACITY (2U * BOARD_LOOP_MAX_SAMPLES)
#define BOARD_LOOP_PROGRESS_FRAMES 20U

typedef struct { unsigned n; double mean, m2; } board_loop_stat_t;
typedef struct {
    unsigned capture;
    adc_cal_loop_row_t row;
} board_loop_saved_row_t;
typedef struct {
    bool ready, closed, streaming;
    unsigned run_id, wave, requested, qualified, captures, rejected;
    unsigned stored, seen, head, accepted;
    int code, abort;
    double polarity;
    double deadband_ps, step_ps;
    unsigned long elapsed_ms, compute_ms;
    adc_cal_loop_state_t state;
} board_loop_result_t;

static board_loop_saved_row_t g_board_loop_history[BOARD_LOOP_HISTORY_CAPACITY];
static board_loop_result_t g_board_loop_result;
static board_loop_stat_t g_board_loop_progress[3];
static void board_loop_results_header(void);
static void board_loop_results_meta(void);
static void board_loop_results_data(const board_loop_saved_row_t *saved);

static void board_loop_stat_add(board_loop_stat_t *s, double x)
{
    double delta;
    if (!board_loop_isfinite(x)) return;
    s->n++;
    delta = x - s->mean;
    s->mean += delta / (double)s->n;
    s->m2 += delta * (x - s->mean);
}

static void board_loop_result_stats(board_loop_stat_t *s, const adc_cal_loop_row_t *row)
{
    board_loop_stat_add(&s[0], row->est.gain_mag_ratio - 1.0);
    board_loop_stat_add(&s[1], row->est.ch_b.offset_codes - row->est.ch_a.offset_codes);
    board_loop_stat_add(&s[2], row->est.skew_used_ps);
}

static void board_loop_result_print_stats(const board_loop_stat_t *s, bool sd)
{
    static const char *mean_keys[3] = {"gain_ppm", "offset_mc", "skew_cps"};
    static const char *sd_keys[3] = {"gain_sd_ppm", "offset_sd_mc", "skew_sd_cps"};
    const double scales[3] = {1e6, 1e3, 100.0};
    for (unsigned i = 0U; i < 3U; ++i) {
        board_loop_put(mean_keys[i], s[i].n ? s[i].mean : NAN, scales[i]);
        if (sd) board_loop_put(sd_keys[i], s[i].n > 1U ?
            sqrt(fmax(0.0, s[i].m2 / (double)(s[i].n - 1U))) : NAN, scales[i]);
    }
}

static const board_loop_saved_row_t *board_loop_result_row(unsigned i)
{
    return &g_board_loop_history[(g_board_loop_result.head + BOARD_LOOP_HISTORY_CAPACITY -
                                 g_board_loop_result.stored + i) % BOARD_LOOP_HISTORY_CAPACITY];
}

static void board_loop_results_begin(unsigned wave, unsigned requested, bool closed, bool streaming)
{
    const unsigned run_id = g_board_loop_result.run_id + 1U;
    memset(&g_board_loop_result, 0, sizeof(g_board_loop_result));
    memset(g_board_loop_progress, 0, sizeof(g_board_loop_progress));
    g_board_loop_result.run_id = run_id;
    g_board_loop_result.wave = wave;
    g_board_loop_result.requested = requested;
    g_board_loop_result.closed = closed;
    g_board_loop_result.streaming = streaming;
}

static void board_loop_results_stream_begin(void)
{
    if (!g_board_loop_result.streaming) return;
    xil_printf("LOOP-EXPORT-BEGIN version=2 run_id=%u mode=stream\r\n", g_board_loop_result.run_id);
    board_loop_results_header();
}

static void board_loop_results_progress(int code)
{
    if (g_board_loop_progress[0].n == 0U) return;
    xil_printf("LOOP-PROGRESS accepted=%u/%u window=%u code=%d",
               g_board_loop_result.accepted, g_board_loop_result.requested,
               g_board_loop_progress[0].n, code);
    board_loop_result_print_stats(g_board_loop_progress, false);
    xil_printf("\r\n");
    memset(g_board_loop_progress, 0, sizeof(g_board_loop_progress));
}

static void board_loop_results_record(const adc_cal_loop_row_t *row, unsigned capture)
{
    board_loop_saved_row_t *saved = &g_board_loop_history[g_board_loop_result.head];
    saved->row = *row;
    saved->capture = capture;
    g_board_loop_result.head = (g_board_loop_result.head + 1U) % BOARD_LOOP_HISTORY_CAPACITY;
    g_board_loop_result.seen++;
    if (g_board_loop_result.stored < BOARD_LOOP_HISTORY_CAPACITY) g_board_loop_result.stored++;
    /* PC mode transmits EVERY record immediately, independent of the bounded
     * optional RAM snapshot. The PC keeps data out of the human UART log. */
    if (g_board_loop_result.streaming) board_loop_results_data(saved);
    if (row->rejected == ADC_CAL_LOOP_REJECT_NONE) {
        g_board_loop_result.accepted++;
        board_loop_result_stats(g_board_loop_progress, row);
        if (g_board_loop_result.accepted % BOARD_LOOP_PROGRESS_FRAMES == 0U)
            board_loop_results_progress(row->skew_code);
    }
}

static void board_loop_results_finish(const adc_cal_loop_t *loop,
                                      unsigned long elapsed_ms, unsigned long compute_ms)
{
    board_loop_result_t *r = &g_board_loop_result;
    board_loop_stat_t tail[3] = {{0}};
    unsigned n = 0U;
    board_loop_results_progress(loop->act.code);
    r->qualified = loop->qualified;
    r->captures = loop->captures;
    r->rejected = loop->rejected;
    r->code = loop->act.code;
    r->abort = loop->skew_abort;
    r->polarity = loop->polarity;
    r->deadband_ps = loop->act.deadband_ps;
    r->step_ps = loop->act.step_ps;
    r->state = loop->state;
    r->elapsed_ms = elapsed_ms;
    r->compute_ms = compute_ms;
    r->ready = true;
    if (r->streaming) {
        board_loop_results_meta();
        xil_printf("LOOP-EXPORT-END result=OK run_id=%u rows=%u total_rows=%u dropped=0\r\n",
                   r->run_id, r->seen, r->seen);
    }
    for (unsigned i = r->stored; i > 0U && n < BOARD_LOOP_PROGRESS_FRAMES; --i) {
        const adc_cal_loop_row_t *row = &board_loop_result_row(i - 1U)->row;
        if (row->rejected == ADC_CAL_LOOP_REJECT_NONE) {
            board_loop_result_stats(tail, row);
            n++;
        }
    }
    xil_printf("LOOP-RESULT window=%u feedback=active", n);
    board_loop_result_print_stats(tail, true);
    xil_printf("\r\n");
}

/* Scaled integer transport matches ordinary logs. PC converts to physical units. */
static void board_loop_csv_number(double x, double scale)
{
    if (!board_loop_isfinite(x) || fabs(x * scale) > 9.0e15) xil_printf(",nan");
    else xil_printf(",%ld", (long)lround(x * scale));
}

static void board_loop_results_meta(void)
{
    const board_loop_result_t *r = &g_board_loop_result;
    xil_printf("LOOP-EXPORT-META run_id=%u mode=%s waveform=%s requested=%u qualified=%u "
               "captures=%u rejected=%u code=%d skew_abort=%d elapsed_ms=%lu compute_ms_avg=%lu "
               "gsrc=dither_spectrum skew_route=dither_cross_spectrum reference_free=1 feedback=active",
               r->run_id, r->closed ? "closed" : "open", "reference_free_dither",
               r->requested, r->qualified, r->captures, r->rejected, r->code, r->abort,
               r->elapsed_ms, r->compute_ms);
    board_loop_put("corr_a_ppm", r->state.gain_corr_a - 1.0, 1e6);
    board_loop_put("corr_b_ppm", r->state.gain_corr_b - 1.0, 1e6);
    board_loop_put("offa_mc", r->state.offset_a, 1e3);
    board_loop_put("offb_mc", r->state.offset_b, 1e3);
    board_loop_put("polarity", r->polarity, 1.0);
    board_loop_put("deadband_cps", r->deadband_ps, 100.0);
    board_loop_put("step_cps", r->step_ps, 100.0);
    board_loop_put("mu_offset_ppm", r->state.mu_offset, 1e6);
    board_loop_put("mu_gain_ppm", r->state.mu_gain, 1e6);
    xil_printf("\r\n");
}

static void board_loop_results_header(void)
{
    xil_printf("LOOP-EXPORT-HEADER capture,it,st,used_gain_ppm,doff_mc,skew_cps,corr_a_ppm,"
               "corr_b_ppm,offa_mc,offb_mc,code,act,steps,batch_used,batch_n,bmean_cps,clips,n0,rot,margin_x100\r\n");
}

static void board_loop_results_data(const board_loop_saved_row_t *saved)
{
        const adc_cal_loop_row_t *row = &saved->row;
        const char *action = row->skew_action == ADC_CAL_LOOP_SKEW_ACTION_NONE ?
                             "none" : adc_cal_loop_skew_action_name(row->skew_action);
        xil_printf("LOOP-EXPORT-DATA %u,%u,%s", saved->capture, row->iteration,
                   board_loop_reject_tag(row->rejected));
        board_loop_csv_number(row->est.gain_mag_ratio - 1.0, 1e6);
        board_loop_csv_number(row->est.ch_b.offset_codes - row->est.ch_a.offset_codes, 1e3);
        board_loop_csv_number(row->est.skew_used_ps, 100.0);
        board_loop_csv_number(row->gain_corr_a - 1.0, 1e6);
        board_loop_csv_number(row->gain_corr_b - 1.0, 1e6);
        board_loop_csv_number(row->offset_a_state, 1e3);
        board_loop_csv_number(row->offset_b_state, 1e3);
        xil_printf(",%d,%s,%d,%d,%d", row->skew_code, action, row->skew_step_codes,
                   row->skew_batch_used, row->skew_batch_n);
        board_loop_csv_number(row->skew_batch_n ? row->skew_batch_mean_ps : NAN, 100.0);
        xil_printf(",%u,%d,%d", row->clipped_samples, row->prep.n0, row->prep.rotation);
        board_loop_csv_number(row->prep.align_margin, 100.0);
        xil_printf("\r\n");
}

static void board_loop_results_export(void)
{
    const board_loop_result_t *r = &g_board_loop_result;
    if (!r->ready) {
        xil_printf("LOOP-EXPORT-END result=ERROR reason=no-completed-run\r\n");
        return;
    }
    xil_printf("LOOP-EXPORT-BEGIN version=1 run_id=%u rows=%u total_rows=%u dropped=%u\r\n",
               r->run_id, r->stored, r->seen, r->seen - r->stored);
    board_loop_results_meta();
    board_loop_results_header();
    for (unsigned i = 0U; i < r->stored; ++i) board_loop_results_data(board_loop_result_row(i));
    xil_printf("LOOP-EXPORT-END result=OK run_id=%u rows=%u\r\n", r->run_id, r->stored);
}

#endif /* BUTILS_LOOP_FRAGMENT */
