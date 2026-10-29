/* Included by butils_loop.c. Experiments use the same production estimator and
 * actuator transaction as the ordinary loop. Compiled alone by Vitis: empty. */
#ifdef BUTILS_LOOP_FRAGMENT

#define LOOP_TEST_IFC 0x1910U
#define LOOP_TEST_INDEX 0x0008U
#define LOOP_TEST_DC1 0x0701U
#define LOOP_TEST_DC2 0x073BU

typedef struct {
    unsigned n;
    double mean;
    double m2;
} loop_test_stat_t;

typedef struct {
    unsigned attempts;
    unsigned accepted;
    loop_test_stat_t offset;
    loop_test_stat_t gain;
    loop_test_stat_t skew;
} loop_test_phase_t;

static adc_cal_loop_state_t g_loop_test_baseline;
static bool g_loop_test_baseline_valid;
static bool g_loop_test_snapshot_valid;
static bool g_loop_test_fault; /* a failed write is never retried */
static bool g_loop_test_full_rows; /* per-command PC data mode; default is compact */
static unsigned g_loop_test_wave;
static int g_loop_test_code;
static double g_loop_test_polarity;
static uint8_t g_loop_test_registers[2][3];
static const uint16_t k_loop_test_registers[3] = {
    LOOP_TEST_IFC, LOOP_TEST_DC1, LOOP_TEST_DC2
};
static double g_loop_test_offset[2]; /* software, AFTER the ADC */
static long g_loop_test_gain_ppm = 200000L;
static long g_loop_test_skew_milli = 250L;
static long g_loop_test_deadband_cps = 1000L;
static long g_loop_test_step_cps = 1910L;
static long g_loop_test_batch = 20L;

static int loop_test_number(const char *s, long lo, long hi, long *out)
{
    char *end = NULL;
    long v;
    if (s == NULL || *s == '\0') return -1;
    v = strtol(s, &end, 0);
    if (end == s || *end != '\0' || v < lo || v > hi) return -1;
    *out = v;
    return 0;
}

static void loop_test_add(loop_test_stat_t *s, double value)
{
    double delta;
    if (!board_loop_isfinite(value)) return;
    s->n++;
    delta = value - s->mean;
    s->mean += delta / (double)s->n;
    s->m2 += delta * (value - s->mean);
}

static double loop_test_sd(const loop_test_stat_t *s)
{
    return s->n > 1U ? sqrt(fmax(0.0, s->m2 / (double)(s->n - 1U))) : NAN;
}

static void loop_test_print_stats(const loop_test_phase_t *p, bool include_sd)
{
    board_loop_put("offset_mc", p->offset.n ? p->offset.mean : NAN, 1e3);
    if (include_sd) board_loop_put("offset_sd_mc", loop_test_sd(&p->offset), 1e3);
    board_loop_put("gain_ppm", p->gain.n ? p->gain.mean : NAN, 1e6);
    if (include_sd) board_loop_put("gain_sd_ppm", loop_test_sd(&p->gain), 1e6);
    board_loop_put("skew_cps", p->skew.n ? p->skew.mean : NAN, 100.0);
    if (include_sd) board_loop_put("skew_sd_cps", loop_test_sd(&p->skew), 100.0);
    xil_printf("\r\n");
}

/* Driver reads have no return code. Two differently initialised readbacks must
 * agree. Every local access saves, verifies and restores the bank selector. */
static int loop_test_reg(unsigned channel, uint16_t reg, uint8_t *value, bool write)
{
    uint8_t saved = 0x55U, saved2 = 0xAAU, check = 0U;
    uint8_t first = 0x55U, second = 0xAAU;
    int ok;
    ad9695_read_register(&spi_inst, LOOP_TEST_INDEX, &saved);
    ad9695_read_register(&spi_inst, LOOP_TEST_INDEX, &saved2);
    if (saved != saved2 || channel > 1U) return -1;
    ad9695_write_register(&spi_inst, LOOP_TEST_INDEX, (uint8_t)(1U << channel));
    ad9695_read_register(&spi_inst, LOOP_TEST_INDEX, &check);
    ok = check == (uint8_t)(1U << channel);
    if (ok) {
        if (write) ad9695_write_register(&spi_inst, reg, *value);
        ad9695_read_register(&spi_inst, reg, &first);
        ad9695_read_register(&spi_inst, reg, &second);
        ok = first == second && (!write || first == *value);
        if (!write) *value = first;
    }
    ad9695_write_register(&spi_inst, LOOP_TEST_INDEX, saved);
    ad9695_read_register(&spi_inst, LOOP_TEST_INDEX, &check);
    if (check != saved) ok = 0;
    if (!ok) g_loop_test_fault = true;
    return ok ? 0 : -1;
}

static int loop_test_snapshot(void)
{
    g_loop_test_snapshot_valid = false;
    for (unsigned c = 0U; c < 2U; ++c) {
        for (unsigned r = 0U; r < 3U; ++r) {
            if (loop_test_reg(c, k_loop_test_registers[r],
                              &g_loop_test_registers[c][r], false) != 0) return -1;
        }
    }
    g_loop_test_snapshot_valid = true;
    return 0;
}

static int loop_test_goto(int target)
{
    adc_cal_loop_skew_ack_t ack = {0, -1};
    if (target < 0 || target > 48 || g_loop_test_fault) return -1;
    (void)board_loop_request_steps(NULL, 0, &ack);
    if (!ack.ok || ack.code_after < 0 || ack.code_after > 48) goto failed;
    for (unsigned moves = 0U; ack.code_after != target && moves < 48U; ++moves) {
        const int previous = ack.code_after;
        const int delta = target > previous ? 1 : -1;
        ack.ok = 0;
        (void)board_loop_request_steps(NULL, delta, &ack);
        if (!ack.ok || ack.code_after != previous + delta) goto failed;
    }
    if (ack.code_after == target) return 0;
failed:
    g_loop_test_fault = true;
    return -1;
}

static void loop_test_configure(adc_cal_loop_options_t *opt,
                                 adc_cal_loop_actuator_t *act)
{
    adc_cal_loop_default_options(opt);
    adc_cal_loop_default_actuator(act);
    opt->max_gain_deviation = (double)g_loop_test_gain_ppm / 1e6;
    opt->max_skew_samples = (double)g_loop_test_skew_milli / 1000.0;
    opt->skew_batch_frames = (int)g_loop_test_batch;
    opt->reject_clipping = 1;
    opt->require_tone = 1;
    loop_waveform_options(g_board_loop_wave, opt);
    /* Explicit experiment gates apply to whichever observable was selected. */
    opt->max_skew_samples = (double)g_loop_test_skew_milli / 1000.0;
    opt->input_offset[0] = g_loop_test_offset[0];
    opt->input_offset[1] = g_loop_test_offset[1];
    act->deadband_ps = (double)g_loop_test_deadband_cps / 100.0;
    act->step_ps = (double)g_loop_test_step_cps / 100.0;
    act->request_steps = board_loop_request_steps;
}

static int loop_test_init(const adc_cal_loop_state_t *state)
{
    adc_cal_loop_config_t cfg;
    adc_cal_loop_options_t opt;
    adc_cal_loop_actuator_t act;
    loop_waveform_config(g_board_loop_wave, &cfg);
    loop_test_configure(&opt, &act);
    if (adc_skew_actuator_read_value(&act.code) != 0 || act.code < 0) return -1;
    g_board_loop_valid = false;
    g_board_loop_run_wave = g_board_loop_wave;
    if (adc_cal_loop_init(&g_board_loop, &cfg, &opt, state, &act) != 0) return -1;
    if (state != NULL) g_board_loop.polarity = g_loop_test_polarity;
    return 0;
}

/* A fixed ATTEMPT budget, including rejects. TRAIN and HOLD share state; HOLD
 * disables all feedback and uses fresh captures, never training residuals. */
static int loop_test_phase(const char *name, unsigned axes, unsigned frames,
                           loop_test_phase_t *out)
{
    unsigned failures = 0U;
    loop_test_phase_t window = {0};
    loop_test_stat_t freeze_a = {0}, freeze_b = {0};
    const bool refine_offset = strcmp(name, "baseline_train") == 0 ||
        (strcmp(name, "train") == 0 && axes == ADC_CAL_LOOP_AXIS_OFFSET);
    const unsigned freeze_frames = frames / 2U < 100U ? frames / 2U : 100U;
    const bool quiet = g_quiet_calibration_capture;
    memset(out, 0, sizeof(*out));
    g_board_loop.opt.update_axes = axes;
    g_board_loop.opt.close_skew_loop = (axes & ADC_CAL_LOOP_AXIS_SKEW) != 0U;
    g_board_loop.skew_frame_count = 0;
    g_quiet_calibration_capture = true;
    xil_printf("LOOP-TEST-PHASE phase=%s axes=%u budget=%u code=%d\r\n",
               name, axes, frames, g_board_loop.act.code);
    for (unsigned i = 0U; i < frames; ++i) {
        adc_cal_loop_row_t row;
        const adc_cal_loop_state_t before = g_board_loop.state;
        const int code_before = g_board_loop.act.code;
        const long got = board_loop_capture(NULL, g_board_loop_frame, sizeof(g_board_loop_frame));
        int rc;
        out->attempts++;
        if (got <= 0) {
            xil_printf("LOOP-TEST-CAPTURE phase=%s attempt=%u st=capture\r\n", name, i);
            if (++failures >= 3U) break;
            continue;
        }
        rc = adc_cal_loop_step(&g_board_loop, g_board_loop_frame, (size_t)got, &row);
        /* Estimate the input-domain DC from the PRE-update state. Average only
         * the training tail at one actuator code; HOLD remains independent.
         * Averaging residuals alone would be wrong because feedback is active. */
        if (refine_offset && i >= frames - freeze_frames) {
            if (code_before != g_board_loop.act.code) {
                memset(&freeze_a, 0, sizeof(freeze_a));
                memset(&freeze_b, 0, sizeof(freeze_b));
            } else if (rc == 0 && row.rejected == ADC_CAL_LOOP_REJECT_NONE &&
                       before.gain_corr_a > 0.0 && before.gain_corr_b > 0.0) {
                const double a = before.offset_a + row.est.ch_a.offset_codes / before.gain_corr_a;
                const double b = before.offset_b + row.est.ch_b.offset_codes / before.gain_corr_b;
                if (board_loop_isfinite(a) && board_loop_isfinite(b)) {
                    loop_test_add(&freeze_a, a);
                    loop_test_add(&freeze_b, b);
                }
            }
        }
        /* Commit on the last accepted training frame, before its post-update
         * state is exported. HOLD therefore starts at the exact recorded state.
         * A failed/rejected last attempt keeps the existing state unchanged. */
        if (refine_offset && i + 1U == frames && rc == 0 &&
            row.rejected == ADC_CAL_LOOP_REJECT_NONE &&
            !g_loop_test_fault && !g_board_loop.skew_abort &&
            freeze_a.n >= 20U && freeze_b.n == freeze_a.n) {
            g_board_loop.state.offset_a = row.offset_a_state = freeze_a.mean;
            g_board_loop.state.offset_b = row.offset_b_state = freeze_b.mean;
            xil_printf("LOOP-TEST-FREEZE source=dither_training_mean samples=%u code=%d phase=%s",
                       freeze_a.n, g_board_loop.act.code, name);
            board_loop_put("offa_mc", freeze_a.mean, 1e3);
            board_loop_put("offb_mc", freeze_b.mean, 1e3);
            xil_printf("\r\n");
        }
        if (g_loop_test_full_rows) {
            xil_printf("LOOP-TEST-ROW phase=%s attempt=%u code=%d", name, i, g_board_loop.act.code);
            board_loop_put("dcmean_mc", row.est.ch_b.offset_record_codes -
                           row.est.ch_a.offset_record_codes, 1e3);
            xil_printf(" ");
            board_loop_print_row(&row);
        }
        if (rc != 0 || row.rejected != ADC_CAL_LOOP_REJECT_NONE ||
            (row.skew_action != ADC_CAL_LOOP_SKEW_ACTION_NONE &&
             row.skew_action != ADC_CAL_LOOP_SKEW_ACTION_INSIDE_DEADBAND &&
             row.skew_action != ADC_CAL_LOOP_SKEW_ACTION_MEASURE_ONLY)) {
            xil_printf("LOOP-TEST-EVENT phase=%s attempt=%u st=%s act=%s code=%d clips=%u\r\n",
                       name, i, board_loop_reject_tag(row.rejected),
                       row.skew_action == ADC_CAL_LOOP_SKEW_ACTION_NONE ? "none" :
                           adc_cal_loop_skew_action_name(row.skew_action),
                       g_board_loop.act.code, row.clipped_samples);
        }
        if (rc != 0) {
            if (++failures >= 3U) break;
            continue;
        }
        failures = 0U;
        if (row.rejected == ADC_CAL_LOOP_REJECT_NONE) {
            const double offset = row.est.ch_b.offset_codes - row.est.ch_a.offset_codes;
            const double gain = g_board_loop.opt.tone_free ? row.est.gain_mag_ratio - 1.0 :
                                row.est.tone_ratio - 1.0;
            out->accepted++;
            loop_test_add(&out->offset, offset);
            loop_test_add(&out->gain, gain);
            loop_test_add(&out->skew, row.est.skew_used_ps);
            /* All phases report compact progress. These are diagnostics only:
             * never feed window statistics back into the calibration state. */
            {
                window.accepted++;
                loop_test_add(&window.offset, offset);
                loop_test_add(&window.gain, gain);
                loop_test_add(&window.skew, row.est.skew_used_ps);
                if (window.accepted == 20U) {
                    xil_printf("LOOP-TEST-WINDOW phase=%s through_attempt=%u accepted=%u",
                               name, i, window.accepted);
                    loop_test_print_stats(&window, false);
                    memset(&window, 0, sizeof(window));
                }
            }
        }
        if (row.skew_action == ADC_CAL_LOOP_SKEW_ACTION_MOVE_FAILED ||
            row.skew_action == ADC_CAL_LOOP_SKEW_ACTION_ABORT_DIRECTION) {
            g_loop_test_fault = true;
        }
        if (g_loop_test_fault || row.rejected == ADC_CAL_LOOP_REJECT_CLIPPING ||
            row.rejected == ADC_CAL_LOOP_REJECT_TONE) break;
    }
    g_quiet_calibration_capture = quiet;
    if (window.accepted != 0U) {
        xil_printf("LOOP-TEST-WINDOW phase=%s through_attempt=%u accepted=%u",
                   name, out->attempts - 1U, window.accepted);
        loop_test_print_stats(&window, false);
    }
    xil_printf("LOOP-TEST-SUMMARY phase=%s attempts=%u accepted=%u code=%d abort=%d",
               name, out->attempts, out->accepted, g_board_loop.act.code, g_board_loop.skew_abort);
    loop_test_print_stats(out, true);
    return out->attempts == frames && out->accepted != 0U &&
           !g_loop_test_fault && !g_board_loop.skew_abort ? 0 : -1;
}

static void loop_test_status(void)
{
    int current_code = -1;
    if (adc_skew_actuator_read_value(&current_code) != 0) current_code = -1;
    xil_printf("LOOP-TEST-BASELINE valid=%u snapshot=%u wave=%d code=%d fault=%u\r\n",
               (unsigned)g_loop_test_baseline_valid, (unsigned)g_loop_test_snapshot_valid,
               g_loop_test_baseline_valid ? (int)g_loop_test_wave : -1,
               g_loop_test_baseline_valid ? g_loop_test_code : -1, (unsigned)g_loop_test_fault);
    xil_printf("LOOP-TEST-CURRENT wave=%u waveform=%s code=%d\r\n",
               g_board_loop_wave, "reference_free_dither", current_code);
    xil_printf("LOOP-TEST-CONFIG gain_ppm=%ld skew_milli=%ld deadband_cps=%ld step_cps=%ld batch=%ld",
               g_loop_test_gain_ppm, g_loop_test_skew_milli, g_loop_test_deadband_cps,
               g_loop_test_step_cps, g_loop_test_batch);
    board_loop_put("software_offset_a_mc", g_loop_test_offset[0], 1e3);
    board_loop_put("software_offset_b_mc", g_loop_test_offset[1], 1e3);
    xil_printf("\r\n");
    xil_printf("LOOP-TEST-ROUTES gain=%s skew=%s tone_free=%u reference_free=1\r\n",
               "dither_spectrum",
               "dither_cross_spectrum",
               1U);
    if (g_loop_test_snapshot_valid) {
        for (unsigned c = 0U; c < 2U; ++c) {
            xil_printf("LOOP-TEST-SNAPSHOT channel=%u ifc=%u dc1=%u dc2=%u\r\n", c,
                       (unsigned)g_loop_test_registers[c][0],
                       (unsigned)g_loop_test_registers[c][1],
                       (unsigned)g_loop_test_registers[c][2]);
        }
    }
}

static int loop_test_baseline_check(const loop_test_phase_t *p, unsigned hold)
{
    const bool yield_ok = p->accepted * 5U >= hold * 4U;
    const bool complete = p->offset.n == p->accepted && p->gain.n == p->accepted &&
                          p->skew.n == p->accepted && p->skew.n > 1U;
    const double skew_limit = complete ?
        fmax(g_board_loop.act.deadband_ps, 0.5 * g_board_loop.act.step_ps) +
        3.0 * loop_test_sd(&p->skew) / sqrt((double)p->skew.n) : NAN;
    const bool offset_ok = complete && fabs(p->offset.mean) <= 0.5;
    const bool gain_ok = complete && fabs(p->gain.mean) <= 0.002;
    const bool skew_ok = complete && fabs(p->skew.mean) <= skew_limit;
    xil_printf("LOOP-TEST-BASELINE-CHECK kind=yield accepted=%u budget=%u required_percent=80 pass=%u\r\n",
               p->accepted, hold, (unsigned)yield_ok);
    xil_printf("LOOP-TEST-BASELINE-CHECK kind=finite-statistics pass=%u\r\n", (unsigned)complete);
    xil_printf("LOOP-TEST-BASELINE-CHECK axis=offset pass=%u", (unsigned)offset_ok);
    board_loop_put("mean_mc", p->offset.n ? p->offset.mean : NAN, 1e3);
    xil_printf(" limit_mc=500\r\n");
    xil_printf("LOOP-TEST-BASELINE-CHECK axis=gain pass=%u", (unsigned)gain_ok);
    board_loop_put("mean_ppm", p->gain.n ? p->gain.mean : NAN, 1e6);
    xil_printf(" limit_ppm=2000\r\n");
    xil_printf("LOOP-TEST-BASELINE-CHECK axis=skew pass=%u", (unsigned)skew_ok);
    board_loop_put("mean_cps", p->skew.n ? p->skew.mean : NAN, 100.0);
    board_loop_put("limit_cps", skew_limit, 100.0);
    xil_printf("\r\n");
    if (!yield_ok || !complete || !offset_ok || !gain_ok || !skew_ok) {
        xil_printf("LOOP-TEST-ERROR baseline-hold-out-of-spec; inspect BASELINE-CHECK and WINDOW rows.\r\n");
        return -1;
    }
    return 0;
}

static int loop_test_baseline(unsigned train, unsigned hold)
{
    loop_test_phase_t p;
    adc_cal_loop_skew_ack_t ack = {0, -1};
    g_loop_test_baseline_valid = false;
    if (g_loop_test_offset[0] != 0.0 || g_loop_test_offset[1] != 0.0) return -1;
    /* Includes neutral setup when required; a zero request does not park an
     * already initialised actuator at neutral. */
    (void)board_loop_request_steps(NULL, 0, &ack);
    if (!ack.ok) { g_loop_test_fault = true; return -1; }
    if (loop_test_snapshot() != 0 || loop_test_init(NULL) != 0) return -1;
    if (loop_test_phase("baseline_train", ADC_CAL_LOOP_AXIS_ALL, train, &p) != 0) return -1;
    if (loop_test_phase("baseline_hold", 0U, hold, &p) != 0) return -1;
    /* A baseline is usable, not merely a run that printed an END line. */
    if (loop_test_baseline_check(&p, hold) != 0) {
        loop_test_status();
        return -1;
    }
    g_loop_test_baseline = g_board_loop.state;
    g_loop_test_polarity = g_board_loop.polarity;
    g_loop_test_wave = g_board_loop_wave;
    g_loop_test_code = g_board_loop.act.code;
    g_loop_test_baseline_valid = true;
    loop_test_status();
    return 0;
}

static int loop_test_restore(void)
{
    if (!g_loop_test_snapshot_valid || !g_loop_test_baseline_valid) {
        xil_printf("LOOP-TEST-ERROR restore-no-valid-baseline valid=%u snapshot=%u; establish baseline first.\r\n",
                   (unsigned)g_loop_test_baseline_valid, (unsigned)g_loop_test_snapshot_valid);
        return -1;
    }
    if (g_loop_test_wave != g_board_loop_wave) {
        xil_printf("LOOP-TEST-ERROR restore-wave-mismatch saved=%u current=%u\r\n",
                   g_loop_test_wave, g_board_loop_wave);
        return -1;
    }
    for (unsigned c = 0U; c < 2U; ++c) {
        for (unsigned r = 0U; r < 3U; ++r) {
            uint8_t value = g_loop_test_registers[c][r];
            if (loop_test_reg(c, k_loop_test_registers[r], &value, true) != 0) return -1;
        }
    }
    g_loop_test_offset[0] = g_loop_test_offset[1] = 0.0;
    return loop_test_goto(g_loop_test_code);
}

/* Refuse unintended extra hardware perturbations on supposedly frozen axes. */
static int loop_test_check_isolation(unsigned axes)
{
    int code = -1;
    unsigned changed_channels = 0U;
    if (adc_skew_actuator_read_value(&code) != 0) return -1;
    if (axes != ADC_CAL_LOOP_AXIS_SKEW && code != g_loop_test_code) return -1;
    if (axes != ADC_CAL_LOOP_AXIS_OFFSET &&
        (g_loop_test_offset[0] != 0.0 || g_loop_test_offset[1] != 0.0)) return -1;
    for (unsigned c = 0U; c < 2U; ++c) {
        bool changed = axes == ADC_CAL_LOOP_AXIS_OFFSET && g_loop_test_offset[c] != 0.0;
        for (unsigned r = 0U; r < 3U; ++r) {
            uint8_t value;
            const bool allowed = (axes == ADC_CAL_LOOP_AXIS_GAIN && r == 0U) ||
                                 (axes == ADC_CAL_LOOP_AXIS_OFFSET && r != 0U);
            if (loop_test_reg(c, k_loop_test_registers[r], &value, false) != 0) return -1;
            if (!allowed && value != g_loop_test_registers[c][r]) return -1;
            if (allowed && value != g_loop_test_registers[c][r]) changed = true;
        }
        if (changed) changed_channels++;
    }
    return changed_channels <= 1U ? 0 : -1;
}

static int loop_test_run(unsigned axes, unsigned pre, unsigned train, unsigned hold)
{
    loop_test_phase_t p;
    if (!g_loop_test_baseline_valid || g_loop_test_wave != g_board_loop_wave ||
        loop_test_check_isolation(axes) != 0 || loop_test_init(&g_loop_test_baseline) != 0) return -1;
    if (loop_test_phase("pre", 0U, pre, &p) != 0) return -1;
    if (loop_test_phase("train", axes, train, &p) != 0) return -1;
    return loop_test_phase("hold", 0U, hold, &p);
}

static void loop_test_help(void)
{
    xil_printf("LOOP-TEST-HELP (reference-free dither-only; no waveform selection)\r\n");
    xil_printf("  adc -cal loop test baseline TRAIN HOLD   Calibrate all; save state, code and registers.\r\n");
    xil_printf("  adc -cal loop test run offset|gain|skew PRE TRAIN HOLD\r\n");
    xil_printf("  adc -cal loop test offset A|B CODES     SOFTWARE post-ADC offset, -4096..4096.\r\n");
    xil_printf("  adc -cal loop test ifc A|B CODE         Local 0x1910: 0,10,11,12,13,14,15.\r\n");
    xil_printf("  adc -cal loop test dcfilter A|B on|off  Local DC filter; NOT an offset DAC.\r\n");
    xil_printf("  adc -cal loop test park CODE           ACK-checked single moves, 0..48.\r\n");
    xil_printf("  adc -cal loop test step CODE FRAMES    N,N+1,N,N+1,N; returns to N.\r\n");
    xil_printf("  adc -cal loop test config GAIN_PPM SKEW_MILLI DEADBAND_CPS STEP_CPS BATCH\r\n");
    xil_printf("  adc -cal loop test restore             Restore baseline IFC/filter/code; clear injection.\r\n");
    xil_printf("  adc -cal loop test status              Show baseline and experiment settings.\r\n");
    xil_printf("  Budgets count attempted frames. HOLD freezes all feedback. Failed writes latch a fault.\r\n");
    xil_printf("  Compact output by default; append full to baseline/run/step for PC data capture.\r\n");
}

static void board_loop_test_command(void)
{
    char *args[8];
    unsigned n = 0U;
    char *s;
    long a = 0L, b = 0L, c = 0L;
    int rc = -1;
    g_loop_test_full_rows = false;
    while ((s = strtok(NULL, " ")) != NULL && n < 8U) args[n++] = s;
    if (n > 1U && strcmp(args[n - 1U], "full") == 0 &&
        (strcmp(args[0], "baseline") == 0 || strcmp(args[0], "run") == 0 ||
         strcmp(args[0], "step") == 0)) {
        g_loop_test_full_rows = true;
        n--;
    }
    if (s != NULL || n == 0U || (n == 1U && strcmp(args[0], "help") == 0)) {
        loop_test_help();
        xil_printf("LOOP-TEST-END result=%s\r\n", s == NULL ? "OK" : "ERROR");
        return;
    }
    if (n == 1U && strcmp(args[0], "status") == 0) {
        loop_test_status();
        rc = 0;
    } else if (g_loop_test_fault) {
        xil_printf("LOOP-TEST-ERROR recovery-required; restart firmware and repeat bring-up.\r\n");

    } else if (n == 3U && strcmp(args[0], "baseline") == 0 &&
               loop_test_number(args[1], 1L, BOARD_LOOP_MAX_SAMPLES, &a) == 0 &&
               loop_test_number(args[2], 2L, BOARD_LOOP_MAX_SAMPLES, &b) == 0) {
        rc = loop_test_baseline((unsigned)a, (unsigned)b);
    } else if (n == 5U && strcmp(args[0], "run") == 0 &&
               loop_test_number(args[2], 2L, BOARD_LOOP_MAX_SAMPLES, &a) == 0 &&
               loop_test_number(args[3], 1L, BOARD_LOOP_MAX_SAMPLES, &b) == 0 &&
               loop_test_number(args[4], 2L, BOARD_LOOP_MAX_SAMPLES, &c) == 0) {
        const unsigned axes = strcmp(args[1], "offset") == 0 ? ADC_CAL_LOOP_AXIS_OFFSET :
                              strcmp(args[1], "gain") == 0 ? ADC_CAL_LOOP_AXIS_GAIN :
                              strcmp(args[1], "skew") == 0 ? ADC_CAL_LOOP_AXIS_SKEW : 0U;
        if (axes != 0U) rc = loop_test_run(axes, (unsigned)a, (unsigned)b, (unsigned)c);
    } else if (n == 1U && strcmp(args[0], "restore") == 0) {
        rc = loop_test_restore();
    } else if (n == 2U && strcmp(args[0], "park") == 0 &&
               loop_test_number(args[1], 0L, 48L, &a) == 0) {
        rc = loop_test_goto((int)a);
    } else if (n == 3U && strcmp(args[0], "step") == 0 &&
               loop_test_number(args[1], 0L, 47L, &a) == 0 &&
               loop_test_number(args[2], 2L, BOARD_LOOP_MAX_SAMPLES, &b) == 0 &&
               g_loop_test_baseline_valid && g_loop_test_wave == g_board_loop_wave &&
               loop_test_check_isolation(ADC_CAL_LOOP_AXIS_SKEW) == 0) {
        const int offsets[5] = {0, 1, 0, 1, 0};
        const char *names[5] = {"step0", "step1", "step2", "step3", "step4"};
        rc = 0;
        for (unsigned i = 0U; i < 5U; ++i) {
            loop_test_phase_t p;
            if (loop_test_goto((int)a + offsets[i]) != 0 ||
                loop_test_init(&g_loop_test_baseline) != 0 ||
                loop_test_phase(names[i], 0U, (unsigned)b, &p) != 0) {
                rc = -1;
                break;
            }
        }
    } else if (n == 6U && strcmp(args[0], "config") == 0) {
        long values[5];
        const long lo[5] = {1L, 1L, 0L, 1L, 2L};
        const long hi[5] = {2000000L, 500L, 100000L, 100000L, ADC_CAL_LOOP_MAX_BATCH};
        rc = 0;
        for (unsigned i = 0U; i < 5U; ++i) {
            if (loop_test_number(args[i + 1U], lo[i], hi[i], &values[i]) != 0) rc = -1;
        }
        if (rc == 0) {
            g_loop_test_gain_ppm = values[0]; g_loop_test_skew_milli = values[1];
            g_loop_test_deadband_cps = values[2]; g_loop_test_step_cps = values[3];
            g_loop_test_batch = values[4];
            loop_test_status();
        }
    } else if (n == 3U && (strcmp(args[1], "A") == 0 || strcmp(args[1], "B") == 0)) {
        const unsigned ch = strcmp(args[1], "B") == 0 ? 1U : 0U;
        if (strcmp(args[0], "offset") == 0 && loop_test_number(args[2], -4096L, 4096L, &a) == 0) {
            g_loop_test_offset[ch] = (double)a;
            xil_printf("LOOP-TEST-INJECT kind=software-offset channel=%u codes=%ld\r\n", ch, a);
            rc = 0;
        } else if (g_loop_test_snapshot_valid && g_loop_test_baseline_valid &&
                   g_loop_test_wave == g_board_loop_wave && strcmp(args[0], "ifc") == 0 &&
                   loop_test_number(args[2], 0L, 15L, &a) == 0 && (a == 0L || a >= 10L)) {
            uint8_t value;
            if (loop_test_reg(ch, LOOP_TEST_IFC, &value, false) == 0) {
                value = (uint8_t)((value & 0xF0U) | (uint8_t)a);
                rc = loop_test_reg(ch, LOOP_TEST_IFC, &value, true);
                xil_printf("LOOP-TEST-INJECT kind=ifc channel=%u code=%u\r\n", ch, (unsigned)value);
            }
        } else if (g_loop_test_snapshot_valid && g_loop_test_baseline_valid &&
                   g_loop_test_wave == g_board_loop_wave && strcmp(args[0], "dcfilter") == 0 &&
                   (strcmp(args[2], "on") == 0 || strcmp(args[2], "off") == 0)) {
            uint8_t r1, r2;
            if (loop_test_reg(ch, LOOP_TEST_DC1, &r1, false) == 0 &&
                loop_test_reg(ch, LOOP_TEST_DC2, &r2, false) == 0) {
                const bool enable = strcmp(args[2], "on") == 0;
                r1 = enable ? (uint8_t)(r1 | 0x80U) : (uint8_t)(r1 & 0x7FU);
                if (enable) r2 = (uint8_t)(r2 & 0x7FU);
                if (loop_test_reg(ch, LOOP_TEST_DC1, &r1, true) == 0)
                    rc = loop_test_reg(ch, LOOP_TEST_DC2, &r2, true);
                xil_printf("LOOP-TEST-INJECT kind=dcfilter channel=%u enabled=%u\r\n", ch, (unsigned)enable);
            }
        }
    }
    if (rc != 0) {
        xil_printf("LOOP-TEST-ERROR invalid arguments, unavailable baseline, isolation, capture, or write failure; see preceding rows.\r\n");
    }
    g_loop_test_full_rows = false;
    xil_printf("LOOP-TEST-END result=%s\r\n", rc == 0 ? "OK" : "ERROR");
}

#endif /* BUTILS_LOOP_FRAGMENT */
