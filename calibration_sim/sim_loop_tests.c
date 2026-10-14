#include "sim_loop_tests.h"

#include "adc_calibration_loop.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LT_PI 3.14159265358979323846
#define LT_ADC_FULL_SCALE 8191.0
#define LT_WORDS 2048

static adc_cal_loop_t g_loop; /* ~300 KB: static, never on the stack */

static int lt_isfinite(double v)
{
    return v == v && v <= DBL_MAX && v >= -DBL_MAX;
}

/* ------------------------------------------------------------------------ */
/* C version of calibration_loop.simulate.BenchModel.  Same physics, its own  */
/* RNG, so it is a behavioural twin rather than a bit-exact one.              */
/* ------------------------------------------------------------------------ */

typedef struct {
    const adc_cal_loop_config_t *cfg;
    double gain_a;
    double gain_b;
    double offset_a;
    double offset_b;
    double skew_a_ps;
    double skew_b_ps;
    double noise;
    double adc_scale;
    int invert_b;
    double phase;
    double subsample;
    double skew_cmd_b_ps;
    uint64_t rng;
    int last_rotation;
    int noise_only;
    /* stub actuator (tools/loop_direction_check.py) */
    double direction;
    double step_ps;
    double commanded_ps;
    int code;
    int moves;
} lt_bench_t;

static uint64_t lt_next(lt_bench_t *b)
{
    b->rng ^= b->rng >> 12;
    b->rng ^= b->rng << 25;
    b->rng ^= b->rng >> 27;
    return b->rng * 2685821657736338717ULL;
}

static double lt_uniform(lt_bench_t *b)
{
    return ((double)(lt_next(b) >> 11) + 0.5) / 9007199254740992.0;
}

static double lt_gauss(lt_bench_t *b)
{
    const double u1 = lt_uniform(b);
    const double u2 = lt_uniform(b);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * LT_PI * u2);
}

static int lt_randint(lt_bench_t *b, int hi)
{
    return (int)(lt_next(b) % (uint64_t)hi);
}

static void lt_bench_init(lt_bench_t *b, const adc_cal_loop_config_t *cfg, uint64_t seed)
{
    memset(b, 0, sizeof(*b));
    b->cfg = cfg;
    b->gain_a = 1.0;
    b->gain_b = 1.021;
    b->offset_a = 14.0;
    b->offset_b = -23.0;
    b->skew_b_ps = 3.6;
    b->noise = 3.0;
    b->adc_scale = LT_ADC_FULL_SCALE / ADC_CAL_LOOP_DAC_FULL_SCALE;
    b->invert_b = 1;
    b->rng = seed * 0x9E3779B97F4A7C15ULL + 1ULL;
    b->phase = (double)lt_randint(b, adc_cal_loop_n_adc_period(cfg));
    b->subsample = 0.37;
    b->direction = 1.0;
    b->step_ps = 19.1;
    b->code = 24;
}

static double lt_analog(const lt_bench_t *b, double t)
{
    const adc_cal_loop_config_t *cfg = b->cfg;
    const double L = (double)adc_cal_loop_n_adc_period(cfg);
    const int slot_p = cfg->dither_period_dac / cfg->adc_ratio;
    const double er = (double)cfg->dither_edge_dac / (double)cfg->adc_ratio;
    const double tw = (double)cfg->dither_top_dac / (double)cfg->adc_ratio;
    const double off = (double)cfg->dither_position_dac / (double)cfg->adc_ratio;
    double tm = fmod(t, L);
    int slot;
    double tone;
    double dith;

    if (tm < 0.0) {
        tm += L;
    }
    tone = pow(10.0, cfg->amp_dbfs / 20.0) * ADC_CAL_LOOP_DAC_FULL_SCALE *
           sin(2.0 * LT_PI * (double)cfg->sig_cycles * tm / L);
    slot = ((int)floor(tm / (double)slot_p)) % cfg->n_events;
    dith = (double)cfg->polarity[slot] * cfg->dither_scale_lsb *
           adc_cal_loop_pulse(tm - (double)(slot * slot_p) - off, er, tw);
    return tone + dith;
}

static long lt_capture(void *ctx, uint8_t *buf, size_t capacity)
{
    lt_bench_t *b = (lt_bench_t *)ctx;
    const double ts_ps = 1e12 / adc_cal_loop_fs_adc(b->cfg);
    const int n_samples = LT_WORDS / 2;
    int16_t words[LT_WORDS];
    int rot;

    if (capacity < 2U * LT_WORDS) {
        return -1;
    }
    for (int i = 0; i < n_samples; ++i) {
        const double base = b->phase + b->subsample + (double)i;
        double va;
        double vb;
        if (b->noise_only) {
            va = 40.0 * lt_gauss(b);
            vb = 40.0 * lt_gauss(b);
        } else {
            va = b->gain_a * b->adc_scale * lt_analog(b, base + b->skew_a_ps / ts_ps) +
                 b->offset_a + b->noise * lt_gauss(b);
            vb = b->gain_b * b->adc_scale *
                     lt_analog(b, base + (b->skew_b_ps + b->skew_cmd_b_ps) / ts_ps) +
                 b->offset_b + b->noise * lt_gauss(b);
        }
        if (b->invert_b) {
            vb = -vb;
        }
        va = fmin(fmax(nearbyint(va), -LT_ADC_FULL_SCALE - 1.0), LT_ADC_FULL_SCALE);
        vb = fmin(fmax(nearbyint(vb), -LT_ADC_FULL_SCALE - 1.0), LT_ADC_FULL_SCALE);
        words[8 * (i / 4) + (i % 4)] = (int16_t)(va * 4.0);
        words[8 * (i / 4) + 4 + (i % 4)] = (int16_t)(vb * 4.0);
    }
    rot = lt_randint(b, 8);
    b->last_rotation = rot;
    for (int i = 0; i < LT_WORDS; ++i) {
        const uint16_t w = (uint16_t)words[i];
        const int j = (i + rot) % LT_WORDS; /* np.roll(packed, rot) */
        buf[2 * j] = (uint8_t)(w & 0xFFU);
        buf[2 * j + 1] = (uint8_t)(w >> 8);
    }
    b->phase = fmod(b->phase + (double)n_samples + (double)lt_randint(b, 64),
                    (double)adc_cal_loop_n_adc_period(b->cfg));
    return 2L * LT_WORDS;
}

static void lt_command_skew(void *ctx, double delay_ps)
{
    ((lt_bench_t *)ctx)->skew_cmd_b_ps = delay_ps;
}

static int lt_request_steps(void *ctx, int delta, adc_cal_loop_skew_ack_t *ack)
{
    lt_bench_t *b = (lt_bench_t *)ctx;
    b->code += delta;
    if (b->code < 0) {
        b->code = 0;
    }
    if (b->code > 48) {
        b->code = 48;
    }
    b->commanded_ps += b->direction * (double)delta * b->step_ps;
    b->skew_cmd_b_ps = b->commanded_ps;
    b->moves++;
    ack->ok = 1;
    ack->code_after = b->code;
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Native tests                                                               */
/* ------------------------------------------------------------------------ */

static void test_config_and_pulse(sim_assert_context_t *ctx)
{
    adc_cal_loop_config_t cfg;
    const char *why = NULL;
    int sum = 0;

    adc_cal_loop_default_config(&cfg);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_loop_config_validate(&cfg, &why), 0);
    for (int k = 0; k < cfg.n_events; ++k) {
        sum += cfg.polarity[k];
    }
    SIM_ASSERT_EQ_INT(ctx, sum, 0);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_loop_n_adc_period(&cfg), 8320);
    SIM_ASSERT_NEAR(ctx, adc_cal_loop_f_sig(&cfg), 199.375e6, 1.0);

    /* 1280 cycles puts every impulse on the same tone phase: validate() rejects it */
    cfg.sig_cycles = 1280;
    SIM_ASSERT_TRUE(ctx, adc_cal_loop_config_validate(&cfg, &why) != 0);
    adc_cal_loop_default_config(&cfg);
    cfg.polarity[0] = (int8_t)-cfg.polarity[0];
    SIM_ASSERT_TRUE(ctx, adc_cal_loop_config_validate(&cfg, &why) != 0);

    SIM_ASSERT_NEAR(ctx, adc_cal_loop_pulse(0.0, 8.0, 16.0), 0.0, 1e-15);
    SIM_ASSERT_NEAR(ctx, adc_cal_loop_pulse(4.0, 8.0, 16.0), 0.5, 1e-12);
    SIM_ASSERT_NEAR(ctx, adc_cal_loop_pulse(8.0, 8.0, 16.0), 1.0, 1e-15);
    SIM_ASSERT_NEAR(ctx, adc_cal_loop_pulse(28.0, 8.0, 16.0), 0.5, 1e-12);
    SIM_ASSERT_NEAR(ctx, adc_cal_loop_pulse(32.0, 8.0, 16.0), 0.0, 1e-15);
    SIM_ASSERT_NEAR(ctx, adc_cal_loop_pulse_derivative(4.0, 8.0, 16.0), LT_PI / 16.0, 1e-12);
    SIM_ASSERT_NEAR(ctx, adc_cal_loop_pulse_derivative(28.0, 8.0, 16.0), -LT_PI / 16.0, 1e-12);
}

static void test_fit_tone(sim_assert_context_t *ctx)
{
    static double y[1020];
    const double f_nom = 1276.0 / 8320.0;
    const double f_true = f_nom * (1.0 + 2e-5);
    adc_cal_loop_tone_fit_t fit;

    for (int i = 0; i < 1020; ++i) {
        const double w = 2.0 * LT_PI * f_true * (double)i;
        y[i] = 300.0 * cos(w) - 200.0 * sin(w) + 12.5;
    }
    SIM_ASSERT_EQ_INT(ctx, adc_cal_loop_fit_tone(y, 1020U, f_nom, 1, &fit), 0);
    SIM_ASSERT_NEAR(ctx, fit.f0, f_true, 1e-9);
    SIM_ASSERT_NEAR(ctx, fit.a, 300.0, 1e-3);
    SIM_ASSERT_NEAR(ctx, fit.b, -200.0, 1e-3);
    SIM_ASSERT_NEAR(ctx, fit.dc, 12.5, 1e-3);
    SIM_ASSERT_NEAR(ctx, fit.amplitude, hypot(300.0, 200.0), 1e-3);
}

static void test_prepare_capture(sim_assert_context_t *ctx)
{
    adc_cal_loop_config_t cfg;
    lt_bench_t bench;
    uint8_t raw[2 * LT_WORDS];
    adc_cal_loop_prep_t prep;
    int rot_ok = 0;
    int n0_ok = 0;
    int inv_ok = 0;
    const int trials = 16;

    adc_cal_loop_default_config(&cfg);
    SIM_ASSERT_EQ_INT(ctx, adc_cal_loop_init(&g_loop, &cfg, NULL, NULL, NULL), 0);
    lt_bench_init(&bench, &cfg, 3U);
    for (int t = 0; t < trials; ++t) {
        const double phase = bench.phase + bench.subsample;
        (void)lt_capture(&bench, raw, sizeof(raw));
        if (adc_cal_loop_prepare_capture(&g_loop, raw, sizeof(raw), &prep) != 0) {
            continue;
        }
        rot_ok += (prep.rotation == bench.last_rotation) && !prep.lagged;
        inv_ok += (prep.corr < 0.0) && (prep.sign_a > 0.0);
        {
            /* Sample 0 sits at loop position `phase`.  The polarity pattern of
             * the ~8 visible events recurs in the 64-event sequence, so n0 is
             * only determined up to whole 130-sample slots; within a slot it
             * must be within a sample. */
            double d = fmod((double)prep.n0 - phase, 130.0);
            if (d > 65.0) d -= 130.0;
            if (d < -65.0) d += 130.0;
            n0_ok += fabs(d) <= 1.0;
        }
    }
    SIM_ASSERT_EQ_INT(ctx, rot_ok, trials);
    SIM_ASSERT_EQ_INT(ctx, inv_ok, trials);
    SIM_ASSERT_EQ_INT(ctx, n0_ok, trials);
}

static void test_closed_loop_model(sim_assert_context_t *ctx, FILE *summary)
{
    adc_cal_loop_config_t cfg;
    adc_cal_loop_actuator_t act;
    lt_bench_t bench;
    static uint8_t buf[2 * LT_WORDS];
    unsigned got;
    double skew_residual;

    adc_cal_loop_default_config(&cfg);
    lt_bench_init(&bench, &cfg, 7U);
    bench.gain_b = 1.03;
    bench.offset_b = -40.0;
    bench.skew_b_ps = 25.0;
    adc_cal_loop_default_actuator(&act);
    act.ctx = &bench;
    act.command_skew_ps = lt_command_skew;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_loop_init(&g_loop, &cfg, NULL, NULL, &act), 0);

    got = adc_cal_loop_run(&g_loop, 40U, lt_capture, &bench, NULL, NULL, buf, sizeof(buf));
    skew_residual = bench.skew_b_ps + bench.skew_cmd_b_ps - bench.skew_a_ps;
    fprintf(summary,
            "loop model: %u qualified / %u captures, gain_b/gain_a applied %.5f (ideal %.5f), "
            "offset_a %.3f (14), offset_b %.3f (-40), skew residual %+.3f ps\n",
            got, g_loop.captures, g_loop.state.gain_corr_b / g_loop.state.gain_corr_a,
            1.0 / 1.03, g_loop.state.offset_a, g_loop.state.offset_b, skew_residual);
    SIM_ASSERT_EQ_INT(ctx, got, 40);
    SIM_ASSERT_NEAR(ctx, g_loop.state.gain_corr_b / g_loop.state.gain_corr_a, 1.0 / 1.03, 2e-3);
    SIM_ASSERT_NEAR(ctx, g_loop.state.offset_a, 14.0, 1.0);
    SIM_ASSERT_NEAR(ctx, g_loop.state.offset_b, -40.0, 1.0);
    SIM_ASSERT_NEAR(ctx, skew_residual, 0.0, 2.0);
}

static void test_batch_decision(sim_assert_context_t *ctx, FILE *summary, double direction)
{
    adc_cal_loop_config_t cfg;
    adc_cal_loop_options_t opt;
    adc_cal_loop_actuator_t act;
    lt_bench_t bench;
    static uint8_t buf[2 * LT_WORDS];
    unsigned got;
    double residual;

    adc_cal_loop_default_config(&cfg);
    adc_cal_loop_default_options(&opt);
    opt.skew_batch_frames = 5;
    lt_bench_init(&bench, &cfg, 11U);
    bench.skew_b_ps = 45.0;
    bench.noise = 2.0;
    bench.direction = direction;
    adc_cal_loop_default_actuator(&act);
    act.ctx = &bench;
    act.request_steps = lt_request_steps;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_loop_init(&g_loop, &cfg, &opt, NULL, &act), 0);

    got = adc_cal_loop_run(&g_loop, 40U, lt_capture, &bench, NULL, NULL, buf, sizeof(buf));
    residual = bench.skew_b_ps + bench.skew_cmd_b_ps;
    fprintf(summary, "loop batch %s: %u qualified, %d moves, code %d, abort %d, residual %+.2f ps\n",
            direction > 0 ? "honest" : "lying", got, bench.moves, g_loop.act.code,
            g_loop.skew_abort, residual);
    SIM_ASSERT_EQ_INT(ctx, got, 40);
    if (direction > 0) {
        SIM_ASSERT_EQ_INT(ctx, g_loop.skew_abort, 0);
        SIM_ASSERT_TRUE(ctx, bench.moves >= 2);
        SIM_ASSERT_TRUE(ctx, fabs(residual) <= g_loop.act.deadband_ps + 2.0);
    } else {
        SIM_ASSERT_EQ_INT(ctx, g_loop.skew_abort, 1);
        SIM_ASSERT_EQ_INT(ctx, bench.moves, 1);
    }
    SIM_ASSERT_EQ_INT(ctx, g_loop.act.code, bench.code);
}

static void test_reject_streak(sim_assert_context_t *ctx)
{
    adc_cal_loop_config_t cfg;
    lt_bench_t bench;
    static uint8_t buf[2 * LT_WORDS];
    unsigned got;

    adc_cal_loop_default_config(&cfg);
    lt_bench_init(&bench, &cfg, 5U);
    bench.noise_only = 1;
    SIM_ASSERT_EQ_INT(ctx, adc_cal_loop_init(&g_loop, &cfg, NULL, NULL, NULL), 0);
    got = adc_cal_loop_run(&g_loop, 10U, lt_capture, &bench, NULL, NULL, buf, sizeof(buf));
    SIM_ASSERT_EQ_INT(ctx, got, 0);
    SIM_ASSERT_EQ_INT(ctx, g_loop.captures, 20);
    SIM_ASSERT_EQ_INT(ctx, g_loop.rejected, 20);
    SIM_ASSERT_NEAR(ctx, g_loop.state.gain_corr_a, 1.0, 0.0);
    SIM_ASSERT_NEAR(ctx, g_loop.state.offset_b, 0.0, 0.0);
}

void sim_loop_tests_run_native(sim_assert_context_t *ctx, FILE *summary)
{
    fprintf(summary, "\n== calibration_loop_c native tests ==\n");
    test_config_and_pulse(ctx);
    test_fit_tone(ctx);
    test_prepare_capture(ctx);
    test_closed_loop_model(ctx, summary);
    test_batch_decision(ctx, summary, +1.0);
    test_batch_decision(ctx, summary, -1.0);
    test_reject_streak(ctx);
}

/* ------------------------------------------------------------------------ */
/* Golden replay                                                              */
/* ------------------------------------------------------------------------ */

enum {
    G_REJECTED, G_ROTATION, G_N0, G_SWAPPED, G_MARGIN, G_POLARITY,
    G_OFFSET_A, G_OFFSET_B, G_GAIN_A, G_GAIN_B, G_GAIN_RATIO, G_TONE_RATIO,
    G_TONE_AMP_A, G_RESID_A, G_SKEW_PHASE, G_SKEW_DIFF, G_SKEW_CENTROID, G_SKEW_MISMATCH,
    G_SKEW_SOURCE, G_ST_OFFSET_A, G_ST_OFFSET_B, G_ST_GAIN_A, G_ST_GAIN_B, G_ST_SKEW_CMD,
    G_SKEW_ACTION, G_SKEW_STEP, G_COUNT
};

static const char *const k_field_names[G_COUNT] = {
    "rejected", "rotation", "n0", "swapped", "align_margin", "polarity",
    "offset_a", "offset_b", "gain_a", "gain_b", "gain_ratio", "tone_ratio",
    "tone_amp_a", "resid_rms_a", "skew_phase_ps", "skew_diff_route_ps", "skew_centroid_ps",
    "skew_mismatch_ps", "skew_source", "state.offset_a", "state.offset_b", "state.gain_corr_a",
    "state.gain_corr_b", "state.skew_cmd_ps", "skew_action", "skew_step_codes"
};

static int is_exact_field(int f)
{
    return f == G_REJECTED || f == G_ROTATION || f == G_N0 || f == G_SWAPPED ||
           f == G_POLARITY || f == G_SKEW_SOURCE || f == G_SKEW_ACTION || f == G_SKEW_STEP;
}

static int source_code(adc_cal_loop_skew_source_t s)
{
    return (int)s;
}

static int action_code(adc_cal_loop_skew_action_t a)
{
    return (int)a; /* enum order matches export_golden.action_code */
}

static void row_values(const adc_cal_loop_t *loop, const adc_cal_loop_row_t *row,
                       double v[G_COUNT])
{
    v[G_REJECTED] = row->rejected != ADC_CAL_LOOP_REJECT_NONE ? 1.0 : 0.0;
    v[G_ROTATION] = row->prep.rotation;
    v[G_N0] = row->prep.n0;
    v[G_SWAPPED] = row->prep.lagged;
    v[G_MARGIN] = row->prep.align_margin;
    v[G_POLARITY] = row->polarity;
    v[G_OFFSET_A] = row->est.ch_a.offset_codes;
    v[G_OFFSET_B] = row->est.ch_b.offset_codes;
    v[G_GAIN_A] = row->est.ch_a.gain_codes;
    v[G_GAIN_B] = row->est.ch_b.gain_codes;
    v[G_GAIN_RATIO] = row->est.gain_ratio;
    v[G_TONE_RATIO] = row->est.tone_ratio;
    v[G_TONE_AMP_A] = row->est.ch_a.tone_amplitude;
    v[G_RESID_A] = row->est.ch_a.residual_rms;
    v[G_SKEW_PHASE] = row->est.skew_phase_ps;
    v[G_SKEW_DIFF] = row->est.skew_diff_route_ps;
    v[G_SKEW_CENTROID] = row->est.skew_centroid_ps;
    v[G_SKEW_MISMATCH] = row->est.skew_mismatch_ps;
    v[G_SKEW_SOURCE] = source_code(row->est.skew_source);
    v[G_ST_OFFSET_A] = loop->state.offset_a;
    v[G_ST_OFFSET_B] = loop->state.offset_b;
    v[G_ST_GAIN_A] = loop->state.gain_corr_a;
    v[G_ST_GAIN_B] = loop->state.gain_corr_b;
    v[G_ST_SKEW_CMD] = loop->state.skew_cmd_ps;
    v[G_SKEW_ACTION] = action_code(row->skew_action);
    v[G_SKEW_STEP] = row->skew_step_codes;
}

typedef struct {
    int code;
} lt_replay_actuator_t;

static int lt_replay_request(void *ctx, int delta, adc_cal_loop_skew_ack_t *ack)
{
    lt_replay_actuator_t *r = (lt_replay_actuator_t *)ctx;
    r->code += delta;
    if (r->code < 0) r->code = 0;
    if (r->code > 48) r->code = 48;
    ack->ok = 1;
    ack->code_after = r->code;
    return 0;
}

static char g_line[1 << 16];
static uint8_t g_raw[ADC_CAL_LOOP_MAX_WORDS * 2];

static int read_line(FILE *fp)
{
    size_t n;
    if (fgets(g_line, (int)sizeof(g_line), fp) == NULL) {
        return -1;
    }
    n = strlen(g_line);
    while (n > 0 && (g_line[n - 1] == '\n' || g_line[n - 1] == '\r')) {
        g_line[--n] = '\0';
    }
    return 0;
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int parse_numbers(const char *text, double *out, int max)
{
    int n = 0;
    const char *p = text;
    while (n < max) {
        char *end = NULL;
        const double v = strtod(p, &end);
        if (end == p) {
            break;
        }
        out[n++] = v;
        p = end;
    }
    return n;
}

static int golden_case(sim_assert_context_t *ctx, FILE *summary, FILE *fp)
{
    char name[128];
    double num[ADC_CAL_LOOP_MAX_EVENTS + 16];
    adc_cal_loop_config_t cfg;
    adc_cal_loop_options_t opt;
    adc_cal_loop_actuator_t act;
    lt_replay_actuator_t replay;
    int batch;
    int n_frames;
    double max_dev[G_COUNT];
    int mismatches[G_COUNT];
    int frames_done = 0;
    int reported = 0;
    int n0_ties = 0;

    if (read_line(fp) != 0 || sscanf(g_line, "case %127s", name) != 1) return -1;

    memset(&cfg, 0, sizeof(cfg));
    if (read_line(fp) != 0 || strncmp(g_line, "config ", 7) != 0 ||
        parse_numbers(g_line + 7, num, 11) != 11) return -1;
    cfg.fs_dac = num[0];
    cfg.n_dac_points = (int)num[1];
    cfg.adc_ratio = (int)num[2];
    cfg.sig_cycles = (int)num[3];
    cfg.amp_dbfs = num[4];
    cfg.dither_period_dac = (int)num[5];
    cfg.dither_position_dac = (int)num[6];
    cfg.dither_edge_dac = (int)num[7];
    cfg.dither_top_dac = (int)num[8];
    cfg.dither_scale_lsb = num[9];
    cfg.n_events = (int)num[10];
    if (cfg.n_events > ADC_CAL_LOOP_MAX_EVENTS) return -1;
    if (read_line(fp) != 0 || strncmp(g_line, "polarity ", 9) != 0 ||
        parse_numbers(g_line + 9, num, cfg.n_events) != cfg.n_events) return -1;
    for (int k = 0; k < cfg.n_events; ++k) {
        cfg.polarity[k] = (int8_t)num[k];
    }

    if (read_line(fp) != 0 || strncmp(g_line, "options ", 8) != 0 ||
        parse_numbers(g_line + 8, num, 11) != 11) return -1;
    adc_cal_loop_default_options(&opt);
    adc_cal_loop_default_actuator(&act);
    batch = (int)num[0];
    opt.cancel_signal = (int)num[1];
    opt.close_skew_loop = (int)num[2];
    opt.skew_batch_frames = (int)num[3];
    opt.min_align_margin = num[4];
    opt.max_skew_samples = num[5];
    opt.max_gain_deviation = num[6];
    opt.skew_direction_tol_ps = num[7];
    act.step_ps = num[8];
    act.deadband_ps = num[9];
    act.code = (int)num[10];
    replay.code = act.code;
    if (batch) {
        act.ctx = &replay;
        act.request_steps = lt_replay_request;
    }
    if (adc_cal_loop_init(&g_loop, &cfg, &opt, NULL, &act) != 0) return -1;

    if (read_line(fp) != 0 || sscanf(g_line, "frames %d", &n_frames) != 1) return -1;
    for (int f = 0; f < G_COUNT; ++f) {
        max_dev[f] = 0.0;
        mismatches[f] = 0;
    }

    for (int i = 0; i < n_frames; ++i) {
        int n_bytes;
        size_t hex_len;
        double expect[G_COUNT];
        double got[G_COUNT];
        adc_cal_loop_row_t row;

        if (read_line(fp) != 0 || sscanf(g_line, "frame %d", &n_bytes) != 1 ||
            n_bytes <= 0 || (size_t)n_bytes > sizeof(g_raw)) return -1;
        if (read_line(fp) != 0) return -1;
        hex_len = strlen(g_line);
        if (hex_len != 2U * (size_t)n_bytes) return -1;
        for (int b = 0; b < n_bytes; ++b) {
            const int hi = hex_nibble(g_line[2 * b]);
            const int lo = hex_nibble(g_line[2 * b + 1]);
            if (hi < 0 || lo < 0) return -1;
            g_raw[b] = (uint8_t)(hi * 16 + lo);
        }
        if (read_line(fp) != 0 || strncmp(g_line, "expect ", 7) != 0 ||
            parse_numbers(g_line + 7, expect, G_COUNT) != G_COUNT) return -1;
        if (read_line(fp) != 0) return -1; /* "# reason" */

        /* Replay on the host's loop position; the C must show that position is
         * a tie with its own argmax (see adc_cal_loop_prepare_capture). */
        g_loop.n0_override = (int)expect[G_N0];
        (void)adc_cal_loop_step(&g_loop, g_raw, (size_t)n_bytes, &row);
        row_values(&g_loop, &row, got);
        if (row.prep.n0_natural != (int)expect[G_N0]) {
            if (row.prep.n0_score_ratio >= 1.0 - 1e-9) {
                n0_ties++;
            } else {
                fprintf(summary, "    frame %d: host n0 %d scores %.9f of C's n0 %d\n", i,
                        (int)expect[G_N0], row.prep.n0_score_ratio, row.prep.n0_natural);
                mismatches[G_N0]++;
            }
        }
        if (reported < 5 &&
            (got[G_N0] != expect[G_N0] ||
             fabs(got[G_OFFSET_A] - expect[G_OFFSET_A]) > 1e-6 * fmax(1.0, fabs(expect[G_OFFSET_A])))) {
            fprintf(summary,
                    "    frame %d: n0 %g/%g (C/py) margin %.6f/%.6f offset_a %.9g/%.9g "
                    "tone_a %.9g/%.9g\n",
                    i, got[G_N0], expect[G_N0], got[G_MARGIN], expect[G_MARGIN],
                    got[G_OFFSET_A], expect[G_OFFSET_A], got[G_TONE_AMP_A], expect[G_TONE_AMP_A]);
            reported++;
        }
        for (int f = 0; f < G_COUNT; ++f) {
            const int e_fin = lt_isfinite(expect[f]);
            const int g_fin = lt_isfinite(got[f]);
            if (e_fin != g_fin) {
                mismatches[f]++;
                continue;
            }
            if (!e_fin) {
                continue;
            }
            if (is_exact_field(f)) {
                if (got[f] != expect[f]) {
                    mismatches[f]++;
                }
            } else {
                const double dev = fabs(got[f] - expect[f]) / fmax(1.0, fabs(expect[f]));
                if (dev > max_dev[f]) {
                    max_dev[f] = dev;
                }
            }
        }
        frames_done++;
    }

    fprintf(summary, "loop golden %-15s %d frames (%s branch), %d n0 ties resolved to the host\n",
            name, frames_done, batch ? "batch" : "model", n0_ties);
    for (int f = 0; f < G_COUNT; ++f) {
        if (is_exact_field(f)) {
            fprintf(summary, "    %-20s mismatches %d\n", k_field_names[f], mismatches[f]);
            SIM_ASSERT_EQ_INT(ctx, mismatches[f], 0);
        } else {
            fprintf(summary, "    %-20s max rel dev %.3e  finite/NaN mismatches %d\n",
                    k_field_names[f], max_dev[f], mismatches[f]);
            SIM_ASSERT_EQ_INT(ctx, mismatches[f], 0);
            SIM_ASSERT_TRUE(ctx, max_dev[f] <= 1e-6);
        }
    }
    return 0;
}

void sim_loop_tests_run_golden(sim_assert_context_t *ctx, FILE *summary, const char *path)
{
    FILE *fp = fopen(path, "r");
    int n_cases = 0;

    fprintf(summary, "\n== calibration_loop_c golden replay: %s ==\n", path);
    SIM_ASSERT_TRUE(ctx, fp != NULL);
    if (fp == NULL) {
        return;
    }
    if (read_line(fp) != 0 || strcmp(g_line, "LOOPGOLDEN 1") != 0 ||
        read_line(fp) != 0 || sscanf(g_line, "cases %d", &n_cases) != 1) {
        SIM_ASSERT_TRUE(ctx, 0 && "golden header");
        fclose(fp);
        return;
    }
    for (int c = 0; c < n_cases; ++c) {
        const int rc = golden_case(ctx, summary, fp);
        SIM_ASSERT_EQ_INT(ctx, rc, 0);
        if (rc != 0) {
            break;
        }
    }
    fclose(fp);
}
