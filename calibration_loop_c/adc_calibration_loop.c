#include "adc_calibration_loop.h"

#include <float.h>
#include <math.h>
#include <string.h>

/*
 * Port of calibration_loop/{dither,estimator,loop}.py, tone route.  Function
 * names follow the Python ones so the two can be read side by side; comments
 * only note where the C differs from a line-for-line translation.
 */

#define LOOP_PI 3.14159265358979323846

/* Function forms: the libm macros warn under -Wconversion on some toolchains. */
static int loop_isfinite(double value)
{
    return value == value && value <= DBL_MAX && value >= -DBL_MAX;
}

static int loop_isnan(double value)
{
    return value != value;
}

static const int8_t k_default_polarity[64] = {
    1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1, -1,
    -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1,
    1, 1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1, -1,
    -1, -1, 1, -1, 1, 1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1
};

/* ------------------------------------------------------------------------ */
/* DitherConfig                                                               */
/* ------------------------------------------------------------------------ */

void adc_cal_loop_default_config(adc_cal_loop_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->fs_dac = 2600.0e6;
    cfg->n_dac_points = 16640;
    cfg->adc_ratio = 2;
    cfg->sig_cycles = 1276;
    cfg->amp_dbfs = -1.5;
    cfg->dither_period_dac = 260;
    cfg->dither_position_dac = 96;
    cfg->dither_edge_dac = 16;
    cfg->dither_top_dac = 32;
    cfg->dither_scale_lsb = 2000.0;
    cfg->n_events = 64;
    memcpy(cfg->polarity, k_default_polarity, sizeof(k_default_polarity));
}

double adc_cal_loop_fs_adc(const adc_cal_loop_config_t *cfg)
{
    return cfg->fs_dac / (double)cfg->adc_ratio;
}

int adc_cal_loop_n_adc_period(const adc_cal_loop_config_t *cfg)
{
    return cfg->n_dac_points / cfg->adc_ratio;
}

double adc_cal_loop_f_sig(const adc_cal_loop_config_t *cfg)
{
    return (double)cfg->sig_cycles * cfg->fs_dac / (double)cfg->n_dac_points;
}

static int slot_period(const adc_cal_loop_config_t *cfg)
{
    return cfg->dither_period_dac / cfg->adc_ratio;
}

static double pulse_offset(const adc_cal_loop_config_t *cfg)
{
    return (double)cfg->dither_position_dac / (double)cfg->adc_ratio;
}

static double edge_r(const adc_cal_loop_config_t *cfg)
{
    return (double)cfg->dither_edge_dac / (double)cfg->adc_ratio;
}

static double top_w(const adc_cal_loop_config_t *cfg)
{
    return (double)cfg->dither_top_dac / (double)cfg->adc_ratio;
}

static double pulse_len(const adc_cal_loop_config_t *cfg)
{
    return 2.0 * edge_r(cfg) + top_w(cfg);
}

static double a_sine(const adc_cal_loop_config_t *cfg)
{
    return pow(10.0, cfg->amp_dbfs / 20.0);
}

static double a_dither(const adc_cal_loop_config_t *cfg)
{
    return cfg->dither_scale_lsb / ADC_CAL_LOOP_DAC_FULL_SCALE;
}

/* Template span [m_lo, m_hi] of adc_templates(cfg, guard=2). */
static int template_m_lo(void)
{
    return -2;
}

static int template_m_hi(const adc_cal_loop_config_t *cfg)
{
    return (int)ceil(pulse_len(cfg)) + 1;
}

int adc_cal_loop_config_validate(const adc_cal_loop_config_t *cfg, const char **reason)
{
    const char *why = NULL;
    const int ratio = cfg->adc_ratio;

    if (ratio < 1) {
        why = "adc_ratio must be a positive integer";
    } else if ((cfg->n_dac_points % ratio) || (cfg->dither_period_dac % ratio) ||
               (cfg->dither_edge_dac % ratio) || (cfg->dither_top_dac % ratio) ||
               (cfg->dither_position_dac % ratio)) {
        why = "DAC lengths must be multiples of adc_ratio";
    } else if (cfg->dither_period_dac <= 0 || (cfg->n_dac_points % cfg->dither_period_dac)) {
        why = "n_dac_points must be a whole number of dither periods";
    } else if (cfg->n_events != cfg->n_dac_points / cfg->dither_period_dac) {
        why = "n_events must equal n_dac_points / dither_period_dac";
    } else if (cfg->n_events > ADC_CAL_LOOP_MAX_EVENTS ||
               adc_cal_loop_n_adc_period(cfg) > ADC_CAL_LOOP_MAX_LOOP_SAMPLES ||
               template_m_hi(cfg) - template_m_lo() + 1 > ADC_CAL_LOOP_MAX_WINDOW) {
        why = "geometry exceeds the compiled-in buffer limits";
    } else if (pulse_offset(cfg) + pulse_len(cfg) > (double)slot_period(cfg)) {
        why = "pulse does not fit in its slot";
    } else if (cfg->n_events % 2) {
        why = "n_events must be even so the polarity can be exactly balanced";
    } else if (top_w(cfg) < 2.0) {
        why = "flat top shorter than 2 ADC samples";
    } else if (edge_r(cfg) < 2.0) {
        why = "edge shorter than 2 ADC samples";
    } else if (a_sine(cfg) + a_dither(cfg) > 1.0) {
        why = "main tone plus dither would clip the DAC";
    } else {
        int sum = 0;
        for (int k = 0; k < cfg->n_events; ++k) {
            if (cfg->polarity[k] != 1 && cfg->polarity[k] != -1) {
                why = "polarity entries must be +1 or -1";
                break;
            }
            sum += cfg->polarity[k];
        }
        if (why == NULL && sum != 0) {
            why = "polarity sequence is not balanced";
        }
    }

    if (why == NULL) {
        /* tone_phase_coherence(15) <= 0.3 */
        const double step = fmod((double)cfg->sig_cycles * (double)slot_period(cfg) /
                                 (double)adc_cal_loop_n_adc_period(cfg), 1.0);
        double re = 0.0;
        double im = 0.0;
        for (int k = 0; k < 15; ++k) {
            re += cos(2.0 * LOOP_PI * (double)k * step);
            im += sin(2.0 * LOOP_PI * (double)k * step);
        }
        if (hypot(re, im) / 15.0 > 0.3) {
            why = "tone phase step too coherent across events; pick another sig_cycles";
        }
    }

    if (reason != NULL) {
        *reason = why;
    }
    return why == NULL ? 0 : -1;
}

/* ------------------------------------------------------------------------ */
/* Pulse shape                                                                */
/* ------------------------------------------------------------------------ */

double adc_cal_loop_pulse(double t, double er, double tw)
{
    const double total = 2.0 * er + tw;
    if (t >= 0.0 && t < er) {
        return 0.5 * (1.0 - cos(LOOP_PI * t / er));
    }
    if (t >= er && t < er + tw) {
        return 1.0;
    }
    if (t >= er + tw && t < total) {
        return 0.5 * (1.0 - cos(LOOP_PI * (total - t) / er));
    }
    return 0.0;
}

double adc_cal_loop_pulse_derivative(double t, double er, double tw)
{
    const double total = 2.0 * er + tw;
    if (t >= 0.0 && t < er) {
        return (LOOP_PI / (2.0 * er)) * sin(LOOP_PI * t / er);
    }
    if (t >= er + tw && t < total) {
        return -(LOOP_PI / (2.0 * er)) * sin(LOOP_PI * (total - t) / er);
    }
    return 0.0;
}

/* ------------------------------------------------------------------------ */
/* Small numeric helpers                                                      */
/* ------------------------------------------------------------------------ */

static double mean_of(const double *x, int n)
{
    double s = 0.0;
    for (int i = 0; i < n; ++i) {
        s += x[i];
    }
    return n > 0 ? s / (double)n : NAN;
}

static void sort_doubles(double *x, int n)
{
    for (int i = 1; i < n; ++i) {
        const double v = x[i];
        int j = i - 1;
        while (j >= 0 && x[j] > v) {
            x[j + 1] = x[j];
            --j;
        }
        x[j + 1] = v;
    }
}

/* np.nanmedian over at most ADC_CAL_LOOP_MAX_BATCH values; NaN when none finite. */
static double nan_median(const double *x, int n)
{
    double tmp[ADC_CAL_LOOP_MAX_BATCH];
    int m = 0;
    for (int i = 0; i < n && m < ADC_CAL_LOOP_MAX_BATCH; ++i) {
        if (loop_isfinite(x[i])) {
            tmp[m++] = x[i];
        }
    }
    if (m == 0) {
        return NAN;
    }
    sort_doubles(tmp, m);
    return (m % 2) ? tmp[m / 2] : 0.5 * (tmp[m / 2 - 1] + tmp[m / 2]);
}

static double sign_of(double x)
{
    return (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0);
}

/* ------------------------------------------------------------------------ */
/* fit_tone                                                                   */
/* ------------------------------------------------------------------------ */

static int solve3(double m[3][3], double rhs[3], double out[3])
{
    for (int col = 0; col < 3; ++col) {
        int piv = col;
        for (int r = col + 1; r < 3; ++r) {
            if (fabs(m[r][col]) > fabs(m[piv][col])) {
                piv = r;
            }
        }
        if (fabs(m[piv][col]) < 1e-300) {
            return -1;
        }
        if (piv != col) {
            for (int c = 0; c < 3; ++c) {
                const double t = m[col][c];
                m[col][c] = m[piv][c];
                m[piv][c] = t;
            }
            {
                const double t = rhs[col];
                rhs[col] = rhs[piv];
                rhs[piv] = t;
            }
        }
        for (int r = col + 1; r < 3; ++r) {
            const double f = m[r][col] / m[col][col];
            for (int c = col; c < 3; ++c) {
                m[r][c] -= f * m[col][c];
            }
            rhs[r] -= f * rhs[col];
        }
    }
    for (int r = 2; r >= 0; --r) {
        double s = rhs[r];
        for (int c = r + 1; c < 3; ++c) {
            s -= m[r][c] * out[c];
        }
        out[r] = s / m[r][r];
    }
    return 0;
}

/* Least squares of y ~ a cos + b sin + dc via the 3x3 normal equations
 * (numpy uses an SVD; same answer for this well-conditioned basis). */
static int tone_solve(const double *y, size_t n, double freq, double coef[3], double *sse)
{
    const double w = 2.0 * LOOP_PI * freq;
    double m[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double rhs[3] = {0.0, 0.0, 0.0};
    double s = 0.0;

    for (size_t i = 0; i < n; ++i) {
        const double c = cos(w * (double)i);
        const double sn = sin(w * (double)i);
        m[0][0] += c * c;
        m[0][1] += c * sn;
        m[0][2] += c;
        m[1][1] += sn * sn;
        m[1][2] += sn;
        rhs[0] += y[i] * c;
        rhs[1] += y[i] * sn;
        rhs[2] += y[i];
    }
    m[1][0] = m[0][1];
    m[2][0] = m[0][2];
    m[2][1] = m[1][2];
    m[2][2] = (double)n;
    if (solve3(m, rhs, coef) != 0) {
        return -1;
    }
    for (size_t i = 0; i < n; ++i) {
        const double r = y[i] - (coef[0] * cos(w * (double)i) +
                                 coef[1] * sin(w * (double)i) + coef[2]);
        s += r * r;
    }
    *sse = s;
    return 0;
}

int adc_cal_loop_fit_tone(const double *y, size_t n, double f0, int refine,
                          adc_cal_loop_tone_fit_t *fit)
{
    double coef[3];
    double sse;

    if (y == NULL || fit == NULL || n < 4) {
        return -1;
    }
    if (tone_solve(y, n, f0, coef, &sse) != 0) {
        return -2;
    }

    if (refine) {
        double step = 0.25 / (double)n;
        for (int it = 0; it < 40; ++it) {
            int improved = 0;
            for (int side = 0; side < 2; ++side) {
                const double cand = (side == 0) ? f0 - step : f0 + step;
                double c2[3];
                double s2;
                if (cand <= 0.0 || cand >= 0.5) {
                    continue;
                }
                if (tone_solve(y, n, cand, c2, &s2) != 0) {
                    continue;
                }
                if (s2 < sse) {
                    f0 = cand;
                    memcpy(coef, c2, sizeof(coef));
                    sse = s2;
                    improved = 1;
                    break;
                }
            }
            if (!improved) {
                step *= 0.5;
                if (step < 1e-12) {
                    break;
                }
            }
        }
    }

    fit->f0 = f0;
    fit->a = coef[0];
    fit->b = coef[1];
    fit->dc = coef[2];
    fit->amplitude = hypot(coef[0], coef[1]);
    fit->phase = atan2(-coef[1], coef[0]);
    fit->sse = sse;
    return 0;
}

static double tone_at(const adc_cal_loop_tone_fit_t *fit, size_t i)
{
    const double w = 2.0 * LOOP_PI * fit->f0 * (double)i;
    return fit->a * cos(w) + fit->b * sin(w);
}

/* ------------------------------------------------------------------------ */
/* De-framing                                                                 */
/* ------------------------------------------------------------------------ */

static double corr_of(const double *x, const double *y, int n)
{
    double mx;
    double my;
    double sxy = 0.0;
    double sxx = 0.0;
    double syy = 0.0;
    double d;

    if (n < 8) {
        return 0.0;
    }
    mx = mean_of(x, n);
    my = mean_of(y, n);
    for (int i = 0; i < n; ++i) {
        const double a = x[i] - mx;
        const double b = y[i] - my;
        sxy += a * b;
        sxx += a * a;
        syy += b * b;
    }
    d = sqrt(sxx * syy);
    return d > 0.0 ? sxy / d : 0.0;
}

/* split_at(): returns samples per channel, or 0 when too short. */
static int split_at(const int32_t *words, int n_words, int rotation, double *a, double *b)
{
    const int body = ((n_words - rotation) / 8) * 8;
    if (body < 32) {
        return 0;
    }
    for (int g = 0; g < body / 8; ++g) {
        for (int j = 0; j < 4; ++j) {
            a[4 * g + j] = (double)words[rotation + 8 * g + j];
            b[4 * g + j] = (double)words[rotation + 8 * g + 4 + j];
        }
    }
    return body / 2;
}

/* _tone_purity(): fraction of AC power in the strongest bin of a Hann-windowed
 * spectrum.  Plain DFT; it is only evaluated when it can decide a tie. */
static double tone_purity(const double *x, int n, double *scratch)
{
    const double mu = mean_of(x, n);
    int any = 0;
    double total = 0.0;
    double peak = 0.0;

    for (int i = 0; i < n; ++i) {
        const double v = x[i] - mu;
        const double win = (n > 1) ? 0.5 - 0.5 * cos(2.0 * LOOP_PI * (double)i / (double)(n - 1))
                                   : 1.0;
        if (v != 0.0) {
            any = 1;
        }
        scratch[i] = v * win;
    }
    if (!any) {
        return 0.0;
    }
    for (int k = 1; k <= n / 2; ++k) {
        double re = 0.0;
        double im = 0.0;
        for (int i = 0; i < n; ++i) {
            const long idx = ((long)k * (long)i) % (long)n;
            const double ang = 2.0 * LOOP_PI * (double)idx / (double)n;
            re += scratch[i] * cos(ang);
            im -= scratch[i] * sin(ang);
        }
        {
            const double p = re * re + im * im;
            total += p;
            if (p > peak) {
                peak = p;
            }
        }
    }
    return total > 0.0 ? peak / total : 0.0;
}

static void unpack_words(const uint8_t *raw, size_t n_bytes, int32_t *words, int *n_words)
{
    const size_t n = n_bytes / 2;
    for (size_t i = 0; i < n; ++i) {
        int32_t v = (int32_t)((uint32_t)raw[2 * i] | ((uint32_t)raw[2 * i + 1] << 8));
        if (v >= 32768) {
            v -= 65536;
        }
        /* arithmetic >> 2 of the left-justified 14-bit code, i.e. floor(v / 4) */
        words[i] = (v >= 0) ? (v >> 2) : -((-v + 3) >> 2);
    }
    *n_words = (int)n;
}

/* ------------------------------------------------------------------------ */
/* Alignment                                                                  */
/* ------------------------------------------------------------------------ */

typedef struct {
    int n0;
    double peak;
    double sign;
    double margin;
    double probe;   /* score at the probe index (the n0 override), 0 without one */
} align_result_t;

/* align_to_loop().  score[n0] = sum_n r[n] * ref[(n + n0) mod L], evaluated over
 * the nonzero template samples instead of by FFT: identical up to rounding
 * (the template's mean is zero because the polarity is balanced). */
static void align_to_loop(adc_cal_loop_t *loop, const double *residual, int n_res,
                          int probe_index, align_result_t *out)
{
    const int L = loop->n_loop;
    double mu;
    double s = 0.0;
    double s2 = 0.0;
    int best = 0;

    if (n_res > L) {
        n_res = L;
    }
    mu = mean_of(residual, n_res);
    for (int i = 0; i < L; ++i) {
        loop->score[i] = 0.0;
    }
    for (int z = 0; z < loop->ref_nz_count; ++z) {
        const int j = loop->ref_nz_index[z];
        const double r = loop->loop_ref[j];
        for (int n = 0; n < n_res; ++n) {
            int idx = j - n;
            if (idx < 0) {
                idx += L;
            }
            loop->score[idx] += r * (residual[n] - mu);
        }
    }
    for (int i = 1; i < L; ++i) {
        if (fabs(loop->score[i]) > fabs(loop->score[best])) {
            best = i;
        }
    }
    for (int i = 0; i < L; ++i) {
        if (i != best) {
            s += fabs(loop->score[i]);
        }
    }
    {
        const double m_others = s / (double)(L - 1);
        for (int i = 0; i < L; ++i) {
            if (i != best) {
                const double d = fabs(loop->score[i]) - m_others;
                s2 += d * d;
            }
        }
        out->n0 = best;
        out->peak = loop->score[best];
        out->probe = (probe_index >= 0 && probe_index < L) ? loop->score[probe_index] : 0.0;
        out->sign = (out->peak >= 0.0) ? 1.0 : -1.0;
        out->margin = (fabs(out->peak) - m_others) / (sqrt(s2 / (double)(L - 1)) + 1e-30);
    }
}

int adc_cal_loop_prepare_capture(adc_cal_loop_t *loop, const uint8_t *raw, size_t n_bytes,
                                 adc_cal_loop_prep_t *prep)
{
    int n_words = 0;
    double abs_corr[8];
    int n_split[8];
    double best_abs = -1.0;
    int best = -1;
    double best_score = -1.0;
    int n;
    double corr;
    adc_cal_loop_tone_fit_t fit;
    align_result_t align_a;
    align_result_t align_b;
    const double f0 = (double)loop->cfg.sig_cycles / (double)loop->n_loop;

    if (raw == NULL || prep == NULL || n_bytes / 2 > ADC_CAL_LOOP_MAX_WORDS) {
        return -1;
    }
    memset(prep, 0, sizeof(*prep));
    unpack_words(raw, n_bytes, loop->words, &n_words);

    /* deframe(): primary key |corr|, tie-break 1e-3 * purity */
    for (int rot = 0; rot < 8; ++rot) {
        n_split[rot] = split_at(loop->words, n_words, rot, loop->work_y, loop->work_r);
        abs_corr[rot] = n_split[rot] ? fabs(corr_of(loop->work_y, loop->work_r, n_split[rot]))
                                     : -1.0;
        if (n_split[rot] && abs_corr[rot] > best_abs) {
            best_abs = abs_corr[rot];
        }
    }
    if (best_abs < 0.0) {
        return -2;
    }
    for (int rot = 0; rot < 8; ++rot) {
        double score;
        if (!n_split[rot]) {
            continue;
        }
        score = abs_corr[rot];
        /* Purity is at most 2, so a candidate more than 2e-3 below the best
         * |corr| can never win; only evaluate the spectra when it could. */
        if (abs_corr[rot] >= best_abs - 2.5e-3) {
            split_at(loop->words, n_words, rot, loop->work_y, loop->work_r);
            score += 1e-3 * (tone_purity(loop->work_y, n_split[rot], loop->cal_a) +
                             tone_purity(loop->work_r, n_split[rot], loop->cal_a));
        }
        if (best < 0 || score > best_score) {
            best = rot;
            best_score = score;
        }
    }

    n = split_at(loop->words, n_words, best, loop->ch_a, loop->ch_b);
    {
        const double aligned = fabs(corr_of(loop->ch_a, loop->ch_b, n));
        const double shifted = fabs(corr_of(loop->ch_a + 4, loop->ch_b, n - 4));
        prep->lagged = shifted > aligned;
    }
    if (prep->lagged) {
        best = (best + 4) % 8;
        n = split_at(loop->words, n_words, best, loop->ch_a, loop->ch_b);
        if (n == 0) {
            return -2;
        }
    }
    corr = corr_of(loop->ch_a, loop->ch_b, n);
    if (corr < 0.0) {
        for (int i = 0; i < n; ++i) {
            loop->ch_b[i] = -loop->ch_b[i];
        }
    }

    if (adc_cal_loop_fit_tone(loop->ch_a, (size_t)n, f0, 1, &fit) != 0) {
        return -3;
    }
    for (int i = 0; i < n; ++i) {
        loop->work_r[i] = loop->ch_a[i] - (tone_at(&fit, (size_t)i) + fit.dc);
    }
    align_to_loop(loop, loop->work_r, n, loop->n0_override, &align_a);
    if (adc_cal_loop_fit_tone(loop->ch_b, (size_t)n, f0, 1, &fit) != 0) {
        return -3;
    }
    for (int i = 0; i < n; ++i) {
        loop->work_r[i] = loop->ch_b[i] - (tone_at(&fit, (size_t)i) + fit.dc);
    }
    align_to_loop(loop, loop->work_r, n, loop->n0_override, &align_b);

    prep->n_samples = (size_t)n;
    prep->rotation = best;
    prep->corr = corr;
    {
        /* The dither train only identifies the loop position up to the polarity
         * pattern of the ~8 events a capture sees; that pattern recurs in the
         * 64-event sequence, so two lags a whole number of slots apart can score
         * identically.  The host resolves such a tie by FFT rounding, so a
         * replay has to be allowed to adopt the host's choice (n0_override). */
        const align_result_t *pick = (fabs(align_a.peak) >= fabs(align_b.peak)) ? &align_a
                                                                                  : &align_b;
        prep->n0_natural = pick->n0;
        prep->n0 = pick->n0;
        prep->n0_score_ratio = 1.0;
        if (loop->n0_override >= 0 && loop->n0_override < loop->n_loop) {
            prep->n0 = loop->n0_override;
            prep->n0_score_ratio = fabs(pick->probe) / fabs(pick->peak);
        }
    }
    prep->align_margin = (align_a.margin < align_b.margin) ? align_a.margin : align_b.margin;
    prep->sign_a = align_a.sign;
    prep->sign_b = align_b.sign;
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Per-channel estimate                                                       */
/* ------------------------------------------------------------------------ */

static void channel_estimate_reset(adc_cal_loop_channel_estimate_t *est)
{
    est->offset_codes = NAN;
    est->offset_record_codes = NAN;
    est->gain_codes = NAN;
    est->skew_samples = NAN;
    est->phase_rad = NAN;
    est->centroid_samples = NAN;
    est->tone_f0 = NAN;
    est->skew_ps = NAN;
    est->tone_amplitude = NAN;
    est->n_events_used = 0;
    est->align_n0 = -1;
    est->residual_rms = NAN;
    est->gain_sign_fit = NAN;
    est->hat_gain = NAN;
}

/* synthesize_dither() at sample i: the injected dither as the channel saw it. */
static double dither_at(const adc_cal_loop_t *loop, int i, int n0, double gain,
                        double polarity_sign)
{
    const int idx = (i + n0) % loop->n_loop;
    return polarity_sign * gain * (loop->loop_ref[idx] / a_dither(&loop->cfg));
}

static int visible_events(const adc_cal_loop_t *loop, int n0, int n_capture, int m_lo, int m_hi,
                          int *ks, int *starts)
{
    const adc_cal_loop_config_t *cfg = &loop->cfg;
    int count = 0;
    for (int k = 0; k < cfg->n_events; ++k) {
        const double loop_pos = (double)(k * slot_period(cfg)) + pulse_offset(cfg);
        double n_k = fmod(loop_pos - (double)n0, (double)loop->n_loop);
        if (n_k < 0.0) {
            n_k += (double)loop->n_loop;
        }
        if (n_k + (double)m_lo >= 0.0 && n_k + (double)m_hi < (double)n_capture) {
            ks[count] = k;
            starts[count] = (int)nearbyint(n_k);
            ++count;
        }
    }
    return count;
}

static double event_centroid_samples(const double *resid, const int *starts, int n_ev,
                                     int m_lo, int m_hi)
{
    const int span = m_hi - m_lo + 1;
    const int guard = (span / 4 > 2) ? span / 4 : 2;
    double acc = 0.0;
    int good = 0;
    int any_pulse = 0;
    int any_base = 0;

    if (n_ev < 3) {
        return NAN;
    }
    for (int i = 0; i < span; ++i) {
        const int m = m_lo + i;
        if (m < m_lo + guard || m > m_hi - guard) {
            any_base = 1;
        } else {
            any_pulse = 1;
        }
    }
    if (!any_pulse || !any_base) {
        return NAN;
    }
    for (int e = 0; e < n_ev; ++e) {
        double base = 0.0;
        int n_base = 0;
        double tot = 0.0;
        double mom = 0.0;
        for (int i = 0; i < span; ++i) {
            const int m = m_lo + i;
            if (m < m_lo + guard || m > m_hi - guard) {
                base += fabs(resid[starts[e] + m]);
                ++n_base;
            }
        }
        base /= (double)n_base;
        for (int i = 0; i < span; ++i) {
            const int m = m_lo + i;
            if (!(m < m_lo + guard || m > m_hi - guard)) {
                double w = fabs(resid[starts[e] + m]) - base;
                if (w < 0.0) {
                    w = 0.0;
                }
                tot += w;
                mom += w * (double)m;
            }
        }
        if (tot > 0.0) {
            acc += mom / tot;
            ++good;
        }
    }
    return (good < 3) ? NAN : acc / (double)good;
}

/* estimate_channel(), passes = 2, refine_frequency = True, no f0_override
 * (see refit_phase) and no accumulator. */
static int estimate_channel(adc_cal_loop_t *loop, const double *y, int n, int n0,
                            double skew_prior, double polarity_sign, int pin_polarity,
                            int cancel_signal, adc_cal_loop_channel_estimate_t *est)
{
    const adc_cal_loop_config_t *cfg = &loop->cfg;
    const int passes = 2;
    const int m_lo = template_m_lo();
    const int m_hi = template_m_hi(cfg);
    const int W = m_hi - m_lo + 1;
    const double er = edge_r(cfg);
    const double tw = top_w(cfg);
    const double f0 = (double)cfg->sig_cycles / (double)loop->n_loop;
    double d[ADC_CAL_LOOP_MAX_WINDOW];
    double dp[ADC_CAL_LOOP_MAX_WINDOW];
    double v[ADC_CAL_LOOP_MAX_WINDOW];
    double u[ADC_CAL_LOOP_MAX_WINDOW];
    int ks[ADC_CAL_LOOP_MAX_EVENTS];
    int starts[ADC_CAL_LOOP_MAX_EVENTS];
    double dd = 0.0;
    double dpdp = 0.0;
    double hat = NAN;
    double *ytmp = loop->work_y;
    double *resid = loop->work_r;

    channel_estimate_reset(est);
    for (int i = 0; i < W; ++i) {
        const double m = (double)(m_lo + i);
        d[i] = adc_cal_loop_pulse(m + skew_prior, er, tw);
        dp[i] = adc_cal_loop_pulse_derivative(m + skew_prior, er, tw);
        dd += d[i] * d[i];
        dpdp += dp[i] * dp[i];
    }

    for (int pass = 0; pass < passes; ++pass) {
        adc_cal_loop_tone_fit_t fit;
        int n_ev;
        double pbar = 0.0;
        double denom;
        double gain;

        est->hat_gain = hat;
        for (int i = 0; i < n; ++i) {
            ytmp[i] = y[i] - (loop_isfinite(hat) ? dither_at(loop, i, n0, hat, polarity_sign) : 0.0);
        }
        if (adc_cal_loop_fit_tone(ytmp, (size_t)n, f0, 1, &fit) != 0) {
            return -1;
        }
        est->phase_rad = fit.phase;
        est->tone_f0 = fit.f0;
        est->align_n0 = n0;
        est->offset_record_codes = fit.dc;
        est->tone_amplitude = fit.amplitude;
        {
            double s = 0.0;
            double s2 = 0.0;
            for (int i = 0; i < n; ++i) {
                const double r = ytmp[i] - (tone_at(&fit, (size_t)i) + fit.dc);
                s += r;
            }
            s /= (double)n;
            for (int i = 0; i < n; ++i) {
                const double r = ytmp[i] - (tone_at(&fit, (size_t)i) + fit.dc) - s;
                s2 += r * r;
            }
            est->residual_rms = sqrt(s2 / (double)n);
        }

        /* Cancellation removes the fitted tone from the *original* record and
         * keeps DC, exactly like the host. */
        for (int i = 0; i < n; ++i) {
            resid[i] = cancel_signal ? y[i] - tone_at(&fit, (size_t)i) : y[i];
        }

        n_ev = visible_events(loop, n0, n, m_lo, m_hi, ks, starts);
        est->n_events_used = n_ev;
        if (n_ev < 2) {
            return 0;
        }

        for (int i = 0; i < W; ++i) {
            v[i] = 0.0;
            u[i] = 0.0;
        }
        for (int e = 0; e < n_ev; ++e) {
            const double p = (double)cfg->polarity[ks[e]] * polarity_sign;
            pbar += p;
            for (int i = 0; i < W; ++i) {
                const double w = resid[starts[e] + m_lo + i];
                v[i] += p * w;
                u[i] += w;
            }
        }
        pbar /= (double)n_ev;
        for (int i = 0; i < W; ++i) {
            v[i] /= (double)n_ev;
            u[i] /= (double)n_ev;
        }

        denom = 1.0 - pbar * pbar;
        if (denom < 1e-6) {
            return 0;
        }
        for (int i = 0; i < W; ++i) {
            const double vd = (v[i] - pbar * u[i]) / denom;
            const double uo = (u[i] - pbar * v[i]) / denom;
            v[i] = vd;
            u[i] = uo;
        }

        if (dd > 0.0) {
            double s = 0.0;
            for (int i = 0; i < W; ++i) {
                s += v[i] * d[i];
            }
            gain = s / dd;
        } else {
            gain = NAN;
        }
        if (pin_polarity && loop_isfinite(gain)) {
            est->gain_sign_fit = (gain >= 0.0) ? 1.0 : -1.0;
            gain = fabs(gain);
        }
        est->gain_codes = gain;

        if (loop_isfinite(gain) && fabs(gain) > 1e-9 && dpdp > 0.0) {
            double s = 0.0;
            for (int i = 0; i < W; ++i) {
                s += (v[i] - gain * d[i]) * dp[i];
            }
            est->skew_samples = skew_prior + s / (gain * dpdp);
            est->skew_ps = est->skew_samples / adc_cal_loop_fs_adc(cfg) * 1e12;
        }

        est->centroid_samples = event_centroid_samples(resid, starts, n_ev, m_lo, m_hi);

        {
            double s = 0.0;
            int cnt = 0;
            for (int i = 0; i < W; ++i) {
                const double m = (double)(m_lo + i);
                if (m >= er && m < er + tw) {
                    s += u[i];
                    ++cnt;
                }
            }
            est->offset_codes = cnt ? s / (double)cnt : mean_of(u, W);
        }

        if (pass + 1 < passes && loop_isfinite(gain)) {
            hat = gain;
        }
    }
    return 0;
}

/* The host calls estimate_channel a second time with f0_override = f_sh.  That
 * repeats every quantity except the phase, which is refitted (no refine) on the
 * record the last pass used.  Refitting only the phase gives the same numbers
 * at a third of the cost. */
static int refit_phase(adc_cal_loop_t *loop, const double *y, int n, int n0,
                       double polarity_sign, double f_sh, adc_cal_loop_channel_estimate_t *est)
{
    adc_cal_loop_tone_fit_t fit;
    const double hat = est->hat_gain;
    for (int i = 0; i < n; ++i) {
        loop->work_y[i] = y[i] -
            (loop_isfinite(hat) ? dither_at(loop, i, n0, hat, polarity_sign) : 0.0);
    }
    if (adc_cal_loop_fit_tone(loop->work_y, (size_t)n, f_sh, 0, &fit) != 0) {
        return -1;
    }
    est->phase_rad = fit.phase;
    est->tone_f0 = fit.f0;
    return 0;
}

static double wrap_to_half_period(double delta, double f0)
{
    double period;
    if (!loop_isfinite(delta) || f0 <= 0.0) {
        return NAN;
    }
    period = 1.0 / f0;
    return delta - period * nearbyint(delta / period);
}

int adc_cal_loop_estimate_block(adc_cal_loop_t *loop, const double *ch_a, const double *ch_b,
                                size_t n_samples, int n0, double skew_prior_samples,
                                double polarity_sign, int pin_polarity,
                                adc_cal_loop_block_estimate_t *out)
{
    const adc_cal_loop_config_t *cfg = &loop->cfg;
    const int n = (int)n_samples;
    const int cancel = loop->opt.cancel_signal;
    const double ts_ps = 1e12 / adc_cal_loop_fs_adc(cfg);
    const double f_nom = (double)cfg->sig_cycles / (double)loop->n_loop;
    adc_cal_loop_channel_estimate_t a;
    adc_cal_loop_channel_estimate_t b;
    double common;
    double f_sh;
    double dphi;
    double skew_phase;
    double skew_centroid;
    double skew_samples;
    double half_period;
    int branch_by = 0; /* 0 none, 1 prior, 2 centroid */
    adc_cal_loop_skew_source_t source;

    if (n_samples > ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES || n < 16) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->gain_ratio = NAN;
    out->tone_ratio = NAN;
    out->offset_mismatch_codes = NAN;
    out->skew_mismatch_samples = NAN;
    out->skew_mismatch_ps = NAN;
    out->skew_phase_ps = NAN;
    out->skew_diff_route_ps = NAN;
    out->skew_centroid_ps = NAN;
    out->skew_used_ps = NAN;
    out->rotation = -1;

    if (estimate_channel(loop, ch_a, n, n0, 0.0, polarity_sign, pin_polarity, cancel, &a) != 0) {
        return -2;
    }
    common = loop_isfinite(a.skew_samples) ? a.skew_samples : 0.0;
    if (estimate_channel(loop, ch_a, n, a.align_n0, common, polarity_sign, pin_polarity,
                         cancel, &a) != 0 ||
        estimate_channel(loop, ch_b, n, a.align_n0, common + skew_prior_samples, polarity_sign,
                         pin_polarity, cancel, &b) != 0) {
        return -2;
    }

    if (loop_isfinite(a.gain_codes) && fabs(a.gain_codes) > 1e-9) {
        out->gain_ratio = b.gain_codes / a.gain_codes;
    }
    if (loop_isfinite(a.tone_amplitude) && loop_isfinite(b.tone_amplitude) &&
        fabs(a.tone_amplitude) > 1e-9) {
        out->tone_ratio = fabs(b.tone_amplitude) / fabs(a.tone_amplitude);
    }
    out->offset_mismatch_codes = b.offset_codes - a.offset_codes;

    f_sh = 0.5 * (a.tone_f0 + b.tone_f0);
    if (!loop_isfinite(f_sh) || f_sh <= 0.0) {
        f_sh = f_nom;
    }
    if (!(f_sh > 0.0 && f_sh < 0.5)) {
        f_sh = f_nom;
    }
    if (refit_phase(loop, ch_a, n, a.align_n0, polarity_sign, f_sh, &a) != 0 ||
        refit_phase(loop, ch_b, n, a.align_n0, polarity_sign, f_sh, &b) != 0) {
        return -2;
    }

    dphi = b.phase_rad - a.phase_rad;
    if (loop_isfinite(dphi)) {
        dphi = atan2(sin(dphi), cos(dphi));
        skew_phase = wrap_to_half_period(dphi / (2.0 * LOOP_PI * f_sh), f_sh);
    } else {
        skew_phase = NAN;
    }

    /* a - b difference route */
    {
        const double amp_ref = loop_isfinite(a.tone_amplitude) ? a.tone_amplitude : b.tone_amplitude;
        if (n > 8 && loop_isfinite(amp_ref) && amp_ref > 0.0) {
            adc_cal_loop_tone_fit_t fit;
            double mu = 0.0;
            for (int i = 0; i < n; ++i) {
                loop->work_y[i] = ch_a[i] - ch_b[i];
                mu += loop->work_y[i];
            }
            mu /= (double)n;
            for (int i = 0; i < n; ++i) {
                loop->work_y[i] -= mu;
            }
            if (adc_cal_loop_fit_tone(loop->work_y, (size_t)n, f_sh, 0, &fit) == 0) {
                double ratio = fabs(fit.amplitude) / (2.0 * amp_ref);
                if (ratio > 1.0) {
                    ratio = 1.0;
                }
                out->skew_diff_route_ps = -2.0 * asin(ratio) / (2.0 * LOOP_PI * f_sh) * ts_ps;
            }
        }
    }

    skew_centroid = (loop_isfinite(a.centroid_samples) && loop_isfinite(b.centroid_samples))
                        ? b.centroid_samples - a.centroid_samples
                        : NAN;

    half_period = (f_sh > 0.0) ? 0.5 / f_sh : NAN;
    skew_samples = skew_phase;
    if (loop_isfinite(skew_phase) && loop_isfinite(half_period)) {
        const double cands[3] = {skew_phase - half_period, skew_phase, skew_phase + half_period};
        double target = NAN;
        if (fabs(skew_prior_samples) > 1e-12) {
            target = skew_prior_samples;
            branch_by = 1;
        } else if (loop_isfinite(skew_centroid)) {
            target = skew_centroid;
            branch_by = 2;
        }
        if (branch_by) {
            int pick = 0;
            for (int c = 1; c < 3; ++c) {
                if (fabs(cands[c] - target) < fabs(cands[pick] - target)) {
                    pick = c;
                }
            }
            skew_samples = cands[pick];
        }
    }

    source = ADC_CAL_LOOP_SKEW_SOURCE_PHASE;
    if (!(loop_isfinite(skew_samples) && fabs(skew_samples) < 2.0)) {
        skew_samples = loop_isfinite(a.skew_samples) ? a.skew_samples : NAN;
        source = ADC_CAL_LOOP_SKEW_SOURCE_PROFILE_FALLBACK;
    } else if (loop_isfinite(skew_centroid) && fabs(skew_samples - skew_centroid) > 1.0 &&
               branch_by != 1) {
        skew_samples = NAN;
        source = ADC_CAL_LOOP_SKEW_SOURCE_OBSERVABLES_DISAGREE;
    } else if (loop_isfinite(skew_phase) && loop_isfinite(half_period)) {
        if (fabs(skew_samples - skew_phase) > 1e-12) {
            source = (branch_by == 1) ? ADC_CAL_LOOP_SKEW_SOURCE_PHASE_BRANCH_PRIOR
                                      : ADC_CAL_LOOP_SKEW_SOURCE_PHASE_BRANCH_CENTROID;
        }
    }

    out->skew_mismatch_samples = skew_samples;
    out->skew_mismatch_ps = loop_isfinite(skew_samples) ? skew_samples * ts_ps : NAN;
    out->skew_source = source;
    out->skew_phase_ps = loop_isfinite(skew_phase) ? skew_phase * ts_ps : NAN;
    out->skew_centroid_ps = loop_isfinite(skew_centroid) ? skew_centroid * ts_ps : NAN;

    a.skew_samples = loop_isfinite(skew_samples) ? 0.0 : NAN;
    a.skew_ps = loop_isfinite(skew_samples) ? 0.0 : NAN;
    b.skew_samples = skew_samples;
    b.skew_ps = out->skew_mismatch_ps;
    out->ch_a = a;
    out->ch_b = b;
    return 0;
}

/* ------------------------------------------------------------------------ */
/* CalibrationState                                                           */
/* ------------------------------------------------------------------------ */

void adc_cal_loop_default_state(adc_cal_loop_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->gain_corr_a = 1.0;
    state->gain_corr_b = 1.0;
    state->mu_offset = 0.35;
    state->mu_gain = 0.35;
    state->mu_skew = 0.30;
}

void adc_cal_loop_default_options(adc_cal_loop_options_t *options)
{
    memset(options, 0, sizeof(*options));
    options->cancel_signal = 1;
    options->close_skew_loop = 1;
    options->min_align_margin = 6.0;
    options->max_skew_samples = 0.25;
    options->max_gain_deviation = 0.20;
    options->gain_observable = ADC_CAL_LOOP_GAIN_TONE;
    options->skew_batch_frames = 20;
    options->skew_direction_tol_ps = 8.0;
    options->skew_min_yield = 0.4;
    options->max_consecutive_rejects = 20;
}

void adc_cal_loop_default_actuator(adc_cal_loop_actuator_t *actuator)
{
    memset(actuator, 0, sizeof(*actuator));
    actuator->step_ps = 19.1;
    actuator->deadband_ps = 10.0;
    actuator->code = 24;
    actuator->code_min = 0;
    actuator->code_max = 48;
}

/* gain_observable_pair(): the requested amplitudes, falling back when unusable.
 * dither_mag (tone-free) is not ported, so it is always unusable here. */
static const char *gain_observable_pair(const adc_cal_loop_block_estimate_t *est,
                                        adc_cal_loop_gain_observable_t source,
                                        double *ga, double *gb)
{
    const double tone_a = fabs(est->ch_a.tone_amplitude);
    const double tone_b = fabs(est->ch_b.tone_amplitude);
    const double dith_a = est->ch_a.gain_codes;
    const double dith_b = est->ch_b.gain_codes;
    const int tone_ok = loop_isfinite(tone_a) && loop_isfinite(tone_b) && fabs(tone_a) > 1e-9 &&
                        fabs(tone_b) > 1e-9;
    const int dith_ok = loop_isfinite(dith_a) && loop_isfinite(dith_b) && fabs(dith_a) > 1e-9 &&
                        fabs(dith_b) > 1e-9;

    if (source == ADC_CAL_LOOP_GAIN_DITHER) {
        if (dith_ok) {
            *ga = dith_a;
            *gb = dith_b;
            return "dither";
        }
        if (tone_ok) {
            *ga = tone_a;
            *gb = tone_b;
            return "tone";
        }
    } else {
        if (tone_ok) {
            *ga = tone_a;
            *gb = tone_b;
            return "tone";
        }
        if (dith_ok) {
            *ga = dith_a;
            *gb = dith_b;
            return "dither";
        }
    }
    *ga = NAN;
    *gb = NAN;
    return "none";
}

const char *adc_cal_loop_state_update(adc_cal_loop_state_t *state,
                                      const adc_cal_loop_block_estimate_t *est,
                                      int integrate_skew,
                                      adc_cal_loop_gain_observable_t gain_observable,
                                      double skew_ps, adc_cal_loop_row_t *row)
{
    const double e_off_a = est->ch_a.offset_codes;
    const double e_off_b = est->ch_b.offset_codes;
    double ga;
    double gb;
    double e_gain = NAN;
    double e_skew = NAN;
    const double measured = loop_isnan(skew_ps) ? est->skew_mismatch_ps : skew_ps;
    const char *source;

    if (loop_isfinite(e_off_a) && fabs(state->gain_corr_a) > 1e-9) {
        state->offset_a += state->mu_offset * e_off_a / state->gain_corr_a;
    }
    if (loop_isfinite(e_off_b) && fabs(state->gain_corr_b) > 1e-9) {
        state->offset_b += state->mu_offset * e_off_b / state->gain_corr_b;
    }

    source = gain_observable_pair(est, gain_observable, &ga, &gb);
    if (loop_isfinite(ga) && loop_isfinite(gb) && fabs(ga) > 1e-9 && fabs(gb) > 1e-9) {
        const double mean_g = 0.5 * (ga + gb);
        state->gain_corr_a *= 1.0 + state->mu_gain * (mean_g / ga - 1.0);
        state->gain_corr_b *= 1.0 + state->mu_gain * (mean_g / gb - 1.0);
        e_gain = gb / ga - 1.0;
    }

    if (loop_isfinite(measured)) {
        e_skew = measured - state->skew_target_ps;
        if (integrate_skew) {
            state->skew_cmd_ps -= state->mu_skew * e_skew;
        }
    }

    state->iteration += 1U;
    if (row != NULL) {
        row->offset_error_a = e_off_a;
        row->offset_error_b = e_off_b;
        row->gain_error = e_gain;
        row->gain_source = source;
        row->skew_error_ps = e_skew;
    }
    return source;
}

static void state_apply(const adc_cal_loop_state_t *state, const double *a, const double *b,
                        double *cal_a, double *cal_b, int n)
{
    for (int i = 0; i < n; ++i) {
        cal_a[i] = (a[i] - state->offset_a) * state->gain_corr_a;
        cal_b[i] = (b[i] - state->offset_b) * state->gain_corr_b;
    }
}

/* ------------------------------------------------------------------------ */
/* Skew batch decision                                                        */
/* ------------------------------------------------------------------------ */

int adc_cal_loop_error_to_steps(const adc_cal_loop_actuator_t *actuator, double error_ps)
{
    int steps;
    if (!loop_isfinite(error_ps) || fabs(error_ps) < actuator->deadband_ps) {
        return 0;
    }
    steps = (int)nearbyint(-error_ps / actuator->step_ps);
    if (steps > 1) {
        steps = 1;
    } else if (steps < -1) {
        steps = -1;
    }
    if (steps > 0 && actuator->code >= actuator->code_max) {
        return 0;
    }
    if (steps < 0 && actuator->code <= actuator->code_min) {
        return 0;
    }
    return steps;
}

void adc_cal_loop_skew_batch(const adc_cal_loop_skew_frame_t *frames, int n_frames,
                             adc_cal_loop_skew_batch_t *out)
{
    const double tone_tol = 0.10;
    const double resid_lo = 0.5;
    const double resid_hi = 2.0;
    const double margin_min = 6.0;
    const double agree_k = 6.0;
    const double agree_floor_ps = 1.5;
    double tones[ADC_CAL_LOOP_MAX_BATCH];
    double resids[ADC_CAL_LOOP_MAX_BATCH];
    double deltas[ADC_CAL_LOOP_MAX_BATCH];
    double kept[ADC_CAL_LOOP_MAX_BATCH];
    int n_delta = 0;
    int n_kept = 0;
    double tone_med;
    double resid_med;
    double center;
    double bound;

    out->n_total = n_frames;
    out->n_used = 0;
    out->mean_ps = NAN;
    out->se_ps = NAN;
    out->route_center_ps = NAN;
    out->route_bound_ps = NAN;
    if (n_frames <= 0) {
        return;
    }
    if (n_frames > ADC_CAL_LOOP_MAX_BATCH) {
        n_frames = ADC_CAL_LOOP_MAX_BATCH;
    }

    for (int i = 0; i < n_frames; ++i) {
        const double dl = frames[i].phase_ps - frames[i].diff_route_ps;
        tones[i] = frames[i].tone;
        resids[i] = frames[i].resid;
        if (loop_isfinite(dl)) {
            deltas[n_delta++] = dl;
        }
    }
    tone_med = nan_median(tones, n_frames);
    resid_med = nan_median(resids, n_frames);

    if (n_delta >= 5) {
        double dev[ADC_CAL_LOOP_MAX_BATCH];
        center = nan_median(deltas, n_delta);
        for (int i = 0; i < n_delta; ++i) {
            dev[i] = fabs(deltas[i] - center);
        }
        bound = agree_k * nan_median(dev, n_delta) * 1.4826;
        if (bound < agree_floor_ps) {
            bound = agree_floor_ps;
        }
    } else {
        center = 0.0;
        bound = agree_floor_ps;
    }
    out->route_center_ps = center;
    out->route_bound_ps = bound;

    for (int i = 0; i < n_frames; ++i) {
        const adc_cal_loop_skew_frame_t *f = &frames[i];
        if (f->source == ADC_CAL_LOOP_SKEW_SOURCE_PROFILE_FALLBACK) {
            continue;
        }
        if (!loop_isfinite(f->phase_ps)) {
            continue;
        }
        if (loop_isfinite(f->diff_route_ps) &&
            fabs((f->phase_ps - f->diff_route_ps) - center) > bound) {
            continue;
        }
        if (loop_isfinite(tone_med) && loop_isfinite(f->tone) &&
            fabs(f->tone - tone_med) > tone_tol * fabs(tone_med)) {
            continue;
        }
        if (loop_isfinite(resid_med) && loop_isfinite(f->resid) &&
            !(resid_lo * resid_med < f->resid && f->resid < resid_hi * resid_med)) {
            continue;
        }
        if (!(loop_isfinite(f->margin) && f->margin >= margin_min)) {
            continue;
        }
        kept[n_kept++] = f->phase_ps;
    }

    out->n_used = n_kept;
    if (n_kept > 0) {
        const double mu = mean_of(kept, n_kept);
        out->mean_ps = mu;
        if (n_kept > 1) {
            double s = 0.0;
            for (int i = 0; i < n_kept; ++i) {
                s += (kept[i] - mu) * (kept[i] - mu);
            }
            out->se_ps = sqrt(s / (double)(n_kept - 1)) / sqrt((double)n_kept);
        }
    }
}

/* CalibrationLoop._skew_decision(), tone route. */
static void skew_decision(adc_cal_loop_t *loop, const adc_cal_loop_block_estimate_t *est,
                          adc_cal_loop_row_t *row)
{
    adc_cal_loop_skew_batch_t batch;
    adc_cal_loop_skew_frame_t *f;
    double error;
    int steps;
    adc_cal_loop_skew_ack_t ack;
    int batch_frames = loop->opt.skew_batch_frames;

    if (batch_frames < 1) {
        batch_frames = 1;
    }
    if (batch_frames > ADC_CAL_LOOP_MAX_BATCH) {
        batch_frames = ADC_CAL_LOOP_MAX_BATCH;
    }
    f = &loop->skew_frames[loop->skew_frame_count++];
    f->phase_ps = est->skew_phase_ps;
    f->diff_route_ps = est->skew_diff_route_ps;
    f->source = est->skew_source;
    f->tone = est->ch_a.tone_amplitude;
    f->resid = est->ch_a.residual_rms;
    f->margin = row->prep.align_margin;
    row->skew_batch_n = loop->skew_frame_count;
    if (loop->skew_frame_count < batch_frames) {
        return;
    }

    adc_cal_loop_skew_batch(loop->skew_frames, loop->skew_frame_count, &batch);
    loop->skew_frame_count = 0;
    row->skew_batch_used = batch.n_used;
    row->skew_batch_mean_ps = batch.mean_ps;
    row->skew_batch_se_ps = batch.se_ps;
    error = batch.mean_ps - loop->state.skew_target_ps;
    row->skew_batch_error_ps = error;

    if (!loop->opt.close_skew_loop || loop->act.request_steps == NULL) {
        row->skew_action = ADC_CAL_LOOP_SKEW_ACTION_MEASURE_ONLY;
        return;
    }
    if (loop->skew_abort) {
        row->skew_action = ADC_CAL_LOOP_SKEW_ACTION_ABORTED;
        return;
    }
    if (batch.n_used == 0) {
        row->skew_action = ADC_CAL_LOOP_SKEW_ACTION_NO_USABLE_FRAMES;
        return;
    }
    if (batch.n_total &&
        (double)batch.n_used / (double)batch.n_total < loop->opt.skew_min_yield) {
        row->skew_action = ADC_CAL_LOOP_SKEW_ACTION_LOW_YIELD;
        return;
    }

    if (loop->skew_have_prev && loop->skew_expected_shift != 0.0) {
        const double moved = error - loop->skew_prev_error;
        if (fabs(moved - loop->skew_expected_shift) > loop->opt.skew_direction_tol_ps &&
            sign_of(moved) != sign_of(loop->skew_expected_shift)) {
            loop->skew_abort = 1;
            row->skew_action = ADC_CAL_LOOP_SKEW_ACTION_ABORT_DIRECTION;
            return;
        }
    }

    steps = adc_cal_loop_error_to_steps(&loop->act, error);
    if (steps == 0) {
        row->skew_action = (fabs(error) <= loop->act.deadband_ps)
                               ? ADC_CAL_LOOP_SKEW_ACTION_INSIDE_DEADBAND
                               : ADC_CAL_LOOP_SKEW_ACTION_AT_RANGE_LIMIT;
        return;
    }
    memset(&ack, 0, sizeof(ack));
    ack.code_after = loop->act.code;
    if (loop->act.request_steps(loop->act.ctx, steps, &ack) != 0) {
        ack.ok = 0;
    }
    row->skew_step_codes = steps;
    row->skew_code = ack.code_after;
    if (!ack.ok) {
        row->skew_action = ADC_CAL_LOOP_SKEW_ACTION_MOVE_FAILED;
        loop->skew_abort = 1;
        return;
    }
    row->skew_action = ADC_CAL_LOOP_SKEW_ACTION_MOVE_OK;
    loop->act.code = ack.code_after;
    loop->skew_have_prev = 1;
    loop->skew_prev_error = error;
    loop->skew_expected_shift = (double)steps * loop->act.step_ps;
}

/* ------------------------------------------------------------------------ */
/* CalibrationLoop                                                            */
/* ------------------------------------------------------------------------ */

int adc_cal_loop_init(adc_cal_loop_t *loop, const adc_cal_loop_config_t *cfg,
                      const adc_cal_loop_options_t *opt, const adc_cal_loop_state_t *state,
                      const adc_cal_loop_actuator_t *act)
{
    int span;
    double ad;

    if (loop == NULL || cfg == NULL) {
        return -1;
    }
    if (adc_cal_loop_config_validate(cfg, NULL) != 0) {
        return -2;
    }
    memset(loop, 0, sizeof(*loop));
    loop->cfg = *cfg;
    loop->n0_override = -1;
    if (opt != NULL) {
        loop->opt = *opt;
    } else {
        adc_cal_loop_default_options(&loop->opt);
    }
    if (state != NULL) {
        loop->state = *state;
    } else {
        adc_cal_loop_default_state(&loop->state);
    }
    if (act != NULL) {
        loop->act = *act;
    } else {
        adc_cal_loop_default_actuator(&loop->act);
    }

    /* dither_only_loop(cfg) */
    loop->n_loop = adc_cal_loop_n_adc_period(cfg);
    span = (int)ceil(pulse_len(cfg)) + 2;
    ad = a_dither(cfg);
    for (int k = 0; k < cfg->n_events; ++k) {
        const double start = (double)(k * slot_period(cfg)) + pulse_offset(cfg);
        const int lo = (int)floor(start);
        for (int i = 0; i < span; ++i) {
            const int idx = (lo + i) % loop->n_loop;
            loop->loop_ref[idx] += (double)cfg->polarity[k] * ad *
                                   adc_cal_loop_pulse((double)idx - start, edge_r(cfg), top_w(cfg));
        }
    }
    loop->ref_nz_count = 0;
    for (int i = 0; i < loop->n_loop; ++i) {
        if (loop->loop_ref[i] != 0.0) {
            loop->ref_nz_index[loop->ref_nz_count++] = i;
        }
    }
    return 0;
}

static adc_cal_loop_reject_t reject_reason(const adc_cal_loop_t *loop,
                                           const adc_cal_loop_prep_t *prep,
                                           const adc_cal_loop_block_estimate_t *est)
{
    double residual;
    double ga;
    double gb;

    if (prep->align_margin < loop->opt.min_align_margin) {
        return ADC_CAL_LOOP_REJECT_ALIGN_MARGIN;
    }
    if (est->ch_a.n_events_used < 2 || est->ch_b.n_events_used < 2) {
        return ADC_CAL_LOOP_REJECT_TOO_FEW_EVENTS;
    }
    if (!loop_isfinite(est->ch_a.gain_codes) || !loop_isfinite(est->ch_a.skew_samples)) {
        return ADC_CAL_LOOP_REJECT_A_NOT_FINITE;
    }
    if (!loop_isfinite(est->ch_b.gain_codes) || !loop_isfinite(est->ch_b.skew_samples)) {
        return ADC_CAL_LOOP_REJECT_B_NOT_FINITE;
    }
    residual = (est->skew_used_ps - loop->state.skew_target_ps) * 1e-12 *
               adc_cal_loop_fs_adc(&loop->cfg);
    if (fabs(residual) > loop->opt.max_skew_samples) {
        return ADC_CAL_LOOP_REJECT_SKEW_RANGE;
    }
    (void)gain_observable_pair(est, loop->opt.gain_observable, &ga, &gb);
    if (loop_isfinite(ga) && loop_isfinite(gb) && fabs(ga) > 1e-9) {
        if (fabs(gb / ga - 1.0) > loop->opt.max_gain_deviation) {
            return ADC_CAL_LOOP_REJECT_GAIN_RANGE;
        }
    }
    return ADC_CAL_LOOP_REJECT_NONE;
}

static void row_state(const adc_cal_loop_t *loop, adc_cal_loop_row_t *row)
{
    row->offset_a_state = loop->state.offset_a;
    row->offset_b_state = loop->state.offset_b;
    row->gain_corr_a = loop->state.gain_corr_a;
    row->gain_corr_b = loop->state.gain_corr_b;
    row->skew_cmd_ps = loop->state.skew_cmd_ps;
}

int adc_cal_loop_step(adc_cal_loop_t *loop, const uint8_t *raw, size_t n_bytes,
                      adc_cal_loop_row_t *row)
{
    int n;
    int rc;

    memset(row, 0, sizeof(*row));
    row->iteration = loop->state.iteration;
    row->gain_source = "";
    row->offset_error_a = NAN;
    row->offset_error_b = NAN;
    row->gain_error = NAN;
    row->skew_error_ps = NAN;
    row->skew_batch_mean_ps = NAN;
    row->skew_batch_se_ps = NAN;
    row->skew_batch_error_ps = NAN;
    row->skew_code = loop->act.code;

    rc = adc_cal_loop_prepare_capture(loop, raw, n_bytes, &row->prep);
    if (rc != 0) {
        row->rejected = ADC_CAL_LOOP_REJECT_CAPTURE;
        return rc;
    }
    if (loop->polarity == 0.0) {
        /* polarity_anchor(): a session constant, from channel A's alignment */
        loop->polarity = (row->prep.sign_a < 0.0) ? -1.0 : 1.0;
    }
    row->polarity = loop->polarity;
    n = (int)row->prep.n_samples;

    state_apply(&loop->state, loop->ch_a, loop->ch_b, loop->cal_a, loop->cal_b, n);
    rc = adc_cal_loop_estimate_block(
        loop, loop->cal_a, loop->cal_b, (size_t)n, row->prep.n0,
        loop->state.skew_target_ps * 1e-12 * adc_cal_loop_fs_adc(&loop->cfg),
        loop->polarity, 1, &row->est);
    if (rc != 0) {
        row->rejected = ADC_CAL_LOOP_REJECT_CAPTURE;
        return rc;
    }
    row->est.rotation = row->prep.rotation;
    row->est.skew_used_ps = row->est.skew_mismatch_ps;

    row->rejected = reject_reason(loop, &row->prep, &row->est);
    if (row->rejected != ADC_CAL_LOOP_REJECT_NONE) {
        row_state(loop, row);
        return 0;
    }

    if (loop->act.request_steps != NULL) {
        /* hardware: gain/offset per frame, skew as a batch decision */
        (void)adc_cal_loop_state_update(&loop->state, &row->est, 0, loop->opt.gain_observable,
                                        NAN, row);
        skew_decision(loop, &row->est, row);
    } else {
        /* model: per-frame skew integrator */
        (void)adc_cal_loop_state_update(&loop->state, &row->est, loop->opt.close_skew_loop,
                                        loop->opt.gain_observable, row->est.skew_used_ps, row);
        if (loop->opt.close_skew_loop && loop->act.command_skew_ps != NULL) {
            loop->act.command_skew_ps(loop->act.ctx, loop->state.skew_cmd_ps);
        }
    }
    row_state(loop, row);
    return 0;
}

unsigned adc_cal_loop_run(adc_cal_loop_t *loop, unsigned n_qualified,
                          adc_cal_loop_capture_fn capture, void *capture_ctx,
                          adc_cal_loop_row_fn on_row, void *row_ctx,
                          uint8_t *capture_buf, size_t capture_capacity)
{
    const unsigned max_capture_retries = 3U;
    unsigned qualified = 0U;
    unsigned failures = 0U;
    unsigned streak = 0U;
    adc_cal_loop_row_t row;

    loop->captures = 0U;
    loop->rejected = 0U;
    while (qualified < n_qualified) {
        const long got = capture(capture_ctx, capture_buf, capture_capacity);
        int rc;
        loop->captures++;
        if (got <= 0) {
            if (++failures >= max_capture_retries) {
                break;
            }
            continue;
        }
        rc = adc_cal_loop_step(loop, capture_buf, (size_t)got, &row);
        if (on_row != NULL) {
            on_row(row_ctx, &row);
        }
        if (rc != 0) {
            if (++failures >= max_capture_retries) {
                break;
            }
            continue;
        }
        failures = 0U;
        if (row.rejected != ADC_CAL_LOOP_REJECT_NONE) {
            loop->rejected++;
            if (++streak >= loop->opt.max_consecutive_rejects) {
                break;
            }
            continue;
        }
        streak = 0U;
        qualified++;
    }
    loop->qualified = qualified;
    return qualified;
}

/* ------------------------------------------------------------------------ */
/* Names                                                                      */
/* ------------------------------------------------------------------------ */

const char *adc_cal_loop_reject_name(adc_cal_loop_reject_t reason)
{
    switch (reason) {
    case ADC_CAL_LOOP_REJECT_NONE: return "";
    case ADC_CAL_LOOP_REJECT_ALIGN_MARGIN: return "align_margin";
    case ADC_CAL_LOOP_REJECT_TOO_FEW_EVENTS: return "too few dither events";
    case ADC_CAL_LOOP_REJECT_A_NOT_FINITE: return "channel A estimate not finite";
    case ADC_CAL_LOOP_REJECT_B_NOT_FINITE: return "channel B estimate not finite";
    case ADC_CAL_LOOP_REJECT_SKEW_RANGE: return "skew mismatch residual out of range";
    case ADC_CAL_LOOP_REJECT_GAIN_RANGE: return "gain ratio out of range";
    case ADC_CAL_LOOP_REJECT_CAPTURE: return "capture could not be de-framed";
    }
    return "?";
}

const char *adc_cal_loop_skew_action_name(adc_cal_loop_skew_action_t action)
{
    switch (action) {
    case ADC_CAL_LOOP_SKEW_ACTION_NONE: return "";
    case ADC_CAL_LOOP_SKEW_ACTION_MEASURE_ONLY: return "measure-only";
    case ADC_CAL_LOOP_SKEW_ACTION_ABORTED: return "aborted";
    case ADC_CAL_LOOP_SKEW_ACTION_NO_USABLE_FRAMES: return "no-usable-frames";
    case ADC_CAL_LOOP_SKEW_ACTION_LOW_YIELD: return "low-yield";
    case ADC_CAL_LOOP_SKEW_ACTION_ABORT_DIRECTION: return "abort:direction";
    case ADC_CAL_LOOP_SKEW_ACTION_INSIDE_DEADBAND: return "inside-deadband";
    case ADC_CAL_LOOP_SKEW_ACTION_AT_RANGE_LIMIT: return "at-range-limit";
    case ADC_CAL_LOOP_SKEW_ACTION_MOVE_OK: return "move:OK";
    case ADC_CAL_LOOP_SKEW_ACTION_MOVE_FAILED: return "move:FAILED";
    }
    return "?";
}

const char *adc_cal_loop_skew_source_name(adc_cal_loop_skew_source_t source)
{
    switch (source) {
    case ADC_CAL_LOOP_SKEW_SOURCE_NONE: return "none";
    case ADC_CAL_LOOP_SKEW_SOURCE_PHASE: return "phase";
    case ADC_CAL_LOOP_SKEW_SOURCE_PHASE_BRANCH_PRIOR: return "phase+branch(prior)";
    case ADC_CAL_LOOP_SKEW_SOURCE_PHASE_BRANCH_CENTROID: return "phase+branch(centroid)";
    case ADC_CAL_LOOP_SKEW_SOURCE_PROFILE_FALLBACK: return "profile-fallback";
    case ADC_CAL_LOOP_SKEW_SOURCE_OBSERVABLES_DISAGREE: return "observables-disagree";
    }
    return "?";
}
