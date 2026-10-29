/* Private implementation: two observations of the SAME broadband dither.
 * No waveform, pulse parameters, polarity sequence, or synthesized reference.
 * The arbitrary common offset/gain/delay cannot be recovered: match A and B.
 * Requires equal analog transfer shapes apart from gain and relative delay.
 */
static void blind_fft(double *re, double *im, int n)
{
    for (int i = 1, j = 0; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            double t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        const double angle = -6.2831853071795864769 / len;
        const double cr = cos(angle), ci = sin(angle);
        for (int base = 0; base < n; base += len) {
            double wr = 1.0, wi = 0.0;
            for (int j = 0; j < len / 2; ++j) {
                const int a = base + j, b = a + len / 2;
                const double vr = wr * re[b] - wi * im[b];
                const double vi = wr * im[b] + wi * re[b];
                const double next_wr = wr * cr - wi * ci;
                re[b] = re[a] - vr; im[b] = im[a] - vi;
                re[a] += vr; im[a] += vi;
                wi = wr * ci + wi * cr; wr = next_wr;
            }
        }
    }
}

static int blind_estimate(adc_cal_loop_t *loop, const double *a, const double *b,
                          size_t count, adc_cal_loop_block_estimate_t *out)
{
    int n = 1, bins = 0;
    double ma = 0.0, mb = 0.0, peak = 0.0, sw = 0.0, sw2 = 0.0;
    double xx = 0.0, xy = 0.0, pa = 0.0, pb = 0.0, error = 0.0;
    double *ar = loop->score, *ai, *br, *bi;
    memset(out, 0, sizeof(*out));
    channel_estimate_reset(&out->ch_a); channel_estimate_reset(&out->ch_b);
    out->tone_ratio = out->skew_phase_ps = out->skew_fold_ps = NAN;
    out->gain_mag_ratio = out->skew_slope_ps = out->skew_used_ps = NAN;
    out->gain_ratio = out->offset_mismatch_codes = NAN;
    out->mag_a = out->mag_b = NAN;
    if (count < 256 || count > ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES) return -1;
    while ((size_t)(2 * n) <= count) n *= 2;
    ai = ar + n; br = ai + n; bi = br + n;
    for (int i = 0; i < n; ++i) { ma += a[i]; mb += b[i]; }
    ma /= n; mb /= n;
    for (int i = 0; i < n; ++i) {
        const double w = 0.5 - 0.5 * cos(6.2831853071795864769 * i / (n - 1));
        ar[i] = (a[i] - ma) * w; br[i] = (b[i] - mb) * w;
        ai[i] = bi[i] = 0.0;
    }
    blind_fft(ar, ai, n); blind_fft(br, bi, n);
    for (int k = 2; k < n * 2 / 5; ++k) {
        const double p = sqrt((ar[k]*ar[k] + ai[k]*ai[k]) *
                              (br[k]*br[k] + bi[k]*bi[k]));
        if (p > peak) peak = p;
    }
    if (!(peak > 0.0)) return 0; /* invalid estimate, rejected by normal gates */
    for (int k = 2; k < n * 2 / 5; ++k) {
        const double aa = ar[k]*ar[k] + ai[k]*ai[k];
        const double bb = br[k]*br[k] + bi[k]*bi[k];
        const double w = sqrt(aa * bb);
        const double omega = 6.2831853071795864769 * k / n;
        /* B samples later -> positive phase of B * conjugate(A). */
        const double phase = atan2(bi[k]*ar[k] - br[k]*ai[k],
                                   br[k]*ar[k] + bi[k]*ai[k]);
        if (w < peak * 0.01) continue;
        bins++; sw += w; sw2 += w*w; xx += w*omega*omega;
        xy += w*omega*phase; pa += aa; pb += bb;
    }
    if (bins < 8 || !(xx > 0.0) || sw*sw < 4.0*sw2) return 0;
    const double delay = xy / xx;
    const double ratio = sqrt(pb / pa);
    for (int k = 2; k < n * 2 / 5; ++k) {
        const double w = sqrt((ar[k]*ar[k] + ai[k]*ai[k]) *
                              (br[k]*br[k] + bi[k]*bi[k]));
        if (w < peak * 0.01) continue;
        const double phase = atan2(bi[k]*ar[k] - br[k]*ai[k],
                                   br[k]*ar[k] + bi[k]*ai[k]);
        const double e = phase - 6.2831853071795864769*k/n*delay;
        error += w*e*e;
    }
    /* Fail closed on incompatible spectra, poor excitation, or excess noise.
     * No claim that a narrowband tone or a constant can identify this mode. */
    if (error / sw > 0.0225 || pa / (n*(double)n) < 4.0 ||
        pb / (n*(double)n) < 4.0) return 0;
    const double dc = mb - ratio * ma;
    out->ch_a.offset_codes = -dc / (2.0 * ratio);
    out->ch_b.offset_codes = dc / 2.0;
    out->ch_a.offset_record_codes = ma; out->ch_b.offset_record_codes = mb;
    out->ch_a.n_events_used = out->ch_b.n_events_used = bins;
    out->mag_a = sqrt(pa); out->mag_b = sqrt(pb);
    out->ch_a.gain_codes = out->mag_a; out->ch_b.gain_codes = out->mag_b;
    out->gain_ratio = out->gain_mag_ratio = ratio;
    out->offset_mismatch_codes = out->ch_b.offset_codes - out->ch_a.offset_codes;
    out->skew_mismatch_samples = delay;
    out->skew_source = ADC_CAL_LOOP_SKEW_SOURCE_DITHER_CROSS_SPECTRUM;
    out->skew_mismatch_ps = out->skew_slope_ps = out->skew_used_ps =
        delay * 1e12 / adc_cal_loop_fs_adc(&loop->cfg);
    out->dither_margin = 1.0 / sqrt(fmax(error / sw, 1e-12));
    out->ch_a.residual_rms = out->ch_b.residual_rms = sqrt(error / sw);
    return 0;
}
