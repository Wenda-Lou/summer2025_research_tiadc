/* Private output stage. The calibration estimator never calls this code.
 * Single-tone + sparse, equally spaced signed impulses is the supported signal
 * class. Frequency, spacing, pulse shape and every sign are learned from ADC
 * samples. We subtract cross-validated pulse replicas, NOT a regenerated tone.
 */
static double output_median(double *x, int n)
{
    /* Quickselect: bounded scratch, no quadratic insertion sort on ADC frames. */
    int lo = 0, hi = n - 1;
    const int mid = n / 2;
    while (lo < hi) {
        const double pivot = x[(lo + hi) / 2];
        int i = lo, j = hi;
        while (i <= j) {
            while (x[i] < pivot) i++;
            while (x[j] > pivot) j--;
            if (i <= j) {
                const double v = x[i]; x[i++] = x[j]; x[j--] = v;
            }
        }
        if (mid <= j) hi = j;
        else if (mid >= i) lo = i;
        else break;
    }
    return x[mid];
}

static int output_solve(const double *y, int n, double freq, const double *mask,
                         double coef[3], double *sse)
{
    double m[3][3] = {{0}}, rhs[3] = {0};
    double error = 0.0;
    for (int i = 0; i < n; ++i) {
        if (mask && mask[i] == 0.0) continue;
        const double c = cos(2.0 * LOOP_PI * freq * i);
        const double s = sin(2.0 * LOOP_PI * freq * i);
        m[0][0] += c*c; m[0][1] += c*s; m[0][2] += c;
        m[1][1] += s*s; m[1][2] += s; m[2][2] += 1.0;
        rhs[0] += y[i]*c; rhs[1] += y[i]*s; rhs[2] += y[i];
    }
    m[1][0] = m[0][1]; m[2][0] = m[0][2]; m[2][1] = m[1][2];
    if (m[2][2] < 0.65*n || solve3(m, rhs, coef) != 0) return -1;
    for (int i = 0; i < n; ++i) {
        if (mask && mask[i] == 0.0) continue;
        const double r = y[i] - coef[0]*cos(2.0*LOOP_PI*freq*i) -
                         coef[1]*sin(2.0*LOOP_PI*freq*i) - coef[2];
        error += r*r;
    }
    *sse = error;
    return 0;
}

static double output_model(const double coef[3], double freq, int i)
{
    return coef[0]*cos(2.0*LOOP_PI*freq*i) + coef[1]*sin(2.0*LOOP_PI*freq*i) + coef[2];
}

static int output_fit(const double *y, int n, double *freq, int refine,
                       double *mask, double *scratch, double *resid, double *sigma)
{
    double coef[3], error;
    if (output_solve(y, n, *freq, NULL, coef, &error) != 0) return -1;
    for (int pass = 0; pass < 5; ++pass) {
        for (int i = 0; i < n; ++i)
            scratch[i] = fabs(y[i] - output_model(coef, *freq, i));
        *sigma = fmax(0.05, 1.4826*output_median(scratch, n));
        for (int i = 0; i < n; ++i)
            mask[i] = fabs(y[i] - output_model(coef, *freq, i)) <=
                      fmax(1.0, 4.5 * *sigma) ? 1.0 : 0.0;
        if (output_solve(y, n, *freq, mask, coef, &error) != 0) return -1;
        if (refine) {
            double step = 0.5 / n;
            for (int it = 0; it < 32; ++it) {
                int improved = 0;
                for (int side = -1; side <= 1; side += 2) {
                    const double f = *freq + side*step;
                    double c[3], e;
                    if (f <= 1.0/n || f >= 0.5 - 1.0/n) continue;
                    if (output_solve(y, n, f, mask, c, &e) == 0 && e < error) {
                        *freq = f; error = e;
                        memcpy(coef, c, sizeof(coef)); improved = 1; break;
                    }
                }
                if (!improved) step *= 0.5;
                if (step < 1e-11) break;
            }
        }
    }
    for (int i = 0; i < n; ++i) {
        resid[i] = y[i] - output_model(coef, *freq, i);
        scratch[i] = fabs(resid[i]);
    }
    *sigma = fmax(0.05, 1.4826*output_median(scratch, n));
    return hypot(coef[0], coef[1]) >= 8.0 * *sigma ? 0 : -1;
}

int adc_cal_loop_cancel_dither(adc_cal_loop_t *loop, const double *a, const double *b,
                               size_t count)
{
    int nfft = 1, peak_bin = 0, peaks[ADC_CAL_LOOP_MAX_EVENTS], np = 0;
    int period = 0, phase = 0, events = 0, group_n[2] = {0, 0};
    double total = 0.0, peak = 0.0, freq, sigma[2], before[2] = {0}, after[2] = {0};
    double *ar, *ai, *mask, *scratch, *res[2], *profile[2][2];
    if (!loop) return -1;
    memset(&loop->output, 0, sizeof(loop->output));
    if (!a || !b || count < 512 || count > ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES) return -1;
    const int n = (int)count;
    for (int i = 0; i < n; ++i)
        if (!loop_isfinite(a[i]) || !loop_isfinite(b[i])) return -1;
    memmove(loop->output_a, a, count*sizeof(double));
    memmove(loop->output_b, b, count*sizeof(double));
    loop->output.n_samples = count;
    loop->output.status = ADC_CAL_LOOP_OUTPUT_NO_TONE;
    while (2*nfft <= n) nfft *= 2;
    ar = loop->score; ai = ar + nfft;
    mask = loop->score + 2*ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES;
    scratch = mask + ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES;
    const double mean = mean_of(a, nfft);
    for (int i = 0; i < nfft; ++i) {
        ar[i] = (a[i] - mean)*(0.5 - 0.5*cos(2.0*LOOP_PI*i/(nfft-1)));
        ai[i] = 0.0;
    }
    blind_fft(ar, ai, nfft);
    for (int k = 2; k < nfft/2 - 1; ++k) {
        const double p = ar[k]*ar[k] + ai[k]*ai[k];
        total += p;
        if (p > peak) { peak = p; peak_bin = k; }
    }
    /* A comb line alone is not evidence of the user's main tone. */
    if (!(peak > 0.15*total) || peak_bin == 0) return 0;
    freq = (double)peak_bin / nfft;
    res[0] = loop->work_y; res[1] = loop->work_r;
    if (output_fit(a, n, &freq, 1, mask, scratch, res[0], &sigma[0]) != 0 ||
        output_fit(b, n, &freq, 0, mask, scratch, res[1], &sigma[1]) != 0) return 0;
    loop->output.tone_hz = freq * adc_cal_loop_fs_adc(&loop->cfg);
    loop->output.status = ADC_CAL_LOOP_OUTPUT_NO_DITHER;
    peak = 0.0;
    for (int i = 0; i < n; ++i) peak = fmax(peak, fabs(res[0][i]));
    const double threshold = fmax(8.0*sigma[0], 0.10*peak);
    /* Discover pulse events from contiguous outliers after robust tone fitting. */
    for (int i = 0; i < n; ++i) {
        if (fabs(res[0][i]) <= threshold) continue;
        int at = i;
        while (i + 1 < n && fabs(res[0][i+1]) > threshold) {
            i++;
            if (fabs(res[0][i]) > fabs(res[0][at])) at = i;
        }
        if (np >= ADC_CAL_LOOP_MAX_EVENTS) return 0;
        peaks[np++] = at;
    }
    if (np == 0) return 0;
    loop->output.status = ADC_CAL_LOOP_OUTPUT_AMBIGUOUS;
    if (np < 6) return 0;
    for (int i = 1; i < np; ++i) scratch[i-1] = peaks[i] - peaks[i-1];
    const int spacing = (int)lround(output_median(scratch, np-1));
    double best = DBL_MAX;
    for (int p = spacing-2; p <= spacing+2; ++p) {
        if (p < 8 || p > n/6) continue;
        for (int shift = -2; shift <= 2; ++shift) {
            const int ph = ((peaks[0] + shift) % p + p) % p;
            double error = 0.0;
            for (int j = 0; j < np; ++j) {
                int d = (peaks[j] - ph + p) % p;
                if (d > p/2) d -= p;
                error += d*d;
            }
            if (error < best) { best = error; period = p; phase = ph; }
        }
    }
    if (!period || best > np || abs((peaks[np-1]-peaks[0]) - (np-1)*period) > 2)
        return 0;
    /* Four learned profiles: A/B, even/odd events. Every output event uses the
     * opposite group, so its own ADC noise is never fitted out of that event. */
    for (int ch = 0; ch < 2; ++ch)
        for (int g = 0; g < 2; ++g) {
            profile[ch][g] = loop->score + (ch*2+g)*period;
            memset(profile[ch][g], 0, (size_t)period*sizeof(double));
        }
    int start = phase - period/2;
    while (start < 0) start += period;
    for (int base = start; base + period <= n; base += period) {
        const int g = events % 2;
        for (int ch = 0; ch < 2; ++ch) {
            const double sign = sign_of(res[ch][base + period/2]);
            for (int j = 0; j < period; ++j)
                profile[ch][g][j] += sign*res[ch][base+j];
        }
        group_n[g]++; events++;
    }
    if (group_n[0] < 3 || group_n[1] < 3) return 0;
    for (int ch = 0; ch < 2; ++ch)
        for (int g = 0; g < 2; ++g)
            for (int j = 0; j < period; ++j) profile[ch][g][j] /= group_n[g];
    int left = period/2, right = period/2;
    for (int j = 0; j < period; ++j) {
        const double v = 0.5*(profile[0][0][j] + profile[0][1][j]);
        if (fabs(v) > fmax(3.0*sigma[0]/sqrt(events/2.0), 0.01*peak)) {
            if (j < left) left = j;
            if (j > right) right = j;
        }
    }
    if (left > 0) left--;
    if (right < period-1) right++;
    if (right-left+1 > period/3 || left == 0 || right == period-1) return 0;
    int event = 0;
    for (int base = start; base + period <= n; base += period, ++event) {
        const int g = 1 - event%2;
        for (int ch = 0; ch < 2; ++ch) {
            const double sign = sign_of(res[ch][base+period/2]);
            for (int j = left; j <= right; ++j) {
                const double r = res[ch][base+j];
                const double e = r - sign*profile[ch][g][j];
                before[ch] += r*r; after[ch] += e*e;
            }
        }
    }
    for (int ch = 0; ch < 2; ++ch) {
        if (!(before[ch] > 4.0*after[ch])) return 0;
        if (after[ch] > 9.0*sigma[ch]*sigma[ch]*events*(right-left+1)) return 0;
        double outside = 0.0; int used = 0;
        for (int i = 0; i < n; ++i) {
            const int j = ((i-start)%period + period)%period;
            if (j >= left && j <= right) continue;
            outside += res[ch][i]*res[ch][i]; used++;
        }
        /* Multiple carriers, broad pulses, or irregular excitation are not
         * silently relabelled dither. The non-pulse region must fit the tone. */
        if (outside > 4.0*sigma[ch]*sigma[ch]*used) return 0;
    }
    double *out[2] = {loop->output_a, loop->output_b};
    event = -1;
    for (int base = start-period; base < n; base += period, ++event) {
        const int g = (event % 2 == 0) ? 1 : 0;
        for (int ch = 0; ch < 2; ++ch) {
            double dot = 0.0;
            for (int j = left; j <= right; ++j)
                if (base+j >= 0 && base+j < n) dot += res[ch][base+j]*profile[ch][g][j];
            const double sign = sign_of(dot);
            for (int j = left; j <= right; ++j)
                if (base+j >= 0 && base+j < n) out[ch][base+j] -= sign*profile[ch][g][j];
        }
    }
    loop->output.period_samples = (unsigned)period;
    loop->output.events = (unsigned)events;
    loop->output.validation_db_a = 10.0*log10(before[0]/fmax(after[0], 1e-30));
    loop->output.validation_db_b = 10.0*log10(before[1]/fmax(after[1], 1e-30));
    loop->output.status = ADC_CAL_LOOP_OUTPUT_CANCELLED;
    return 0;
}

int adc_cal_loop_make_output(adc_cal_loop_t *loop, const uint8_t *raw, size_t n_bytes)
{
    adc_cal_loop_prep_t prep;
    if (!loop) return -1;
    memset(&loop->output, 0, sizeof(loop->output));
    if (adc_cal_loop_prepare_capture(loop, raw, n_bytes, &prep) != 0 ||
        prep.n_samples < 512 || prep.align_margin < loop->opt.min_align_margin) return -1;
    for (size_t i = 0; i < prep.n_samples; ++i)
        if (loop->ch_a[i] <= -8192.0 || loop->ch_a[i] >= 8191.0 ||
            loop->ch_b[i] <= -8192.0 || loop->ch_b[i] >= 8191.0) return -1;
    for (size_t i = 0; i < prep.n_samples; ++i) {
        loop->ch_a[i] += loop->opt.input_offset[0];
        loop->ch_b[i] += loop->opt.input_offset[1];
    }
    state_apply(&loop->state, loop->ch_a, loop->ch_b, loop->cal_a, loop->cal_b,
                (int)prep.n_samples);
    return adc_cal_loop_cancel_dither(loop, loop->cal_a, loop->cal_b, prep.n_samples);
}

const char *adc_cal_loop_output_status_name(adc_cal_loop_output_status_t status)
{
    switch (status) {
    case ADC_CAL_LOOP_OUTPUT_CANCELLED: return "cancelled";
    case ADC_CAL_LOOP_OUTPUT_NO_TONE: return "no-tone";
    case ADC_CAL_LOOP_OUTPUT_NO_DITHER: return "no-dither";
    case ADC_CAL_LOOP_OUTPUT_AMBIGUOUS: return "ambiguous";
    case ADC_CAL_LOOP_OUTPUT_UNAVAILABLE: return "unavailable";
    }
    return "unavailable";
}
