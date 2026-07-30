/* Private implementation split out of butils.c. */
void adc_timing_capture(uint32_t frame_count) {
    static int16_t even_reference[ADC_CHANNEL_SAMPLE_COUNT];
    static int16_t odd_reference[ADC_CHANNEL_SAMPLE_COUNT];
    static calibration_frame_workspace_t frame_workspace;
    uint32_t captured_frames = 0U;
    uint32_t accepted_frames = 0U;
    size_t reference_count = 0U;
    double even_variance;
    double odd_variance;
    int timing_channel = -1;
    if ((frame_count == 0U) || (frame_count > ADC_TIMING_MAX_FRAMES))     {
        ERR("Timing frame count must be between 1 and %u.",             ADC_TIMING_MAX_FRAMES);
        return;
    }
    if (adc_sweep_active)     {
        ERR("Another automatic ADC capture is already in progress.");
        return;
    }
    print_adc_analysis_rate_header();
    if (calibration_prepare_uploaded_dac_reference(             even_reference, odd_reference, &reference_count,             &even_variance, &odd_variance, 1) != 0) {
        return;
    }
    adc_sweep_active = 1;
    g_quiet_calibration_capture = true;
    xil_printf("\r\n");
    xil_printf("========================================\r\n");
    xil_printf("Starting DAC-Referenced ADC Timing Test\r\n");
    xil_printf("Reference source : uploaded DAC TXT\r\n");
    xil_printf("Requested frames : %lu\r\n",                (unsigned long)frame_count);
    xil_printf("Samples/channel  : %u\r\n",                ADC_CHANNEL_SAMPLE_COUNT);
    xil_printf("Analysis         : uploaded reference + circular/fractional alignment\r\n");
    xil_printf("========================================\r\n");
    print_adc_test_configuration(ADC_CHANNEL_SAMPLE_COUNT);
    for (uint32_t frame = 1U;
    frame <= frame_count;
    ++frame)     {
        calibration_frame_config_t frame_config;
        calibration_aligned_frame_t aligned_frame;
        int analysis_status;
        xil_printf("\r\n[TIMING_FRAME_BEGIN %lu/%lu]\r\n",                    (unsigned long)frame,                    (unsigned long)frame_count);
        frame_config.locked_channel = timing_channel;
        frame_config.adc_gain_correction = 1.0f;
        frame_config.adc_offset_correction = 0.0f;
        frame_config.reference_scale = 1.0f;
        frame_config.reject_clipped_input = false;
        analysis_status = calibration_capture_and_align(             even_reference, odd_reference, reference_count,             &frame_config, &frame_workspace, &aligned_frame         );
        if (aligned_frame.capture_succeeded) {
            ++captured_frames;
        }
        xil_printf("[TIMING_RESULT %lu/%lu]\r\n",                    (unsigned long)frame,                    (unsigned long)frame_count);
        xil_printf("Sample count           : %lu\r\n",                    (unsigned long)reference_count);
        xil_printf("Channel                : %s\r\n",                    aligned_frame.selected_channel_name);
        xil_printf("Canonical reference phase: %s\r\n",                    aligned_frame.canonical_reference_phase == 0 ?                    "EVEN" : "ODD");
        xil_printf("Selected input phase     : %s\r\n",                    aligned_frame.selected_phase_name);
        if ((analysis_status != 0) || !aligned_frame.frame_valid) {
            if (!aligned_frame.capture_succeeded) {
                xil_printf("[TIMING_FRAME_FAILED %lu]\r\n",                            (unsigned long)frame);
            }
            xil_printf("Timing status          : REJECTED\r\n");
            xil_printf("Rejection reason       : %s\r\n",                 aligned_frame.rejection_reason);
            goto timing_frame_end;
        }
        xil_printf("Integer lag            : %ld samples\r\n",                    (long)aligned_frame.integer_lag);
        print_float_value("Fractional lag", aligned_frame.fractional_lag,                           " samples");
        print_float_value("Total estimated lag", aligned_frame.total_lag,                           " samples");
        print_float_value("Correlation", aligned_frame.correlation, "");
        print_overlap_measurements(             &aligned_frame.metrics, &aligned_frame.overlap);
        {
            const float normalized_gain =                 aligned_frame.metrics.measured_gain *                 (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
            const float normalized_offset =                 aligned_frame.metrics.measured_offset /                 CAL_ADC_FULL_SCALE_CODES;
            print_float_value("Normalized gain", normalized_gain, "");
            print_float_value("Scale deviation from nominal",                               normalized_gain - 1.0f, "");
            print_float_value("Normalized DC offset", normalized_offset,                               " full-scale");
        }
        if (timing_channel < 0)             timing_channel = aligned_frame.selected_channel;
        ++accepted_frames;
        xil_printf("Timing status          : PASS\r\n");
        timing_frame_end:         xil_printf("[TIMING_FRAME_END %lu]\r\n",                    (unsigned long)frame);
        if (frame < frame_count)         {
            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
        }
    }
    g_quiet_calibration_capture = false;
    adc_sweep_active = 0;
    xil_printf("\r\n");
    xil_printf("========================================\r\n");
    xil_printf("DAC-referenced timing test finished.\r\n");
    xil_printf("Captured frames    : %lu/%lu\r\n",                (unsigned long)captured_frames,                (unsigned long)frame_count);
    xil_printf("Accepted frames    : %lu/%lu\r\n",                (unsigned long)accepted_frames,                (unsigned long)frame_count);
    xil_printf("Timing channel     : %s\r\n",                timing_channel == 0 ? "Channel A" :                (timing_channel == 1 ? "Channel B" : "none"));
    xil_printf("Reference source   : uploaded DAC TXT\r\n");
    xil_printf("No UDP receiver was required.\r\n");
    xil_printf("========================================\r\n");
}
static double calibration_reference_variance(     const int16_t *samples,     size_t sample_count ) {
    double mean = 0.0;
    double sum = 0.0;
    if ((samples == NULL) || (sample_count == 0U)) return 0.0;
    for (size_t i = 0U;
    i < sample_count;
    ++i) mean += samples[i];
    mean /= (double)sample_count;
    for (size_t i = 0U;
    i < sample_count;
    ++i) {
        const double centered = (double)samples[i] - mean;
        sum += centered * centered;
    }
    return sum / (double)sample_count;
}
static int calibration_build_adc_reference_from_raw_dac(     const int16_t *raw_dac,     size_t raw_count,     int16_t *even_reference,     int16_t *odd_reference,     size_t output_capacity,     size_t *reconstructed_count ) {
    const double source_step = DAC_SAMPLE_RATE_HZ /                                adc_get_effective_sample_rate_hz();
    if ((raw_dac == NULL) || (even_reference == NULL) ||         (odd_reference == NULL) || (reconstructed_count == NULL) ||         (output_capacity == 0U)) return -1;
    if (!(source_step > 0.0) || !isfinite(source_step)) return -2;
    for (size_t i = 0U;
    i < output_capacity;
    ++i) {
        const double base_position = (double)i * source_step;
        const double positions[2] = {
            base_position, base_position + 1.0}
            ;
            int16_t *outputs[2] = {
                even_reference, odd_reference}
                ;
                for (size_t phase = 0U;
                phase < 2U;
                ++phase) {
                    const double position = fmod(positions[phase], (double)raw_count);
                    const size_t index0 = (size_t)floor(position);
                    const size_t index1 = (index0 + 1U) % raw_count;
                    const double fraction = position - (double)index0;
                    long interpolated;
                    if (index0 >= raw_count) return -3;
                    interpolated = lround((1.0 - fraction) * raw_dac[index0] +                                   fraction * raw_dac[index1]);
                    if (interpolated > INT16_MAX) interpolated = INT16_MAX;
                    if (interpolated < INT16_MIN) interpolated = INT16_MIN;
                    outputs[phase][i] = calibration_convert_reference_to_adc_units(                 (int16_t)interpolated);
                }
            }
            *reconstructed_count = output_capacity;
            return 0;
        }
        typedef struct {
            size_t bin;
            double magnitude;
        }
        calibration_spectral_peak_t;
        typedef struct {
            size_t dominant_bin;
            double dominant_frequency_hz;
            double bin245_magnitude;
            calibration_spectral_peak_t peaks[CAL_REF_SPECTRAL_PEAK_COUNT];
        }
        calibration_spectrum_t;
        static double calibration_goertzel_magnitude(     const int16_t *samples,     size_t sample_count,     size_t bin,     double mean ) {
            const double omega = 6.28318530717958647692 * (double)bin /                          (double)sample_count;
            const double coefficient = 2.0 * cos(omega);
            double previous = 0.0;
            double previous2 = 0.0;
            for (size_t i = 0U;
            i < sample_count;
            ++i) {
                const double current = ((double)samples[i] - mean) +                                coefficient * previous - previous2;
                previous2 = previous;
                previous = current;
            }
            {
                double power = previous2 * previous2 + previous * previous -                        coefficient * previous * previous2;
                if (power < 0.0 && power > -1.0e-6) power = 0.0;
                return power > 0.0 ? sqrt(power) / (double)sample_count : 0.0;
            }
        }
        static int calibration_calculate_full_spectrum(     const int16_t *samples,     size_t sample_count,     double sample_rate_hz,     calibration_spectrum_t *spectrum ) {
            double mean = 0.0;
            if ((samples == NULL) || (spectrum == NULL) || (sample_count < 4U))         return -1;
            memset(spectrum, 0, sizeof(*spectrum));
            for (size_t i = 0U;
            i < sample_count;
            ++i) mean += samples[i];
            mean /= (double)sample_count;
            for (size_t bin = 1U;
            bin < sample_count / 2U;
            ++bin) {
                const double magnitude = calibration_goertzel_magnitude(             samples, sample_count, bin, mean);
                if (bin == 245U) spectrum->bin245_magnitude = magnitude;
                for (size_t rank = 0U;
                rank < CAL_REF_SPECTRAL_PEAK_COUNT;
                ++rank) {
                    if (magnitude > spectrum->peaks[rank].magnitude) {
                        for (size_t move = CAL_REF_SPECTRAL_PEAK_COUNT - 1U;
                        move > rank;
                        --move)                     spectrum->peaks[move] = spectrum->peaks[move - 1U];
                        spectrum->peaks[rank].bin = bin;
                        spectrum->peaks[rank].magnitude = magnitude;
                        break;
                    }
                }
            }
            spectrum->dominant_bin = spectrum->peaks[0].bin;
            spectrum->dominant_frequency_hz =         (double)spectrum->dominant_bin * sample_rate_hz /         (double)sample_count;
            return spectrum->dominant_bin != 0U ? 0 : -2;
        }
        /* Goertzel evaluates one bin of an arbitrary-length DFT.  Stage 4 therefore  * analyzes all 800 samples directly and does not need zero padding. */
        static double adc_performance_goertzel_power(     const double *windowed_samples, size_t sample_count, size_t bin) {
            const double omega = 6.28318530717958647692 * (double)bin /                          (double)sample_count;
            const double coefficient = 2.0 * cos(omega);
            double previous = 0.0;
            double previous2 = 0.0;
            for (size_t i = 0U;
            i < sample_count;
            ++i) {
                const double current = windowed_samples[i] +                                coefficient * previous - previous2;
                previous2 = previous;
                previous = current;
            }
            {
                double power = previous2 * previous2 + previous * previous -                        coefficient * previous * previous2;
                if (power < 0.0 && power > -1.0e-6) power = 0.0;
                return power;
            }
        }
        static void adc_performance_statistics_init(     adc_performance_statistics_t *statistics) {
            memset(statistics, 0, sizeof(*statistics));
            statistics->minimum = DBL_MAX;
            statistics->maximum = -DBL_MAX;
        }
        static void adc_performance_statistics_add(     adc_performance_statistics_t *statistics, double value) {
            const double delta = value - statistics->mean;
            ++statistics->count;
            statistics->mean += delta / (double)statistics->count;
            statistics->m2 += delta * (value - statistics->mean);
            if (value < statistics->minimum) statistics->minimum = value;
            if (value > statistics->maximum) statistics->maximum = value;
        }
        static float adc_performance_statistics_stddev(     const adc_performance_statistics_t *statistics) {
            return statistics->count > 1U ?         (float)sqrt(statistics->m2 / (double)(statistics->count - 1U)) :         0.0f;
        }
        static void adc_performance_frame_result_reset(     adc_performance_frame_result_t *result, uint32_t frame_number) {
            memset(result, 0, sizeof(*result));
            result->frame_number = frame_number;
            result->mean_residual = NAN;
            result->rmse = NAN;
            result->correlation = NAN;
            result->normalized_gain = NAN;
            result->raw_reference_mean = NAN;
            result->scaled_reference_mean = NAN;
            result->raw_adc_mean = NAN;
            result->offset_corrected_adc_mean = NAN;
            result->gain_corrected_adc_mean = NAN;
            result->reference_fit_s_error_db = NAN;
            result->sndr_db = NAN;
            result->enob = NAN;
            result->sample_rate_hz = NAN;
            result->expected_fundamental_hz = NAN;
            result->expected_fundamental_bin = NAN;
            result->detected_fundamental_hz = NAN;
            result->cycles_in_window = NAN;
            result->window_name = "UNAVAILABLE";
            result->failure_reason = "performance frame not evaluated";
        }
        /* The integrated calibration output convention is deliberately centralized:  *   final_code[i] = round(gain * (raw_adc[i] + offset))  *   expected[i]   = nominal_system_gain * scaled_reference[i]  * Offset verification uses this same model with gain == 1. */
        static int adc_calculate_final_reference_metrics(     const calibration_aligned_frame_t *frame,     float final_gain_correction,     float final_offset_correction,     float nominal_system_gain,     adc_final_reference_metrics_t *metrics) {
            const size_t count = frame != NULL ?         frame->valid_analysis_sample_count : 0U;
            double residual_sum = 0.0;
            double residual_square_sum = 0.0;
            double raw_reference_sum = 0.0;
            double scaled_reference_sum = 0.0;
            double raw_adc_sum = 0.0;
            double offset_corrected_sum = 0.0;
            double gain_corrected_sum = 0.0;
            double fitted_signal_power = 0.0;
            double fitted_error_power = 0.0;
            if (frame == NULL || !frame->frame_valid || metrics == NULL ||         frame->aligned_corrected_adc_samples == NULL ||         frame->aligned_raw_adc_samples == NULL ||         frame->aligned_reference_samples == NULL ||         frame->canonical_reference_window == NULL ||         count != CAL_FIXED_WINDOW_LENGTH ||         count != frame->calibration_window_length ||         !isfinite(final_gain_correction) ||         final_gain_correction <= 0.0f ||         !isfinite(final_offset_correction) ||         !isfinite(nominal_system_gain) || nominal_system_gain <= 0.0f)         return -1;
            memset(metrics, 0, sizeof(*metrics));
            for (size_t i = 0U;
            i < count;
            ++i) {
                const double raw_adc = frame->aligned_raw_adc_samples[i];
                const double offset_corrected =             calibration_apply_offset_correction(                 raw_adc, final_offset_correction);
                const double gain_corrected =             (double)final_gain_correction * offset_corrected;
                const long final_code = lround(gain_corrected);
                const double expected = (double)nominal_system_gain *             frame->aligned_reference_samples[i];
                const double residual =             (double)frame->aligned_corrected_adc_samples[i] - expected;
                if (!isfinite(gain_corrected) || !isfinite(expected) ||             final_code < CALIBRATION_ADC_MIN_CODE ||             final_code > CALIBRATION_ADC_MAX_CODE ||             frame->aligned_corrected_adc_samples[i] != (int16_t)final_code)             return -2;
                residual_sum += residual;
                residual_square_sum += residual * residual;
                raw_reference_sum += frame->canonical_reference_window[i];
                scaled_reference_sum += frame->aligned_reference_samples[i];
                raw_adc_sum += raw_adc;
                offset_corrected_sum += offset_corrected;
                gain_corrected_sum += frame->aligned_corrected_adc_samples[i];
            }
            metrics->mean_residual =         (float)(residual_sum / (double)count);
            metrics->rmse =         (float)sqrt(residual_square_sum / (double)count);
            metrics->raw_reference_mean =         (float)(raw_reference_sum / (double)count);
            metrics->scaled_reference_mean =         (float)(scaled_reference_sum / (double)count);
            metrics->raw_adc_mean = (float)(raw_adc_sum / (double)count);
            metrics->offset_corrected_adc_mean =         (float)(offset_corrected_sum / (double)count);
            metrics->gain_corrected_adc_mean =         (float)(gain_corrected_sum / (double)count);
            /* Debug-only reference-fit diagnostic.  The official SNDR remains the      * spectral result; this ratio uses the best affine reference fit. */
            for (size_t i = 0U;
            i < count;
            ++i) {
                const double centered_reference =             (double)frame->aligned_reference_samples[i] -             frame->metrics.reference_mean;
                const double fitted_signal =             frame->metrics.measured_gain * centered_reference;
                const double fitted = frame->metrics.measured_gain *             frame->aligned_reference_samples[i] +             frame->metrics.measured_offset;
                const double error =             (double)frame->aligned_corrected_adc_samples[i] - fitted;
                fitted_signal_power += fitted_signal * fitted_signal;
                fitted_error_power += error * error;
            }
            metrics->reference_fit_s_error_db =         fitted_signal_power > DBL_EPSILON &&         fitted_error_power > DBL_EPSILON ?         (float)(10.0 * log10(fitted_signal_power / fitted_error_power)) :         NAN;
            return isfinite(metrics->mean_residual) &&            isfinite(metrics->rmse) &&            isfinite(metrics->raw_reference_mean) &&            isfinite(metrics->scaled_reference_mean) &&            isfinite(metrics->raw_adc_mean) &&            isfinite(metrics->offset_corrected_adc_mean) &&            isfinite(metrics->gain_corrected_adc_mean) ? 0 : -3;
        }
        /* Per-frame measurement only.  The gain controller targets  * measured/nominal gain = 1, so the expected waveform is:  *   residual[i] = calibrated_adc[i]  *                 - nominal_system_gain * scaled_reference[i]  *   RMSE = sqrt(mean(residual^2))  *   SNDR = 10*log10(Pfundamental / Pnoise+distortion)  *   ENOB = (SNDR - 1.76)/6.02  * DC and the fundamental main lobe are removed from total power.  Harmonic  * bins intentionally remain in the SNDR denominator. */
        static int adc_evaluate_performance_frame(     const calibration_aligned_frame_t *frame,     uint32_t frame_number,     float final_gain_correction,     float final_offset_correction,     float nominal_system_gain,     double sample_rate_hz,     double expected_fundamental_hz,     adc_performance_frame_result_t *result) {
            static double adc_windowed[CAL_FIXED_WINDOW_LENGTH];
            static double reference_windowed[CAL_FIXED_WINDOW_LENGTH];
            const size_t sample_count = frame != NULL ?         frame->valid_analysis_sample_count : 0U;
            size_t half_spectrum;
            size_t search_first;
            size_t search_last;
            size_t fundamental_bin = 0U;
            size_t signal_half_width;
            adc_final_reference_metrics_t reference_metrics;
            double adc_mean = 0.0;
            double reference_mean = 0.0;
            double best_reference_power = 0.0;
            double total_non_dc_power = 0.0;
            double signal_power = 0.0;
            double noise_distortion_power;
            if (result == NULL) return -1;
            adc_performance_frame_result_reset(result, frame_number);
            if (frame == NULL || !frame->frame_valid ||         sample_count != CAL_FIXED_WINDOW_LENGTH ||         frame->calibration_window_length != sample_count ||         frame->aligned_corrected_adc_samples == NULL ||         frame->aligned_reference_samples == NULL ||         !isfinite(frame->analysis_reference_scale) ||         frame->analysis_reference_scale <= 0.0f ||         !isfinite(nominal_system_gain) || nominal_system_gain <= 0.0f) {
                result->failure_reason = "invalid final calibrated analysis window";
                return -2;
            }
            result->sample_count = sample_count;
            result->transform_length = sample_count;
            result->sample_rate_hz = sample_rate_hz;
            result->fundamental_known =         isfinite(expected_fundamental_hz) &&         expected_fundamental_hz > 0.0 &&         isfinite(sample_rate_hz) &&         expected_fundamental_hz < 0.5 * sample_rate_hz;
            result->expected_fundamental_hz = result->fundamental_known ?         expected_fundamental_hz : NAN;
            if (adc_calculate_final_reference_metrics(             frame, final_gain_correction, final_offset_correction,             nominal_system_gain, &reference_metrics) != 0) {
                result->failure_reason = "final sample equation mismatch";
                return -3;
            }
            result->mean_residual = reference_metrics.mean_residual;
            result->rmse = reference_metrics.rmse;
            result->raw_reference_mean = reference_metrics.raw_reference_mean;
            result->scaled_reference_mean = reference_metrics.scaled_reference_mean;
            result->raw_adc_mean = reference_metrics.raw_adc_mean;
            result->offset_corrected_adc_mean =         reference_metrics.offset_corrected_adc_mean;
            result->gain_corrected_adc_mean =         reference_metrics.gain_corrected_adc_mean;
            result->reference_fit_s_error_db =         reference_metrics.reference_fit_s_error_db;
            /* The final-frame analyzer already calculated Pearson correlation on      * these same aligned arrays.  Positive nominal scaling of the reference      * does not change Pearson correlation. */
            result->correlation = frame->metrics.correlation;
            result->normalized_gain =         frame->metrics.measured_gain / nominal_system_gain;
            if (!isfinite(result->mean_residual) ||         !isfinite(result->rmse) || result->rmse < 0.0f ||         !isfinite(result->correlation) ||         result->correlation < CAL_DAC_REF_MIN_CORRELATION ||         result->correlation > 1.0001f ||         !isfinite(result->normalized_gain)) {
                result->failure_reason = "invalid reference-based performance metrics";
                return -4;
            }
            if (!isfinite(result->sample_rate_hz) ||         result->sample_rate_hz <= 0.0 || sample_count < 8U) {
                result->failure_reason = "invalid sample rate or transform length";
                return -5;
            }
            if (result->fundamental_known) {
                result->cycles_in_window = result->expected_fundamental_hz *             (double)sample_count / result->sample_rate_hz;
                result->expected_fundamental_bin = result->cycles_in_window;
                result->coherent_sampling =             fabs(result->cycles_in_window -                  round(result->cycles_in_window)) <=             CAL_COHERENCE_TOLERANCE;
            }
            result->window_name = result->coherent_sampling ?         "RECTANGULAR" : "HANN";
            for (size_t i = 0U;
            i < sample_count;
            ++i) {
                adc_mean += frame->aligned_corrected_adc_samples[i];
                reference_mean += frame->aligned_reference_samples[i];
            }
            adc_mean /= (double)sample_count;
            reference_mean /= (double)sample_count;
            for (size_t i = 0U;
            i < sample_count;
            ++i) {
                const double window = result->coherent_sampling ? 1.0 :             0.5 - 0.5 * cos(6.28318530717958647692 * (double)i /                             (double)(sample_count - 1U));
                adc_windowed[i] =             ((double)frame->aligned_corrected_adc_samples[i] - adc_mean) *             window;
                reference_windowed[i] =             ((double)frame->aligned_reference_samples[i] - reference_mean) *             window;
                if (!isfinite(adc_windowed[i]) ||             !isfinite(reference_windowed[i])) {
                    result->failure_reason = "invalid windowed sample";
                    return -6;
                }
            }
            half_spectrum = sample_count / 2U;
            if (result->fundamental_known) {
                const size_t expected_bin = (size_t)lround(             result->expected_fundamental_bin);
                search_first = expected_bin >                 ADC_PERFORMANCE_FUNDAMENTAL_SEARCH_BINS ?             expected_bin - ADC_PERFORMANCE_FUNDAMENTAL_SEARCH_BINS : 1U;
                search_last = expected_bin +             ADC_PERFORMANCE_FUNDAMENTAL_SEARCH_BINS;
                if (search_last > half_spectrum)             search_last = half_spectrum;
            }
            else {
                search_first = 1U;
                search_last = half_spectrum;
            }
            if (search_first > search_last || search_last > half_spectrum) {
                result->failure_reason = "expected fundamental is outside spectrum";
                return -7;
            }
            for (size_t bin = search_first;
            bin <= search_last;
            ++bin) {
                const double power = adc_performance_goertzel_power(             reference_windowed, sample_count, bin);
                if (isfinite(power) && power > best_reference_power) {
                    best_reference_power = power;
                    fundamental_bin = bin;
                }
            }
            if (fundamental_bin == 0U || !isfinite(best_reference_power) ||         best_reference_power <= DBL_EPSILON) {
                result->failure_reason = "reference fundamental was not detected";
                return -8;
            }
            result->fundamental_bin = fundamental_bin;
            result->detected_fundamental_hz =         (double)fundamental_bin * result->sample_rate_hz /         (double)sample_count;
            signal_half_width = result->coherent_sampling ? 0U :         ADC_PERFORMANCE_HANN_SIGNAL_HALF_WIDTH;
            result->signal_bin_first = fundamental_bin >             signal_half_width ?         fundamental_bin - signal_half_width : 1U;
            result->signal_bin_last = fundamental_bin +         signal_half_width;
            if (result->signal_bin_last > half_spectrum)         result->signal_bin_last = half_spectrum;
            result->dc_power = adc_performance_goertzel_power(         adc_windowed, sample_count, 0U);
            /* Numerator and denominator use the same DFT and window power scale.      * No one-sided amplitude or window correction is applied to only one      * term, so the common scale cancels in the SNDR ratio. */
            for (size_t bin = 1U;
            bin <= half_spectrum;
            ++bin) {
                const double power = adc_performance_goertzel_power(             adc_windowed, sample_count, bin);
                const double one_sided_weight =             (sample_count % 2U == 0U && bin == half_spectrum) ? 1.0 : 2.0;
                const double weighted_power = one_sided_weight * power;
                if (!isfinite(weighted_power) || weighted_power < 0.0) {
                    result->failure_reason = "invalid spectral power";
                    return -9;
                }
                total_non_dc_power += weighted_power;
                if (bin >= result->signal_bin_first &&             bin <= result->signal_bin_last)             signal_power += weighted_power;
            }
            noise_distortion_power = total_non_dc_power - signal_power;
            result->total_non_dc_power = total_non_dc_power;
            result->signal_power = signal_power;
            result->noise_distortion_power = noise_distortion_power;
            if (!isfinite(result->dc_power) || result->dc_power < 0.0 ||         !isfinite(signal_power) || signal_power <= DBL_EPSILON ||         !isfinite(noise_distortion_power) ||         noise_distortion_power <= DBL_EPSILON) {
                result->failure_reason = "nonpositive signal or noise-and-distortion power";
                return -10;
            }
            result->sndr_db = (float)(10.0 * log10(         signal_power / noise_distortion_power));
            result->enob = (result->sndr_db - 1.76f) / 6.02f;
            if (!isfinite(result->sndr_db) || !isfinite(result->enob)) {
                result->sndr_db = NAN;
                result->enob = NAN;
                result->failure_reason = "nonfinite SNDR or ENOB";
                return -11;
            }
            result->valid = true;
            result->failure_reason = "none";
            return 0;
        }
        static int adc_evaluate_performance_batch(     const calibration_pending_frame_t *saved_output,     float final_gain_correction,     float final_offset_correction,     float nominal_system_gain,     double expected_fundamental_hz,     float offset_verification_residual,     float offset_verification_standard_error,     float post_gain_residual,     float post_gain_residual_standard_error,     adc_performance_result_t *result) {
            static calibration_frame_workspace_t workspace;
            adc_performance_statistics_t residual_statistics;
            adc_performance_statistics_t rmse_statistics;
            adc_performance_statistics_t correlation_statistics;
            adc_performance_statistics_t sndr_statistics;
            adc_performance_statistics_t enob_statistics;
            adc_performance_statistics_t normalized_gain_statistics;
            const bool previous_quiet_capture = g_quiet_calibration_capture;
            if (result == NULL) return -1;
            memset(result, 0, sizeof(*result));
            result->mean_residual = NAN;
            result->residual_stddev = NAN;
            result->residual_standard_error = NAN;
            result->residual_minimum = NAN;
            result->residual_maximum = NAN;
            result->rmse = NAN;
            result->rmse_stddev = NAN;
            result->correlation = NAN;
            result->minimum_correlation = NAN;
            result->sndr_db = NAN;
            result->sndr_stddev = NAN;
            result->minimum_sndr_db = NAN;
            result->enob = NAN;
            result->enob_stddev = NAN;
            result->minimum_enob = NAN;
            result->mean_normalized_gain = NAN;
            result->normalized_gain_stddev = NAN;
            result->offset_verification_residual = offset_verification_residual;
            result->offset_verification_standard_error =         offset_verification_standard_error;
            result->offset_residual_difference = NAN;
            result->combined_offset_standard_error = NAN;
            result->offset_difference_z_like = NAN;
            result->post_gain_residual = post_gain_residual;
            result->post_gain_residual_standard_error =         post_gain_residual_standard_error;
            result->calibration_channel = -1;
            result->canonical_reference_phase = -1;
            result->final_offset_correction = NAN;
            result->final_gain_correction = NAN;
            result->frames_attempted = ADC_PERFORMANCE_FRAMES;
            result->frames_rejected = ADC_PERFORMANCE_FRAMES;
            result->failure_reason = "performance batch not evaluated";
            if (saved_output == NULL || !saved_output->valid ||         saved_output->consumed ||         saved_output->analysis_sample_count != CAL_FIXED_WINDOW_LENGTH ||         saved_output->calibration_window_length != CAL_FIXED_WINDOW_LENGTH ||         !isfinite(saved_output->analysis_reference_scale) ||         saved_output->analysis_reference_scale <= 0.0f ||         !isfinite(saved_output->effective_sample_rate_hz) ||         saved_output->effective_sample_rate_hz <= 0.0 ||         !isfinite(final_gain_correction) || final_gain_correction <= 0.0f ||         !isfinite(final_offset_correction) ||         !isfinite(nominal_system_gain) || nominal_system_gain <= 0.0f ||         fabsf(saved_output->software_gain_correction -               final_gain_correction) > 1.0e-6f ||         fabsf(saved_output->software_offset_correction -               final_offset_correction) > 1.0e-6f ||         fabsf(calibration_software_gain_correction() -               final_gain_correction) > 1.0e-6f ||         fabsf(calibration_software_offset_correction() -               final_offset_correction) > 1.0e-6f) {
                result->failure_reason = "invalid frozen performance configuration";
                return -2;
            }
            result->calibration_channel = saved_output->selected_channel;
            result->canonical_reference_phase =         saved_output->canonical_reference_phase;
            result->fixed_window_start = saved_output->calibration_window_start;
            result->fixed_window_length = saved_output->calibration_window_length;
            result->final_offset_correction = final_offset_correction;
            result->final_gain_correction = final_gain_correction;
            adc_performance_statistics_init(&residual_statistics);
            adc_performance_statistics_init(&rmse_statistics);
            adc_performance_statistics_init(&correlation_statistics);
            adc_performance_statistics_init(&sndr_statistics);
            adc_performance_statistics_init(&enob_statistics);
            adc_performance_statistics_init(&normalized_gain_statistics);
            /* The owned-reference capture path performs the same local alignment,      * fixed-window mapping, offset correction, and gain correction used by      * final gain verification.  Stage 4 only reads the frozen coefficients. */
            g_quiet_calibration_capture = true;
            for (uint32_t frame_number = 1U;
            frame_number <= ADC_PERFORMANCE_FRAMES;
            ++frame_number) {
                adc_performance_frame_result_t *frame_result =             &result->frames[frame_number - 1U];
                calibration_aligned_frame_t frame;
                const char *reason = NULL;
                int status;
                if (frame_number > 1U)             usleep(ADC_TIMING_INTERFRAME_DELAY_US);
                adc_performance_frame_result_reset(frame_result, frame_number);
                status = calibration_capture_against_owned_reference(             saved_output, true, final_gain_correction,             final_offset_correction, saved_output->analysis_reference_scale,             &workspace, &frame, &reason);
                if (status == 0 && frame.frame_valid) {
                    status = adc_evaluate_performance_frame(                 &frame, frame_number, final_gain_correction,                 final_offset_correction, nominal_system_gain,                 saved_output->effective_sample_rate_hz,                 expected_fundamental_hz, frame_result);
                }
                else {
                    frame_result->failure_reason = reason != NULL ?                 reason : "performance capture or alignment failed";
                }
                if (status == 0 && frame_result->valid) {
                    ++result->frames_valid;
                    adc_performance_statistics_add(                 &residual_statistics, frame_result->mean_residual);
                    adc_performance_statistics_add(                 &rmse_statistics, frame_result->rmse);
                    adc_performance_statistics_add(                 &correlation_statistics, frame_result->correlation);
                    adc_performance_statistics_add(                 &sndr_statistics, frame_result->sndr_db);
                    adc_performance_statistics_add(                 &enob_statistics, frame_result->enob);
                    adc_performance_statistics_add(                 &normalized_gain_statistics,                 frame_result->normalized_gain);
                }
                if (ADC_CAL_VERBOSE_DEBUG) {
                    xil_printf("Performance frame %lu/%u: %s\r\n",                        (unsigned long)frame_number,                        ADC_PERFORMANCE_FRAMES,                        frame_result->valid ? "VALID" : "REJECTED");
                    if (frame_result->valid) {
                        print_signed_float_value_or_invalid("  residual mean",                     frame_result->mean_residual, " codes");
                        print_float_value_or_invalid("  RMSE",                     frame_result->rmse, " codes");
                        print_float_value_or_invalid("  correlation",                     frame_result->correlation, "");
                        print_float_value_2("  SNDR",                     frame_result->sndr_db, " dB");
                        print_float_value_2("  ENOB",                     frame_result->enob, " bits");
                    }
                    else {
                        xil_printf("  Reason                : %s\r\n",                     frame_result->failure_reason != NULL ?                     frame_result->failure_reason : "unknown");
                    }
                }
            }
            g_quiet_calibration_capture = previous_quiet_capture;
            result->frames_rejected =         result->frames_attempted - result->frames_valid;
            if (fabsf(calibration_software_gain_correction() -               final_gain_correction) > 1.0e-6f ||         fabsf(calibration_software_offset_correction() -               final_offset_correction) > 1.0e-6f) {
                result->failure_reason =             "calibration coefficient changed during performance batch";
                return -3;
            }
            if (result->frames_valid == 0U) {
                result->failure_reason = "no valid performance frames";
                return -4;
            }
            result->mean_residual = (float)residual_statistics.mean;
            result->residual_stddev =         adc_performance_statistics_stddev(&residual_statistics);
            result->residual_standard_error = result->residual_stddev /         sqrtf((float)result->frames_valid);
            result->residual_minimum = (float)residual_statistics.minimum;
            result->residual_maximum = (float)residual_statistics.maximum;
            result->rmse = (float)rmse_statistics.mean;
            result->rmse_stddev =         adc_performance_statistics_stddev(&rmse_statistics);
            result->correlation = (float)correlation_statistics.mean;
            result->minimum_correlation = (float)correlation_statistics.minimum;
            result->sndr_db = (float)sndr_statistics.mean;
            result->sndr_stddev =         adc_performance_statistics_stddev(&sndr_statistics);
            result->minimum_sndr_db = (float)sndr_statistics.minimum;
            result->enob = (float)enob_statistics.mean;
            result->enob_stddev =         adc_performance_statistics_stddev(&enob_statistics);
            result->minimum_enob = (float)enob_statistics.minimum;
            result->mean_normalized_gain = (float)normalized_gain_statistics.mean;
            result->normalized_gain_stddev =         adc_performance_statistics_stddev(&normalized_gain_statistics);
            result->offset_residual_difference =         isfinite(offset_verification_residual) ?         result->mean_residual - offset_verification_residual : NAN;
            if (isfinite(offset_verification_standard_error) &&         offset_verification_standard_error >= 0.0f &&         isfinite(result->residual_standard_error) &&         result->residual_standard_error >= 0.0f) {
                result->combined_offset_standard_error = hypotf(             offset_verification_standard_error,             result->residual_standard_error);
                result->offset_difference_z_like =             result->combined_offset_standard_error > FLT_EPSILON ?             fabsf(result->offset_residual_difference) /                 result->combined_offset_standard_error : NAN;
            }
            result->reference_metrics_valid =         isfinite(result->mean_residual) &&         isfinite(result->residual_stddev) &&         isfinite(result->residual_standard_error) &&         isfinite(result->residual_minimum) &&         isfinite(result->residual_maximum) &&         isfinite(result->rmse) && isfinite(result->rmse_stddev) &&         isfinite(result->correlation) &&         isfinite(result->minimum_correlation) &&         isfinite(result->mean_normalized_gain) &&         isfinite(result->normalized_gain_stddev);
            result->spectral_metrics_valid =         isfinite(result->sndr_db) && isfinite(result->sndr_stddev) &&         isfinite(result->minimum_sndr_db) && isfinite(result->enob) &&         isfinite(result->enob_stddev) && isfinite(result->minimum_enob);
            result->valid =         result->frames_valid >= ADC_PERFORMANCE_MIN_VALID_FRAMES &&         result->reference_metrics_valid && result->spectral_metrics_valid;
            result->failure_reason = result->valid ? "none" :         result->frames_valid < ADC_PERFORMANCE_MIN_VALID_FRAMES ?         "insufficient valid performance frames" :         "invalid aggregate performance metrics";
            return result->valid ? 0 : -5;
        }
        static void adc_print_performance_result(     const adc_performance_result_t *result) {
            const adc_performance_frame_result_t *diagnostic = NULL;
            if (result == NULL) return;
            xil_printf("Frames evaluated        : %lu/%lu\r\n",                (unsigned long)result->frames_valid,                (unsigned long)result->frames_attempted);
            print_signed_float_value_or_invalid("Mean residual offset",         result->reference_metrics_valid ? result->mean_residual : NAN,         " codes");
            print_float_value_or_invalid("Residual std dev",         result->reference_metrics_valid ? result->residual_stddev : NAN,         " codes");
            print_float_value_or_invalid("Residual standard error",         result->reference_metrics_valid ?             result->residual_standard_error : NAN, " codes");
            print_signed_float_value_or_invalid("Residual minimum",         result->reference_metrics_valid ? result->residual_minimum : NAN,         " codes");
            print_signed_float_value_or_invalid("Residual maximum",         result->reference_metrics_valid ? result->residual_maximum : NAN,         " codes");
            xil_printf("\r\n");
            print_float_value_or_invalid("Mean RMSE",         result->reference_metrics_valid ? result->rmse : NAN, " codes");
            print_float_value_or_invalid("RMSE std dev",         result->reference_metrics_valid ? result->rmse_stddev : NAN,         " codes");
            xil_printf("\r\n");
            print_float_value_or_invalid("Mean correlation",         result->reference_metrics_valid ? result->correlation : NAN, "");
            print_float_value_or_invalid("Minimum correlation",         result->reference_metrics_valid ?             result->minimum_correlation : NAN, "");
            xil_printf("\r\n");
            print_float_value_2("Mean SNDR",         result->spectral_metrics_valid ? result->sndr_db : NAN, " dB");
            print_float_value_2("SNDR std dev",         result->spectral_metrics_valid ? result->sndr_stddev : NAN, " dB");
            print_float_value_2("Minimum SNDR",         result->spectral_metrics_valid ?             result->minimum_sndr_db : NAN, " dB");
            xil_printf("\r\n");
            print_float_value_2("Mean ENOB",         result->spectral_metrics_valid ? result->enob : NAN, " bits");
            print_float_value_2("ENOB std dev",         result->spectral_metrics_valid ? result->enob_stddev : NAN,         " bits");
            print_float_value_2("Minimum ENOB",         result->spectral_metrics_valid ? result->minimum_enob : NAN,         " bits");
            xil_printf("\r\nStatus                  : %s\r\n",                result->valid ? "VALID" : "INVALID");
            if (ADC_CAL_VERBOSE_DEBUG) {
                for (uint32_t i = 0U;
                i < result->frames_attempted;
                ++i) {
                    if (result->frames[i].valid) {
                        diagnostic = &result->frames[i];
                        break;
                    }
                }
                xil_printf("\r\nPerformance batch diagnostics:\r\n");
                xil_printf("  Calibration channel   : %s\r\n",             calibration_channel_name(result->calibration_channel));
                xil_printf("  Canonical phase       : %s\r\n",             result->canonical_reference_phase == 0 ? "EVEN" :             result->canonical_reference_phase == 1 ? "ODD" :             "UNAVAILABLE");
                if (result->fixed_window_length > 0U) {
                    xil_printf("  Fixed window          : %lu ... %lu (%lu samples)\r\n",                 (unsigned long)result->fixed_window_start,                 (unsigned long)(result->fixed_window_start +                     result->fixed_window_length - 1U),                 (unsigned long)result->fixed_window_length);
                }
                else {
                    xil_printf("  Fixed window          : UNAVAILABLE\r\n");
                }
                print_float_value_or_invalid("  Offset correction",             result->final_offset_correction, " codes");
                print_float_value_or_invalid("  Gain correction",             result->final_gain_correction, "");
                xil_printf("  Valid frames          : %lu/%lu (minimum %u)\r\n",             (unsigned long)result->frames_valid,             (unsigned long)result->frames_attempted,             ADC_PERFORMANCE_MIN_VALID_FRAMES);
                print_float_value_or_invalid("  Offset verification",             result->offset_verification_residual, " codes");
                print_float_value_or_invalid("  Offset verification SE",             result->offset_verification_standard_error, " codes");
                print_float_value_or_invalid("  Post-gain residual",             result->post_gain_residual, " codes");
                print_float_value_or_invalid("  Post-gain residual SE",             result->post_gain_residual_standard_error, " codes");
                print_float_value_or_invalid("  Performance residual",             result->mean_residual, " codes");
                print_float_value_or_invalid("  Residual difference",             result->offset_residual_difference, " codes");
                print_float_value_or_invalid("  Combined residual SE",             result->combined_offset_standard_error, " codes");
                print_float_value_or_invalid("  Residual difference z-like",             result->offset_difference_z_like, "");
                print_float_value_or_invalid("  Mean normalized gain",             result->mean_normalized_gain, "");
                print_float_value_or_invalid("  Normalized gain std dev",             result->normalized_gain_stddev, "");
                if (diagnostic != NULL) {
                    xil_printf("\r\nFirst valid frame spectral diagnostics:\r\n");
                    xil_printf("  Frame                 : %lu\r\n",                 (unsigned long)diagnostic->frame_number);
                    xil_printf("  Samples               : %lu\r\n",                 (unsigned long)diagnostic->sample_count);
                    xil_printf("  DFT length            : %lu\r\n",                 (unsigned long)diagnostic->transform_length);
                    xil_printf("  Zero padding          : NO\r\n");
                    print_double_value("  Sample rate",                 diagnostic->sample_rate_hz / 1.0e6, " MSPS");
                    print_double_value("  Known tone frequency",                 diagnostic->expected_fundamental_hz / 1.0e6, " MHz");
                    print_double_value("  Expected tone bin",                 diagnostic->expected_fundamental_bin, "");
                    print_double_value("  Cycles in window",                 diagnostic->cycles_in_window, "");
                    xil_printf("  Coherent sampling     : %s\r\n",                 diagnostic->coherent_sampling ? "YES" : "NO");
                    xil_printf("  DFT window            : %s\r\n",                 diagnostic->window_name);
                    xil_printf("  Fundamental source    : %s\r\n",                 diagnostic->fundamental_known ?                     "KNOWN TONE" : "DETECTED FROM REFERENCE");
                    xil_printf("  Fundamental bin       : %lu\r\n",                 (unsigned long)diagnostic->fundamental_bin);
                    print_double_value("  Fundamental frequency",                 diagnostic->detected_fundamental_hz / 1.0e6, " MHz");
                    xil_printf("  Fundamental bins      : %lu ... %lu\r\n",                 (unsigned long)diagnostic->signal_bin_first,                 (unsigned long)diagnostic->signal_bin_last);
                    print_double_value("  DC power",                 diagnostic->dc_power, "");
                    print_double_value("  Signal power",                 diagnostic->signal_power, "");
                    print_double_value("  Total non-DC power",                 diagnostic->total_non_dc_power, "");
                    print_double_value("  Noise+distortion power",                 diagnostic->noise_distortion_power, "");
                    print_float_value_2("  SNDR",                 diagnostic->sndr_db, " dB");
                    print_float_value_2("  ENOB",                 diagnostic->enob, " bits");
                    xil_printf("\r\nFirst valid frame reference-path audit:\r\n");
                    print_float_value_or_invalid("  Raw reference mean",                 diagnostic->raw_reference_mean, " codes");
                    print_float_value_or_invalid("  Scaled reference mean",                 diagnostic->scaled_reference_mean, " codes");
                    print_float_value_or_invalid("  Raw ADC mean",                 diagnostic->raw_adc_mean, " codes");
                    print_float_value_or_invalid("  Offset-corrected ADC mean",                 diagnostic->offset_corrected_adc_mean, " codes");
                    print_float_value_or_invalid("  Gain-corrected ADC mean",                 diagnostic->gain_corrected_adc_mean, " codes");
                    print_signed_float_value_or_invalid("  Final residual mean",                 diagnostic->mean_residual, " codes");
                    print_float_value_2("  Reference-fit S/(error)",                 diagnostic->reference_fit_s_error_db, " dB");
                }
                if (!result->valid)             xil_printf("  Invalid reason        : %s\r\n",                        result->failure_reason != NULL ?                        result->failure_reason : "unknown");
            }
        }
        static void calibration_print_spectrum(     const char *name,     const int16_t *samples,     size_t sample_count,     double sample_rate_hz,     calibration_spectrum_t *spectrum ) {
            xil_printf("\r\n---------- %s Spectrum ----------\r\n", name);
            if (calibration_calculate_full_spectrum(             samples, sample_count, sample_rate_hz, spectrum) != 0) {
                xil_printf("Spectrum status         : FAIL\r\n");
                return;
            }
            xil_printf("Dominant bin            : %lu\r\n",                (unsigned long)spectrum->dominant_bin);
            print_double_value("Dominant frequency",                        spectrum->dominant_frequency_hz / 1.0e6, " MHz");
            xil_printf("Rank   Bin   Frequency MHz   Magnitude\r\n");
            for (size_t rank = 0U;
            rank < CAL_REF_SPECTRAL_PEAK_COUNT;
            ++rank) {
                const calibration_spectral_peak_t *peak = &spectrum->peaks[rank];
                xil_printf("%-6lu %-5lu ", (unsigned long)(rank + 1U),                    (unsigned long)peak->bin);
                print_double_inline((double)peak->bin * sample_rate_hz /                             (double)sample_count / 1.0e6);
                xil_printf("          ");
                print_double_inline(peak->magnitude);
                xil_printf("\r\n");
            }
        }
        static float calibration_best_reference_correlation(     const int16_t *candidate,     size_t candidate_count,     const int16_t *even_reference,     const int16_t *odd_reference,     size_t reference_stride ) {
            static int16_t reference_work[ADC_VALID_SAMPLE_COUNT];
            timing_alignment_result_t result;
            float best = -1.0f;
            if ((candidate == NULL) || (candidate_count == 0U) ||         (reference_stride == 0U)) return 0.0f;
            for (unsigned int phase = 0U;
            phase < 2U;
            ++phase) {
                const int16_t *source = phase == 0U ? even_reference : odd_reference;
                for (size_t i = 0U;
                i < candidate_count;
                ++i)             reference_work[i] = source[i * reference_stride];
                if (timing_find_circular_lag(reference_work, candidate,                 candidate_count, &result) == 0 && result.correlation > best)             best = result.correlation;
            }
            return best;
        }
        static void calibration_print_order_candidate(     const char *name,     const int16_t *candidate,     size_t candidate_count,     double sample_rate_hz,     const int16_t *even_reference,     const int16_t *odd_reference,     size_t reference_stride,     float *correlation_out,     size_t *dominant_bin_out ) {
            calibration_spectrum_t spectrum;
            const float correlation = calibration_best_reference_correlation(         candidate, candidate_count, even_reference, odd_reference,         reference_stride);
            const int spectral_status = calibration_calculate_full_spectrum(         candidate, candidate_count, sample_rate_hz, &spectrum);
            xil_printf("\r\nCandidate name          : %s\r\n", name);
            xil_printf("Sample count            : %lu\r\n", (unsigned long)candidate_count);
            print_double_value("Effective sample rate", sample_rate_hz / 1.0e6, " MHz");
            xil_printf("Dominant bin            : %lu\r\n",                (unsigned long)(spectral_status == 0 ? spectrum.dominant_bin : 0U));
            print_double_value("Dominant frequency", spectral_status == 0 ?         spectrum.dominant_frequency_hz / 1.0e6 : 0.0, " MHz");
            print_float_value("DAC-reference correlation", correlation, "");
            if (correlation_out != NULL) *correlation_out = correlation;
            if (dominant_bin_out != NULL) *dominant_bin_out =         spectral_status == 0 ? spectrum.dominant_bin : 0U;
        }
        static void calibration_decode_raw_mapping(     const uint8_t *raw, size_t beats, unsigned int mapping,     int16_t *channel_a, int16_t *channel_b) {
            static const uint8_t indices[4][8] = {
                {
                    0,1,2,3,4,5,6,7}
                    , {
                        4,5,6,7,0,1,2,3}
                        ,         {
                            3,2,1,0,7,6,5,4}
                            , {
                                7,6,5,4,3,2,1,0}
                            }
                            ;
                            for (size_t beat = 0U;
                            beat < beats;
                            ++beat) {
                                uint16_t words[8];
                                for (size_t word = 0U;
                                word < 8U;
                                ++word) {
                                    const size_t offset = (beat * 16U) + (word * 2U);
                                    words[word] = (uint16_t)raw[offset] |                           ((uint16_t)raw[offset + 1U] << 8U);
                                }
                                for (size_t i = 0U;
                                i < 4U;
                                ++i) {
                                    channel_a[beat * 4U + i] =                 ((int16_t)words[indices[mapping][i]]) >> 2;
                                    channel_b[beat * 4U + i] =                 ((int16_t)words[indices[mapping][i + 4U]]) >> 2;
                                }
                            }
                        }
                        static void calibration_print_raw_beats(const uint8_t *raw, const char *title) {
                            xil_printf("\r\n%s\r\n", title);
                            for (size_t beat = 0U;
                            beat < 16U;
                            ++beat) {
                                xil_printf("Beat %lu hex   :", (unsigned long)beat);
                                for (size_t word = 0U;
                                word < 8U;
                                ++word) {
                                    const size_t offset = beat * 16U + word * 2U;
                                    const uint16_t value = (uint16_t)raw[offset] |                                    ((uint16_t)raw[offset + 1U] << 8U);
                                    xil_printf(" %04X", value);
                                }
                                xil_printf("\r\nBeat %lu signed:", (unsigned long)beat);
                                for (size_t word = 0U;
                                word < 8U;
                                ++word) {
                                    const size_t offset = beat * 16U + word * 2U;
                                    const int16_t value = (int16_t)((uint16_t)raw[offset] |                 ((uint16_t)raw[offset + 1U] << 8U));
                                    xil_printf(" %d", (int)value);
                                }
                                xil_printf("\r\n");
                            }
                        }
                        static float calibration_ramp_order_score(const int16_t *samples, size_t count) {
                            size_t matches = 0U;
                            if (count < 2U) return 0.0f;
                            for (size_t i = 1U;
                            i < count;
                            ++i) {
                                const int16_t previous = samples[i - 1U];
                                const int16_t current = samples[i];
                                if ((current == previous + 1) ||             ((previous >= 8190) && (current <= -8191))) ++matches;
                            }
                            return (float)matches / (float)(count - 1U);
                        }
                        static void calibration_print_raw_channel_metrics(     const char *name, const int16_t *samples, size_t count,     const int16_t *even_reference, const int16_t *odd_reference) {
                            calibration_spectrum_t spectrum;
                            calibration_spectrum_t reference_spectrum;
                            double mean = 0.0, rms = 0.0;
                            int16_t minimum, maximum;
                            minimum = maximum = samples[0];
                            for (size_t i = 0U;
                            i < count;
                            ++i) {
                                mean += samples[i];
                                rms += (double)samples[i] * samples[i];
                                if (samples[i] < minimum) minimum = samples[i];
                                if (samples[i] > maximum) maximum = samples[i];
                            }
                            mean /= count;
                            rms = sqrt(rms / count);
                            (void)calibration_calculate_full_spectrum(         samples, count, adc_get_effective_sample_rate_hz(), &spectrum);
                            (void)calibration_calculate_full_spectrum(even_reference, count,         adc_get_effective_sample_rate_hz(), &reference_spectrum);
                            xil_printf("\r\n%s\r\n", name);
                            xil_printf("Sample count            : %lu\r\n", (unsigned long)count);
                            print_double_value("Mean", mean, " codes");
                            print_double_value("RMS", rms, " codes");
                            xil_printf("Minimum                 : %d\r\n", (int)minimum);
                            xil_printf("Maximum                 : %d\r\n", (int)maximum);
                            xil_printf("Dominant spectral bin   : %lu\r\n",                (unsigned long)spectrum.dominant_bin);
                            print_double_value("Dominant frequency",         spectrum.dominant_frequency_hz / 1.0e6, " MHz");
                            print_double_value("Reference tone frequency",         reference_spectrum.dominant_frequency_hz / 1.0e6, " MHz");
                            print_double_value("Magnitude at reference bin",         calibration_goertzel_magnitude(samples, count,             reference_spectrum.dominant_bin, mean), "");
                            print_double_value("Magnitude at dominant bin",         calibration_goertzel_magnitude(samples, count,             spectrum.dominant_bin, mean), "");
                            print_float_value("Uploaded-DAC correlation",         calibration_best_reference_correlation(samples, count,             even_reference, odd_reference, 1U), "");
                            xil_printf("First 32 samples        :\r\n");
                            for (size_t i = 0U;
                            i < 32U && i < count;
                            ++i)         xil_printf("%d%s", (int)samples[i],                    ((i + 1U) % 8U) == 0U ? "\r\n" : " ");
                        }
                        static calibration_timing_frame_result_t calibration_print_channel_alignment(     const char *label, const int16_t *reference, const int16_t *measurement,     size_t count) {
                            static int16_t work_reference[ADC_VALID_SAMPLE_COUNT];
                            static int16_t work_measurement[ADC_VALID_SAMPLE_COUNT];
                            calibration_timing_frame_result_t result;
                            memset(&result, 0, sizeof(result));
                            (void)adc_measure_timing_frame(reference, measurement, count,         work_reference, work_measurement, &result);
                            xil_printf("\r\n%s\r\n", label);
                            print_float_value("Correlation", result.correlation, "");
                            xil_printf("Lag                     : %ld samples\r\n",                (long)result.integer_lag);
                            print_float_value("Fractional lag", result.fractional_lag, " samples");
                            print_float_value("RMSE", result.fitted_rmse, " codes");
                            return result;
                        }
                        static int calibration_prepare_uploaded_dac_reference(     int16_t *even_reference,     int16_t *odd_reference,     size_t *reconstructed_count,     double *even_variance,     double *odd_variance,     int print_errors ) {
                            const int16_t *raw_reference;
                            size_t raw_count;
                            int status;
                            if (!reference_buffer_is_ready()) {
                                if (print_errors) xil_printf("ERROR: No valid DAC reference uploaded.\r\n");
                                return -1;
                            }
                            if (!isfinite(adc_get_effective_sample_rate_hz()) ||         adc_get_effective_sample_rate_hz() <= 0.0) {
                                if (print_errors) xil_printf("ERROR: Invalid effective ADC sample-rate value.\r\n");
                                return -7;
                            }
                            raw_reference = reference_buffer_data();
                            raw_count = reference_buffer_length();
                            if (raw_reference == NULL) {
                                if (print_errors) xil_printf("ERROR: Uploaded DAC reference pointer is NULL.\r\n");
                                return -2;
                            }
                            if (reference_buffer_format() != REFERENCE_FORMAT_DAC_RATE_2X) {
                                if (print_errors) xil_printf("ERROR: Uploaded reference is not tagged as raw DAC-rate data.\r\n");
                                return -3;
                            }
                            if (raw_count < (2U * ADC_CHANNEL_SAMPLE_COUNT)) {
                                if (print_errors) {
                                    xil_printf("ERROR: Uploaded DAC reference is too short.\r\n");
                                    xil_printf("Raw DAC samples       : %lu\r\n", (unsigned long)raw_count);
                                    xil_printf("Required DAC samples  : %lu\r\n",                        (unsigned long)(2U * ADC_CHANNEL_SAMPLE_COUNT));
                                }
                                return -4;
                            }
                            /* Analyze exactly one ADC DMA frame; extra uploaded samples stay intact. */
                            status = calibration_build_adc_reference_from_raw_dac(         raw_reference, raw_count,         even_reference, odd_reference, ADC_CHANNEL_SAMPLE_COUNT,         reconstructed_count);
                            if ((status != 0) || (*reconstructed_count != ADC_CHANNEL_SAMPLE_COUNT)) {
                                if (print_errors) xil_printf("ERROR: DAC reference reconstruction bounds check failed.\r\n");
                                return -5;
                            }
                            *even_variance = calibration_reference_variance(         even_reference, *reconstructed_count);
                            *odd_variance = calibration_reference_variance(         odd_reference, *reconstructed_count);
                            if (!isfinite(*even_variance) || !isfinite(*odd_variance) ||         (*even_variance <= CAL_REF_VARIANCE_EPSILON) ||         (*odd_variance <= CAL_REF_VARIANCE_EPSILON)) {
                                if (print_errors) xil_printf("ERROR: Reconstructed DAC reference has near-zero variance.\r\n");
                                return -6;
                            }
                            return 0;
                        }
                        static const calibration_timing_frame_result_t *calibration_select_phase(     const calibration_timing_frame_result_t *even_result,     const calibration_timing_frame_result_t *odd_result ) {
                            if (!even_result->alignment_success) return odd_result;
                            if (!odd_result->alignment_success) return even_result;
                            if (odd_result->correlation > even_result->correlation) return odd_result;
                            if ((fabsf(odd_result->correlation - even_result->correlation) <= 1.0e-6f) &&         (odd_result->fitted_rmse < even_result->fitted_rmse)) return odd_result;
                            return even_result;
                        }
                        /* Analyze one already-captured two-channel frame against both DAC phases. */
                        static int calibration_analyze_reference_frame(     const int16_t *even_reference,     const int16_t *odd_reference,     const int16_t *channel_a,     const int16_t *channel_b,     size_t sample_count,     int16_t *fractional_reference,     int16_t *fractional_measurement,     int locked_channel,     adc_reference_analysis_t *analysis) {
                            calibration_timing_frame_result_t results[2][2];
                            const int16_t *channels[2] = {
                                channel_a, channel_b }
                                ;
                                const int16_t *references[2] = {
                                    even_reference, odd_reference }
                                    ;
                                    const char *channel_names[2] = {
                                        "Channel A", "Channel B" }
                                        ;
                                        const char *phase_names[2] = {
                                            "EVEN", "ODD" }
                                            ;
                                            calibration_spectrum_t reference_spectrum;
                                            calibration_spectrum_t adc_spectrum;
                                            size_t best_channel = 0U;
                                            size_t best_phase = 0U;
                                            int have_alignment = 0;
                                            if (analysis == NULL) return -1;
                                            memset(analysis, 0, sizeof(*analysis));
                                            analysis->failure_reason = "invalid analysis input";
                                            if (sample_count == 0U) return -1;
                                            if (!isfinite(adc_get_effective_sample_rate_hz()) ||         adc_get_effective_sample_rate_hz() <= 0.0) {
                                                analysis->failure_reason = "invalid sample-rate value";
                                                return -1;
                                            }
                                            for (size_t channel = 0U;
                                            channel < 2U;
                                            ++channel) {
                                                if (locked_channel >= 0 && channel != (size_t)locked_channel)             continue;
                                                for (size_t phase = 0U;
                                                phase < 2U;
                                                ++phase) {
                                                    (void)adc_measure_timing_frame(references[phase], channels[channel],                 sample_count, fractional_reference, fractional_measurement,                 &results[channel][phase]);
                                                    if (results[channel][phase].alignment_success &&                 (!have_alignment ||                  results[channel][phase].correlation >                      results[best_channel][best_phase].correlation ||                  (fabsf(results[channel][phase].correlation -                         results[best_channel][best_phase].correlation) <= 1.0e-6f &&                   results[channel][phase].fitted_rmse <                       results[best_channel][best_phase].fitted_rmse))) {
                                                        best_channel = channel;
                                                        best_phase = phase;
                                                        have_alignment = 1;
                                                    }
                                                }
                                            }
                                            if (!have_alignment) {
                                                analysis->failure_reason = "no valid reference phase";
                                                return -2;
                                            }
                                            analysis->selected_reference = references[best_phase];
                                            analysis->selected_adc = channels[best_channel];
                                            analysis->selected_channel_name = channel_names[best_channel];
                                            analysis->selected_phase_name = phase_names[best_phase];
                                            analysis->sample_count = sample_count;
                                            analysis->timing = results[best_channel][best_phase];
                                            if (adc_analyze_fractional_overlap(analysis->selected_reference,             analysis->selected_adc, sample_count,             (double)analysis->timing.total_lag, fractional_reference,             fractional_measurement, &analysis->fit_state,             &analysis->fit_overlap) != 0)     {
                                                analysis->failure_reason = "insufficient overlap";
                                                return -3;
                                            }
                                            if (calibration_calculate_full_spectrum(analysis->selected_reference,             sample_count, adc_get_effective_sample_rate_hz(),             &reference_spectrum) != 0 ||         calibration_calculate_full_spectrum(analysis->selected_adc,             sample_count, adc_get_effective_sample_rate_hz(),             &adc_spectrum) != 0)     {
                                                analysis->failure_reason = "reference frequency measurement failed";
                                                return -4;
                                            }
                                            analysis->reference_frequency_hz = reference_spectrum.dominant_frequency_hz;
                                            analysis->adc_frequency_hz = adc_spectrum.dominant_frequency_hz;
                                            if (!isfinite(analysis->reference_frequency_hz) ||         !isfinite(analysis->adc_frequency_hz)) {
                                                analysis->failure_reason = "invalid numerical result";
                                                return -5;
                                            }
                                            if (analysis->fit_overlap.analysis_count < CAL_MIN_ANALYSIS_SAMPLES ||         analysis->timing.reject_reason == CAL_TIMING_REJECT_TOO_FEW_SAMPLES) {
                                                analysis->failure_reason = "insufficient overlap";
                                                return -5;
                                            }
                                            if (analysis->timing.correlation < CAL_DAC_REF_MIN_CORRELATION ||         analysis->timing.reject_reason == CAL_TIMING_REJECT_LOW_CORRELATION) {
                                                analysis->failure_reason = "aligned correlation below threshold";
                                                return -5;
                                            }
                                            if (!analysis->timing.accepted) {
                                                analysis->failure_reason =             cal_timing_reject_reason_text(analysis->timing.reject_reason);
                                                return -5;
                                            }
                                            if (fabs(analysis->reference_frequency_hz - analysis->adc_frequency_hz) >         fmax(CAL_REF_FREQ_TOLERANCE_HZ,              2.0 * adc_get_effective_sample_rate_hz() / (double)sample_count)) {
                                                analysis->failure_reason = "reference frequency mismatch";
                                                return -5;
                                            }
                                            analysis->failure_reason = "none";
                                            analysis->status = 0;
                                            return 0;
                                        }
                                        static void calibration_run_adc_reference_diagnostic(void) {
                                            static int16_t even_reference[ADC_VALID_SAMPLE_COUNT];
                                            static int16_t odd_reference[ADC_VALID_SAMPLE_COUNT];
                                            static int16_t adc_samples[ADC_VALID_SAMPLE_COUNT];
                                            static int16_t candidate[ADC_VALID_SAMPLE_COUNT];
                                            calibration_spectrum_t even_spectrum;
                                            calibration_spectrum_t odd_spectrum;
                                            calibration_spectrum_t adc_spectrum;
                                            calibration_timing_frame_result_t even_alignment;
                                            calibration_timing_frame_result_t odd_alignment;
                                            const calibration_timing_frame_result_t *selected_alignment;
                                            size_t reference_count = 0U;
                                            size_t adc_count = 0U;
                                            double even_variance, odd_variance;
                                            int16_t minimum, maximum;
                                            double mean = 0.0, rms_sum = 0.0;
                                            float best_candidate_correlation = -1.0f;
                                            const char *best_candidate_name = "none";
                                            int capture_status;
                                            int diagnostic_pass = 0;
                                            print_adc_sample_rate_state();
                                            if (calibration_prepare_uploaded_dac_reference(             even_reference, odd_reference, &reference_count,             &even_variance, &odd_variance, 1) != 0) return;
                                            if (adc_sweep_active) {
                                                ERR("Another automatic ADC capture is already in progress.");
                                                return;
                                            }
                                            adc_sweep_active = 1U;
                                            xil_printf("\r\nCapturing one 4095-byte DMA frame for diagnosis.\r\n");
                                            capture_status = adc_capture_frame();
                                            if (capture_status != XST_SUCCESS) {
                                                xil_printf("ERROR: Diagnostic DMA capture failed.\r\n");
                                                goto diagnostic_done;
                                            }
                                            if (adc_reconstruct_frame(RxBufferPtr, DMA_CMD_BUF_SIZE, adc_samples,             ADC_VALID_SAMPLE_COUNT, &adc_count) != 0 ||         adc_count != ADC_CHANNEL_SAMPLE_COUNT) {
                                                xil_printf("ERROR: Diagnostic ADC reconstruction failed.\r\n");
                                                goto diagnostic_done;
                                            }
                                            minimum = maximum = adc_samples[0];
                                            for (size_t i = 0U;
                                            i < adc_count;
                                            ++i) {
                                                const double value = adc_samples[i];
                                                if (adc_samples[i] < minimum) minimum = adc_samples[i];
                                                if (adc_samples[i] > maximum) maximum = adc_samples[i];
                                                mean += value;
                                                rms_sum += value * value;
                                            }
                                            mean /= (double)adc_count;
                                            rms_sum = sqrt(rms_sum / (double)adc_count);
                                            xil_printf("\r\n========== DMA-Reconstructed ADC Frame ==========\r\n");
                                            xil_printf("ADC sample count        : %lu\r\n", (unsigned long)adc_count);
                                            xil_printf("ADC minimum             : %d\r\n", (int)minimum);
                                            xil_printf("ADC maximum             : %d\r\n", (int)maximum);
                                            print_double_value("ADC mean", mean, " codes");
                                            print_double_value("ADC RMS", rms_sum, " codes");
                                            xil_printf("First 32 reconstructed ADC samples:\r\n");
                                            for (size_t i = 0U;
                                            i < CAL_REF_ADC_DEBUG_SAMPLE_COUNT;
                                            ++i) {
                                                xil_printf("%d%s", (int)adc_samples[i],                    ((i + 1U) % 8U) == 0U ? "\r\n" : " ");
                                            }
                                            xil_printf("\r\nReconstruction assumptions under test:\r\n");
                                            xil_printf("Raw words              : little-endian signed int16\r\n");
                                            xil_printf("ADC width/alignment    : signed left-aligned 14-bit, arithmetic >> 2\r\n");
                                            xil_printf("Eight-word ordering    : ADC0 S0..S3, ADC1 S0..S3\r\n");
                                            xil_printf("JESD configuration     : M=2, L=4, N=16, NP=16; no I/Q remap in this function\r\n");
                                            xil_printf("Lane/beat ordering     : assumed to be resolved by FPGA DMA producer\r\n");
                                            calibration_print_spectrum("DAC EVEN reference", even_reference,         reference_count, adc_get_effective_sample_rate_hz(), &even_spectrum);
                                            calibration_print_spectrum("DAC ODD reference", odd_reference,         reference_count, adc_get_effective_sample_rate_hz(), &odd_spectrum);
                                            calibration_print_spectrum("Real ADC capture", adc_samples,         adc_count, adc_get_effective_sample_rate_hz(), &adc_spectrum);
                                            /* Inspect the untouched DMA words and both independent converters. */
                                            calibration_run_raw_mapping_diagnostic(even_reference, odd_reference);
                                            xil_printf("\r\n========== Sample-Order Experiments ==========\r\n");
                                            #define RUN_CANDIDATE(candidate_name, data, count, rate, stride) \
                                                do { \
                                                    float candidate_correlation; \
                                                    size_t candidate_bin; \
                                                    calibration_print_order_candidate(candidate_name, data, count, rate, \
                                                        even_reference, odd_reference, stride, \
                                                        &candidate_correlation, &candidate_bin); \
                                                    if (candidate_correlation > best_candidate_correlation) { \
                                                        best_candidate_correlation = candidate_correlation; \
                                                        best_candidate_name = candidate_name; \
                                                    } \
                                                } while (0)
                                                    RUN_CANDIDATE("A. Current reconstructed sequence", adc_samples,                   adc_count, adc_get_effective_sample_rate_hz(), 1U);
                                                    for (size_t i = 0U;
                                                    i < adc_count;
                                                    ++i) {
                                                        const uint16_t value = (uint16_t)adc_samples[i];
                                                        candidate[i] = (int16_t)((value << 8U) | (value >> 8U));
                                                    }
                                                    RUN_CANDIDATE("B. Byte-swapped 16-bit samples", candidate,                   adc_count, adc_get_effective_sample_rate_hz(), 1U);
                                                    for (size_t i = 0U;
                                                    i < adc_count;
                                                    i += 2U) {
                                                        candidate[i] = adc_samples[i + 1U];
                                                        candidate[i + 1U] = adc_samples[i];
                                                    }
                                                    RUN_CANDIDATE("C. Adjacent sample pairs swapped", candidate,                   adc_count, adc_get_effective_sample_rate_hz(), 1U);
                                                    for (size_t phase = 0U;
                                                    phase < 2U;
                                                    ++phase) {
                                                        const size_t count = adc_count / 2U;
                                                        for (size_t i = 0U;
                                                        i < count;
                                                        ++i)             candidate[i] = adc_samples[(2U * i) + phase];
                                                        RUN_CANDIDATE(phase == 0U ? "D. Even-index samples only" :                       "E. Odd-index samples only", candidate, count,                       adc_get_effective_sample_rate_hz() / 2.0, 2U);
                                                    }
                                                    for (size_t phase = 0U;
                                                    phase < 4U;
                                                    ++phase) {
                                                        const char *names[4] = {
                                                            "F0. Every-fourth phase 0", "F1. Every-fourth phase 1",             "F2. Every-fourth phase 2", "F3. Every-fourth phase 3"}
                                                            ;
                                                            const size_t count = adc_count / 4U;
                                                            for (size_t i = 0U;
                                                            i < count;
                                                            ++i)             candidate[i] = adc_samples[(4U * i) + phase];
                                                            RUN_CANDIDATE(names[phase], candidate, count,                       adc_get_effective_sample_rate_hz() / 4.0, 4U);
                                                        }
                                                        #undef RUN_CANDIDATE
                                                        (void)adc_measure_timing_frame(even_reference, adc_samples, adc_count,         candidate, candidate + adc_count, &even_alignment);
                                                        (void)adc_measure_timing_frame(odd_reference, adc_samples, adc_count,         candidate, candidate + adc_count, &odd_alignment);
                                                        selected_alignment = calibration_select_phase(&even_alignment,         &odd_alignment);
                                                        diagnostic_pass = selected_alignment->accepted &&         selected_alignment->correlation >= CAL_DAC_REF_MIN_CORRELATION &&         selected_alignment->analysis_samples >= CAL_MIN_ANALYSIS_SAMPLES &&         fabs(adc_spectrum.dominant_frequency_hz -              (selected_alignment == &odd_alignment ?               odd_spectrum.dominant_frequency_hz :               even_spectrum.dominant_frequency_hz)) <=             fmax(CAL_REF_FREQ_TOLERANCE_HZ,                  2.0 * adc_get_effective_sample_rate_hz() / (double)adc_count);
                                                        xil_printf("\r\n========== ADC Reference Diagnostic ==========\r\n");
                                                        xil_printf("DAC EVEN dominant bin       : %lu\r\n",                (unsigned long)even_spectrum.dominant_bin);
                                                        xil_printf("DAC ODD dominant bin        : %lu\r\n",                (unsigned long)odd_spectrum.dominant_bin);
                                                        xil_printf("ADC dominant bin            : %lu\r\n",                (unsigned long)adc_spectrum.dominant_bin);
                                                        print_float_value("Aligned reference correlation",                       selected_alignment->correlation, "");
                                                        xil_printf("Aligned overlap samples      : %lu\r\n",                (unsigned long)selected_alignment->analysis_samples);
                                                        xil_printf("Best sample-order candidate : %s\r\n", best_candidate_name);
                                                        print_float_value("Best candidate correlation",                       best_candidate_correlation, "");
                                                        xil_printf("Diagnostic status           : %s\r\n",                diagnostic_pass ? "PASS" : "FAIL");
                                                        xil_printf("==============================================\r\n");
                                                        diagnostic_done:     xil_printf("No correction coefficients were updated.\r\n");
                                                        adc_sweep_active = 0U;
                                                    }
                                                    static void calibration_run_raw_mapping_diagnostic(     const int16_t *even_reference, const int16_t *odd_reference) {
                                                        static int16_t channel_a[ADC_VALID_SAMPLE_COUNT];
                                                        static int16_t channel_b[ADC_VALID_SAMPLE_COUNT];
                                                        static int16_t current_reconstruction[ADC_VALID_SAMPLE_COUNT];
                                                        const size_t beat_count = ADC_VALID_SAMPLE_COUNT / 8U;
                                                        const size_t channel_count = beat_count * 4U;
                                                        uint8_t saved_mode_a = AD9695_TESTMODE_OFF;
                                                        uint8_t saved_mode_b = AD9695_TESTMODE_OFF;
                                                        float best_ramp_score = 0.0f;
                                                        float best_dac_correlation = -1.0f;
                                                        size_t best_dominant_bin = 0U;
                                                        size_t selected_a_bin = 0U, selected_b_bin = 0U;
                                                        size_t normal_bins[4][2] = {
                                                            {
                                                                0U}
                                                            }
                                                            ;
                                                            calibration_spectrum_t reference_spectrum, legacy_spectrum;
                                                            calibration_spectrum_t channel_a_spectrum, channel_b_spectrum;
                                                            calibration_timing_frame_result_t ab_alignment;
                                                            float legacy_dac_correlation = 0.0f;
                                                            float channel_a_dac_correlation = 0.0f;
                                                            float channel_b_dac_correlation = 0.0f;
                                                            unsigned int best_mapping = 0U;
                                                            char best_channel = 'A';
                                                            int ramp_capture_ok = 0;
                                                            size_t current_count = 0U;
                                                            struct jesd_param_t jesd_readback;
                                                            memset(&jesd_readback, 0, sizeof(jesd_readback));
                                                            ad9695_jesd_get_cfg_param(&jesd_readback);
                                                            xil_printf("\r\nJESD SPI readback: M=%u L=%u F=%u S=%u N=%u NP=%u CS=%u HD=%u\r\n",         jesd_readback.jesd_M, jesd_readback.jesd_L,         jesd_readback.jesd_F, jesd_readback.jesd_S,         jesd_readback.jesd_N, jesd_readback.jesd_NP,         jesd_readback.jesd_CS, jesd_readback.jesd_HD);
                                                            calibration_print_raw_beats(RxBufferPtr,         "========== First 16 Normal Raw DMA Beats ==========");
                                                            calibration_decode_raw_mapping(RxBufferPtr, beat_count, 0U,         channel_a, channel_b);
                                                            (void)adc_reconstruct_frame(RxBufferPtr, DMA_CMD_BUF_SIZE,         current_reconstruction, ADC_VALID_SAMPLE_COUNT, &current_count);
                                                            xil_printf("\r\nFirst 32 reconstruction comparison:\r\n");
                                                            xil_printf("Index   Compatibility A    Channel A direct Channel B direct\r\n");
                                                            for (size_t i = 0U;
                                                            i < 32U;
                                                            ++i) {
                                                                xil_printf("%-7lu %-18d %-16d %d\r\n", (unsigned long)i,             i < current_count ? (int)current_reconstruction[i] : 0,             (int)channel_a[i], (int)channel_b[i]);
                                                            }
                                                            for (unsigned int mapping = 0U;
                                                            mapping < 1U;
                                                            ++mapping) {
                                                                char label_a[40], label_b[40];
                                                                calibration_decode_raw_mapping(RxBufferPtr, beat_count, mapping,             channel_a, channel_b);
                                                                snprintf(label_a, sizeof(label_a), "Mapping %u Channel A", mapping + 1U);
                                                                snprintf(label_b, sizeof(label_b), "Mapping %u Channel B", mapping + 1U);
                                                                calibration_print_raw_channel_metrics(label_a, channel_a,             channel_count, even_reference, odd_reference);
                                                                calibration_print_raw_channel_metrics(label_b, channel_b,             channel_count, even_reference, odd_reference);
                                                                {
                                                                    calibration_spectrum_t spectrum;
                                                                    float correlation = calibration_best_reference_correlation(                 channel_a, channel_count, even_reference, odd_reference, 1U);
                                                                    (void)calibration_calculate_full_spectrum(channel_a, channel_count,                 adc_get_effective_sample_rate_hz(), &spectrum);
                                                                    normal_bins[mapping][0] = spectrum.dominant_bin;
                                                                    if (correlation > best_dac_correlation) {
                                                                        best_dac_correlation = correlation;
                                                                        best_dominant_bin = spectrum.dominant_bin;
                                                                        best_mapping = mapping;
                                                                        best_channel = 'A';
                                                                    }
                                                                    correlation = calibration_best_reference_correlation(                 channel_b, channel_count, even_reference, odd_reference, 1U);
                                                                    (void)calibration_calculate_full_spectrum(channel_b, channel_count,                 adc_get_effective_sample_rate_hz(), &spectrum);
                                                                    normal_bins[mapping][1] = spectrum.dominant_bin;
                                                                    if (correlation > best_dac_correlation) {
                                                                        best_dac_correlation = correlation;
                                                                        best_dominant_bin = spectrum.dominant_bin;
                                                                        best_mapping = mapping;
                                                                        best_channel = 'B';
                                                                    }
                                                                }
                                                            }
                                                            /* Mapping 1 is proven chronological by the ramp: analyze only it. */
                                                            calibration_decode_raw_mapping(RxBufferPtr, beat_count, 0U,         channel_a, channel_b);
                                                            (void)calibration_calculate_full_spectrum(current_reconstruction,         current_count, adc_get_effective_sample_rate_hz(), &legacy_spectrum);
                                                            (void)calibration_calculate_full_spectrum(channel_a, channel_count,         adc_get_effective_sample_rate_hz(), &channel_a_spectrum);
                                                            (void)calibration_calculate_full_spectrum(channel_b, channel_count,         adc_get_effective_sample_rate_hz(), &channel_b_spectrum);
                                                            (void)calibration_calculate_full_spectrum(even_reference, channel_count,         adc_get_effective_sample_rate_hz(), &reference_spectrum);
                                                            legacy_dac_correlation = calibration_best_reference_correlation(         current_reconstruction, current_count, even_reference, odd_reference, 1U);
                                                            channel_a_dac_correlation = calibration_best_reference_correlation(         channel_a, channel_count, even_reference, odd_reference, 1U);
                                                            channel_b_dac_correlation = calibration_best_reference_correlation(         channel_b, channel_count, even_reference, odd_reference, 1U);
                                                            {
                                                                timing_alignment_result_t even_result, odd_result;
                                                                const int16_t *dac_a = even_reference;
                                                                const int16_t *dac_b = even_reference;
                                                                if (timing_find_circular_lag(even_reference, channel_a,                 channel_count, &even_result) == 0 &&             timing_find_circular_lag(odd_reference, channel_a,                 channel_count, &odd_result) == 0 &&             odd_result.correlation > even_result.correlation) dac_a = odd_reference;
                                                                if (timing_find_circular_lag(even_reference, channel_b,                 channel_count, &even_result) == 0 &&             timing_find_circular_lag(odd_reference, channel_b,                 channel_count, &odd_result) == 0 &&             odd_result.correlation > even_result.correlation) dac_b = odd_reference;
                                                                (void)calibration_print_channel_alignment(             "DAC reference vs Channel A", dac_a, channel_a, channel_count);
                                                                (void)calibration_print_channel_alignment(             "DAC reference vs Channel B", dac_b, channel_b, channel_count);
                                                            }
                                                            ab_alignment = calibration_print_channel_alignment(         "Channel A vs Channel B", channel_a, channel_b, channel_count);
                                                            xil_printf("\r\n========== AD9695 Ramp Transport Test ==========\r\n");
                                                            ad9695_adc_set_channel_select(0U);
                                                            ad9695_read_register(&spi_inst, AD9695_REG_TEST_MODE, &saved_mode_a);
                                                            ad9695_adc_set_channel_select(1U);
                                                            ad9695_read_register(&spi_inst, AD9695_REG_TEST_MODE, &saved_mode_b);
                                                            ad9695_adc_set_channel_select(0U);
                                                            ad9695_write_register(&spi_inst, AD9695_REG_TEST_MODE, AD9695_TESTMODE_RAMP);
                                                            ad9695_adc_set_channel_select(1U);
                                                            ad9695_write_register(&spi_inst, AD9695_REG_TEST_MODE, AD9695_TESTMODE_RAMP);
                                                            ad9695_adc_set_channel_select(2U);
                                                            usleep(1000U);
                                                            if (adc_capture_frame() == XST_SUCCESS) {
                                                                ramp_capture_ok = 1;
                                                                calibration_print_raw_beats(RxBufferPtr,             "First 16 ramp-mode raw DMA beats:");
                                                                for (unsigned int mapping = 0U;
                                                                mapping < 1U;
                                                                ++mapping) {
                                                                    float score_a, score_b;
                                                                    calibration_decode_raw_mapping(RxBufferPtr, beat_count, mapping,                 channel_a, channel_b);
                                                                    score_a = calibration_ramp_order_score(channel_a, channel_count);
                                                                    score_b = calibration_ramp_order_score(channel_b, channel_count);
                                                                    xil_printf("Mapping %u ramp score A/B: ", mapping + 1U);
                                                                    print_double_inline(score_a);
                                                                    xil_printf(" / ");
                                                                    print_double_inline(score_b);
                                                                    xil_printf("\r\n");
                                                                    if ((mapping == best_mapping) && (best_channel == 'A'))                 best_ramp_score = score_a;
                                                                    if ((mapping == best_mapping) && (best_channel == 'B'))                 best_ramp_score = score_b;
                                                                }
                                                            }
                                                            else {
                                                                xil_printf("ERROR: Ramp-mode DMA capture failed.\r\n");
                                                            }
                                                            /* Mandatory cleanup: restore both channel registers on every path. */
                                                            ad9695_adc_set_channel_select(0U);
                                                            ad9695_write_register(&spi_inst, AD9695_REG_TEST_MODE, saved_mode_a);
                                                            ad9695_adc_set_channel_select(1U);
                                                            ad9695_write_register(&spi_inst, AD9695_REG_TEST_MODE, saved_mode_b);
                                                            ad9695_adc_set_channel_select(2U);
                                                            selected_a_bin = normal_bins[best_mapping][0];
                                                            selected_b_bin = normal_bins[best_mapping][1];
                                                            xil_printf("\r\n========== Raw JESD/DMA Mapping Summary ==========\r\n");
                                                            xil_printf("ADC channel mapping      : Mapping %u, Channel %c\r\n",                best_mapping + 1U, best_channel);
                                                            xil_printf("Words per beat           : 8\r\n");
                                                            xil_printf("Samples/channel/beat     : 4 (M=2 assumption)\r\n");
                                                            xil_printf("Configured JESD          : M=2 L=4 F=1 S=derived/readback N=16 NP=16\r\n");
                                                            xil_printf("DMA bus width            : 128 bits (8 x 16-bit words)\r\n");
                                                            xil_printf("Bit alignment            : unresolved; compare raw and ramp hex\r\n");
                                                            xil_printf("Required right shift     : %s\r\n",                best_ramp_score > 0.90f ? "2 indicated by ramp" : "UNCONFIRMED");
                                                            xil_printf("Selected calibration channel: %c (diagnostic only)\r\n", best_channel);
                                                            xil_printf("Selected dominant bin    : %lu\r\n",                (unsigned long)best_dominant_bin);
                                                            xil_printf("Channel A dominant bin   : %lu\r\n", (unsigned long)selected_a_bin);
                                                            xil_printf("Channel B dominant bin   : %lu\r\n", (unsigned long)selected_b_bin);
                                                            print_float_value("Selected DAC correlation", best_dac_correlation, "");
                                                            print_float_value("Best ramp order score", best_ramp_score, "");
                                                            {
                                                                const double selected_frequency = (double)best_dominant_bin *             adc_get_effective_sample_rate_hz() / (double)channel_count;
                                                                const int hypothesis_pass = ramp_capture_ok &&             best_ramp_score > 0.90f && best_dac_correlation > 0.98f &&             fabs(selected_frequency - reference_spectrum.dominant_frequency_hz) <=                 CAL_REF_FREQ_TOLERANCE_HZ;
                                                                print_double_value("Selected dominant freq",                            selected_frequency / 1.0e6, " MHz");
                                                                if (hypothesis_pass)             xil_printf("Likely calibration channel: %c (diagnostic only)\r\n",                        best_channel);
                                                                else             xil_printf("Two-channel hypothesis not confirmed.\r\n");
                                                                xil_printf("Diagnostic status        : %s\r\n",                    hypothesis_pass ? "PASS" : "FAIL");
                                                            }
                                                            xil_printf("No calibration mapping or coefficients were changed.\r\n");
                                                            xil_printf("==================================================\r\n");
                                                            {
                                                                const double legacy_frequency = legacy_spectrum.dominant_frequency_hz;
                                                                const double channel_a_frequency = channel_a_spectrum.dominant_frequency_hz;
                                                                const double channel_b_frequency = channel_b_spectrum.dominant_frequency_hz;
                                                                const int both_tones =             fabs(channel_a_frequency - reference_spectrum.dominant_frequency_hz) <=                 CAL_REF_FREQ_TOLERANCE_HZ &&             fabs(channel_b_frequency - reference_spectrum.dominant_frequency_hz) <=                 CAL_REF_FREQ_TOLERANCE_HZ;
                                                                const char *recommended = "Neither";
                                                                if (both_tones && ab_alignment.correlation > 0.98f) {
                                                                    recommended = channel_a_dac_correlation >=                           channel_b_dac_correlation ? "A" : "B";
                                                                }
                                                                xil_printf("\r\n========================================\r\n");
                                                                xil_printf("Compatibility Channel A reconstruction:\r\n");
                                                                print_double_value("Dominant frequency", legacy_frequency / 1.0e6, " MHz");
                                                                print_float_value("Correlation to DAC", legacy_dac_correlation, "");
                                                                xil_printf("Channel A:\r\n");
                                                                print_double_value("Dominant frequency", channel_a_frequency / 1.0e6, " MHz");
                                                                print_float_value("Correlation to DAC", channel_a_dac_correlation, "");
                                                                xil_printf("Channel B:\r\n");
                                                                print_double_value("Dominant frequency", channel_b_frequency / 1.0e6, " MHz");
                                                                print_float_value("Correlation to DAC", channel_b_dac_correlation, "");
                                                                print_float_value("A-B correlation", ab_alignment.correlation, "");
                                                                xil_printf("Recommended calibration channel: %s\r\n", recommended);
                                                                if (both_tones && ab_alignment.correlation > 0.98f)             xil_printf("Conclusion: independent ADC data paths are consistent.\r\n");
                                                                xil_printf("========================================\r\n");
                                                            }
                                                        }
                                                        void handle_adc_reference_status_cmd(void) {
                                                            static int16_t even_reference[ADC_CHANNEL_SAMPLE_COUNT];
                                                            static int16_t odd_reference[ADC_CHANNEL_SAMPLE_COUNT];
                                                            static int16_t channel_a[ADC_CHANNEL_SAMPLE_COUNT];
                                                            static int16_t channel_b[ADC_CHANNEL_SAMPLE_COUNT];
                                                            static int16_t fractional_reference[ADC_CHANNEL_SAMPLE_COUNT];
                                                            static int16_t fractional_measurement[ADC_CHANNEL_SAMPLE_COUNT];
                                                            adc_reference_analysis_t analysis;
                                                            size_t reference_count = 0U;
                                                            size_t adc_count = 0U;
                                                            double even_variance = 0.0;
                                                            double odd_variance = 0.0;
                                                            int status = -1;
                                                            if (!reference_buffer_is_ready() || reference_buffer_length() == 0U) {
                                                                xil_printf("ADC reference analysis stopped: uploaded DAC reference is not available.\r\n");
                                                                return;
                                                            }
                                                            if (adc_sweep_active) {
                                                                ERR("Another automatic ADC capture is already in progress.");
                                                                return;
                                                            }
                                                            print_adc_analysis_rate_header();
                                                            if (calibration_prepare_uploaded_dac_reference(even_reference,             odd_reference, &reference_count, &even_variance,             &odd_variance, 1) != 0)         return;
                                                            adc_sweep_active = 1U;
                                                            if (adc_capture_frame() != XST_SUCCESS) {
                                                                xil_printf("ADC reference analysis stopped: DMA capture failed.\r\n");
                                                                goto reference_done;
                                                            }
                                                            if (adc_reconstruct_channels(RxBufferPtr, DMA_CMD_BUF_SIZE,             channel_a, ADC_CHANNEL_SAMPLE_COUNT, channel_b,             ADC_CHANNEL_SAMPLE_COUNT, &adc_count) != 0 ||         adc_count != reference_count) {
                                                                xil_printf("ADC reference analysis stopped: sample reconstruction failed.\r\n");
                                                                goto reference_done;
                                                            }
                                                            status = calibration_analyze_reference_frame(even_reference,         odd_reference, channel_a, channel_b, adc_count,         fractional_reference, fractional_measurement, -1, &analysis);
                                                            xil_printf("\r\n========== ADC Reference Analysis ==========\r\n");
                                                            xil_printf("ADC samples             : %lu\r\n", (unsigned long)adc_count);
                                                            print_double_value("Configured sample rate",         adc_get_configured_sample_rate_hz() / 1.0e6, " MSPS");
                                                            print_double_value("Analysis sample rate",         adc_get_effective_sample_rate_hz() / 1.0e6, " MSPS");
                                                            if (status == 0) {
                                                                print_double_value("Reference dominant freq",             analysis.reference_frequency_hz / 1.0e6, " MHz");
                                                                print_double_value("ADC dominant freq",             analysis.adc_frequency_hz / 1.0e6, " MHz");
                                                                xil_printf("Selected ADC channel    : %s\r\n",             analysis.selected_channel_name);
                                                                xil_printf("Selected reference phase: %s\r\n",             analysis.selected_phase_name);
                                                                xil_printf("Integer lag             : %ld samples\r\n",             (long)analysis.timing.integer_lag);
                                                                print_float_value("Fractional lag", analysis.timing.fractional_lag,             " samples");
                                                                print_float_value("Alignment correlation",             analysis.timing.correlation, "");
                                                                print_float_value("Aligned RMSE",             analysis.fit_state.metrics.fitted_rmse_codes, " codes");
                                                                print_reference_coherence(analysis.reference_frequency_hz, adc_count);
                                                            }
                                                            else {
                                                                xil_printf("Failure reason          : %s\r\n",             analysis.failure_reason != NULL ? analysis.failure_reason :             "unknown analysis error");
                                                            }
                                                            xil_printf("Reference status        : %s\r\n",         status == 0 ? "PASS" : "FAIL");
                                                            xil_printf("============================================\r\n");
                                                            xil_printf("No correction coefficients were updated.\r\n");
                                                            reference_done:     adc_sweep_active = 0U;
                                                        }
                                                        static const char *calibration_channel_name(int channel) {
                                                            switch (channel) {
                                                                case 0:         return "Channel A";
                                                                case 1:         return "Channel B";
                                                                case -1:         return "auto";
                                                                default:         return "invalid";
                                                            }
                                                        }
                                                        static const char *calibration_automatic_stage_name(     adc_calibration_stage_t stage) {
                                                            switch (stage) {
                                                                case ADC_CAL_STAGE_IDLE: return "IDLE";
                                                                case ADC_CAL_STAGE_TIMING: return "TIMING";
                                                                case ADC_CAL_STAGE_OFFSET: return "OFFSET";
                                                                case ADC_CAL_STAGE_GAIN: return "GAIN";
                                                                case ADC_CAL_STAGE_VERIFY: return "VERIFY";
                                                                case ADC_CAL_STAGE_PERFORMANCE: return "PERFORMANCE";
                                                                case ADC_CAL_STAGE_COMPLETE: return "COMPLETE";
                                                                case ADC_CAL_STAGE_FAILED: return "FAILED";
                                                                default: return "UNKNOWN";
                                                            }
                                                        }
                                                        static const char *calibration_offset_result_name(     calibration_offset_result_t result) {
                                                            switch (result) {
                                                                case CALIBRATION_OFFSET_RESULT_CONVERGED: return "CONVERGED";
                                                                case CALIBRATION_OFFSET_RESULT_PROVISIONAL: return "PROVISIONAL";
                                                                case CALIBRATION_OFFSET_RESULT_FAILED: return "FAILED";
                                                                default: return "NOT RUN";
                                                            }
                                                        }
                                                        static const char *calibration_existing_offset_loop_status_name(     const calibration_offset_loop_state_t *state) {
                                                            if (state == NULL) return "FAIL";
                                                            switch (state->final_status) {
                                                            case CALIBRATION_OFFSET_LOOP_PASS:
                                                                return "PASS";
                                                            case CALIBRATION_OFFSET_LOOP_RUNNING:
                                                            case CALIBRATION_OFFSET_LOOP_CONTROLLER_CONVERGED:
                                                            case CALIBRATION_OFFSET_LOOP_VERIFYING:
                                                                return "RUNNING";
                                                            case CALIBRATION_OFFSET_LOOP_BEST_AVAILABLE:
                                                            case CALIBRATION_OFFSET_LOOP_NOT_CONVERGED:
                                                            case CALIBRATION_OFFSET_LOOP_IDLE:
                                                                return "NOT CONVERGED";
                                                            case CALIBRATION_OFFSET_LOOP_FAILED:
                                                            default:
                                                                return "FAIL";
                                                            }
                                                        }
                                                        static const char *calibration_automatic_result_name(     adc_calibration_result_t result) {
                                                            switch (result) {
                                                                case ADC_CAL_RESULT_PASS: return "PASS";
                                                                case ADC_CAL_RESULT_PROVISIONAL: return "PROVISIONAL";
                                                                case ADC_CAL_RESULT_FAILED: return "FAILED";
                                                                default: return "NOT COMPLETE";
                                                            }
                                                        }
                                                        static void calibration_print_offset_batch_compact(     uint32_t iteration,     const calibration_offset_batch_result_t *batch,     const calibration_offset_loop_state_t *state,     bool pass,     const char *status) {
                                                            xil_printf("Batch %2lu | %lu/%u | residual ",                (unsigned long)iteration,                (unsigned long)batch->accepted,                CALIBRATION_OFFSET_BATCH_SIZE);
                                                            print_signed_float_inline(batch->mean);
                                                            xil_printf(" | filtered ");
                                                            print_signed_float_inline(state->filtered_residual);
                                                            xil_printf(" | correction ");
                                                            print_signed_float_inline(state->offset_correction);
                                                            if (batch->dither_valid_estimates > 0U) {
                                                                xil_printf(" | dither ");
                                                                print_signed_float_inline((float)batch->mean_dither_offset);
                                                                xil_printf(" | delta ");
                                                                print_signed_float_inline((float)batch->mean_existing_dither_delta);
                                                                xil_printf(" | events %lu | new %s",         (unsigned long)batch->dither_latest.complete_event_count,         calibration_dither_offset_status_name(             batch->dither_latest.status));
                                                            }
                                                            else if (batch->dither_invalid > 0U) {
                                                                xil_printf(" | dither INVALID | events %lu | reason %s",         (unsigned long)batch->dither_latest.complete_event_count,         calibration_dither_offset_reason_name(             batch->dither_latest.reason));
                                                            }
                                                            if (pass)         xil_printf(" | PASS %lu/%u\r\n",                    (unsigned long)state->convergence_count,                    CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES);
                                                            else         xil_printf(" | %s\r\n", status != NULL ? status : "RUNNING");
                                                        }
                                                        static void calibration_print_gain_batch_compact(     uint32_t iteration,     const calibration_gain_batch_result_t *batch,     const calibration_gain_loop_state_t *state,     bool pass,     const char *status) {
                                                            xil_printf("Batch %2lu | %lu/%u | gain ",                (unsigned long)iteration,                (unsigned long)batch->accepted,                CALIBRATION_GAIN_BATCH_SIZE);
                                                            print_double_inline((double)batch->mean_gain);
                                                            if (batch->dither_valid_estimates > 0U) {
                                                                xil_printf(" | dither ");
                                                                print_double_inline(batch->mean_dither_gain);
                                                                xil_printf(" | flat ");
                                                                print_double_inline(batch->mean_dither_flat_gain);
                                                                xil_printf(" | events %lu | new %s",         (unsigned long)batch->dither_latest.complete_event_count,         calibration_dither_gain_status_name(             batch->dither_latest.status));
                                                            }
                                                            else if (batch->dither_invalid > 0U) {
                                                                xil_printf(" | dither INVALID | events %lu | reason %s",         (unsigned long)batch->dither_latest.complete_event_count,         calibration_dither_gain_reason_name(             batch->dither_latest.reason));
                                                            }
                                                            xil_printf(" | error ");
                                                            print_signed_float_inline(batch->mean_error);
                                                            xil_printf(" | correction ");
                                                            print_double_inline((double)state->gain_correction);
                                                            if (pass)         xil_printf(" | PASS %lu/%u\r\n",                    (unsigned long)state->convergence_count,                    CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES);
                                                            else         xil_printf(" | %s\r\n", status != NULL ? status : "RUNNING");
                                                        }
                                                        static const char *calibration_timing_stage_status(     const adc_automatic_calibration_state_t *state) {
                                                            if (state->timing_pass) return "PASS";
                                                            if (state->active && state->stage == ADC_CAL_STAGE_TIMING)         return "RUNNING";
                                                            if (state->failed_stage == ADC_CAL_STAGE_TIMING) return "FAILED";
                                                            return "NOT RUN";
                                                        }
                                                        static const char *calibration_gain_stage_status(     const adc_automatic_calibration_state_t *state) {
                                                            if (state->gain_pass && state->gain_verification_pass)         return "CONVERGED";
                                                            if (state->active && (state->stage == ADC_CAL_STAGE_GAIN ||                           state->stage == ADC_CAL_STAGE_VERIFY))         return "RUNNING";
                                                            if (state->failed_stage == ADC_CAL_STAGE_GAIN) return "FAILED";
                                                            return "NOT RUN";
                                                        }
                                                        static const char *calibration_performance_status(     const adc_automatic_calibration_state_t *state) {
                                                            if (state->performance.valid) return "VALID";
                                                            if (state->active && state->stage == ADC_CAL_STAGE_PERFORMANCE)         return "RUNNING";
                                                            if (state->stage == ADC_CAL_STAGE_COMPLETE) return "INVALID";
                                                            return "NOT RUN";
                                                        }
                                                        static void calibration_automatic_print_summary(void) {
                                                            const adc_automatic_calibration_state_t *state =         &g_automatic_calibration;
                                                            xil_printf("\r\n=========================================\r\n");
                                                            if (state->stage == ADC_CAL_STAGE_COMPLETE)         xil_printf("       ADC CALIBRATION COMPLETE\r\n");
                                                            else if (state->stage == ADC_CAL_STAGE_FAILED)         xil_printf("       ADC CALIBRATION FAILED\r\n");
                                                            else         xil_printf("       ADC CALIBRATION STATUS\r\n");
                                                            xil_printf("=========================================\r\n");
                                                            if (state->stage == ADC_CAL_STAGE_FAILED) {
                                                                xil_printf("Failed stage            : %s\r\n",                    calibration_automatic_stage_name(state->failed_stage));
                                                                if (state->failure_reason != NULL)             xil_printf("Reason                  : %s\r\n",                        state->failure_reason);
                                                                xil_printf("\r\n");
                                                            }
                                                            if (state->timing_pass) {
                                                                xil_printf("Channel                 : %s\r\n",             calibration_channel_name(state->calibration_channel));
                                                                print_float_value("Timing correlation",                           state->timing_mean_correlation, "");
                                                                xil_printf("Canonical phase         : %s\r\n",                    state->canonical_reference_phase == 0 ? "EVEN" : "ODD");
                                                                xil_printf("Analysis samples        : %lu\r\n",                    (unsigned long)(state->output_valid ?                        state->final_output.analysis_sample_count :                        state->fixed_window_length));
                                                            }
                                                            if (state->offset_result != CALIBRATION_OFFSET_RESULT_NONE) {
                                                                xil_printf("\r\n");
                                                                print_float_value("Offset correction",                           state->offset_correction, " codes");
                                                                print_float_value("Offset verification",                           state->offset_verification_error, " codes");
                                                            }
                                                            if (state->gain_pass || state->failed_stage == ADC_CAL_STAGE_GAIN) {
                                                                xil_printf("\r\n");
                                                                print_float_value("Gain correction", state->gain_correction, "");
                                                                print_float_value("Normalized final gain",                           state->final_normalized_gain, "");
                                                                print_float_value("Gain verification error",                           state->gain_verification_error, "");
                                                            }
                                                            if (state->performance.reference_metrics_valid ||         state->performance.spectral_metrics_valid ||         state->stage == ADC_CAL_STAGE_COMPLETE) {
                                                                xil_printf("\r\n");
                                                                xil_printf("Performance frames      : %lu\r\n",             (unsigned long)state->performance.frames_valid);
                                                                print_signed_float_value_or_invalid("Mean residual offset",             state->performance.reference_metrics_valid ?             state->performance.mean_residual : NAN, " codes");
                                                                print_float_value_or_invalid("Mean RMSE",             state->performance.reference_metrics_valid ?             state->performance.rmse : NAN, " codes");
                                                                print_float_value_or_invalid("Mean correlation",             state->performance.reference_metrics_valid ?             state->performance.correlation : NAN, "");
                                                                print_float_value_2("Mean SNDR",             state->performance.spectral_metrics_valid ?             state->performance.sndr_db : NAN, " dB");
                                                                print_float_value_2("Mean ENOB",             state->performance.spectral_metrics_valid ?             state->performance.enob : NAN, " bits");
                                                            }
                                                            xil_printf("\r\nTiming                  : %s\r\n",                calibration_timing_stage_status(state));
                                                            xil_printf("Offset                  : %s\r\n",                calibration_offset_result_name(state->offset_result));
                                                            xil_printf("Gain                    : %s\r\n",                calibration_gain_stage_status(state));
                                                            xil_printf("Performance             : %s\r\n",                calibration_performance_status(state));
                                                            xil_printf("Overall calibration     : %s\r\n",                state->active ? "RUNNING" :                calibration_automatic_result_name(state->overall_result));
                                                            xil_printf("Output usable           : %s\r\n",                state->valid && state->output_valid ? "YES" : "NO");
                                                            xil_printf("=========================================\r\n");
                                                        }
                                                        static void calibration_offset_loop_begin_run(     calibration_offset_loop_state_t *state ) {
                                                            memset(state, 0, sizeof(*state));
                                                            state->offset_correction = calibration_software_offset_correction();
                                                            state->gain_correction = calibration_software_gain_correction();
                                                            state->calibration_channel = calibration_channel_selection();
                                                            state->final_status = CALIBRATION_OFFSET_LOOP_RUNNING;
                                                            state->latest_correlation = 0.0f;
                                                            state->latest_mean_residual = 0.0f;
                                                            state->latest_fitted_offset = 0.0f;
                                                            state->latest_fitted_gain = 0.0f;
                                                            state->latest_rmse = 0.0f;
                                                            state->latest_raw_mean = 0.0f;
                                                            state->latest_corrected_mean = 0.0f;
                                                            state->best_abs_residual = FLT_MAX;
                                                            state->best_score = FLT_MAX;
                                                        }
                                                        static int calibration_samples_are_clipped(     const int16_t *samples,     size_t sample_count ) {
                                                            if ((samples == NULL) || (sample_count == 0U)) {
                                                                return 1;
                                                            }
                                                            for (size_t i = 0U;
                                                            i < sample_count;
                                                            ++i) {
                                                                if ((samples[i] <= CALIBRATION_ADC_MIN_CODE) ||             (samples[i] >= CALIBRATION_ADC_MAX_CODE)) {
                                                                    return 1;
                                                                }
                                                            }
                                                            return 0;
                                                        }
                                                        static int calibration_fit_metrics_are_valid(     const calibration_state_t *fit_state ) {
                                                            const calibration_metrics_t *metrics;
                                                            if (fit_state == NULL) {
                                                                return 0;
                                                            }
                                                            metrics = &fit_state->metrics;
                                                            return isfinite(metrics->adc_mean) &&            isfinite(metrics->reference_mean) &&            isfinite(metrics->measured_gain) &&            isfinite(metrics->measured_offset) &&            isfinite(metrics->fitted_rmse_codes) &&            isfinite(metrics->fitted_mae_codes) &&            isfinite(metrics->rmse_codes) &&            isfinite(metrics->mae_codes) &&            isfinite(metrics->correlation) &&            isfinite(metrics->offset_error_codes) &&            isfinite(metrics->gain_error_ratio) &&            isfinite(metrics->adc_rms_ac) &&            isfinite(metrics->reference_rms_ac) &&            (fabsf(metrics->measured_gain) > FLT_EPSILON) &&            (metrics->fitted_rmse_codes >= 0.0f) &&            (metrics->rmse_codes >= 0.0f);
                                                        }
                                                        /* Map a capture onto an immutable reference-coordinate window.  Lag follows  * aligned[i] = measurement[i + lag]; interpolation is performed before the  * fixed reference indices are extracted. */
                                                        static int calibration_map_fixed_window(     const int16_t *reference,     const int16_t *measurement,     size_t full_count,     double total_lag,     size_t window_start,     size_t window_length,     int16_t *mapped_reference,     int16_t *mapped_measurement,     calibration_state_t *fit_state) {
                                                            calibration_config_t fit_config;
                                                            if (reference == NULL || measurement == NULL ||         mapped_reference == NULL || mapped_measurement == NULL ||         fit_state == NULL || !isfinite(total_lag) ||         window_length < CAL_MIN_ANALYSIS_SAMPLES ||         window_start + window_length > full_count)         return -1;
                                                            for (size_t j = 0U;
                                                            j < window_length;
                                                            ++j) {
                                                                const size_t reference_index = window_start + j;
                                                                const double source_position =             (double)reference_index + total_lag;
                                                                size_t lower;
                                                                double fraction;
                                                                double interpolated;
                                                                long rounded;
                                                                if (source_position < 0.0 ||             source_position >= (double)(full_count - 1U))             return -2;
                                                                lower = (size_t)floor(source_position);
                                                                fraction = source_position - (double)lower;
                                                                interpolated =             (1.0 - fraction) * (double)measurement[lower] +             fraction * (double)measurement[lower + 1U];
                                                                if (!isfinite(interpolated)) return -3;
                                                                rounded = lround(interpolated);
                                                                if (rounded < INT16_MIN || rounded > INT16_MAX) return -3;
                                                                mapped_reference[j] = reference[reference_index];
                                                                mapped_measurement[j] = (int16_t)rounded;
                                                            }
                                                            calibration_default_config(&fit_config);
                                                            if (calibration_init(fit_state, &fit_config) != CALIBRATION_OK ||         calibration_analyze_frame(fit_state, mapped_measurement,             mapped_reference, window_length) != CALIBRATION_OK ||         !calibration_fit_metrics_are_valid(fit_state))         return -4;
                                                            return 0;
                                                        }
                                                        static uint32_t calibration_reference_checksum(     const int16_t *samples, size_t count) {
                                                            uint32_t hash = 2166136261U;
                                                            for (size_t i = 0U;
                                                            i < count;
                                                            ++i) {
                                                                const uint16_t value = (uint16_t)samples[i];
                                                                hash ^= (uint8_t)(value & 0xFFU);
                                                                hash *= 16777619U;
                                                                hash ^= (uint8_t)(value >> 8);
                                                                hash *= 16777619U;
                                                            }
                                                            return hash;
                                                        }
                                                        static float calibration_reference_mean_value(     const int16_t *samples, size_t count) {
                                                            double sum = 0.0;
                                                            for (size_t i = 0U;
                                                            i < count;
                                                            ++i) sum += samples[i];
                                                            return count > 0U ? (float)(sum / (double)count) : 0.0f;
                                                        }
                                                        static int calibration_fixed_window_correlation(     const int16_t *canonical_reference,     const int16_t *measurement,     size_t measurement_count,     size_t window_start,     size_t window_length,     double lag,     float *correlation) {
                                                            double reference_sum = 0.0, measurement_sum = 0.0;
                                                            double numerator = 0.0, reference_power = 0.0, measurement_power = 0.0;
                                                            if (canonical_reference == NULL || measurement == NULL ||         correlation == NULL || !isfinite(lag) || window_length == 0U)         return -1;
                                                            for (size_t j = 0U;
                                                            j < window_length;
                                                            ++j) {
                                                                const size_t k = window_start + j;
                                                                const double position = (double)k + lag;
                                                                size_t lower;
                                                                double fraction;
                                                                if (position < 0.0 || position >= (double)(measurement_count - 1U))             return -2;
                                                                lower = (size_t)floor(position);
                                                                fraction = position - (double)lower;
                                                                reference_sum += canonical_reference[k];
                                                                measurement_sum +=             (1.0 - fraction) * measurement[lower] +             fraction * measurement[lower + 1U];
                                                            }
                                                            reference_sum /= (double)window_length;
                                                            measurement_sum /= (double)window_length;
                                                            for (size_t j = 0U;
                                                            j < window_length;
                                                            ++j) {
                                                                const size_t k = window_start + j;
                                                                const double position = (double)k + lag;
                                                                const size_t lower = (size_t)floor(position);
                                                                const double fraction = position - (double)lower;
                                                                const double x = canonical_reference[k] - reference_sum;
                                                                const double sample =             (1.0 - fraction) * measurement[lower] +             fraction * measurement[lower + 1U];
                                                                const double y = sample - measurement_sum;
                                                                numerator += x * y;
                                                                reference_power += x * x;
                                                                measurement_power += y * y;
                                                            }
                                                            if (reference_power <= CAL_REF_VARIANCE_EPSILON ||         measurement_power <= CAL_REF_VARIANCE_EPSILON)         return -3;
                                                            *correlation = (float)(numerator /         sqrt(reference_power * measurement_power));
                                                            return isfinite(*correlation) ? 0 : -4;
                                                        }
                                                        static double calibration_wrap_lag(double lag, size_t sample_count) {
                                                            const double count = (double)sample_count;
                                                            double wrapped;
                                                            if (!isfinite(lag) || sample_count == 0U) return NAN;
                                                            wrapped = fmod(lag, count);
                                                            if (wrapped > 0.5 * count) wrapped -= count;
                                                            if (wrapped < -0.5 * count) wrapped += count;
                                                            return wrapped;
                                                        }
                                                        static double calibration_circular_sample_double(     const double *samples, size_t sample_count, double position) {
                                                            double wrapped;
                                                            size_t lower;
                                                            size_t upper;
                                                            double fraction;
                                                            if (samples == NULL || sample_count == 0U || !isfinite(position))         return NAN;
                                                            wrapped = fmod(position, (double)sample_count);
                                                            if (wrapped < 0.0) wrapped += (double)sample_count;
                                                            lower = (size_t)floor(wrapped);
                                                            if (lower >= sample_count) lower = 0U;
                                                            upper = lower + 1U;
                                                            if (upper >= sample_count) upper = 0U;
                                                            fraction = wrapped - (double)lower;
                                                            return (1.0 - fraction) * samples[lower] + fraction * samples[upper];
                                                        }
                                                        static int calibration_solve_3x3(double matrix[3][4], double solution[3]) {
                                                            for (size_t pivot = 0U;
                                                            pivot < 3U;
                                                            ++pivot) {
                                                                size_t best = pivot;
                                                                double best_abs = fabs(matrix[pivot][pivot]);
                                                                for (size_t row = pivot + 1U;
                                                                row < 3U;
                                                                ++row) {
                                                                    const double value = fabs(matrix[row][pivot]);
                                                                    if (value > best_abs) {
                                                                        best = row;
                                                                        best_abs = value;
                                                                    }
                                                                }
                                                                if (best_abs <= 1.0e-18 || !isfinite(best_abs)) return -1;
                                                                if (best != pivot) {
                                                                    for (size_t col = pivot;
                                                                    col < 4U;
                                                                    ++col) {
                                                                        const double tmp = matrix[pivot][col];
                                                                        matrix[pivot][col] = matrix[best][col];
                                                                        matrix[best][col] = tmp;
                                                                    }
                                                                }
                                                                for (size_t row = pivot + 1U;
                                                                row < 3U;
                                                                ++row) {
                                                                    const double scale = matrix[row][pivot] / matrix[pivot][pivot];
                                                                    for (size_t col = pivot;
                                                                    col < 4U;
                                                                    ++col)                 matrix[row][col] -= scale * matrix[pivot][col];
                                                                }
                                                            }
                                                            for (int row = 2;
                                                            row >= 0;
                                                            --row) {
                                                                double sum = matrix[row][3];
                                                                for (size_t col = (size_t)row + 1U;
                                                                col < 3U;
                                                                ++col)             sum -= matrix[row][col] * solution[col];
                                                                if (fabs(matrix[row][row]) <= 1.0e-18) return -2;
                                                                solution[row] = sum / matrix[row][row];
                                                                if (!isfinite(solution[row])) return -3;
                                                            }
                                                            return 0;
                                                        }
                                                        static double calibration_normalized_correlation_double(     const double *a, const double *b, size_t count) {
                                                            double mean_a = 0.0;
                                                            double mean_b = 0.0;
                                                            double numerator = 0.0;
                                                            double power_a = 0.0;
                                                            double power_b = 0.0;
                                                            if (a == NULL || b == NULL || count == 0U) return NAN;
                                                            for (size_t i = 0U;
                                                            i < count;
                                                            ++i) {
                                                                mean_a += a[i];
                                                                mean_b += b[i];
                                                            }
                                                            mean_a /= (double)count;
                                                            mean_b /= (double)count;
                                                            for (size_t i = 0U;
                                                            i < count;
                                                            ++i) {
                                                                const double x = a[i] - mean_a;
                                                                const double y = b[i] - mean_b;
                                                                numerator += x * y;
                                                                power_a += x * x;
                                                                power_b += y * y;
                                                            }
                                                            if (power_a <= DBL_EPSILON || power_b <= DBL_EPSILON) return NAN;
                                                            return numerator / sqrt(power_a * power_b);
                                                        }
                                                        static int calibration_fit_tone_at_frequency(     const double *samples,     size_t sample_count,     double frequency_cycles_per_sample,     double sample_rate_hz,     double expected_frequency_hz,     double initial_frequency_hz,     calibration_tone_fit_result_t *fit,     double *fitted_waveform,     double *residual) {
                                                            double cc = 0.0, ss = 0.0, cs = 0.0, c1 = 0.0, s1 = 0.0;
                                                            double yc = 0.0, ys = 0.0, y1 = 0.0;
                                                            double matrix[3][4];
                                                            double solution[3] = {
                                                                0.0, 0.0, 0.0}
                                                                ;
                                                                double sse = 0.0;
                                                                double fitted_power = 0.0;
                                                                double error_power = 0.0;
                                                                if (samples == NULL || fit == NULL || sample_count < 4U ||         !isfinite(frequency_cycles_per_sample) ||         frequency_cycles_per_sample <= 0.0 ||         frequency_cycles_per_sample >= 0.5 ||         !isfinite(sample_rate_hz) || sample_rate_hz <= 0.0)         return -1;
                                                                for (size_t i = 0U;
                                                                i < sample_count;
                                                                ++i) {
                                                                    const double angle = 6.28318530717958647692 *             frequency_cycles_per_sample * (double)i;
                                                                    const double c = cos(angle);
                                                                    const double s = sin(angle);
                                                                    const double y = samples[i];
                                                                    cc += c * c;
                                                                    ss += s * s;
                                                                    cs += c * s;
                                                                    c1 += c;
                                                                    s1 += s;
                                                                    yc += y * c;
                                                                    ys += y * s;
                                                                    y1 += y;
                                                                }
                                                                matrix[0][0] = cc;
                                                                matrix[0][1] = cs;
                                                                matrix[0][2] = c1;
                                                                matrix[0][3] = yc;
                                                                matrix[1][0] = cs;
                                                                matrix[1][1] = ss;
                                                                matrix[1][2] = s1;
                                                                matrix[1][3] = ys;
                                                                matrix[2][0] = c1;
                                                                matrix[2][1] = s1;
                                                                matrix[2][2] = (double)sample_count;
                                                                matrix[2][3] = y1;
                                                                if (calibration_solve_3x3(matrix, solution) != 0) return -2;
                                                                for (size_t i = 0U;
                                                                i < sample_count;
                                                                ++i) {
                                                                    const double angle = 6.28318530717958647692 *             frequency_cycles_per_sample * (double)i;
                                                                    const double fitted = solution[0] * cos(angle) +                               solution[1] * sin(angle) +                               solution[2];
                                                                    const double e = samples[i] - fitted;
                                                                    if (fitted_waveform != NULL) fitted_waveform[i] = fitted;
                                                                    if (residual != NULL) residual[i] = e;
                                                                    sse += e * e;
                                                                    fitted_power += fitted * fitted;
                                                                    error_power += e * e;
                                                                }
                                                                memset(fit, 0, sizeof(*fit));
                                                                fit->valid = 1U;
                                                                fit->fitted_frequency_hz =         frequency_cycles_per_sample * sample_rate_hz;
                                                                fit->initial_frequency_hz = initial_frequency_hz;
                                                                fit->expected_frequency_hz = expected_frequency_hz;
                                                                fit->cosine_coefficient = solution[0];
                                                                fit->sine_coefficient = solution[1];
                                                                fit->amplitude = hypot(solution[0], solution[1]);
                                                                fit->phase_rad = atan2(-solution[1], solution[0]);
                                                                fit->dc_offset_codes = solution[2];
                                                                fit->rmse = sqrt(sse / (double)sample_count);
                                                                fit->frequency_error_hz =         isfinite(expected_frequency_hz) ?         fit->fitted_frequency_hz - expected_frequency_hz : NAN;
                                                                fit->tone_only_correlation =         calibration_normalized_correlation_double(             samples, fitted_waveform, sample_count);
                                                                if (!isfinite(fit->tone_only_correlation) &&         fitted_power > DBL_EPSILON && error_power >= 0.0)         fit->tone_only_correlation =             sqrt(fitted_power / (fitted_power + error_power));
                                                                return isfinite(fit->rmse) ? 0 : -3;
                                                            }
                                                            static int calibration_fit_tone_refined(     const double *samples,     size_t sample_count,     double expected_frequency_hz,     double sample_rate_hz,     calibration_tone_fit_result_t *fit,     double *fitted_waveform,     double *residual) {
                                                                calibration_tone_fit_result_t candidate_fit;
                                                                double best_frequency;
                                                                double step;
                                                                double best_rmse;
                                                                static double scratch_fit[ADC_CHANNEL_SAMPLE_COUNT];
                                                                static double scratch_residual[ADC_CHANNEL_SAMPLE_COUNT];
                                                                if (samples == NULL || fit == NULL || sample_count == 0U ||         !isfinite(expected_frequency_hz) || expected_frequency_hz <= 0.0 ||         !isfinite(sample_rate_hz) || sample_rate_hz <= 0.0)         return -1;
                                                                best_frequency = expected_frequency_hz / sample_rate_hz;
                                                                if (best_frequency <= 0.0 || best_frequency >= 0.5) return -2;
                                                                step = 0.25 / (double)sample_count;
                                                                if (calibration_fit_tone_at_frequency(samples, sample_count,             best_frequency, sample_rate_hz, expected_frequency_hz,             expected_frequency_hz, fit, fitted_waveform, residual) != 0)         return -3;
                                                                best_rmse = fit->rmse;
                                                                for (size_t iteration = 0U;
                                                                iteration < CAL_TONE_REFINE_MAX_ITERATIONS;
                                                                ++iteration) {
                                                                    int improved = 0;
                                                                    const double half_range =             CAL_TONE_REFINE_HALF_RANGE_BINS / (double)sample_count;
                                                                    const double probes[2] = {
                                                                        best_frequency - step,             best_frequency + step         }
                                                                        ;
                                                                        for (size_t p = 0U;
                                                                        p < 2U;
                                                                        ++p) {
                                                                            const double candidate = probes[p];
                                                                            if (candidate <= 0.0 || candidate >= 0.5 ||                 fabs(candidate -                     expected_frequency_hz / sample_rate_hz) > half_range)                 continue;
                                                                            if (calibration_fit_tone_at_frequency(samples, sample_count,                     candidate, sample_rate_hz, expected_frequency_hz,                     expected_frequency_hz, &candidate_fit,                     scratch_fit, scratch_residual) == 0 &&                 candidate_fit.rmse < best_rmse) {
                                                                                best_frequency = candidate;
                                                                                best_rmse = candidate_fit.rmse;
                                                                                *fit = candidate_fit;
                                                                                if (fitted_waveform != NULL)                     memcpy(fitted_waveform, scratch_fit,                            sample_count * sizeof(fitted_waveform[0]));
                                                                                if (residual != NULL)                     memcpy(residual, scratch_residual,                            sample_count * sizeof(residual[0]));
                                                                                improved = 1;
                                                                                break;
                                                                            }
                                                                        }
                                                                        if (!improved) {
                                                                            step *= 0.5;
                                                                            if (step < CAL_TONE_REFINE_MIN_STEP) break;
                                                                        }
                                                                    }
                                                                    return 0;
                                                                }
                                                                static int calibration_align_dither_residual(     const double *dither_reference,     const double *residual,     size_t sample_count,     calibration_dither_channel_alignment_t *alignment) {
                                                                    static double scores[ADC_CHANNEL_SAMPLE_COUNT];
                                                                    double residual_mean = 0.0;
                                                                    double ref_power = 0.0;
                                                                    double residual_power = 0.0;
                                                                    double best_abs = -1.0;
                                                                    double second_abs = -1.0;
                                                                    double background_sum = 0.0;
                                                                    double background_square_sum = 0.0;
                                                                    size_t background_count = 0U;
                                                                    size_t best_lag = 0U;
                                                                    size_t second_lag = 0U;
                                                                    if (dither_reference == NULL || residual == NULL ||         alignment == NULL || sample_count == 0U ||         sample_count > ADC_CHANNEL_SAMPLE_COUNT)         return -1;
                                                                    memset(alignment, 0, sizeof(*alignment));
                                                                    for (size_t i = 0U;
                                                                    i < sample_count;
                                                                    ++i)         residual_mean += residual[i];
                                                                    residual_mean /= (double)sample_count;
                                                                    for (size_t i = 0U;
                                                                    i < sample_count;
                                                                    ++i) {
                                                                        ref_power += dither_reference[i] * dither_reference[i];
                                                                        residual_power +=             (residual[i] - residual_mean) *             (residual[i] - residual_mean);
                                                                    }
                                                                    if (ref_power <= DBL_EPSILON || residual_power <= DBL_EPSILON)         return -2;
                                                                    for (size_t lag = 0U;
                                                                    lag < sample_count;
                                                                    ++lag) {
                                                                        double numerator = 0.0;
                                                                        for (size_t i = 0U;
                                                                        i < sample_count;
                                                                        ++i) {
                                                                            size_t sample_index = i + lag;
                                                                            if (sample_index >= sample_count) sample_index -= sample_count;
                                                                            numerator += dither_reference[i] *                 (residual[sample_index] - residual_mean);
                                                                        }
                                                                        scores[lag] = numerator / sqrt(ref_power * residual_power);
                                                                        if (fabs(scores[lag]) > best_abs) {
                                                                            best_abs = fabs(scores[lag]);
                                                                            best_lag = lag;
                                                                        }
                                                                    }
                                                                    for (size_t lag = 0U;
                                                                    lag < sample_count;
                                                                    ++lag) {
                                                                        size_t distance = lag > best_lag ? lag - best_lag : best_lag - lag;
                                                                        if (distance > sample_count - distance)             distance = sample_count - distance;
                                                                        if (distance <= CAL_DITHER_PEAK_GUARD_SAMPLES) continue;
                                                                        if (fabs(scores[lag]) > second_abs) {
                                                                            second_abs = fabs(scores[lag]);
                                                                            second_lag = lag;
                                                                        }
                                                                        background_sum += fabs(scores[lag]);
                                                                        background_square_sum += fabs(scores[lag]) * fabs(scores[lag]);
                                                                        ++background_count;
                                                                    }
                                                                    {
                                                                        const size_t left = best_lag == 0U ? sample_count - 1U : best_lag - 1U;
                                                                        const size_t right = best_lag + 1U == sample_count ? 0U : best_lag + 1U;
                                                                        const double y0 = fabs(scores[left]);
                                                                        const double y1 = fabs(scores[best_lag]);
                                                                        const double y2 = fabs(scores[right]);
                                                                        const double denominator = y0 - 2.0 * y1 + y2;
                                                                        double fraction = 0.0;
                                                                        double signed_lag;
                                                                        double n0;
                                                                        if (fabs(denominator) > 1.0e-18) {
                                                                            fraction = 0.5 * (y0 - y2) / denominator;
                                                                            if (!isfinite(fraction) || fraction < -0.5 || fraction > 0.5)                 fraction = 0.0;
                                                                        }
                                                                        signed_lag = best_lag <= sample_count / 2U ?             (double)best_lag + fraction :             (double)best_lag - (double)sample_count + fraction;
                                                                        n0 = -signed_lag;
                                                                        n0 = fmod(n0, (double)sample_count);
                                                                        if (n0 < 0.0) n0 += (double)sample_count;
                                                                        alignment->n0_integer = floor(n0);
                                                                        alignment->n0_fractional = n0 - alignment->n0_integer;
                                                                        alignment->derived_lag =             calibration_wrap_lag(signed_lag, sample_count);
                                                                    }
                                                                    alignment->peak = scores[best_lag];
                                                                    alignment->second_peak =         second_abs >= 0.0 ? scores[second_lag] : NAN;
                                                                    alignment->sign = scores[best_lag] >= 0.0 ? 1.0 : -1.0;
                                                                    alignment->peak_ratio =         second_abs > DBL_EPSILON ? best_abs / second_abs : NAN;
                                                                    if (background_count > 1U) {
                                                                        const double mean = background_sum / (double)background_count;
                                                                        double variance =             background_square_sum / (double)background_count -             mean * mean;
                                                                        if (variance < 0.0 && variance > -1.0e-18) variance = 0.0;
                                                                        alignment->align_margin =             variance > DBL_EPSILON ?             (best_abs - mean) / sqrt(variance) : NAN;
                                                                    }
                                                                    else {
                                                                        alignment->align_margin = NAN;
                                                                    }
                                                                    alignment->valid = isfinite(alignment->peak) &&         isfinite(alignment->derived_lag);
                                                                    return alignment->valid ? 0 : -3;
                                                                }
                                                                static double calibration_estimate_dither_gain(     const double *dither_reference,     const double *residual,     size_t sample_count,     double lag) {
                                                                    double residual_mean = 0.0;
                                                                    double numerator = 0.0;
                                                                    double denominator = 0.0;
                                                                    if (dither_reference == NULL || residual == NULL ||         sample_count == 0U || !isfinite(lag))         return NAN;
                                                                    for (size_t i = 0U;
                                                                    i < sample_count;
                                                                    ++i) residual_mean += residual[i];
                                                                    residual_mean /= (double)sample_count;
                                                                    for (size_t n = 0U;
                                                                    n < sample_count;
                                                                    ++n) {
                                                                        const double ref =             calibration_circular_sample_double(                 dither_reference, sample_count, (double)n - lag);
                                                                        if (!isfinite(ref)) return NAN;
                                                                        numerator += (residual[n] - residual_mean) * ref;
                                                                        denominator += ref * ref;
                                                                    }
                                                                    return denominator > DBL_EPSILON ? numerator / denominator : NAN;
                                                                }
                                                                static void calibration_synthesize_dither(     const double *dither_reference,     size_t sample_count,     double lag,     double gain,     double *dither_hat) {
                                                                    if (dither_reference == NULL || dither_hat == NULL ||         sample_count == 0U || !isfinite(lag) || !isfinite(gain))         return;
                                                                    for (size_t n = 0U;
                                                                    n < sample_count;
                                                                    ++n) {
                                                                        dither_hat[n] = gain * calibration_circular_sample_double(             dither_reference, sample_count, (double)n - lag);
                                                                    }
                                                                }
                                                                static int calibration_two_pass_tone_dither(     const int16_t *samples,     const double *dither_reference,     size_t sample_count,     double expected_tone_hz,     double sample_rate_hz,     calibration_tone_fit_result_t *initial_fit,     calibration_tone_fit_result_t *final_fit,     calibration_dither_channel_alignment_t *final_alignment) {
                                                                    static double y[ADC_CHANNEL_SAMPLE_COUNT];
                                                                    static double tone[ADC_CHANNEL_SAMPLE_COUNT];
                                                                    static double residual[ADC_CHANNEL_SAMPLE_COUNT];
                                                                    static double dither_hat[ADC_CHANNEL_SAMPLE_COUNT];
                                                                    static double y_minus_dither[ADC_CHANNEL_SAMPLE_COUNT];
                                                                    calibration_dither_channel_alignment_t first_alignment;
                                                                    double dither_gain;
                                                                    if (samples == NULL || dither_reference == NULL ||         initial_fit == NULL || final_fit == NULL ||         final_alignment == NULL || sample_count == 0U ||         sample_count > ADC_CHANNEL_SAMPLE_COUNT)         return -1;
                                                                    for (size_t i = 0U;
                                                                    i < sample_count;
                                                                    ++i) {
                                                                        y[i] = samples[i];
                                                                        dither_hat[i] = 0.0;
                                                                    }
                                                                    if (calibration_fit_tone_refined(y, sample_count, expected_tone_hz,             sample_rate_hz, initial_fit, tone, residual) != 0)         return -2;
                                                                    if (calibration_align_dither_residual(dither_reference, residual,             sample_count, &first_alignment) != 0)         return -3;
                                                                    dither_gain = calibration_estimate_dither_gain(         dither_reference, residual, sample_count,         first_alignment.derived_lag);
                                                                    if (isfinite(dither_gain))         calibration_synthesize_dither(dither_reference, sample_count,             first_alignment.derived_lag, dither_gain, dither_hat);
                                                                    for (size_t i = 0U;
                                                                    i < sample_count;
                                                                    ++i)         y_minus_dither[i] = y[i] - dither_hat[i];
                                                                    if (calibration_fit_tone_refined(y_minus_dither, sample_count,             expected_tone_hz, sample_rate_hz, final_fit, tone,             residual) != 0)         return -4;
                                                                    for (size_t i = 0U;
                                                                    i < sample_count;
                                                                    ++i)         residual[i] = y[i] - dither_hat[i] - tone[i];
                                                                    if (calibration_align_dither_residual(dither_reference, residual,             sample_count, final_alignment) != 0)         return -5;
                                                                    return 0;
                                                                }
                                                                static void calibration_count_dither_events(     const double *dither_reference,     size_t sample_count,     size_t fixed_window_start,     size_t fixed_window_length,     uint32_t *complete_count,     double *spacing_samples,     uint32_t *partial_window_count,     uint32_t *total_count,     uint8_t *indices_valid) {
                                                                    double max_abs = 0.0;
                                                                    double last_center = NAN;
                                                                    double spacing_sum = 0.0;
                                                                    uint32_t spacing_count = 0U;
                                                                    uint32_t count = 0U;
                                                                    uint32_t partial_count = 0U;
                                                                    uint32_t event_count = 0U;
                                                                    const size_t fixed_window_end = fixed_window_start + fixed_window_length;
                                                                    if (complete_count != NULL) *complete_count = 0U;
                                                                    if (spacing_samples != NULL) *spacing_samples = NAN;
                                                                    if (partial_window_count != NULL) *partial_window_count = 0U;
                                                                    if (total_count != NULL) *total_count = 0U;
                                                                    if (indices_valid != NULL) *indices_valid = 0U;
                                                                    if (dither_reference == NULL || sample_count == 0U ||         fixed_window_start >= sample_count ||         fixed_window_end > sample_count)         return;
                                                                    for (size_t i = 0U;
                                                                    i < sample_count;
                                                                    ++i)         if (fabs(dither_reference[i]) > max_abs)             max_abs = fabs(dither_reference[i]);
                                                                    if (max_abs <= DBL_EPSILON) return;
                                                                    for (size_t i = 0U;
                                                                    i < sample_count;
                                                                    ) {
                                                                        if (fabs(dither_reference[i]) <             CAL_DITHER_EVENT_THRESHOLD_FRACTION * max_abs) {
                                                                            ++i;
                                                                            continue;
                                                                        }
                                                                        {
                                                                            const size_t start = i;
                                                                            double weighted_sum = 0.0;
                                                                            double weight_sum = 0.0;
                                                                            while (i < sample_count &&                    fabs(dither_reference[i]) >=                        CAL_DITHER_EVENT_THRESHOLD_FRACTION * max_abs) {
                                                                                const double weight = fabs(dither_reference[i]);
                                                                                weighted_sum += (double)i * weight;
                                                                                weight_sum += weight;
                                                                                ++i;
                                                                            }
                                                                            if (weight_sum > DBL_EPSILON) {
                                                                                const double center = weighted_sum / weight_sum;
                                                                                const bool overlaps_window =         start < fixed_window_end && i > fixed_window_start;
                                                                                const bool complete_inside_window =         start >= fixed_window_start && i <= fixed_window_end;
                                                                                ++event_count;
                                                                                if (isfinite(last_center)) {
                                                                                    spacing_sum += center - last_center;
                                                                                    ++spacing_count;
                                                                                }
                                                                                last_center = center;
                                                                                if (complete_inside_window)                     ++count;
                                                                                else if (overlaps_window)                     ++partial_count;
                                                                            }
                                                                        }
                                                                    }
                                                                    if (complete_count != NULL) *complete_count = count;
                                                                    if (spacing_samples != NULL && spacing_count > 0U)         *spacing_samples = spacing_sum / (double)spacing_count;
                                                                    if (partial_window_count != NULL) *partial_window_count = partial_count;
                                                                    if (total_count != NULL) *total_count = event_count;
                                                                    if (indices_valid != NULL) *indices_valid = 1U;
                                                                }
                                                                static int calibration_compute_timing_diagnostics(     const int16_t *alignment_reference,     const int16_t *channel_a,     const int16_t *channel_b,     size_t sample_count,     int selected_channel,     double expected_tone_hz,     double adc_sample_rate_hz,     double existing_lag,     size_t fixed_window_start,     size_t fixed_window_length,     calibration_timing_diagnostics_t *diagnostics) {
                                                                    static double reference_as_double[ADC_CHANNEL_SAMPLE_COUNT];
                                                                    static double reference_tone[ADC_CHANNEL_SAMPLE_COUNT];
                                                                    static double dither_reference[ADC_CHANNEL_SAMPLE_COUNT];
                                                                    calibration_tone_fit_result_t reference_fit;
                                                                    calibration_tone_fit_result_t initial_fits[2];
                                                                    calibration_tone_fit_result_t final_fits[2];
                                                                    calibration_dither_channel_alignment_t channel_alignments[2];
                                                                    const int16_t *channels[2] = {
                                                                        channel_a, channel_b }
                                                                        ;
                                                                        int common_channel = -1;
                                                                        if (diagnostics == NULL) return -1;
                                                                        memset(diagnostics, 0, sizeof(*diagnostics));
                                                                        memset(initial_fits, 0, sizeof(initial_fits));
                                                                        memset(final_fits, 0, sizeof(final_fits));
                                                                        memset(channel_alignments, 0, sizeof(channel_alignments));
                                                                        diagnostics->status_text = "unavailable";
                                                                        diagnostics->selected_common_channel = -1;
                                                                        diagnostics->dither_n0_integer = -1;
                                                                        diagnostics->dither_n0_fractional = NAN;
                                                                        diagnostics->dither_sign = NAN;
                                                                        diagnostics->dither_peak = NAN;
                                                                        diagnostics->dither_second_peak = NAN;
                                                                        diagnostics->dither_peak_ratio = NAN;
                                                                        diagnostics->dither_align_margin = NAN;
                                                                        diagnostics->dither_event_spacing_samples = NAN;
                                                                        diagnostics->dither_derived_lag = NAN;
                                                                        diagnostics->alignment_disagreement_samples = NAN;
                                                                        diagnostics->dither_event_indices_valid = 0U;
                                                                        diagnostics->validation.channel_a_n0 = NAN;
                                                                        diagnostics->validation.channel_b_n0 = NAN;
                                                                        diagnostics->validation.common_selected_n0 = NAN;
                                                                        diagnostics->validation.channel_n0_disagreement_samples = NAN;
                                                                        if (alignment_reference == NULL || channel_a == NULL || channel_b == NULL ||         sample_count == 0U || sample_count > ADC_CHANNEL_SAMPLE_COUNT ||         selected_channel < 0 || selected_channel > 1 ||         !isfinite(expected_tone_hz) || expected_tone_hz <= 0.0 ||         !isfinite(adc_sample_rate_hz) || adc_sample_rate_hz <= 0.0 ||         !isfinite(existing_lag)) {
                                                                            diagnostics->status_text = "invalid input";
                                                                            return -1;
                                                                        }
                                                                        for (size_t i = 0U;
                                                                        i < sample_count;
                                                                        ++i)         reference_as_double[i] = alignment_reference[i];
                                                                        if (calibration_fit_tone_refined(reference_as_double, sample_count,             expected_tone_hz, adc_sample_rate_hz, &reference_fit,             reference_tone, dither_reference) != 0) {
                                                                            diagnostics->status_text = "reference tone fit failed";
                                                                            return -2;
                                                                        }
                                                                        {
                                                                            double mean = 0.0;
                                                                            double power = 0.0;
                                                                            for (size_t i = 0U;
                                                                            i < sample_count;
                                                                            ++i) mean += dither_reference[i];
                                                                            mean /= (double)sample_count;
                                                                            for (size_t i = 0U;
                                                                            i < sample_count;
                                                                            ++i) {
                                                                                dither_reference[i] -= mean;
                                                                                power += dither_reference[i] * dither_reference[i];
                                                                            }
                                                                            if (power <= DBL_EPSILON) {
                                                                                diagnostics->status_text = "no dither component detected";
                                                                                return -3;
                                                                            }
                                                                        }
                                                                        for (size_t channel = 0U;
                                                                        channel < 2U;
                                                                        ++channel) {
                                                                            if (calibration_two_pass_tone_dither(                 channels[channel], dither_reference, sample_count,                 expected_tone_hz, adc_sample_rate_hz,                 &initial_fits[channel], &final_fits[channel],                 &channel_alignments[channel]) == 0) {
                                                                                diagnostics->channel[channel] = channel_alignments[channel];
                                                                                if (common_channel < 0 ||                 fabs(channel_alignments[channel].peak) >                 fabs(channel_alignments[(size_t)common_channel].peak)) {
                                                                                    common_channel = (int)channel;
                                                                                }
                                                                            }
                                                                        }
                                                                        if (common_channel < 0) {
                                                                            diagnostics->status_text = "dither alignment failed";
                                                                            return -4;
                                                                        }
                                                                        if (!channel_alignments[(size_t)selected_channel].valid ||         !final_fits[(size_t)selected_channel].valid) {
                                                                            diagnostics->status_text =             "selected-channel tone/dither diagnostic failed";
                                                                            return -5;
                                                                        }
                                                                        diagnostics->initial_tone = initial_fits[(size_t)selected_channel];
                                                                        diagnostics->tone = final_fits[(size_t)selected_channel];
                                                                        diagnostics->selected_common_channel = (int8_t)common_channel;
                                                                        diagnostics->dither_n0_integer =         (int32_t)diagnostics->channel[(size_t)common_channel].n0_integer;
                                                                        diagnostics->dither_n0_fractional =         diagnostics->channel[(size_t)common_channel].n0_fractional;
                                                                        diagnostics->dither_sign =         diagnostics->channel[(size_t)common_channel].sign;
                                                                        diagnostics->dither_peak =         diagnostics->channel[(size_t)common_channel].peak;
                                                                        diagnostics->dither_second_peak =         diagnostics->channel[(size_t)common_channel].second_peak;
                                                                        diagnostics->dither_peak_ratio =         diagnostics->channel[(size_t)common_channel].peak_ratio;
                                                                        diagnostics->dither_align_margin =         diagnostics->channel[(size_t)common_channel].align_margin;
                                                                        diagnostics->dither_derived_lag =         diagnostics->channel[(size_t)common_channel].derived_lag;
                                                                        diagnostics->alignment_disagreement_samples =         calibration_wrap_lag(existing_lag - diagnostics->dither_derived_lag,                              sample_count);
                                                                        diagnostics->alignment_methods_consistent =         isfinite(diagnostics->alignment_disagreement_samples) &&         fabs(diagnostics->alignment_disagreement_samples) <=             CAL_EXISTING_DITHER_LAG_TOLERANCE_SAMPLES;
                                                                        calibration_count_dither_events(         dither_reference, sample_count, fixed_window_start,         fixed_window_length, &diagnostics->complete_dither_event_count,         &diagnostics->dither_event_spacing_samples,         &diagnostics->partial_dither_event_count,         &diagnostics->total_dither_event_count,         &diagnostics->dither_event_indices_valid);
                                                                        diagnostics->valid = 1U;
                                                                        diagnostics->status_text = "diagnostic only";
                                                                        return 0;
                                                                    }
                                                                    static const char *calibration_validation_status_text(     calibration_timing_validation_status_t status) {
                                                                        switch (status) {
                                                                        case CAL_TIMING_VALIDATION_PASS:
                                                                            return "PASS";
                                                                        case CAL_TIMING_VALIDATION_WARNING:
                                                                            return "WARNING";
                                                                        case CAL_TIMING_VALIDATION_FAIL:
                                                                        default:
                                                                            return "FAIL";
                                                                        }
                                                                    }
                                                                    static double calibration_dither_n0_value(     const calibration_dither_channel_alignment_t *alignment) {
                                                                        if (alignment == NULL || !alignment->valid ||         !isfinite(alignment->n0_integer) ||         !isfinite(alignment->n0_fractional))         return NAN;
                                                                        return alignment->n0_integer + alignment->n0_fractional;
                                                                    }
                                                                    static bool calibration_tone_fit_parameters_are_finite(     const calibration_tone_fit_result_t *fit) {
                                                                        return fit != NULL && fit->valid &&         isfinite(fit->fitted_frequency_hz) &&         isfinite(fit->expected_frequency_hz) &&         isfinite(fit->frequency_error_hz) &&         isfinite(fit->cosine_coefficient) &&         isfinite(fit->sine_coefficient) &&         isfinite(fit->amplitude) &&         isfinite(fit->phase_rad) &&         isfinite(fit->dc_offset_codes) &&         isfinite(fit->rmse) &&         isfinite(fit->tone_only_correlation);
                                                                    }
                                                                    static bool calibration_dither_alignment_values_are_finite(     const calibration_dither_channel_alignment_t *alignment) {
                                                                        return alignment != NULL && alignment->valid &&         isfinite(alignment->n0_integer) &&         isfinite(alignment->n0_fractional) &&         isfinite(alignment->sign) &&         isfinite(alignment->peak) &&         isfinite(alignment->second_peak) &&         isfinite(alignment->peak_ratio) &&         isfinite(alignment->align_margin) &&         isfinite(alignment->derived_lag);
                                                                    }
                                                                    static bool calibration_fixed_window_is_valid(     const calibration_aligned_frame_t *frame) {
                                                                        if (frame == NULL ||         frame->calibration_window_length != CAL_FIXED_WINDOW_LENGTH ||         frame->alignment_reference_count == 0U)         return false;
                                                                        if (frame->calibration_window_start > frame->alignment_reference_count)         return false;
                                                                        return frame->calibration_window_length <=             frame->alignment_reference_count - frame->calibration_window_start;
                                                                    }
                                                                    static void calibration_validate_timing_alignment(     const calibration_aligned_frame_t *frame,     calibration_timing_diagnostics_t *diagnostics) {
                                                                        calibration_timing_validation_t *validation;
                                                                        bool tone_finite;
                                                                        bool selected_dither_finite = false;
                                                                        bool numerical_pass;
                                                                        bool all_new_checks_pass;
                                                                        double channel_a_n0;
                                                                        double channel_b_n0;
                                                                        double common_n0;
                                                                        if (diagnostics == NULL) return;
                                                                        validation = &diagnostics->validation;
                                                                        memset(validation, 0, sizeof(*validation));
                                                                        validation->channel_a_n0 = NAN;
                                                                        validation->channel_b_n0 = NAN;
                                                                        validation->common_selected_n0 = NAN;
                                                                        validation->channel_n0_disagreement_samples = NAN;
                                                                        validation->numerical_reason = "validation not evaluated";
                                                                        validation->existing_status = CAL_TIMING_VALIDATION_FAIL;
                                                                        validation->tone_status = CAL_TIMING_VALIDATION_FAIL;
                                                                        validation->dither_status = CAL_TIMING_VALIDATION_FAIL;
                                                                        validation->channel_status = CAL_TIMING_VALIDATION_FAIL;
                                                                        validation->existing_dither_status = CAL_TIMING_VALIDATION_WARNING;
                                                                        validation->window_status = CAL_TIMING_VALIDATION_FAIL;
                                                                        validation->numerical_status = CAL_TIMING_VALIDATION_FAIL;
                                                                        validation->overall_status = CAL_TIMING_VALIDATION_FAIL;
                                                                        if (frame == NULL) {
                                                                            validation->numerical_reason = "missing timing frame";
                                                                            validation->valid = 1U;
                                                                            return;
                                                                        }
                                                                        validation->existing_status =         frame->frame_valid && frame->timing.accepted &&         frame->timing.reject_reason == CAL_TIMING_REJECT_NONE &&         isfinite(frame->correlation) &&         frame->correlation >= CAL_TIMING_MIN_CORRELATION &&         isfinite(frame->fractional_lag) &&         fabsf(frame->fractional_lag) <= CAL_TIMING_MAX_ABS_FRAC_LAG &&         frame->timing.analysis_samples >= CAL_TIMING_MIN_ANALYSIS_SAMPLES &&         (frame->canonical_reference_phase == 0 ||          frame->canonical_reference_phase == 1) &&         calibration_fixed_window_is_valid(frame) ?         CAL_TIMING_VALIDATION_PASS : CAL_TIMING_VALIDATION_FAIL;
                                                                        if (!diagnostics->valid) {
                                                                            validation->numerical_reason =         diagnostics->status_text != NULL ?         diagnostics->status_text : "tone/dither diagnostics unavailable";
                                                                            validation->valid = 1U;
                                                                            validation->overall_status = CAL_TIMING_VALIDATION_FAIL;
                                                                            return;
                                                                        }
                                                                        tone_finite = calibration_tone_fit_parameters_are_finite(         &diagnostics->tone);
                                                                        validation->tone_status =         tone_finite && diagnostics->tone.rmse <=             CAL_TONE_VALIDATION_MAX_RMSE_CODES &&         diagnostics->tone.tone_only_correlation >=             CAL_TONE_VALIDATION_MIN_CORRELATION ?         CAL_TIMING_VALIDATION_PASS : CAL_TIMING_VALIDATION_FAIL;
                                                                        if (diagnostics->selected_common_channel >= 0 &&         diagnostics->selected_common_channel < 2) {
                                                                            selected_dither_finite =         calibration_dither_alignment_values_are_finite(             &diagnostics->channel[(size_t)diagnostics->selected_common_channel]);
                                                                        }
                                                                        validation->dither_status =         selected_dither_finite &&         diagnostics->complete_dither_event_count >=             CAL_DITHER_VALIDATION_MIN_COMPLETE_EVENTS &&         diagnostics->dither_align_margin >=             CAL_DITHER_VALIDATION_MIN_MARGIN &&         diagnostics->dither_peak_ratio >=             CAL_DITHER_VALIDATION_MIN_PEAK_RATIO ?         CAL_TIMING_VALIDATION_PASS : CAL_TIMING_VALIDATION_FAIL;
                                                                        channel_a_n0 = calibration_dither_n0_value(         &diagnostics->channel[0]);
                                                                        channel_b_n0 = calibration_dither_n0_value(         &diagnostics->channel[1]);
                                                                        common_n0 =         (diagnostics->selected_common_channel >= 0 &&          diagnostics->selected_common_channel < 2) ?         calibration_dither_n0_value(             &diagnostics->channel[(size_t)diagnostics->selected_common_channel]) :         NAN;
                                                                        validation->channel_a_n0 = channel_a_n0;
                                                                        validation->channel_b_n0 = channel_b_n0;
                                                                        validation->common_selected_n0 = common_n0;
                                                                        if (isfinite(channel_a_n0) && isfinite(channel_b_n0) &&         frame->alignment_reference_count > 0U) {
                                                                            validation->channel_n0_disagreement_samples =         fabs(calibration_wrap_lag(channel_a_n0 - channel_b_n0,             frame->alignment_reference_count));
                                                                        }
                                                                        validation->channel_status =         isfinite(validation->channel_n0_disagreement_samples) &&         validation->channel_n0_disagreement_samples <=             CAL_DITHER_CHANNEL_N0_TOLERANCE_SAMPLES ?         CAL_TIMING_VALIDATION_PASS : CAL_TIMING_VALIDATION_FAIL;
                                                                        validation->existing_dither_status =         isfinite(diagnostics->alignment_disagreement_samples) &&         fabs(diagnostics->alignment_disagreement_samples) <=             CAL_EXISTING_DITHER_LAG_TOLERANCE_SAMPLES ?         CAL_TIMING_VALIDATION_PASS : CAL_TIMING_VALIDATION_WARNING;
                                                                        validation->window_partial_event_count =         diagnostics->partial_dither_event_count;
                                                                        validation->total_dither_event_count =         diagnostics->total_dither_event_count;
                                                                        validation->dither_event_indices_valid =         diagnostics->dither_event_indices_valid;
                                                                        validation->window_status =         calibration_fixed_window_is_valid(frame) &&         diagnostics->dither_event_indices_valid &&         diagnostics->complete_dither_event_count >=             CAL_DITHER_VALIDATION_MIN_COMPLETE_EVENTS &&         diagnostics->partial_dither_event_count == 0U ?         CAL_TIMING_VALIDATION_PASS : CAL_TIMING_VALIDATION_FAIL;
                                                                        numerical_pass =         tone_finite && selected_dither_finite &&         diagnostics->dither_event_indices_valid &&         isfinite(frame->correlation) &&         isfinite(frame->fractional_lag) &&         isfinite(frame->total_lag) &&         isfinite(diagnostics->dither_derived_lag) &&         isfinite(diagnostics->alignment_disagreement_samples);
                                                                        if (diagnostics->channel[0].valid)         numerical_pass = numerical_pass &&             calibration_dither_alignment_values_are_finite(                 &diagnostics->channel[0]);
                                                                        if (diagnostics->channel[1].valid)         numerical_pass = numerical_pass &&             calibration_dither_alignment_values_are_finite(                 &diagnostics->channel[1]);
                                                                        validation->numerical_status =         numerical_pass ? CAL_TIMING_VALIDATION_PASS :         CAL_TIMING_VALIDATION_FAIL;
                                                                        validation->numerical_reason =         numerical_pass ? "finite metrics and valid event indices" :         "non-finite metric, failed frequency search, or invalid event index";
                                                                        all_new_checks_pass =         validation->tone_status == CAL_TIMING_VALIDATION_PASS &&         validation->dither_status == CAL_TIMING_VALIDATION_PASS &&         validation->channel_status == CAL_TIMING_VALIDATION_PASS &&         validation->existing_dither_status == CAL_TIMING_VALIDATION_PASS &&         validation->window_status == CAL_TIMING_VALIDATION_PASS &&         validation->numerical_status == CAL_TIMING_VALIDATION_PASS;
                                                                        if (validation->existing_status != CAL_TIMING_VALIDATION_PASS ||         validation->numerical_status != CAL_TIMING_VALIDATION_PASS) {
                                                                            validation->overall_status = CAL_TIMING_VALIDATION_FAIL;
                                                                        }
                                                                        else if (all_new_checks_pass) {
                                                                            validation->overall_status = CAL_TIMING_VALIDATION_PASS;
                                                                        }
                                                                        else {
                                                                            validation->overall_status = CAL_TIMING_VALIDATION_WARNING;
                                                                        }
                                                                        validation->valid = 1U;
                                                                    }
                                                                    static void calibration_print_timing_diagnostics_compact(     const calibration_timing_diagnostics_t *diagnostics) {
                                                                        const calibration_timing_validation_t *validation;
                                                                        if (diagnostics == NULL || !diagnostics->validation.valid) {
                                                                            xil_printf("----------------------------------------\r\n");
                                                                            xil_printf("Timing Alignment Summary\r\n");
                                                                            xil_printf("----------------------------------------\r\n");
                                                                            xil_printf("Existing correlation    : FAIL\r\n");
                                                                            xil_printf("Tone fit                : FAIL\r\n");
                                                                            xil_printf("Dither alignment        : FAIL\r\n");
                                                                            xil_printf("Channel consistency     : FAIL\r\n");
                                                                            xil_printf("Window validation       : FAIL\r\n");
                                                                            xil_printf("Numerical validation    : FAIL\r\n");
                                                                            xil_printf("Overall status          : FAIL\r\n");
                                                                            return;
                                                                        }
                                                                        validation = &diagnostics->validation;
                                                                        xil_printf("----------------------------------------\r\n");
                                                                        xil_printf("Timing Alignment Summary\r\n");
                                                                        xil_printf("----------------------------------------\r\n");
                                                                        xil_printf("Existing correlation    : %s\r\n",         calibration_validation_status_text(             validation->existing_status));
                                                                        xil_printf("Tone fit                : %s\r\n",         calibration_validation_status_text(             validation->tone_status));
                                                                        xil_printf("Dither alignment        : %s\r\n",         calibration_validation_status_text(             validation->dither_status));
                                                                        xil_printf("Channel consistency     : %s\r\n",         calibration_validation_status_text(             validation->channel_status));
                                                                        xil_printf("Existing/dither check   : %s\r\n",         calibration_validation_status_text(             validation->existing_dither_status));
                                                                        xil_printf("Window validation       : %s\r\n",         calibration_validation_status_text(             validation->window_status));
                                                                        xil_printf("Numerical validation    : %s\r\n",         calibration_validation_status_text(             validation->numerical_status));
                                                                        xil_printf("Overall status          : %s\r\n",         calibration_validation_status_text(             validation->overall_status));
                                                                    }
                                                                    static void calibration_print_timing_diagnostics_detail(     const calibration_aligned_frame_t *frame,     const calibration_timing_diagnostics_t *diagnostics) {
                                                                        const calibration_timing_validation_t *validation =         diagnostics != NULL && diagnostics->validation.valid ?         &diagnostics->validation : NULL;
                                                                        if (frame == NULL || diagnostics == NULL) return;
                                                                        xil_printf("\r\n========== Timing Alignment Diagnostics ==========\r\n");
                                                                        xil_printf("\r\nA. Existing timing result\r\n");
                                                                        xil_printf("Selected channel        : %s\r\n",         frame->selected_channel_name != NULL ?         frame->selected_channel_name : "none");
                                                                        xil_printf("Integer lag             : %ld samples\r\n",         (long)frame->integer_lag);
                                                                        print_float_value("Fractional lag", frame->fractional_lag, " samples");
                                                                        print_float_value("Full-waveform correlation", frame->correlation, "");
                                                                        xil_printf("Canonical phase         : %s\r\n",         frame->canonical_reference_phase == 0 ? "EVEN" :         frame->canonical_reference_phase == 1 ? "ODD" : "UNAVAILABLE");
                                                                        if (frame->calibration_window_length > 0U) {
                                                                            xil_printf("Fixed window            : %lu ... %lu (%lu samples)\r\n",             (unsigned long)frame->calibration_window_start,             (unsigned long)(frame->calibration_window_start +                 frame->calibration_window_length - 1U),             (unsigned long)frame->calibration_window_length);
                                                                        }
                                                                        else {
                                                                            xil_printf("Fixed window            : UNAVAILABLE\r\n");
                                                                        }
                                                                        xil_printf("Existing status         : %s\r\n",         frame->frame_valid ? "ACCEPTED" : "REJECTED");
                                                                        if (validation != NULL) {
                                                                            xil_printf("Existing validation     : %s\r\n",         calibration_validation_status_text(             validation->existing_status));
                                                                        }
                                                                        if (diagnostics->valid) {
                                                                            xil_printf("\r\nB. Tone fitting\r\n");
                                                                            if (validation != NULL) {
                                                                                xil_printf("Tone-fit validation     : %s\r\n",         calibration_validation_status_text(             validation->tone_status));
                                                                            }
                                                                            print_double_value("Expected frequency",             diagnostics->tone.expected_frequency_hz / 1.0e6, " MHz");
                                                                            print_double_value("Fitted frequency",             diagnostics->tone.fitted_frequency_hz / 1.0e6, " MHz");
                                                                            print_double_value("Frequency error",             diagnostics->tone.frequency_error_hz, " Hz");
                                                                            print_double_value("Cos coefficient",             diagnostics->tone.cosine_coefficient, " codes");
                                                                            print_double_value("Sin coefficient",             diagnostics->tone.sine_coefficient, " codes");
                                                                            print_double_value("Amplitude",             diagnostics->tone.amplitude, " codes");
                                                                            print_double_value("Phase",             diagnostics->tone.phase_rad, " rad");
                                                                            print_double_value("DC",             diagnostics->tone.dc_offset_codes, " codes");
                                                                            print_double_value("RMSE",             diagnostics->tone.rmse, " codes");
                                                                            print_double_value("Tone-only correlation",             diagnostics->tone.tone_only_correlation, "");
                                                                            xil_printf("\r\nC. Dither alignment\r\n");
                                                                            if (validation != NULL) {
                                                                                xil_printf("Dither validation       : %s\r\n",         calibration_validation_status_text(             validation->dither_status));
                                                                            }
                                                                            for (size_t channel = 0U;
                                                                            channel < 2U;
                                                                            ++channel) {
                                                                                const calibration_dither_channel_alignment_t *metric =                 &diagnostics->channel[channel];
                                                                                xil_printf("%s dither status     : %s\r\n",                 channel == 0U ? "Channel A" : "Channel B",                 metric->valid ? "VALID" : "UNAVAILABLE");
                                                                                if (metric->valid) {
                                                                                    print_double_value(channel == 0U ?                     "Channel A peak" : "Channel B peak",                     metric->peak, "");
                                                                                    print_double_value(channel == 0U ?                     "Channel A margin" : "Channel B margin",                     metric->align_margin, "");
                                                                                    print_double_value(channel == 0U ?                     "Channel A lag" : "Channel B lag",                     metric->derived_lag, " samples");
                                                                                }
                                                                            }
                                                                            xil_printf("Selected common origin : %s\r\n",             diagnostics->selected_common_channel == 0 ? "Channel A" :             diagnostics->selected_common_channel == 1 ? "Channel B" :             "UNAVAILABLE");
                                                                            xil_printf("Dither n0 integer      : %ld\r\n",             (long)diagnostics->dither_n0_integer);
                                                                            print_double_value("Dither n0 fractional",             diagnostics->dither_n0_fractional, " samples");
                                                                            print_double_value("Dither polarity",             diagnostics->dither_sign, "");
                                                                            print_double_value("Dither peak",             diagnostics->dither_peak, "");
                                                                            print_double_value("Dither second peak",             diagnostics->dither_second_peak, "");
                                                                            print_double_value("Dither peak ratio",             diagnostics->dither_peak_ratio, "");
                                                                            print_double_value("Dither margin",             diagnostics->dither_align_margin, "");
                                                                            xil_printf("Complete dither events : %lu\r\n",             (unsigned long)diagnostics->complete_dither_event_count);
                                                                            xil_printf("Partial window events  : %lu\r\n",             (unsigned long)diagnostics->partial_dither_event_count);
                                                                            print_double_value("Event spacing",             diagnostics->dither_event_spacing_samples, " samples");
                                                                            xil_printf("\r\nD. Channel consistency\r\n");
                                                                            if (validation != NULL) {
                                                                                print_double_value("Channel A n0",             validation->channel_a_n0, " samples");
                                                                                print_double_value("Channel B n0",             validation->channel_b_n0, " samples");
                                                                                print_double_value("Common selected n0",             validation->common_selected_n0, " samples");
                                                                                print_double_value("Channel disagreement",             validation->channel_n0_disagreement_samples, " samples");
                                                                                print_double_value("Channel tolerance",             CAL_DITHER_CHANNEL_N0_TOLERANCE_SAMPLES, " samples");
                                                                                xil_printf("Channel status         : %s\r\n",         calibration_validation_status_text(             validation->channel_status));
                                                                            }
                                                                            xil_printf("\r\nE. Existing vs dither alignment\r\n");
                                                                            print_double_value("Existing lag",             frame->total_lag, " samples");
                                                                            print_double_value("Dither-derived lag",             diagnostics->dither_derived_lag, " samples");
                                                                            print_double_value("Wrapped disagreement",             diagnostics->alignment_disagreement_samples, " samples");
                                                                            print_double_value("Consistency tolerance",             CAL_EXISTING_DITHER_LAG_TOLERANCE_SAMPLES, " samples");
                                                                            if (validation != NULL) {
                                                                                xil_printf("Consistency status     : %s\r\n",         calibration_validation_status_text(             validation->existing_dither_status));
                                                                            }
                                                                            else {
                                                                                xil_printf("Consistency status     : %s\r\n",             diagnostics->alignment_methods_consistent ? "CONSISTENT" :             "DISAGREE");
                                                                            }
                                                                            if (validation != NULL) {
                                                                                xil_printf("\r\nF. Window validation\r\n");
                                                                                xil_printf("Minimum events required: %lu\r\n",             (unsigned long)CAL_DITHER_VALIDATION_MIN_COMPLETE_EVENTS);
                                                                                xil_printf("Complete events        : %lu\r\n",             (unsigned long)diagnostics->complete_dither_event_count);
                                                                                xil_printf("Partial window events  : %lu\r\n",             (unsigned long)validation->window_partial_event_count);
                                                                                xil_printf("Event indices valid    : %s\r\n",             validation->dither_event_indices_valid ? "YES" : "NO");
                                                                                xil_printf("Window status          : %s\r\n",         calibration_validation_status_text(             validation->window_status));
                                                                                xil_printf("\r\nG. Numerical validation\r\n");
                                                                                xil_printf("Numerical status       : %s\r\n",         calibration_validation_status_text(             validation->numerical_status));
                                                                                xil_printf("Numerical reason       : %s\r\n",             validation->numerical_reason != NULL ?             validation->numerical_reason : "unknown");
                                                                                xil_printf("\r\nOverall recommendation  : %s\r\n",         calibration_validation_status_text(             validation->overall_status));
                                                                            }
                                                                        }
                                                                        else {
                                                                            xil_printf("\r\nTone/dither diagnostics : UNAVAILABLE\r\n");
                                                                            xil_printf("Diagnostic reason       : %s\r\n",             diagnostics->status_text != NULL ?             diagnostics->status_text : "unknown");
                                                                            if (validation != NULL) {
                                                                                xil_printf("Tone fit                : %s\r\n",         calibration_validation_status_text(             validation->tone_status));
                                                                                xil_printf("Dither alignment        : %s\r\n",         calibration_validation_status_text(             validation->dither_status));
                                                                                xil_printf("Channel consistency     : %s\r\n",         calibration_validation_status_text(             validation->channel_status));
                                                                                xil_printf("Window validation       : %s\r\n",         calibration_validation_status_text(             validation->window_status));
                                                                                xil_printf("Numerical validation    : %s\r\n",         calibration_validation_status_text(             validation->numerical_status));
                                                                                xil_printf("Overall recommendation  : %s\r\n",         calibration_validation_status_text(             validation->overall_status));
                                                                            }
                                                                        }
                                                                        xil_printf("No calibration coefficients were updated by diagnostics.\r\n");
                                                                        xil_printf("==================================================\r\n");
                                                                    }
                                                                    static calibration_dither_offset_diagnostic_t g_latest_dither_offset_diagnostic;
                                                                    static const char *calibration_dither_offset_status_name(     calibration_dither_offset_status_t status) {
                                                                        switch (status) {
                                                                        case CAL_DITHER_OFFSET_STATUS_PASS:
                                                                            return "PASS";
                                                                        case CAL_DITHER_OFFSET_STATUS_WARNING:
                                                                            return "WARNING";
                                                                        case CAL_DITHER_OFFSET_STATUS_INVALID:
                                                                        default:
                                                                            return "INVALID";
                                                                        }
                                                                    }
                                                                    static const char *calibration_dither_offset_reason_name(     calibration_dither_offset_reason_t reason) {
                                                                        switch (reason) {
                                                                        case CAL_DITHER_OFFSET_REASON_NONE:
                                                                            return "NONE";
                                                                        case CAL_DITHER_OFFSET_REASON_TIMING_CONTEXT:
                                                                            return "TIMING_CONTEXT";
                                                                        case CAL_DITHER_OFFSET_REASON_TONE_FIT:
                                                                            return "TONE_FIT";
                                                                        case CAL_DITHER_OFFSET_REASON_TOO_FEW_EVENTS:
                                                                            return "TOO_FEW_EVENTS";
                                                                        case CAL_DITHER_OFFSET_REASON_POLARITY_IMBALANCE:
                                                                            return "POLARITY_IMBALANCE";
                                                                        case CAL_DITHER_OFFSET_REASON_FLAT_TOP:
                                                                            return "FLAT_TOP";
                                                                        case CAL_DITHER_OFFSET_REASON_INTERPOLATION:
                                                                            return "INTERPOLATION";
                                                                        case CAL_DITHER_OFFSET_REASON_NUMERICAL:
                                                                            return "NUMERICAL";
                                                                        case CAL_DITHER_OFFSET_REASON_NO_DITHER:
                                                                            return "NO_DITHER";
                                                                        case CAL_DITHER_OFFSET_REASON_EVENT_PROFILE:
                                                                        default:
                                                                            return "EVENT_PROFILE";
                                                                        }
                                                                    }
                                                                    static void calibration_dither_offset_mark_invalid(     calibration_dither_offset_diagnostic_t *diagnostic,     calibration_dither_offset_reason_t reason,     double existing_offset_codes) {
                                                                        if (diagnostic == NULL) return;
                                                                        memset(diagnostic, 0, sizeof(*diagnostic));
                                                                        diagnostic->status = CAL_DITHER_OFFSET_STATUS_INVALID;
                                                                        diagnostic->reason = reason;
                                                                        diagnostic->existing_offset_codes = existing_offset_codes;
                                                                        diagnostic->dither_offset_codes = NAN;
                                                                        diagnostic->fitted_tone_dc_codes = NAN;
                                                                        diagnostic->existing_vs_dither_codes = NAN;
                                                                        diagnostic->dither_vs_fitted_dc_codes = NAN;
                                                                        diagnostic->offset_profile_std_codes = NAN;
                                                                        diagnostic->offset_profile_min_codes = NAN;
                                                                        diagnostic->offset_profile_max_codes = NAN;
                                                                        diagnostic->mean_event_polarity = NAN;
                                                                        diagnostic->separation_denominator = NAN;
                                                                        diagnostic->fitted_tone_frequency_hz = NAN;
                                                                        diagnostic->fitted_tone_amplitude_codes = NAN;
                                                                        diagnostic->fitted_tone_phase_rad = NAN;
                                                                        diagnostic->tone_fit_rmse_codes = NAN;
                                                                        diagnostic->tone_only_correlation = NAN;
                                                                        diagnostic->units = "corrected ADC codes";
                                                                    }
                                                                    typedef struct {
                                                                        double center;
                                                                        size_t start;
                                                                        size_t end;
                                                                        double polarity;
                                                                    } calibration_dither_offset_event_t;
                                                                    static int calibration_dither_offset_interpolate(     const double *samples, size_t count, double position,     double *value) {
                                                                        size_t lower;
                                                                        double fraction;
                                                                        if (samples == NULL || value == NULL || count == 0U ||         !isfinite(position) || position < 0.0 ||         position > (double)(count - 1U))         return -1;
                                                                        lower = (size_t)floor(position);
                                                                        if (lower >= count - 1U) {
                                                                            *value = samples[count - 1U];
                                                                            return isfinite(*value) ? 0 : -2;
                                                                        }
                                                                        fraction = position - (double)lower;
                                                                        *value = (1.0 - fraction) * samples[lower] +         fraction * samples[lower + 1U];
                                                                        return isfinite(*value) ? 0 : -2;
                                                                    }
                                                                    static int calibration_dither_offset_find_events(     const double *template_samples, size_t count,     calibration_dither_offset_event_t *events, size_t max_events,     uint32_t *complete_count, uint32_t *discarded_boundary_count) {
                                                                        double max_abs = 0.0;
                                                                        uint32_t complete = 0U;
                                                                        uint32_t discarded = 0U;
                                                                        if (complete_count != NULL) *complete_count = 0U;
                                                                        if (discarded_boundary_count != NULL)         *discarded_boundary_count = 0U;
                                                                        if (template_samples == NULL || events == NULL ||         count == 0U || max_events == 0U)         return -1;
                                                                        for (size_t i = 0U; i < count; ++i) {
                                                                            if (!isfinite(template_samples[i])) return -2;
                                                                            if (fabs(template_samples[i]) > max_abs)         max_abs = fabs(template_samples[i]);
                                                                        }
                                                                        if (max_abs <= DBL_EPSILON) return -3;
                                                                        for (size_t i = 0U; i < count;) {
                                                                            if (fabs(template_samples[i]) <             CAL_DITHER_EVENT_THRESHOLD_FRACTION * max_abs) {
                                                                                ++i;
                                                                                continue;
                                                                            }
                                                                            {
                                                                                const size_t start = i;
                                                                                double weighted_sum = 0.0;
                                                                                double weight_sum = 0.0;
                                                                                double signed_sum = 0.0;
                                                                                double peak_abs = 0.0;
                                                                                double peak_value = 0.0;
                                                                                while (i < count &&                    fabs(template_samples[i]) >=                        CAL_DITHER_EVENT_THRESHOLD_FRACTION * max_abs) {
                                                                                    const double sample = template_samples[i];
                                                                                    const double weight = fabs(sample);
                                                                                    weighted_sum += (double)i * weight;
                                                                                    weight_sum += weight;
                                                                                    signed_sum += sample;
                                                                                    if (weight > peak_abs) {
                                                                                        peak_abs = weight;
                                                                                        peak_value = sample;
                                                                                    }
                                                                                    ++i;
                                                                                }
                                                                                if (weight_sum <= DBL_EPSILON) continue;
                                                                                if (start == 0U || i >= count) {
                                                                                    ++discarded;
                                                                                    continue;
                                                                                }
                                                                                if (complete >= max_events) return -4;
                                                                                events[complete].start = start;
                                                                                events[complete].end = i;
                                                                                events[complete].center = weighted_sum / weight_sum;
                                                                                events[complete].polarity =         signed_sum >= 0.0 ? 1.0 : -1.0;
                                                                                if (fabs(signed_sum) <= DBL_EPSILON)         events[complete].polarity = peak_value >= 0.0 ? 1.0 : -1.0;
                                                                                ++complete;
                                                                            }
                                                                        }
                                                                        if (complete_count != NULL) *complete_count = complete;
                                                                        if (discarded_boundary_count != NULL)         *discarded_boundary_count = discarded;
                                                                        return 0;
                                                                    }
                                                                    static int calibration_estimate_dither_offset(     const calibration_aligned_frame_t *frame,     double existing_offset_codes,     calibration_dither_offset_diagnostic_t *diagnostic) {
                                                                        static double adc_samples[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double reference_samples[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double adc_tone[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double adc_residual[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double reference_tone[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double dither_template[CAL_FIXED_WINDOW_LENGTH];
                                                                        static calibration_dither_offset_event_t events[CAL_FIXED_WINDOW_LENGTH / 2U];
                                                                        static double u[CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES];
                                                                        static double v[CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES];
                                                                        static double template_profile[CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES];
                                                                        static double offset_profile[CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES];
                                                                        static double dither_profile[CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES];
                                                                        calibration_tone_fit_result_t adc_fit;
                                                                        calibration_tone_fit_result_t reference_fit;
                                                                        uint32_t complete_events = 0U;
                                                                        uint32_t discarded_boundary = 0U;
                                                                        double p_sum = 0.0;
                                                                        double p_mean;
                                                                        double denominator;
                                                                        double rel_start = -DBL_MAX;
                                                                        double rel_end = DBL_MAX;
                                                                        int m_first;
                                                                        int m_last;
                                                                        size_t profile_count;
                                                                        double max_template_abs = 0.0;
                                                                        uint32_t flat_count = 0U;
                                                                        double offset_sum = 0.0;
                                                                        double offset_square_sum = 0.0;
                                                                        double offset_min = DBL_MAX;
                                                                        double offset_max = -DBL_MAX;
                                                                        if (diagnostic == NULL) return -1;
                                                                        calibration_dither_offset_mark_invalid(         diagnostic, CAL_DITHER_OFFSET_REASON_NUMERICAL,         existing_offset_codes);
                                                                        if (frame == NULL || !frame->frame_valid ||         frame->selected_channel != g_stored_offset_reference.selected_channel ||         frame->canonical_reference_phase !=             g_stored_offset_reference.canonical_reference_phase ||         frame->calibration_window_start !=             g_stored_offset_reference.calibration_window_start ||         frame->calibration_window_length != CAL_FIXED_WINDOW_LENGTH ||         frame->valid_analysis_sample_count != CAL_FIXED_WINDOW_LENGTH ||         frame->aligned_corrected_adc_samples == NULL ||         frame->canonical_reference_window == NULL ||         !isfinite(frame->total_lag) ||         !isfinite(frame->reference_frequency_hz) ||         frame->reference_frequency_hz <= 0.0 ||         !frame->timing_diagnostics.valid ||         frame->timing_diagnostics.selected_common_channel < 0 ||         !isfinite(frame->timing_diagnostics.dither_derived_lag)) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_TIMING_CONTEXT;
                                                                            return -2;
                                                                        }
                                                                        diagnostic->timing_context_pass = 1U;
                                                                        for (size_t i = 0U; i < CAL_FIXED_WINDOW_LENGTH; ++i) {
                                                                            adc_samples[i] = frame->aligned_corrected_adc_samples[i];
                                                                            reference_samples[i] = frame->canonical_reference_window[i];
                                                                        }
                                                                        if (calibration_fit_tone_refined(         adc_samples, CAL_FIXED_WINDOW_LENGTH,         frame->reference_frequency_hz,         adc_get_effective_sample_rate_hz(), &adc_fit, adc_tone,         adc_residual) != 0 ||         !calibration_tone_fit_parameters_are_finite(&adc_fit) ||         adc_fit.rmse > CAL_TONE_VALIDATION_MAX_RMSE_CODES ||         adc_fit.tone_only_correlation <             CAL_TONE_VALIDATION_MIN_CORRELATION) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_TONE_FIT;
                                                                            return -3;
                                                                        }
                                                                        if (calibration_fit_tone_refined(         reference_samples, CAL_FIXED_WINDOW_LENGTH,         frame->reference_frequency_hz,         adc_get_effective_sample_rate_hz(), &reference_fit,         reference_tone, dither_template) != 0 ||         !calibration_tone_fit_parameters_are_finite(&reference_fit)) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_TONE_FIT;
                                                                            return -3;
                                                                        }
                                                                        diagnostic->tone_fit_pass = 1U;
                                                                        if (calibration_dither_offset_find_events(         dither_template, CAL_FIXED_WINDOW_LENGTH, events,         sizeof(events) / sizeof(events[0]), &complete_events,         &discarded_boundary) != 0) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_NO_DITHER;
                                                                            return -4;
                                                                        }
                                                                        diagnostic->complete_event_count = complete_events;
                                                                        diagnostic->discarded_boundary_event_count = discarded_boundary;
                                                                        if (complete_events < CAL_DITHER_OFFSET_MIN_COMPLETE_EVENTS) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_TOO_FEW_EVENTS;
                                                                            return -5;
                                                                        }
                                                                        diagnostic->event_count_pass = 1U;
                                                                        for (uint32_t k = 0U; k < complete_events; ++k) {
                                                                            const double left = (double)events[k].start - events[k].center;
                                                                            const double right =             (double)(events[k].end - 1U) - events[k].center;
                                                                            if (left > rel_start) rel_start = left;
                                                                            if (right < rel_end) rel_end = right;
                                                                            p_sum += events[k].polarity;
                                                                        }
                                                                        m_first = (int)ceil(rel_start);
                                                                        m_last = (int)floor(rel_end);
                                                                        if (m_last < m_first) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_EVENT_PROFILE;
                                                                            return -6;
                                                                        }
                                                                        profile_count = (size_t)(m_last - m_first + 1);
                                                                        if (profile_count == 0U ||         profile_count > CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_EVENT_PROFILE;
                                                                            return -6;
                                                                        }
                                                                        memset(u, 0, profile_count * sizeof(u[0]));
                                                                        memset(v, 0, profile_count * sizeof(v[0]));
                                                                        memset(template_profile, 0,                 profile_count * sizeof(template_profile[0]));
                                                                        for (uint32_t k = 0U; k < complete_events; ++k) {
                                                                            const double p = events[k].polarity;
                                                                            for (size_t j = 0U; j < profile_count; ++j) {
                                                                                const double position =             events[k].center + (double)(m_first + (int)j);
                                                                                double residual_value;
                                                                                double template_value;
                                                                                if (calibration_dither_offset_interpolate(                     adc_residual, CAL_FIXED_WINDOW_LENGTH, position,                     &residual_value) != 0 ||                 calibration_dither_offset_interpolate(                     dither_template, CAL_FIXED_WINDOW_LENGTH, position,                     &template_value) != 0) {
                                                                                    diagnostic->reason =                 CAL_DITHER_OFFSET_REASON_INTERPOLATION;
                                                                                    return -7;
                                                                                }
                                                                                u[j] += residual_value;
                                                                                v[j] += p * residual_value;
                                                                                template_profile[j] += p * template_value;
                                                                            }
                                                                        }
                                                                        p_mean = p_sum / (double)complete_events;
                                                                        denominator = 1.0 - p_mean * p_mean;
                                                                        diagnostic->mean_event_polarity = p_mean;
                                                                        diagnostic->separation_denominator = denominator;
                                                                        if (!isfinite(denominator) ||         denominator <= CAL_DITHER_OFFSET_DENOMINATOR_FLOOR) {
                                                                            diagnostic->reason =             CAL_DITHER_OFFSET_REASON_POLARITY_IMBALANCE;
                                                                            return -8;
                                                                        }
                                                                        diagnostic->polarity_balance_pass = 1U;
                                                                        for (size_t j = 0U; j < profile_count; ++j) {
                                                                            u[j] /= (double)complete_events;
                                                                            v[j] /= (double)complete_events;
                                                                            template_profile[j] /= (double)complete_events;
                                                                            offset_profile[j] = (u[j] - p_mean * v[j]) / denominator;
                                                                            dither_profile[j] = (v[j] - p_mean * u[j]) / denominator;
                                                                            if (!isfinite(offset_profile[j]) ||         !isfinite(dither_profile[j]) ||         !isfinite(template_profile[j])) {
                                                                                diagnostic->reason = CAL_DITHER_OFFSET_REASON_NUMERICAL;
                                                                                return -9;
                                                                            }
                                                                            if (fabs(template_profile[j]) > max_template_abs)         max_template_abs = fabs(template_profile[j]);
                                                                        }
                                                                        if (max_template_abs <= DBL_EPSILON) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_NO_DITHER;
                                                                            return -10;
                                                                        }
                                                                        for (size_t j = 0U; j < profile_count; ++j) {
                                                                            const double previous = j > 0U ?             template_profile[j - 1U] : template_profile[j];
                                                                            const double next = j + 1U < profile_count ?             template_profile[j + 1U] : template_profile[j];
                                                                            const double derivative = 0.5 * (next - previous);
                                                                            const bool flat =         fabs(derivative) <=             CAL_DITHER_OFFSET_FLAT_TOP_DERIVATIVE_FRACTION *                 max_template_abs &&         fabs(template_profile[j]) >=             CAL_DITHER_OFFSET_FLAT_TOP_AMPLITUDE_FRACTION *                 max_template_abs;
                                                                            if (!flat) continue;
                                                                            offset_sum += offset_profile[j];
                                                                            offset_square_sum += offset_profile[j] * offset_profile[j];
                                                                            if (offset_profile[j] < offset_min)         offset_min = offset_profile[j];
                                                                            if (offset_profile[j] > offset_max)         offset_max = offset_profile[j];
                                                                            ++flat_count;
                                                                        }
                                                                        diagnostic->flat_top_sample_count = flat_count;
                                                                        if (flat_count < CAL_DITHER_OFFSET_MIN_FLAT_TOP_SAMPLES) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_FLAT_TOP;
                                                                            return -11;
                                                                        }
                                                                        diagnostic->flat_top_pass = 1U;
                                                                        diagnostic->dither_offset_codes = offset_sum / (double)flat_count;
                                                                        diagnostic->offset_profile_std_codes = sqrt(fmax(0.0,         offset_square_sum / (double)flat_count -         diagnostic->dither_offset_codes * diagnostic->dither_offset_codes));
                                                                        diagnostic->offset_profile_min_codes = offset_min;
                                                                        diagnostic->offset_profile_max_codes = offset_max;
                                                                        diagnostic->existing_offset_codes = existing_offset_codes;
                                                                        diagnostic->fitted_tone_dc_codes = adc_fit.dc_offset_codes;
                                                                        diagnostic->existing_vs_dither_codes =         existing_offset_codes - diagnostic->dither_offset_codes;
                                                                        diagnostic->dither_vs_fitted_dc_codes =         diagnostic->dither_offset_codes - diagnostic->fitted_tone_dc_codes;
                                                                        diagnostic->tone = adc_fit;
                                                                        diagnostic->fitted_tone_frequency_hz = adc_fit.fitted_frequency_hz;
                                                                        diagnostic->fitted_tone_amplitude_codes = adc_fit.amplitude;
                                                                        diagnostic->fitted_tone_phase_rad = adc_fit.phase_rad;
                                                                        diagnostic->tone_fit_rmse_codes = adc_fit.rmse;
                                                                        diagnostic->tone_only_correlation = adc_fit.tone_only_correlation;
                                                                        diagnostic->estimate_consistency_pass =         fabs(diagnostic->existing_vs_dither_codes) <=             CAL_DITHER_OFFSET_AGREEMENT_TOLERANCE_CODES;
                                                                        diagnostic->numerical_pass =         isfinite(diagnostic->dither_offset_codes) &&         isfinite(diagnostic->existing_vs_dither_codes) &&         isfinite(diagnostic->dither_vs_fitted_dc_codes) &&         isfinite(diagnostic->offset_profile_std_codes);
                                                                        if (!diagnostic->numerical_pass) {
                                                                            diagnostic->reason = CAL_DITHER_OFFSET_REASON_NUMERICAL;
                                                                            return -12;
                                                                        }
                                                                        diagnostic->valid = 1U;
                                                                        diagnostic->reason = CAL_DITHER_OFFSET_REASON_NONE;
                                                                        diagnostic->status =         diagnostic->estimate_consistency_pass ?         CAL_DITHER_OFFSET_STATUS_PASS :         CAL_DITHER_OFFSET_STATUS_WARNING;
                                                                        return 0;
                                                                    }
                                                                    static void calibration_print_dither_offset_diagnostic(     const calibration_dither_offset_diagnostic_t *diagnostic) {
                                                                        if (diagnostic == NULL ||         (diagnostic->status == CAL_DITHER_OFFSET_STATUS_INVALID &&          diagnostic->reason == CAL_DITHER_OFFSET_REASON_NONE))         return;
                                                                        xil_printf("\r\n-----------------------------------------\r\n");
                                                                        xil_printf("Dither-Aware Offset Diagnostic\r\n");
                                                                        xil_printf("-----------------------------------------\r\n");
                                                                        xil_printf("Complete events          : %lu\r\n",         (unsigned long)diagnostic->complete_event_count);
                                                                        xil_printf("Discarded boundary events: %lu\r\n",         (unsigned long)diagnostic->discarded_boundary_event_count);
                                                                        print_double_value("Mean event polarity",         diagnostic->mean_event_polarity, "");
                                                                        print_double_value("Separation denominator",         diagnostic->separation_denominator, "");
                                                                        xil_printf("Flat-top samples         : %lu\r\n",         (unsigned long)diagnostic->flat_top_sample_count);
                                                                        print_double_value("Existing residual",         diagnostic->existing_offset_codes, " codes");
                                                                        if (diagnostic->valid) {
                                                                            print_double_value("Dither offset",             diagnostic->dither_offset_codes, " codes");
                                                                            print_double_value("Fitted-tone DC",             diagnostic->fitted_tone_dc_codes, " codes");
                                                                            print_double_value("Existing-dither delta",             diagnostic->existing_vs_dither_codes, " codes");
                                                                            print_double_value("Dither-DC delta",             diagnostic->dither_vs_fitted_dc_codes, " codes");
                                                                            print_double_value("Offset-profile deviation",             diagnostic->offset_profile_std_codes, " codes");
                                                                        }
                                                                        else {
                                                                            xil_printf("Dither offset            : INVALID\r\n");
                                                                            xil_printf("Rejection reason         : %s\r\n",         calibration_dither_offset_reason_name(diagnostic->reason));
                                                                        }
                                                                        print_double_value("Tone fit RMSE",         diagnostic->tone_fit_rmse_codes, " codes");
                                                                        print_double_value("Tone-only correlation",         diagnostic->tone_only_correlation, "");
                                                                        xil_printf("Estimate units           : %s\r\n",         diagnostic->units != NULL ? diagnostic->units :         "corrected ADC codes");
                                                                        xil_printf("New estimator status     : %s\r\n",         calibration_dither_offset_status_name(diagnostic->status));
                                                                    }
                                                                    static calibration_dither_gain_diagnostic_t g_latest_dither_gain_diagnostic;
                                                                    static const char *calibration_dither_gain_status_name(     calibration_dither_gain_status_t status) {
                                                                        switch (status) {
                                                                        case CAL_DITHER_GAIN_STATUS_PASS:
                                                                            return "PASS";
                                                                        case CAL_DITHER_GAIN_STATUS_WARNING:
                                                                            return "WARNING";
                                                                        case CAL_DITHER_GAIN_STATUS_INVALID:
                                                                        default:
                                                                            return "INVALID";
                                                                        }
                                                                    }
                                                                    static const char *calibration_dither_gain_reason_name(     calibration_dither_gain_reason_t reason) {
                                                                        switch (reason) {
                                                                        case CAL_DITHER_GAIN_REASON_NONE:
                                                                            return "NONE";
                                                                        case CAL_DITHER_GAIN_REASON_CONTEXT:
                                                                            return "CONTEXT";
                                                                        case CAL_DITHER_GAIN_REASON_TONE_FIT:
                                                                            return "TONE_FIT";
                                                                        case CAL_DITHER_GAIN_REASON_TOO_FEW_EVENTS:
                                                                            return "TOO_FEW_EVENTS";
                                                                        case CAL_DITHER_GAIN_REASON_POLARITY_IMBALANCE:
                                                                            return "POLARITY_IMBALANCE";
                                                                        case CAL_DITHER_GAIN_REASON_TEMPLATE:
                                                                            return "TEMPLATE";
                                                                        case CAL_DITHER_GAIN_REASON_GAIN_VALUE:
                                                                            return "GAIN_VALUE";
                                                                        case CAL_DITHER_GAIN_REASON_FIT_QUALITY:
                                                                            return "FIT_QUALITY";
                                                                        case CAL_DITHER_GAIN_REASON_INTERPOLATION:
                                                                            return "INTERPOLATION";
                                                                        case CAL_DITHER_GAIN_REASON_NUMERICAL:
                                                                            return "NUMERICAL";
                                                                        case CAL_DITHER_GAIN_REASON_NO_DITHER:
                                                                            return "NO_DITHER";
                                                                        case CAL_DITHER_GAIN_REASON_EVENT_PROFILE:
                                                                        default:
                                                                            return "EVENT_PROFILE";
                                                                        }
                                                                    }
                                                                    static void calibration_dither_gain_mark_invalid(     calibration_dither_gain_diagnostic_t *diagnostic,     calibration_dither_gain_reason_t reason,     double existing_normalized_gain) {
                                                                        if (diagnostic == NULL) return;
                                                                        memset(diagnostic, 0, sizeof(*diagnostic));
                                                                        diagnostic->status = CAL_DITHER_GAIN_STATUS_INVALID;
                                                                        diagnostic->reason = reason;
                                                                        diagnostic->existing_normalized_gain = existing_normalized_gain;
                                                                        diagnostic->dither_full_gain = NAN;
                                                                        diagnostic->dither_flat_gain = NAN;
                                                                        diagnostic->requested_dither_correction = NAN;
                                                                        diagnostic->tone_amplitude_codes = NAN;
                                                                        diagnostic->existing_vs_dither_gain = NAN;
                                                                        diagnostic->full_vs_flat_gain = NAN;
                                                                        diagnostic->template_energy = NAN;
                                                                        diagnostic->template_correlation = NAN;
                                                                        diagnostic->fit_rmse = NAN;
                                                                        diagnostic->normalized_fit_rmse = NAN;
                                                                        diagnostic->peak_residual = NAN;
                                                                        diagnostic->mean_event_polarity = NAN;
                                                                        diagnostic->separation_denominator = NAN;
                                                                        diagnostic->fitted_tone_frequency_hz = NAN;
                                                                        diagnostic->tone_fit_rmse_codes = NAN;
                                                                        diagnostic->tone_only_correlation = NAN;
                                                                        diagnostic->gain_definition = "normalized measured dither gain";
                                                                    }
                                                                    static int calibration_estimate_dither_gain_diagnostic(     const calibration_aligned_frame_t *frame,     double existing_normalized_gain,     double nominal_system_gain,     calibration_dither_gain_diagnostic_t *diagnostic) {
                                                                        static double adc_samples[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double reference_samples[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double adc_tone[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double adc_residual[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double reference_tone[CAL_FIXED_WINDOW_LENGTH];
                                                                        static double dither_template[CAL_FIXED_WINDOW_LENGTH];
                                                                        static calibration_dither_offset_event_t events[CAL_FIXED_WINDOW_LENGTH / 2U];
                                                                        static double u[CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES];
                                                                        static double v[CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES];
                                                                        static double template_profile[CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES];
                                                                        static double dither_profile[CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES];
                                                                        calibration_tone_fit_result_t adc_fit;
                                                                        calibration_tone_fit_result_t reference_fit;
                                                                        uint32_t complete_events = 0U;
                                                                        uint32_t discarded_boundary = 0U;
                                                                        double p_sum = 0.0;
                                                                        double p_mean;
                                                                        double denominator;
                                                                        double rel_start = -DBL_MAX;
                                                                        double rel_end = DBL_MAX;
                                                                        int m_first;
                                                                        int m_last;
                                                                        size_t profile_count;
                                                                        double template_energy = 0.0;
                                                                        double measured_energy = 0.0;
                                                                        double projection = 0.0;
                                                                        double fit_error_sum = 0.0;
                                                                        double peak_residual = 0.0;
                                                                        double template_sum = 0.0;
                                                                        double profile_sum = 0.0;
                                                                        double template_centered_energy = 0.0;
                                                                        double profile_centered_energy = 0.0;
                                                                        double centered_cross = 0.0;
                                                                        double max_template_abs = 0.0;
                                                                        double flat_projection = 0.0;
                                                                        double flat_template_energy = 0.0;
                                                                        uint32_t flat_count = 0U;
                                                                        double raw_dither_gain;
                                                                        double raw_flat_gain = NAN;
                                                                        if (diagnostic == NULL) return -1;
                                                                        calibration_dither_gain_mark_invalid(         diagnostic, CAL_DITHER_GAIN_REASON_NUMERICAL,         existing_normalized_gain);
                                                                        if (frame == NULL || !frame->frame_valid ||         frame->selected_channel != g_stored_offset_reference.selected_channel ||         frame->canonical_reference_phase !=             g_stored_offset_reference.canonical_reference_phase ||         frame->calibration_window_start !=             g_stored_offset_reference.calibration_window_start ||         frame->calibration_window_length != CAL_FIXED_WINDOW_LENGTH ||         frame->valid_analysis_sample_count != CAL_FIXED_WINDOW_LENGTH ||         frame->aligned_corrected_adc_samples == NULL ||         frame->aligned_reference_samples == NULL ||         !isfinite(frame->total_lag) ||         !isfinite(frame->reference_frequency_hz) ||         frame->reference_frequency_hz <= 0.0 ||         !isfinite(nominal_system_gain) ||         fabs(nominal_system_gain) <= DBL_EPSILON ||         !frame->timing_diagnostics.valid ||         frame->timing_diagnostics.selected_common_channel < 0 ||         !isfinite(frame->timing_diagnostics.dither_derived_lag)) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_CONTEXT;
                                                                            return -2;
                                                                        }
                                                                        diagnostic->context_pass = 1U;
                                                                        for (size_t i = 0U; i < CAL_FIXED_WINDOW_LENGTH; ++i) {
                                                                            adc_samples[i] = frame->aligned_corrected_adc_samples[i];
                                                                            reference_samples[i] = frame->aligned_reference_samples[i];
                                                                        }
                                                                        if (calibration_fit_tone_refined(         adc_samples, CAL_FIXED_WINDOW_LENGTH,         frame->reference_frequency_hz,         adc_get_effective_sample_rate_hz(), &adc_fit, adc_tone,         adc_residual) != 0 ||         !calibration_tone_fit_parameters_are_finite(&adc_fit) ||         adc_fit.rmse > CAL_TONE_VALIDATION_MAX_RMSE_CODES ||         adc_fit.tone_only_correlation <             CAL_TONE_VALIDATION_MIN_CORRELATION) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_TONE_FIT;
                                                                            return -3;
                                                                        }
                                                                        if (calibration_fit_tone_refined(         reference_samples, CAL_FIXED_WINDOW_LENGTH,         frame->reference_frequency_hz,         adc_get_effective_sample_rate_hz(), &reference_fit,         reference_tone, dither_template) != 0 ||         !calibration_tone_fit_parameters_are_finite(&reference_fit)) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_TONE_FIT;
                                                                            return -3;
                                                                        }
                                                                        diagnostic->tone_fit_pass = 1U;
                                                                        if (calibration_dither_offset_find_events(         dither_template, CAL_FIXED_WINDOW_LENGTH, events,         sizeof(events) / sizeof(events[0]), &complete_events,         &discarded_boundary) != 0) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_NO_DITHER;
                                                                            return -4;
                                                                        }
                                                                        diagnostic->complete_event_count = complete_events;
                                                                        diagnostic->discarded_boundary_event_count = discarded_boundary;
                                                                        if (complete_events < CAL_DITHER_GAIN_MIN_COMPLETE_EVENTS) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_TOO_FEW_EVENTS;
                                                                            return -5;
                                                                        }
                                                                        diagnostic->event_count_pass = 1U;
                                                                        for (uint32_t k = 0U; k < complete_events; ++k) {
                                                                            const double left = (double)events[k].start - events[k].center;
                                                                            const double right =             (double)(events[k].end - 1U) - events[k].center;
                                                                            if (left > rel_start) rel_start = left;
                                                                            if (right < rel_end) rel_end = right;
                                                                            p_sum += events[k].polarity;
                                                                        }
                                                                        m_first = (int)ceil(rel_start);
                                                                        m_last = (int)floor(rel_end);
                                                                        if (m_last < m_first) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_EVENT_PROFILE;
                                                                            return -6;
                                                                        }
                                                                        profile_count = (size_t)(m_last - m_first + 1);
                                                                        if (profile_count == 0U ||         profile_count > CAL_DITHER_OFFSET_MAX_EVENT_PROFILE_SAMPLES) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_EVENT_PROFILE;
                                                                            return -6;
                                                                        }
                                                                        memset(u, 0, profile_count * sizeof(u[0]));
                                                                        memset(v, 0, profile_count * sizeof(v[0]));
                                                                        memset(template_profile, 0,                 profile_count * sizeof(template_profile[0]));
                                                                        for (uint32_t k = 0U; k < complete_events; ++k) {
                                                                            const double p = events[k].polarity;
                                                                            for (size_t j = 0U; j < profile_count; ++j) {
                                                                                const double position =             events[k].center + (double)(m_first + (int)j);
                                                                                double residual_value;
                                                                                double template_value;
                                                                                if (calibration_dither_offset_interpolate(                     adc_residual, CAL_FIXED_WINDOW_LENGTH, position,                     &residual_value) != 0 ||                 calibration_dither_offset_interpolate(                     dither_template, CAL_FIXED_WINDOW_LENGTH, position,                     &template_value) != 0) {
                                                                                    diagnostic->reason = CAL_DITHER_GAIN_REASON_INTERPOLATION;
                                                                                    return -7;
                                                                                }
                                                                                u[j] += residual_value;
                                                                                v[j] += p * residual_value;
                                                                                template_profile[j] += p * template_value;
                                                                            }
                                                                        }
                                                                        p_mean = p_sum / (double)complete_events;
                                                                        denominator = 1.0 - p_mean * p_mean;
                                                                        diagnostic->mean_event_polarity = p_mean;
                                                                        diagnostic->separation_denominator = denominator;
                                                                        if (!isfinite(denominator) ||         denominator <= CAL_DITHER_GAIN_DENOMINATOR_FLOOR) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_POLARITY_IMBALANCE;
                                                                            return -8;
                                                                        }
                                                                        diagnostic->polarity_pass = 1U;
                                                                        for (size_t j = 0U; j < profile_count; ++j) {
                                                                            u[j] /= (double)complete_events;
                                                                            v[j] /= (double)complete_events;
                                                                            template_profile[j] /= (double)complete_events;
                                                                            dither_profile[j] = (v[j] - p_mean * u[j]) / denominator;
                                                                            if (!isfinite(dither_profile[j]) ||         !isfinite(template_profile[j])) {
                                                                                diagnostic->reason = CAL_DITHER_GAIN_REASON_NUMERICAL;
                                                                                return -9;
                                                                            }
                                                                            projection += dither_profile[j] * template_profile[j];
                                                                            template_energy += template_profile[j] * template_profile[j];
                                                                            measured_energy += dither_profile[j] * dither_profile[j];
                                                                            template_sum += template_profile[j];
                                                                            profile_sum += dither_profile[j];
                                                                            if (fabs(template_profile[j]) > max_template_abs)         max_template_abs = fabs(template_profile[j]);
                                                                        }
                                                                        diagnostic->template_energy = template_energy;
                                                                        if (!isfinite(template_energy) ||         template_energy <= CAL_DITHER_GAIN_TEMPLATE_ENERGY_FLOOR) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_TEMPLATE;
                                                                            return -10;
                                                                        }
                                                                        diagnostic->template_pass = 1U;
                                                                        raw_dither_gain = projection / template_energy;
                                                                        diagnostic->dither_full_gain = raw_dither_gain / nominal_system_gain;
                                                                        if (diagnostic->dither_full_gain >             CAL_DITHER_GAIN_MIN_POSITIVE_GAIN) {
                                                                            diagnostic->requested_dither_correction =             1.0 / diagnostic->dither_full_gain;
                                                                        }
                                                                        if (!isfinite(diagnostic->dither_full_gain) ||         diagnostic->dither_full_gain <=             CAL_DITHER_GAIN_MIN_POSITIVE_GAIN ||         diagnostic->dither_full_gain <             CAL_DITHER_GAIN_MIN_PLAUSIBLE_GAIN ||         diagnostic->dither_full_gain >             CAL_DITHER_GAIN_MAX_PLAUSIBLE_GAIN ||         !isfinite(diagnostic->requested_dither_correction)) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_GAIN_VALUE;
                                                                            return -11;
                                                                        }
                                                                        diagnostic->gain_value_pass = 1U;
                                                                        template_sum /= (double)profile_count;
                                                                        profile_sum /= (double)profile_count;
                                                                        for (size_t j = 0U; j < profile_count; ++j) {
                                                                            const double fit = raw_dither_gain * template_profile[j];
                                                                            const double residual = dither_profile[j] - fit;
                                                                            const double x = template_profile[j] - template_sum;
                                                                            const double y = dither_profile[j] - profile_sum;
                                                                            fit_error_sum += residual * residual;
                                                                            if (fabs(residual) > peak_residual) peak_residual = fabs(residual);
                                                                            template_centered_energy += x * x;
                                                                            profile_centered_energy += y * y;
                                                                            centered_cross += x * y;
                                                                        }
                                                                        diagnostic->fit_rmse = sqrt(fit_error_sum / (double)profile_count);
                                                                        diagnostic->normalized_fit_rmse =         diagnostic->fit_rmse / fmax(sqrt(measured_energy /             (double)profile_count), DBL_EPSILON);
                                                                        diagnostic->peak_residual = peak_residual;
                                                                        diagnostic->template_correlation =         template_centered_energy > DBL_EPSILON &&         profile_centered_energy > DBL_EPSILON ?         centered_cross / sqrt(template_centered_energy *             profile_centered_energy) : NAN;
                                                                        for (size_t j = 0U; j < profile_count; ++j) {
                                                                            const double previous = j > 0U ?             template_profile[j - 1U] : template_profile[j];
                                                                            const double next = j + 1U < profile_count ?             template_profile[j + 1U] : template_profile[j];
                                                                            const double derivative = 0.5 * (next - previous);
                                                                            const bool flat =         fabs(derivative) <=             CAL_DITHER_OFFSET_FLAT_TOP_DERIVATIVE_FRACTION *                 max_template_abs &&         fabs(template_profile[j]) >=             CAL_DITHER_OFFSET_FLAT_TOP_AMPLITUDE_FRACTION *                 max_template_abs;
                                                                            if (!flat) continue;
                                                                            flat_projection += dither_profile[j] * template_profile[j];
                                                                            flat_template_energy += template_profile[j] * template_profile[j];
                                                                            ++flat_count;
                                                                        }
                                                                        diagnostic->flat_top_sample_count = flat_count;
                                                                        if (flat_count < CAL_DITHER_OFFSET_MIN_FLAT_TOP_SAMPLES ||         flat_template_energy <= CAL_DITHER_GAIN_TEMPLATE_ENERGY_FLOOR) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_TEMPLATE;
                                                                            return -12;
                                                                        }
                                                                        raw_flat_gain = flat_projection / flat_template_energy;
                                                                        diagnostic->dither_flat_gain = raw_flat_gain / nominal_system_gain;
                                                                        diagnostic->full_vs_flat_gain =         diagnostic->dither_full_gain - diagnostic->dither_flat_gain;
                                                                        diagnostic->fit_quality_pass =         isfinite(diagnostic->template_correlation) &&         diagnostic->template_correlation >=             CAL_DITHER_GAIN_MIN_TEMPLATE_CORRELATION &&         isfinite(diagnostic->normalized_fit_rmse) &&         diagnostic->normalized_fit_rmse <=             CAL_DITHER_GAIN_MAX_NORMALIZED_FIT_RMSE &&         isfinite(diagnostic->full_vs_flat_gain) &&         fabs(diagnostic->full_vs_flat_gain) <=             CAL_DITHER_GAIN_FULL_FLAT_TOLERANCE;
                                                                        if (!diagnostic->fit_quality_pass) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_FIT_QUALITY;
                                                                            return -13;
                                                                        }
                                                                        diagnostic->existing_normalized_gain = existing_normalized_gain;
                                                                        diagnostic->tone = adc_fit;
                                                                        diagnostic->tone_amplitude_codes = adc_fit.amplitude;
                                                                        diagnostic->existing_vs_dither_gain =         existing_normalized_gain - diagnostic->dither_full_gain;
                                                                        diagnostic->fitted_tone_frequency_hz = adc_fit.fitted_frequency_hz;
                                                                        diagnostic->tone_fit_rmse_codes = adc_fit.rmse;
                                                                        diagnostic->tone_only_correlation = adc_fit.tone_only_correlation;
                                                                        diagnostic->agreement_pass =         isfinite(diagnostic->existing_vs_dither_gain) &&         fabs(diagnostic->existing_vs_dither_gain) <=             CAL_DITHER_GAIN_AGREEMENT_TOLERANCE;
                                                                        diagnostic->numerical_pass =         isfinite(diagnostic->dither_full_gain) &&         isfinite(diagnostic->dither_flat_gain) &&         isfinite(diagnostic->requested_dither_correction) &&         isfinite(diagnostic->template_correlation) &&         isfinite(diagnostic->normalized_fit_rmse);
                                                                        if (!diagnostic->numerical_pass) {
                                                                            diagnostic->reason = CAL_DITHER_GAIN_REASON_NUMERICAL;
                                                                            return -14;
                                                                        }
                                                                        diagnostic->valid = 1U;
                                                                        diagnostic->reason = CAL_DITHER_GAIN_REASON_NONE;
                                                                        diagnostic->status =         diagnostic->agreement_pass ? CAL_DITHER_GAIN_STATUS_PASS :         CAL_DITHER_GAIN_STATUS_WARNING;
                                                                        return 0;
                                                                    }
                                                                    static void calibration_print_dither_gain_diagnostic(     const calibration_dither_gain_diagnostic_t *diagnostic) {
                                                                        if (diagnostic == NULL ||         (diagnostic->status == CAL_DITHER_GAIN_STATUS_INVALID &&          diagnostic->reason == CAL_DITHER_GAIN_REASON_NONE))         return;
                                                                        xil_printf("\r\n-----------------------------------------\r\n");
                                                                        xil_printf("Dither-Aware Gain Diagnostic\r\n");
                                                                        xil_printf("-----------------------------------------\r\n");
                                                                        xil_printf("Complete events          : %lu\r\n",         (unsigned long)diagnostic->complete_event_count);
                                                                        print_double_value("Mean event polarity",         diagnostic->mean_event_polarity, "");
                                                                        print_double_value("Separation denominator",         diagnostic->separation_denominator, "");
                                                                        print_double_value("Template energy",         diagnostic->template_energy, "");
                                                                        xil_printf("Flat-top samples         : %lu\r\n",         (unsigned long)diagnostic->flat_top_sample_count);
                                                                        print_double_value("Existing measured gain",         diagnostic->existing_normalized_gain, "");
                                                                        if (diagnostic->valid) {
                                                                            print_double_value("Dither full-template gain",             diagnostic->dither_full_gain, "");
                                                                            print_double_value("Dither flat-top gain",             diagnostic->dither_flat_gain, "");
                                                                            print_double_value("Requested dither corr.",             diagnostic->requested_dither_correction, "");
                                                                            print_double_value("Template correlation",             diagnostic->template_correlation, "");
                                                                            print_double_value("Normalized fit RMSE",             diagnostic->normalized_fit_rmse, "");
                                                                            print_double_value("Existing-dither delta",             diagnostic->existing_vs_dither_gain, "");
                                                                        }
                                                                        else {
                                                                            xil_printf("Dither full-template gain: INVALID\r\n");
                                                                            xil_printf("Rejection reason         : %s\r\n",         calibration_dither_gain_reason_name(diagnostic->reason));
                                                                        }
                                                                        print_double_value("Tone-fit amplitude",         diagnostic->tone_amplitude_codes, " codes");
                                                                        print_double_value("Tone fit RMSE",         diagnostic->tone_fit_rmse_codes, " codes");
                                                                        xil_printf("Gain definition          : %s\r\n",         diagnostic->gain_definition != NULL ?         diagnostic->gain_definition : "normalized measured dither gain");
                                                                        xil_printf("Dither estimator status  : %s\r\n",         calibration_dither_gain_status_name(diagnostic->status));
                                                                    }
                                                                    static void calibration_run_timing_alignment_diagnostic(uint32_t frame_count) {
                                                                        static int16_t even_reference[ADC_CHANNEL_SAMPLE_COUNT];
                                                                        static int16_t odd_reference[ADC_CHANNEL_SAMPLE_COUNT];
                                                                        static int16_t channel_a[ADC_CHANNEL_SAMPLE_COUNT];
                                                                        static int16_t channel_b[ADC_CHANNEL_SAMPLE_COUNT];
                                                                        static int16_t fractional_reference[ADC_CHANNEL_SAMPLE_COUNT];
                                                                        static int16_t fractional_measurement[ADC_CHANNEL_SAMPLE_COUNT];
                                                                        size_t reference_count = 0U;
                                                                        size_t reconstructed_count = 0U;
                                                                        double even_variance = 0.0;
                                                                        double odd_variance = 0.0;
                                                                        const bool previous_quiet_capture = g_quiet_calibration_capture;
                                                                        if (frame_count == 0U || frame_count > ADC_CAL_MAX_FRAMES) {
                                                                            ERR("Diagnostic frame count must be between 1 and %u.",             ADC_CAL_MAX_FRAMES);
                                                                            return;
                                                                        }
                                                                        if (adc_sweep_active) {
                                                                            ERR("Another automatic ADC capture is already in progress.");
                                                                            return;
                                                                        }
                                                                        print_adc_analysis_rate_header();
                                                                        if (calibration_prepare_uploaded_dac_reference(             even_reference, odd_reference, &reference_count,             &even_variance, &odd_variance, 1) != 0)         return;
                                                                        adc_sweep_active = 1U;
                                                                        g_quiet_calibration_capture = true;
                                                                        xil_printf("\r\nADC timing diagnostic mode\r\n");
                                                                        xil_printf("Requested frames        : %lu\r\n",         (unsigned long)frame_count);
                                                                        for (uint32_t frame_index = 1U;
                                                                        frame_index <= frame_count;
                                                                        ++frame_index) {
                                                                            adc_reference_analysis_t analysis;
                                                                            calibration_aligned_frame_t frame;
                                                                            const int16_t *selected_reference = NULL;
                                                                            int analysis_status;
                                                                            int selected_channel = -1;
                                                                            int selected_phase = -1;
                                                                            size_t window_start =             reference_count > CAL_FIXED_WINDOW_LENGTH ?             (reference_count - CAL_FIXED_WINDOW_LENGTH) / 2U : 0U;
                                                                            if (frame_index > 1U) usleep(ADC_TIMING_INTERFRAME_DELAY_US);
                                                                            memset(&frame, 0, sizeof(frame));
                                                                            frame.selected_channel = -1;
                                                                            frame.selected_reference_phase = -1;
                                                                            frame.canonical_reference_phase = -1;
                                                                            frame.selected_channel_name = "none";
                                                                            frame.selected_phase_name = "none";
                                                                            frame.rejection_reason = "diagnostic capture not evaluated";
                                                                            xil_printf("\r\n---------- Diagnostic frame %lu ----------\r\n",             (unsigned long)frame_index);
                                                                            if (adc_capture_frame() != XST_SUCCESS) {
                                                                                frame.rejection_reason = "DMA capture failed";
                                                                                calibration_validate_timing_alignment(                 &frame, &frame.timing_diagnostics);
                                                                                calibration_print_timing_diagnostics_detail(                 &frame, &frame.timing_diagnostics);
                                                                                continue;
                                                                            }
                                                                            frame.capture_succeeded = true;
                                                                            if (adc_reconstruct_channels(RxBufferPtr, DMA_CMD_BUF_SIZE,                 channel_a, ADC_CHANNEL_SAMPLE_COUNT,                 channel_b, ADC_CHANNEL_SAMPLE_COUNT,                 &reconstructed_count) != 0 ||             reconstructed_count != reference_count) {
                                                                                frame.rejection_reason = "sample reconstruction failed";
                                                                                calibration_validate_timing_alignment(                 &frame, &frame.timing_diagnostics);
                                                                                calibration_print_timing_diagnostics_detail(                 &frame, &frame.timing_diagnostics);
                                                                                continue;
                                                                            }
                                                                            frame.reconstruction_succeeded = true;
                                                                            analysis_status = calibration_analyze_reference_frame(             even_reference, odd_reference, channel_a, channel_b,             reconstructed_count, fractional_reference,             fractional_measurement, calibration_channel_selection(),             &analysis);
                                                                            if (analysis.selected_adc == channel_a) selected_channel = 0;
                                                                            else if (analysis.selected_adc == channel_b) selected_channel = 1;
                                                                            if (analysis.selected_reference == even_reference) {
                                                                                selected_phase = 0;
                                                                                selected_reference = even_reference;
                                                                            }
                                                                            else if (analysis.selected_reference == odd_reference) {
                                                                                selected_phase = 1;
                                                                                selected_reference = odd_reference;
                                                                            }
                                                                            frame.frame_valid = analysis_status == 0;
                                                                            frame.selected_channel = selected_channel;
                                                                            frame.selected_reference_phase = selected_phase;
                                                                            frame.canonical_reference_phase = selected_phase;
                                                                            frame.selected_channel_name =             analysis.selected_channel_name != NULL ?             analysis.selected_channel_name : "none";
                                                                            frame.selected_phase_name =             analysis.selected_phase_name != NULL ?             analysis.selected_phase_name : "none";
                                                                            frame.integer_lag = analysis.timing.integer_lag;
                                                                            frame.fractional_lag = analysis.timing.fractional_lag;
                                                                            frame.total_lag = analysis.timing.total_lag;
                                                                            frame.correlation = analysis.timing.correlation;
                                                                            frame.reference_frequency_hz = analysis.reference_frequency_hz;
                                                                            frame.adc_frequency_hz = analysis.adc_frequency_hz;
                                                                            frame.timing = analysis.timing;
                                                                            frame.alignment_reference_count = reconstructed_count;
                                                                            frame.calibration_window_start = window_start;
                                                                            frame.calibration_window_length =             reference_count >= CAL_FIXED_WINDOW_LENGTH ?             CAL_FIXED_WINDOW_LENGTH : reference_count;
                                                                            frame.valid_analysis_sample_count = frame.calibration_window_length;
                                                                            frame.rejection_reason =             analysis_status == 0 ? "none" :             analysis.failure_reason != NULL ?                 analysis.failure_reason : "legacy timing rejected frame";
                                                                            if (selected_reference != NULL && selected_channel >= 0 &&             frame.calibration_window_length >= CAL_MIN_ANALYSIS_SAMPLES) {
                                                                                (void)calibration_compute_timing_diagnostics(                 selected_reference, channel_a, channel_b,                 reconstructed_count, selected_channel,                 analysis.reference_frequency_hz,                 adc_get_effective_sample_rate_hz(),                 (double)analysis.timing.total_lag,                 frame.calibration_window_start,                 frame.calibration_window_length,                 &frame.timing_diagnostics);
                                                                            }
                                                                            else {
                                                                                frame.timing_diagnostics.status_text =                 "legacy timing did not identify a reference/channel";
                                                                            }
                                                                            calibration_validate_timing_alignment(             &frame, &frame.timing_diagnostics);
                                                                            calibration_print_timing_diagnostics_detail(             &frame, &frame.timing_diagnostics);
                                                                        }
                                                                        g_quiet_calibration_capture = previous_quiet_capture;
                                                                        adc_sweep_active = 0U;
                                                                        xil_printf("\r\nTiming diagnostic mode complete.\r\n");
                                                                        xil_printf("No offset, gain, skew, or persistent timing state was updated.\r\n");
                                                                    }
                                                                    static int calibration_search_fixed_window_lag(     const int16_t *canonical_reference,     const int16_t *measurement,     size_t sample_count,     size_t window_start,     size_t window_length,     int32_t expected_lag,     calibration_timing_frame_result_t *result) {
                                                                        timing_alignment_result_t raw;
                                                                        int32_t best_lag = 0;
                                                                        float best_correlation = -FLT_MAX;
                                                                        int found = 0;
                                                                        if (result == NULL || sample_count < 2U ||         window_start + window_length > sample_count)         return -1;
                                                                        memset(result, 0, sizeof(*result));
                                                                        result->capture_success = 1U;
                                                                        if (timing_find_circular_lag(canonical_reference, measurement,             sample_count, &raw) == 0) {
                                                                            result->raw_candidate_lag = raw.lag_samples;
                                                                            result->raw_candidate_correlation = raw.correlation;
                                                                        }
                                                                        /* +/-0.5 interpolation refinement must remain inside [0, N-1). */
                                                                        result->valid_lag_min = (int32_t)ceil(         0.5 - (double)window_start);
                                                                        result->valid_lag_max = (int32_t)floor(         (double)(sample_count - 1U) -         (double)(window_start + window_length - 1U) - 0.5);
                                                                        result->raw_candidate_coverage_valid =         result->raw_candidate_lag >= result->valid_lag_min &&         result->raw_candidate_lag <= result->valid_lag_max;
                                                                        result->expected_lag =         expected_lag >= result->valid_lag_min &&         expected_lag <= result->valid_lag_max ? expected_lag : 0;
                                                                        result->local_lag_min =         result->expected_lag - CAL_FIXED_LAG_SEARCH_MARGIN;
                                                                        result->local_lag_max =         result->expected_lag + CAL_FIXED_LAG_SEARCH_MARGIN;
                                                                        if (result->local_lag_min < result->valid_lag_min)         result->local_lag_min = result->valid_lag_min;
                                                                        if (result->local_lag_max > result->valid_lag_max)         result->local_lag_max = result->valid_lag_max;
                                                                        for (int stage = 0;
                                                                        stage < 2 && !found;
                                                                        ++stage) {
                                                                            const int32_t first = stage == 0 ?             result->local_lag_min : result->valid_lag_min;
                                                                            const int32_t last = stage == 0 ?             result->local_lag_max : result->valid_lag_max;
                                                                            for (int32_t lag = first;
                                                                            lag <= last;
                                                                            ++lag) {
                                                                                float corr;
                                                                                if (calibration_fixed_window_correlation(                     canonical_reference, measurement, sample_count,                     window_start, window_length, lag, &corr) != 0)                 continue;
                                                                                if (!found || corr > best_correlation +                     CAL_FIXED_LAG_CORRELATION_TIE_EPSILON ||                 (fabsf(corr - best_correlation) <=                      CAL_FIXED_LAG_CORRELATION_TIE_EPSILON &&                  abs(lag - result->expected_lag) <                      abs(best_lag - result->expected_lag))) {
                                                                                    best_lag = lag;
                                                                                    best_correlation = corr;
                                                                                    found = 1;
                                                                                }
                                                                            }
                                                                            if (stage == 0 && found &&             best_correlation < CAL_TIMING_MIN_CORRELATION) {
                                                                                found = 0;
                                                                                best_correlation = -FLT_MAX;
                                                                            }
                                                                        }
                                                                        if (!found) {
                                                                            result->reject_reason = CAL_TIMING_REJECT_INVALID_OVERLAP;
                                                                            return -2;
                                                                        }
                                                                        result->integer_lag = best_lag;
                                                                        result->fractional_lag = 0.0f;
                                                                        if (best_lag > result->valid_lag_min &&         best_lag < result->valid_lag_max) {
                                                                            float left, center, right;
                                                                            if (calibration_fixed_window_correlation(canonical_reference,                 measurement, sample_count, window_start, window_length,                 best_lag - 1, &left) == 0 &&             calibration_fixed_window_correlation(canonical_reference,                 measurement, sample_count, window_start, window_length,                 best_lag, &center) == 0 &&             calibration_fixed_window_correlation(canonical_reference,                 measurement, sample_count, window_start, window_length,                 best_lag + 1, &right) == 0) {
                                                                                const double denominator =                 (double)left - 2.0 * center + right;
                                                                                const double fraction = fabs(denominator) > 1.0e-12 ?                 0.5 * ((double)left - right) / denominator : 0.0;
                                                                                if (isfinite(fraction) && fraction >= -0.5 && fraction <= 0.5)                 result->fractional_lag = (float)fraction;
                                                                            }
                                                                        }
                                                                        result->total_lag = result->integer_lag + result->fractional_lag;
                                                                        if (calibration_fixed_window_correlation(canonical_reference,             measurement, sample_count, window_start, window_length,             result->total_lag, &result->correlation) != 0) {
                                                                            result->reject_reason = CAL_TIMING_REJECT_INVALID_OVERLAP;
                                                                            return -3;
                                                                        }
                                                                        result->alignment_success = 1U;
                                                                        result->accepted =         result->correlation >= CAL_TIMING_MIN_CORRELATION;
                                                                        result->analysis_samples = (uint32_t)window_length;
                                                                        result->valid_overlap_samples = (uint32_t)window_length;
                                                                        result->reject_reason = result->accepted ? CAL_TIMING_REJECT_NONE :         CAL_TIMING_REJECT_LOW_CORRELATION;
                                                                        return 0;
                                                                    }
                                                                    static size_t calibration_phase_source_offset(     int selected_input_phase, int canonical_reference_phase) {
                                                                        /* The canonical reference never moves.  The candidate lag is estimated      * after applying this raw-array phase offset, so the canonical phase must      * not be added a second time. */
                                                                        (void)canonical_reference_phase;
                                                                        return selected_input_phase == 1 ? 1U : 0U;
                                                                    }
                                                                    static void calibration_print_mapping_test(     const calibration_pending_frame_t *saved,     const int16_t *phase_candidate,     size_t candidate_count,     double lag,     float gain_correction,     float offset_correction,     const char *label) {
                                                                        double adc_sum = 0.0, residual_sum = 0.0;
                                                                        float correlation;
                                                                        const size_t count = saved->calibration_window_length;
                                                                        if (calibration_fixed_window_correlation(             saved->alignment_reference, phase_candidate, candidate_count,             saved->calibration_window_start, count, lag,             &correlation) != 0)         return;
                                                                        for (size_t j = 0U;
                                                                        j < count;
                                                                        ++j) {
                                                                            const double position =             (double)(saved->calibration_window_start + j) + lag;
                                                                            const size_t lower = (size_t)floor(position);
                                                                            const double fraction = position - (double)lower;
                                                                            const double raw =             (1.0 - fraction) * phase_candidate[lower] +             fraction * phase_candidate[lower + 1U];
                                                                            const double adc = gain_correction *             calibration_apply_offset_correction(raw, offset_correction);
                                                                            const double predicted =             saved->canonical_nominal_system_gain *             saved->canonical_reference_window[j];
                                                                            adc_sum += adc;
                                                                            residual_sum += adc - predicted;
                                                                        }
                                                                        xil_printf("Mapping %s:\r\n", label);
                                                                        print_float_value("  correlation", correlation, "");
                                                                        print_float_value("  ADC mean",                       (float)(adc_sum / (double)count), " codes");
                                                                        print_float_value("  residual mean",                       (float)(residual_sum / (double)count), " codes");
                                                                    }
                                                                    static int calibration_canonical_reference_is_valid(     const calibration_aligned_frame_t *frame) {
                                                                        if (frame == NULL || frame->canonical_reference_window == NULL ||         frame->calibration_window_length != CAL_FIXED_WINDOW_LENGTH ||         frame->valid_analysis_sample_count !=             frame->calibration_window_length)         return 0;
                                                                        if (calibration_reference_checksum(                frame->canonical_reference_window,                frame->calibration_window_length) ==                frame->canonical_reference_checksum &&         isfinite(frame->analysis_reference_scale) &&         frame->analysis_reference_scale > 0.0f) {
                                                                            for (size_t i = 0U;
                                                                            i < frame->calibration_window_length;
                                                                            ++i) {
                                                                                if (frame->aligned_reference_samples[i] != (int16_t)lround(                     (double)frame->canonical_reference_window[i] *                     frame->analysis_reference_scale))                 return 0;
                                                                            }
                                                                            return 1;
                                                                        }
                                                                        return 0;
                                                                    }
                                                                    static void calibration_print_fixed_window(     const calibration_aligned_frame_t *frame) {
                                                                        const size_t last = frame->calibration_window_start +         frame->calibration_window_length - 1U;
                                                                        xil_printf("Canonical reference phase: %s\r\n",                frame->canonical_reference_phase == 0 ? "EVEN" : "ODD");
                                                                        xil_printf("Selected input phase     : %s\r\n",                frame->selected_phase_name);
                                                                        xil_printf("Integer lag             : %ld samples\r\n",                (long)frame->integer_lag);
                                                                        print_float_value("Fractional lag", frame->fractional_lag, " samples");
                                                                        xil_printf("Fixed window start      : %lu\r\n",                (unsigned long)frame->calibration_window_start);
                                                                        xil_printf("Fixed window length     : %lu\r\n",                (unsigned long)frame->calibration_window_length);
                                                                        xil_printf("Mapped reference first index: %lu\r\n",                (unsigned long)frame->calibration_window_start);
                                                                        xil_printf("Mapped reference last index : %lu\r\n",                (unsigned long)last);
                                                                        xil_printf("Analysis sample count   : %lu\r\n",                (unsigned long)frame->valid_analysis_sample_count);
                                                                        print_float_value("Fixed-window ADC mean",                       frame->metrics.adc_mean, " codes");
                                                                        print_float_value("Fixed-window reference mean",                       frame->canonical_reference_mean, " codes");
                                                                        xil_printf("Reference first sample  : %d\r\n",                frame->canonical_reference_window[0]);
                                                                        xil_printf("Reference last sample   : %d\r\n",                frame->canonical_reference_window[                    frame->calibration_window_length - 1U]);
                                                                        xil_printf("Reference checksum      : 0x%08lX\r\n",                (unsigned long)frame->canonical_reference_checksum);
                                                                        print_float_value("Fixed-window correlation",                       frame->correlation, "");
                                                                    }
                                                                    /*  * Capture exactly one DMA frame and carry that same frame through channel  * reconstruction, uploaded-reference alignment, validation, correction, and  * regression.  The returned sample pointers remain valid until workspace is  * reused by the next iteration.  */
                                                                    static int calibration_capture_and_align(     const int16_t *even_reference,     const int16_t *odd_reference,     size_t reference_count,     const calibration_frame_config_t *config,     calibration_frame_workspace_t *workspace,     calibration_aligned_frame_t *frame ) {
                                                                        static uint32_t capture_sequence;
                                                                        adc_reference_analysis_t analysis;
                                                                        calibration_state_t fit_state;
                                                                        calibration_config_t fit_config;
                                                                        const int16_t *selected_raw_adc;
                                                                        size_t reconstructed_count = 0U;
                                                                        size_t analysis_count;
                                                                        size_t window_start;
                                                                        double raw_sum = 0.0;
                                                                        int status;
                                                                        if (frame == NULL) {
                                                                            return -1;
                                                                        }
                                                                        memset(frame, 0, sizeof(*frame));
                                                                        frame->capture_sequence = ++capture_sequence;
                                                                        frame->selected_channel = -1;
                                                                        frame->selected_reference_phase = -1;
                                                                        frame->canonical_reference_phase = -1;
                                                                        frame->selected_channel_name = "none";
                                                                        frame->selected_phase_name = "none";
                                                                        frame->rejection_reason = "invalid shared-frame input";
                                                                        if ((even_reference == NULL) || (odd_reference == NULL) ||         (config == NULL) || (workspace == NULL) ||         (reference_count != ADC_CHANNEL_SAMPLE_COUNT) ||         (config->locked_channel < -1) || (config->locked_channel > 1) ||         !isfinite(config->adc_gain_correction) ||         !isfinite(config->adc_offset_correction) ||         !isfinite(config->reference_scale) ||         (config->adc_gain_correction <= 0.0f) ||         (config->reference_scale <= 0.0f)) {
                                                                            return -1;
                                                                        }
                                                                        if (adc_capture_frame() != XST_SUCCESS) {
                                                                            frame->rejection_reason = "DMA capture failed";
                                                                            return -2;
                                                                        }
                                                                        frame->capture_succeeded = true;
                                                                        status = adc_reconstruct_channels(         RxBufferPtr, DMA_CMD_BUF_SIZE,         workspace->channel_a, ADC_CHANNEL_SAMPLE_COUNT,         workspace->channel_b, ADC_CHANNEL_SAMPLE_COUNT,         &reconstructed_count     );
                                                                        if ((status != 0) || (reconstructed_count != reference_count)) {
                                                                            frame->rejection_reason = "sample reconstruction failed";
                                                                            return -3;
                                                                        }
                                                                        frame->reconstruction_succeeded = true;
                                                                        status = calibration_analyze_reference_frame(         even_reference, odd_reference,         workspace->channel_a, workspace->channel_b,         reconstructed_count,         workspace->fractional_reference,         workspace->fractional_measurement,         config->locked_channel, &analysis     );
                                                                        frame->selected_channel_name =         analysis.selected_channel_name != NULL ?         analysis.selected_channel_name : "none";
                                                                        frame->selected_phase_name =         analysis.selected_phase_name != NULL ?         analysis.selected_phase_name : "none";
                                                                        frame->timing = analysis.timing;
                                                                        frame->integer_lag = analysis.timing.integer_lag;
                                                                        frame->fractional_lag = analysis.timing.fractional_lag;
                                                                        frame->total_lag = analysis.timing.total_lag;
                                                                        frame->correlation = analysis.timing.correlation;
                                                                        frame->reference_frequency_hz = analysis.reference_frequency_hz;
                                                                        frame->adc_frequency_hz = analysis.adc_frequency_hz;
                                                                        if (analysis.selected_adc == workspace->channel_a) {
                                                                            frame->selected_channel = 0;
                                                                        }
                                                                        else if (analysis.selected_adc == workspace->channel_b) {
                                                                            frame->selected_channel = 1;
                                                                        }
                                                                        if (analysis.selected_reference == even_reference) {
                                                                            frame->selected_reference_phase = 0;
                                                                        }
                                                                        else if (analysis.selected_reference == odd_reference) {
                                                                            frame->selected_reference_phase = 1;
                                                                        }
                                                                        if (status != 0) {
                                                                            frame->rejection_reason =             analysis.failure_reason != NULL ?             analysis.failure_reason : "invalid alignment";
                                                                            return -4;
                                                                        }
                                                                        selected_raw_adc = frame->selected_channel == 1 ?         workspace->channel_b : workspace->channel_a;
                                                                        if (config->reject_clipped_input &&         calibration_samples_are_clipped(             selected_raw_adc, reconstructed_count)) {
                                                                            frame->rejection_reason = "ADC clipping detected";
                                                                            return -5;
                                                                        }
                                                                        analysis_count = CAL_FIXED_WINDOW_LENGTH;
                                                                        if (reconstructed_count < analysis_count + 2U) {
                                                                            frame->rejection_reason = "fixed calibration window not fully covered";
                                                                            return -6;
                                                                        }
                                                                        window_start = (reconstructed_count - analysis_count) / 2U;
                                                                        if (calibration_map_fixed_window(             analysis.selected_reference, selected_raw_adc,             reconstructed_count, analysis.timing.total_lag,             window_start, analysis_count,             workspace->aligned_reference, workspace->aligned_raw_adc,             &fit_state) != 0) {
                                                                            frame->rejection_reason =             "fixed calibration window not fully covered";
                                                                            return -6;
                                                                        }
                                                                        for (size_t i = 0U;
                                                                        i < analysis_count;
                                                                        ++i) {
                                                                            const int16_t raw_adc = workspace->aligned_raw_adc[i];
                                                                            const double corrected_adc =             (double)raw_adc * (double)config->adc_gain_correction +             (double)config->adc_offset_correction;
                                                                            const double scaled_reference =             (double)workspace->aligned_reference[i] *             (double)config->reference_scale;
                                                                            long corrected_code;
                                                                            long reference_code;
                                                                            if (!isfinite(corrected_adc) || !isfinite(scaled_reference)) {
                                                                                frame->rejection_reason = "nonfinite corrected aligned sample";
                                                                                return -7;
                                                                            }
                                                                            corrected_code = lround(corrected_adc);
                                                                            reference_code = lround(scaled_reference);
                                                                            if ((corrected_code < INT16_MIN) || (corrected_code > INT16_MAX) ||             (reference_code < INT16_MIN) || (reference_code > INT16_MAX) ||             (config->reject_clipped_input &&              ((corrected_code < CALIBRATION_ADC_MIN_CODE) ||               (corrected_code > CALIBRATION_ADC_MAX_CODE)))) {
                                                                                frame->rejection_reason = "corrected aligned sample out of range";
                                                                                return -8;
                                                                            }
                                                                            workspace->aligned_corrected_adc[i] = (int16_t)corrected_code;
                                                                            workspace->aligned_reference[i] = (int16_t)reference_code;
                                                                            raw_sum += (double)raw_adc;
                                                                        }
                                                                        calibration_default_config(&fit_config);
                                                                        status = calibration_init(&fit_state, &fit_config);
                                                                        if (status != CALIBRATION_OK) {
                                                                            frame->rejection_reason = "regression initialization failed";
                                                                            return -9;
                                                                        }
                                                                        status = calibration_analyze_frame(         &fit_state,         workspace->aligned_corrected_adc,         workspace->aligned_reference,         analysis_count     );
                                                                        if ((status != CALIBRATION_OK) ||         !calibration_fit_metrics_are_valid(&fit_state)) {
                                                                            frame->rejection_reason = "invalid aligned regression";
                                                                            return -10;
                                                                        }
                                                                        frame->aligned_reference_samples = workspace->aligned_reference;
                                                                        frame->aligned_raw_adc_samples = workspace->aligned_raw_adc;
                                                                        frame->aligned_corrected_adc_samples =         workspace->aligned_corrected_adc;
                                                                        frame->alignment_reference_samples = analysis.selected_reference;
                                                                        frame->canonical_reference_window = workspace->aligned_reference;
                                                                        frame->canonical_reference_phase = frame->selected_reference_phase;
                                                                        frame->alignment_reference_count = reconstructed_count;
                                                                        frame->calibration_window_start = window_start;
                                                                        frame->calibration_window_length = analysis_count;
                                                                        frame->analysis_reference_scale = config->reference_scale;
                                                                        frame->valid_analysis_sample_count = analysis_count;
                                                                        frame->raw_aligned_adc_mean =         (float)(raw_sum / (double)analysis_count);
                                                                        frame->metrics = fit_state.metrics;
                                                                        frame->canonical_reference_checksum =         calibration_reference_checksum(             workspace->aligned_reference, analysis_count);
                                                                        frame->canonical_reference_mean =         calibration_reference_mean_value(             workspace->aligned_reference, analysis_count);
                                                                        frame->canonical_nominal_system_gain = frame->metrics.measured_gain;
                                                                        memset(&frame->overlap, 0, sizeof(frame->overlap));
                                                                        frame->overlap.reference_start = window_start;
                                                                        frame->overlap.analysis_count = analysis_count;
                                                                        frame->overlap.overlap_count = analysis_count;
                                                                        frame->correlation = frame->metrics.correlation;
                                                                        (void)calibration_compute_timing_diagnostics(         analysis.selected_reference,         workspace->channel_a,         workspace->channel_b,         reconstructed_count,         frame->selected_channel,         analysis.reference_frequency_hz,         adc_get_effective_sample_rate_hz(),         (double)frame->total_lag,         window_start,         analysis_count,         &frame->timing_diagnostics);
                                                                        if (!calibration_canonical_reference_is_valid(frame)) {
                                                                            frame->rejection_reason =             "ERROR: canonical calibration reference changed";
                                                                            return -11;
                                                                        }
                                                                        if (frame->correlation < CAL_DAC_REF_MIN_CORRELATION) {
                                                                            frame->rejection_reason =             "fixed-window correlation below threshold";
                                                                            return -11;
                                                                        }
                                                                        frame->frame_valid = true;
                                                                        frame->rejection_reason = "none";
                                                                        calibration_validate_timing_alignment(         frame, &frame->timing_diagnostics);
                                                                        return 0;
                                                                    }
                                                                    static int calibration_pending_frame_is_compatible(const char **reason) {
                                                                        const calibration_pending_frame_t *pending =         &g_pending_calibration_frame;
                                                                        const double current_ratio =         DAC_SAMPLE_RATE_HZ / adc_get_effective_sample_rate_hz();
                                                                        if (reason != NULL) {
                                                                            *reason = "No valid pending aligned frame.";
                                                                        }
                                                                        if (!pending->valid || pending->consumed) {
                                                                            return 0;
                                                                        }
                                                                        if (!reference_buffer_is_ready() ||         (pending->reference_generation != reference_buffer_generation()) ||         (pending->reference_length != reference_buffer_length()) ||         (pending->reference_format != reference_buffer_format())) {
                                                                            if (reason != NULL) *reason = "Uploaded DAC reference changed.";
                                                                            return 0;
                                                                        }
                                                                        if ((pending->sample_rate_generation != g_adc_sample_rate.generation) ||         (pending->configured_sample_rate_hz !=              adc_get_configured_sample_rate_hz()) ||         (pending->effective_sample_rate_hz !=              adc_get_effective_sample_rate_hz()) ||         (pending->dac_adc_rate_ratio != current_ratio)) {
                                                                            if (reason != NULL) *reason = "ADC/DAC sample-rate configuration changed.";
                                                                            return 0;
                                                                        }
                                                                        if (pending->channel_configuration !=         calibration_channel_selection()) {
                                                                            if (reason != NULL) *reason = "Calibration channel setting changed.";
                                                                            return 0;
                                                                        }
                                                                        if ((pending->software_gain_correction !=              calibration_software_gain_correction()) ||         (pending->software_offset_correction !=              calibration_software_offset_correction())) {
                                                                            if (reason != NULL) *reason = "Software calibration coefficients changed.";
                                                                            return 0;
                                                                        }
                                                                        if ((pending->analysis_sample_count < CAL_MIN_ANALYSIS_SAMPLES) ||         (pending->analysis_sample_count > ADC_CHANNEL_SAMPLE_COUNT) ||         (pending->calibration_window_length !=              pending->analysis_sample_count) ||         (pending->alignment_reference_count != ADC_CHANNEL_SAMPLE_COUNT) ||         (pending->calibration_window_start +              pending->calibration_window_length >              pending->alignment_reference_count) ||         calibration_reference_checksum(             pending->canonical_reference_window,             pending->calibration_window_length) !=             pending->canonical_reference_checksum ||         !isfinite(pending->canonical_nominal_system_gain)) {
                                                                            if (reason != NULL) *reason = "Pending aligned sample count is invalid.";
                                                                            return 0;
                                                                        }
                                                                        return 1;
                                                                    }
                                                                    static int calibration_stored_reference_is_compatible(const char **reason) {
                                                                        const calibration_pending_frame_t *saved = &g_stored_offset_reference;
                                                                        const double current_ratio =         DAC_SAMPLE_RATE_HZ / adc_get_effective_sample_rate_hz();
                                                                        if (reason != NULL) *reason = "No valid stored adc -cal frame.";
                                                                        if (!saved->valid || saved->consumed) return 0;
                                                                        if (!reference_buffer_is_ready() ||         saved->reference_generation != reference_buffer_generation() ||         saved->reference_length != reference_buffer_length() ||         saved->reference_format != reference_buffer_format()) {
                                                                            if (reason != NULL) *reason = "Uploaded DAC reference changed.";
                                                                            return 0;
                                                                        }
                                                                        if (saved->sample_rate_generation != g_adc_sample_rate.generation ||         saved->configured_sample_rate_hz != adc_get_configured_sample_rate_hz() ||         saved->effective_sample_rate_hz != adc_get_effective_sample_rate_hz() ||         saved->dac_adc_rate_ratio != current_ratio) {
                                                                            if (reason != NULL) *reason = "ADC/DAC sample-rate configuration changed.";
                                                                            return 0;
                                                                        }
                                                                        if (saved->channel_configuration != calibration_channel_selection() ||         saved->software_gain_correction != calibration_software_gain_correction()) {
                                                                            if (reason != NULL) *reason = "Calibration channel or gain changed.";
                                                                            return 0;
                                                                        }
                                                                        if (saved->analysis_sample_count < CAL_MIN_ANALYSIS_SAMPLES ||         saved->analysis_sample_count > ADC_CHANNEL_SAMPLE_COUNT ||         saved->calibration_window_length != saved->analysis_sample_count ||         saved->alignment_reference_count != ADC_CHANNEL_SAMPLE_COUNT ||         saved->calibration_window_start +             saved->calibration_window_length >             saved->alignment_reference_count ||         calibration_reference_checksum(             saved->canonical_reference_window,             saved->calibration_window_length) !=             saved->canonical_reference_checksum ||         !isfinite(saved->canonical_nominal_system_gain)) {
                                                                            if (reason != NULL) *reason = "Stored reference sample count is invalid.";
                                                                            return 0;
                                                                        }
                                                                        return 1;
                                                                    }
                                                                    static int calibration_pending_frame_copy(     calibration_pending_frame_t *destination,     const calibration_aligned_frame_t *frame,     uint32_t frame_number,     uint32_t reference_generation,     size_t reference_length,     reference_buffer_format_t reference_format ) {
                                                                        const size_t sample_count = frame != NULL ?         frame->valid_analysis_sample_count : 0U;
                                                                        if ((destination == NULL) || (frame == NULL) || !frame->frame_valid ||         (frame->aligned_reference_samples == NULL) ||         (frame->aligned_raw_adc_samples == NULL) ||         (frame->aligned_corrected_adc_samples == NULL) ||         (frame->alignment_reference_samples == NULL) ||         (frame->canonical_reference_window == NULL) ||         (frame->alignment_reference_count != ADC_CHANNEL_SAMPLE_COUNT) ||         (frame->calibration_window_length != sample_count) ||         (frame->calibration_window_start + sample_count >              frame->alignment_reference_count) ||         !calibration_canonical_reference_is_valid(frame) ||         (sample_count < CAL_MIN_ANALYSIS_SAMPLES) ||         (sample_count > ADC_CHANNEL_SAMPLE_COUNT) ||         !reference_buffer_is_ready() ||         (reference_generation != reference_buffer_generation()) ||         (reference_length != reference_buffer_length()) ||         (reference_format != reference_buffer_format())) {
                                                                            return -1;
                                                                        }
                                                                        memset(destination, 0, sizeof(*destination));
                                                                        destination->selected_channel = -1;
                                                                        destination->selected_reference_phase = -1;
                                                                        destination->canonical_reference_phase = -1;
                                                                        destination->capture_sequence = frame->capture_sequence;
                                                                        destination->retained_frame_number = frame_number;
                                                                        destination->selected_channel = frame->selected_channel;
                                                                        destination->selected_reference_phase = frame->selected_reference_phase;
                                                                        destination->canonical_reference_phase =         frame->canonical_reference_phase;
                                                                        destination->integer_lag = frame->integer_lag;
                                                                        destination->fractional_lag = frame->fractional_lag;
                                                                        destination->total_lag = frame->total_lag;
                                                                        destination->analysis_sample_count = sample_count;
                                                                        destination->raw_aligned_adc_mean = frame->raw_aligned_adc_mean;
                                                                        destination->correlation = frame->correlation;
                                                                        destination->metrics = frame->metrics;
                                                                        destination->timing = frame->timing;
                                                                        destination->overlap = frame->overlap;
                                                                        destination->reference_frequency_hz = frame->reference_frequency_hz;
                                                                        destination->adc_frequency_hz = frame->adc_frequency_hz;
                                                                        destination->timing_diagnostics = frame->timing_diagnostics;
                                                                        memcpy(destination->aligned_reference,            frame->aligned_reference_samples,            sample_count * sizeof(destination->aligned_reference[0]));
                                                                        memcpy(destination->aligned_raw_adc,            frame->aligned_raw_adc_samples,            sample_count * sizeof(destination->aligned_raw_adc[0]));
                                                                        memcpy(destination->aligned_corrected_adc,            frame->aligned_corrected_adc_samples,            sample_count * sizeof(destination->aligned_corrected_adc[0]));
                                                                        memcpy(destination->alignment_reference,            frame->alignment_reference_samples,            frame->alignment_reference_count *                sizeof(destination->alignment_reference[0]));
                                                                        destination->alignment_reference_count =         frame->alignment_reference_count;
                                                                        memcpy(destination->canonical_reference_window,            frame->canonical_reference_window,            sample_count *                sizeof(destination->canonical_reference_window[0]));
                                                                        destination->calibration_window_start =         frame->calibration_window_start;
                                                                        destination->calibration_window_length =         frame->calibration_window_length;
                                                                        destination->canonical_reference_checksum =         frame->canonical_reference_checksum;
                                                                        destination->canonical_reference_mean =         frame->canonical_reference_mean;
                                                                        destination->analysis_reference_scale =         frame->analysis_reference_scale;
                                                                        destination->canonical_nominal_system_gain =         frame->canonical_nominal_system_gain;
                                                                        destination->reference_generation = reference_generation;
                                                                        destination->reference_length = reference_length;
                                                                        destination->reference_format = reference_format;
                                                                        destination->sample_rate_generation = g_adc_sample_rate.generation;
                                                                        destination->configured_sample_rate_hz =         adc_get_configured_sample_rate_hz();
                                                                        destination->effective_sample_rate_hz =         adc_get_effective_sample_rate_hz();
                                                                        destination->dac_adc_rate_ratio =         DAC_SAMPLE_RATE_HZ / adc_get_effective_sample_rate_hz();
                                                                        destination->channel_configuration = calibration_channel_selection();
                                                                        destination->software_gain_correction =         calibration_software_gain_correction();
                                                                        destination->software_offset_correction =         calibration_software_offset_correction();
                                                                        destination->consumed = false;
                                                                        destination->valid = true;
                                                                        return 0;
                                                                    }
                                                                    /*  * Select a typical accepted frame, not the final frame or the frame with the  * best correlation.  Ranking is lexicographic and therefore deterministic.  */
                                                                    static int calibration_select_representative_frame(     const calibration_pending_frame_t *candidates,     size_t candidate_count,     size_t *selected_index,     calibration_selection_medians_t *medians ) {
                                                                        float offsets[ADC_CAL_MAX_FRAMES];
                                                                        float normalized_gains[ADC_CAL_MAX_FRAMES];
                                                                        float fitted_rmse[ADC_CAL_MAX_FRAMES];
                                                                        size_t best_index = 0U;
                                                                        if ((candidates == NULL) || (selected_index == NULL) ||         (medians == NULL) || (candidate_count == 0U) ||         (candidate_count > ADC_CAL_MAX_FRAMES)) {
                                                                            return -1;
                                                                        }
                                                                        for (size_t i = 0U;
                                                                        i < candidate_count;
                                                                        ++i) {
                                                                            if (!candidates[i].valid || candidates[i].consumed ||             !isfinite(candidates[i].metrics.measured_offset) ||             !isfinite(candidates[i].metrics.measured_gain) ||             !isfinite(candidates[i].metrics.fitted_rmse_codes) ||             !isfinite(candidates[i].correlation)) {
                                                                                return -2;
                                                                            }
                                                                            offsets[i] = candidates[i].metrics.measured_offset;
                                                                            normalized_gains[i] = candidates[i].metrics.measured_gain *             (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
                                                                            fitted_rmse[i] = candidates[i].metrics.fitted_rmse_codes;
                                                                        }
                                                                        medians->median_fitted_offset =         median_float(offsets, candidate_count);
                                                                        medians->median_normalized_gain =         median_float(normalized_gains, candidate_count);
                                                                        medians->median_fitted_rmse =         median_float(fitted_rmse, candidate_count);
                                                                        if (!isfinite(medians->median_fitted_offset) ||         !isfinite(medians->median_normalized_gain) ||         !isfinite(medians->median_fitted_rmse)) {
                                                                            return -3;
                                                                        }
                                                                        for (size_t i = 1U;
                                                                        i < candidate_count;
                                                                        ++i) {
                                                                            const float candidate_offset_distance = fabsf(             candidates[i].metrics.measured_offset -             medians->median_fitted_offset);
                                                                            const float best_offset_distance = fabsf(             candidates[best_index].metrics.measured_offset -             medians->median_fitted_offset);
                                                                            const float candidate_normalized_gain =             candidates[i].metrics.measured_gain *             (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
                                                                            const float best_normalized_gain =             candidates[best_index].metrics.measured_gain *             (CAL_DAC_FULL_SCALE_CODES / CAL_ADC_FULL_SCALE_CODES);
                                                                            const float candidate_gain_distance = fabsf(             candidate_normalized_gain - medians->median_normalized_gain);
                                                                            const float best_gain_distance = fabsf(             best_normalized_gain - medians->median_normalized_gain);
                                                                            int choose_candidate = 0;
                                                                            if (candidate_offset_distance <             best_offset_distance - CAL_REPRESENTATIVE_TIE_EPSILON) {
                                                                                choose_candidate = 1;
                                                                            }
                                                                            else if (fabsf(candidate_offset_distance - best_offset_distance) <=                    CAL_REPRESENTATIVE_TIE_EPSILON) {
                                                                                if (candidate_gain_distance <                 best_gain_distance - CAL_REPRESENTATIVE_TIE_EPSILON) {
                                                                                    choose_candidate = 1;
                                                                                }
                                                                                else if (fabsf(candidate_gain_distance - best_gain_distance) <=                        CAL_REPRESENTATIVE_TIE_EPSILON) {
                                                                                    if (candidates[i].metrics.fitted_rmse_codes <                     candidates[best_index].metrics.fitted_rmse_codes -                         CAL_REPRESENTATIVE_TIE_EPSILON) {
                                                                                        choose_candidate = 1;
                                                                                    }
                                                                                    else if (fabsf(                         candidates[i].metrics.fitted_rmse_codes -                         candidates[best_index].metrics.fitted_rmse_codes) <=                            CAL_REPRESENTATIVE_TIE_EPSILON) {
                                                                                        if (candidates[i].correlation >                         candidates[best_index].correlation +                             CAL_REPRESENTATIVE_TIE_EPSILON) {
                                                                                            choose_candidate = 1;
                                                                                        }
                                                                                        else if (fabsf(candidates[i].correlation -                                      candidates[best_index].correlation) <=                                    CAL_REPRESENTATIVE_TIE_EPSILON &&                                candidates[i].retained_frame_number <                                    candidates[best_index].retained_frame_number) {
                                                                                            choose_candidate = 1;
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                            if (choose_candidate) {
                                                                                best_index = i;
                                                                            }
                                                                        }
                                                                        *selected_index = best_index;
                                                                        return 0;
                                                                    }
                                                                    static int calibration_restore_owned_frame(     const calibration_pending_frame_t *source,     calibration_frame_workspace_t *workspace,     calibration_aligned_frame_t *frame ) {
                                                                        const size_t sample_count = source != NULL ?         source->analysis_sample_count : 0U;
                                                                        if ((source == NULL) || !source->valid || source->consumed ||         (workspace == NULL) || (frame == NULL) ||         (sample_count < CAL_MIN_ANALYSIS_SAMPLES) ||         (sample_count > ADC_CHANNEL_SAMPLE_COUNT)) {
                                                                            return -1;
                                                                        }
                                                                        memset(frame, 0, sizeof(*frame));
                                                                        memcpy(workspace->aligned_reference,            source->aligned_reference,            sample_count * sizeof(workspace->aligned_reference[0]));
                                                                        memcpy(workspace->aligned_raw_adc,            source->aligned_raw_adc,            sample_count * sizeof(workspace->aligned_raw_adc[0]));
                                                                        memcpy(workspace->aligned_corrected_adc,            source->aligned_corrected_adc,            sample_count * sizeof(workspace->aligned_corrected_adc[0]));
                                                                        frame->capture_sequence = source->capture_sequence;
                                                                        frame->retained_frame_number = source->retained_frame_number;
                                                                        frame->capture_succeeded = true;
                                                                        frame->reconstruction_succeeded = true;
                                                                        frame->frame_valid = true;
                                                                        frame->selected_channel = source->selected_channel;
                                                                        frame->selected_reference_phase = source->selected_reference_phase;
                                                                        frame->canonical_reference_phase = source->canonical_reference_phase;
                                                                        frame->selected_channel_name = source->selected_channel == 0 ?         "Channel A" : "Channel B";
                                                                        frame->selected_phase_name = source->selected_reference_phase == 0 ?         "EVEN" : "ODD";
                                                                        frame->integer_lag = source->integer_lag;
                                                                        frame->fractional_lag = source->fractional_lag;
                                                                        frame->total_lag = source->total_lag;
                                                                        frame->aligned_reference_samples = workspace->aligned_reference;
                                                                        frame->aligned_raw_adc_samples = workspace->aligned_raw_adc;
                                                                        frame->aligned_corrected_adc_samples =         workspace->aligned_corrected_adc;
                                                                        frame->alignment_reference_samples = source->alignment_reference;
                                                                        frame->canonical_reference_window =         source->canonical_reference_window;
                                                                        frame->alignment_reference_count = source->alignment_reference_count;
                                                                        frame->calibration_window_start = source->calibration_window_start;
                                                                        frame->calibration_window_length = source->calibration_window_length;
                                                                        frame->canonical_reference_checksum =         source->canonical_reference_checksum;
                                                                        frame->canonical_reference_mean = source->canonical_reference_mean;
                                                                        frame->analysis_reference_scale = source->analysis_reference_scale;
                                                                        frame->canonical_nominal_system_gain =         source->canonical_nominal_system_gain;
                                                                        frame->valid_analysis_sample_count = sample_count;
                                                                        frame->raw_aligned_adc_mean = source->raw_aligned_adc_mean;
                                                                        frame->correlation = source->correlation;
                                                                        frame->metrics = source->metrics;
                                                                        frame->timing = source->timing;
                                                                        frame->overlap = source->overlap;
                                                                        frame->reference_frequency_hz = source->reference_frequency_hz;
                                                                        frame->adc_frequency_hz = source->adc_frequency_hz;
                                                                        frame->timing_diagnostics = source->timing_diagnostics;
                                                                        frame->rejection_reason = "none";
                                                                        return 0;
                                                                    }
                                                                    static int calibration_pending_frame_consume(     float adc_gain_correction,     float adc_offset_correction,     float reference_scale,     calibration_frame_workspace_t *workspace,     calibration_aligned_frame_t *frame,     const char **reason ) {
                                                                        calibration_pending_frame_t *pending =         &g_pending_calibration_frame;
                                                                        calibration_state_t fit_state;
                                                                        calibration_config_t fit_config;
                                                                        const size_t sample_count = pending->analysis_sample_count;
                                                                        int status;
                                                                        if ((workspace == NULL) || (frame == NULL) ||         !isfinite(adc_gain_correction) ||         !isfinite(adc_offset_correction) ||         !isfinite(reference_scale) || (adc_gain_correction <= 0.0f) ||         (reference_scale <= 0.0f)) {
                                                                            if (reason != NULL) *reason = "Invalid pending-frame consumer.";
                                                                            return -1;
                                                                        }
                                                                        if (!calibration_pending_frame_is_compatible(reason)) {
                                                                            calibration_gain_input_frame_invalidate();
                                                                            return -2;
                                                                        }
                                                                        /* Claim the pending frame before copying it into the loop workspace. */
                                                                        pending->consumed = true;
                                                                        pending->valid = false;
                                                                        memset(frame, 0, sizeof(*frame));
                                                                        frame->capture_sequence = pending->capture_sequence;
                                                                        frame->retained_frame_number = pending->retained_frame_number;
                                                                        frame->capture_succeeded = true;
                                                                        frame->reconstruction_succeeded = true;
                                                                        frame->selected_channel = pending->selected_channel;
                                                                        frame->selected_reference_phase = pending->selected_reference_phase;
                                                                        frame->canonical_reference_phase = pending->canonical_reference_phase;
                                                                        frame->selected_channel_name = pending->selected_channel == 0 ?         "Channel A" : "Channel B";
                                                                        frame->selected_phase_name = pending->selected_reference_phase == 0 ?         "EVEN" : "ODD";
                                                                        frame->integer_lag = pending->integer_lag;
                                                                        frame->fractional_lag = pending->fractional_lag;
                                                                        frame->total_lag = pending->total_lag;
                                                                        frame->correlation = pending->correlation;
                                                                        frame->timing = pending->timing;
                                                                        frame->overlap = pending->overlap;
                                                                        frame->reference_frequency_hz = pending->reference_frequency_hz;
                                                                        frame->adc_frequency_hz = pending->adc_frequency_hz;
                                                                        frame->timing_diagnostics = pending->timing_diagnostics;
                                                                        frame->raw_aligned_adc_mean = pending->raw_aligned_adc_mean;
                                                                        frame->alignment_reference_samples = pending->alignment_reference;
                                                                        frame->canonical_reference_window =         pending->canonical_reference_window;
                                                                        frame->alignment_reference_count = pending->alignment_reference_count;
                                                                        frame->calibration_window_start = pending->calibration_window_start;
                                                                        frame->calibration_window_length = pending->calibration_window_length;
                                                                        frame->canonical_reference_checksum =         pending->canonical_reference_checksum;
                                                                        frame->canonical_reference_mean = pending->canonical_reference_mean;
                                                                        frame->analysis_reference_scale = reference_scale;
                                                                        frame->canonical_nominal_system_gain =         pending->canonical_nominal_system_gain;
                                                                        for (size_t i = 0U;
                                                                        i < sample_count;
                                                                        ++i) {
                                                                            const int16_t raw_adc = pending->aligned_raw_adc[i];
                                                                            const double corrected_adc = (double)adc_gain_correction *             calibration_apply_offset_correction(                 raw_adc, adc_offset_correction);
                                                                            const double scaled_reference =             (double)pending->canonical_reference_window[i] *             (double)reference_scale;
                                                                            const long corrected_code = lround(corrected_adc);
                                                                            const long reference_code = lround(scaled_reference);
                                                                            if (!isfinite(corrected_adc) || !isfinite(scaled_reference) ||             (corrected_code < CALIBRATION_ADC_MIN_CODE) ||             (corrected_code > CALIBRATION_ADC_MAX_CODE) ||             (reference_code < INT16_MIN) ||             (reference_code > INT16_MAX)) {
                                                                                if (reason != NULL) *reason =                 "Pending aligned sample correction failed.";
                                                                                return -3;
                                                                            }
                                                                            workspace->aligned_raw_adc[i] = raw_adc;
                                                                            workspace->aligned_reference[i] = (int16_t)reference_code;
                                                                            workspace->aligned_corrected_adc[i] = (int16_t)corrected_code;
                                                                        }
                                                                        calibration_default_config(&fit_config);
                                                                        status = calibration_init(&fit_state, &fit_config);
                                                                        if (status == CALIBRATION_OK) {
                                                                            status = calibration_analyze_frame(             &fit_state,             workspace->aligned_corrected_adc,             workspace->aligned_reference,             sample_count         );
                                                                        }
                                                                        if ((status != CALIBRATION_OK) ||         !calibration_fit_metrics_are_valid(&fit_state)) {
                                                                            if (reason != NULL) *reason = "Pending-frame regression failed.";
                                                                            return -4;
                                                                        }
                                                                        frame->aligned_reference_samples = workspace->aligned_reference;
                                                                        frame->aligned_raw_adc_samples = workspace->aligned_raw_adc;
                                                                        frame->aligned_corrected_adc_samples =         workspace->aligned_corrected_adc;
                                                                        frame->valid_analysis_sample_count = sample_count;
                                                                        frame->metrics = fit_state.metrics;
                                                                        frame->correlation = frame->metrics.correlation;
                                                                        if (frame->valid_analysis_sample_count !=             frame->calibration_window_length ||         frame->overlap.reference_start !=             frame->calibration_window_start ||         !calibration_canonical_reference_is_valid(frame)) {
                                                                            if (reason != NULL) *reason =             "INTERNAL ALIGNMENT ERROR: canonical calibration reference changed";
                                                                            return -5;
                                                                        }
                                                                        frame->frame_valid = true;
                                                                        frame->rejection_reason = "none";
                                                                        if (reason != NULL) *reason = "none";
                                                                        return 0;
                                                                    }
                                                                    /* Capture one fresh ADC frame and align its fixed channel to the immutable  * waveform retained by adc -cal. */
                                                                    static int calibration_capture_against_owned_reference(     const calibration_pending_frame_t *saved,     bool use_saved_calibration_reference,     float gain_correction,     float offset_correction,     float reference_scale,     calibration_frame_workspace_t *workspace,     calibration_aligned_frame_t *frame,     const char **reason) {
                                                                        calibration_timing_frame_result_t phase_results[2];
                                                                        calibration_state_t fit_state;
                                                                        calibration_config_t fit_config;
                                                                        const int16_t *captured_channel;
                                                                        const int16_t *phase_candidates[2];
                                                                        const int16_t *selected_candidate;
                                                                        const calibration_timing_frame_result_t *selected_result = NULL;
                                                                        size_t selected_phase = 0U;
                                                                        size_t selected_available_count;
                                                                        size_t reconstructed_count = 0U;
                                                                        size_t count;
                                                                        double raw_sum = 0.0;
                                                                        int status;
                                                                        if (reason != NULL) *reason = "fresh ADC capture is invalid";
                                                                        if (saved == NULL || !saved->valid || workspace == NULL || frame == NULL ||         !isfinite(gain_correction) || gain_correction <= 0.0f ||         !isfinite(offset_correction) || !isfinite(reference_scale) ||         reference_scale <= 0.0f) return -1;
                                                                        memset(frame, 0, sizeof(*frame));
                                                                        if (adc_capture_frame() != XST_SUCCESS) return -2;
                                                                        if (adc_reconstruct_channels(RxBufferPtr, DMA_CMD_BUF_SIZE,             workspace->channel_a, ADC_CHANNEL_SAMPLE_COUNT,             workspace->channel_b, ADC_CHANNEL_SAMPLE_COUNT,             &reconstructed_count) != 0 ||         reconstructed_count != saved->alignment_reference_count) {
                                                                            if (reason != NULL) *reason = "ADC reconstruction failed";
                                                                            return -3;
                                                                        }
                                                                        captured_channel = saved->selected_channel == 1 ?         workspace->channel_b : workspace->channel_a;
                                                                        /* DMA may start on either converter sample phase.  Keep the saved      * waveform fixed, but independently test both possible fresh starts. */
                                                                        phase_candidates[0] = captured_channel +         calibration_phase_source_offset(             0, saved->canonical_reference_phase);
                                                                        phase_candidates[1] = captured_channel +         calibration_phase_source_offset(             1, saved->canonical_reference_phase);
                                                                        for (size_t phase = 0U;
                                                                        phase < 2U;
                                                                        ++phase) {
                                                                            const size_t phase_count = reconstructed_count - phase;
                                                                            (void)calibration_search_fixed_window_lag(             saved->alignment_reference, phase_candidates[phase],             phase_count, saved->calibration_window_start,             saved->calibration_window_length, saved->integer_lag,             &phase_results[phase]);
                                                                        }
                                                                        if (ADC_CAL_VERBOSE_DEBUG) {
                                                                            xil_printf("Expected lag            : %ld samples\r\n",                (long)phase_results[0].expected_lag);
                                                                            xil_printf("Local search range      : %ld ... %ld samples\r\n",                (long)phase_results[0].local_lag_min,                (long)phase_results[0].local_lag_max);
                                                                            for (size_t phase = 0U;
                                                                            phase < 2U;
                                                                            ++phase) {
                                                                                const char *name = phase == 0U ? "EVEN" : "ODD";
                                                                                xil_printf("%s best raw candidate:\r\n", name);
                                                                                print_float_value("  raw correlation",                           phase_results[phase].raw_candidate_correlation, "");
                                                                                xil_printf("  raw candidate lag    : %ld\r\n",                    (long)phase_results[phase].raw_candidate_lag);
                                                                                xil_printf("  raw coverage valid   : %s\r\n",                    phase_results[phase].raw_candidate_coverage_valid ?                    "YES" : "NO");
                                                                                if (phase_results[phase].alignment_success) {
                                                                                    print_float_value("  best valid correlation",                               phase_results[phase].correlation, "");
                                                                                    xil_printf("  best valid lag       : %ld\r\n",                        (long)phase_results[phase].integer_lag);
                                                                                }
                                                                                else {
                                                                                    xil_printf("  best valid candidate : NONE\r\n");
                                                                                }
                                                                            }
                                                                        }
                                                                        frame->even_candidate_correlation = phase_results[0].correlation;
                                                                        frame->odd_candidate_correlation = phase_results[1].correlation;
                                                                        frame->even_candidate_lag = phase_results[0].integer_lag;
                                                                        frame->odd_candidate_lag = phase_results[1].integer_lag;
                                                                        frame->even_candidate_valid = phase_results[0].alignment_success;
                                                                        frame->odd_candidate_valid = phase_results[1].alignment_success;
                                                                        if (phase_results[0].alignment_success &&         phase_results[1].alignment_success) {
                                                                            if (phase_results[0].correlation >= phase_results[1].correlation) {
                                                                                selected_phase = 0U;
                                                                                selected_result = &phase_results[0];
                                                                            }
                                                                            else {
                                                                                selected_phase = 1U;
                                                                                selected_result = &phase_results[1];
                                                                            }
                                                                        }
                                                                        else if (phase_results[0].alignment_success) {
                                                                            selected_phase = 0U;
                                                                            selected_result = &phase_results[0];
                                                                        }
                                                                        else if (phase_results[1].alignment_success) {
                                                                            selected_phase = 1U;
                                                                            selected_result = &phase_results[1];
                                                                        }
                                                                        else {
                                                                            if (reason != NULL) *reason = "neither fresh phase candidate aligned";
                                                                            return -4;
                                                                        }
                                                                        selected_candidate = phase_candidates[selected_phase];
                                                                        selected_available_count = reconstructed_count -         calibration_phase_source_offset(             (int)selected_phase, saved->canonical_reference_phase);
                                                                        frame->selected_reference_phase = (int8_t)selected_phase;
                                                                        frame->canonical_reference_phase = saved->canonical_reference_phase;
                                                                        frame->selected_phase_name = selected_phase == 0U ? "EVEN" : "ODD";
                                                                        frame->integer_lag = selected_result->integer_lag;
                                                                        frame->fractional_lag = selected_result->fractional_lag;
                                                                        frame->total_lag = selected_result->total_lag;
                                                                        frame->correlation = selected_result->correlation;
                                                                        if (!isfinite(selected_result->correlation)) {
                                                                            if (reason != NULL) *reason = "selected correlation is not finite";
                                                                            return -4;
                                                                        }
                                                                        if (selected_result->correlation < CAL_DAC_REF_MIN_CORRELATION) {
                                                                            if (reason != NULL) *reason = "correlation to stored adc -cal frame below threshold";
                                                                            return -4;
                                                                        }
                                                                        if (ADC_CAL_VERBOSE_DEBUG) {
                                                                            xil_printf("Canonical reference phase: %s\r\n",                saved->canonical_reference_phase == 0 ? "EVEN" : "ODD");
                                                                            xil_printf("Selected input phase    : %s\r\n",                selected_phase == 0U ? "EVEN" : "ODD");
                                                                            xil_printf("Source-index equation   : raw[k + phase_offset + lag + fraction]\r\n");
                                                                            print_double_value("First ADC source position",         (double)saved->calibration_window_start + selected_phase +             selected_result->total_lag, "");
                                                                            print_double_value("Last ADC source position",         (double)(saved->calibration_window_start +                  saved->calibration_window_length - 1U) +             selected_phase + selected_result->total_lag, "");
                                                                            calibration_print_mapping_test(saved, selected_candidate,         selected_available_count, selected_result->total_lag - 1.0,         gain_correction, offset_correction, "lag L-1");
                                                                            calibration_print_mapping_test(saved, selected_candidate,         selected_available_count, selected_result->total_lag,         gain_correction, offset_correction, "lag L");
                                                                            calibration_print_mapping_test(saved, selected_candidate,         selected_available_count, selected_result->total_lag + 1.0,         gain_correction, offset_correction, "lag L+1");
                                                                            if (fabsf(selected_result->fractional_lag) > FLT_EPSILON) {
                                                                                const double opposite_fraction_lag =             (double)selected_result->integer_lag -             selected_result->fractional_lag;
                                                                                calibration_print_mapping_test(saved, selected_candidate,             selected_available_count, opposite_fraction_lag,             gain_correction, offset_correction,             "opposite fractional sign");
                                                                            }
                                                                        }
                                                                        (void)use_saved_calibration_reference;
                                                                        count = saved->calibration_window_length;
                                                                        if (calibration_map_fixed_window(             saved->alignment_reference, selected_candidate,             selected_available_count,             (double)selected_result->total_lag,             saved->calibration_window_start, count,             workspace->aligned_reference, workspace->aligned_raw_adc,             &fit_state) != 0) {
                                                                            if (reason != NULL) *reason =             "fixed calibration window not fully covered";
                                                                            return -5;
                                                                        }
                                                                        for (size_t i = 0U;
                                                                        i < count;
                                                                        ++i) {
                                                                            const int16_t raw = workspace->aligned_raw_adc[i];
                                                                            const double corrected = (double)gain_correction *             calibration_apply_offset_correction(raw, offset_correction);
                                                                            const long code = lround(corrected);
                                                                            if (!isfinite(corrected) || code < CALIBRATION_ADC_MIN_CODE ||             code > CALIBRATION_ADC_MAX_CODE) {
                                                                                if (reason != NULL) *reason = "corrected ADC sample out of range";
                                                                                return -7;
                                                                            }
                                                                            {
                                                                                const long reference_code = lround(                 (double)saved->canonical_reference_window[i] *                 (double)reference_scale);
                                                                                if (reference_code < INT16_MIN || reference_code > INT16_MAX) {
                                                                                    if (reason != NULL) *reason = "scaled reference out of range";
                                                                                    return -7;
                                                                                }
                                                                                workspace->aligned_reference[i] = (int16_t)reference_code;
                                                                            }
                                                                            workspace->aligned_corrected_adc[i] = (int16_t)code;
                                                                            raw_sum += (double)raw;
                                                                        }
                                                                        calibration_default_config(&fit_config);
                                                                        status = calibration_init(&fit_state, &fit_config);
                                                                        if (status == CALIBRATION_OK)         status = calibration_analyze_frame(&fit_state,             workspace->aligned_corrected_adc, workspace->aligned_reference,             count);
                                                                        if (status != CALIBRATION_OK || !calibration_fit_metrics_are_valid(&fit_state)) {
                                                                            if (reason != NULL) *reason = "stored-reference regression failed";
                                                                            return -8;
                                                                        }
                                                                        frame->capture_succeeded = true;
                                                                        frame->reconstruction_succeeded = true;
                                                                        frame->frame_valid = true;
                                                                        frame->selected_channel = saved->selected_channel;
                                                                        frame->selected_channel_name = saved->selected_channel == 0 ? "Channel A" : "Channel B";
                                                                        frame->integer_lag = selected_result->integer_lag;
                                                                        frame->fractional_lag = selected_result->fractional_lag;
                                                                        frame->total_lag = selected_result->total_lag;
                                                                        frame->correlation = selected_result->correlation;
                                                                        frame->timing = *selected_result;
                                                                        frame->aligned_reference_samples = workspace->aligned_reference;
                                                                        frame->aligned_raw_adc_samples = workspace->aligned_raw_adc;
                                                                        frame->aligned_corrected_adc_samples = workspace->aligned_corrected_adc;
                                                                        frame->alignment_reference_samples = saved->alignment_reference;
                                                                        frame->canonical_reference_window =         saved->canonical_reference_window;
                                                                        frame->alignment_reference_count = saved->alignment_reference_count;
                                                                        frame->calibration_window_start = saved->calibration_window_start;
                                                                        frame->calibration_window_length = saved->calibration_window_length;
                                                                        frame->canonical_reference_checksum =         saved->canonical_reference_checksum;
                                                                        frame->canonical_reference_mean = saved->canonical_reference_mean;
                                                                        frame->analysis_reference_scale = reference_scale;
                                                                        frame->canonical_nominal_system_gain =         saved->canonical_nominal_system_gain;
                                                                        frame->valid_analysis_sample_count = count;
                                                                        frame->raw_aligned_adc_mean = (float)(raw_sum / (double)count);
                                                                        frame->metrics = fit_state.metrics;
                                                                        memset(&frame->overlap, 0, sizeof(frame->overlap));
                                                                        frame->overlap.reference_start = saved->calibration_window_start;
                                                                        frame->overlap.analysis_count = count;
                                                                        frame->overlap.overlap_count = count;
                                                                        frame->correlation = frame->metrics.correlation;
                                                                        if (!calibration_canonical_reference_is_valid(frame)) {
                                                                            if (reason != NULL) *reason =             "INTERNAL ALIGNMENT ERROR: canonical calibration reference changed";
                                                                            return -8;
                                                                        }
                                                                        if (frame->correlation < CAL_DAC_REF_MIN_CORRELATION) {
                                                                            if (reason != NULL) *reason =             "fixed-window correlation below threshold";
                                                                            return -8;
                                                                        }
                                                                        frame->rejection_reason = "none";
                                                                        if (reason != NULL) *reason = "none";
                                                                        return 0;
                                                                    }
                                                                    static int calibration_capture_against_stored_reference(     float offset_correction,     calibration_frame_workspace_t *workspace,     calibration_aligned_frame_t *frame,     const char **reason) {
                                                                        if (!calibration_stored_reference_is_compatible(reason)) return -1;
                                                                        return calibration_capture_against_owned_reference(         &g_stored_offset_reference, false, 1.0f, offset_correction, 1.0f,         workspace, frame, reason);
                                                                    }
                                                                    static int calibration_offset_model_residual(     const calibration_aligned_frame_t *frame,     float nominal_system_gain,     float *samplewise_mean,     float *mean_identity) {
                                                                        double residual_sum = 0.0;
                                                                        const size_t count = frame != NULL ?         frame->valid_analysis_sample_count : 0U;
                                                                        if (frame == NULL || samplewise_mean == NULL || mean_identity == NULL ||         frame->aligned_corrected_adc_samples == NULL ||         frame->canonical_reference_window == NULL ||         count != frame->calibration_window_length || count == 0U ||         !isfinite(nominal_system_gain))         return -1;
                                                                        for (size_t i = 0U;
                                                                        i < count;
                                                                        ++i) {
                                                                            const double predicted = (double)nominal_system_gain *             frame->canonical_reference_window[i];
                                                                            residual_sum +=             (double)frame->aligned_corrected_adc_samples[i] - predicted;
                                                                        }
                                                                        *samplewise_mean = (float)(residual_sum / (double)count);
                                                                        *mean_identity = frame->metrics.adc_mean -         nominal_system_gain * frame->canonical_reference_mean;
                                                                        return isfinite(*samplewise_mean) && isfinite(*mean_identity) ? 0 : -2;
                                                                    }
                                                                    static void calibration_print_interframe_difference(     const int16_t *previous,     const int16_t *current,     size_t count) {
                                                                        double difference_sum = 0.0, difference_square_sum = 0.0;
                                                                        double previous_sum = 0.0, current_sum = 0.0;
                                                                        double cross = 0.0, previous_power = 0.0, current_power = 0.0;
                                                                        for (size_t i = 0U;
                                                                        i < count;
                                                                        ++i) {
                                                                            const double difference = (double)current[i] - previous[i];
                                                                            difference_sum += difference;
                                                                            difference_square_sum += difference * difference;
                                                                            previous_sum += previous[i];
                                                                            current_sum += current[i];
                                                                        }
                                                                        previous_sum /= (double)count;
                                                                        current_sum /= (double)count;
                                                                        for (size_t i = 0U;
                                                                        i < count;
                                                                        ++i) {
                                                                            const double a = previous[i] - previous_sum;
                                                                            const double b = current[i] - current_sum;
                                                                            cross += a * b;
                                                                            previous_power += a * a;
                                                                            current_power += b * b;
                                                                        }
                                                                        print_float_value("Inter-frame mean difference",         (float)(difference_sum / (double)count), " codes");
                                                                        print_float_value("Inter-frame RMS difference",         (float)sqrt(difference_square_sum / (double)count), " codes");
                                                                        if (previous_power > CAL_REF_VARIANCE_EPSILON &&         current_power > CAL_REF_VARIANCE_EPSILON)         print_float_value("Inter-frame correlation",             (float)(cross / sqrt(previous_power * current_power)), "");
                                                                    }
                                                                    static int calibration_compute_offset_batch(     float offset_correction,     calibration_frame_workspace_t *workspace,     calibration_offset_loop_state_t *state,     calibration_offset_batch_result_t *batch) {
                                                                        double sum = 0.0;
                                                                        double sum_squares = 0.0;
                                                                        double correlation_sum = 0.0;
                                                                        double fitted_rmse_sum = 0.0;
                                                                        static int16_t adc_history[8][CAL_FIXED_WINDOW_LENGTH];
                                                                        int8_t history_phase[8] = {
                                                                            0}
                                                                            ;
                                                                            int32_t history_lag[8] = {
                                                                                0}
                                                                                ;
                                                                                float history_fractional_lag[8] = {
                                                                                    0.0f}
                                                                                    ;
                                                                                    bool history_valid[8] = {
                                                                                        false}
                                                                                        ;
                                                                                        size_t history_slot = 0U;
                                                                                        if (workspace == NULL || state == NULL || batch == NULL) return -1;
                                                                                        memset(batch, 0, sizeof(*batch));
                                                                                        batch->minimum = FLT_MAX;
                                                                                        batch->maximum = -FLT_MAX;
                                                                                        for (uint32_t frame_number = 1U;
                                                                                        frame_number <= CALIBRATION_OFFSET_BATCH_SIZE;
                                                                                        ++frame_number) {
                                                                                            calibration_aligned_frame_t frame;
                                                                                            const char *reason = NULL;
                                                                                            float residual;
                                                                                            float mean_identity;
                                                                                            int status;
                                                                                            if (!g_automatic_calibration.active || ADC_CAL_VERBOSE_DEBUG)             xil_printf("\r\nFrame %lu/%u\r\n",                        (unsigned long)frame_number,                        CALIBRATION_OFFSET_BATCH_SIZE);
                                                                                            status = calibration_capture_against_stored_reference(             offset_correction, workspace, &frame, &reason);
                                                                                            if (ADC_CAL_VERBOSE_DEBUG) {
                                                                                                print_float_value("EVEN correlation",                           frame.even_candidate_correlation, "");
                                                                                                print_float_value("ODD correlation",                           frame.odd_candidate_correlation, "");
                                                                                                if (frame.even_candidate_valid || frame.odd_candidate_valid) {
                                                                                                    xil_printf("Selected input phase    : %s\r\n",                        frame.selected_phase_name);
                                                                                                    print_float_value("Selected correlation", frame.correlation, "");
                                                                                                    xil_printf("Selected integer lag    : %ld samples\r\n",                        (long)frame.integer_lag);
                                                                                                    print_float_value("Selected fractional lag",                               frame.fractional_lag, " samples");
                                                                                                }
                                                                                            }
                                                                                            if (status != 0 || !frame.frame_valid ||             !isfinite(frame.metrics.adc_mean) ||             !isfinite(frame.metrics.fitted_rmse_codes) ||             !isfinite(frame.correlation)) {
                                                                                                ++batch->rejected;
                                                                                                ++state->rejected_frame_count;
                                                                                                if (!g_automatic_calibration.active ||                 ADC_CAL_VERBOSE_DEBUG) {
                                                                                                    xil_printf("Status                  : REJECTED\r\n");
                                                                                                    xil_printf("Rejection reason        : %s\r\n",                     reason != NULL ? reason : "invalid frame metrics");
                                                                                                }
                                                                                            }
                                                                                            else {
                                                                                                if (calibration_offset_model_residual(                     &frame,                     g_stored_offset_reference.canonical_nominal_system_gain,                     &residual, &mean_identity) != 0) {
                                                                                                    ++batch->rejected;
                                                                                                    ++state->rejected_frame_count;
                                                                                                    if (!g_automatic_calibration.active ||                     ADC_CAL_VERBOSE_DEBUG) {
                                                                                                        xil_printf("Status                  : REJECTED\r\n");
                                                                                                        xil_printf("Rejection reason        : nonfinite residual\r\n");
                                                                                                    }
                                                                                                }
                                                                                                else {
                                                                                                    calibration_dither_offset_diagnostic_t dither_diagnostic;
                                                                                                    ++batch->accepted;
                                                                                                    ++state->accepted_frame_count;
                                                                                                    if (ADC_CAL_VERBOSE_DEBUG)                     calibration_print_fixed_window(&frame);
                                                                                                    if (calibration_estimate_dither_offset(                     &frame, residual, &dither_diagnostic) == 0 &&                 dither_diagnostic.valid) {
                                                                                                        batch->dither_latest = dither_diagnostic;
                                                                                                        g_latest_dither_offset_diagnostic = dither_diagnostic;
                                                                                                        ++batch->dither_valid_estimates;
                                                                                                        if (dither_diagnostic.status ==                         CAL_DITHER_OFFSET_STATUS_PASS)                     ++batch->dither_pass;
                                                                                                        else                     ++batch->dither_warning;
                                                                                                        batch->mean_dither_offset +=                     dither_diagnostic.dither_offset_codes;
                                                                                                        batch->mean_existing_dither_delta +=                     dither_diagnostic.existing_vs_dither_codes;
                                                                                                    }
                                                                                                    else {
                                                                                                        batch->dither_latest = dither_diagnostic;
                                                                                                        g_latest_dither_offset_diagnostic = dither_diagnostic;
                                                                                                        ++batch->dither_invalid;
                                                                                                    }
                                                                                                    sum += residual;
                                                                                                    sum_squares += (double)residual * (double)residual;
                                                                                                    correlation_sum += frame.correlation;
                                                                                                    fitted_rmse_sum += frame.metrics.fitted_rmse_codes;
                                                                                                    if (residual < batch->minimum) batch->minimum = residual;
                                                                                                    if (residual > batch->maximum) batch->maximum = residual;
                                                                                                    state->latest_fitted_offset = frame.metrics.measured_offset;
                                                                                                    state->latest_fitted_gain = frame.metrics.measured_gain;
                                                                                                    state->latest_raw_mean = frame.raw_aligned_adc_mean;
                                                                                                    state->latest_corrected_mean = frame.metrics.adc_mean;
                                                                                                    if (ADC_CAL_VERBOSE_DEBUG) {
                                                                                                        xil_printf("Offset residual equation: mean(ADC[k] - G_nominal * reference[k])\r\n");
                                                                                                        print_float_value("Nominal system gain",                     g_stored_offset_reference.canonical_nominal_system_gain,                     "");
                                                                                                        print_float_value("Sample-wise residual mean",                                   residual, " codes");
                                                                                                        print_float_value("Mean-identity residual",                                   mean_identity, " codes");
                                                                                                        calibration_print_dither_offset_diagnostic(                     &dither_diagnostic);
                                                                                                    }
                                                                                                    if (ADC_CAL_VERBOSE_DEBUG) {
                                                                                                        for (size_t h = 0U;
                                                                                                        h < 8U;
                                                                                                        ++h) {
                                                                                                            if (history_valid[h] &&                         history_phase[h] == frame.selected_reference_phase &&                         history_lag[h] == frame.integer_lag &&                         fabsf(history_fractional_lag[h] -                               frame.fractional_lag) <= 0.05f) {
                                                                                                                calibration_print_interframe_difference(                             adc_history[h],                             frame.aligned_corrected_adc_samples,                             frame.valid_analysis_sample_count);
                                                                                                                break;
                                                                                                            }
                                                                                                        }
                                                                                                        memcpy(adc_history[history_slot],                     frame.aligned_corrected_adc_samples,                     frame.valid_analysis_sample_count *                         sizeof(adc_history[0][0]));
                                                                                                        history_phase[history_slot] =                     frame.selected_reference_phase;
                                                                                                        history_lag[history_slot] = frame.integer_lag;
                                                                                                        history_fractional_lag[history_slot] =                     frame.fractional_lag;
                                                                                                        history_valid[history_slot] = true;
                                                                                                        history_slot = (history_slot + 1U) % 8U;
                                                                                                    }
                                                                                                    if (!g_automatic_calibration.active ||                     ADC_CAL_VERBOSE_DEBUG) {
                                                                                                        xil_printf("Selected input phase    : %s\r\n",                         frame.selected_reference_phase == 0 ? "EVEN" : "ODD");
                                                                                                        xil_printf("Integer lag             : %ld samples\r\n",                         (long)frame.integer_lag);
                                                                                                        print_float_value("Fractional lag",                         frame.fractional_lag, " samples");
                                                                                                        print_float_value("Correlation", frame.correlation, "");
                                                                                                        print_float_value("Residual", residual, " codes");
                                                                                                        xil_printf("Status                  : ACCEPTED\r\n");
                                                                                                    }
                                                                                                }
                                                                                            }
                                                                                            if (frame_number < CALIBRATION_OFFSET_BATCH_SIZE)             usleep(ADC_TIMING_INTERFRAME_DELAY_US);
                                                                                        }
                                                                                        if (batch->accepted == 0U) return -2;
                                                                                        batch->mean = (float)(sum / (double)batch->accepted);
                                                                                        batch->rmse = (float)sqrt(sum_squares / (double)batch->accepted);
                                                                                        batch->stddev = (float)sqrt(fmax(0.0,         sum_squares / (double)batch->accepted -         (double)batch->mean * (double)batch->mean));
                                                                                        batch->mean_correlation =         (float)(correlation_sum / (double)batch->accepted);
                                                                                        batch->mean_fitted_rmse =         (float)(fitted_rmse_sum / (double)batch->accepted);
                                                                                        if (batch->dither_valid_estimates > 0U) {
                                                                                            batch->mean_dither_offset /=         (double)batch->dither_valid_estimates;
                                                                                            batch->mean_existing_dither_delta /=         (double)batch->dither_valid_estimates;
                                                                                        }
                                                                                        return 0;
                                                                                    }
                                                                                    /* Gain-stage estimator.  Offset calibration stores the additive correction  * C = -O, so the fixed-offset signal is gain_correction * (raw + C).  * Centering both signals keeps any small residual DC error out of the AC gain  * estimate; that residual is reported separately and never fed back. */
                                                                                    static int calibration_estimate_gain_fixed_offset(     const calibration_aligned_frame_t *frame,     float gain_correction,     float fixed_offset_correction,     calibration_fixed_offset_gain_metrics_t *metrics) {
                                                                                        double raw_sum = 0.0;
                                                                                        double corrected_sum = 0.0;
                                                                                        double reference_sum = 0.0;
                                                                                        double numerator = 0.0;
                                                                                        double denominator = 0.0;
                                                                                        double residual_sum = 0.0;
                                                                                        double residual_square_sum = 0.0;
                                                                                        double raw_mean;
                                                                                        double corrected_mean;
                                                                                        double reference_mean;
                                                                                        double measured_gain;
                                                                                        const size_t count = frame != NULL ?         frame->valid_analysis_sample_count : 0U;
                                                                                        if (frame == NULL || !frame->frame_valid || metrics == NULL ||         frame->aligned_raw_adc_samples == NULL ||         frame->aligned_reference_samples == NULL ||         count < CAL_MIN_ANALYSIS_SAMPLES ||         count != frame->calibration_window_length ||         frame->overlap.reference_start !=             frame->calibration_window_start ||         !isfinite(gain_correction) || gain_correction <= 0.0f ||         !isfinite(fixed_offset_correction))         return -1;
                                                                                        memset(metrics, 0, sizeof(*metrics));
                                                                                        for (size_t i = 0U;
                                                                                        i < count;
                                                                                        ++i) {
                                                                                            const double raw = frame->aligned_raw_adc_samples[i];
                                                                                            const double corrected = gain_correction *             calibration_apply_offset_correction(                 raw, fixed_offset_correction);
                                                                                            const double reference = frame->aligned_reference_samples[i];
                                                                                            raw_sum += raw;
                                                                                            corrected_sum += corrected;
                                                                                            reference_sum += reference;
                                                                                        }
                                                                                        raw_mean = raw_sum / (double)count;
                                                                                        corrected_mean = corrected_sum / (double)count;
                                                                                        reference_mean = reference_sum / (double)count;
                                                                                        for (size_t i = 0U;
                                                                                        i < count;
                                                                                        ++i) {
                                                                                            const double corrected = gain_correction *             calibration_apply_offset_correction(                 frame->aligned_raw_adc_samples[i],                 fixed_offset_correction);
                                                                                            const double x =             (double)frame->aligned_reference_samples[i] - reference_mean;
                                                                                            const double y = corrected - corrected_mean;
                                                                                            numerator += x * y;
                                                                                            denominator += x * x;
                                                                                        }
                                                                                        if (!isfinite(numerator) || !isfinite(denominator) ||         denominator <= CAL_REF_VARIANCE_EPSILON)         return -2;
                                                                                        measured_gain = numerator / denominator;
                                                                                        for (size_t i = 0U;
                                                                                        i < count;
                                                                                        ++i) {
                                                                                            const double corrected = gain_correction *             calibration_apply_offset_correction(                 frame->aligned_raw_adc_samples[i],                 fixed_offset_correction);
                                                                                            const double predicted = measured_gain *             (double)frame->aligned_reference_samples[i];
                                                                                            const double residual = corrected - predicted;
                                                                                            residual_sum += residual;
                                                                                            residual_square_sum += residual * residual;
                                                                                        }
                                                                                        metrics->raw_system_gain = (float)measured_gain;
                                                                                        metrics->raw_adc_mean = (float)raw_mean;
                                                                                        metrics->corrected_adc_mean = (float)corrected_mean;
                                                                                        metrics->reference_mean = (float)reference_mean;
                                                                                        metrics->residual_offset =         (float)(residual_sum / (double)count);
                                                                                        metrics->rmse =         (float)sqrt(residual_square_sum / (double)count);
                                                                                        return isfinite(metrics->raw_system_gain) &&            isfinite(metrics->raw_adc_mean) &&            isfinite(metrics->corrected_adc_mean) &&            isfinite(metrics->reference_mean) &&            isfinite(metrics->residual_offset) &&            isfinite(metrics->rmse) ? 0 : -3;
                                                                                    }
                                                                                    static int calibration_compute_gain_batch(     const calibration_pending_frame_t *input,     float gain_correction,     float fixed_offset_correction,     float nominal_system_gain,     calibration_frame_workspace_t *workspace,     calibration_gain_loop_state_t *state,     calibration_gain_batch_result_t *batch,     bool use_offset_output_first) {
                                                                                        double raw_gain_sum = 0.0;
                                                                                        double gain_sum = 0.0, gain_square_sum = 0.0;
                                                                                        double gain_rmse_sum = 0.0;
                                                                                        double corrected_adc_sum = 0.0;
                                                                                        double correlation_sum = 0.0;
                                                                                        bool diagnostic_printed = false;
                                                                                        if (input == NULL || workspace == NULL || state == NULL || batch == NULL ||         !isfinite(nominal_system_gain) || nominal_system_gain <= FLT_EPSILON)         return -1;
                                                                                        memset(batch, 0, sizeof(*batch));
                                                                                        for (uint32_t frame_number = 1U;
                                                                                        frame_number <= CALIBRATION_GAIN_BATCH_SIZE;
                                                                                        ++frame_number) {
                                                                                            calibration_aligned_frame_t frame;
                                                                                            calibration_fixed_offset_gain_metrics_t gain_metrics;
                                                                                            const char *reason = NULL;
                                                                                            int status;
                                                                                            if (!g_automatic_calibration.active || ADC_CAL_VERBOSE_DEBUG)             xil_printf("\r\nGain frame %lu/%u\r\n",                        (unsigned long)frame_number,                        CALIBRATION_GAIN_BATCH_SIZE);
                                                                                            if (use_offset_output_first && frame_number == 1U) {
                                                                                                status = calibration_pending_frame_consume(                 gain_correction, fixed_offset_correction,                 CAL_ADC_FULL_SCALE_CODES / CAL_DAC_FULL_SCALE_CODES,                 workspace, &frame, &reason);
                                                                                                if (!g_automatic_calibration.active ||                 ADC_CAL_VERBOSE_DEBUG)                 xil_printf("Input source            : stored offset output\r\n");
                                                                                            }
                                                                                            else {
                                                                                                status = calibration_capture_against_owned_reference(                 input, true, gain_correction,                 fixed_offset_correction,                 CAL_ADC_FULL_SCALE_CODES / CAL_DAC_FULL_SCALE_CODES,                 workspace, &frame, &reason);
                                                                                                if (!g_automatic_calibration.active ||                 ADC_CAL_VERBOSE_DEBUG)                 xil_printf("Input source            : fresh gain capture\r\n");
                                                                                            }
                                                                                            if (status == 0 && frame.frame_valid)             status = calibration_estimate_gain_fixed_offset(                 &frame, gain_correction, fixed_offset_correction,                 &gain_metrics);
                                                                                            if (status != 0 || !frame.frame_valid ||             !isfinite(gain_metrics.raw_system_gain) ||             !isfinite(gain_metrics.residual_offset) ||             !isfinite(gain_metrics.rmse) ||             !isfinite(frame.correlation)) {
                                                                                                ++batch->rejected;
                                                                                                ++state->rejected_frame_count;
                                                                                                if (!g_automatic_calibration.active ||                 ADC_CAL_VERBOSE_DEBUG) {
                                                                                                    xil_printf("Status                  : REJECTED\r\n");
                                                                                                    xil_printf("Reason                  : %s\r\n",                     reason != NULL ? reason : "invalid gain metrics");
                                                                                                }
                                                                                            }
                                                                                            else {
                                                                                                const float raw_system_gain = gain_metrics.raw_system_gain;
                                                                                                const float normalized_gain =                 raw_system_gain / nominal_system_gain;
                                                                                                calibration_dither_gain_diagnostic_t dither_diagnostic;
                                                                                                ++batch->accepted;
                                                                                                ++state->accepted_frame_count;
                                                                                                if (ADC_CAL_VERBOSE_DEBUG)                 calibration_print_fixed_window(&frame);
                                                                                                if (calibration_estimate_dither_gain_diagnostic(                 &frame, normalized_gain, nominal_system_gain,                 &dither_diagnostic) == 0 &&             dither_diagnostic.valid) {
                                                                                                    batch->dither_latest = dither_diagnostic;
                                                                                                    g_latest_dither_gain_diagnostic = dither_diagnostic;
                                                                                                    ++batch->dither_valid_estimates;
                                                                                                    if (dither_diagnostic.status ==                     CAL_DITHER_GAIN_STATUS_PASS)                 ++batch->dither_pass;
                                                                                                    else                 ++batch->dither_warning;
                                                                                                    batch->mean_dither_gain +=                 dither_diagnostic.dither_full_gain;
                                                                                                    batch->mean_dither_flat_gain +=                 dither_diagnostic.dither_flat_gain;
                                                                                                    batch->mean_existing_dither_delta +=                 dither_diagnostic.existing_vs_dither_gain;
                                                                                                }
                                                                                                else {
                                                                                                    batch->dither_latest = dither_diagnostic;
                                                                                                    g_latest_dither_gain_diagnostic = dither_diagnostic;
                                                                                                    ++batch->dither_invalid;
                                                                                                }
                                                                                                raw_gain_sum += raw_system_gain;
                                                                                                gain_sum += normalized_gain;
                                                                                                gain_square_sum +=                 (double)normalized_gain * (double)normalized_gain;
                                                                                                gain_rmse_sum += gain_metrics.rmse;
                                                                                                corrected_adc_sum += gain_metrics.corrected_adc_mean;
                                                                                                correlation_sum += frame.correlation;
                                                                                                if (!diagnostic_printed &&                 (!g_automatic_calibration.active ||                  ADC_CAL_VERBOSE_DEBUG)) {
                                                                                                    diagnostic_printed = true;
                                                                                                    xil_printf("\r\nFixed-offset frame diagnostic:\r\n");
                                                                                                    print_float_value("Raw ADC mean",                                   gain_metrics.raw_adc_mean, " codes");
                                                                                                    print_float_value("Applied fixed offset correction",                                   fixed_offset_correction, " codes");
                                                                                                    print_float_value("Corrected ADC mean",                                   gain_metrics.corrected_adc_mean, " codes");
                                                                                                    print_float_value("Reference mean",                                   gain_metrics.reference_mean, " codes");
                                                                                                    xil_printf("Fresh gain ADC data is : RAW\r\n");
                                                                                                    xil_printf("Offset correction applied at: software estimator\r\n");
                                                                                                }
                                                                                                if (!g_automatic_calibration.active ||                 ADC_CAL_VERBOSE_DEBUG) {
                                                                                                    print_float_value("Raw system gain", raw_system_gain, "");
                                                                                                    print_float_value("Nominal system gain",                                   nominal_system_gain, "");
                                                                                                    print_float_value("Normalized ADC gain", normalized_gain, "");
                                                                                                    print_float_value("Gain error", normalized_gain - 1.0f, "");
                                                                                                    print_float_value("Fixed offset correction",                                   fixed_offset_correction, " codes");
                                                                                                    print_float_value("Raw ADC mean",                                   gain_metrics.raw_adc_mean, " codes");
                                                                                                    print_float_value("Offset-corrected ADC mean",                                   gain_metrics.corrected_adc_mean, " codes");
                                                                                                    print_float_value("Reference mean",                                   gain_metrics.reference_mean, " codes");
                                                                                                    print_float_value("Gain RMSE", gain_metrics.rmse, " codes");
                                                                                                    print_float_value("Correlation", frame.correlation, "");
                                                                                                    calibration_print_dither_gain_diagnostic(                 &dither_diagnostic);
                                                                                                    xil_printf("Status                  : ACCEPTED\r\n");
                                                                                                }
                                                                                            }
                                                                                            if (frame_number < CALIBRATION_GAIN_BATCH_SIZE)             usleep(ADC_TIMING_INTERFRAME_DELAY_US);
                                                                                        }
                                                                                        if (batch->accepted == 0U) return -2;
                                                                                        batch->mean_raw_system_gain =         (float)(raw_gain_sum / (double)batch->accepted);
                                                                                        batch->mean_gain = (float)(gain_sum / (double)batch->accepted);
                                                                                        batch->mean_error = batch->mean_gain - 1.0f;
                                                                                        batch->stddev = (float)sqrt(fmax(0.0,         gain_square_sum / (double)batch->accepted -         (double)batch->mean_gain * (double)batch->mean_gain));
                                                                                        batch->standard_error =         batch->stddev / sqrtf((float)batch->accepted);
                                                                                        batch->mean_gain_rmse =         (float)(gain_rmse_sum / (double)batch->accepted);
                                                                                        batch->mean_corrected_adc =         (float)(corrected_adc_sum / (double)batch->accepted);
                                                                                        batch->mean_correlation =         (float)(correlation_sum / (double)batch->accepted);
                                                                                        if (batch->dither_valid_estimates > 0U) {
                                                                                            batch->mean_dither_gain /=         (double)batch->dither_valid_estimates;
                                                                                            batch->mean_dither_flat_gain /=         (double)batch->dither_valid_estimates;
                                                                                            batch->mean_existing_dither_delta /=         (double)batch->dither_valid_estimates;
                                                                                        }
                                                                                        return 0;
                                                                                    }
                                                                                    /*  * Publish a fresh representative capture measured with the final coefficient.  * The stored reference remains at its canonical adc -cal scale so a later  * calibration stage can apply its own stage-specific scaling exactly once.  */
                                                                                    static int calibration_publish_final_capture(     const char *stage_name,     const int16_t *even_reference,     const int16_t *odd_reference,     size_t reference_count,     const calibration_frame_config_t *config,     calibration_frame_workspace_t *workspace) {
                                                                                        static calibration_pending_frame_t         candidates[CAL_UPDATE_FRAME_BATCH_SIZE];
                                                                                        calibration_aligned_frame_t selected_frame;
                                                                                        calibration_selection_medians_t medians;
                                                                                        size_t selected_index = 0U;
                                                                                        uint32_t accepted_count = 0U;
                                                                                        const char *reason = NULL;
                                                                                        int status;
                                                                                        const bool compact = calibration_compact_output_enabled();
                                                                                        calibration_gain_input_frame_invalidate();
                                                                                        if (!compact) {
                                                                                            xil_printf("\r\n========== Calibrated Capture Output ==========\r\n");
                                                                                            xil_printf("Source stage             : %s\r\n",                    stage_name != NULL ? stage_name : "unknown");
                                                                                            xil_printf("Capture timing           : after final coefficient update\r\n");
                                                                                        }
                                                                                        (void)even_reference;
                                                                                        (void)odd_reference;
                                                                                        (void)reference_count;
                                                                                        status = -1;
                                                                                        for (uint32_t i = 0U;
                                                                                        i < CAL_UPDATE_FRAME_BATCH_SIZE;
                                                                                        ++i) {
                                                                                            calibration_aligned_frame_t frame;
                                                                                            usleep(ADC_TIMING_INTERFRAME_DELAY_US);
                                                                                            if (calibration_capture_against_owned_reference(                 &g_stored_offset_reference, false,                 config->adc_gain_correction,                 config->adc_offset_correction,                 config->reference_scale,                 workspace, &frame, &reason) == 0 &&             frame.frame_valid &&             calibration_pending_frame_copy(                 &candidates[accepted_count], &frame, i + 1U,                 reference_buffer_generation(), reference_buffer_length(),                 reference_buffer_format()) == 0) {
                                                                                                ++accepted_count;
                                                                                            }
                                                                                        }
                                                                                        if (accepted_count >= CAL_TIMING_MIN_ACCEPTED_FRAMES &&         calibration_select_representative_frame(             candidates, accepted_count, &selected_index, &medians) == 0 &&         calibration_restore_owned_frame(             &candidates[selected_index], workspace, &selected_frame) == 0)         status = 0;
                                                                                        if (!compact)         xil_printf("Accepted output frames   : %lu/%u\r\n",                    (unsigned long)accepted_count,                    CAL_UPDATE_FRAME_BATCH_SIZE);
                                                                                        if ((status != 0) || !selected_frame.frame_valid) {
                                                                                            if (compact)             xil_printf("Calibrated capture output: FAILED\r\n");
                                                                                            else {
                                                                                                xil_printf("Pending input frame      : none\r\n");
                                                                                                xil_printf("Capture output status    : NOT READY\r\n");
                                                                                            }
                                                                                            xil_printf("Reason                   : %s\r\n",                    reason != NULL ? reason : "final capture failed");
                                                                                            if (!compact)             xil_printf("===============================================\r\n");
                                                                                            return -1;
                                                                                        }
                                                                                        status = calibration_pending_frame_copy(         &g_pending_calibration_frame, &selected_frame,         selected_frame.retained_frame_number,         reference_buffer_generation(), reference_buffer_length(),         reference_buffer_format()     );
                                                                                        if (status != 0) {
                                                                                            calibration_gain_input_frame_invalidate();
                                                                                            if (compact)             xil_printf("Calibrated capture output: FAILED\r\n");
                                                                                            else {
                                                                                                xil_printf("Pending input frame      : none\r\n");
                                                                                                xil_printf("Capture output status    : NOT READY\r\n");
                                                                                            }
                                                                                            xil_printf("Reason                   : pending-frame copy failed\r\n");
                                                                                            if (!compact)             xil_printf("===============================================\r\n");
                                                                                            return -2;
                                                                                        }
                                                                                        if (!compact) {
                                                                                            xil_printf("Pending input frame      : Frame %lu\r\n",                    (unsigned long)selected_frame.retained_frame_number);
                                                                                            xil_printf("Selection reason         : Closest to median calibration metrics\r\n");
                                                                                            xil_printf("Channel                  : %s\r\n",                    selected_frame.selected_channel_name);
                                                                                            xil_printf("Canonical reference phase: %s\r\n",                    selected_frame.canonical_reference_phase == 0 ?                    "EVEN" : "ODD");
                                                                                            xil_printf("Selected input phase     : %s\r\n",                    selected_frame.selected_phase_name);
                                                                                            print_float_value("Correlation", selected_frame.correlation, "");
                                                                                            xil_printf("Integer lag              : %ld samples\r\n",                    (long)selected_frame.integer_lag);
                                                                                            print_float_value("Fractional lag",                           selected_frame.fractional_lag, " samples");
                                                                                            xil_printf("Valid aligned samples    : %lu\r\n",                    (unsigned long)selected_frame.valid_analysis_sample_count);
                                                                                            xil_printf("Capture output status    : READY\r\n");
                                                                                            xil_printf("===============================================\r\n");
                                                                                        }
                                                                                        return 0;
                                                                                    }
                                                                                    static const char *calibration_offset_termination_name(uint8_t reason) {
                                                                                        switch ((calibration_offset_termination_t)reason) {
                                                                                            case CAL_OFFSET_TERMINATION_CONVERGED:         return "converged";
                                                                                            case CAL_OFFSET_TERMINATION_NO_IMPROVEMENT:         return "no further improvement";
                                                                                            case CAL_OFFSET_TERMINATION_ITERATION_LIMIT:         return "iteration limit";
                                                                                            case CAL_OFFSET_TERMINATION_VERIFICATION_FAILED:         return "verification exceeds provisional limit";
                                                                                            case CAL_OFFSET_TERMINATION_ERROR:         return "stage error";
                                                                                            default:         return "not terminated";
                                                                                        }
                                                                                    }
                                                                                    static const char *calibration_offset_verification_name(uint8_t status) {
                                                                                        switch ((calibration_offset_verification_status_t)status) {
                                                                                            case CAL_OFFSET_VERIFICATION_PASS: return "PASS";
                                                                                            case CAL_OFFSET_VERIFICATION_MARGINAL: return "MARGINAL";
                                                                                            case CAL_OFFSET_VERIFICATION_FAIL: return "FAIL";
                                                                                            default: return "NOT RUN";
                                                                                        }
                                                                                    }
                                                                                    static void calibration_offset_loop_print_summary(     const calibration_offset_loop_state_t *state ) {
                                                                                        if (calibration_compact_output_enabled()) {
                                                                                            xil_printf("\r\nOffset controller       : %s\r\n",                    state->controller_converged ? "CONVERGED" : "FAILED");
                                                                                            xil_printf("Existing offset loop status : %s\r\n",                    calibration_existing_offset_loop_status_name(state));
                                                                                            xil_printf("Dither estimator status     : %s\r\n",                    calibration_dither_offset_status_name(                        g_latest_dither_offset_diagnostic.status));
                                                                                            xil_printf("Verification            : %s\r\n",                    calibration_offset_verification_name(                        state->verification_status));
                                                                                            xil_printf("Verification frames     : %lu\r\n",                    (unsigned long)state->verification_accepted_frames);
                                                                                            print_float_value("Verification residual",                           state->verification_residual, " codes");
                                                                                            print_float_value("Verification standard error",                           state->verification_standard_error, " codes");
                                                                                            print_float_value("Verification correlation",                           state->verification_correlation, "");
                                                                                            print_float_value("Offset correction",                           state->offset_correction, " codes");
                                                                                            xil_printf("Status                  : %s\r\n",                    calibration_offset_result_name(state->stage_result));
                                                                                            if (state->stage_result == CALIBRATION_OFFSET_RESULT_FAILED)             xil_printf("Reason                  : %s\r\n",                 calibration_offset_termination_name(                     state->termination_reason));
                                                                                            calibration_print_dither_offset_diagnostic(         &g_latest_dither_offset_diagnostic);
                                                                                            return;
                                                                                        }
                                                                                        xil_printf("\r\n========== Offset Calibration Summary ==========\r\n");
                                                                                        xil_printf("Iterations completed     : %lu\r\n",                (unsigned long)state->batch_iteration_count);
                                                                                        xil_printf("Accepted frames          : %lu\r\n",                (unsigned long)state->accepted_frame_count);
                                                                                        xil_printf("Rejected frames          : %lu\r\n",                (unsigned long)state->rejected_frame_count);
                                                                                        xil_printf("Calibration channel      : %s\r\n",                calibration_channel_name(state->calibration_channel));
                                                                                        print_float_value("Final applied correction",                       state->offset_correction, " codes");
                                                                                        print_float_value("Final batch residual",                       state->latest_mean_residual, " codes");
                                                                                        print_float_value("Final filtered residual",                       state->filtered_residual, " codes");
                                                                                        print_float_value("Final batch RMSE",                       state->latest_batch_rmse, " codes");
                                                                                        print_float_value("Best residual",                       state->best_filtered_residual, " codes");
                                                                                        print_float_value("Best raw residual",                       state->best_raw_residual, " codes");
                                                                                        print_float_value("Best correction",                       state->best_offset_correction, " codes");
                                                                                        print_float_value("Best RMSE", state->best_rmse, " codes");
                                                                                        print_float_value("Residual standard deviation",                       state->latest_residual_stddev, " codes");
                                                                                        print_float_value("Residual standard error",                       state->latest_standard_error, " codes");
                                                                                        print_float_value("Controller gain",                       state->latest_controller_gain, "");
                                                                                        print_float_value("Filter alpha",                       CALIBRATION_OFFSET_FILTER_ALPHA, "");
                                                                                        print_float_value("Verification residual",                       state->verification_residual, " codes");
                                                                                        print_float_value("Verification correlation",                       state->verification_correlation, "");
                                                                                        print_float_value("Verification std dev",                       state->verification_stddev, " codes");
                                                                                        print_float_value("Verification standard error",                       state->verification_standard_error, " codes");
                                                                                        xil_printf("Verification frames     : %lu\r\n",                (unsigned long)state->verification_accepted_frames);
                                                                                        xil_printf("Verification status     : %s\r\n",                calibration_offset_verification_name(                    state->verification_status));
                                                                                        xil_printf("Controller converged    : %s\r\n",                state->controller_converged ? "YES" : "NO");
                                                                                        xil_printf("Existing offset loop status : %s\r\n",                calibration_existing_offset_loop_status_name(state));
                                                                                        xil_printf("Dither estimator status     : %s\r\n",                calibration_dither_offset_status_name(                    g_latest_dither_offset_diagnostic.status));
                                                                                        xil_printf("Consecutive passes      : %lu/%u\r\n",                (unsigned long)state->convergence_count,                CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES);
                                                                                        xil_printf("Termination reason       : %s\r\n",                calibration_offset_termination_name(state->termination_reason));
                                                                                        xil_printf("Offset stage status      : %s\r\n",                calibration_offset_result_name(state->stage_result));
                                                                                        xil_printf("Output usable for gain   : %s\r\n",                state->stage_result == CALIBRATION_OFFSET_RESULT_CONVERGED ||                state->stage_result == CALIBRATION_OFFSET_RESULT_PROVISIONAL ?                "YES" : "NO");
                                                                                        calibration_print_dither_offset_diagnostic(         &g_latest_dither_offset_diagnostic);
                                                                                        xil_printf("===============================================\r\n");
                                                                                    }
                                                                                    static void handle_adc_offset_calibration_status_cmd(void) {
                                                                                        const calibration_offset_loop_state_t *state =         calibration_offset_loop_state();
                                                                                        xil_printf("\r\n========== ADC Offset Calibration Status ==========\r\n");
                                                                                        print_float_value("Offset correction",                       state->offset_correction, " codes");
                                                                                        print_float_value("Gain correction",                       calibration_software_gain_correction(), "");
                                                                                        xil_printf("Accepted frames        : %lu\r\n",                (unsigned long)state->accepted_frame_count);
                                                                                        xil_printf("Rejected frames        : %lu\r\n",                (unsigned long)state->rejected_frame_count);
                                                                                        xil_printf("Consecutive passes     : %lu\r\n",                (unsigned long)state->convergence_count);
                                                                                        xil_printf("Batch iterations       : %lu\r\n",                (unsigned long)state->batch_iteration_count);
                                                                                        xil_printf("Calibration channel    : %s\r\n",                calibration_channel_name(state->calibration_channel));
                                                                                        print_float_value("Latest correlation",                       state->latest_correlation, "");
                                                                                        print_float_value("Latest mean aligned residual",                       state->latest_mean_residual, " codes");
                                                                                        print_float_value("Latest residual std dev",                       state->latest_residual_stddev, " codes");
                                                                                        print_float_value("Latest batch RMSE",                       state->latest_batch_rmse, " codes");
                                                                                        print_float_value("Latest fitted gain",                       state->latest_fitted_gain, "");
                                                                                        print_float_value("Latest fitted offset",                       state->latest_fitted_offset, " codes");
                                                                                        print_float_value("Latest fitted RMSE",                       state->latest_rmse, " codes");
                                                                                        xil_printf("Controller converged   : %s\r\n",                state->controller_converged ? "YES" : "NO");
                                                                                        xil_printf("Existing offset loop status: %s\r\n",                calibration_existing_offset_loop_status_name(state));
                                                                                        xil_printf("Dither estimator status    : %s\r\n",                calibration_dither_offset_status_name(                    g_latest_dither_offset_diagnostic.status));
                                                                                        xil_printf("Verification status    : %s\r\n",                calibration_offset_verification_name(                    state->verification_status));
                                                                                        xil_printf("Offset stage status    : %s\r\n",                calibration_offset_result_name(state->stage_result));
                                                                                        xil_printf("==================================================\r\n");
                                                                                    }
                                                                                    static float calibration_stability_correlation(     const calibration_offset_stability_record_t *records,     size_t count, bool use_fractional_lag) {
                                                                                        double x_sum = 0.0, y_sum = 0.0;
                                                                                        double cross = 0.0, x_power = 0.0, y_power = 0.0;
                                                                                        if (records == NULL || count < 2U) return 0.0f;
                                                                                        for (size_t i = 0U;
                                                                                        i < count;
                                                                                        ++i) {
                                                                                            x_sum += use_fractional_lag ? records[i].fractional_lag :                                      records[i].integer_lag;
                                                                                            y_sum += records[i].residual;
                                                                                        }
                                                                                        x_sum /= (double)count;
                                                                                        y_sum /= (double)count;
                                                                                        for (size_t i = 0U;
                                                                                        i < count;
                                                                                        ++i) {
                                                                                            const double x = (use_fractional_lag ?             records[i].fractional_lag : records[i].integer_lag) - x_sum;
                                                                                            const double y = records[i].residual - y_sum;
                                                                                            cross += x * y;
                                                                                            x_power += x * x;
                                                                                            y_power += y * y;
                                                                                        }
                                                                                        if (x_power <= CAL_REF_VARIANCE_EPSILON ||         y_power <= CAL_REF_VARIANCE_EPSILON) return 0.0f;
                                                                                        return (float)(cross / sqrt(x_power * y_power));
                                                                                    }
                                                                                    static void handle_adc_offset_stability_cmd(uint32_t frame_count) {
                                                                                        static calibration_frame_workspace_t workspace;
                                                                                        static calibration_offset_stability_record_t         records[CAL_OFFSET_STABILITY_MAX_FRAMES];
                                                                                        float residual_values[CAL_OFFSET_STABILITY_MAX_FRAMES];
                                                                                        uint32_t accepted = 0U, rejected = 0U;
                                                                                        double sum = 0.0, square_sum = 0.0;
                                                                                        float minimum = FLT_MAX, maximum = -FLT_MAX;
                                                                                        const float fixed_offset = calibration_software_offset_correction();
                                                                                        const calibration_offset_loop_state_t *offset_state =         calibration_offset_loop_state();
                                                                                        if (adc_sweep_active || RxBufferPtr == NULL) {
                                                                                            ERR("ADC capture is unavailable for offset stability analysis.");
                                                                                            return;
                                                                                        }
                                                                                        if ((offset_state->stage_result != CALIBRATION_OFFSET_RESULT_CONVERGED &&          offset_state->stage_result !=              CALIBRATION_OFFSET_RESULT_PROVISIONAL) ||         !calibration_stored_reference_is_compatible(NULL)) {
                                                                                            ERR("Run adc -cal and a successful adc -cal offset first.");
                                                                                            return;
                                                                                        }
                                                                                        adc_sweep_active = 1U;
                                                                                        g_quiet_calibration_capture = true;
                                                                                        xil_printf("\r\n========== Offset Stability Capture ==========\r\n");
                                                                                        xil_printf("Requested frames         : %lu\r\n",                (unsigned long)frame_count);
                                                                                        print_float_value("Fixed offset correction", fixed_offset, " codes");
                                                                                        xil_printf("Coefficient updates      : DISABLED\r\n");
                                                                                        for (uint32_t frame_number = 1U;
                                                                                        frame_number <= frame_count;
                                                                                        ++frame_number) {
                                                                                            calibration_aligned_frame_t frame;
                                                                                            const char *reason = NULL;
                                                                                            float residual, mean_identity;
                                                                                            if (calibration_capture_against_stored_reference(                 fixed_offset, &workspace, &frame, &reason) != 0 ||             !frame.frame_valid ||             calibration_offset_model_residual(&frame,                 g_stored_offset_reference.canonical_nominal_system_gain,                 &residual, &mean_identity) != 0) {
                                                                                                ++rejected;
                                                                                                xil_printf("Frame %lu/%lu            : REJECTED (%s)\r\n",                 (unsigned long)frame_number, (unsigned long)frame_count,                 reason != NULL ? reason : "invalid residual");
                                                                                            }
                                                                                            else {
                                                                                                calibration_offset_stability_record_t *record =                 &records[accepted];
                                                                                                record->frame_number = frame_number;
                                                                                                record->phase = frame.selected_reference_phase;
                                                                                                record->canonical_phase = frame.canonical_reference_phase;
                                                                                                record->integer_lag = frame.integer_lag;
                                                                                                record->fractional_lag = frame.fractional_lag;
                                                                                                record->correlation = frame.correlation;
                                                                                                record->adc_mean = frame.metrics.adc_mean;
                                                                                                record->residual = residual;
                                                                                                residual_values[accepted] = residual;
                                                                                                sum += residual;
                                                                                                square_sum += (double)residual * residual;
                                                                                                if (residual < minimum) minimum = residual;
                                                                                                if (residual > maximum) maximum = residual;
                                                                                                ++accepted;
                                                                                                xil_printf("Frame %lu/%lu\r\n",                 (unsigned long)frame_number, (unsigned long)frame_count);
                                                                                                xil_printf("  Selected input phase  : %s\r\n",                 record->phase == 0 ? "EVEN" : "ODD");
                                                                                                xil_printf("  Canonical reference phase: %s\r\n",                 record->canonical_phase == 0 ? "EVEN" : "ODD");
                                                                                                xil_printf("  Integer lag           : %ld\r\n",                 (long)frame.integer_lag);
                                                                                                print_float_value("  Fractional lag", frame.fractional_lag, "");
                                                                                                print_float_value("  Fixed-window correlation",                               frame.correlation, "");
                                                                                                print_float_value("  Fixed-window ADC mean",                               frame.metrics.adc_mean, " codes");
                                                                                                print_float_value("  Residual mean", residual, " codes");
                                                                                            }
                                                                                            if (frame_number < frame_count)             usleep(ADC_TIMING_INTERFRAME_DELAY_US);
                                                                                        }
                                                                                        xil_printf("\r\nLag-group residual statistics:\r\n");
                                                                                        xil_printf("Lag    Frames    Mean residual    Residual std dev\r\n");
                                                                                        for (size_t i = 0U;
                                                                                        i < accepted;
                                                                                        ++i) {
                                                                                            bool already_printed = false;
                                                                                            uint32_t n = 0U;
                                                                                            double group_sum = 0.0, group_square_sum = 0.0;
                                                                                            for (size_t j = 0U;
                                                                                            j < i;
                                                                                            ++j)             if (records[j].integer_lag == records[i].integer_lag)                 already_printed = true;
                                                                                            if (already_printed) continue;
                                                                                            for (size_t j = i;
                                                                                            j < accepted;
                                                                                            ++j) {
                                                                                                if (records[j].integer_lag == records[i].integer_lag) {
                                                                                                    ++n;
                                                                                                    group_sum += records[j].residual;
                                                                                                    group_square_sum +=                     (double)records[j].residual * records[j].residual;
                                                                                                }
                                                                                            }
                                                                                            if (n >= 2U) {
                                                                                                const double mean = group_sum / n;
                                                                                                const double stddev = sqrt(fmax(0.0,                 group_square_sum / n - mean * mean));
                                                                                                xil_printf("%ld     %lu       ",                 (long)records[i].integer_lag, (unsigned long)n);
                                                                                                print_double_inline(mean);
                                                                                                xil_printf(" codes       ");
                                                                                                print_double_inline(stddev);
                                                                                                xil_printf(" codes\r\n");
                                                                                            }
                                                                                        }
                                                                                        xil_printf("\r\nSame-phase/same-lag repeatability:\r\n");
                                                                                        for (size_t i = 0U;
                                                                                        i < accepted;
                                                                                        ++i) {
                                                                                            bool already_printed = false;
                                                                                            uint32_t n = 0U;
                                                                                            double adc_sum = 0.0, adc_sq = 0.0, res_sum = 0.0, res_sq = 0.0;
                                                                                            for (size_t j = 0U;
                                                                                            j < i;
                                                                                            ++j)             if (records[j].phase == records[i].phase &&                 records[j].integer_lag == records[i].integer_lag)                 already_printed = true;
                                                                                            if (already_printed) continue;
                                                                                            for (size_t j = i;
                                                                                            j < accepted;
                                                                                            ++j) {
                                                                                                if (records[j].phase == records[i].phase &&                 records[j].integer_lag == records[i].integer_lag) {
                                                                                                    ++n;
                                                                                                    adc_sum += records[j].adc_mean;
                                                                                                    adc_sq += (double)records[j].adc_mean * records[j].adc_mean;
                                                                                                    res_sum += records[j].residual;
                                                                                                    res_sq += (double)records[j].residual * records[j].residual;
                                                                                                }
                                                                                            }
                                                                                            if (n >= 2U) {
                                                                                                const double adc_mean = adc_sum / n, res_mean = res_sum / n;
                                                                                                xil_printf("%s lag %ld, frames %lu\r\n",                 records[i].phase == 0 ? "EVEN" : "ODD",                 (long)records[i].integer_lag, (unsigned long)n);
                                                                                                print_double_value("  ADC mean", adc_mean, " codes");
                                                                                                print_double_value("  ADC mean std dev", sqrt(fmax(0.0,                 adc_sq / n - adc_mean * adc_mean)), " codes");
                                                                                                print_double_value("  residual mean", res_mean, " codes");
                                                                                                print_double_value("  residual std dev", sqrt(fmax(0.0,                 res_sq / n - res_mean * res_mean)), " codes");
                                                                                            }
                                                                                        }
                                                                                        xil_printf("\r\n========== Offset Stability Analysis ==========\r\n");
                                                                                        xil_printf("Frames accepted          : %lu/%lu\r\n",         (unsigned long)accepted, (unsigned long)frame_count);
                                                                                        xil_printf("Frames rejected          : %lu\r\n", (unsigned long)rejected);
                                                                                        if (accepted > 0U) {
                                                                                            const double mean = sum / accepted;
                                                                                            const double variance = fmax(0.0,             square_sum / accepted - mean * mean);
                                                                                            const double stddev = sqrt(variance);
                                                                                            const double standard_error = stddev / sqrt((double)accepted);
                                                                                            float sorted[CAL_OFFSET_STABILITY_MAX_FRAMES];
                                                                                            double phase_sum[2] = {
                                                                                                0.0, 0.0}
                                                                                                ;
                                                                                                double phase_sq[2] = {
                                                                                                    0.0, 0.0}
                                                                                                    ;
                                                                                                    double phase_corr[2] = {
                                                                                                        0.0, 0.0}
                                                                                                        ;
                                                                                                        uint32_t phase_count[2] = {
                                                                                                            0U, 0U}
                                                                                                            ;
                                                                                                            const float lag_corr = calibration_stability_correlation(             records, accepted, false);
                                                                                                            const float fractional_corr = calibration_stability_correlation(             records, accepted, true);
                                                                                                            memcpy(sorted, residual_values, accepted * sizeof(sorted[0]));
                                                                                                            print_double_value("Mean residual", mean, " codes");
                                                                                                            print_float_value("Median residual",                           median_float(sorted, accepted), " codes");
                                                                                                            print_double_value("Residual std dev", stddev, " codes");
                                                                                                            print_double_value("Residual standard error", standard_error,                            " codes");
                                                                                                            print_float_value("Minimum residual", minimum, " codes");
                                                                                                            print_float_value("Maximum residual", maximum, " codes");
                                                                                                            print_double_value("Residual RMS",                            sqrt(square_sum / accepted), " codes");
                                                                                                            for (size_t i = 0U;
                                                                                                            i < accepted;
                                                                                                            ++i) {
                                                                                                                const unsigned phase = records[i].phase == 1 ? 1U : 0U;
                                                                                                                ++phase_count[phase];
                                                                                                                phase_sum[phase] += records[i].residual;
                                                                                                                phase_sq[phase] +=                 (double)records[i].residual * records[i].residual;
                                                                                                                phase_corr[phase] += records[i].correlation;
                                                                                                            }
                                                                                                            for (unsigned phase = 0U;
                                                                                                            phase < 2U;
                                                                                                            ++phase) {
                                                                                                                xil_printf("%s:\r\n", phase == 0U ? "EVEN" : "ODD");
                                                                                                                xil_printf("  frames                : %lu\r\n",                 (unsigned long)phase_count[phase]);
                                                                                                                if (phase_count[phase] > 0U) {
                                                                                                                    const double n = phase_count[phase];
                                                                                                                    const double pmean = phase_sum[phase] / n;
                                                                                                                    const double pstd = sqrt(fmax(0.0,                     phase_sq[phase] / n - pmean * pmean));
                                                                                                                    print_double_value("  mean residual", pmean, " codes");
                                                                                                                    print_double_value("  std dev", pstd, " codes");
                                                                                                                    print_double_value("  standard error", pstd / sqrt(n),                                    " codes");
                                                                                                                    print_double_value("  mean correlation",                                    phase_corr[phase] / n, "");
                                                                                                                }
                                                                                                            }
                                                                                                            {
                                                                                                                const double even_mean = phase_count[0] ?                 phase_sum[0] / phase_count[0] : 0.0;
                                                                                                                const double odd_mean = phase_count[1] ?                 phase_sum[1] / phase_count[1] : 0.0;
                                                                                                                const double phase_difference = even_mean - odd_mean;
                                                                                                                const double even_variance = phase_count[0] ? fmax(0.0,                 phase_sq[0] / phase_count[0] -                 even_mean * even_mean) : 0.0;
                                                                                                                const double odd_variance = phase_count[1] ? fmax(0.0,                 phase_sq[1] / phase_count[1] -                 odd_mean * odd_mean) : 0.0;
                                                                                                                const double phase_difference_se = sqrt(                 (phase_count[0] ? even_variance / phase_count[0] : 0.0) +                 (phase_count[1] ? odd_variance / phase_count[1] : 0.0));
                                                                                                                const bool systematic = fabs(mean) > fmax(                 CALIBRATION_OFFSET_TOLERANCE_CODES,                 2.0 * standard_error);
                                                                                                                const bool phase_bias = phase_count[0] > 1U &&                 phase_count[1] > 1U &&                 fabs(phase_difference) > 2.0 * phase_difference_se &&                 fabs(phase_difference) >                     CALIBRATION_OFFSET_TOLERANCE_CODES;
                                                                                                                const bool lag_bias = fabsf(lag_corr) >=                     CAL_OFFSET_STABILITY_BIAS_CORRELATION ||                 fabsf(fractional_corr) >=                     CAL_OFFSET_STABILITY_BIAS_CORRELATION;
                                                                                                                print_double_value("Phase mean difference",                                phase_difference, " codes");
                                                                                                                print_double_value("Phase difference standard error",                                phase_difference_se, " codes");
                                                                                                                print_float_value("Residual/lag correlation", lag_corr, "");
                                                                                                                print_float_value("Residual/fractional-lag correlation",                               fractional_corr, "");
                                                                                                                xil_printf("Assessment               : %s\r\n",                 systematic ? "SYSTEMATIC OFFSET REMAINS" :                 phase_bias ? "POSSIBLE PHASE-DEPENDENT BIAS" :                 lag_bias ? "POSSIBLE LAG-DEPENDENT BIAS" :                            "STABLE ZERO-MEAN OFFSET");
                                                                                                            }
                                                                                                        }
                                                                                                        else {
                                                                                                            xil_printf("Assessment               : INSUFFICIENT VALID FRAMES\r\n");
                                                                                                        }
                                                                                                        xil_printf("=================================================\r\n");
                                                                                                        g_quiet_calibration_capture = false;
                                                                                                        adc_sweep_active = 0U;
                                                                                                    }
                                                                                                    static void handle_adc_offset_calibration_loop_cmd(void) {
                                                                                                        static int16_t even_reference[ADC_VALID_SAMPLE_COUNT];
                                                                                                        static int16_t odd_reference[ADC_VALID_SAMPLE_COUNT];
                                                                                                        static calibration_frame_workspace_t frame_workspace;
                                                                                                        calibration_offset_loop_state_t *state =         calibration_offset_loop_state();
                                                                                                        size_t reconstructed_count = 0U;
                                                                                                        double even_variance;
                                                                                                        double odd_variance;
                                                                                                        float last_active_correction = NAN;
                                                                                                        const char *pending_reason = NULL;
                                                                                                        const bool compact = calibration_compact_output_enabled();
                                                                                                        if (adc_sweep_active) {
                                                                                                            ERR("Another automatic ADC capture is already in progress.");
                                                                                                            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                            state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                            return;
                                                                                                        }
                                                                                                        /* A failed rerun must never expose a handoff from an earlier offset run. */
                                                                                                        calibration_gain_input_frame_invalidate();
                                                                                                        calibration_offset_loop_begin_run(state);
                                                                                                        memset(&g_latest_dither_offset_diagnostic, 0,         sizeof(g_latest_dither_offset_diagnostic));
                                                                                                        if ((state->calibration_channel < -1) ||         (state->calibration_channel > 1)) {
                                                                                                            ERR("Invalid calibration channel in offset calibration state.");
                                                                                                            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                            state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                            calibration_offset_loop_print_summary(state);
                                                                                                            return;
                                                                                                        }
                                                                                                        if (RxBufferPtr == NULL) {
                                                                                                            ERR("DMA receive buffer is not available.");
                                                                                                            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                            state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                            calibration_offset_loop_print_summary(state);
                                                                                                            return;
                                                                                                        }
                                                                                                        if (!calibration_stored_reference_is_compatible(&pending_reason)) {
                                                                                                            xil_printf("No valid stored adc -cal reference frame.\r\n");
                                                                                                            if (pending_reason != NULL) {
                                                                                                                xil_printf("Reason: %s\r\n", pending_reason);
                                                                                                            }
                                                                                                            xil_printf("Development workflow: run 'adc -cal timing' first.\r\n");
                                                                                                            memset(&g_stored_offset_reference, 0,                sizeof(g_stored_offset_reference));
                                                                                                            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                            state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                            calibration_offset_loop_print_summary(state);
                                                                                                            return;
                                                                                                        }
                                                                                                        if (!compact) print_adc_analysis_rate_header();
                                                                                                        if (calibration_prepare_uploaded_dac_reference(             even_reference, odd_reference, &reconstructed_count,             &even_variance, &odd_variance, 1) != 0) {
                                                                                                            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                            state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                            calibration_offset_loop_print_summary(state);
                                                                                                            return;
                                                                                                        }
                                                                                                        adc_sweep_active = 1U;
                                                                                                        g_quiet_calibration_capture = true;
                                                                                                        if (!compact) {
                                                                                                            xil_printf("\r\n========== ADC Offset Calibration ==========\r\n");
                                                                                                            xil_printf("Offset reference source : stored adc -cal frame\r\n");
                                                                                                            xil_printf("Reference channel       : %s\r\n",                    calibration_channel_name(                        g_stored_offset_reference.selected_channel));
                                                                                                            xil_printf("Canonical reference phase: %s\r\n",                    g_stored_offset_reference.canonical_reference_phase == 0 ?                    "EVEN" : "ODD");
                                                                                                            xil_printf("Reference samples       : %lu\r\n",                    (unsigned long)                        g_stored_offset_reference.analysis_sample_count);
                                                                                                            print_float_value("Gain correction", state->gain_correction, "");
                                                                                                            print_float_value("Initial offset correction",                           state->offset_correction, " codes");
                                                                                                            xil_printf("Batch size              : %u frames\r\n",                    CALIBRATION_OFFSET_BATCH_SIZE);
                                                                                                            print_float_value("Reference mean",             g_stored_offset_reference.metrics.adc_mean, " codes");
                                                                                                            print_float_value("Offset tolerance",             CALIBRATION_OFFSET_TOLERANCE_CODES, " codes");
                                                                                                            xil_printf("Maximum batch iterations: %u\r\n",                    CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS);
                                                                                                            print_float_value("Far offset update gain",                           CALIBRATION_OFFSET_UPDATE_STEP, "");
                                                                                                            print_float_value("Near offset update gain",                           CALIBRATION_OFFSET_NEAR_UPDATE_STEP, "");
                                                                                                            print_float_value("Residual filter alpha",                           CALIBRATION_OFFSET_FILTER_ALPHA, "");
                                                                                                            print_float_value("Standard error diagnostic guide",             CALIBRATION_OFFSET_MAX_STANDARD_ERROR_CODES, " codes");
                                                                                                            print_float_value("Enter tolerance",             CALIBRATION_OFFSET_ENTER_TOLERANCE_CODES, " codes");
                                                                                                            print_float_value("Exit tolerance",             CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES, " codes");
                                                                                                            print_float_value("Maximum stable update",             CALIBRATION_OFFSET_MAX_UPDATE_CODES, " codes");
                                                                                                            xil_printf("Minimum batches before stall: %u\r\n",                    CALIBRATION_OFFSET_MIN_BATCHES_BEFORE_STALL);
                                                                                                            print_float_value("Minimum residual improvement",             CALIBRATION_OFFSET_MIN_IMPROVEMENT_CODES, " codes");
                                                                                                            print_float_value("Stall update threshold",             CALIBRATION_OFFSET_STALL_UPDATE_THRESHOLD_CODES, " codes");
                                                                                                            print_float_value("Verification limit",             CALIBRATION_OFFSET_VERIFICATION_TARGET_CODES, " codes");
                                                                                                            print_float_value("Verification provisional limit",             CALIBRATION_OFFSET_VERIFICATION_PROVISIONAL_LIMIT_CODES,             " codes");
                                                                                                        }
                                                                                                        while ((state->batch_iteration_count <             CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS) &&            (state->convergence_count <             CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES)) {
                                                                                                            calibration_offset_batch_result_t batch;
                                                                                                            float coefficient_delta = 0.0f;
                                                                                                            float proposed_offset_update;
                                                                                                            float next_offset_correction;
                                                                                                            float previous_filtered_error;
                                                                                                            float current_filtered_error;
                                                                                                            float standard_error;
                                                                                                            float score;
                                                                                                            bool batch_pass;
                                                                                                            bool had_previous_filtered_residual;
                                                                                                            bool meaningful_best_improvement;
                                                                                                            bool meaningful_batch_improvement;
                                                                                                            bool inside_convergence_region;
                                                                                                            bool correction_held;
                                                                                                            bool stall_allowed;
                                                                                                            ++state->batch_iteration_count;
                                                                                                            if (!compact)             xil_printf("\r\n========== Offset Batch %lu ==========\r\n",                        (unsigned long)state->batch_iteration_count);
                                                                                                            if (calibration_compute_offset_batch(state->offset_correction,                 &frame_workspace, state, &batch) != 0) {
                                                                                                                state->convergence_count = 0U;
                                                                                                                if (compact) {
                                                                                                                    calibration_print_offset_batch_compact(                     state->batch_iteration_count, &batch, state,                     false, "REJECTED");
                                                                                                                }
                                                                                                                else {
                                                                                                                    xil_printf("Batch status            : REJECTED\r\n");
                                                                                                                    xil_printf("Reason                  : no valid aligned frames\r\n");
                                                                                                                }
                                                                                                                continue;
                                                                                                            }
                                                                                                            state->latest_mean_residual = batch.mean;
                                                                                                            state->latest_residual_stddev = batch.stddev;
                                                                                                            state->latest_residual_min = batch.minimum;
                                                                                                            state->latest_residual_max = batch.maximum;
                                                                                                            state->latest_batch_rmse = batch.mean_fitted_rmse;
                                                                                                            state->latest_correlation = batch.mean_correlation;
                                                                                                            state->latest_rmse = batch.mean_fitted_rmse;
                                                                                                            state->calibration_channel = g_stored_offset_reference.selected_channel;
                                                                                                            standard_error = batch.stddev / sqrtf((float)batch.accepted);
                                                                                                            state->latest_standard_error = standard_error;
                                                                                                            had_previous_filtered_residual =             state->filtered_residual_valid != 0U;
                                                                                                            previous_filtered_error = had_previous_filtered_residual ?             fabsf(state->filtered_residual) : FLT_MAX;
                                                                                                            if (state->filtered_residual_valid == 0U) {
                                                                                                                state->filtered_residual = batch.mean;
                                                                                                                state->filtered_residual_valid = 1U;
                                                                                                            }
                                                                                                            else {
                                                                                                                state->filtered_residual =                 CALIBRATION_OFFSET_FILTER_ALPHA * state->filtered_residual +                 (1.0f - CALIBRATION_OFFSET_FILTER_ALPHA) * batch.mean;
                                                                                                            }
                                                                                                            current_filtered_error = fabsf(state->filtered_residual);
                                                                                                            if (!compact) {
                                                                                                                xil_printf("\r\n---------- Batch Statistics ----------\r\n");
                                                                                                                print_float_value("Residual mean", batch.mean, " codes");
                                                                                                                print_float_value("Residual std dev", batch.stddev, " codes");
                                                                                                                print_float_value("Minimum residual", batch.minimum, " codes");
                                                                                                                print_float_value("Maximum residual", batch.maximum, " codes");
                                                                                                                print_float_value("Residual RMS", batch.rmse, " codes");
                                                                                                                print_float_value("Batch fitted RMSE",                               batch.mean_fitted_rmse, " codes");
                                                                                                                print_float_value("Residual standard error",                               standard_error, " codes");
                                                                                                                print_float_value("Filtered residual",                               state->filtered_residual, " codes");
                                                                                                                xil_printf("Accepted frames         : %lu/%u\r\n",                        (unsigned long)batch.accepted,                        CALIBRATION_OFFSET_BATCH_SIZE);
                                                                                                                xil_printf("Rejected frames         : %lu/%u\r\n",                        (unsigned long)batch.rejected,                        CALIBRATION_OFFSET_BATCH_SIZE);
                                                                                                                xil_printf("Dither estimator frames : PASS %lu | WARNING %lu | INVALID %lu\r\n",                        (unsigned long)batch.dither_pass,                        (unsigned long)batch.dither_warning,                        (unsigned long)batch.dither_invalid);
                                                                                                                if (batch.dither_valid_estimates > 0U) {
                                                                                                                    print_double_value("Mean dither offset",                               batch.mean_dither_offset, " codes");
                                                                                                                    print_double_value("Mean existing-dither delta",                               batch.mean_existing_dither_delta,                               " codes");
                                                                                                                }
                                                                                                                else if (batch.dither_invalid > 0U) {
                                                                                                                    xil_printf("Dither rejection reason : %s\r\n",                            calibration_dither_offset_reason_name(                                batch.dither_latest.reason));
                                                                                                                }
                                                                                                            }
                                                                                                            score = fabsf(state->filtered_residual) +                 CALIBRATION_OFFSET_SCORE_RMSE_WEIGHT *                     batch.mean_fitted_rmse +                 CALIBRATION_OFFSET_SCORE_STDERR_WEIGHT * standard_error;
                                                                                                            if (!compact) print_float_value("Batch solution score", score, "");
                                                                                                            if (score < state->best_score)             state->best_score = score;
                                                                                                            /* Stall progress follows the filtered residual that drives the          * controller, not the RMSE/noise-weighted diagnostic score. */
                                                                                                            meaningful_best_improvement = state->best_abs_residual == FLT_MAX ||             state->best_abs_residual - current_filtered_error >                 CALIBRATION_OFFSET_MIN_IMPROVEMENT_CODES;
                                                                                                            meaningful_batch_improvement = had_previous_filtered_residual &&             previous_filtered_error - current_filtered_error >                 CALIBRATION_OFFSET_MIN_IMPROVEMENT_CODES;
                                                                                                            if (current_filtered_error < state->best_abs_residual) {
                                                                                                                state->best_abs_residual = current_filtered_error;
                                                                                                                state->best_residual = state->filtered_residual;
                                                                                                                state->best_filtered_residual = state->filtered_residual;
                                                                                                                state->best_raw_residual = batch.mean;
                                                                                                                state->best_offset_correction = state->offset_correction;
                                                                                                                state->best_rmse = batch.mean_fitted_rmse;
                                                                                                            }
                                                                                                            if (meaningful_best_improvement || meaningful_batch_improvement)             state->no_improvement_count = 0U;
                                                                                                            else             ++state->no_improvement_count;
                                                                                                            state->latest_controller_gain =             fabsf(state->filtered_residual) <                 CALIBRATION_OFFSET_NEAR_RESIDUAL_CODES ?             CALIBRATION_OFFSET_NEAR_UPDATE_STEP :             CALIBRATION_OFFSET_UPDATE_STEP;
                                                                                                            proposed_offset_update =             -state->latest_controller_gain * state->filtered_residual;
                                                                                                            coefficient_delta = proposed_offset_update;
                                                                                                            inside_convergence_region =             fabsf(state->filtered_residual) <=                 (state->convergence_count > 0U ?                     CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES :                     CALIBRATION_OFFSET_ENTER_TOLERANCE_CODES);
                                                                                                            batch_pass =             batch.accepted >= CALIBRATION_OFFSET_MIN_ACCEPTED_FRAMES &&             inside_convergence_region &&             fabsf(coefficient_delta) <=                 CALIBRATION_OFFSET_MAX_UPDATE_CODES;
                                                                                                            correction_held = batch_pass;
                                                                                                            if (batch_pass) {
                                                                                                                ++state->convergence_count;
                                                                                                                coefficient_delta = 0.0f;
                                                                                                            }
                                                                                                            else {
                                                                                                                if (fabsf(state->filtered_residual) >                     CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES)                 state->convergence_count = 0U;
                                                                                                            }
                                                                                                            if (fabsf(state->filtered_residual) <=                 CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES ||             state->convergence_count > 0U) {
                                                                                                                state->no_improvement_count = 0U;
                                                                                                            }
                                                                                                            if (!compact) {
                                                                                                                print_float_value("Proposed offset update",                               proposed_offset_update, " codes");
                                                                                                                xil_printf("Correction held for confirmation: %s\r\n",                        correction_held ? "YES" : "NO");
                                                                                                                xil_printf("Inside convergence region: %s\r\n",                        inside_convergence_region ? "YES" : "NO");
                                                                                                                xil_printf("No-improvement batches  : %lu/%u\r\n",                        (unsigned long)state->no_improvement_count,                        CALIBRATION_OFFSET_NO_IMPROVEMENT_LIMIT);
                                                                                                            }
                                                                                                            /* A stall is actual controller stagnation: it is old enough to be          * meaningful, outside the confirmation region, and proposing no          * material coefficient movement.  The iteration limit remains the          * safety net for noisy non-convergent runs. */
                                                                                                            stall_allowed =             state->batch_iteration_count >=                 CALIBRATION_OFFSET_MIN_BATCHES_BEFORE_STALL &&             fabsf(state->filtered_residual) >                 CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES &&             fabsf(proposed_offset_update) <=                 CALIBRATION_OFFSET_STALL_UPDATE_THRESHOLD_CODES &&             state->no_improvement_count >=                 CALIBRATION_OFFSET_NO_IMPROVEMENT_LIMIT;
                                                                                                            if (stall_allowed) {
                                                                                                                state->termination_reason =                 CAL_OFFSET_TERMINATION_NO_IMPROVEMENT;
                                                                                                                if (compact)                 calibration_print_offset_batch_compact(                     state->batch_iteration_count, &batch, state,                     false, "FAILED");
                                                                                                                break;
                                                                                                            }
                                                                                                            next_offset_correction =             state->offset_correction + coefficient_delta;
                                                                                                            if (!isfinite(coefficient_delta) ||             !isfinite(next_offset_correction)) {
                                                                                                                state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                                xil_printf("Batch status            : FAILED\r\n");
                                                                                                                xil_printf("Reason                  : nonfinite offset update\r\n");
                                                                                                                break;
                                                                                                            }
                                                                                                            next_offset_correction = fmaxf(             -CALIBRATION_OFFSET_MAX_ABS_CORRECTION_CODES,             fminf(CALIBRATION_OFFSET_MAX_ABS_CORRECTION_CODES,                   next_offset_correction));
                                                                                                            if (coefficient_delta != 0.0f) {
                                                                                                                if (calibration_set_software_offset_correction(                     next_offset_correction) != 0) {
                                                                                                                    state->convergence_count = 0U;
                                                                                                                    state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                                    state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                                    xil_printf("Batch status            : FAILED\r\n");
                                                                                                                    xil_printf("Reason                  : offset coefficient rejected\r\n");
                                                                                                                    break;
                                                                                                                }
                                                                                                                state->offset_correction = next_offset_correction;
                                                                                                            }
                                                                                                            if (compact) {
                                                                                                                calibration_print_offset_batch_compact(                 state->batch_iteration_count, &batch, state,                 batch_pass, "RUNNING");
                                                                                                            }
                                                                                                            else {
                                                                                                                print_float_value("Offset update",                               coefficient_delta, " codes");
                                                                                                                print_float_value("Offset correction",                               state->offset_correction, " codes");
                                                                                                                xil_printf("Consecutive passes      : %lu/%u\r\n",                        (unsigned long)state->convergence_count,                        CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES);
                                                                                                                xil_printf("Batch status            : %s\r\n",                        batch_pass ? "PASS" : "ACCEPTED");
                                                                                                            }
                                                                                                            if (state->convergence_count <             CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES) {
                                                                                                                usleep(ADC_TIMING_INTERFRAME_DELAY_US);
                                                                                                            }
                                                                                                        }
                                                                                                        last_active_correction = state->offset_correction;
                                                                                                        if (state->final_status == CALIBRATION_OFFSET_LOOP_RUNNING) {
                                                                                                            if (state->convergence_count >=             CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES) {
                                                                                                                state->controller_converged = 1U;
                                                                                                                state->final_status =                 CALIBRATION_OFFSET_LOOP_CONTROLLER_CONVERGED;
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_CONVERGED;
                                                                                                            }
                                                                                                            else {
                                                                                                                state->final_status = CALIBRATION_OFFSET_LOOP_NOT_CONVERGED;
                                                                                                                if (state->termination_reason == CAL_OFFSET_TERMINATION_NONE)                 state->termination_reason =                     CAL_OFFSET_TERMINATION_ITERATION_LIMIT;
                                                                                                            }
                                                                                                        }
                                                                                                        if (state->final_status ==             CALIBRATION_OFFSET_LOOP_CONTROLLER_CONVERGED) {
                                                                                                            double residual_sum = 0.0, residual_square_sum = 0.0;
                                                                                                            double correlation_sum = 0.0, rmse_sum = 0.0;
                                                                                                            uint32_t total_frames = 0U;
                                                                                                            state->final_status = CALIBRATION_OFFSET_LOOP_VERIFYING;
                                                                                                            for (uint32_t batch_number = 1U;
                                                                                                            batch_number <= CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES;
                                                                                                            ++batch_number) {
                                                                                                                calibration_offset_batch_result_t verification;
                                                                                                                bool verification_pass;
                                                                                                                if (!compact)                 xil_printf("\r\n========== Offset Verification Batch %lu/%u ==========\r\n",                            (unsigned long)batch_number,                            CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES);
                                                                                                                ++state->verification_batch_count;
                                                                                                                if (calibration_compute_offset_batch(state->offset_correction,                     &frame_workspace, state, &verification) == 0) {
                                                                                                                    const double n = (double)verification.accepted;
                                                                                                                    residual_sum += n * (double)verification.mean;
                                                                                                                    residual_square_sum += n *                     ((double)verification.stddev * verification.stddev +                      (double)verification.mean * verification.mean);
                                                                                                                    correlation_sum += n * verification.mean_correlation;
                                                                                                                    rmse_sum += n * verification.mean_fitted_rmse;
                                                                                                                    total_frames += verification.accepted;
                                                                                                                }
                                                                                                                if (total_frames > 0U) {
                                                                                                                    const double n = (double)total_frames;
                                                                                                                    const double mean = residual_sum / n;
                                                                                                                    const double variance = fmax(0.0,                     residual_square_sum / n - mean * mean);
                                                                                                                    state->verification_residual = (float)mean;
                                                                                                                    state->verification_stddev = (float)sqrt(variance);
                                                                                                                    state->verification_standard_error =                     state->verification_stddev / sqrtf((float)total_frames);
                                                                                                                    state->verification_correlation =                     (float)(correlation_sum / n);
                                                                                                                    state->verification_rmse = (float)(rmse_sum / n);
                                                                                                                    state->verification_accepted_frames = total_frames;
                                                                                                                }
                                                                                                                verification_pass =                 total_frames >= CALIBRATION_OFFSET_MIN_ACCEPTED_FRAMES &&                 state->verification_correlation >=                     CAL_DAC_REF_MIN_CORRELATION &&                 fabsf(state->verification_residual) <=                     CALIBRATION_OFFSET_VERIFICATION_TARGET_CODES;
                                                                                                                if (compact) {
                                                                                                                    xil_printf("Verify %lu/%u | frames %lu | residual ",                            (unsigned long)batch_number,                            CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES,                            (unsigned long)total_frames);
                                                                                                                    print_signed_float_inline(state->verification_residual);
                                                                                                                    xil_printf(" | %s\r\n",                     verification_pass ? "PASS" :                     batch_number <                         CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES ?                     "RETRY" : "COMPLETE");
                                                                                                                }
                                                                                                                else {
                                                                                                                    print_float_value("Combined verification mean",                                   state->verification_residual, " codes");
                                                                                                                    xil_printf("Combined verification frames: %lu\r\n",                            (unsigned long)total_frames);
                                                                                                                }
                                                                                                                if (verification_pass) {
                                                                                                                    break;
                                                                                                                }
                                                                                                                if (!compact && batch_number <                     CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES) {
                                                                                                                    xil_printf("Verification status     : RETRY REQUIRED\r\n");
                                                                                                                }
                                                                                                            }
                                                                                                            state->verification_valid =             state->verification_accepted_frames >=                 CALIBRATION_OFFSET_MIN_ACCEPTED_FRAMES &&             isfinite(state->verification_residual) &&             isfinite(state->verification_correlation) &&             isfinite(state->verification_stddev) &&             isfinite(state->verification_standard_error) &&             isfinite(state->verification_rmse) &&             state->verification_correlation >= CAL_DAC_REF_MIN_CORRELATION;
                                                                                                            if (!state->verification_valid) {
                                                                                                                state->verification_status = CAL_OFFSET_VERIFICATION_FAIL;
                                                                                                                state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                                state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                            }
                                                                                                            else if (fabsf(state->verification_residual) <=                    CALIBRATION_OFFSET_VERIFICATION_TARGET_CODES) {
                                                                                                                state->verification_status = CAL_OFFSET_VERIFICATION_PASS;
                                                                                                                state->final_status = CALIBRATION_OFFSET_LOOP_PASS;
                                                                                                                state->stage_result = CALIBRATION_OFFSET_RESULT_CONVERGED;
                                                                                                            }
                                                                                                            else if (fabsf(state->verification_residual) <=                    CALIBRATION_OFFSET_VERIFICATION_PROVISIONAL_LIMIT_CODES) {
                                                                                                                state->verification_status = CAL_OFFSET_VERIFICATION_MARGINAL;
                                                                                                                state->final_status = CALIBRATION_OFFSET_LOOP_BEST_AVAILABLE;
                                                                                                                state->stage_result = CALIBRATION_OFFSET_RESULT_PROVISIONAL;
                                                                                                            }
                                                                                                            else {
                                                                                                                state->verification_status = CAL_OFFSET_VERIFICATION_FAIL;
                                                                                                                state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                                state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                                state->termination_reason =                 CAL_OFFSET_TERMINATION_VERIFICATION_FAILED;
                                                                                                            }
                                                                                                            if (!compact) {
                                                                                                                print_float_value("Verification residual",                               state->verification_residual, " codes");
                                                                                                                print_float_value("Verification std dev",                               state->verification_stddev, " codes");
                                                                                                                print_float_value("Verification standard error",                               state->verification_standard_error, " codes");
                                                                                                                print_float_value("Verification correlation",                               state->verification_correlation, "");
                                                                                                                xil_printf("Verification frames     : %lu\r\n",                        (unsigned long)state->verification_accepted_frames);
                                                                                                                xil_printf("Verification status     : %s\r\n",                        calibration_offset_verification_name(                            state->verification_status));
                                                                                                            }
                                                                                                        }
                                                                                                        if (state->final_status == CALIBRATION_OFFSET_LOOP_NOT_CONVERGED &&         state->best_abs_residual < FLT_MAX &&         isfinite(state->best_offset_correction)) {
                                                                                                            if (calibration_set_software_offset_correction(                 state->best_offset_correction) == 0) {
                                                                                                                state->offset_correction = state->best_offset_correction;
                                                                                                                state->restored_best_solution = 1U;
                                                                                                                state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                                state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                            }
                                                                                                            else {
                                                                                                                state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                                state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                            }
                                                                                                        }
                                                                                                        if (state->final_status == CALIBRATION_OFFSET_LOOP_NOT_CONVERGED) {
                                                                                                            state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                            state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                        }
                                                                                                        if (state->final_status == CALIBRATION_OFFSET_LOOP_FAILED &&         state->stage_result == CALIBRATION_OFFSET_RESULT_NONE)         state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                        if (state->restored_best_solution != 0U) {
                                                                                                            if (state->termination_reason ==                 CAL_OFFSET_TERMINATION_NO_IMPROVEMENT) {
                                                                                                                xil_printf("\r\nOffset controller stalled after %lu batches.\r\n",                        (unsigned long)state->batch_iteration_count);
                                                                                                            }
                                                                                                            print_float_value("Best filtered residual",                           state->best_filtered_residual, " codes");
                                                                                                            print_float_value("Last correction",                           last_active_correction, " codes");
                                                                                                            print_float_value("Best correction",                           state->best_offset_correction, " codes");
                                                                                                            print_float_value("Restored correction",                           state->offset_correction, " codes");
                                                                                                        }
                                                                                                        if (state->stage_result == CALIBRATION_OFFSET_RESULT_CONVERGED ||         state->stage_result == CALIBRATION_OFFSET_RESULT_PROVISIONAL) {
                                                                                                            calibration_frame_config_t output_config;
                                                                                                            int output_status;
                                                                                                            const char *output_stage =             state->stage_result == CALIBRATION_OFFSET_RESULT_CONVERGED ?             "offset calibration (converged)" :             "offset calibration (provisional)";
                                                                                                            output_config.locked_channel = state->calibration_channel;
                                                                                                            output_config.adc_gain_correction = 1.0f;
                                                                                                            output_config.adc_offset_correction = state->offset_correction;
                                                                                                            output_config.reference_scale = 1.0f;
                                                                                                            output_config.reject_clipped_input = true;
                                                                                                            output_status = calibration_publish_final_capture(             output_stage, even_reference, odd_reference,             reconstructed_count, &output_config, &frame_workspace);
                                                                                                            if (output_status != 0) {
                                                                                                                state->final_status = CALIBRATION_OFFSET_LOOP_FAILED;
                                                                                                                state->stage_result = CALIBRATION_OFFSET_RESULT_FAILED;
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                                state->restored_best_solution = 0U;
                                                                                                            }
                                                                                                            else {
                                                                                                                g_pending_calibration_frame.source_offset_result =                 state->stage_result;
                                                                                                                if (!compact)                 xil_printf("Output classification   : %s\r\n",                     calibration_offset_result_name(state->stage_result));
                                                                                                            }
                                                                                                        }
                                                                                                        else {
                                                                                                            calibration_gain_input_frame_invalidate();
                                                                                                            if (!compact)             xil_printf("\r\nCalibrated capture output: none "                        "(offset calibration produced no usable solution).\r\n");
                                                                                                        }
                                                                                                        calibration_offset_loop_print_summary(state);
                                                                                                        g_quiet_calibration_capture = false;
                                                                                                        adc_sweep_active = 0U;
                                                                                                    }
                                                                                                    static void calibration_gain_loop_begin_run(     calibration_gain_loop_state_t *state) {
                                                                                                        memset(state, 0, sizeof(*state));
                                                                                                        state->gain_correction = calibration_software_gain_correction();
                                                                                                        state->initial_gain_correction = state->gain_correction;
                                                                                                        state->final_requested_gain_correction = state->gain_correction;
                                                                                                        state->fixed_offset_correction =         calibration_software_offset_correction();
                                                                                                        state->calibration_channel = calibration_channel_selection();
                                                                                                        state->final_status = CALIBRATION_GAIN_LOOP_RUNNING;
                                                                                                        state->latest_fitted_gain = 0.0f;
                                                                                                        state->latest_gain_error = 0.0f;
                                                                                                        state->latest_fitted_offset = 0.0f;
                                                                                                        state->latest_correlation = 0.0f;
                                                                                                        state->latest_rmse = 0.0f;
                                                                                                        state->latest_waveform_rmse = 0.0f;
                                                                                                        state->latest_waveform_rmse_improvement = 0.0f;
                                                                                                        state->previous_waveform_rmse = 0.0f;
                                                                                                        state->have_previous_waveform_rmse = 0U;
                                                                                                        state->best_score = FLT_MAX;
                                                                                                        state->best_gain_correction = state->gain_correction;
                                                                                                        state->failure_reason = "none";
                                                                                                    }
                                                                                                    static bool calibration_gain_correction_is_at_limit(     float correction) {
                                                                                                        return isfinite(correction) &&         (fabsf(correction - CALIBRATION_GAIN_CORRECTION_MIN) <= 1.0e-6f ||          fabsf(correction - CALIBRATION_GAIN_CORRECTION_MAX) <= 1.0e-6f);
                                                                                                    }
                                                                                                    static bool calibration_gain_correction_requested_out_of_range(     float correction) {
                                                                                                        return isfinite(correction) &&         (correction < CALIBRATION_GAIN_CORRECTION_MIN ||          correction > CALIBRATION_GAIN_CORRECTION_MAX);
                                                                                                    }
                                                                                                    static const char *calibration_existing_gain_loop_status_name(     const calibration_gain_loop_state_t *state) {
                                                                                                        bool correction_numerical_valid;
                                                                                                        bool correction_saturated;
                                                                                                        bool existing_measurements_valid;
                                                                                                        bool residual_out_of_tolerance;
                                                                                                        bool pass_measurements_valid;
                                                                                                        if (state == NULL) return "FAIL";
                                                                                                        correction_numerical_valid =         isfinite(state->gain_correction) &&         isfinite(state->final_requested_gain_correction);
                                                                                                        correction_saturated =         state->saturation_occurred ||         calibration_gain_correction_is_at_limit(state->gain_correction) ||         calibration_gain_correction_requested_out_of_range(             state->final_requested_gain_correction);
                                                                                                        existing_measurements_valid =         state->accepted_frame_count > 0U &&         state->filtered_gain_error_valid &&         isfinite(state->filtered_gain_error) &&         isfinite(state->latest_fitted_gain) &&         state->latest_fitted_gain > FLT_EPSILON &&         isfinite(state->latest_gain_error) &&         isfinite(state->latest_gain_standard_error);
                                                                                                        residual_out_of_tolerance =         !existing_measurements_valid ||         fabsf(state->filtered_gain_error) > CALIBRATION_GAIN_TOLERANCE;
                                                                                                        pass_measurements_valid =         correction_numerical_valid &&         existing_measurements_valid &&         fabsf(state->filtered_gain_error) <= CALIBRATION_GAIN_TOLERANCE &&         state->convergence_count >=             CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES &&         state->termination_reason != CAL_OFFSET_TERMINATION_ERROR &&         !correction_saturated;
                                                                                                        if (!correction_numerical_valid) return "FAIL";
                                                                                                        if (correction_saturated && residual_out_of_tolerance)         return "SATURATED";
                                                                                                        switch (state->final_status) {
                                                                                                        case CALIBRATION_GAIN_LOOP_PASS:
                                                                                                            return pass_measurements_valid ? "PASS" :             correction_saturated ? "SATURATED" : "FAIL";
                                                                                                        case CALIBRATION_GAIN_LOOP_RUNNING:
                                                                                                            return existing_measurements_valid ? "RUNNING" : "FAIL";
                                                                                                        case CALIBRATION_GAIN_LOOP_BEST_AVAILABLE:
                                                                                                            return correction_saturated && residual_out_of_tolerance ?             "SATURATED" : "NOT CONVERGED";
                                                                                                        case CALIBRATION_GAIN_LOOP_NOT_CONVERGED:
                                                                                                        case CALIBRATION_GAIN_LOOP_IDLE:
                                                                                                            return "NOT CONVERGED";
                                                                                                        case CALIBRATION_GAIN_LOOP_FAILED:
                                                                                                        default:
                                                                                                            return correction_saturated && residual_out_of_tolerance ?             "SATURATED" : "FAIL";
                                                                                                        }
                                                                                                    }
                                                                                                    static void calibration_gain_loop_print_summary(     const calibration_gain_loop_state_t *state) {
                                                                                                        if (calibration_compact_output_enabled()) {
                                                                                                            if (state->final_status == CALIBRATION_GAIN_LOOP_PASS) return;
                                                                                                            xil_printf("\r\nGain controller         : FAILED\r\n");
                                                                                                            xil_printf("Existing gain loop status : %s\r\n",                    calibration_existing_gain_loop_status_name(state));
                                                                                                            xil_printf("Dither gain status        : %s\r\n",                    calibration_dither_gain_status_name(                        g_latest_dither_gain_diagnostic.status));
                                                                                                            print_float_value("Normalized gain", state->latest_fitted_gain, "");
                                                                                                            print_float_value("Gain error", state->latest_gain_error, "");
                                                                                                            print_float_value("Gain correction", state->gain_correction, "");
                                                                                                            xil_printf("Verification            : FAIL\r\n");
                                                                                                            xil_printf("Status                  : FAILED\r\n");
                                                                                                            xil_printf("Reason                  : %s\r\n",                    state->failure_reason != NULL &&                        strcmp(state->failure_reason, "none") != 0 ?                    state->failure_reason :                    calibration_offset_termination_name(                        state->termination_reason));
                                                                                                            return;
                                                                                                        }
                                                                                                        xil_printf("\r\n========== Gain Calibration Summary ==========\r\n");
                                                                                                        xil_printf("Iterations completed     : %lu\r\n",                (unsigned long)state->batch_iteration_count);
                                                                                                        xil_printf("Accepted frames          : %lu\r\n",                (unsigned long)state->accepted_frame_count);
                                                                                                        xil_printf("Rejected frames          : %lu\r\n",                (unsigned long)state->rejected_frame_count);
                                                                                                        xil_printf("Calibration channel      : %s\r\n",                calibration_channel_name(state->calibration_channel));
                                                                                                        print_float_value("Fixed offset correction",                       state->fixed_offset_correction, " codes");
                                                                                                        print_float_value("Initial gain correction",                       state->initial_gain_correction, "");
                                                                                                        print_float_value("Final requested gain correction",                       state->final_requested_gain_correction, "");
                                                                                                        print_float_value("Final applied gain correction",                       state->gain_correction, "");
                                                                                                        xil_printf("Gain correction hardware code: N/A (software multiplier)\r\n");
                                                                                                        xil_printf("Saturation occurred      : %s\r\n",                state->saturation_occurred ? "YES" : "NO");
                                                                                                        xil_printf("Coefficient changed      : %s\r\n",                state->coefficient_changed ? "YES" : "NO");
                                                                                                        print_float_value("Nominal system gain",                       state->nominal_system_gain, "");
                                                                                                        print_float_value("Initial raw system gain",                       state->initial_measured_gain, "");
                                                                                                        print_float_value("Initial normalized ADC gain",                       state->initial_normalized_gain, "");
                                                                                                        print_float_value("Final raw system gain",                       state->latest_raw_system_gain, "");
                                                                                                        print_float_value("Final normalized ADC gain",                       state->latest_fitted_gain, "");
                                                                                                        print_float_value("Best normalized ADC gain",                       state->best_measured_gain, "");
                                                                                                        print_float_value("Best gain correction",                       state->best_gain_correction, "");
                                                                                                        print_float_value("Best gain error", state->best_gain_error, "");
                                                                                                        print_float_value("Final gain RMSE", state->latest_rmse, " codes");
                                                                                                        print_float_value("Best gain RMSE", state->best_rmse, " codes");
                                                                                                        print_float_value("Gain standard deviation",                       state->latest_gain_stddev, "");
                                                                                                        print_float_value("Gain standard error",                       state->latest_gain_standard_error, "");
                                                                                                        print_float_value("Controller gain",                       state->latest_controller_gain, "");
                                                                                                        print_float_value("Filter alpha", CALIBRATION_GAIN_FILTER_ALPHA, "");
                                                                                                        xil_printf("Consecutive passes       : %lu\r\n",                (unsigned long)state->convergence_count);
                                                                                                        xil_printf("Existing gain loop status : %s\r\n",                calibration_existing_gain_loop_status_name(state));
                                                                                                        xil_printf("Dither gain status        : %s\r\n",                calibration_dither_gain_status_name(                    g_latest_dither_gain_diagnostic.status));
                                                                                                        xil_printf("Termination reason       : %s\r\n",                calibration_offset_termination_name(state->termination_reason));
                                                                                                        xil_printf("Calibration status       : %s\r\n",                calibration_gain_loop_status_name(state->final_status));
                                                                                                        xil_printf("Failure reason           : %s\r\n",                state->failure_reason != NULL ? state->failure_reason : "none");
                                                                                                        calibration_print_dither_gain_diagnostic(         &g_latest_dither_gain_diagnostic);
                                                                                                        xil_printf("=============================================\r\n");
                                                                                                    }
                                                                                                    static void handle_adc_gain_calibration_status_cmd(void) {
                                                                                                        const calibration_gain_loop_state_t *state =         calibration_gain_loop_state();
                                                                                                        xil_printf("\r\n========== ADC Gain Calibration Status ==========\r\n");
                                                                                                        print_float_value("Gain correction", state->gain_correction, "");
                                                                                                        print_float_value("Fixed offset correction",                       state->fixed_offset_correction, " codes");
                                                                                                        xil_printf("Accepted frames        : %lu\r\n",                (unsigned long)state->accepted_frame_count);
                                                                                                        xil_printf("Rejected frames        : %lu\r\n",                (unsigned long)state->rejected_frame_count);
                                                                                                        xil_printf("Consecutive passes     : %lu\r\n",                (unsigned long)state->convergence_count);
                                                                                                        xil_printf("Calibration channel    : %s\r\n",                calibration_channel_name(state->calibration_channel));
                                                                                                        print_float_value("Latest fitted gain", state->latest_fitted_gain, "");
                                                                                                        print_float_value("Latest normalized gain residual",                       state->latest_gain_error, "");
                                                                                                        print_float_value("Latest centered waveform RMSE",                       state->latest_waveform_rmse, " codes");
                                                                                                        print_float_value("Latest waveform RMSE improvement",                       state->latest_waveform_rmse_improvement, " codes");
                                                                                                        print_float_value("Latest corrected ADC mean",                       state->latest_fitted_offset, " codes");
                                                                                                        print_float_value("Latest correlation", state->latest_correlation, "");
                                                                                                        print_float_value("Latest fitted RMSE", state->latest_rmse, " codes");
                                                                                                        xil_printf("Existing gain loop status : %s\r\n",                calibration_existing_gain_loop_status_name(state));
                                                                                                        xil_printf("Dither gain status        : %s\r\n",                calibration_dither_gain_status_name(                    g_latest_dither_gain_diagnostic.status));
                                                                                                        xil_printf("Calibration status     : %s\r\n",                calibration_gain_loop_status_name(state->final_status));
                                                                                                        xil_printf("================================================\r\n");
                                                                                                    }
                                                                                                    static int calibration_publish_gain_output(     const calibration_pending_frame_t *input,     calibration_frame_workspace_t *workspace,     float gain_correction,     float offset_correction,     float nominal_system_gain) {
                                                                                                        calibration_pending_frame_t best;
                                                                                                        calibration_gain_loop_state_t *state = calibration_gain_loop_state();
                                                                                                        double post_gain_residual_sum = 0.0;
                                                                                                        double post_gain_residual_square_sum = 0.0;
                                                                                                        float best_error = FLT_MAX;
                                                                                                        uint32_t accepted = 0U;
                                                                                                        uint32_t post_gain_residual_count = 0U;
                                                                                                        const bool compact = calibration_compact_output_enabled();
                                                                                                        memset(&best, 0, sizeof(best));
                                                                                                        calibration_gain_input_frame_invalidate();
                                                                                                        if (!compact) {
                                                                                                            xil_printf("\r\n========== ADC Calibrated Output ==========\r\n");
                                                                                                            xil_printf("Source stage            : gain calibration\r\n");
                                                                                                        }
                                                                                                        for (uint32_t i = 1U;
                                                                                                        i <= CALIBRATION_GAIN_BATCH_SIZE;
                                                                                                        ++i) {
                                                                                                            calibration_aligned_frame_t frame;
                                                                                                            calibration_fixed_offset_gain_metrics_t gain_metrics;
                                                                                                            adc_final_reference_metrics_t reference_metrics;
                                                                                                            const char *reason = NULL;
                                                                                                            if (calibration_capture_against_owned_reference(                 input, true, gain_correction,                 offset_correction,                 CAL_ADC_FULL_SCALE_CODES / CAL_DAC_FULL_SCALE_CODES,                 workspace, &frame, &reason) == 0 && frame.frame_valid &&             calibration_estimate_gain_fixed_offset(                 &frame, gain_correction, offset_correction,                 &gain_metrics) == 0) {
                                                                                                                const float normalized_gain =                 gain_metrics.raw_system_gain / nominal_system_gain;
                                                                                                                const float error = fabsf(normalized_gain - 1.0f);
                                                                                                                ++accepted;
                                                                                                                if (adc_calculate_final_reference_metrics(                     &frame, gain_correction, offset_correction,                     nominal_system_gain, &reference_metrics) == 0) {
                                                                                                                    post_gain_residual_sum += reference_metrics.mean_residual;
                                                                                                                    post_gain_residual_square_sum +=                     (double)reference_metrics.mean_residual *                     reference_metrics.mean_residual;
                                                                                                                    ++post_gain_residual_count;
                                                                                                                }
                                                                                                                if (error < best_error &&                 calibration_pending_frame_copy(&best, &frame, i,                     reference_buffer_generation(), reference_buffer_length(),                     reference_buffer_format()) == 0) {
                                                                                                                    best_error = error;
                                                                                                                }
                                                                                                            }
                                                                                                            if (i < CALIBRATION_GAIN_BATCH_SIZE)             usleep(ADC_TIMING_INTERFRAME_DELAY_US);
                                                                                                        }
                                                                                                        if (post_gain_residual_count > 0U) {
                                                                                                            const double count = (double)post_gain_residual_count;
                                                                                                            const double mean = post_gain_residual_sum / count;
                                                                                                            const double variance = post_gain_residual_count > 1U ?             fmax(0.0,                 (post_gain_residual_square_sum -                  post_gain_residual_sum * post_gain_residual_sum / count) /                 (count - 1.0)) : 0.0;
                                                                                                            state->post_gain_residual = (float)mean;
                                                                                                            state->post_gain_residual_stddev = (float)sqrt(variance);
                                                                                                            state->post_gain_residual_standard_error =             state->post_gain_residual_stddev /             sqrtf((float)post_gain_residual_count);
                                                                                                            state->post_gain_residual_frames = post_gain_residual_count;
                                                                                                            state->post_gain_residual_valid = 1U;
                                                                                                        }
                                                                                                        if (!compact) {
                                                                                                            print_float_value("Offset correction", offset_correction, " codes");
                                                                                                            print_float_value("Gain correction", gain_correction, "");
                                                                                                            xil_printf("Verification frames     : %lu/%u\r\n",                    (unsigned long)accepted, CALIBRATION_GAIN_BATCH_SIZE);
                                                                                                        }
                                                                                                        if (!best.valid) {
                                                                                                            if (compact) {
                                                                                                                xil_printf("Final calibrated capture: FAILED\r\n");
                                                                                                                xil_printf("Reason                  : no valid gain verification output\r\n");
                                                                                                            }
                                                                                                            else {
                                                                                                                xil_printf("Capture status          : NOT READY\r\n");
                                                                                                                xil_printf("Output ready            : NO\r\n");
                                                                                                                xil_printf("===========================================\r\n");
                                                                                                            }
                                                                                                            return -1;
                                                                                                        }
                                                                                                        g_pending_calibration_frame = best;
                                                                                                        g_pending_calibration_frame.valid = true;
                                                                                                        g_pending_calibration_frame.consumed = false;
                                                                                                        if (compact && calibration_gain_loop_state()->final_status ==             CALIBRATION_GAIN_LOOP_PASS) {
                                                                                                            const float normalized_gain =             best.metrics.measured_gain / nominal_system_gain;
                                                                                                            xil_printf("\r\nVerification            : PASS\r\n");
                                                                                                            xil_printf("Verification frames     : %lu\r\n",                    (unsigned long)accepted);
                                                                                                            print_float_value("Normalized final gain", normalized_gain, "");
                                                                                                            print_float_value("Verification error", normalized_gain - 1.0f, "");
                                                                                                            print_float_value("Verification correlation", best.correlation, "");
                                                                                                            print_float_value("Gain correction", gain_correction, "");
                                                                                                            xil_printf("Existing gain loop status : %s\r\n",            calibration_existing_gain_loop_status_name(state));
                                                                                                            xil_printf("Dither gain status        : %s\r\n",            calibration_dither_gain_status_name(                g_latest_dither_gain_diagnostic.status));
                                                                                                            xil_printf("Status                  : CONVERGED\r\n");
                                                                                                            xil_printf("\r\nFinal calibrated capture: READY (%lu samples)\r\n",                    (unsigned long)best.analysis_sample_count);
                                                                                                            print_signed_float_value_or_invalid("Post-gain residual check",             state->post_gain_residual_valid ?                 state->post_gain_residual : NAN, " codes");
                                                                                                        }
                                                                                                        else if (!compact) {
                                                                                                            xil_printf("Capture status          : READY\r\n");
                                                                                                            print_float_value("Correlation", best.correlation, "");
                                                                                                            xil_printf("Samples                 : %lu\r\n",                    (unsigned long)best.analysis_sample_count);
                                                                                                            xil_printf("Output ready            : YES\r\n");
                                                                                                            print_signed_float_value_or_invalid("Post-gain residual check",             state->post_gain_residual_valid ?                 state->post_gain_residual : NAN, " codes");
                                                                                                            if (ADC_CAL_VERBOSE_DEBUG) {
                                                                                                                print_float_value_or_invalid("Post-gain residual std dev",                 state->post_gain_residual_valid ?                     state->post_gain_residual_stddev : NAN, " codes");
                                                                                                                print_float_value_or_invalid("Post-gain residual SE",                 state->post_gain_residual_valid ?                     state->post_gain_residual_standard_error : NAN,                 " codes");
                                                                                                                xil_printf("Post-gain residual frames: %lu\r\n",                 (unsigned long)state->post_gain_residual_frames);
                                                                                                            }
                                                                                                            xil_printf("===========================================\r\n");
                                                                                                        }
                                                                                                        return 0;
                                                                                                    }
                                                                                                    static void handle_adc_gain_calibration_loop_cmd(void) {
                                                                                                        static calibration_pending_frame_t gain_input;
                                                                                                        static calibration_frame_workspace_t workspace;
                                                                                                        calibration_gain_loop_state_t *state = calibration_gain_loop_state();
                                                                                                        const char *reason = NULL;
                                                                                                        bool baseline_validated = false;
                                                                                                        float gain_stage_offset;
                                                                                                        const bool compact = calibration_compact_output_enabled();
                                                                                                        if (adc_sweep_active || RxBufferPtr == NULL) {
                                                                                                            ERR("ADC capture is unavailable for gain calibration.");
                                                                                                            state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                            return;
                                                                                                        }
                                                                                                        if (!calibration_pending_frame_is_compatible(&reason)) {
                                                                                                            xil_printf("ERROR\r\n\r\nNo calibrated offset capture available.\r\n");
                                                                                                            xil_printf("Run 'adc -cal' for automatic calibration, or run\r\n");
                                                                                                            xil_printf("'adc -cal timing' then 'adc -cal offset' for stage testing.\r\n");
                                                                                                            return;
                                                                                                        }
                                                                                                        gain_input = g_pending_calibration_frame;
                                                                                                        calibration_gain_loop_begin_run(state);
                                                                                                        memset(&g_latest_dither_gain_diagnostic, 0,         sizeof(g_latest_dither_gain_diagnostic));
                                                                                                        if (compact) {
                                                                                                            xil_printf("Offset input status     : %s\r\n",                    gain_input.source_offset_result ==                        CALIBRATION_OFFSET_RESULT_CONVERGED ?                    "CONVERGED" : "PROVISIONAL");
                                                                                                            print_float_value("Fixed offset correction",                           gain_input.software_offset_correction, " codes");
                                                                                                        }
                                                                                                        else {
                                                                                                            xil_printf("Offset output valid      : %s\r\n",                    gain_input.valid && !gain_input.consumed ? "YES" : "NO");
                                                                                                            xil_printf("Offset input status      : %s\r\n",                    gain_input.source_offset_result ==                        CALIBRATION_OFFSET_RESULT_CONVERGED ?                    "CONVERGED" : "PROVISIONAL");
                                                                                                            xil_printf("Offset output channel    : %s\r\n",                    calibration_channel_name(gain_input.selected_channel));
                                                                                                            print_float_value("Fixed offset correction",                           gain_input.software_offset_correction, " codes");
                                                                                                            xil_printf("Gain requested channel   : %s\r\n",                    calibration_channel_name(                        calibration_channel_selection()));
                                                                                                        }
                                                                                                        if (!gain_input.valid || gain_input.consumed ||         gain_input.selected_channel < 0 ||         gain_input.selected_channel > 1 ||         (gain_input.source_offset_result !=              CALIBRATION_OFFSET_RESULT_CONVERGED &&          gain_input.source_offset_result !=              CALIBRATION_OFFSET_RESULT_PROVISIONAL) ||         !isfinite(gain_input.software_offset_correction) ||         (calibration_channel_selection() >= 0 &&          calibration_channel_selection() != gain_input.selected_channel)) {
                                                                                                            state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                            state->failure_reason = "offset-to-gain calibration state mismatch";
                                                                                                            calibration_gain_loop_print_summary(state);
                                                                                                            return;
                                                                                                        }
                                                                                                        state->calibration_channel = gain_input.selected_channel;
                                                                                                        state->fixed_offset_correction =         gain_input.software_offset_correction;
                                                                                                        gain_stage_offset = state->fixed_offset_correction;
                                                                                                        adc_sweep_active = 1U;
                                                                                                        g_quiet_calibration_capture = true;
                                                                                                        {
                                                                                                            calibration_aligned_frame_t nominal_frame;
                                                                                                            calibration_fixed_offset_gain_metrics_t nominal_metrics;
                                                                                                            if (calibration_pending_frame_consume(                 1.0f, gain_stage_offset,                 CAL_ADC_FULL_SCALE_CODES / CAL_DAC_FULL_SCALE_CODES,                 &workspace, &nominal_frame, &reason) != 0 ||             calibration_estimate_gain_fixed_offset(                 &nominal_frame, 1.0f, gain_stage_offset,                 &nominal_metrics) != 0 ||             nominal_metrics.raw_system_gain <= FLT_EPSILON ||             calibration_set_software_gain_correction(1.0f) != 0) {
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                                state->failure_reason = "unable to establish nominal system gain";
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                                calibration_gain_loop_print_summary(state);
                                                                                                                g_quiet_calibration_capture = false;
                                                                                                                adc_sweep_active = 0U;
                                                                                                                return;
                                                                                                            }
                                                                                                            state->gain_correction = 1.0f;
                                                                                                            state->initial_gain_correction = 1.0f;
                                                                                                            state->final_requested_gain_correction = 1.0f;
                                                                                                            state->nominal_system_gain = nominal_metrics.raw_system_gain;
                                                                                                            state->nominal_system_gain_valid = 1U;
                                                                                                            state->initial_measured_gain = nominal_metrics.raw_system_gain;
                                                                                                            state->initial_measured_gain_valid = 1U;
                                                                                                            state->initial_normalized_gain = 1.0f;
                                                                                                            if (ADC_CAL_VERBOSE_DEBUG)             calibration_print_fixed_window(&nominal_frame);
                                                                                                            if (!compact) {
                                                                                                                print_float_value("Nominal system gain",                               state->nominal_system_gain, "");
                                                                                                                print_float_value("Initial measured gain",                               state->initial_measured_gain, "");
                                                                                                                print_float_value("Initial normalized gain",                               state->initial_normalized_gain, "");
                                                                                                                print_float_value("Nominal raw ADC mean",                               nominal_metrics.raw_adc_mean, " codes");
                                                                                                                print_float_value("Nominal offset-corrected ADC mean",                               nominal_metrics.corrected_adc_mean, " codes");
                                                                                                                print_float_value("Nominal reference mean",                               nominal_metrics.reference_mean, " codes");
                                                                                                            }
                                                                                                        }
                                                                                                        if (!compact) {
                                                                                                            xil_printf("\r\n========== ADC Gain Calibration ==========\r\n");
                                                                                                            xil_printf("Gain input source       : offset-calibrated capture\r\n");
                                                                                                            xil_printf("Source status           : %s\r\n",                    gain_input.source_offset_result ==                        CALIBRATION_OFFSET_RESULT_CONVERGED ?                    "CONVERGED" : "PROVISIONAL");
                                                                                                            xil_printf("Input frame             : Frame %lu\r\n",                    (unsigned long)gain_input.retained_frame_number);
                                                                                                            xil_printf("Channel                 : %s\r\n",                    calibration_channel_name(gain_input.selected_channel));
                                                                                                            xil_printf("Canonical reference phase: %s\r\n",                    gain_input.canonical_reference_phase == 0 ?                    "EVEN" : "ODD");
                                                                                                            xil_printf("Selected input phase     : %s\r\n",                    gain_input.selected_reference_phase == 0 ?                    "EVEN" : "ODD");
                                                                                                            xil_printf("Samples                 : %lu\r\n",                    (unsigned long)gain_input.analysis_sample_count);
                                                                                                        }
                                                                                                        while (state->batch_iteration_count <                CALIBRATION_GAIN_MAX_ACCEPTED_ITERATIONS &&            state->convergence_count <                CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES) {
                                                                                                            calibration_gain_batch_result_t batch;
                                                                                                            float gain_update = 0.0f;
                                                                                                            float desired_gain;
                                                                                                            float requested_gain;
                                                                                                            float relative_update;
                                                                                                            float previous_gain;
                                                                                                            float score;
                                                                                                            float next_gain;
                                                                                                            bool pass;
                                                                                                            ++state->batch_iteration_count;
                                                                                                            if (!compact)             xil_printf("\r\n========== Gain Batch %lu ==========\r\n",                        (unsigned long)state->batch_iteration_count);
                                                                                                            if (state->fixed_offset_correction != gain_stage_offset ||             calibration_software_offset_correction() != gain_stage_offset) {
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                                state->failure_reason =                 "fixed offset changed during gain calibration";
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                                break;
                                                                                                            }
                                                                                                            if (!compact)             print_float_value("Fixed offset correction",                               gain_stage_offset, " codes");
                                                                                                            if (calibration_compute_gain_batch(&gain_input,                 state->gain_correction, gain_stage_offset,                 state->nominal_system_gain,                 &workspace, state, &batch,                 false) != 0) {
                                                                                                                state->convergence_count = 0U;
                                                                                                                ++state->no_improvement_count;
                                                                                                                if (compact)                 calibration_print_gain_batch_compact(                     state->batch_iteration_count, &batch, state,                     false, "REJECTED");
                                                                                                                continue;
                                                                                                            }
                                                                                                            if (!baseline_validated) {
                                                                                                                if (!compact) {
                                                                                                                    print_float_value("Nominal system gain",                                   state->nominal_system_gain, "");
                                                                                                                    print_float_value("Current system gain",                                   batch.mean_raw_system_gain, "");
                                                                                                                    print_float_value("Normalized gain", batch.mean_gain, "");
                                                                                                                }
                                                                                                                if (fabsf(batch.mean_gain - 1.0f) >                     CALIBRATION_GAIN_BASELINE_TOLERANCE) {
                                                                                                                    state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                                    state->failure_reason = "gain baseline inconsistency";
                                                                                                                    state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                                    xil_printf("ERROR: gain baseline inconsistency\r\n");
                                                                                                                    break;
                                                                                                                }
                                                                                                                baseline_validated = true;
                                                                                                            }
                                                                                                            state->latest_fitted_gain = batch.mean_gain;
                                                                                                            state->latest_raw_system_gain = batch.mean_raw_system_gain;
                                                                                                            state->latest_gain_error = batch.mean_error;
                                                                                                            state->latest_fitted_offset = batch.mean_corrected_adc;
                                                                                                            state->latest_rmse = batch.mean_gain_rmse;
                                                                                                            state->latest_correlation = batch.mean_correlation;
                                                                                                            state->latest_gain_stddev = batch.stddev;
                                                                                                            state->latest_gain_standard_error = batch.standard_error;
                                                                                                            if (state->filtered_gain_error_valid == 0U) {
                                                                                                                state->filtered_gain_error = batch.mean_error;
                                                                                                                state->filtered_gain_error_valid = 1U;
                                                                                                            }
                                                                                                            else {
                                                                                                                state->filtered_gain_error =                 CALIBRATION_GAIN_FILTER_ALPHA * state->filtered_gain_error +                 (1.0f - CALIBRATION_GAIN_FILTER_ALPHA) * batch.mean_error;
                                                                                                            }
                                                                                                            /* Fitted RMSE scales with this software multiplier and therefore          * must not bias the best-coefficient selection back toward unity. */
                                                                                                            score = fabsf(batch.mean_error) + 0.5f * batch.standard_error;
                                                                                                            if (score < state->best_score) {
                                                                                                                const float improvement = state->best_score < FLT_MAX ?                 state->best_score - score : FLT_MAX;
                                                                                                                state->best_score = score;
                                                                                                                state->best_gain_correction = state->gain_correction;
                                                                                                                state->best_gain_error = batch.mean_error;
                                                                                                                state->best_measured_gain = batch.mean_gain;
                                                                                                                state->best_rmse = batch.mean_gain_rmse;
                                                                                                                state->no_improvement_count =                 improvement >= CALIBRATION_GAIN_MIN_IMPROVEMENT ? 0U :                 state->no_improvement_count + 1U;
                                                                                                            }
                                                                                                            else {
                                                                                                                ++state->no_improvement_count;
                                                                                                            }
                                                                                                            if (!isfinite(batch.mean_gain) || batch.mean_gain <= FLT_EPSILON) {
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                                state->failure_reason = "measured gain is not positive";
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                                break;
                                                                                                            }
                                                                                                            desired_gain = state->gain_correction / batch.mean_gain;
                                                                                                            relative_update = fabsf(desired_gain - state->gain_correction) /                           fabsf(state->gain_correction);
                                                                                                            pass = fabsf(state->filtered_gain_error) <=                    CALIBRATION_GAIN_TOLERANCE &&                batch.standard_error <                    CALIBRATION_GAIN_MAX_STANDARD_ERROR &&                relative_update <= CALIBRATION_GAIN_UPDATE_TOLERANCE;
                                                                                                            state->latest_controller_gain =             fabsf(state->filtered_gain_error) < CALIBRATION_GAIN_NEAR_ERROR ?             CALIBRATION_GAIN_NEAR_UPDATE_STEP :             CALIBRATION_GAIN_UPDATE_STEP;
                                                                                                            if (pass) {
                                                                                                                ++state->convergence_count;
                                                                                                            }
                                                                                                            else {
                                                                                                                state->convergence_count = 0U;
                                                                                                                gain_update = state->latest_controller_gain *                           (desired_gain - state->gain_correction);
                                                                                                            }
                                                                                                            if (!compact) {
                                                                                                                xil_printf("\r\n---------- Gain Batch Statistics ----------\r\n");
                                                                                                                print_float_value("Raw system gain",                               batch.mean_raw_system_gain, "");
                                                                                                                print_float_value("Nominal system gain",                               state->nominal_system_gain, "");
                                                                                                                print_float_value("Normalized ADC gain", batch.mean_gain, "");
                                                                                                                print_float_value("Gain error", batch.mean_error, "");
                                                                                                                print_float_value("Filtered gain error",                               state->filtered_gain_error, "");
                                                                                                                print_float_value("Normalized gain std dev", batch.stddev, "");
                                                                                                                print_float_value("Normalized gain standard error",                               batch.standard_error, "");
                                                                                                                print_float_value("Fixed offset correction",                               gain_stage_offset, " codes");
                                                                                                                print_float_value("Corrected ADC mean",                               batch.mean_corrected_adc, " codes");
                                                                                                                print_float_value("Gain RMSE", batch.mean_gain_rmse, " codes");
                                                                                                                print_float_value("Correlation", batch.mean_correlation, "");
                                                                                                                xil_printf("Accepted frames         : %lu/%u\r\n",                        (unsigned long)batch.accepted,                        CALIBRATION_GAIN_BATCH_SIZE);
                                                                                                                xil_printf("Rejected frames         : %lu/%u\r\n",                        (unsigned long)batch.rejected,                        CALIBRATION_GAIN_BATCH_SIZE);
                                                                                                                xil_printf("Dither gain frames      : PASS %lu | WARNING %lu | INVALID %lu\r\n",                        (unsigned long)batch.dither_pass,                        (unsigned long)batch.dither_warning,                        (unsigned long)batch.dither_invalid);
                                                                                                                if (batch.dither_valid_estimates > 0U) {
                                                                                                                    print_double_value("Mean dither gain",                               batch.mean_dither_gain, "");
                                                                                                                    print_double_value("Mean dither flat gain",                               batch.mean_dither_flat_gain, "");
                                                                                                                    print_double_value("Mean existing-dither delta",                               batch.mean_existing_dither_delta, "");
                                                                                                                }
                                                                                                                else if (batch.dither_invalid > 0U) {
                                                                                                                    xil_printf("Dither gain rejection   : %s\r\n",                            calibration_dither_gain_reason_name(                                batch.dither_latest.reason));
                                                                                                                }
                                                                                                            }
                                                                                                            if (!pass && state->no_improvement_count >=                 CALIBRATION_GAIN_NO_IMPROVEMENT_LIMIT) {
                                                                                                                state->termination_reason =                 CAL_OFFSET_TERMINATION_NO_IMPROVEMENT;
                                                                                                                if (compact)                 calibration_print_gain_batch_compact(                     state->batch_iteration_count, &batch, state,                     false, "FAILED");
                                                                                                                break;
                                                                                                            }
                                                                                                            previous_gain = state->gain_correction;
                                                                                                            requested_gain = state->gain_correction + gain_update;
                                                                                                            state->final_requested_gain_correction = requested_gain;
                                                                                                            next_gain = fmaxf(CALIBRATION_GAIN_CORRECTION_MIN,             fminf(CALIBRATION_GAIN_CORRECTION_MAX, requested_gain));
                                                                                                            if (next_gain != requested_gain)             state->saturation_occurred = 1U;
                                                                                                            if (!g_automatic_calibration.active || ADC_CAL_VERBOSE_DEBUG) {
                                                                                                                xil_printf("\r\nGain controller:\r\n");
                                                                                                                print_float_value("  raw system gain",                               batch.mean_raw_system_gain, "");
                                                                                                                print_float_value("  nominal system gain",                               state->nominal_system_gain, "");
                                                                                                                print_float_value("  normalized ADC gain", batch.mean_gain, "");
                                                                                                                print_float_value("  gain error", batch.mean_error, "");
                                                                                                                print_float_value("  filtered gain error",                               state->filtered_gain_error, "");
                                                                                                                print_float_value("  previous correction", previous_gain, "");
                                                                                                                print_float_value("  desired correction", desired_gain, "");
                                                                                                                print_float_value("  requested correction", requested_gain, "");
                                                                                                                print_float_value("  clamped correction", next_gain, "");
                                                                                                                xil_printf("  HW coefficient        : N/A (software multiplier)\r\n");
                                                                                                                xil_printf("  register address      : N/A\r\n");
                                                                                                                xil_printf("  register write        : N/A\r\n");
                                                                                                                xil_printf("  register readback     : N/A\r\n");
                                                                                                            }
                                                                                                            if (!isfinite(next_gain) ||             (gain_update != 0.0f &&              calibration_set_software_gain_correction(next_gain) != 0)) {
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                                break;
                                                                                                            }
                                                                                                            if (gain_update != 0.0f) state->gain_correction = next_gain;
                                                                                                            if (state->gain_correction != state->initial_gain_correction)             state->coefficient_changed = 1U;
                                                                                                            if (!compact) {
                                                                                                                print_float_value("  effective software correction",                               calibration_software_gain_correction(), "");
                                                                                                                print_float_value("Fixed offset correction",                               state->fixed_offset_correction, " codes");
                                                                                                                print_float_value("Current gain correction",                               state->gain_correction, "");
                                                                                                            }
                                                                                                            if (state->previous_measured_gain_valid) {
                                                                                                                const float observed_gain_change =                 fabsf(batch.mean_gain - state->previous_measured_gain);
                                                                                                                const float combined_gain_standard_error = hypotf(                 batch.standard_error,                 state->previous_gain_standard_error);
                                                                                                                const float observable_threshold = fmaxf(                 CALIBRATION_GAIN_EFFECT_EPSILON,                 CALIBRATION_GAIN_EFFECT_SIGMA_MULTIPLIER *                     combined_gain_standard_error);
                                                                                                                const float prior_measurement_correction =                 previous_gain - state->last_applied_gain_delta;
                                                                                                                const float expected_gain_response =                 fabsf(prior_measurement_correction) > FLT_EPSILON ?                 fabsf(state->previous_measured_gain *                     state->last_applied_gain_delta /                     prior_measurement_correction) : 0.0f;
                                                                                                                const bool meaningful_previous_update =                 fabsf(state->last_applied_gain_delta) > FLT_EPSILON &&                 expected_gain_response >= observable_threshold;
                                                                                                                if (!compact) {
                                                                                                                    print_float_value("Previous measured gain",                                   state->previous_measured_gain, "");
                                                                                                                    print_float_value("New measured gain", batch.mean_gain, "");
                                                                                                                    print_float_value("Applied correction delta",                                   state->last_applied_gain_delta, "");
                                                                                                                }
                                                                                                                if (ADC_CAL_VERBOSE_DEBUG) {
                                                                                                                    print_float_value("Observed gain response",                                   observed_gain_change, "");
                                                                                                                    print_float_value("Expected gain response",                                   expected_gain_response, "");
                                                                                                                    print_float_value("Combined gain standard error",                                   combined_gain_standard_error, "");
                                                                                                                    print_float_value("Observable response threshold",                                   observable_threshold, "");
                                                                                                                }
                                                                                                                if (meaningful_previous_update &&                 observed_gain_change < observable_threshold) {
                                                                                                                    ++state->no_observable_effect_count;
                                                                                                                    if (state->no_observable_effect_count ==                         CALIBRATION_GAIN_NO_EFFECT_LIMIT)                     xil_printf("WARNING: gain coefficient update has no observable effect\r\n");
                                                                                                                }
                                                                                                                else {
                                                                                                                    state->no_observable_effect_count = 0U;
                                                                                                                }
                                                                                                            }
                                                                                                            state->previous_measured_gain = batch.mean_gain;
                                                                                                            state->previous_gain_standard_error = batch.standard_error;
                                                                                                            state->previous_measured_gain_valid = 1U;
                                                                                                            state->last_applied_gain_delta = next_gain - previous_gain;
                                                                                                            if (compact) {
                                                                                                                calibration_print_gain_batch_compact(                 state->batch_iteration_count, &batch, state,                 pass, "RUNNING");
                                                                                                            }
                                                                                                            else {
                                                                                                                print_float_value("Gain update", gain_update, "");
                                                                                                                print_float_value("Gain correction", state->gain_correction, "");
                                                                                                                xil_printf("Consecutive passes      : %lu\r\n",                        (unsigned long)state->convergence_count);
                                                                                                                xil_printf("Batch status            : %s\r\n",                        pass ? "PASS" : "ACCEPTED");
                                                                                                            }
                                                                                                        }
                                                                                                        if (state->final_status == CALIBRATION_GAIN_LOOP_RUNNING) {
                                                                                                            if (state->convergence_count >=             CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES) {
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_PASS;
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_CONVERGED;
                                                                                                            }
                                                                                                            else if (!state->coefficient_changed &&                    fabsf(state->latest_gain_error) >                        CALIBRATION_GAIN_TOLERANCE) {
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                                state->failure_reason = state->saturation_occurred ?                 "gain correction saturated" :                 "gain coefficient did not change";
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                            }
                                                                                                            else if (state->saturation_occurred &&                    fabsf(state->latest_gain_error) >                        CALIBRATION_GAIN_TOLERANCE) {
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                                state->failure_reason = "gain correction saturated";
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                            }
                                                                                                            else if (state->best_score < FLT_MAX &&                    calibration_set_software_gain_correction(                        state->best_gain_correction) == 0) {
                                                                                                                state->gain_correction = state->best_gain_correction;
                                                                                                                state->restored_best_solution = 1U;
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_BEST_AVAILABLE;
                                                                                                                if (state->termination_reason == CAL_OFFSET_TERMINATION_NONE)                 state->termination_reason =                     CAL_OFFSET_TERMINATION_ITERATION_LIMIT;
                                                                                                            }
                                                                                                            else {
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                            }
                                                                                                        }
                                                                                                        if (state->final_status == CALIBRATION_GAIN_LOOP_PASS ||         state->final_status == CALIBRATION_GAIN_LOOP_BEST_AVAILABLE) {
                                                                                                            if (calibration_publish_gain_output(&gain_input, &workspace,                 state->gain_correction, state->fixed_offset_correction,                 state->nominal_system_gain) != 0) {
                                                                                                                state->final_status = CALIBRATION_GAIN_LOOP_FAILED;
                                                                                                                state->termination_reason = CAL_OFFSET_TERMINATION_ERROR;
                                                                                                            }
                                                                                                        }
                                                                                                        else {
                                                                                                            calibration_gain_input_frame_invalidate();
                                                                                                        }
                                                                                                        calibration_gain_loop_print_summary(state);
                                                                                                        g_quiet_calibration_capture = false;
                                                                                                        adc_sweep_active = 0U;
                                                                                                    }
                                                                                                    static int adc_run_timing_calibration(uint32_t frame_count) {
                                                                                                        static int16_t even_reference[ADC_VALID_SAMPLE_COUNT];
                                                                                                        static int16_t odd_reference[ADC_VALID_SAMPLE_COUNT];
                                                                                                        static calibration_frame_workspace_t frame_workspace;
                                                                                                        static float correlations[ADC_CAL_MAX_FRAMES];
                                                                                                        static float integer_lags[ADC_CAL_MAX_FRAMES];
                                                                                                        static float fractional_lags[ADC_CAL_MAX_FRAMES];
                                                                                                        static float total_lags[ADC_CAL_MAX_FRAMES];
                                                                                                        static calibration_pending_frame_t         accepted_candidates[ADC_CAL_MAX_FRAMES];
                                                                                                        size_t reconstructed_count = 0U;
                                                                                                        uint32_t accepted_frames = 0U;
                                                                                                        size_t candidate_count = 0U;
                                                                                                        size_t selected_candidate = 0U;
                                                                                                        int calibration_channel = calibration_channel_selection();
                                                                                                        int alignment_pass = 0;
                                                                                                        int representative_selected = 0;
                                                                                                        double sum_correlation = 0.0;
                                                                                                        double even_variance;
                                                                                                        double odd_variance;
                                                                                                        uint32_t reference_generation;
                                                                                                        size_t uploaded_reference_length;
                                                                                                        reference_buffer_format_t uploaded_reference_format;
                                                                                                        calibration_selection_medians_t selection_medians;
                                                                                                        float minimum_correlation = 0.0f;
                                                                                                        float median_total_lag = 0.0f;
                                                                                                        const bool compact = calibration_compact_output_enabled();
                                                                                                        calibration_pending_frame_invalidate();
                                                                                                        if ((frame_count < ADC_CAL_MIN_FRAMES) ||         (frame_count > ADC_CAL_MAX_FRAMES)) {
                                                                                                            ERR("Calibration frame count must be between %u and %u.",             ADC_CAL_MIN_FRAMES, ADC_CAL_MAX_FRAMES);
                                                                                                            return -1;
                                                                                                        }
                                                                                                        if (!compact) print_adc_analysis_rate_header();
                                                                                                        if (calibration_prepare_uploaded_dac_reference(             even_reference, odd_reference, &reconstructed_count,             &even_variance, &odd_variance, 1) != 0) return -1;
                                                                                                        reference_generation = reference_buffer_generation();
                                                                                                        uploaded_reference_length = reference_buffer_length();
                                                                                                        uploaded_reference_format = reference_buffer_format();
                                                                                                        if (adc_sweep_active)     {
                                                                                                            ERR("Another automatic ADC capture is already in progress.");
                                                                                                            return -1;
                                                                                                        }
                                                                                                        adc_sweep_active = 1;
                                                                                                        g_quiet_calibration_capture = true;
                                                                                                        if (!compact) {
                                                                                                            xil_printf("\r\n");
                                                                                                            xil_printf("ADC Timing Alignment Measurement\r\n");
                                                                                                            xil_printf("Reference source : uploaded DAC TXT\r\n");
                                                                                                            xil_printf("Requested frames : %lu\r\n",                    (unsigned long)frame_count);
                                                                                                        }
                                                                                                        for (uint32_t frame = 1U;
                                                                                                        frame <= frame_count;
                                                                                                        ++frame)     {
                                                                                                            calibration_frame_config_t frame_config;
                                                                                                            calibration_aligned_frame_t aligned_frame;
                                                                                                            int fit_status;
                                                                                                            if (!g_automatic_calibration.active || ADC_CAL_VERBOSE_DEBUG)             xil_printf("\r\n---------- Frame %lu ----------\r\n",                        (unsigned long)frame);
                                                                                                            frame_config.locked_channel = calibration_channel;
                                                                                                            frame_config.adc_gain_correction =             calibration_software_gain_correction();
                                                                                                            frame_config.adc_offset_correction =             calibration_software_offset_correction();
                                                                                                            frame_config.reference_scale = 1.0f;
                                                                                                            frame_config.reject_clipped_input = false;
                                                                                                            fit_status = calibration_capture_and_align(             even_reference, odd_reference, reconstructed_count,             &frame_config, &frame_workspace, &aligned_frame         );
                                                                                                            if ((fit_status != 0) || !aligned_frame.frame_valid) {
                                                                                                                if (g_automatic_calibration.active &&                 !ADC_CAL_VERBOSE_DEBUG) {
                                                                                                                    xil_printf("Timing frame %lu/%lu     : REJECTED (%s)\r\n",                     (unsigned long)frame, (unsigned long)frame_count,                     aligned_frame.rejection_reason);
                                                                                                                }
                                                                                                                else {
                                                                                                                    xil_printf("Status           : REJECTED\r\n");
                                                                                                                    xil_printf("Reason           : %s\r\n",                            aligned_frame.rejection_reason);
                                                                                                                }
                                                                                                                continue;
                                                                                                            }
                                                                                                            if (!compact) {
                                                                                                                xil_printf("Channel          : %s\r\n",                        aligned_frame.selected_channel_name);
                                                                                                                xil_printf("Canonical reference phase: %s\r\n",                        aligned_frame.canonical_reference_phase == 0 ?                        "EVEN" : "ODD");
                                                                                                                xil_printf("Selected input phase     : %s\r\n",                        aligned_frame.selected_phase_name);
                                                                                                                print_float_value("Correlation", aligned_frame.correlation, "");
                                                                                                                xil_printf("Integer lag      : %ld samples\r\n",                        (long)aligned_frame.integer_lag);
                                                                                                                print_float_value("Fractional lag",                               aligned_frame.fractional_lag, " samples");
                                                                                                                xil_printf("Status           : ACCEPTED\r\n");
                                                                                                            }
                                                                                                            if (ADC_CAL_VERBOSE_DEBUG)             calibration_print_fixed_window(&aligned_frame);
                                                                                                            correlations[accepted_frames] = aligned_frame.correlation;
                                                                                                            integer_lags[accepted_frames] = (float)aligned_frame.integer_lag;
                                                                                                            fractional_lags[accepted_frames] = aligned_frame.fractional_lag;
                                                                                                            total_lags[accepted_frames] = aligned_frame.total_lag;
                                                                                                            sum_correlation += (double)aligned_frame.correlation;
                                                                                                            if (calibration_channel < 0)             calibration_channel = aligned_frame.selected_channel;
                                                                                                            if (calibration_pending_frame_copy(                 &accepted_candidates[candidate_count],                 &aligned_frame, frame, reference_generation,                 uploaded_reference_length,                 uploaded_reference_format) != 0) {
                                                                                                                memset(&accepted_candidates[candidate_count], 0,                    sizeof(accepted_candidates[candidate_count]));
                                                                                                            }
                                                                                                            else {
                                                                                                                ++candidate_count;
                                                                                                            }
                                                                                                            ++accepted_frames;
                                                                                                            if (frame < frame_count) usleep(ADC_TIMING_INTERFRAME_DELAY_US);
                                                                                                        }
                                                                                                        if (accepted_frames > 0U) {
                                                                                                            minimum_correlation = correlations[0];
                                                                                                            for (uint32_t i = 1U;
                                                                                                            i < accepted_frames;
                                                                                                            ++i)             if (correlations[i] < minimum_correlation)                 minimum_correlation = correlations[i];
                                                                                                            median_total_lag = median_float(total_lags, accepted_frames);
                                                                                                        }
                                                                                                        if (!compact) {
                                                                                                            xil_printf("\r\n========== Timing Alignment Summary ==========\r\n");
                                                                                                            xil_printf("Requested frames    : %lu\r\n",                    (unsigned long)frame_count);
                                                                                                            xil_printf("Accepted frames     : %lu\r\n",                    (unsigned long)accepted_frames);
                                                                                                            xil_printf("Rejected frames     : %lu\r\n",                    (unsigned long)(frame_count - accepted_frames));
                                                                                                            xil_printf("Calibration channel : %s\r\n",                    calibration_channel == 0 ? "Channel A" :                    (calibration_channel == 1 ? "Channel B" : "none"));
                                                                                                            if (accepted_frames > 0U) {
                                                                                                                xil_printf("\r\n");
                                                                                                                print_double_value("Mean correlation",                                sum_correlation / (double)accepted_frames, "");
                                                                                                                print_float_value("Minimum correlation",                               minimum_correlation, "");
                                                                                                                print_float_value("Median lag",                               median_float(integer_lags, accepted_frames),                               " samples");
                                                                                                                print_float_value("Median frac. lag",                               median_float(fractional_lags, accepted_frames),                               " samples");
                                                                                                            }
                                                                                                        }
                                                                                                        {
                                                                                                            const float acceptance_rate =             (float)accepted_frames / (float)frame_count;
                                                                                                            const uint32_t required =             frame_count < CAL_TIMING_MIN_ACCEPTED_FRAMES ?             frame_count : CAL_TIMING_MIN_ACCEPTED_FRAMES;
                                                                                                            alignment_pass =             (accepted_frames >= required) &&             (acceptance_rate >= CAL_TIMING_MIN_ACCEPTANCE_RATE);
                                                                                                            if (alignment_pass && (candidate_count >= required) &&             (candidate_count == accepted_frames) &&             (calibration_select_representative_frame(                 accepted_candidates, candidate_count,                 &selected_candidate, &selection_medians) == 0)) {
                                                                                                                g_stored_offset_reference =                 accepted_candidates[selected_candidate];
                                                                                                                g_stored_offset_reference.valid = true;
                                                                                                                g_stored_offset_reference.consumed = false;
                                                                                                                representative_selected = 1;
                                                                                                            }
                                                                                                            else {
                                                                                                                calibration_pending_frame_invalidate();
                                                                                                            }
                                                                                                            if (!compact)             xil_printf("\r\nAlignment status    : %s\r\n",                        alignment_pass ? "PASS" : "FAIL");
                                                                                                        }
                                                                                                        if (!compact) {
                                                                                                            if (representative_selected && g_stored_offset_reference.valid) {
                                                                                                                xil_printf("Stored offset reference: Frame %lu\r\n",                        (unsigned long)                            g_stored_offset_reference.retained_frame_number);
                                                                                                                xil_printf("Selection reason    : Closest to median calibration metrics\r\n");
                                                                                                                calibration_print_timing_diagnostics_compact(                 &g_stored_offset_reference.timing_diagnostics);
                                                                                                            }
                                                                                                            else {
                                                                                                                xil_printf("Stored offset reference: none\r\n");
                                                                                                            }
                                                                                                            xil_printf("==============================================\r\n");
                                                                                                        }
                                                                                                        else {
                                                                                                            const bool timing_pass = representative_selected && alignment_pass &&             g_stored_offset_reference.valid;
                                                                                                            xil_printf("Frames accepted         : %lu/%lu\r\n",                    (unsigned long)accepted_frames,                    (unsigned long)frame_count);
                                                                                                            xil_printf("Channel                 : %s\r\n",                    calibration_channel_name(calibration_channel));
                                                                                                            if (accepted_frames > 0U) {
                                                                                                                print_double_value("Mean correlation",                                sum_correlation / (double)accepted_frames, "");
                                                                                                                print_float_value("Minimum correlation",                               minimum_correlation, "");
                                                                                                                print_float_value("Median lag", median_total_lag, " samples");
                                                                                                            }
                                                                                                            if (representative_selected && g_stored_offset_reference.valid) {
                                                                                                                xil_printf("Canonical phase         : %s\r\n",                 g_stored_offset_reference.canonical_reference_phase == 0 ?                 "EVEN" : "ODD");
                                                                                                                xil_printf("Fixed window            : %lu ... %lu\r\n",                 (unsigned long)                     g_stored_offset_reference.calibration_window_start,                 (unsigned long)(                     g_stored_offset_reference.calibration_window_start +                     g_stored_offset_reference.calibration_window_length -                     1U));
                                                                                                                calibration_print_timing_diagnostics_compact(                 &g_stored_offset_reference.timing_diagnostics);
                                                                                                            }
                                                                                                            xil_printf("Status                  : %s\r\n",                    timing_pass ? "PASS" : "FAILED");
                                                                                                            if (!timing_pass)             xil_printf("Reason                  : %s\r\n",                 accepted_frames <                     (frame_count < CAL_TIMING_MIN_ACCEPTED_FRAMES ?                      frame_count : CAL_TIMING_MIN_ACCEPTED_FRAMES) ?                 "insufficient valid frames" :                 "representative timing reference was not selected");
                                                                                                        }
                                                                                                        g_quiet_calibration_capture = false;
                                                                                                        adc_sweep_active = 0;
                                                                                                        if (representative_selected && alignment_pass &&         g_stored_offset_reference.valid) {
                                                                                                            g_automatic_calibration.timing_pass = true;
                                                                                                            g_automatic_calibration.calibration_channel =             g_stored_offset_reference.selected_channel;
                                                                                                            g_automatic_calibration.canonical_reference_phase =             g_stored_offset_reference.canonical_reference_phase;
                                                                                                            g_automatic_calibration.fixed_window_start =             g_stored_offset_reference.calibration_window_start;
                                                                                                            g_automatic_calibration.fixed_window_length =             g_stored_offset_reference.calibration_window_length;
                                                                                                            g_automatic_calibration.expected_lag =             g_stored_offset_reference.integer_lag;
                                                                                                            g_automatic_calibration.timing_mean_correlation =             accepted_frames > 0U ?             (float)(sum_correlation / (double)accepted_frames) : 0.0f;
                                                                                                            return 0;
                                                                                                        }
                                                                                                        return -1;
                                                                                                    }
                                                                                                    static void handle_adc_timing_calibration_stage_cmd(uint32_t frame_count) {
                                                                                                        if (adc_sweep_active) {
                                                                                                            ERR("Another automatic ADC capture is already in progress.");
                                                                                                            return;
                                                                                                        }
                                                                                                        calibration_all_loops_reset();
                                                                                                        calibration_automatic_state_reset();
                                                                                                        g_automatic_calibration.stage = ADC_CAL_STAGE_TIMING;
                                                                                                        xil_printf("\r\nDevelopment command: timing/reference selection only.\r\n");
                                                                                                        if (adc_run_timing_calibration(frame_count) == 0) {
                                                                                                            xil_printf("Timing stage ready. Run 'adc -cal offset' to test offset only.\r\n");
                                                                                                        }
                                                                                                        else {
                                                                                                            g_automatic_calibration.stage = ADC_CAL_STAGE_FAILED;
                                                                                                            g_automatic_calibration.failed_stage = ADC_CAL_STAGE_TIMING;
                                                                                                            g_automatic_calibration.overall_result = ADC_CAL_RESULT_FAILED;
                                                                                                            xil_printf("Calibration status      : FAILED\r\n");
                                                                                                            xil_printf("Failed stage            : TIMING\r\n");
                                                                                                        }
                                                                                                    }
                                                                                                    static void calibration_automatic_fail(     adc_calibration_stage_t failed_stage, const char *reason) {
                                                                                                        g_automatic_calibration.active = false;
                                                                                                        g_automatic_calibration.valid = false;
                                                                                                        g_automatic_calibration.output_valid = false;
                                                                                                        g_automatic_calibration.stage = ADC_CAL_STAGE_FAILED;
                                                                                                        g_automatic_calibration.failed_stage = failed_stage;
                                                                                                        g_automatic_calibration.overall_result = ADC_CAL_RESULT_FAILED;
                                                                                                        g_automatic_calibration.failure_reason = reason;
                                                                                                        calibration_print_stage_header(4U, "Performance Evaluation");
                                                                                                        xil_printf("Status                  : NOT RUN\r\n");
                                                                                                        xil_printf("Reason                  : no usable calibrated output\r\n");
                                                                                                        calibration_automatic_print_summary();
                                                                                                    }
                                                                                                    void handle_adc_calibration_cmd(uint32_t frame_count) {
                                                                                                        calibration_offset_loop_state_t *offset_state;
                                                                                                        calibration_gain_loop_state_t *gain_state;
                                                                                                        float final_normalized_gain;
                                                                                                        if (adc_sweep_active) {
                                                                                                            ERR("Another automatic ADC capture is already in progress.");
                                                                                                            return;
                                                                                                        }
                                                                                                        calibration_all_loops_reset();
                                                                                                        calibration_automatic_state_reset();
                                                                                                        g_automatic_calibration.active = true;
                                                                                                        g_automatic_calibration.stage = ADC_CAL_STAGE_TIMING;
                                                                                                        xil_printf("\r\n=========================================\r\n");
                                                                                                        xil_printf("       ADC AUTOMATIC CALIBRATION\r\n");
                                                                                                        xil_printf("=========================================\r\n");
                                                                                                        if (!reference_buffer_is_ready() || reference_buffer_length() == 0U) {
                                                                                                            xil_printf("ADC calibration cannot start.\r\n");
                                                                                                            xil_printf("Upload DAC reference first.\r\n");
                                                                                                            calibration_pending_frame_invalidate();
                                                                                                            calibration_automatic_fail(             ADC_CAL_STAGE_TIMING, "uploaded DAC reference is unavailable");
                                                                                                            return;
                                                                                                        }
                                                                                                        if (RxBufferPtr == NULL) {
                                                                                                            calibration_pending_frame_invalidate();
                                                                                                            calibration_automatic_fail(             ADC_CAL_STAGE_TIMING, "DMA receive buffer is unavailable");
                                                                                                            return;
                                                                                                        }
                                                                                                        print_adc_calibration_rate_summary();
                                                                                                        calibration_print_stage_header(1U, "Timing Alignment");
                                                                                                        if (adc_run_timing_calibration(frame_count) != 0 ||         !g_automatic_calibration.timing_pass ||         !g_stored_offset_reference.valid) {
                                                                                                            calibration_pending_frame_invalidate();
                                                                                                            calibration_automatic_fail(             ADC_CAL_STAGE_TIMING, "timing alignment did not pass");
                                                                                                            return;
                                                                                                        }
                                                                                                        g_automatic_calibration.stage = ADC_CAL_STAGE_OFFSET;
                                                                                                        calibration_print_stage_header(2U, "Offset Calibration");
                                                                                                        handle_adc_offset_calibration_loop_cmd();
                                                                                                        offset_state = calibration_offset_loop_state();
                                                                                                        g_automatic_calibration.offset_correction =         offset_state->offset_correction;
                                                                                                        g_automatic_calibration.offset_verification_error =         offset_state->verification_residual;
                                                                                                        g_automatic_calibration.offset_controller_converged =         offset_state->controller_converged != 0U;
                                                                                                        g_automatic_calibration.offset_verification_status =         offset_state->verification_status;
                                                                                                        g_automatic_calibration.offset_result = offset_state->stage_result;
                                                                                                        if ((offset_state->stage_result != CALIBRATION_OFFSET_RESULT_CONVERGED &&          offset_state->stage_result !=              CALIBRATION_OFFSET_RESULT_PROVISIONAL) ||         !offset_state->controller_converged ||         !offset_state->verification_valid ||         (offset_state->verification_status != CAL_OFFSET_VERIFICATION_PASS &&          offset_state->verification_status !=              CAL_OFFSET_VERIFICATION_MARGINAL) ||         !calibration_pending_frame_is_compatible(NULL) ||         !g_pending_calibration_frame.valid ||         g_pending_calibration_frame.consumed ||         g_pending_calibration_frame.source_offset_result !=             offset_state->stage_result) {
                                                                                                            calibration_gain_input_frame_invalidate();
                                                                                                            calibration_automatic_fail(             ADC_CAL_STAGE_OFFSET,             "offset result is unusable");
                                                                                                            return;
                                                                                                        }
                                                                                                        g_automatic_calibration.offset_pass = true;
                                                                                                        if (ADC_CAL_VERBOSE_DEBUG) {
                                                                                                            xil_printf("Offset:\r\n");
                                                                                                            print_float_value("  Applied correction",                           offset_state->offset_correction, " codes");
                                                                                                            print_float_value("  Verification residual",                           offset_state->verification_residual, " codes");
                                                                                                            xil_printf("  Verification status   : %s\r\n",             calibration_offset_verification_name(                 offset_state->verification_status));
                                                                                                            xil_printf("  Status                : %s\r\n",             calibration_offset_result_name(offset_state->stage_result));
                                                                                                        }
                                                                                                        if (offset_state->stage_result ==             CALIBRATION_OFFSET_RESULT_PROVISIONAL) {
                                                                                                            xil_printf("\r\nWARNING: Controller converged successfully.\r\n");
                                                                                                            xil_printf("Verification exceeded the preferred limit but remains usable.\r\n");
                                                                                                            xil_printf("Continuing to gain calibration with provisional offset.\r\n");
                                                                                                            print_float_value("Fixed offset correction",                           offset_state->offset_correction, " codes");
                                                                                                        }
                                                                                                        g_automatic_calibration.stage = ADC_CAL_STAGE_GAIN;
                                                                                                        calibration_print_stage_header(3U, "Gain Calibration");
                                                                                                        handle_adc_gain_calibration_loop_cmd();
                                                                                                        gain_state = calibration_gain_loop_state();
                                                                                                        g_automatic_calibration.gain_correction = gain_state->gain_correction;
                                                                                                        g_automatic_calibration.nominal_system_gain =         gain_state->nominal_system_gain;
                                                                                                        g_automatic_calibration.stage = ADC_CAL_STAGE_VERIFY;
                                                                                                        final_normalized_gain =         gain_state->nominal_system_gain > FLT_EPSILON ?         g_pending_calibration_frame.metrics.measured_gain /             gain_state->nominal_system_gain : NAN;
                                                                                                        if (g_pending_calibration_frame.valid &&         isfinite(final_normalized_gain)) {
                                                                                                            g_automatic_calibration.final_normalized_gain =             final_normalized_gain;
                                                                                                            g_automatic_calibration.gain_verification_error =             final_normalized_gain - 1.0f;
                                                                                                        }
                                                                                                        else if (isfinite(gain_state->latest_fitted_gain)) {
                                                                                                            g_automatic_calibration.final_normalized_gain =             gain_state->latest_fitted_gain;
                                                                                                            g_automatic_calibration.gain_verification_error =             gain_state->latest_gain_error;
                                                                                                        }
                                                                                                        if (gain_state->final_status != CALIBRATION_GAIN_LOOP_PASS ||         !gain_state->nominal_system_gain_valid ||         !isfinite(final_normalized_gain) ||         !calibration_pending_frame_is_compatible(NULL) ||         !g_pending_calibration_frame.valid ||         g_pending_calibration_frame.consumed ||         g_pending_calibration_frame.analysis_sample_count !=             CAL_FIXED_WINDOW_LENGTH ||         g_pending_calibration_frame.selected_channel !=             g_stored_offset_reference.selected_channel ||         g_pending_calibration_frame.canonical_reference_phase !=             g_stored_offset_reference.canonical_reference_phase ||         g_pending_calibration_frame.calibration_window_start !=             g_stored_offset_reference.calibration_window_start ||         g_pending_calibration_frame.calibration_window_length !=             g_stored_offset_reference.calibration_window_length) {
                                                                                                            calibration_gain_input_frame_invalidate();
                                                                                                            calibration_automatic_fail(             ADC_CAL_STAGE_GAIN,             "gain did not converge or publish a verified output");
                                                                                                            return;
                                                                                                        }
                                                                                                        g_automatic_calibration.gain_pass = true;
                                                                                                        g_automatic_calibration.gain_verification_pass = true;
                                                                                                        g_automatic_calibration.final_normalized_gain = final_normalized_gain;
                                                                                                        g_automatic_calibration.gain_verification_error =         final_normalized_gain - 1.0f;
                                                                                                        g_automatic_calibration.final_output = g_pending_calibration_frame;
                                                                                                        g_automatic_calibration.output_valid = true;
                                                                                                        g_automatic_calibration.valid = true;
                                                                                                        if (ADC_CAL_VERBOSE_DEBUG) {
                                                                                                            xil_printf("Gain:\r\n");
                                                                                                            print_float_value("  Nominal system gain",                           gain_state->nominal_system_gain, "");
                                                                                                            print_float_value("  Normalized ADC gain", final_normalized_gain, "");
                                                                                                            print_float_value("  Applied correction",                           gain_state->gain_correction, "");
                                                                                                            print_float_value("  Verification error",                           g_automatic_calibration.gain_verification_error, "");
                                                                                                            xil_printf("  Verification status   : PASS\r\n");
                                                                                                            xil_printf("  Status                : CONVERGED\r\n");
                                                                                                        }
                                                                                                        g_automatic_calibration.stage = ADC_CAL_STAGE_PERFORMANCE;
                                                                                                        calibration_print_stage_header(4U, "Performance Evaluation");
                                                                                                        (void)adc_evaluate_performance_batch(         &g_automatic_calibration.final_output,         gain_state->gain_correction,         gain_state->fixed_offset_correction,         gain_state->nominal_system_gain,         g_stored_offset_reference.reference_frequency_hz,         offset_state->verification_residual,         offset_state->verification_standard_error,         gain_state->post_gain_residual_valid ?             gain_state->post_gain_residual : NAN,         gain_state->post_gain_residual_valid ?             gain_state->post_gain_residual_standard_error : NAN,         &g_automatic_calibration.performance);
                                                                                                        adc_print_performance_result(&g_automatic_calibration.performance);
                                                                                                        g_automatic_calibration.active = false;
                                                                                                        g_automatic_calibration.stage = ADC_CAL_STAGE_COMPLETE;
                                                                                                        g_automatic_calibration.overall_result =         offset_state->stage_result ==             CALIBRATION_OFFSET_RESULT_PROVISIONAL ?         ADC_CAL_RESULT_PROVISIONAL : ADC_CAL_RESULT_PASS;
                                                                                                        calibration_automatic_print_summary();
                                                                                                    }
