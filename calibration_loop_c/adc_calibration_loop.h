#ifndef ADC_CALIBRATION_LOOP_H
#define ADC_CALIBRATION_LOOP_H

/*
 * Impulse-dither closed calibration loop, ported from the host package
 * calibration_loop/ (dither.py, estimator.py, loop.py).  Hardware-independent
 * (C11, libm only, no Xilinx headers) so it can later be added to the firmware
 * unchanged; today it is built and tested by calibration_sim.
 *
 * Scope of this port is the tone route, i.e. the host defaults:
 *   cancel_signal = true, gain_observable = "tone" (falls back to "dither"),
 *   skew_observable = "phase".
 * The tone-free routes (dither_raw.py) and the SNDR/SFDR metrics are not ported.
 *
 * One iteration, exactly as CalibrationLoop.step():
 *   raw DMA bytes -> de-frame -> apply correction -> estimate residual errors
 *   -> acceptance filter -> block-LMS update of offset/gain
 *   -> skew: per-frame ps integrator (model) or batch code decision (hardware)
 *
 * Correction convention is the host one:  y_cal = (y - offset) * gain_corr.
 * It is NOT the butils_calibration.c convention; do not mix them.
 *
 * Memory: no heap.  adc_cal_loop_t carries every buffer and is large (~570 KB on aarch64),
 * so give it static storage.  Not reentrant per instance.
 */

#include <stddef.h>
#include <stdint.h>

#define ADC_CAL_LOOP_MAX_EVENTS          512
#define ADC_CAL_LOOP_MAX_LOOP_SAMPLES    16384
#define ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES 4096
#define ADC_CAL_LOOP_MAX_WORDS           (2 * ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES + 8)
#define ADC_CAL_LOOP_MAX_WINDOW          160
#define ADC_CAL_LOOP_MAX_BATCH           256
#define ADC_CAL_LOOP_DAC_FULL_SCALE      32767.0

/* ------------------------------------------------------------------------ */
/* Waveform description (DitherConfig).  Must describe the vector on the DPG. */
/* ------------------------------------------------------------------------ */

typedef struct {
    double fs_dac;             /* DAC rate [Hz] */
    int n_dac_points;          /* samples in one DPG vector */
    int adc_ratio;             /* fs_dac / fs_adc, integer */
    int sig_cycles;            /* tone cycles per vector */
    double amp_dbfs;           /* tone amplitude [dBFS] */
    int dither_period_dac;
    int dither_position_dac;
    int dither_edge_dac;
    int dither_top_dac;
    double dither_scale_lsb;
    int n_events;              /* n_dac_points / dither_period_dac */
    int8_t polarity[ADC_CAL_LOOP_MAX_EVENTS];
    /* +-1 per event.  polarity_sequence() shuffles with numpy's RandomState, so it
     * is carried as data: copy the "polarity" list of the waveform's JSON. */
} adc_cal_loop_config_t;

/* Host defaults (DitherConfig()), including the seed-20260725 polarity table. */
void adc_cal_loop_default_config(adc_cal_loop_config_t *cfg);
/* 0 when valid, negative with a reason otherwise (mirrors DitherConfig.validate). */
int adc_cal_loop_config_validate(const adc_cal_loop_config_t *cfg, const char **reason);

double adc_cal_loop_fs_adc(const adc_cal_loop_config_t *cfg);
int adc_cal_loop_n_adc_period(const adc_cal_loop_config_t *cfg);
double adc_cal_loop_f_sig(const adc_cal_loop_config_t *cfg);

double adc_cal_loop_pulse(double t, double edge_r, double top_w);
double adc_cal_loop_pulse_derivative(double t, double edge_r, double top_w);

/* ------------------------------------------------------------------------ */
/* Tone fit                                                                   */
/* ------------------------------------------------------------------------ */

typedef struct {
    double f0;        /* cycles per ADC sample actually used */
    double a;         /* y ~ a*cos(w n) + b*sin(w n) + dc */
    double b;
    double dc;
    double amplitude;
    double phase;     /* atan2(-b, a) */
    double sse;
} adc_cal_loop_tone_fit_t;

int adc_cal_loop_fit_tone(const double *y, size_t n, double f0, int refine,
                          adc_cal_loop_tone_fit_t *fit);

/* ------------------------------------------------------------------------ */
/* Estimates                                                                  */
/* ------------------------------------------------------------------------ */

typedef struct {
    double offset_codes;
    double offset_record_codes;
    double gain_codes;
    double skew_samples;
    double phase_rad;
    double centroid_samples;
    double tone_f0;
    double skew_ps;
    double tone_amplitude;
    int n_events_used;
    int align_n0;
    double residual_rms;
    double gain_sign_fit;
    double hat_gain;  /* gain behind the dither estimate the last pass subtracted (NaN: none) */
} adc_cal_loop_channel_estimate_t;

typedef enum {
    ADC_CAL_LOOP_SKEW_SOURCE_NONE = 0,
    ADC_CAL_LOOP_SKEW_SOURCE_PHASE,
    ADC_CAL_LOOP_SKEW_SOURCE_PHASE_BRANCH_PRIOR,
    ADC_CAL_LOOP_SKEW_SOURCE_PHASE_BRANCH_CENTROID,
    ADC_CAL_LOOP_SKEW_SOURCE_PROFILE_FALLBACK,
    ADC_CAL_LOOP_SKEW_SOURCE_OBSERVABLES_DISAGREE
} adc_cal_loop_skew_source_t;

typedef struct {
    adc_cal_loop_channel_estimate_t ch_a;
    adc_cal_loop_channel_estimate_t ch_b;
    double gain_ratio;
    double tone_ratio;
    double offset_mismatch_codes;
    double skew_mismatch_samples;
    double skew_mismatch_ps;
    double skew_phase_ps;
    double skew_diff_route_ps;
    double skew_centroid_ps;
    adc_cal_loop_skew_source_t skew_source;
    double skew_used_ps;
    int rotation;
} adc_cal_loop_block_estimate_t;

typedef struct {
    size_t n_samples;     /* per channel */
    int rotation;
    int lagged;           /* "swapped" in the host log */
    double corr;          /* pre-normalisation A/B correlation (negative = inverted branch) */
    int n0;
    double align_margin;
    double sign_a;
    double sign_b;
    int n0_natural;          /* the alignment's own argmax, before any override */
    double n0_score_ratio;   /* |score[n0]| / |score[n0_natural]| on the deciding channel */
} adc_cal_loop_prep_t;

/* ------------------------------------------------------------------------ */
/* Controller state (CalibrationState) and options (LoopOptions)             */
/* ------------------------------------------------------------------------ */

typedef struct {
    double offset_a;
    double offset_b;
    double gain_corr_a;
    double gain_corr_b;
    double skew_target_ps;
    double skew_cmd_ps;
    double mu_offset;
    double mu_gain;
    double mu_skew;
    unsigned iteration;
} adc_cal_loop_state_t;

typedef enum {
    ADC_CAL_LOOP_GAIN_TONE = 0,
    ADC_CAL_LOOP_GAIN_DITHER
} adc_cal_loop_gain_observable_t;

typedef struct {
    int cancel_signal;
    int close_skew_loop;
    double min_align_margin;
    double max_skew_samples;
    double max_gain_deviation;
    adc_cal_loop_gain_observable_t gain_observable;
    int skew_batch_frames;
    double skew_direction_tol_ps;
    double skew_min_yield;
    unsigned max_consecutive_rejects;
} adc_cal_loop_options_t;

void adc_cal_loop_default_state(adc_cal_loop_state_t *state);
void adc_cal_loop_default_options(adc_cal_loop_options_t *options);

/* ------------------------------------------------------------------------ */
/* Skew actuator.                                                             */
/*  request_steps != NULL: hardware branch, batch decision in control codes   */
/*                         through an ACK-checked transaction.                */
/*  request_steps == NULL: model branch, per-frame ps integrator pushed out   */
/*                         through command_skew_ps (may be NULL).             */
/* ------------------------------------------------------------------------ */

typedef struct {
    int ok;
    int code_after;
} adc_cal_loop_skew_ack_t;

typedef struct {
    void *ctx;
    int (*request_steps)(void *ctx, int delta, adc_cal_loop_skew_ack_t *ack);
    void (*command_skew_ps)(void *ctx, double delay_ps);
    double step_ps;       /* SkewActuator.HOST_STEP_PS */
    double deadband_ps;   /* SkewActuator.deadband_ps */
    int code;             /* current control code, updated from ACKs only */
    int code_min;
    int code_max;
} adc_cal_loop_actuator_t;

void adc_cal_loop_default_actuator(adc_cal_loop_actuator_t *actuator);
int adc_cal_loop_error_to_steps(const adc_cal_loop_actuator_t *actuator, double error_ps);

typedef enum {
    ADC_CAL_LOOP_SKEW_ACTION_NONE = 0,
    ADC_CAL_LOOP_SKEW_ACTION_MEASURE_ONLY,
    ADC_CAL_LOOP_SKEW_ACTION_ABORTED,
    ADC_CAL_LOOP_SKEW_ACTION_NO_USABLE_FRAMES,
    ADC_CAL_LOOP_SKEW_ACTION_LOW_YIELD,
    ADC_CAL_LOOP_SKEW_ACTION_ABORT_DIRECTION,
    ADC_CAL_LOOP_SKEW_ACTION_INSIDE_DEADBAND,
    ADC_CAL_LOOP_SKEW_ACTION_AT_RANGE_LIMIT,
    ADC_CAL_LOOP_SKEW_ACTION_MOVE_OK,
    ADC_CAL_LOOP_SKEW_ACTION_MOVE_FAILED
} adc_cal_loop_skew_action_t;

typedef enum {
    ADC_CAL_LOOP_REJECT_NONE = 0,
    ADC_CAL_LOOP_REJECT_ALIGN_MARGIN,
    ADC_CAL_LOOP_REJECT_TOO_FEW_EVENTS,
    ADC_CAL_LOOP_REJECT_A_NOT_FINITE,
    ADC_CAL_LOOP_REJECT_B_NOT_FINITE,
    ADC_CAL_LOOP_REJECT_SKEW_RANGE,
    ADC_CAL_LOOP_REJECT_GAIN_RANGE,
    ADC_CAL_LOOP_REJECT_CAPTURE
} adc_cal_loop_reject_t;

/* One log row (the fields of loop.py's row that this port produces). */
typedef struct {
    unsigned iteration;
    adc_cal_loop_reject_t rejected;
    adc_cal_loop_prep_t prep;
    double polarity;
    adc_cal_loop_block_estimate_t est;
    const char *gain_source;
    double offset_error_a;
    double offset_error_b;
    double gain_error;
    double skew_error_ps;
    double offset_a_state;
    double offset_b_state;
    double gain_corr_a;
    double gain_corr_b;
    double skew_cmd_ps;
    int skew_batch_n;
    int skew_batch_used;
    double skew_batch_mean_ps;
    double skew_batch_se_ps;
    double skew_batch_error_ps;
    adc_cal_loop_skew_action_t skew_action;
    int skew_step_codes;
    int skew_code;
} adc_cal_loop_row_t;

typedef struct {
    double phase_ps;
    double diff_route_ps;
    adc_cal_loop_skew_source_t source;
    double tone;
    double resid;
    double margin;
} adc_cal_loop_skew_frame_t;

typedef struct {
    int n_total;
    int n_used;
    double mean_ps;
    double se_ps;
    double route_center_ps;
    double route_bound_ps;
} adc_cal_loop_skew_batch_t;

/* skew_batch() with the host defaults (tone_tol 0.10, resid 0.5..2.0, margin 6.0,
 * relative route-agreement bound 6 robust sigma, 1.5 ps floor). */
void adc_cal_loop_skew_batch(const adc_cal_loop_skew_frame_t *frames, int n_frames,
                             adc_cal_loop_skew_batch_t *out);

typedef struct {
    adc_cal_loop_config_t cfg;
    adc_cal_loop_options_t opt;
    adc_cal_loop_state_t state;
    adc_cal_loop_actuator_t act;

    double polarity;            /* session anchor, 0 until measured */
    int n0_override;            /* test hook: >= 0 replaces prepare's n0 (golden replay) */
    adc_cal_loop_skew_frame_t skew_frames[ADC_CAL_LOOP_MAX_BATCH];
    int skew_frame_count;
    int skew_have_prev;
    double skew_prev_error;
    double skew_expected_shift;
    int skew_abort;             /* latched */

    unsigned captures;
    unsigned qualified;
    unsigned rejected;

    /* derived from cfg by adc_cal_loop_init */
    int n_loop;
    double loop_ref[ADC_CAL_LOOP_MAX_LOOP_SAMPLES];   /* dither_only_loop(cfg) */
    int ref_nz_index[ADC_CAL_LOOP_MAX_LOOP_SAMPLES];
    int ref_nz_count;

    /* work buffers */
    int32_t words[ADC_CAL_LOOP_MAX_WORDS];
    double ch_a[ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES];
    double ch_b[ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES];
    double cal_a[ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES];
    double cal_b[ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES];
    double work_y[ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES];
    double work_r[ADC_CAL_LOOP_MAX_CHANNEL_SAMPLES];
    double score[ADC_CAL_LOOP_MAX_LOOP_SAMPLES];
} adc_cal_loop_t;

/* Validates cfg and precomputes the alignment template.  opt/state/act may be
 * NULL for defaults.  Returns 0 on success. */
int adc_cal_loop_init(adc_cal_loop_t *loop, const adc_cal_loop_config_t *cfg,
                      const adc_cal_loop_options_t *opt, const adc_cal_loop_state_t *state,
                      const adc_cal_loop_actuator_t *act);

/* prepare_capture(): raw little-endian DMA bytes -> loop->ch_a / loop->ch_b. */
int adc_cal_loop_prepare_capture(adc_cal_loop_t *loop, const uint8_t *raw, size_t n_bytes,
                                 adc_cal_loop_prep_t *prep);

/* estimate_block() on two corrected channels. */
int adc_cal_loop_estimate_block(adc_cal_loop_t *loop, const double *ch_a, const double *ch_b,
                                size_t n, int n0, double skew_prior_samples,
                                double polarity_sign, int pin_polarity,
                                adc_cal_loop_block_estimate_t *est);

/* CalibrationState.update(); returns the gain observable used ("tone"/"dither"/"none"). */
const char *adc_cal_loop_state_update(adc_cal_loop_state_t *state,
                                      const adc_cal_loop_block_estimate_t *est,
                                      int integrate_skew,
                                      adc_cal_loop_gain_observable_t gain_observable,
                                      double skew_ps, adc_cal_loop_row_t *row);

/* One iteration on one captured buffer.  Returns 0 when a row was produced
 * (check row->rejected), negative when the buffer could not be de-framed. */
int adc_cal_loop_step(adc_cal_loop_t *loop, const uint8_t *raw, size_t n_bytes,
                      adc_cal_loop_row_t *row);

/* Capture callback for adc_cal_loop_run: fill buf, return bytes or <= 0 on failure. */
typedef long (*adc_cal_loop_capture_fn)(void *ctx, uint8_t *buf, size_t capacity);
typedef void (*adc_cal_loop_row_fn)(void *ctx, const adc_cal_loop_row_t *row);

/* CalibrationLoop.run(): collect n_qualified accepted samples.  Stops early after
 * 3 consecutive capture failures or opt.max_consecutive_rejects rejects in a row.
 * Returns the number of qualified samples collected. */
unsigned adc_cal_loop_run(adc_cal_loop_t *loop, unsigned n_qualified,
                          adc_cal_loop_capture_fn capture, void *capture_ctx,
                          adc_cal_loop_row_fn on_row, void *row_ctx,
                          uint8_t *capture_buf, size_t capture_capacity);

const char *adc_cal_loop_reject_name(adc_cal_loop_reject_t reason);
const char *adc_cal_loop_skew_action_name(adc_cal_loop_skew_action_t action);
const char *adc_cal_loop_skew_source_name(adc_cal_loop_skew_source_t source);

#endif /* ADC_CALIBRATION_LOOP_H */
