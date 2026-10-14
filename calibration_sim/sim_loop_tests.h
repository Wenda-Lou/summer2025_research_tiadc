#ifndef SIM_LOOP_TESTS_H
#define SIM_LOOP_TESTS_H

#include "sim_assert.h"

/* Tests of calibration_loop_c/adc_calibration_loop.c (the C port of the host
 * calibration_loop package).  Native tests need no input files. */
void sim_loop_tests_run_native(sim_assert_context_t *ctx, FILE *summary);

/* Replays a golden file written by calibration_loop_c/export_golden.py and
 * compares the C loop with the Python loop field by field. */
void sim_loop_tests_run_golden(sim_assert_context_t *ctx, FILE *summary, const char *path);

#endif /* SIM_LOOP_TESTS_H */
