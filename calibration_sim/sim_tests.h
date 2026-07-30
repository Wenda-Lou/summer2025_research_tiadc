#ifndef SIM_TESTS_H
#define SIM_TESTS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    const char *output_dir;
    uint32_t seed;
    bool run_unit_tests;
    bool run_controller_tests;
    bool run_scenarios;
    bool run_pipeline_scenarios;
    const char *scenario;
    const char *pipeline_scenario;
    uint32_t stress_seeds;
} sim_run_options_t;

typedef struct {
    unsigned tests_passed;
    unsigned tests_failed;
    unsigned scenarios_passed;
    unsigned scenarios_failed;
    unsigned pipeline_scenarios_passed;
    unsigned pipeline_scenarios_failed;
    unsigned stress_passed;
    unsigned stress_failed;
} sim_run_summary_t;

void sim_tests_print_scenarios(void);
int sim_tests_run(const sim_run_options_t *options, sim_run_summary_t *summary);

#endif /* SIM_TESTS_H */
