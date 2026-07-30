#include "sim_tests.h"

#include "sim_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void print_help(const char *program)
{
    printf("Usage: %s [options]\n", program);
    printf("  --help\n");
    printf("  --list-scenarios\n");
    printf("  --scenario NAME\n");
    printf("  --run-pipeline NAME\n");
    printf("  --run-all-pipeline-scenarios\n");
    printf("  --run-unit-tests\n");
    printf("  --run-controller-tests\n");
    printf("  --run-all\n");
    printf("  --stress-seeds N\n");
    printf("  --seed N\n");
    printf("  --output-dir DIR\n");
}

int main(int argc, char **argv)
{
    sim_run_options_t options;
    sim_run_summary_t summary;
    bool list_scenarios = false;

    memset(&options, 0, sizeof(options));
    options.output_dir = SIM_OUTPUT_DIR_DEFAULT;
    options.seed = 1U;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            print_help(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--list-scenarios") == 0) {
            list_scenarios = true;
        } else if (strcmp(argv[i], "--scenario") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "--scenario requires a name\n");
                return 2;
            }
            options.scenario = argv[++i];
            options.run_scenarios = true;
        } else if (strcmp(argv[i], "--run-pipeline") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "--run-pipeline requires a scenario name\n");
                return 2;
            }
            options.pipeline_scenario = argv[++i];
            options.run_pipeline_scenarios = true;
        } else if (strcmp(argv[i], "--run-all-pipeline-scenarios") == 0) {
            options.run_pipeline_scenarios = true;
            options.pipeline_scenario = NULL;
        } else if (strcmp(argv[i], "--run-unit-tests") == 0) {
            options.run_unit_tests = true;
        } else if (strcmp(argv[i], "--run-controller-tests") == 0) {
            options.run_controller_tests = true;
        } else if (strcmp(argv[i], "--run-all") == 0) {
            options.run_unit_tests = true;
            options.run_controller_tests = true;
            options.run_scenarios = true;
            options.run_pipeline_scenarios = true;
            options.scenario = NULL;
            options.pipeline_scenario = NULL;
        } else if (strcmp(argv[i], "--stress-seeds") == 0) {
            char *endptr = NULL;
            unsigned long parsed;
            if (i + 1 >= argc) {
                fprintf(stderr, "--stress-seeds requires a count\n");
                return 2;
            }
            parsed = strtoul(argv[++i], &endptr, 0);
            if (endptr == argv[i] || *endptr != '\0') {
                fprintf(stderr, "Invalid stress seed count: %s\n", argv[i]);
                return 2;
            }
            options.stress_seeds = (uint32_t)parsed;
        } else if (strcmp(argv[i], "--seed") == 0) {
            char *endptr = NULL;
            unsigned long parsed;
            if (i + 1 >= argc) {
                fprintf(stderr, "--seed requires a value\n");
                return 2;
            }
            parsed = strtoul(argv[++i], &endptr, 0);
            if (endptr == argv[i] || *endptr != '\0') {
                fprintf(stderr, "Invalid seed: %s\n", argv[i]);
                return 2;
            }
            options.seed = (uint32_t)parsed;
        } else if (strcmp(argv[i], "--output-dir") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "--output-dir requires a path\n");
                return 2;
            }
            options.output_dir = argv[++i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_help(argv[0]);
            return 2;
        }
    }

    if (list_scenarios) {
        sim_tests_print_scenarios();
        if (!options.run_unit_tests && !options.run_controller_tests &&
            !options.run_scenarios && !options.run_pipeline_scenarios &&
            options.stress_seeds == 0U) {
            return 0;
        }
    }

    if (!options.run_unit_tests && !options.run_controller_tests &&
        !options.run_scenarios && !options.run_pipeline_scenarios &&
        options.stress_seeds == 0U) {
        options.run_unit_tests = true;
    }

    const int status = sim_tests_run(&options, &summary);
    printf("Tests passed      : %u\n", summary.tests_passed);
    printf("Tests failed      : %u\n", summary.tests_failed);
    printf("Scenarios passed  : %u\n", summary.scenarios_passed);
    printf("Scenarios failed  : %u\n", summary.scenarios_failed);
    printf("Pipeline passed   : %u\n", summary.pipeline_scenarios_passed);
    printf("Pipeline failed   : %u\n", summary.pipeline_scenarios_failed);
    printf("Stress passed     : %u\n", summary.stress_passed);
    printf("Stress failed     : %u\n", summary.stress_failed);
    printf("Output directory  : %s\n", options.output_dir);
    return status;
}
