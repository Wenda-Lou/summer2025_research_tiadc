#ifndef SIM_ASSERT_H
#define SIM_ASSERT_H

#include <math.h>
#include <stdio.h>

#define SIM_ASSERT_TRUE(ctx, condition) \
    sim_assert_true((ctx), (condition) != 0, #condition, __FILE__, __LINE__)

#define SIM_ASSERT_EQ_INT(ctx, actual, expected) \
    sim_assert_eq_int((ctx), (long)(actual), (long)(expected), \
                      #actual, #expected, __FILE__, __LINE__)

#define SIM_ASSERT_NEAR(ctx, actual, expected, tolerance) \
    sim_assert_near((ctx), (double)(actual), (double)(expected), \
                    (double)(tolerance), #actual, #expected, __FILE__, __LINE__)

typedef struct {
    unsigned passed;
    unsigned failed;
    FILE *summary;
    FILE *unit_csv;
} sim_assert_context_t;

void sim_assert_context_init(
    sim_assert_context_t *ctx,
    FILE *summary,
    FILE *unit_csv);
int sim_assert_true(
    sim_assert_context_t *ctx,
    int condition,
    const char *expression,
    const char *file,
    int line);
int sim_assert_eq_int(
    sim_assert_context_t *ctx,
    long actual,
    long expected,
    const char *actual_expr,
    const char *expected_expr,
    const char *file,
    int line);
int sim_assert_near(
    sim_assert_context_t *ctx,
    double actual,
    double expected,
    double tolerance,
    const char *actual_expr,
    const char *expected_expr,
    const char *file,
    int line);

#endif /* SIM_ASSERT_H */
