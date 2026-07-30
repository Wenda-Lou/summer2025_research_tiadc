#ifndef SIM_PLATFORM_H
#define SIM_PLATFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "sim_signal.h"

typedef struct {
    sim_signal_state_t *signal;
    uint8_t *dma_buffer;
    size_t dma_buffer_size;
    bool performance_measurement_available;
    uint32_t pending_invalidations;
    uint32_t gain_input_invalidations;
} sim_platform_state_t;

void sim_platform_init(
    sim_platform_state_t *platform,
    sim_signal_state_t *signal,
    uint8_t *dma_buffer,
    size_t dma_buffer_size);
void sim_platform_set_active(sim_platform_state_t *platform);
sim_platform_state_t *sim_platform_active(void);
int sim_platform_capture_dma_frame(uint8_t *destination, size_t destination_size);
void sim_platform_sleep_us(uint32_t microseconds);
void sim_platform_log_text(const char *text);
void sim_platform_mark_performance_available(bool available);

#endif /* SIM_PLATFORM_H */
