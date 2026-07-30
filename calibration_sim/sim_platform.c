#include "sim_platform.h"

#include "sim_dma.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static sim_platform_state_t *g_active_platform;

void sim_platform_init(
    sim_platform_state_t *platform,
    sim_signal_state_t *signal,
    uint8_t *dma_buffer,
    size_t dma_buffer_size)
{
    if (platform == NULL) return;
    memset(platform, 0, sizeof(*platform));
    platform->signal = signal;
    platform->dma_buffer = dma_buffer;
    platform->dma_buffer_size = dma_buffer_size;
}

void sim_platform_set_active(sim_platform_state_t *platform)
{
    g_active_platform = platform;
}

sim_platform_state_t *sim_platform_active(void)
{
    return g_active_platform;
}

int sim_platform_capture_dma_frame(uint8_t *destination, size_t destination_size)
{
    static int16_t channel_a[SIM_ADC_CHANNEL_SAMPLES];
    static int16_t channel_b[SIM_ADC_CHANNEL_SAMPLES];
    const char *reason = NULL;

    if (g_active_platform == NULL || g_active_platform->signal == NULL ||
        destination == NULL || destination_size < SIM_DMA_BYTES) {
        return -1;
    }

    if (sim_signal_generate_frame(
            g_active_platform->signal,
            channel_a,
            channel_b,
            SIM_ADC_CHANNEL_SAMPLES,
            &reason) != 0) {
        (void)reason;
        return -2;
    }

    return sim_dma_encode_channels(
        channel_a,
        channel_b,
        SIM_ADC_CHANNEL_SAMPLES,
        destination,
        destination_size);
}

void sim_platform_sleep_us(uint32_t microseconds)
{
    (void)microseconds;
}

void sim_platform_log_text(const char *text)
{
    if (text != NULL) fputs(text, stdout);
}

void sim_platform_mark_performance_available(bool available)
{
    if (g_active_platform != NULL) {
        g_active_platform->performance_measurement_available = available;
    }
}

void calibration_pending_frame_invalidate(void)
{
    if (g_active_platform != NULL) {
        g_active_platform->performance_measurement_available = false;
        ++g_active_platform->pending_invalidations;
    }
}

void calibration_gain_input_frame_invalidate(void)
{
    if (g_active_platform != NULL) {
        g_active_platform->performance_measurement_available = false;
        ++g_active_platform->gain_input_invalidations;
    }
}
