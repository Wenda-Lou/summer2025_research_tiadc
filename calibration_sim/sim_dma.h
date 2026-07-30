#ifndef SIM_DMA_H
#define SIM_DMA_H

#include <stddef.h>
#include <stdint.h>

#include "sim_config.h"

int sim_dma_encode_channels(
    const int16_t *channel_a,
    const int16_t *channel_b,
    size_t sample_count,
    uint8_t *raw_bytes,
    size_t raw_byte_count);

#endif /* SIM_DMA_H */
