#ifndef BAXIDMA_H
#define BAXIDMA_H

#include "xaxidma.h"
#include "xil_types.h"
#include <stdint.h>

#define DMA_CMD_BUF_SIZE   4095U
#define DMA_DEVICE_ID      0
#define ADC_CAPTURE_SAMPLE_SIZE 2U
#define ADC_CAPTURE_SAMPLES    (DMA_CMD_BUF_SIZE / ADC_CAPTURE_SAMPLE_SIZE)
#define ADC_CAPTURE_USED_BYTES (ADC_CAPTURE_SAMPLES * ADC_CAPTURE_SAMPLE_SIZE)
#define ADC_CAPTURE_PAD_BYTES  (DMA_CMD_BUF_SIZE - ADC_CAPTURE_USED_BYTES)

XAxiDma_Config* dma_init(XAxiDma* dma);


#endif // BAXIDMA_H
