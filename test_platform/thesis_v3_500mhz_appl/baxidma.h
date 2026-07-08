#ifndef BAXIDMA_H
#define BAXIDMA_H

#include "xaxidma.h"
#include "xil_types.h"

#define DMA_CMD_BUF_SIZE   4095
#define DMA_DEVICE_ID      0

XAxiDma_Config* dma_init(XAxiDma* dma);


#endif // BAXIDMA_H
