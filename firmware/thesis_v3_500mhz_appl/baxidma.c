#include "baxidma.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include <xaxidma.h>

/* We keep a single static instance under the hood */
extern XAxiDma dma_inst;

/* Initialize AXI DMA in simple (non-SG) mode */
XAxiDma_Config* dma_init(XAxiDma* dma)
{
    XAxiDma_Config* config = XAxiDma_LookupConfig(DMA_DEVICE_ID);
    if (!config) {
        xil_printf("DMA config lookup failed!\r\n");
        return NULL;
    }

    if (XAxiDma_CfgInitialize(dma, config) != XST_SUCCESS) {
        xil_printf("DMA initialization failed!\r\n");
        return NULL;
    }

    if (XAxiDma_HasSg(&dma_inst)) {
		xil_printf("Device configured as SG mode \r\n");
		return NULL;
	}

    // Disable Intr, we will use polled mode.
    XAxiDma_IntrDisable(&dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

    xil_printf("DMA initialized successfully.\r\n");
    return config;
}

