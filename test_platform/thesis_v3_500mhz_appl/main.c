#include "xuartps.h"
#include "xspips.h"
#include "xgpiops.h"
#include "sleep.h"
#include "xil_cache.h"
#include <xil_printf.h>

// My Custom Libraries
#include "peripherals.h"
#include "butils.h"
#include "bjesdlink.h"
#include "bjesdphy.h"
#include "baxidma.h"
#include "ethernet.h"

// AD9695 Libs
#include "ad9695_api.h"
#include "ad9695.h"
#include "ad9695_registers.h"

#define DDR_BASE_ADDR       XPAR_PSU_DDR_0_BASEADDRESS
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x01000000ULL)
#define RX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00300000ULL)



// DMA buffer
uint8_t *RxBufferPtr = (uint8_t *)RX_BUFFER_BASE;

// UART instance and configuration pointer
XUartPs uart_inst;
XUartPs_Config* uart_config;

// DMA instance and config pointer
XAxiDma dma_inst;
XAxiDma_Config* dma_config;

// UART instance and configuration pointer
XUartPs uart_inst;
XUartPs_Config* uart_config;

// SPI instance and configuration pointer
XSpiPs spi_inst;
XSpiPs_Config* spi_config;

// GPIO instance and configuration pointer
XGpioPs gpio_inst;
XGpioPs_Config* gpio_config;

uint8_t uart_send_flag = 0; //Send flag enabled by the uart
uint8_t* dma_rx_base_ptr = (uint8_t*) RX_BUFFER_BASE;

int main()
{
    // UART initialization
    uart_config = uart_init(&uart_inst);

    // SPI initialization
    spi_config = spi_init(&spi_inst);

    // GPIO initialization
    gpio_config = gpio_init(&gpio_inst);

    // DMA init
    dma_config = dma_init(&dma_inst);

    //lwIP init
    if(lwIP_UDP_init()){
        xil_printf("lwIP init fails\n");
    }

    // line command received from UART
    char uart_line [MAX_UART_LINE_LENGTH];

    // init parameters for AD9695 JESD204B link
    struct jesd_param_t jesd_param_init = {
        .jesd_L = 4,
        .jesd_F = 1,
        .jesd_M = 2,
        .jesd_HD = 0,
        .jesd_K = 32,
        .jesd_N = 16,
        .jesd_NP = 16,
        .jesd_CS = 0
    };

    // init parameters for AD9695
    struct ad9695_state ad9695_0_param = {
        .sample_clk_freq_khz = 1300000,
        .powerdown_pin_en = 0,
        .powerdown_mode = AD9695_POWERDOWN,
        .fc_ch = ad9695_FULL_BANDWIDTH_MODE,
        .test_mode_ch0 = AD9695_TESTMODE_OFF,
        .test_mode_ch1 = AD9695_TESTMODE_OFF,
        .jesd_param = &jesd_param_init,
        .jesd_subclass = 0,
        .force_cgs = 0,

        .num_of_half_clk_cycle_delay_ch0 = 0,
        .num_of_half_clk_cycle_delay_ch1 = 0,
        .clk_delay_mode_sel = AD9695_NO_CLOCK_DELAY,
    };

    struct jesdphy_pll_status pll_status = {
        .cpll_unlocked = 1,
        .qpll0_unlocked = 1,
        .qpll1_unlocked = 1,
        .rx_reset_in_prog = 1,
        .tx_reset_in_prog = 1
    };


    /* Enable the instruction cache. */
	Xil_ICacheEnable();
	/* Enable the data cache. */
	Xil_DCacheEnable();

    // init AD9695 (note that CGS force is embedeed in the setup function)
    ad9695_initialize(&ad9695_0_param);

    // init JESDPHY
    jesdphy_tx_disable();
    jesdphy_rx_reset();

    // init JESDLINK: reset needed after new parameters
    jesdlink_en_scrambling(0);
    jesdlink_subclass_set(0);
    jesdlink_k_f_set(jesd_param_init.jesd_K, jesd_param_init.jesd_F);
    jesdlink_reset();

    // Check JESDPHY status
    jesdphy_check_pll_status(&pll_status);

    usleep(100000);
    while (1) {
        
        uart_get_line(uart_line);
        handle_cmd(uart_line);
        udp_update();
    }

    return 0;
}

