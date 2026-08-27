#include <stdio.h>
#include "ad9695.h"
#include <stdint.h>
#include <xil_printf.h>
#include "xspips.h"
#include "peripherals.h"
#include "ad9695_registers.h"
#include "sleep.h"
#include "ad9695_api.h"

// External SPI inst for ZCU102
extern XSpiPs spi_inst; 

/**
 * Set the test mode compatible with ad9695 API
 * mode - Test mode. Accepted values:
 * 		 AD9695_TESTMODE_OFF
 * 		 AD9695_TESTMODE_MIDSCALE_SHORT
 * 		 AD9695_TESTMODE_POS_FULLSCALE
 * 		 AD9695_TESTMODE_NEG_FULLSCALE
 * 		 AD9695_TESTMODE_ALT_CHECKERBOARD
 * 		 AD9695_TESTMODE_PN23_SEQ
 * 		 AD9695_TESTMODE_PN9_SEQ
 * 		 AD9695_TESTMODE_ONE_ZERO_TOGGLE
 * 		 AD9695_TESTMODE_USER
 * 		 AD9695_TESTMODE_RAMP
 */
static void ad9695_testmode_set(uint8_t ch, uint8_t mode)
{
    ad9695_adc_set_channel_select(ch);
	ad9695_write_register(&spi_inst, AD9695_REG_TEST_MODE, mode);
    ad9695_adc_set_channel_select(2);
}


/**
 * Initialize the device.
 */
void ad9695_initialize(struct ad9695_state* state_ptr)
{
	uint64_t sample_rate_khz, lane_rate_kbps;
	uint8_t pll_stat;
	int32_t timeout;

    // Reset
    ad9695_hardware_reset();

    // Try Scratch Pad Function
	ad9695_init();

	ad9695_software_reset();

    // Use all channels
	ad9695_adc_set_channel_select(AD9695_ADC_CH_ALL - 1);

    // Set powerdown pin enable and mode
	ad9695_set_pdn_pin_mode(state_ptr->powerdown_pin_en, state_ptr->powerdown_mode);

	sample_rate_khz = state_ptr->sample_clk_freq_khz;

    // Set Operation Mode
	ad9695_adc_set_fc_ch_mode(state_ptr->fc_ch);

    // Channel test mode, will return to both channels
	ad9695_testmode_set(0, state_ptr->test_mode_ch0);
    ad9695_testmode_set(1, state_ptr->test_mode_ch1);

    // JESD
	ad9695_jesd_set_if_config((struct jesd_param_t) * state_ptr->jesd_param, sample_rate_khz, &lane_rate_kbps);
	ad9695_jesd_subclass_set(state_ptr->jesd_subclass);
	ad9695_jesd_enable_scrambler(0);

    // Force CGS Mode
    if (state_ptr->force_cgs == 1) {
        ad9695_jesd_link_force_cgs();
        xil_printf("AD9695 is forcing CGS. (constatnt k28.5)\r\n");
    }

    // Set appropriate delays
    

    // Enable the link with JESD init sequence. IMPORTANT!
	ad9695_jesd_enable_link(1);

	timeout = 10;

	do {
		usleep(10000);
		ad9695_jesd_get_pll_status(&pll_stat);
	} while (!(pll_stat & AD9695_JESD_PLL_LOCK_STAT) && timeout--);

	xil_printf("ad9695 PLL %s\r\n", (pll_stat & AD9695_JESD_PLL_LOCK_STAT) ? "LOCKED" : "UNLOCKED");
	printf("ad9695 successfully initialized\n");
}

/*
 * Forces the link to output K28.5
 */
void ad9695_jesd_link_force_cgs() {
    ad9695_write_register(&spi_inst, AD9695_JESD_LINK_CTRL2_REG, AD9695_JESD_LINK_FORCE_CGS);
}
