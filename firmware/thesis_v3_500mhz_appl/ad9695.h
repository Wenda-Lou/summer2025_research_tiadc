/***************************************************************************//**
 *   @file   ad9695.h
 *   @brief  Header file of ad9695 Driver.
 *   @author Stefan Popa (stefan.popa@analog.com)
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __ad9695_H__
#define __ad9695_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ad9695_FULL_BANDWIDTH_MODE 0

#define ad9695_NCO_MODE_VIF 0	/* Variable IF Mode */
#define ad9695_NCO_MODE_ZIF 1	/* Zero IF Mode */
#define ad9695_NCO_MODE_TEST 3	/* Test Mode*/

#define ad9695_BUFF_CURR_400_UA  0x4	/* Buffer Current set to 400 uA */
#define ad9695_BUFF_CURR_500_UA  0x9	/* Buffer Current set to 500 uA */
#define ad9695_BUFF_CURR_600_UA  0x1E	/* Buffer Current set to 600 uA */
#define ad9695_BUFF_CURR_700_UA  0x23	/* Buffer Current set to 700 uA */
#define ad9695_BUFF_CURR_800_UA  0x28	/* Buffer Current set to 800 uA */
#define ad9695_BUFF_CURR_1000_UA  0x32	/* Buffer Current set to 1000 uA */

#define ad9695_CHIP_TYPE	0x03
#define ad9695_CHIP_ID		0xDF

struct ad9695_state{
	uint64_t sample_clk_freq_khz;
	uint8_t powerdown_pin_en;
	uint32_t powerdown_mode;
	uint8_t fc_ch;
	uint32_t test_mode_ch0;
	uint32_t test_mode_ch1;
    struct jesd_param_t *jesd_param;
	uint32_t jesd_subclass;

    // force CGS 
    uint8_t force_cgs;

    // interleaving controls
    uint8_t num_of_half_clk_cycle_delay_ch0;
    uint8_t num_of_half_clk_cycle_delay_ch1;

    // delay mode select
    uint8_t clk_delay_mode_sel;

    //    
};

// Additional Test Mode functions
void ad9695_jesd_link_force_cgs();

/* Initialize the device. */
//void ad9695_initialize(ad9695_init_param *init_param);
void ad9695_initialize(struct ad9695_state* init_state_ptr);

#endif // __ad9695_H__
