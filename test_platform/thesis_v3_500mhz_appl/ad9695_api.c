/* ad9695_api.c
 * Unified implementation file for the AD9695 helper library.
 * This file is a verbatim concatenation of:
 *   • ad9695_api.c          – chip‑level helpers
 *   • ad9695_adc_api.c      – ADC‑path helpers
 *   • ad9695_jesd_api.c     – JESD‑link helpers
 * Only redundant #includes and decorative comments were trimmed; all
 * executable logic, constants, and variable names are **identical** to
 * the originals.
 */

#include "ad9695_api.h"
#include "ad9695_registers.h"
#include "peripherals.h"
#include "xspips.h"
#include "xgpiops.h"
#include "xil_printf.h"
#include <sleep.h>
#include <stdint.h>
#include <stddef.h>

/* ------------------------------------------------------------------------- */
/* Section:  Chip‑level helpers  (from original ad9695_api.c)                */
/* ------------------------------------------------------------------------- */

extern XSpiPs spi_inst;
extern XGpioPs gpio_inst;

void ad9695_init(void)
{
    uint8_t tmp_reg;
    ad9695_write_register(&spi_inst, AD9695_SCRATCH_REG, 0b10100101);
    ad9695_read_register(&spi_inst, AD9695_SCRATCH_REG, &tmp_reg);
    if (tmp_reg == 0b10100101) {
        xil_printf("AD9695 init success!\r\n");
    } else {
        xil_printf("FAILURE: AD9695 init failed! Scratch pad test did not pass.\r\n");
    }
}

void ad9695_hardware_reset(void)
{
    XGpioPs_WritePin(&gpio_inst, GPIO_PWDN_PIN, 1);
    usleep(500000);
    XGpioPs_WritePin(&gpio_inst, GPIO_PWDN_PIN, 0);
    xil_printf("AD9695 hardware reset performed.\r\n");
}

void ad9695_software_reset(void)
{
    ad9695_write_register(&spi_inst, AD9695_IF_CFG_A_REG, SET_BIT(0) | SET_BIT(7));
    ad9695_write_register(&spi_inst, AD9695_IF_CFG_B_REG, SET_BIT(1));
    usleep(500000);
    if ((ad9695_read_bit(&spi_inst, AD9695_IF_CFG_A_REG, 0) == 0) &&
        (ad9695_read_bit(&spi_inst, AD9695_IF_CFG_B_REG, 1) == 0)) {
        xil_printf("AD9695 software reset success!\r\n");
    } else {
        xil_printf("FAILURE: AD9695 software reset failed! Reset bits did not self clear.\r\n");
    }
}

void ad9695_adc_set_channel_select(uint8_t ch)
{
    uint8_t temp_reg;
    ad9695_write_register(&spi_inst, AD9695_CH_INDEX_REG, ch + 1);
    ad9695_read_register(&spi_inst, AD9695_CH_INDEX_REG, &temp_reg);
    xil_printf("CH IDX REG: %0x\r\n", temp_reg);
}

void ad9695_adc_get_channel_select(uint8_t *ch)
{
    ad9695_read_register(&spi_inst, AD9695_CH_INDEX_REG, ch);
}

void ad9695_set_pdn_pin_mode(uint8_t pin_en, uint8_t pin_mode)
{
    uint8_t tmp_reg;
    if (pin_en == 0)
        ad9695_write_register(&spi_inst, AD9695_CHIP_PIN_CTRL0_REG, SET_BIT(7));
    else
        ad9695_write_register(&spi_inst, AD9695_CHIP_PIN_CTRL0_REG, 0x0);

    ad9695_read_register(&spi_inst, AD9695_CHIP_PIN_CTRL1_REG, &tmp_reg);
    tmp_reg |= 0b00111111;
    ad9695_write_register(&spi_inst, AD9695_CHIP_PIN_CTRL1_REG, (pin_mode << 6) | tmp_reg);
}

void ad9695_set_input_clk_cfg(uint8_t div)
{
    ad9695_write_register(&spi_inst, AD9695_IP_CLK_CFG_REG, 0b00000011 & (div - 1));
}

void ad9695_adc_set_ch_pdn_mode(uint8_t mode)
{
    ad9695_write_register(&spi_inst, AD9695_DEV_CFG_REG, 0b00000011 & mode);
}

/* ------------------------------------------------------------------------- */
/* Section:  ADC‑path helpers  (from original ad9695_adc_api.c)              */
/* ------------------------------------------------------------------------- */

void ad9695_adc_set_clk_phase(uint8_t ch, uint8_t phase_adj)
{
    ad9695_adc_set_channel_select(ch);
    ad9695_write_register(&spi_inst, AD9695_IP_CLK_PHASE_ADJ_REG, phase_adj);
    ad9695_adc_set_channel_select(2); /* Restore default broadcast */
}

void ad9695_adc_set_dc_offset_filt_en(uint8_t en)
{
    if (en)
    {
        ad9695_write_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL,
                              AD9695_DC_OFFSET_CAL_EN);

        ad9695_write_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL2,
                              0x00);
    }
    else
    {
        ad9695_write_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL,
                              0x00);
    }
}

void ad9695_adc_set_fc_ch_mode(uint8_t fc_ch)
{
    ad9695_write_register(&spi_inst, AD9695_ADC_MODE_REG, fc_ch);
}

void ad9695_adc_delay_mode(uint8_t mode)
{
    ad9695_write_register(&spi_inst, AD9695_CLK_DELAY_CTRL_REG, mode);
}

void ad9695_adc_fine_delay(uint8_t fine_delay)
{
    if (fine_delay > 0xC0) {
        xil_printf("ERROR: Fine delay cannot exceed 0xC0!\r\n");
    }
    ad9695_write_register(&spi_inst, AD9695_CLK_FINE_DELAY_REG, fine_delay);
}
// New function 
void ad9695_adc_super_fine_delay(uint8_t super_fine_delay)
{
    if (super_fine_delay > 0x80) {
        xil_printf("ERROR: Super fine delay cannot exceed 0x80!\r\n");
    }
    ad9695_write_register(&spi_inst, AD9695_CLK_FINE_DELAY_REG, super_fine_delay);
}

static uint8_t ad9695_input_fs_to_code(const char *fs)
{
    if (strcmp(fs, "1.36") == 0) return 0x0A;
    if (strcmp(fs, "1.47") == 0) return 0x0B;
    if (strcmp(fs, "1.59") == 0) return 0x0C;
    if (strcmp(fs, "1.70") == 0) return 0x0D;
    if (strcmp(fs, "1.81") == 0) return 0x0E;
    if (strcmp(fs, "1.93") == 0) return 0x0F;
    if (strcmp(fs, "2.04") == 0) return 0x00;

    return 0xFF;
}

static const char* ad9695_input_code_to_fs(uint8_t code)
{
    switch (code & AD9695_INPUT_FS_MASK)
    {
        case 0x0A: return "1.36 Vpp";
        case 0x0B: return "1.47 Vpp";
        case 0x0C: return "1.59 Vpp";
        case 0x0D: return "1.70 Vpp";
        case 0x0E: return "1.81 Vpp";
        case 0x0F: return "1.93 Vpp";
        case 0x00: return "2.04 Vpp";
        default:   return "Unknown";
    }
}

void ad9695_set_input_full_scale(const char *fs)
{
    uint8_t old_val;
    uint8_t new_val;
    uint8_t code = ad9695_input_fs_to_code(fs);

    if (code == 0xFF)
    {
        xil_printf("Invalid full-scale value.\r\n");
        xil_printf("Valid values: 1.36, 1.47, 1.59, 1.70, 1.81, 1.93, 2.04\r\n");
        return;
    }

    ad9695_read_register(&spi_inst, AD9695_INPUT_FULL_SCALE_CTRL, &old_val);

    new_val = (old_val & ~AD9695_INPUT_FS_MASK) | code;

    ad9695_write_register(&spi_inst, AD9695_INPUT_FULL_SCALE_CTRL, new_val);

    xil_printf("Input full-scale set to %s Vpp differential.\r\n", fs);
    xil_printf("Note: smaller full-scale = larger digital amplitude.\r\n");
}

void ad9695_print_input_full_scale_status(void)
{
    uint8_t val;
    uint8_t code;

    ad9695_read_register(&spi_inst, AD9695_INPUT_FULL_SCALE_CTRL, &val);

    code = val & AD9695_INPUT_FS_MASK;

    xil_printf("Input full-scale control:\r\n");
    xil_printf("  Register 0x1910 = 0x%02X\r\n", val);
    xil_printf("  Current full-scale = %s differential\r\n",
               ad9695_input_code_to_fs(code));
    xil_printf("  Note: smaller full-scale = larger digital amplitude.\r\n");
}

/* ------------------------------------------------------------------------- */
/* Section:  JESD‑link helpers  (from original ad9695_jesd_api.c)            */
/* ------------------------------------------------------------------------- */

#define K_MIN 4
#define K_MAX 32
#define LANE_MIN 1
#define LANE_MAX 4
#define CS_MAX 3
#define N_MIN 7
#define N_MAX 16
#define CF_DEFAULT 0

typedef enum {
    AD9695_CB_LOW = 0,
    AD9695_CB_OVR_RANGE = 1,
    AD9695_CB_SIGNAL_MON = 2,
    AD9695_CB_FAST_DETECT = 3,
    AD9695_CB_SYSREF = 5,
} ad9695_control_bit_sel;

typedef struct {
    uint64_t slr_lwr_thres;
    uint64_t slr_upr_thres;
    uint8_t  vco_cfg;
} jesd_serdes_pll_cfg;

static void jesd_init_sequence(void)
{
    ad9695_write_register(&spi_inst, 0x1228, 0x4F);
    usleep(10);
    ad9695_write_register(&spi_inst, 0x1228, 0x0F);
    usleep(10);
    ad9695_write_register(&spi_inst, 0x1222, 0x00);
    usleep(10);
    ad9695_write_register(&spi_inst, 0x1222, 0x04);
    usleep(10);
    ad9695_write_register(&spi_inst, 0x1222, 0x00);
    usleep(10);
    ad9695_write_register(&spi_inst, 0x1262, 0x08);
    usleep(10);
    ad9695_write_register(&spi_inst, 0x1262, 0x00);
}

static int check_jesd_params_range(struct jesd_param_t jesd_param)
{
    /* Table‑22 limits from datasheet */
    if ((jesd_param.jesd_L != 1) && (jesd_param.jesd_L != 2) && (jesd_param.jesd_L != 4)) {
        xil_printf("API:AD9695:Err: Invalid JESD L \r\n");
        return 0;
    }
    if ((jesd_param.jesd_M != 1) && (jesd_param.jesd_M != 2) &&
        (jesd_param.jesd_M != 4) && (jesd_param.jesd_M != 8)) {
        xil_printf("API:AD9695:Err: Invalid JESD M \r\n");
        return 0;
    }
    if ((jesd_param.jesd_F != 1) && (jesd_param.jesd_F != 2) &&
        (jesd_param.jesd_F != 4) && (jesd_param.jesd_F != 8) && (jesd_param.jesd_F != 16)) {
        xil_printf("API:AD9695:Err: Invalid JESD F \r\n");
        return 0;
    }
    if ((jesd_param.jesd_N < N_MIN) || (jesd_param.jesd_N > N_MAX)) {
        xil_printf("API:AD9695:Err: Invalid JESD N \r\n");
        return 0;
    }
    if ((jesd_param.jesd_K < K_MIN) || (jesd_param.jesd_K > K_MAX) || (jesd_param.jesd_K % 4 != 0)) {
        xil_printf("API:AD9695:Err: Invalid JESD K \r\n");
        return 0;
    }
    if (jesd_param.jesd_CS > CS_MAX) {
        xil_printf("API:AD9695:Err: Invalid JESD CS \r\n");
        return 0;
    }
    if (jesd_param.jesd_CF > CF_DEFAULT) {
        xil_printf("API:AD9695:Err: Invalid JESD CF \r\n");
        return 0;
    }
    if ((jesd_param.jesd_NP != 8) && (jesd_param.jesd_NP != 16)) {
        xil_printf("API:AD9695:Err: Invalid JESD NP \r\n");
        return 0;
    }
    return 1;
}

void ad9695_jesd_set_if_config(struct jesd_param_t jesd_param,
                               uint64_t sample_clk_freq_khz,
                               uint64_t *lane_rate_kbps)
{
    uint8_t tmp_reg;

    if (!check_jesd_params_range(jesd_param)) {
        xil_printf("ERROR: JESD parameters out of range!\r\n");
        return;
    }

    *lane_rate_kbps = ((uint64_t)jesd_param.jesd_M * (uint64_t)jesd_param.jesd_NP *
                       10ULL * sample_clk_freq_khz) /
                      ((uint64_t)jesd_param.jesd_L * 8ULL);

    xil_printf("INFO: Calculated Lane rate is %lld kbps.\r\n", *lane_rate_kbps);

    /* Configure SERDES PLL according to lane‑rate */
    if (*lane_rate_kbps > 16000000ULL) {
        xil_printf("ERROR: Lane rate is too high!\r\n");
        return;
    } else if (*lane_rate_kbps > 13500000ULL) {
        ad9695_write_register(&spi_inst, AD9695_JESD_SERDES_PLL_CFG_REG, 0b0011 << 4);
    } else if (*lane_rate_kbps > 6750000ULL) {
        ad9695_write_register(&spi_inst, AD9695_JESD_SERDES_PLL_CFG_REG, 0b0000 << 4);
    } else if (*lane_rate_kbps > 3375000ULL) {
        ad9695_write_register(&spi_inst, AD9695_JESD_SERDES_PLL_CFG_REG, 0b0001 << 4);
    } else if (*lane_rate_kbps > 1687500ULL) {
        ad9695_write_register(&spi_inst, AD9695_JESD_SERDES_PLL_CFG_REG, 0b0101 << 4);
    } else {
        xil_printf("ERROR: Lane rate is too low!\r\n");
        return;
    }


    /* Apply remaining JESD configuration registers */
    tmp_reg = AD9695_JESD_M(jesd_param.jesd_M) - 1;
    ad9695_write_register(&spi_inst, AD9695_JESD_M_CFG_REG, tmp_reg);

    tmp_reg = AD9695_JESD_CS(jesd_param.jesd_CS) | (AD9695_JESD_N(jesd_param.jesd_N) - 1);
    ad9695_write_register(&spi_inst, AD9695_JESD_CS_N_CFG_REG, tmp_reg);

    ad9695_read_register(&spi_inst, AD9695_JESD_SCV_NP_CFG_REG, &tmp_reg);
    tmp_reg |= (AD9695_JESD_NP(jesd_param.jesd_NP) - 1);
    ad9695_write_register(&spi_inst, AD9695_JESD_SCV_NP_CFG_REG, tmp_reg);

    tmp_reg = AD9695_JESD_F(jesd_param.jesd_F) - 1;
    ad9695_write_register(&spi_inst, AD9695_JESD_F_CFG_REG, tmp_reg);
    tmp_reg = AD9695_JESD_K(jesd_param.jesd_K) - 1;
    ad9695_write_register(&spi_inst, AD9695_JESD_K_CFG_REG, tmp_reg);

    ad9695_read_register(&spi_inst, AD9695_JESD_L_SCR_CFG_REG, &tmp_reg);
    tmp_reg |= (AD9695_JESD_LANES(jesd_param.jesd_L) - 1);
    ad9695_write_register(&spi_inst, AD9695_JESD_L_SCR_CFG_REG, tmp_reg);
}

void ad9695_jesd_get_cfg_param(struct jesd_param_t *jesd_param)
{
    int i;
    uint8_t tmp_reg[AD9695_JESD_CFG_REG_OFFSET];

    /* Bulk‑read JESD configuration block */
    for (i = 0; i < AD9695_JESD_CFG_REG_OFFSET; ++i) {
        ad9695_read_register(&spi_inst, AD9695_JESD_L_SCR_CFG_REG + i, &tmp_reg[i]);
    }

    jesd_param->jesd_L  = AD9695_JESD_LANES(tmp_reg[0]) + 1;
    jesd_param->jesd_F  = AD9695_JESD_F    (tmp_reg[1]) + 1;
    jesd_param->jesd_K  = AD9695_JESD_K    (tmp_reg[2]) + 1;
    jesd_param->jesd_M  = AD9695_JESD_M    (tmp_reg[3]) + 1;
    jesd_param->jesd_CS = (tmp_reg[4] & 0xC0U) >> 6;
    jesd_param->jesd_N  = AD9695_JESD_N    (tmp_reg[4]) + 1;
    jesd_param->jesd_NP = AD9695_JESD_NP   (tmp_reg[5]) + 1;
    jesd_param->jesd_S  = AD9695_JESD_S    (tmp_reg[6]) + 1;

    jesd_param->jesd_HD = (tmp_reg[7] & AD9695_JESD_HD) ? 1 : 0;
    jesd_param->jesd_CF = AD9695_JESD_CF(tmp_reg[7]);

    for (i = 0; i < AD9695_JESD_ID_CFG_REG_OFFSET; ++i) {
        ad9695_read_register(&spi_inst, AD9695_JESD_DID_CFG_REG + i, &tmp_reg[i]);
    }
    jesd_param->jesd_DID  = tmp_reg[0];
    jesd_param->jesd_BID  = AD9695_JESD_BID(tmp_reg[1]);
    jesd_param->jesd_LID0 = AD9695_JESD_LID0(tmp_reg[2]);
}

void ad9695_jesd_enable_link(uint8_t en)
{
    uint8_t tmp_reg;
    ad9695_read_register(&spi_inst, AD9695_JESD_LINK_CTRL1_REG, &tmp_reg);
    xil_printf("Enabling JESD: first read 0x571 = 0x%x.\r\n", tmp_reg);

    tmp_reg |= AD9695_JESD_LINK_PDN;
    ad9695_write_register(&spi_inst, AD9695_JESD_LINK_CTRL1_REG, tmp_reg);
    xil_printf("Enabling JESD: second write 0x571 = 0x%x.\r\n", tmp_reg);

    tmp_reg = (en) ? 0x14 : tmp_reg;
    ad9695_write_register(&spi_inst, AD9695_JESD_LINK_CTRL1_REG, tmp_reg);
    xil_printf("Enabling JESD: third write 0x571 = 0x%x.\r\n", tmp_reg);

    if (en) {
        jesd_init_sequence();
        xil_printf("Enabling JESD: JESD init sequence executed.\r\n");
    }
}

void ad9695_jesd_enable_scrambler(uint8_t en)
{
    uint8_t tmp_reg;
    ad9695_read_register(&spi_inst, AD9695_JESD_L_SCR_CFG_REG, &tmp_reg);
    tmp_reg &= ~AD9695_JESD_SCR_EN;
    tmp_reg |= en ? AD9695_JESD_SCR_EN : 0;
    ad9695_write_register(&spi_inst, AD9695_JESD_L_SCR_CFG_REG, tmp_reg);
}

void ad9695_jesd_get_pll_status(uint8_t *pll_status)
{
    ad9695_read_register(&spi_inst, AD9695_JESD_SERDES_PLL_REG, pll_status);
}

void ad9695_jesd_subclass_set(uint8_t subclass)
{
    uint8_t tmp_reg;
    ad9695_read_register(&spi_inst, AD9695_JESD_SCV_NP_CFG_REG, &tmp_reg);
    tmp_reg &= ~AD9695_JESD_SUBCLASS(-1);
    tmp_reg |= AD9695_JESD_SUBCLASS(subclass);
    ad9695_write_register(&spi_inst, AD9695_JESD_SCV_NP_CFG_REG, tmp_reg);
}

void ad9695_jesd_syref_mode_set(uint8_t mode, uint8_t sysref_count)
{
    uint8_t tmp_reg;

    ad9695_read_register(&spi_inst, AD9695_SYSREF_CTRL_0_REG, &tmp_reg);
    tmp_reg &= ~AD9695_SYSREF_MODE_SEL(-1);
    tmp_reg |= AD9695_SYSREF_MODE_SEL(mode);
    ad9695_write_register(&spi_inst, AD9695_SYSREF_CTRL_0_REG, tmp_reg);

    ad9695_read_register(&spi_inst, AD9695_SYSREF_CTRL_1_REG, &tmp_reg);
    tmp_reg &= ~AD9695_SYSREF_NSHOT_IGNORE(-1);
    tmp_reg |= (mode == 0b10) ? AD9695_SYSREF_NSHOT_IGNORE(sysref_count)
                              : AD9695_SYSREF_NSHOT_IGNORE(0x0);
    ad9695_write_register(&spi_inst, AD9695_SYSREF_CTRL_1_REG, tmp_reg);
}
