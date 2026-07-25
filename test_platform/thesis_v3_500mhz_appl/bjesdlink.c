#include <stdint.h>
#include "bjesdlink.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"
#include "xparameters.h"

#define SET_BIT(n) ((uint32_t)1 << n)
#define CLEAR_BIT(n) (~((uint32_t)1 << n))

void jesdlink_en_scrambling(uint8_t en) {
    uint32_t tmp_reg;
    jesdlink_read(JESDLINK_CTRL_8B10B_CFG_REG, &tmp_reg);
    if (en > 0) {
        tmp_reg |= SET_BIT(16);
        jesdlink_write(JESDLINK_CTRL_8B10B_CFG_REG, tmp_reg);
        xil_printf("JESD204C IP scramble enabled.\r\n");
    }   
    else {
        tmp_reg &= CLEAR_BIT(16);
        jesdlink_write(JESDLINK_CTRL_8B10B_CFG_REG, tmp_reg);
        xil_printf("JESD204C IP scramble disabled.\r\n");
    }
    xil_printf("JESDS204C SUBCLASS REG = %x.\r\n", tmp_reg);
}
    
void jesdlink_k_f_set(uint8_t k, uint8_t f){
    uint32_t tmp_reg;
    jesdlink_read(JESDLINK_CTRL_8B10B_CFG_REG, &tmp_reg);
    tmp_reg &= ~(0x00001fff);
    tmp_reg = tmp_reg | ((k-1)<<8 | (f-1));
    jesdlink_write(JESDLINK_CTRL_8B10B_CFG_REG, tmp_reg);
    xil_printf("JESD204C K = %d, F =%d.\r\n",k,f);
    xil_printf("JESD204C SUBCLASS REG = %x.\r\n", tmp_reg);
}

void jesdlink_subclass_set(uint8_t subclass) {
    if (subclass > 3) {
        xil_printf("Error: subclass can only be 0 - 2!\r\n");
        return;
    }
    jesdlink_write(JESDLINK_CTRL_SUB_CLASS_REG, subclass);
    xil_printf("JESD204C subclass subclass = %d.\r\n", subclass);
}

void jesdlink_reset(void)
{
    uint32_t tmp_reg;

    xil_printf("JESD204C IP Reset Starting...\r\n");

    jesdlink_read(JESDLINK_RESET_REG, &tmp_reg);
    xil_printf("Initial RESET_REG = 0x%08lx\r\n",
               (unsigned long)tmp_reg);

    /* Assert reset bit 0 */
    tmp_reg |= (1U << 0);
    jesdlink_write(JESDLINK_RESET_REG, tmp_reg);

    usleep(1000);

    jesdlink_read(JESDLINK_RESET_REG, &tmp_reg);
    xil_printf("After assert RESET_REG = 0x%08lx\r\n",
               (unsigned long)tmp_reg);

    /* Release reset bit 0 */
    tmp_reg &= ~(1U << 0);
    jesdlink_write(JESDLINK_RESET_REG, tmp_reg);

    usleep(1000);

    jesdlink_read(JESDLINK_RESET_REG, &tmp_reg);
    xil_printf("After release RESET_REG = 0x%08lx\r\n",
               (unsigned long)tmp_reg);

    xil_printf("JESD204C IP Reset Finished.\r\n");
}

void jesdlink_read(uint32_t addr, uint32_t* data_ptr) {
    *data_ptr = Xil_In32(XPAR_JESD204C_0_BASEADDR + addr);
}
void jesdlink_write(uint32_t addr, uint32_t data) {
    Xil_Out32(XPAR_JESD204C_0_BASEADDR + addr, data);
}