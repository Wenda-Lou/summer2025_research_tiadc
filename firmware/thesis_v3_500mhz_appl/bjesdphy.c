#include "bjesdphy.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"
#include "xparameters.h"
#include <stdint.h>

void jesdphy_read(uint32_t addr, uint32_t* data_ptr) {
    *data_ptr = Xil_In32(XPAR_JESD204_PHY_0_BASEADDR + addr);
}

void jesdphy_write(uint32_t addr, uint32_t data) {
    Xil_Out32(XPAR_JESD204_PHY_0_BASEADDR + addr, data);
}

void jesdphy_tx_disable() {
    // keep the reset high
    jesdphy_write(JESDPHY_TX_RESET_REG, 0x0001);
    // power down TX
    jesdphy_write(JESDPHY_TXPD_REG, 0x0003);
    xil_printf("JESDPHY TX has been disabled.\r\n");
}

void jesdphy_rx_reset() {
    jesdphy_write(JESDPHY_RX_RESET_REG, 0x0001);
    usleep(1000);
    jesdphy_write(JESDPHY_RX_RESET_REG, 0x0000);
}

void jesdphy_get_pll_status(struct jesdphy_pll_status* status_ptr) {
    uint32_t tmp_reg;
    jesdphy_read(JESDPHY_PLL_STATUS_REG, &tmp_reg);
    status_ptr->tx_reset_in_prog = ((uint8_t)tmp_reg >> 4) & 1;
    status_ptr->rx_reset_in_prog = ((uint8_t)tmp_reg >> 3) & 1;
    status_ptr->cpll_unlocked = ((uint8_t)tmp_reg >> 2) & 1; 
    status_ptr->qpll0_unlocked = ((uint8_t)tmp_reg >> 1) & 1; 
    status_ptr->qpll1_unlocked = ((uint8_t)tmp_reg >> 0) & 1;
}

void jesdphy_drp_reset() {
    jesdphy_write(JESDPHY_COMMON_DRP_RESET_REG, 0x0001);
    jesdphy_write(JESDPHY_TRANSC_DRP_RESET_REG, 0x0001);
    usleep(1000);
}

void jesdphy_drp_common_read(uint8_t interface_sel, uint32_t drp_addr, uint32_t* data_ptr) {
    uint8_t timeout = 100;
    uint32_t tmp_reg;

    // set interface
    jesdphy_write(JESDPHY_COMMON_INTERFACE_SEL_REG, interface_sel);

    // write to drp addr reg
    jesdphy_write(JESDPHY_COMMON_DRP_ADDR_REG, drp_addr&0x40000000); // bit 30 is set to 1 to perform a read

    // check complete
    do {
        jesdphy_read(JESDPHY_COMMON_DRP_STATUS_REG, &tmp_reg);
        timeout --;
    }while(((tmp_reg&0x00000001) == 0x1) && (timeout > 0));
    
    if ((tmp_reg&0x00000001) == 0x1) {
        xil_printf("Timeout during DRP access!\r\n");
        return;
    }

    jesdphy_read(JESDPHY_COMMON_DRP_RDATA_REG, data_ptr);
    *data_ptr = *data_ptr & 0x0000ffff;
}

void jesdphy_drp_common_write(uint8_t interface_sel, uint32_t drp_addr, uint32_t data) {
    uint8_t timeout = 100;
    uint32_t tmp_reg;

    // set interface
    jesdphy_write(JESDPHY_COMMON_INTERFACE_SEL_REG, interface_sel);

    // write to drp wdata reg
    jesdphy_write(JESDPHY_COMMON_DRP_WDATA_REG, data&0x0000ffff);

    // write to drp addr reg
    jesdphy_write(JESDPHY_COMMON_DRP_ADDR_REG, drp_addr&0x80000000); // bit 30 is set to 1 to perform a read

    // check complete
    do {
        jesdphy_read(JESDPHY_COMMON_DRP_STATUS_REG, &tmp_reg);
        timeout --;
    }while(((tmp_reg&0x00000001) == 0x1) && (timeout > 0));
    
    if ((tmp_reg&0x00000001) == 0x1) {
        xil_printf("Timeout during DRP access!\r\n");
        return;
    }
}

void jesdphy_drp_transceiver_read(uint8_t interface_sel, uint32_t drp_addr, uint32_t* data_ptr) {
    uint8_t timeout = 100;
    uint32_t tmp_reg;

    // set interface
    jesdphy_write(JESDPHY_GT_INTERFACE_SEL_REG, interface_sel);

    // write to drp addr reg
    jesdphy_write(JESDPHY_TRANSC_DRP_ADDR_REG, drp_addr&0x40000000); // bit 30 is set to 1 to perform a read

    // check complete
    do {
        jesdphy_read(JESDPHY_TRANSC_DRP_STATUS_REG, &tmp_reg);
        timeout --;
    }while(((tmp_reg&0x00000001) == 0x1) && (timeout > 0));
    
    if ((tmp_reg&0x00000001) == 0x1) {
        xil_printf("Timeout during DRP access!\r\n");
        return;
    }

    jesdphy_read(JESDPHY_TRANSC_DRP_RDATA_REG, data_ptr);
    *data_ptr = *data_ptr & 0x0000ffff;
}
void jesdphy_drp_transceiver_write(uint8_t interface_sel, uint32_t drp_addr, uint32_t data) {
    uint8_t timeout = 100;
    uint32_t tmp_reg;

    // set interface
    jesdphy_write(JESDPHY_GT_INTERFACE_SEL_REG, interface_sel);

    // write to drp wdata reg
    jesdphy_write(JESDPHY_TRANSC_DRP_WDATA_REG, data&0x0000ffff);

    // write to drp addr reg
    jesdphy_write(JESDPHY_TRANSC_DRP_ADDR_REG, drp_addr&0x80000000); // bit 30 is set to 1 to perform a read

    // check complete
    do {
        jesdphy_read(JESDPHY_TRANSC_DRP_STATUS_REG, &tmp_reg);
        timeout --;
    }while(((tmp_reg&0x00000001) == 0x1) && (timeout > 0));
    
    if ((tmp_reg&0x00000001) == 0x1) {
        xil_printf("Timeout during DRP access!\r\n");
        return;
    }
}

void jesdphy_check_pll_status(struct jesdphy_pll_status* pll_status_ptr) {
    int timeout = 1000;
     do {
        timeout--;
        jesdphy_get_pll_status(pll_status_ptr);
    }while ((timeout > 0) && (pll_status_ptr->qpll0_unlocked == 1) && (pll_status_ptr->rx_reset_in_prog == 1));

    if ((pll_status_ptr->rx_reset_in_prog) == 0 && (pll_status_ptr->qpll0_unlocked == 0)) {
        xil_printf("JESDPHY: RX reset complete and QPLL locked.\r\n");
    }
    else {
        xil_printf("JESDPHY: RX reset not complete or QPLL not locked.\r\n");
    }
}