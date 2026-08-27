#ifndef BJESDPHY_H
#define BJESDPHY_H
#include <stdint.h>

#define JESDPHY_COMMON_INTERFACE_NUM_REG    0x0008
#define JESDPHY_GT_INTERFACE_NUM_REG        0x000c
#define JESDPHY_TIMEOUT_EN_REG              0x0014
#define JESDPHY_TIMEOUT_VALUE_REG           0x001c
#define JESDPHY_COMMON_INTERFACE_SEL_REG    0x0020
#define JESDPHY_GT_INTERFACE_SEL_REG        0x0024
#define JESDPHY_PLL_STATUS_REG              0x0080
#define JESDPHY_COMMON_DRP_ADDR_REG         0x0104
#define JESDPHY_TRANSC_DRP_ADDR_REG         0x0204
#define JESDPHY_COMMON_DRP_WDATA_REG        0x0108
#define JESDPHY_TRANSC_DRP_WDATA_REG        0x0208
#define JESDPHY_COMMON_DRP_RDATA_REG        0x010c
#define JESDPHY_TRANSC_DRP_RDATA_REG        0x020c
#define JESDPHY_COMMON_DRP_RESET_REG        0x0110
#define JESDPHY_TRANSC_DRP_RESET_REG        0x0210
#define JESDPHY_COMMON_DRP_STATUS_REG       0x0114
#define JESDPHY_TRANSC_DRP_STATUS_REG       0x0214
#define JESDPHY_COMMON_DRP_COMPLETE_REG     0x011c
#define JESDPHY_TRANSC_DRP_COMPLETE_REG     0x021c
#define JESDPHY_QPLL0_PDWN_REG              0x0304
#define JESDPHY_QPLL1_PDWN_REG              0x0308
#define JESDPHY_RXPD_REG                    0x0404
#define JESDPHY_CPLL_PDWN_REG               0x0408
#define JESDPHY_TX_PLL_SEL_REG              0x040c
#define JESDPHY_RX_PLL_SEL_REG              0x0410
#define JESDPHY_TX_RESET_REG                0x0420
#define JESDPHY_RX_RESET_REG                0x0424
#define JESDPHY_TXPD_REG                    0x0504

struct jesdphy_pll_status {
    uint8_t tx_reset_in_prog;
    uint8_t rx_reset_in_prog;
    uint8_t cpll_unlocked;
    uint8_t qpll0_unlocked;
    uint8_t qpll1_unlocked;
};

void jesdphy_tx_disable();
void jesdphy_rx_reset();
void jesdphy_get_pll_status(struct jesdphy_pll_status* status_ptr);
void jesdphy_read(uint32_t addr, uint32_t* data_ptr);
void jesdphy_write(uint32_t addr, uint32_t data);
void jesdphy_drp_reset();
void jesdphy_drp_common_read(uint8_t interface_sel, uint32_t drp_addr, uint32_t* data_ptr);
void jesdphy_drp_common_write(uint8_t interface_sel, uint32_t drp_addr, uint32_t data);
void jesdphy_drp_transceiver_read(uint8_t interface_sel, uint32_t drp_addr, uint32_t* data_ptr);
void jesdphy_drp_transceiver_write(uint8_t interface_sel, uint32_t drp_addr, uint32_t data);
void jesdphy_check_pll_status(struct jesdphy_pll_status* pll_status_ptr);


#endif