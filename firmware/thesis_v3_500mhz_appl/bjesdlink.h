#ifndef BJESDLINK_H
#define BJESDLINK_H
#include <stdint.h>

#define JESDLINK_RESET_REG                  0x0020
#define JESDLINK_CTRL_SUB_CLASS_REG         0x0034
#define JESDLINK_CTRL_8B10B_CFG_REG         0x003c
#define JESDLINK_CTRL_LANE_ENA_REG          0x0040
#define JESDLINK_CTRL_RX_BUF_ADV_REG        0x0044
#define JESDLINK_CTRL_TEST_MODE_REG         0x0048
#define JESDLINK_CTRL_SYSREF_REG            0x0050
#define JESDLINK_STAT_RX_ERR_REG            0x0058
#define JESDLINK_STAT_RX_DEBUG_REG          0x005c
#define JESDLINK_STAT_STATUS_REG            0x0060
#define JESDLINK_CTRL_IRQ_REG               0x0064
#define JESDLINK_STAT_IRQ_REG               0x0068
#define JESDLINK_STAT_RX_BUF_LVL_REG(n)     (0x0400 + (n*0x0080))
#define JESDLINK_STAT_LINK_ERR_CNT(n)       (0x0420 + (n*0x0080))
#define JESDLINK_STAT_TEST_ERR_CNT(n)       (0x0424 + (n*0x0080))
#define JESDLINK_STAT_TEST_ILA_CNT(n)       (0x0428 + (n*0x0080))
#define JESDLINK_STAT_TEST_MF_CNT(n)        (0x042c + (n*0x0080))
#define JESDLINK_CTRL_RX_ILACFG0_REG(n)     (0x0430 + (n*0x0080))
#define JESDLINK_CTRL_RX_ILACFG1_REG(n)     (0x0434 + (n*0x0080))
#define JESDLINK_CTRL_RX_ILACFG2_REG(n)     (0x0438 + (n*0x0080))
#define JESDLINK_CTRL_RX_ILACFG3_REG(n)     (0x043c + (n*0x0080))
#define JESDLINK_CTRL_RX_ILACFG4_REG(n)     (0x0440 + (n*0x0080))
#define JESDLINK_CTRL_RX_ILACFG5_REG(n)     (0x0444 + (n*0x0080))
#define JESDLINK_CTRL_RX_ILACFG6_REG(n)     (0x0448 + (n*0x0080))
#define JESDLINK_CTRL_RX_ILACFG7_REG(n)     (0x044c + (n*0x0080))
#define JESDLINK_CTRL_TX_GT(n)              (0x0460 + (n*0x0080))
#define JESDLINK_CTRL_RX_GT(n)              (0x0464 + (n*0x0080))

void jesdlink_read(uint32_t addr, uint32_t* data_ptr);
void jesdlink_write(uint32_t addr, uint32_t data);
void jesdlink_reset();
void jesdlink_subclass_set(uint8_t subclass);
void jesdlink_en_scrambling(uint8_t en);
void jesdlink_k_f_set(uint8_t k, uint8_t f);


#endif