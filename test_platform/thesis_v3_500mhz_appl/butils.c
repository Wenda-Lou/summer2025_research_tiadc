/* butils.c
 * Concise, table-driven UART command handler.
 */

#include "butils.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "xil_printf.h"
#include "xparameters.h"
#include "peripherals.h"
#include "bjesdphy.h"
#include "bjesdlink.h"
#include "ad9695_api.h"
#include "xspips.h"
#include "xaxidma.h"
#include "baxidma.h"
#include "sleep.h"
#include "ad9695_registers.h"
#include "ethernet.h"

#include "xil_cache.h"
#include "xuartps.h"

extern XUartPs uart_inst;
extern XSpiPs spi_inst;
extern XAxiDma dma_inst;
extern u8 *RxBufferPtr;

extern uint8_t uart_send_flag; //Send flag enabled by the uart
volatile uint8_t adc_sweep_active = 0;

#define ERR(fmt, ...) xil_printf("Command Error: " fmt "\r\n", ##__VA_ARGS__)

static int adc_capture_frame(void);
static void adc_ifc_sweep(void);

static int next_tok(char **ctx, char *out, size_t len) {
    char *t = strtok(NULL, " ");
    if (!t) return 0;
    strncpy(out, t, len - 1);
    out[len - 1] = '\0';
    return 1;
}

static void parse_cmd_args(char *line, char *option, size_t opt_len, char *addr_str, size_t addr_len, char *data_str, size_t data_len, const char *cmd_name) {
    char *ctx = line;
    strtok(ctx, " "); // skip command name
    if (!next_tok(&ctx, option, opt_len)) { ERR("Missing option (-r / -w)"); return; }
    if (!next_tok(&ctx, addr_str, addr_len)) { ERR("Missing address"); return; }
    if (!strcmp(option, "-w") && !next_tok(&ctx, data_str, data_len)) { ERR("Missing write data"); return; }
}

// Handler for SPI commands
void handle_spi_cmd(char *line) {
    char option[4], addr_str[8], data_str[4];
    uint16_t addr;
    uint8_t data;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "spi");
    addr = (uint16_t)strtol(addr_str, NULL, 0);

    if (!strcmp(option, "-r")) {
        ad9695_read_register(&spi_inst, addr, &data);
        xil_printf("Value at 0x%04X = 0x%02X\r\n", addr, data);
    } else if (!strcmp(option, "-w")) {
        data = (uint8_t)strtol(data_str, NULL, 0);
        ad9695_write_register(&spi_inst, addr, data);
        xil_printf("Command Success: Wrote 0x%02X to 0x%04X\r\n", data, addr);
    } else ERR("Invalid option '%s' (use -r or -w)", option);
}

// Handler for JESD204 PHY commands
void handle_phy_cmd(char *line) {
    char option[4], addr_str[12], data_str[12];
    uint32_t addr, data, tmp_reg;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "phy");
    addr = (uint32_t)strtoul(addr_str, NULL, 0);

    if (!strcmp(option, "-r")) {
        jesdphy_read(addr, &tmp_reg);
        xil_printf("Value at 0x%08X = 0x%08X\r\n", XPAR_JESD204_PHY_0_BASEADDR + addr, tmp_reg);
    } else if (!strcmp(option, "-w")) {
        data = (uint32_t)strtoul(data_str, NULL, 0);
        jesdphy_write(addr, data);
        xil_printf("Wrote 0x%08X to 0x%08X\r\n", data, XPAR_JESD204_PHY_0_BASEADDR + addr);
    } else ERR("Invalid option '%s' (use -r or -w)", option);
}

// Handler for JESD204 Link-layer commands
void handle_link_cmd(char *line) {
    char option[4], addr_str[12], data_str[12];
    uint32_t addr, data, tmp_reg;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "link");
    addr = (uint32_t)strtoul(addr_str, NULL, 0);

    if (!strcmp(option, "-r")) {
        jesdlink_read(addr, &tmp_reg);
        xil_printf("Value at 0x%08X = 0x%08X\r\n", XPAR_JESD204C_0_BASEADDR + addr, tmp_reg);
    } else if (!strcmp(option, "-w")) {
        data = (uint32_t)strtoul(data_str, NULL, 0);
        jesdlink_write(addr, data);
        xil_printf("Wrote 0x%08X to 0x%08X\r\n", data, XPAR_JESD204C_0_BASEADDR + addr);
    } else ERR("Invalid option '%s' (use -r or -w)", option);
}

void handle_dma_cmd(char* line) {
    char copy[MAX_UART_LINE_LENGTH];
    char option[4];

    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char* token = strtok(copy, " ");
    if (!token || strcmp(token, "dma") != 0) { ERR("Expected \"dma\""); return; }

    token = strtok(NULL, " ");
    if (!token) { ERR("Missing option (-r or -w)"); return; }
    strncpy(option, token, sizeof(option) - 1);
    option[sizeof(option) - 1] = '\0';

    if (strcmp(option, "-w") == 0) {
        xil_printf("Starting DMA capture of %d bytes...\r\n", DMA_CMD_BUF_SIZE);
        if (adc_sweep_active)
        {
            ERR("DMA commands are disabled while an ADC sweep is in progress.");
            return;
        }
        Xil_DCacheFlushRange((UINTPTR)RxBufferPtr, DMA_CMD_BUF_SIZE);
        int res =XAxiDma_SimpleTransfer(&dma_inst, (UINTPTR) RxBufferPtr,
                        DMA_CMD_BUF_SIZE, XAXIDMA_DEVICE_TO_DMA);

        if (res != XST_SUCCESS) { ERR("XAxiDma_SimpleTransfer failed. Error Code: %d.", res); return; }
        u32 timeout = 100000;
        int busy;
        do {
            busy = XAxiDma_Busy(&dma_inst, XAXIDMA_DEVICE_TO_DMA);
            if(!busy){ break; }
            timeout --;
            usleep(1);
        }while(timeout > 0);
        if (busy) { xil_printf("DMA was still busy and timed out.\r\n"); }
        else { 
            xil_printf("DMA Finished Successfully.\r\n"); 
            Xil_DCacheInvalidateRange((UINTPTR)RxBufferPtr, DMA_CMD_BUF_SIZE);
        }
        xil_printf("dma -w complete.\r\n");
    } else if (strcmp(option, "-r") == 0) {
            Xil_DCacheInvalidateRange((UINTPTR)RxBufferPtr, DMA_CMD_BUF_SIZE);
        if (adc_sweep_active)
        {
            ERR("DMA commands are disabled while an ADC sweep is in progress.");
            return;
        }
        xil_printf("Reading back %d bytes:\r\n", DMA_CMD_BUF_SIZE);
        for (uint32_t i = 0; i < DMA_CMD_BUF_SIZE; i+=16) {
            xil_printf("@0x%02X = 0x%02X ", i, RxBufferPtr[i]);
            xil_printf("\r\n");
        }
        xil_printf("\r\n");
    } else if (strcmp(option, "-d") == 0) {
        if (adc_sweep_active)
        {
            ERR("DMA commands are disabled while an ADC sweep is in progress.");
            return;
        }
        XAxiDma_Reset(&dma_inst);
        xil_printf("reset completed!\r\n");
    } else if (strcmp(option, "-c") == 0) {
        if (adc_sweep_active)
        {
            ERR("DMA commands are disabled while an ADC sweep is in progress.");
            return;
        }
        XAxiDma_Resume(&dma_inst);
        xil_printf("resume completed!\r\n");
    } else { ERR("Invalid option \"%s\" (use -r or -w or -d)", option); }
}

void handle_mem_cmd(char* line) {
    char option[4], addr_str[12], data_str[12];
    uint32_t addr, data, tmp_reg;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "mem");
    addr = (uint32_t)strtoul(addr_str, NULL, 0);

    if (strcmp(option, "-r") == 0) {
        tmp_reg = Xil_In32(addr);
        xil_printf("Command Success: Value at 0x%08X = 0x%08X\r\n", addr, tmp_reg);
    } else if (strcmp(option, "-w") == 0) {
        data = (uint32_t)strtoul(data_str, NULL, 0);
        Xil_Out32(addr, data);
        xil_printf("Command Success: Wrote 0x%08X to 0x%08X\r\n", data, addr);
    } else { ERR("Invalid option \"%s\" (use -r or -w)", option); }
}

#define DMA_CTRL_BASE XPAR_AXI_DMA_0_BASEADDR // Unused macro

void handle_dma_dbg_cmd(char* line) {
    char option[4], addr_str[12], data_str[12];
    uint32_t offset, data, reg_val;

    parse_cmd_args(line, option, sizeof(option), addr_str, sizeof(addr_str), data_str, sizeof(data_str), "dbg");
    offset = (uint32_t)strtoul(addr_str, NULL, 0);
    uint32_t addr = RxBufferPtr + offset;

    if (strcmp(option, "-r") == 0) {
        reg_val = Xil_In32(addr);
        xil_printf("Command Success: DMA[0x%08X] = 0x%08X\r\n", offset, reg_val);
    } else if (strcmp(option, "-w") == 0) {
        data = (uint32_t)strtoul(data_str, NULL, 0);
        Xil_Out32(addr, data);
        xil_printf("Command Success: Wrote 0x%08X to DMA[0x%08X]\r\n", data, offset);
    } else { ERR("Invalid option \"%s\" (use -r or -w)", option); }
} 

void handle_udp_cmd(char *line)
{
    if (adc_sweep_active)
    {
        ERR("UDP transmission is disabled while an ADC sweep is in progress.");
        return;
    }

    uart_send_flag = 1;
}

void handle_adc_cmd(char* line)
{
    char copy[MAX_UART_LINE_LENGTH];
    char option[16];

    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char* token = strtok(copy, " ");
    if (!token || strcmp(token, "adc") != 0) { ERR("Expected \"adc\""); return; }

    token = strtok(NULL, " ");
    if (!token) { ERR("Missing option"); return; }
    strncpy(option, token, sizeof(option) - 1);
    option[sizeof(option) - 1] = '\0';

    if(strcmp(option, "-c") == 0){
        int timeout = 10;
        uint8_t pll_stat;

        do {
            usleep(10000);
            ad9695_jesd_get_pll_status(&pll_stat);
        } while (!(pll_stat & AD9695_JESD_PLL_LOCK_STAT) && timeout--);

        xil_printf("ad9695 PLL %s\r\n", (pll_stat & AD9695_JESD_PLL_LOCK_STAT) ? "LOCKED" : "UNLOCKED");
        jesdphy_check_pll_status(&pll_stat);
    } else if (strcmp(token, "?") == 0 || strcmp(token, "status") == 0){
        uint8_t r701, r73b;

        ad9695_read_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL, &r701);
        ad9695_read_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL2, &r73b);

        if ((r701 & AD9695_DC_OFFSET_CAL_EN) && ((r73b & AD9695_DC_OFFSET_CAL_EN) == 0))
        {
            xil_printf("DC offset calibration: ON\r\n");
        }
        else
        {
            xil_printf("DC offset calibration: OFF\r\n");
        }

        xil_printf("0x0701 = 0x%02X\r\n", r701);
        xil_printf("0x073B = 0x%02X\r\n", r73b);
    } else if (strcmp(option, "-gain") == 0) {
        
        handle_adc_gain_cmd();

    } else if (strcmp(option, "-offset") == 0)
    {
        handle_adc_offset_cmd();
    } else {ERR("Invalid option \"%s\" (use -c or -offset)", option);}
}

typedef void (*cmd_fn)(char *line);
static const struct { const char *name; cmd_fn fn; } cmd_table[] = {
    { "spi",  handle_spi_cmd  },
    { "phy",  handle_phy_cmd  },
    { "link", handle_link_cmd },
    { "dma",  handle_dma_cmd  },
    { "dbg",  handle_dma_dbg_cmd  },
    { "mem",  handle_mem_cmd  },
    { "udp",  handle_udp_cmd  },
    { "adc",  handle_adc_cmd  }
};

void handle_cmd(char *line) {
    if (!line || !*line) { ERR("empty command"); return; }

    char cmd[8];
    strncpy(cmd, line, sizeof cmd - 1);
    cmd[sizeof cmd - 1] = '\0';
    char *space = strchr(cmd, ' ');
    if (space) *space = '\0';

    for (size_t i = 0; i < sizeof cmd_table / sizeof cmd_table[0]; ++i) {
        if (!strcmp(cmd, cmd_table[i].name)) { cmd_table[i].fn(line); return; }
    }
    xil_printf("Invalid command type: %s\r\n", cmd);
}

void handle_adc_gain_cmd(void)
{
    char line[MAX_UART_LINE_LENGTH];
    char copy[MAX_UART_LINE_LENGTH];
    char *token;

    xil_printf("\r\nEntering ADC Gain setting menu\r\n");
    xil_printf("Available commands:\r\n");
    xil_printf("  IFC   Input full-scale mode\r\n");
    xil_printf("  DDC   Digital downconverter gain mode, not for full-bandwidth capture\r\n");
    xil_printf("  help  Print this menu\r\n");
    xil_printf("  back  Quit gain setting menu\r\n");

    while (1)
    {
        xil_printf("gain-cmd$: ");
        uart_get_line(line);

        strncpy(copy, line, sizeof(copy) - 1);
        copy[sizeof(copy) - 1] = '\0';

        token = strtok(copy, " ");

        if (!token)
            continue;

        if (strcmp(token, "back") == 0 || strcmp(token, "quit") == 0 || strcmp(token, "exit") == 0)
        {
            xil_printf("Leaving ADC Gain setting menu.\r\n");
            return;
        }

        else if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0)
        {
            xil_printf("\r\nADC Gain setting menu\r\n");
            xil_printf("Available commands:\r\n");
            xil_printf("  IFC   Input full-scale mode\r\n");
            xil_printf("  DDC   Digital downconverter gain mode, not used for current full-bandwidth capture\r\n");
            xil_printf("  back  Quit gain setting menu\r\n");
        }

        else if (strcmp(token, "IFC") == 0 || strcmp(token, "ifc") == 0)
        {
            xil_printf("\r\nGain setting IFC mode\r\n");
            xil_printf("Input full-scale control changes ADC sensitivity using register 0x1910.\r\n");
            xil_printf("Smaller full-scale voltage gives larger digital sample amplitude.\r\n");
            xil_printf("\r\nAvailable commands:\r\n");
            xil_printf("  set <num>   Set input full-scale value\r\n");
            xil_printf("              Range: 1.36 largest amplitude  -->  2.04 smallest amplitude\r\n");
            xil_printf("              Valid values: 1.36, 1.47, 1.59, 1.70, 1.81, 1.93, 2.04\r\n");
            xil_printf("  status      Check current input full-scale status\r\n");
            xil_printf("  back        Back to gain mode selection\r\n");
            xil_printf("  quit        Quit gain setting menu\r\n");
            xil_printf("  sweep       Run sweep test across the IFC range\r\n");

            while (1)
            {
                xil_printf("gain-ifc$: ");
                uart_get_line(line);

                strncpy(copy, line, sizeof(copy) - 1);
                copy[sizeof(copy) - 1] = '\0';

                token = strtok(copy, " ");

                if (!token)
                    continue;

                if (strcmp(token, "back") == 0 || strcmp(token, "exit") == 0)
                {
                    xil_printf("Back to gain mode selection.\r\n");
                    break;
                }

                else if (strcmp(token, "quit") == 0)
                {
                    xil_printf("Leaving ADC Gain setting menu.\r\n");
                    return;
                }

                else if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0)
                {
                    xil_printf("\r\nGain setting IFC mode\r\n");
                    xil_printf("Available commands:\r\n");
                    xil_printf("  set <num>   Set input full-scale value\r\n");
                    xil_printf("              Range: 1.36 largest amplitude  -->  2.04 smallest amplitude\r\n");
                    xil_printf("              Valid values: 1.36, 1.47, 1.59, 1.70, 1.81, 1.93, 2.04\r\n");
                    xil_printf("  status      Check current input full-scale status\r\n");
                    xil_printf("  back        Back to gain mode selection\r\n");
                    xil_printf("  quit        Quit gain setting menu\r\n");
                    xil_printf("  sweep       Sweep all supported IFC values\r\n");
                }

                else if (strcmp(token, "status") == 0)
                {
                    ad9695_print_input_full_scale_status();
                }

                else if (strcmp(token, "set") == 0)
                {
                    token = strtok(NULL, " ");

                    if (!token)
                    {
                        xil_printf("Missing value. Example: set 1.59\r\n");
                    }
                    else
                    {
                        ad9695_set_input_full_scale(token);
                    }
                } 
                
                else if (strcmp(token, "sweep") == 0)
                {
                    adc_ifc_sweep();
                }

                else
                {
                    xil_printf("Invalid IFC command. Use set <num>, status, back, quit, or help.\r\n");
                }
            }
        }

        else if (strcmp(token, "DDC") == 0 || strcmp(token, "ddc") == 0)
        {
            xil_printf("\r\nGain setting DDC mode\r\n");
            xil_printf("DDC gain controls digital downconverter gain.\r\n");
            xil_printf("It is not used for the current full-bandwidth ADC capture path.\r\n");
            xil_printf("Use IFC mode for now.\r\n");
        }

        else
        {
            xil_printf("Invalid gain command. Use IFC, DDC, back, or help.\r\n");
        }
    }
}

void handle_adc_offset_cmd(void)
{
    char line[MAX_UART_LINE_LENGTH];
    char copy[MAX_UART_LINE_LENGTH];
    char *token;

    xil_printf("\r\nEntering ADC DC Offset Calibration menu\r\n");
    xil_printf("DC offset calibration removes the average DC bias from the ADC output.\r\n");
    xil_printf("Correction range is approximately +/-512 ADC codes.\r\n");
    xil_printf("\r\nAvailable commands:\r\n");
    xil_printf("  on        Enable DC offset calibration\r\n");
    xil_printf("  off       Disable DC offset calibration\r\n");
    xil_printf("  status    Check current calibration status\r\n");
    xil_printf("  help      Print this menu\r\n");
    xil_printf("  back      Return to UART command prompt\r\n");

    while (1)
    {
        xil_printf("offset-cmd$: ");
        uart_get_line(line);

        strncpy(copy, line, sizeof(copy) - 1);
        copy[sizeof(copy) - 1] = '\0';

        token = strtok(copy, " ");

        if (!token)
            continue;

        if (strcmp(token, "back") == 0 || strcmp(token, "exit") == 0)
        {
            xil_printf("Leaving ADC DC Offset Calibration menu.\r\n");
            return;
        }

        else if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0)
        {
            xil_printf("\r\nADC DC Offset Calibration menu\r\n");
            xil_printf("Available commands:\r\n");
            xil_printf("  on        Enable DC offset calibration\r\n");
            xil_printf("  off       Disable DC offset calibration\r\n");
            xil_printf("  status    Check current calibration status\r\n");
            xil_printf("  help      Print this menu\r\n");
            xil_printf("  back      Return to UART command prompt\r\n");
        }

        else if (strcmp(token, "on") == 0)
        {
            ad9695_adc_set_dc_offset_filt_en(1);
            xil_printf("DC offset calibration enabled.\r\n");
        }

        else if (strcmp(token, "off") == 0)
        {
            ad9695_adc_set_dc_offset_filt_en(0);
            xil_printf("DC offset calibration disabled.\r\n");
        }

        else if (strcmp(token, "status") == 0)
        {
            uint8_t r701;
            uint8_t r73b;

            ad9695_read_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL, &r701);
            ad9695_read_register(&spi_inst, AD9695_DC_OFFSET_CAL_CTRL2, &r73b);

            xil_printf("\r\nDC Offset Calibration Status\r\n");

            if ((r701 & AD9695_DC_OFFSET_CAL_EN) &&
                ((r73b & AD9695_DC_OFFSET_CAL_EN) == 0))
            {
                xil_printf("Status           : ON\r\n");
            }
            else
            {
                xil_printf("Status           : OFF\r\n");
            }

            xil_printf("Register 0x0701  : 0x%02X\r\n", r701);
            xil_printf("Register 0x073B  : 0x%02X\r\n", r73b);
        }

        else
        {
            xil_printf("Invalid offset command. Use on, off, status, help, or back.\r\n");
        }
    }
}

static int adc_capture_frame(void)
{
    int res;
    u32 timeout;
    u32 status;

    /*
     * Match the working manual sequence:
     *   dma -d
     *   wait
     *   dma -w
     */
    xil_printf("Resetting DMA...\r\n");

    XAxiDma_Reset(&dma_inst);

    timeout = 1000000;

    while (!XAxiDma_ResetIsDone(&dma_inst))
    {
        if (--timeout == 0)
        {
            ERR("DMA reset timeout.");
            return XST_FAILURE;
        }

        usleep(1);
    }

    /*
     * The manual command has a natural delay before dma -w.
     * Give the DMA hardware time to settle after reset.
     */
    usleep(100000);  /* 100 ms */

    status = XAxiDma_ReadReg(
        dma_inst.RegBase,
        XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET
    );

    xil_printf("DMA status after reset: 0x%08X\r\n", status);

    /*
     * Prepare destination buffer.
     */
    Xil_DCacheFlushRange(
        (UINTPTR)RxBufferPtr,
        DMA_CMD_BUF_SIZE
    );

    xil_printf("Starting DMA capture of %d bytes...\r\n",
               DMA_CMD_BUF_SIZE);

    res = XAxiDma_SimpleTransfer(
        &dma_inst,
        (UINTPTR)RxBufferPtr,
        DMA_CMD_BUF_SIZE,
        XAXIDMA_DEVICE_TO_DMA
    );

    if (res != XST_SUCCESS)
    {
        status = XAxiDma_ReadReg(
            dma_inst.RegBase,
            XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET
        );

        ERR(
            "XAxiDma_SimpleTransfer failed. Error code: %d, "
            "S2MM status: 0x%08X",
            res,
            status
        );

        return XST_FAILURE;
    }

    /*
     * Wait for the capture to finish.
     */
    timeout = 1000000;

    while (XAxiDma_Busy(&dma_inst, XAXIDMA_DEVICE_TO_DMA))
    {
        if (--timeout == 0)
        {
            status = XAxiDma_ReadReg(
                dma_inst.RegBase,
                XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET
            );

            ERR("DMA capture timeout. S2MM status: 0x%08X", status);
            return XST_FAILURE;
        }

        usleep(1);
    }

    Xil_DCacheInvalidateRange(
        (UINTPTR)RxBufferPtr,
        DMA_CMD_BUF_SIZE
    );

    xil_printf("DMA capture complete.\r\n");

    return XST_SUCCESS;
}

static void adc_ifc_sweep(void)
{
    static const char *ifc_values[] =
    {
        "2.04",
        "1.93",
        "1.81",
        "1.70",
        "1.59",
        "1.47",
        "1.36"
    };

    const int number_of_steps =
        sizeof(ifc_values) / sizeof(ifc_values[0]);

    int successful_captures = 0;
    int transmitted_frames = 0;

    /*
     * Prevent manual DMA or UDP commands from interfering with
     * the automatic sweep.
     */
    adc_sweep_active = 1;

    xil_printf("\r\n");
    xil_printf("===============================\r\n");
    xil_printf("Starting Input Full-Scale Sweep\r\n");
    xil_printf("===============================\r\n");

    for (int i = 0; i < number_of_steps; i++)
    {
        xil_printf("----------------------------------\r\n");
        xil_printf(
            "Step %d of %d\r\n",
            i + 1,
            number_of_steps
        );

        xil_printf(
            "Input Full-Scale : %s Vpp\r\n",
            ifc_values[i]
        );

        /*
         * Program the AD9695 input full-scale register.
         */
        ad9695_set_input_full_scale(ifc_values[i]);

        /*
         * Allow the ADC analog and digital datapaths to settle.
         */
        usleep(200000);  /* 200 ms */

        /*
         * Reset DMA, capture one frame, wait until complete,
         * and invalidate the data cache.
         */
        if (adc_capture_frame() != XST_SUCCESS)
        {
            xil_printf(
                "Capture failed for %s Vpp.\r\n",
                ifc_values[i]
            );

            continue;
        }

        successful_captures++;

        /*
         * adc_capture_frame() already invalidates the cache after
         * DMA completion. Repeating it here is harmless and ensures
         * udp_send_mem() reads the newest samples from DDR.
         */
        Xil_DCacheInvalidateRange(
            (UINTPTR)RxBufferPtr,
            DMA_CMD_BUF_SIZE
        );

        xil_printf("Transmitting frame...\r\n");

        /*
         * Use the exact UDP function already proven to work.
         *
         * Do not set uart_send_flag and wait for it. The flag is
         * normally processed by udp_update() in the main loop, but
         * the sweep blocks that main loop until it returns.
         */
        udp_send_mem();

        transmitted_frames++;

        xil_printf(
            "Transmission complete for %s Vpp.\r\n",
            ifc_values[i]
        );

        /*
         * Small separation between complete frames so the Python
         * receiver can finish storing the current frame.
         */
        usleep(100000);  /* 100 ms */
    }

    adc_sweep_active = 0;

    xil_printf("\r\n");
    xil_printf("=================================\r\n");
    xil_printf("IFC sweep finished.\r\n");

    xil_printf(
        "Successful captures     : %d/%d\r\n",
        successful_captures,
        number_of_steps
    );

    xil_printf(
        "Transmitted frames      : %d/%d\r\n",
        transmitted_frames,
        number_of_steps
    );

    xil_printf("=================================\r\n");
}