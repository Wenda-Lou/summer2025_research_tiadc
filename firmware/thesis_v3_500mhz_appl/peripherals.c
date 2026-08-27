/* peripherals.c
 * Unified implementation for PS GPIO, SPI, and UART helpers.
 * Direct merge of bgpiops.c, bspips.c, and buartps.c.
 * Logic and variable names have been preserved verbatim.
 */

#include "peripherals.h"
#include "ethernet.h"
#include "lwip/pbuf.h"

/* ============================ GPIO ============================ */
XGpioPs_Config *gpio_init(XGpioPs *gpio)
{
    XGpioPs_Config *config = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
    if (!config)
    {
        xil_printf("GPIO config lookup failed!\r\n");
        return NULL;
    }

    if (XGpioPs_CfgInitialize(gpio, config, config->BaseAddr) != XST_SUCCESS)
    {
        xil_printf("GPIO init failed!\r\n");
        return NULL;
    }

    /* MIO 22 & 23 */
    XGpioPs_SetDirectionPin(gpio, GPIO_SW19_PIN, GPIO_IN);
    XGpioPs_SetOutputEnablePin(gpio, GPIO_SW19_PIN, 0);

    XGpioPs_SetDirectionPin(gpio, GPIO_DS50_PIN, GPIO_OUT);
    XGpioPs_WritePin(gpio, GPIO_DS50_PIN, 0);
    XGpioPs_SetOutputEnablePin(gpio, GPIO_DS50_PIN, 1);

    /* EMIOs */
    XGpioPs_SetDirectionPin(gpio, GPIO_PWDN_PIN, GPIO_OUT);
    XGpioPs_WritePin(gpio, GPIO_PWDN_PIN, 0);
    XGpioPs_SetOutputEnablePin(gpio, GPIO_PWDN_PIN, 1);

    XGpioPs_SetDirectionPin(gpio, GPIO_FDA_PIN, GPIO_IN);
    XGpioPs_SetOutputEnablePin(gpio, GPIO_FDA_PIN, 0);

    XGpioPs_SetDirectionPin(gpio, GPIO_FDA_PIN, GPIO_IN);
    XGpioPs_SetOutputEnablePin(gpio, GPIO_FDA_PIN, 0);

    xil_printf("GPIO initialized successfully!\r\n");

    return config;
};

/* ============================= SPI ============================ */
void ad9695_read_register(XSpiPs *Spi, u16 reg_addr, u8 *data)
{
    u8 tx_buf[3], rx_buf[3];

    /* Construct 16‑bit read instruction (bit 15 = 1 for read) */
    tx_buf[0] = 0x80 | ((reg_addr >> 8) & 0x7F); /* R/W=1, bits 14–8 */
    tx_buf[1] = reg_addr & 0xFF;                 /* bits 7–0 */
    tx_buf[2] = 0x00;                            /* Dummy byte */

    XSpiPs_PolledTransfer(Spi, tx_buf, rx_buf, 3);

    *data = rx_buf[2]; /* Received data is in the third byte */
}

u8 ad9695_read_bit(XSpiPs *Spi, u16 reg_addr, u8 bit_pos)
{
    u8 data;

    /* Validate bit position */
    if (bit_pos > 7)
    {
        xil_printf("ERROR: read bit range can only be 0-7!\r\n");
        return 0xff; /* Invalid bit_pos */
    }
    ad9695_read_register(Spi, reg_addr, &data);
    return (data >> bit_pos) & 0x01;
}

void ad9695_write_register(XSpiPs *Spi, u16 reg_addr, u8 value)
{
    u8 tx_buf[3];

    /* Construct 16‑bit write instruction (bit 15 = 0 for write) */
    tx_buf[0] = (reg_addr >> 8) & 0x7F; /* R/W=0, bits 14–8 */
    tx_buf[1] = reg_addr & 0xFF;        /* bits 7–0 */
    tx_buf[2] = value;

    XSpiPs_PolledTransfer(Spi, tx_buf, NULL, 3);
}

XSpiPs_Config *spi_init(XSpiPs *spi)
{
    XSpiPs_Config *config = XSpiPs_LookupConfig(SPI_DEVICE_ID);
    if (!config)
    {
        xil_printf("SPI config lookup failed!\r\n");
        return NULL;
    }

    if (XSpiPs_CfgInitialize(spi, config, config->BaseAddress) != XST_SUCCESS)
    {
        xil_printf("SPI init failed!\r\n");
        return NULL;
    }

    /* SPI settings: Manual CS, Mode 0 (CPOL=0, CPHA=0), Master */
    XSpiPs_SetOptions(spi, XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION);
    XSpiPs_SetClkPrescaler(spi, XSPIPS_CLK_PRESCALE_16);
    XSpiPs_SetSlaveSelect(spi, 0); /* Select CS0 */

    xil_printf("SPI master initialized successfully.\r\n");

    return config;
}

/* ============================ UART ============================ */
extern XUartPs_Config *uart_config;

XUartPs_Config *uart_init(XUartPs *uart)
{
    XUartPs_Config *config = XUartPs_LookupConfig(UART0_DEVICE_ID);
    if (!config)
    {
        xil_printf("UART config lookup failed!\r\n");
        return NULL;
    }

    if (XUartPs_CfgInitialize(uart, config, config->BaseAddress) != XST_SUCCESS)
    {
        xil_printf("UART initialization failed!\r\n");
        return NULL;
    }

    XUartPs_SetBaudRate(uart, 115200);
    xil_printf("UART initialized successfully.\r\n");

    return config;
}

void uart_get_line(char *buffer)
{
    int i = 0;
    u8 c;

    xil_printf("uart-cmd$: ");

    while (1)
    {
        /* Wait until data is available */
        while (!XUartPs_IsReceiveData(uart_config->BaseAddress))
        {
            xemacif_input(&server_netif);
        }

        c = XUartPs_ReadReg(uart_config->BaseAddress, XUARTPS_FIFO_OFFSET);

        /* Enter key: CR or LF */
        if (c == '\r' || c == '\n')
        {
            xil_printf("\r\n");
            break;
        }

        /* Backspace or Delete */
        else if (c == 0x08 || c == 0x7F)
        {
            if (i > 0)
            {
                i--;
                buffer[i] = '\0';

                /* Move back, erase char, move back again */
                xil_printf("\b \b");
            }
        }

        /* Normal printable character */
        else if (c >= 32 && c <= 126)
        {
            if (i < MAX_UART_LINE_LENGTH - 1)
            {
                buffer[i++] = c;
                buffer[i] = '\0';

                /* Echo typed character */
                //xil_printf("%c", c);
            }
        }
    }

    buffer[i] = '\0';
}
