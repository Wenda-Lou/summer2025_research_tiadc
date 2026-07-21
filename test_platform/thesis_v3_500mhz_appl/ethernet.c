#include "ethernet.h"
#include "lwip/pbuf.h"
#include "ad9695_api.h"
#include "ad9695_registers.h"
#include "peripherals.h"
#include "xspips.h"
#include "xgpiops.h"
#include "xil_printf.h"
#include <lwip/netif.h>
#include <sleep.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>
#include <xemacps.h>
#include "bjesdlink.h"
#include "reference_buffer.h"
#include <string.h>

static unsigned char mac_address[6] = {0x00,0x0A,0x35,0x00,0x01,0x02};  /* Xilinx OUI + unique ID :contentReference[oaicite:1]{index=1} */

struct netif server_netif;

// //A very crucial struct for lwIP, regulating multiple network connectors, including ethernet, WiFi etc
// /*
// The basic structure of netif struct
// 1. Hardware address : MAC address
// 2. IP addr / netmask / Gateway
// 3. Connector status
// 4. pointer to functions for Receive and Transmit
// */

struct udp_pcb *udp_pcb_block;

extern uint8_t uart_send_flag; //Send flag enabled by the uart
extern uint8_t* RxBufferPtr;

#define REFERENCE_PACKET_HEADER_BYTES 8U
#define REFERENCE_PACKET_MAX_SAMPLES \
    ((512U - REFERENCE_PACKET_HEADER_BYTES) / sizeof(int16_t))

static uint16_t read_u16_le(const uint8_t *data);

static void handle_reference_packet(
    const uint8_t *data,
    uint16_t length
);

/* -------------------------------------------------------------------------------- */
/*  UDP receive callback: Output the receive parameters using uart                  */
/* -------------------------------------------------------------------------------- */
void recv_callback(
    void *arg,
    struct udp_pcb *pcb,
    struct pbuf *p,
    const ip_addr_t *addr,
    u16_t port
)
{
    uint8_t receive_buf[512];
    uint16_t copied_length;

    (void)arg;
    (void)pcb;
    (void)addr;
    (void)port;

    if (p == NULL)
    {
        return;
    }

    copied_length = (p->tot_len < sizeof(receive_buf))
        ? p->tot_len
        : sizeof(receive_buf);

    pbuf_copy_partial(
        p,
        receive_buf,
        copied_length,
        0U
    );

    if ((copied_length >= 4U) &&
        ((memcmp(receive_buf, "REFB", 4U) == 0) ||
         (memcmp(receive_buf, "REFD", 4U) == 0) ||
         (memcmp(receive_buf, "REFE", 4U) == 0) ||
         (memcmp(receive_buf, "REFC", 4U) == 0)))
    {
        handle_reference_packet(
            receive_buf,
            copied_length
        );
    }
    else
    {
        /*
         * Preserve the existing timing-delay command.
         */
        if (copied_length >= 4U)
        {
            uint8_t channel_idx = receive_buf[3];

            xil_printf(
                "Clk Mode: %x\r\n"
                "Fine delay steps: %u\r\n"
                "Super Fine delay steps: %u\r\n",
                receive_buf[0],
                receive_buf[1],
                receive_buf[2]
            );

            if (channel_idx >= 1U)
            {
                ad9695_adc_delay_mode(receive_buf[0]);

                ad9695_adc_set_channel_select(
                    channel_idx - 1U
                );

                ad9695_adc_fine_delay(receive_buf[1]);

                ad9695_adc_set_channel_select(
                    channel_idx - 1U
                );

                ad9695_adc_super_fine_delay(
                    receive_buf[2]
                );

                ad9695_adc_set_channel_select(2U);

                jesdlink_reset();
            }
        }
    }

    pbuf_free(p);
}

ip_addr_t ipaddr, netmask, gw;
ip_addr_t user_ip;

int lwIP_UDP_init(void)
{

    Xil_ICacheEnable();
    Xil_DCacheEnable();
    xil_printf("I/D Cache initialized\r\n");

    /* 1. lwIP stack init */
    xil_printf("lwIP initializing…\r\n");
    lwip_init();

    /* 2. Configure IP */
    IP4_ADDR(&ipaddr,  IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP4_ADDR(&netmask, NETMASK0, NETMASK1, NETMASK2, NETMASK3);
    IP4_ADDR(&gw,      GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

    IP4_ADDR(&user_ip, USR_IP_ADDR0, USR_IP_ADDR1, USR_IP_ADDR2, USR_IP_ADDR3);

    /* 3. Register netif (GEM0) */
    if (!xemac_add(&server_netif, &ipaddr, &netmask, &gw,
                   mac_address, 0xff0e0000)) {
        xil_printf("Error registering netif\r\n");
        return 1;
    }
    netif_set_default(&server_netif);
    netif_set_up(&server_netif);
    netif_set_link_up(&server_netif);

    xil_printf("IP      : %d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    xil_printf("Gateway : %d.%d.%d.%d\r\n", GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

    /* 4. Create & bind UDP PCB */
    udp_pcb_block = udp_new();
    if (udp_pcb_block == NULL) {
        xil_printf("udp_new() failed\r\n");
        return 1;
    }

    if(udp_bind(udp_pcb_block, IPADDR_ANY, SERVER_PORT) != ERR_OK){
        xil_printf("upd_bind failed\r\n");
        return 1;
    }

    udp_recv(udp_pcb_block, recv_callback, NULL);   //Register receive callback handler
    xil_printf("UDP server port %d\r\n", SERVER_PORT);
    
    xil_printf("UDP init successul\r\n");


    return 0;
}

//Loading the payload with 1024 byte from the memory and send to the client 
void udp_send_mem(void)
{   
    for (int i = 0; i < NUM_OF_TX; i++){
        //xil_printf("UDP sending Package #%d\r\n", i + 1);
         //Reallocate a new Packet buffer so that we do not accidentally change the data packet that is already inside the data frame
        struct pbuf *temp_packetBuffer = pbuf_alloc(PBUF_TRANSPORT, 512, PBUF_RAM); //Reallocate a pbuf of 512 bytes 

        if(!temp_packetBuffer){
            xil_printf("pbuf allocate failed\r\n");
            return;
        }


        //Fill Pbuf payload with contend from the memory 
        memcpy(temp_packetBuffer->payload, RxBufferPtr + 512 * i, 512);
        //memcpy(temp_packetBuffer->payload, RxBufferPtr, 512);

        //sending payload to the client
        if(udp_sendto(udp_pcb_block, temp_packetBuffer, &user_ip, SERVER_PORT) == ERR_OK){
            //xil_printf("UDP loaded and sent the payload with data from 0x%x to 0x%x to the client terminal\r\n", dma_rx_base_ptr + 1024 * i, dma_rx_base_ptr + 1024 * i + 1024);
        } else {
            xil_printf("UDP sendto(_) failed\r\n");
            return;
        }

        //freeing the pbuf
        pbuf_free(temp_packetBuffer);        
    }
    xil_printf("UDP package sent successfully\r\n");

}

//This function is to start 
// void udp_connect()
// {

// }

//This function must be put into a while loop because the xemacif_input function must be
//repeatedly called so that new ethernet data frames can be accepted by the lwip platform
//Otherwise the data frames will block the RX channel of the platform and the RX queue of the EMAc RX intr
void udp_update(void)
{
    xemacif_input(&server_netif);
    if(uart_send_flag){
        xil_printf("UDP will start to send received DMA samples to the computer station\r\n");
        uart_send_flag = 0;
        udp_send_mem();        
    }
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] |
           ((uint16_t)data[1] << 8);
}

static void handle_reference_packet(
    const uint8_t *data,
    uint16_t length
)
{
    reference_buffer_status_t status;

    if ((data == NULL) || (length < 4U))
    {
        return;
    }

    if (memcmp(data, "REFB", 4U) == 0)
    {
        uint16_t total_samples;
        reference_buffer_format_t format = REFERENCE_FORMAT_ADC_RATE;

        if (length < 6U)
        {
            xil_printf("Invalid REFB packet.\r\n");
            return;
        }

        total_samples = read_u16_le(&data[4]);
        if (length >= 7U) {
            format = (reference_buffer_format_t)data[6];
        }

        status = reference_buffer_begin_with_format(total_samples, format);

        xil_printf(
            "Reference begin: %u samples, format %u, status %d\r\n",
            total_samples,
            (unsigned int)format,
            (int)status
        );

        return;
    }

    if (memcmp(data, "REFD", 4U) == 0)
    {
        uint16_t offset;
        uint16_t sample_count;
        uint32_t required_bytes;

        if (length < 8U)
        {
            xil_printf("Invalid REFD packet.\r\n");
            return;
        }

        offset = read_u16_le(&data[4]);
        sample_count = read_u16_le(&data[6]);

        required_bytes =
            8U + ((uint32_t)sample_count * sizeof(int16_t));

        if (length < required_bytes)
        {
            xil_printf(
                "Short REFD packet: got %u, expected %lu\r\n",
                length,
                (unsigned long)required_bytes
            );
            return;
        }

        if (sample_count > REFERENCE_PACKET_MAX_SAMPLES)
        {
            xil_printf(
                "REFD packet contains too many samples: %u (max %u).\r\n",
                sample_count,
                (unsigned int)REFERENCE_PACKET_MAX_SAMPLES
            );
            return;
        }

        /*
         * Do not cast the byte payload directly to int16_t*. The UDP byte
         * buffer is not guaranteed to have int16_t alignment on every target.
         */
        int16_t chunk_samples[REFERENCE_PACKET_MAX_SAMPLES];

        memcpy(
            chunk_samples,
            &data[REFERENCE_PACKET_HEADER_BYTES],
            (size_t)sample_count * sizeof(chunk_samples[0])
        );

        status = reference_buffer_write_chunk(
            offset,
            chunk_samples,
            sample_count
        );

        if (status != REFERENCE_BUFFER_OK)
        {
            xil_printf(
                "Reference chunk failed: offset %u, "
                "count %u, status %d\r\n",
                offset,
                sample_count,
                (int)status
            );
        }

        return;
    }

    if (memcmp(data, "REFE", 4U) == 0)
    {
        status = reference_buffer_finalize();

        if ((status == REFERENCE_BUFFER_OK) &&
            reference_buffer_is_ready())
        {
            const size_t sample_count = reference_buffer_length();
            const int16_t *samples = reference_buffer_data();

            xil_printf("\r\nReference uploaded successfully.\r\n");
            xil_printf(
                "Samples : %lu\r\n",
                (unsigned long)sample_count
            );

            if ((samples != NULL) && (sample_count > 0U))
            {
                xil_printf(
                    "First   : %d\r\n"
                    "Last    : %d\r\n",
                    (int)samples[0],
                    (int)samples[sample_count - 1U]
                );
            }

            xil_printf("Ready for calibration.\r\n");
        }
        else
        {
            xil_printf(
                "Reference finalize failed: status %d, "
                "ready %d, samples %lu\r\n",
                (int)status,
                reference_buffer_is_ready(),
                (unsigned long)reference_buffer_length()
            );
        }

        return;
    }

    if (memcmp(data, "REFC", 4U) == 0)
    {
        reference_buffer_clear();
        xil_printf("Reference cleared.\r\n");
    }
}

// int main(void)
// {
//     //declare crucial ip addr and pcb struct
//     ip_addr_t ipaddr, netmask, gw;
//     struct udp_pcb *pcb; //Protcal Control Block

//     //Enable caches for performance
//     //Reduce memory latency significantly
//     //Extremely important for high speed networking 
//     Xil_ICacheEnable();
//     Xil_DCacheEnable();
//     xil_printf("I/D Cache initalized\n");

//     //1. initialize lwIP stack
//     xil_printf("lwIP Initializing\n");
//     lwip_init();

//     //2. Set up static IP addr
//     xil_printf("Setting static IP / netmask / Gateway\n");
//     IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
//     IP4_ADDR(&netmask, NETMASK0, NETMASK1, NETMASK2, NETMASK3);
//     IP4_ADDR(&gw,      GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

//     //3. Add network interface
//     if(!xemac_add(&server_netif, &ipaddr, &netmask, &gw, mac_address, XPAR_GEM0_BASEADDR)) //GEM -> Gigabit Ethernet Module
//     {
//         xil_printf("Error registering netif\n");
//         return 1;
//     }

//     netif_set_default(&server_netif); //Making this interface as the system's default route and bring it online
//     netif_set_up(&server_netif);

//     xil_printf("IP: %d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
//     xil_printf("GATEWAY: %d.%d.%d.%d\r\n", GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

//     //4. Initialize udp PCB and connect the block to server port
//     pcb = udp_new();
//     if (udp_bind(pcb, IPADDR_ANY, SERVER_PORT) != ERR_OK) 
//     {     //IPADDR_ANY is equivalent to 0.0.0.0, that is any ip addr. udp will be bind to any ip addr
//         xil_printf("udp bind fails\n");
//         return 1;       
//     }

//     xil_printf("udp bind successful\n");

//     pcb = udp_listen(pcb);// -> register this pcb instance in the listen PCB list
//     udp_accept(pcb, accept_CallBack);
//     //register the callback function pointer into the listen PCB. Whenever there is a new connection, client PCB will be updated
//     xil_printf("Listening the port %d \n", SERVER_PORT);

//     while(1){
//         //Polling lwIP and send data when connection exists
//         xemacif_input(&server_netif); //Handle incoming & outgoing packets
//          /* Search for active connections (done in callback function)*/

//         //Sending Payload
//         if (client_pcb && client_pcb->state == ESTABLISHED) {
//             /* Repeatedly send our payload */
//             udp_write(client_pcb, payload, sizeof(payload)-1, udp_WRITE_FLAG_COPY);  /* copy payload :contentReference[oaicite:3]{index=3} */
//             udp_output(client_pcb);  /* push it out */
//             xil_printf("Sent payload to client\r\n");

//             /* Simple delay loop */
//             for (int i = 0; i < 10000000; i++) { __asm__("nop"); }        
//         }
//     }

//     return 0;
    

