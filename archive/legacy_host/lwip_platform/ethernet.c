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
extern uint8_t* dma_rx_base_ptr;

/* -------------------------------------------------------------------------------- */
/*  UDP receive callback: Output the receive parameters using uart                  */
/* -------------------------------------------------------------------------------- */
void recv_callback(void *arg,
                          struct udp_pcb *pcb,
                          struct pbuf *p,
                          const ip_addr_t *addr,
                          u16_t port)
{
    uint8_t receive_buf[64] = {0x0}; //clock mode, fine delay, super fine delay
    xil_printf("\r\nUDP Packet received. Enter Callback function\r\n");
    /* Always free the incoming packet as soon as possible */
    if (p != NULL) {
        memcpy(receive_buf, p -> payload, sizeof(receive_buf));
        xil_printf("Clk Mode: %0x\r\nFine delay steps: %0d\r\nSuper Fine delay steps: %0d\r\n", receive_buf[0], receive_buf[1],receive_buf[2]); 
        uint8_t channel_idx = receive_buf[3] & 0xff;
        xil_printf("channel idx: %0x\r\n", channel_idx);
        ad9695_adc_delay_mode(receive_buf[0] & 0xff);
        ad9695_adc_set_channel_select(channel_idx - 1);
        ad9695_adc_fine_delay(receive_buf[1] & 0xff);
        ad9695_adc_set_channel_select(channel_idx - 1);
        ad9695_adc_super_fine_delay(receive_buf[2] & 0xff);
        ad9695_adc_set_channel_select(2);
        jesdlink_reset();
        pbuf_free(p);                          /* release RX pbuf */
    }
    xil_printf("uart-cmd$: ");
}

ip_addr_t ipaddr, netmask, gw;
ip_addr_t user_ip;

int lwIP_UDP_init()
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
void udp_send_mem()
{   
    for (int i = 0; i < NUM_OF_TX; i++){\
        //xil_printf("UDP sending Package #%d\r\n", i + 1);
         //Reallocate a new Packet buffer so that we do not accidentally change the data packet that is already inside the data frame
        struct pbuf *temp_packetBuffer = pbuf_alloc(PBUF_TRANSPORT, 1024, PBUF_RAM); //Reallocate a pbuf of 1024 bytes 

        if(!temp_packetBuffer){
            xil_printf("pbuf allocate failed\r\n");
            return;
        }


        //Fill Pbuf payload with contend from the memory 
        //memcpy(temp_packetBuffer->payload, dma_rx_base_ptr + 1024 * i, 1024);
        memcpy(temp_packetBuffer->payload, dma_rx_base_ptr, 512);

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
void udp_update()
{
    xemacif_input(&server_netif);
    if(uart_send_flag){
        xil_printf("UDP will start to send received DMA samples to the computer station\r\n");
        uart_send_flag = 0;
        udp_send_mem();        
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
    

