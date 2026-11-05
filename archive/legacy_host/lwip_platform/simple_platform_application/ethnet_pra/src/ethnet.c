#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "netif/xadapter.h"
#include "lwip/init.h"
#include "lwip/tcp.h"
#include <lwip/err.h>
#include <lwip/ip4_addr.h>

/* Board MAC: same as lwIP echo server sample */
static unsigned char mac_address[6] = {0x00,0x0A,0x35,0x00,0x01,0x02};  /* Xilinx OUI + unique ID :contentReference[oaicite:1]{index=1} */

/* Static IPv4: 192.168.1.10/24, gateway 192.168.1.1 :contentReference[oaicite:2]{index=2} */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   1
#define IP_ADDR3   10

#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   1
#define GW_ADDR3   1

#define NETMASK0   255
#define NETMASK1   255
#define NETMASK2   255
#define NETMASK3   0

/* TCP server listens on port 5001 */
#define SERVER_PORT    5001

const char payload[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ\n"; /* 27 bytes */

struct netif server_netif;
struct tcp_pcb *client_pcb;
//A very crucial struct for lwIP, regulating multiple network connectors, including ethernet, WiFi etc
/*
The basic structure of netif struct
1. Hardware address : MAC address
2. IP addr / netmask / Gateway
3. Connector status
4. pointer to functions for Receive and Transmit
*/

//Called when new TCP connection is accepted 
err_t accept_CallBack(void *arg, struct tcp_pcb *new_pcb, err_t err)
{
    client_pcb = new_pcb;
    tcp_recv(new_pcb, NULL); //No receive call back needed
    tcp_err(new_pcb, NULL); //No error call back
    tcp_poll(new_pcb, NULL, 0); //No poll callback
    return ERR_OK;
}

int main(void)
{
    //declare crucial ip addr and pcb struct
    ip_addr_t ipaddr, netmask, gw;
    struct tcp_pcb *pcb; //Protcal Control Block

    //Enable caches for performance
    //Reduce memory latency significantly
    //Extremely important for high speed networking 
    Xil_ICacheEnable();
    Xil_DCacheEnable();
    xil_printf("I/D Cache initalized\n");

    //1. initialize lwIP stack
    xil_printf("lwIP Initializing\n");
    lwip_init();

    //2. Set up static IP addr
    xil_printf("Setting static IP / netmask / Gateway\n");
    IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP4_ADDR(&netmask, NETMASK0, NETMASK1, NETMASK2, NETMASK3);
    IP4_ADDR(&gw,      GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

    //3. Add network interface
    if(!xemac_add(&server_netif, &ipaddr, &netmask, &gw, mac_address, XPAR_GEM0_BASEADDR)) //GEM -> Gigabit Ethernet Module
    {
        xil_printf("Error registering netif\n");
        return 1;
    }

    netif_set_default(&server_netif); //Making this interface as the system's default route and bring it online
    netif_set_up(&server_netif);

    xil_printf("IP: %d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    xil_printf("GATEWAY: %d.%d.%d.%d\r\n", GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

    //4. Initialize TCP PCB and connect the block to server port
    pcb = tcp_new();
    if (tcp_bind(pcb, IPADDR_ANY, SERVER_PORT) != ERR_OK) 
    {     //IPADDR_ANY is equivalent to 0.0.0.0, that is any ip addr. TCP will be bind to any ip addr
        xil_printf("TCP bind fails\n");
        return 1;       
    }

    xil_printf("TCP bind successful\n");

    pcb = tcp_listen(pcb);// -> register this pcb instance in the listen PCB list
    tcp_accept(pcb, accept_CallBack);
    //register the callback function pointer into the listen PCB. Whenever there is a new connection, client PCB will be updated
    xil_printf("Listening the port %d \n", SERVER_PORT);

    while(1){
        //Polling lwIP and send data when connection exists
        xemacif_input(&server_netif); //Handle incoming & outgoing packets
         /* Search for active connections (done in callback function)*/

        //Sending Payload
        if (client_pcb && client_pcb->state == ESTABLISHED) {
            /* Repeatedly send our payload */
            tcp_write(client_pcb, payload, sizeof(payload)-1, TCP_WRITE_FLAG_COPY);  /* copy payload :contentReference[oaicite:3]{index=3} */
            tcp_output(client_pcb);  /* push it out */
            xil_printf("Sent payload to client\r\n");

            /* Simple delay loop */
            for (int i = 0; i < 10000000; i++) { __asm__("nop"); }        
        }
    }

    return 0;
}
