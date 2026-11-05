#ifndef __ETHERNET__
#define __ETHERNET__

#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "netif/xadapter.h"
#include "lwip/init.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include <lwip/err.h>
#include <lwip/ip4_addr.h>

/* Static IPv4: 192.168.1.10/24, gateway 192.168.1.1 */
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

#define USR_IP_ADDR0    192
#define USR_IP_ADDR1    168
#define USR_IP_ADDR2    1
#define USR_IP_ADDR3    100

#define NUM_OF_TX 32 //32 k byte

#define SERVER_PORT 5002 //For netAssist -> 5001 For python script -> 5002

extern struct netif server_netif; //Make it can be seen by other .c files

int lwIP_UDP_init();
void udp_send_mem();
void udp_update();



#endif