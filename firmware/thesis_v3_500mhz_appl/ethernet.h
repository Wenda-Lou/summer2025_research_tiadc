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
#include <stddef.h>
#include <stdint.h>

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

#define NUM_OF_TX 8 //32 k byte

#define SERVER_PORT 6666 //For netAssist -> 5001 For python script -> 5002

extern struct netif server_netif; //Make it can be seen by other .c files

typedef enum {
    CALIBRATION_CSV_TIMING_CAPTURES = 0,
    CALIBRATION_CSV_BASELINE_CAPTURES,
    CALIBRATION_CSV_OFFSET_CAPTURES,
    CALIBRATION_CSV_OFFSET_ITERATIONS,
    CALIBRATION_CSV_GAIN_CAPTURES,
    CALIBRATION_CSV_GAIN_ITERATIONS,
    CALIBRATION_CSV_SKEW_CAPTURES,
    CALIBRATION_CSV_SKEW_ITERATIONS,
    CALIBRATION_CSV_PERFORMANCE
} calibration_csv_dataset_t;

int  lwIP_UDP_init(void);
void udp_send_mem(void);
int udp_send_calibration_csv(const uint8_t *data, size_t length);
int udp_send_calibration_csv_dataset(
    calibration_csv_dataset_t dataset,
    const uint8_t *data,
    size_t length);
void udp_update(void);
void udp_service_calibration(void);



#endif
