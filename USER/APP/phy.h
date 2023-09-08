#ifndef PHY_H
#define PHY_H
#include "main.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/ip.h"
#include <lwip/sockets.h>



#define PCS_IP_ADDR0               192
#define PCS_IP_ADDR1               168
#define PCS_IP_ADDR2                60
#define PCS_IP_ADDR3                20

#define BSM_IP_ADDR0               192
#define BSM_IP_ADDR1               168
#define BSM_IP_ADDR2                60
#define BSM_IP_ADDR3                40


#define MB_TCP_PID          2
#define MB_TCP_LEN          4
#define MB_TCP_FUNC         6

#define MB_DEV_ADDR         6
#define MB_TCP_PROTOCOL_ID  0  


#define TCP_PCS_ID          1
#define TCP_BSM_ID          3

#define MBTCP_SENTMAX 512

#define MBTCP_RCVMAX  256

#define L2B32(Little) (((Little & 0xff) << 24) | (((Little) & 0xff00) << 8) | (((Little) & 0xff0000) >> 8) | ((Little >> 24) & 0xff))


typedef struct SERVERINFO
{
    uint16_t Reissued;
    uint16_t tID;
    int g_lwip_connect_state;
    int sock;
    ip4_addr_t server_ipaddr;
}server_info;

void PcsDatPut(void);
void PcsDatGet(uint8_t* dat , uint16_t len);


int8_t MBTCPSend(server_info *info,uint8_t *pucFrame,uint16_t usLength);
void MBTCPRecv(server_info *info,uint8_t *pucFrame,uint16_t usLength);
void NetEndianConver(uint8_t *dat,uint16_t length);


void BsmDatPut(void);
void BsmDatGet(uint8_t* dat , uint16_t len);

void PcsPhyTask(void const *argument);
void BsmPhyTask(void const *argument);


#endif
