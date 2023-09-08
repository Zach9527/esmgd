#include "phy.h"

server_info linkBsm;
extern Struct_InvParameter Para;

uint8_t bms_request[6] = {0x03,0x04,0x00,0x01,0x00,0x2f};
#define BMS_IP_ADDR   "192.168.60.40"
uint8_t b_lwip_recvbuf[MBTCP_RCVMAX]; 

void BsmDatPut(void)
{
    int8_t err;
    err = MBTCPSend(&linkBsm,bms_request,sizeof(bms_request));
    if(err < 0)
        {
        err = 0;
    }
}

void BsmDatGet(uint8_t* dat , uint16_t len)
{
     if((dat[0] == 3) && (dat[1] == 0x4))    
     {
         Para.BmsTotalVolt = ByteCom(dat[5],dat[6]);                   // 电池总电压
         Para.BmsTotalCurr = ByteCom(dat[7],dat[8]);                   // 电池总电流
         Para.BmsSoc       = ByteCom(dat[9],dat[10]);           //  SOC
         Para.BmsSoh       = ByteCom(dat[11],dat[12]);           //  SOH
         Para.BmsMaxVolt   = ByteCom(dat[13],dat[14]);       //  单体最高电压 
         Para.BmsMaxVoltCN = ByteCom(dat[15],dat[16]);     //  最高电压编号 
         
         Para.BmsMinVolt   = ByteCom(dat[19],dat[20]);       //  单体最低电压               
         Para.BmsMinVoltN  = ByteCom(dat[21],dat[22]);      //  最低电压编号   

         Para.BmsMaxTem    = ByteCom(dat[25],dat[26]);        //  单体最高温度 
         Para.BmsMaxTemCN  = ByteCom(dat[27],dat[28]);      //  最高温度编号
        
         Para.BmsMinTem    = ByteCom(dat[31],dat[32]);        //  单体最低温度
         Para.BmsMaxTemRN  = ByteCom(dat[33],dat[34]);      //  最低温度编号 
    }



}
void BsmPhyTask(void const *argument)
 {
     struct sockaddr_in bsm_client_addr;
     err_t err;
     int recv_data_len;
     struct timeval tv_out;
     tv_out.tv_sec = 5;
     tv_out.tv_usec = 0;
      for(;;)
      {
          sock_start:
                  linkBsm.g_lwip_connect_state = 0;
                  bsm_client_addr.sin_family = AF_INET;
                  bsm_client_addr.sin_port = htons(NETPORT); 
                  bsm_client_addr.sin_addr.s_addr = inet_addr(BMS_IP_ADDR);
                  linkBsm.sock = socket(AF_INET, SOCK_STREAM, 0);
                  setsockopt(linkBsm.sock, SOL_SOCKET, SO_RCVTIMEO, &tv_out, sizeof(tv_out));
                  memset(&(bsm_client_addr.sin_zero), 0, sizeof(bsm_client_addr.sin_zero));
                  err = connect(linkBsm.sock, (struct sockaddr *)&bsm_client_addr, sizeof(struct sockaddr));
                  if (err == -1)
                  {
                      linkBsm.sock = -1;
                      closesocket(linkBsm.sock);
                      osDelay(10);
                      goto sock_start;
                  }
                  linkBsm.g_lwip_connect_state = 1;
                  
                  while (1)
                  {
                      recv_data_len = recv(linkBsm.sock,b_lwip_recvbuf,
                                           MBTCP_RCVMAX,0);
                      if (recv_data_len < 0 )
                      {
                          closesocket(linkBsm.sock);
                          linkBsm.sock = -1;
                          goto sock_start;
                      }
                      
                      //NetEndianConver(g_lwip_recvbuf,recv_data_len);
                      
                      taskENTER_CRITICAL();
                      MBTCPRecv(&linkBsm,b_lwip_recvbuf,recv_data_len);
                      //vTaskDelay(1);
                      taskEXIT_CRITICAL();
                  }
              }
 
 }

