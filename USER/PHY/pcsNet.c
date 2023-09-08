
#include "phy.h"

uint8_t PcsData0[6] ={0x01,0x03,0x10,0x01,0x00,0x09};
uint8_t PcsData1[6] ={0x01,0x03,0x10,0x38,0x00,0x28};
uint8_t PcsData2[6] ={0x01,0x03,0x20,0x13,0x00,0x0C};
uint8_t AmmeterData0[6] ={0x05,0x03,0x01,0x64,0x00,0x23};
uint8_t AmmeterData1[6] ={0x05,0x03,0x00,0x8D,0x00,0x02};

extern Struct_InvParameter Para;
extern Struct_SystemFault Sys_Fault;
extern struct_PCSDATA PcsData;
extern uint8_t quick_runstop;

int g_sock = -1;
#define PCS_IP_ADDR   "192.168.60.31"
uint8_t p_lwip_recvbuf[MBTCP_RCVMAX]; 

server_info linkPcs;

/*******************************************************************************/
void  PcsPupDeal(void)
{
	  if(Para.PcsStatus == PCS_CONSTANT_VOLTAGE_RUN ||\
			 Para.PcsStatus == PCS_CONSTANT_CURRENT_RUN ||\
		   Para.PcsStatus == PCS_OFF_GRID_INVERTER_RUN ||\
				Para.PcsStatus == 7u ||\
				Para.PcsStatus == PCS_AC_CONSTANT_POWER_OPERATION)                  
    {
			 SetMppt(MPPT_POWER_ON);                                        // 开MPPT
			 SysControl.SysFlag = STA_RUN;
    }
	  else if(SysControl.WaitCnt < 2)                                   // 启动失败
	  {
			 SysControl.SysFlag = STA_FAILURE;
		}
}

 

/*******************************************************************************
* Function Name  : void  SetPcs(uint8_t Oder)
* Description    : PCS设置处理
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetPcs(uint8_t Oder)
{
//	if( PcsData.PcsWriteF == PCS_DATA_RESTART_SEND)      // 0 重新启动发送
//	{
	 	PcsData.PcsWriteF = PCS_DATA_START_SENDING;        // 1发送准备,应该进入step6了,不然反应很慢
	  PcsData.PcsWriteO = Oder;                          // 指令编号
//	}
}

void PcsDatGet(uint8_t* dat , uint16_t len)
{
    
    switch(PcsData.PcsModbusSta)
    { 
        case PCS_DATD_READ_STEP1:
        {
             if((dat[0] == 1) && (dat[1] == 3))   // 9个数据读取完毕
             {
                 Para.PcsStatus = ByteCom(dat[3],dat[4]);    // 设备状态
                 Para.PcsWarn   = ByteCom(dat[5],dat[6]);    // 告警代码
                 Para.PcsFault  = ByteCom(dat[7],dat[8]);    // 故障代码
                 Para.PcsDcVH   = ByteCom(dat[9],dat[10]);   // 直流电压高16位
                 Para.PcsDcVL   = ByteCom(dat[11],dat[12]);  // 直流电压低16位  420700a4
                 Para.PcsDcCH   = ByteCom(dat[13],dat[14]);  // 直流电流高16位
                 Para.PcsDcCL   = ByteCom(dat[15],dat[16]);  // 直流电流低16位
                 Para.PcsDcPH   = ByteCom(dat[17],dat[18]);  // 直流功率高16位
                 Para.PcsDcPL   = ByteCom(dat[19],dat[20]);  // 直流功率低16位
                 PcsData.PcsModbusSta = PCS_DATD_READ_STEP2;
                 if(Para.PcsFault != 0)
                 {
                         Sys_Fault.pcs_fault = PCS_SYSTEM_FAULT;
                 }
                  else
                 {
                         Sys_Fault.pcs_fault = PCS_SYSTEM_NO_FAULT;// 是在这里清除PCS故障标志还是发出清除故障命令后再清除
                 }
             }
            PcsData.PcsModbusSta = PCS_DATD_READ_STEP2;
             break;
       }
        case PCS_DATD_READ_STEP3:
        {    
             if((dat[0] == 1) && (dat[1] == 3))  // 数据读取完毕
             {
                Para.PcsVuH = ByteCom(dat[3],dat[4]);        // 电网U相电压高16位
                Para.PcsVuL = ByteCom(dat[5],dat[6]);        // 电网U相电压低16位
                Para.PcsVvH = ByteCom(dat[7],dat[8]);        // 电网V相电压高16位
                Para.PcsVvL = ByteCom(dat[9],dat[10]);       // 电网V相电压地16位
                Para.PcsVwH = ByteCom(dat[11],dat[12]);      // 电网W相电压高16位
                Para.PcsVwL = ByteCom(dat[13],dat[14]);      // 电网W相电压低16位
                Para.PcsCuH = ByteCom(dat[15],dat[16]);      // 电网U相电流高16位
                Para.PcsCuL = ByteCom(dat[17],dat[18]);      // 电网U相电流低16位
                Para.PcsCvH = ByteCom(dat[19],dat[20]);      // 电网V相电流高16位
                Para.PcsCvL = ByteCom(dat[21],dat[22]);      // 电网V相电流低16位
                
                Para.PcsCwH = ByteCom(dat[23],dat[24]);      // 电网W相电流高16位
                Para.PcsCwL = ByteCom(dat[25],dat[26]);      // 电网W相电流低16位
                Para.PcsPH  = ByteCom(dat[27],dat[28]);      // 系统有功功率高16位
                Para.PcsPL  = ByteCom(dat[29],dat[30]);      // 系统有功功率低16位
                Para.PcsQH  = ByteCom(dat[31],dat[32]);      // 系统无功功率高16位
                Para.PcsQL  = ByteCom(dat[33],dat[34]);      // 系统无功功率低16位
                Para.PcsSH  = ByteCom(dat[35],dat[36]);      // 系统视在功率高16位
                Para.PcsSL  = ByteCom(dat[37],dat[38]);      // 系统视在功率低16位
                Para.PcsLoadVuH = ByteCom(dat[39],dat[40]);  // 负载U相电压高16位
                Para.PcsLoadVuL = ByteCom(dat[41],dat[42]);  // 负载U相电压低16位
                
                Para.PcsLoadVvH = ByteCom(dat[43],dat[44]);  // 负载V相电压高16位
                Para.PcsLoadVvL = ByteCom(dat[45],dat[46]);  // 负载V相电压低16位
                Para.PcsLoadVwH = ByteCom(dat[47],dat[48]);  // 负载W相电压高16位
                Para.PcsLoadVwL = ByteCom(dat[49],dat[50]);  // 负载W相电压低16位
                Para.PcsLoadCuH = ByteCom(dat[51],dat[52]);  // 负载U相电流高16位
                Para.PcsLoadCuL = ByteCom(dat[53],dat[54]);  // 负载U相电流低16位
                Para.PcsLoadCvH = ByteCom(dat[55],dat[56]);  // 负载V相电流高16位
                Para.PcsLoadCvL = ByteCom(dat[57],dat[58]);  // 负载V相电流低16位
                Para.PcsLoadCwH = ByteCom(dat[59],dat[60]);  // 负载W相电流高16位
                Para.PcsLoadCwL = ByteCom(dat[61],dat[62]);  // 负载W相电流低16位
                
                Para.PcsLoadPH  = ByteCom(dat[63],dat[64]);  // 负载有功功率高16位
                Para.PcsLoadPL  = ByteCom(dat[65],dat[66]);  // 负载有功功率低16位
                Para.PcsLoadQH  = ByteCom(dat[67],dat[68]);  // 负载无功功率高16位
                Para.PcsLoadQL  = ByteCom(dat[69],dat[70]);  // 负载无功功率低16位
                Para.PcsLoadSH  = ByteCom(dat[71],dat[72]);  // 负载视在功率高16位
                Para.PcsLoadSL  = ByteCom(dat[73],dat[74]);  // 负载视在功率低16位
                // PF  ByteCom(dat[75],dat[76])
                // 频率 ByteCom(dat[77],dat[78])  
                Para.PcsIgbtIntakeTempe = ByteCom(dat[79],dat[80]);  // 模块进风口温度 
                Para.PcsIgbtOutletTemp  = ByteCom(dat[81],dat[82]);  // 模块出风口温度
                PcsData.PcsModbusSta = PCS_DATD_READ_STEP4; 
             break;
          }
         }        

            
         case PCS_DATD_READ_STEP5:
         {                            
                 if((dat[0] == 1) && (dat[1] == 3))//数据读取完毕
                 {
                    Para.PcsModeH  = ByteCom(dat[3],dat[4]);       // 工作模式高16位
                    Para.PcsModeL  = ByteCom(dat[5],dat[6]);       // 工作模式低16位 
                    Para.PcsPara1H = ByteCom(dat[7],dat[8]);       // 设置参数1高16位
                    Para.PcsPara1L = ByteCom(dat[9],dat[10]);      // 设置参数1低16位
                    Para.PcsPara2H = ByteCom(dat[11],dat[12]);     // 设置参数2高16位
                    Para.PcsPara2L = ByteCom(dat[13],dat[14]);     // 设置参数2低16位
                    Para.PcsPara3H = ByteCom(dat[15],dat[16]);     // 设置参数3高16位
                    Para.PcsPara3L = ByteCom(dat[17],dat[18]);     // 设置参数3低16位
                    Para.PcsPara4H = ByteCom(dat[19],dat[20]);     // 设置参数4高16位
                    Para.PcsPara4L = ByteCom(dat[21],dat[22]);     // 设置参数4低16位
                    Para.PcsStartStop  = ByteCom(dat[23],dat[24]); // 启动/停机命令
                    Para.PcsFaultClear = ByteCom(dat[25],dat[26]); // 故障状态清除命令
                    PcsData.PcsModbusSta  = PCS_DATD_READ_STEP6;
                   break;
            }

      } 
        default:
        {
                //PcsData.PcsModbusSta  = PCS_DATD_READ_STEP0;
                break;
        }
  }

}
void PcsDatPut(void)
{
    uint8_t  PcsWriteBuff[40];
    switch(PcsData.PcsModbusSta)
    {
          case PCS_DATD_READ_STEP0:
          {
               MBTCPSend(&linkPcs,PcsData0,(uint16_t)sizeof(PcsData0));
               PcsData.PcsModbusWait = TOUT;
               PcsData.PcsModbusSta = PCS_DATD_READ_STEP1;
               while(linkPcs.Reissued == 0)
               {
                    if(PcsData.PcsModbusWait <= 0)
                    {
                    PcsData.PcsModbusSta = PCS_DATD_READ_STEP0;
                        return ;
                    }
               }
               linkPcs.Reissued = 0;
               break;
          }
          case PCS_DATD_READ_STEP2:
          {
               MBTCPSend(&linkPcs,PcsData1,(uint16_t)sizeof(PcsData1)); // 发送03功能码,读地址1038开始的40个数据
               PcsData.PcsModbusWait = TOUT;

               PcsData.PcsModbusSta = PCS_DATD_READ_STEP3;
               while(linkPcs.Reissued == 0)
               {
                    if(PcsData.PcsModbusWait <= 0)
                    {
                    PcsData.PcsModbusSta = PCS_DATD_READ_STEP2;
                        return ;
                    }
               }
               linkPcs.Reissued = 0;
               break;
          }

          case PCS_DATD_READ_STEP4:
          {
               MBTCPSend(&linkPcs,PcsData2,(uint16_t)sizeof(PcsData2)); // 发送03功能码,读地址2013开始的12个数据
               PcsData.PcsModbusWait = TOUT;

               
               PcsData.PcsModbusSta = PCS_DATD_READ_STEP5;
                              while(linkPcs.Reissued == 0)
               {
                    if(PcsData.PcsModbusWait <= 0)
                    {
                    PcsData.PcsModbusSta = PCS_DATD_READ_STEP4;
                        return ;
                    }
               }
               linkPcs.Reissued = 0;
               break;
          }
         case PCS_DATD_READ_STEP6:
         {
             if(quick_runstop == 0x05f)
              {
                   PcsWriteBuff[0] = 0x01;
                   PcsWriteBuff[1] = 0x06;
                   PcsWriteBuff[2] = 0x20;
                   PcsWriteBuff[3] = 0x1D;
                   PcsWriteBuff[4] = PcsData.Value[10] >> 8;
                   PcsWriteBuff[5] = PcsData.Value[10];
                   MBTCPSend(&linkPcs,PcsWriteBuff,6);
                   if(PcsData.Value[10] == PCS_POWER_ON)
                   {
                      SysControl.SysFlag = STA_PCSCHECK;// PCS开机完成确认
                   }
                   quick_runstop = 0;
              }
              else if(PcsData.PcsWriteF == PCS_DATA_START_SENDING)// 1 开始写入
              {
                   if(PcsData.PcsWriteO == PCS_MODE_INVERTER)//  0号指令，即设置模式及其他参数
                   {
                        PcsWriteBuff[0] = 0x01;
                        PcsWriteBuff[1] = 0x10;// 必须是10命令码
                        PcsWriteBuff[2] = 0x20;// 地址2013  设置工作模式及参数
                        PcsWriteBuff[3] = 0x13;
                        PcsWriteBuff[4] = 0x00;
                        PcsWriteBuff[5] = 0x0A;
                        PcsWriteBuff[6] = 0x14;//共发送20个字节
                        PcsWriteBuff[7]  = PcsData.Value[0]>>8;
                        PcsWriteBuff[8]  = PcsData.Value[0];
                        PcsWriteBuff[9]  = PcsData.Value[1]>>8;
                        PcsWriteBuff[10] = PcsData.Value[1];       // 工作模式
                       
                        PcsWriteBuff[11] = PcsData.Value[2]>>8;
                        PcsWriteBuff[12] = PcsData.Value[2];      
                        PcsWriteBuff[13] = PcsData.Value[3]>>8;
                        PcsWriteBuff[14] = PcsData.Value[3];       // 参数1
                       
                        PcsWriteBuff[15] = PcsData.Value[4]>>8;
                        PcsWriteBuff[16] = PcsData.Value[4];
                        PcsWriteBuff[17] = PcsData.Value[5]>>8;
                        PcsWriteBuff[18] = PcsData.Value[5];       // 参数2
                        
                        PcsWriteBuff[19] = PcsData.Value[6]>>8;
                        PcsWriteBuff[20] = PcsData.Value[6];
                        PcsWriteBuff[21] = PcsData.Value[7]>>8;
                        PcsWriteBuff[22] = PcsData.Value[7];       // 参数3
                        
                        PcsWriteBuff[23] = PcsData.Value[8]>>8;
                        PcsWriteBuff[24] = PcsData.Value[8];
                        PcsWriteBuff[25] = PcsData.Value[9]>>8;
                        PcsWriteBuff[26] = PcsData.Value[9];       // 参数4
                        
                        MBTCPSend(&linkPcs,PcsWriteBuff,27);
                   }
                   else if(PcsData.PcsWriteO == PCS_MODE_POWER_ON)          // 开关机  地址201D
                   {
                        PcsWriteBuff[0] = 0x01;
                        PcsWriteBuff[1] = 0x06;
                        PcsWriteBuff[2] = 0x20;
                        PcsWriteBuff[3] = 0x1D;
                        //PcsWriteBuff[4] = PcsData.Value[10] >> 8;
                        //PcsWriteBuff[5] = PcsData.Value[10];
                        PcsWriteBuff[4] = (uint8_t)(Para.OnOff >> 8);
                        PcsWriteBuff[5] = (uint8_t)Para.OnOff;
                        MBTCPSend(&linkPcs,PcsWriteBuff,6);
                        if(PcsData.Value[10] == PCS_POWER_ON)
                        {
                           SysControl.SysFlag = STA_PCSCHECK;               // PCS开机完成确认   
                        }                                      
                   }
                   else if(PcsData.PcsWriteO == PCS_MODE_FAULT_CLEAR)     // 故障清除/复位
                   { 
                        PcsWriteBuff[0] = 0x01;
                        PcsWriteBuff[1] = 0x06;
                        PcsWriteBuff[2] = 0x20;
                        PcsWriteBuff[3] = 0x1E;
                        PcsWriteBuff[4] = PcsData.Value[11] >>8;
                        PcsWriteBuff[5] = PcsData.Value[11];;

                        MBTCPSend(&linkPcs,PcsWriteBuff,6);
                   }
             }
                      PcsData.PcsModbusSta = PCS_DATD_READ_STEP0; //  PCS_DATD_READ_STEP0;
                break;
         }
           
         default:
           {
           
               break;
           }
      }
    
    //osSignalWait(0x1,TCPSIGNAL_TIMEOUT);
}


void PcsPhyTask(void const *argument)
{
    struct sockaddr_in pcs_client_addr;
    err_t err;
    int recv_data_len;
    struct timeval tv_out;
    tv_out.tv_sec = 5;
    tv_out.tv_usec = 0;
     for(;;)
     {
         sock_start:
                 linkPcs.g_lwip_connect_state = 0;
                 pcs_client_addr.sin_family = AF_INET;                   /* 表示IPv4网络协议 */
                 pcs_client_addr.sin_port = htons(NETPORT);       /* 端口号 */
                 pcs_client_addr.sin_addr.s_addr = inet_addr(PCS_IP_ADDR);   /* 远程IP地址 */
                 linkPcs.sock = socket(AF_INET, SOCK_STREAM, 0);                 /* 可靠数据流交付服务既是TCP协议 */
                 setsockopt(linkPcs.sock, SOL_SOCKET, SO_RCVTIMEO, &tv_out, sizeof(tv_out));
                 memset(&(pcs_client_addr.sin_zero), 0, sizeof(pcs_client_addr.sin_zero));
                 /* 连接远程IP地址 */
                 err = connect(linkPcs.sock, (struct sockaddr *)&pcs_client_addr, sizeof(struct sockaddr));
                 if (err == -1)
                 {
                     //printf("连接失败\r\n");
                     linkPcs.sock = -1;
                     closesocket(linkPcs.sock);
                     osDelay(10);
                     goto sock_start;
                 }
                 //printf("连接成功\r\n");
                 linkPcs.g_lwip_connect_state = 1;
                 
                 while (1)
                 {
                     recv_data_len = recv(linkPcs.sock,p_lwip_recvbuf,
                                          MBTCP_RCVMAX,0);
                     if (recv_data_len < 0 )
                     {
                         closesocket(linkPcs.sock);
                         linkPcs.sock = -1;
                         goto sock_start;
                     }
                     
                     /* 接收的数据 */ 
                     //NetEndianConver(p_lwip_recvbuf,recv_data_len);
                     
                     taskENTER_CRITICAL();
                     MBTCPRecv(&linkPcs,p_lwip_recvbuf,recv_data_len);
                     taskEXIT_CRITICAL();
                     //vTaskDelay(1);
                 }
             }

}




