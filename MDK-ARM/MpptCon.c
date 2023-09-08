#include "main.h"


uint8_t MpptOn[8] ={0x2D,0x01,0x02,0x00,0x2E,0x01,0x05,0x00};//模块1开机
uint8_t MpptOff[8] ={0x2D,0x01,0x00,0x00,0x00,0x00,0x00,0x00};//所有关机
uint8_t MpptConf[8] = {0x45,0x01,0x2c,0x01,0x46,0x01,0x5,0x00};//DC模块限功率30kw
uint8_t MpptPara1[8] = {0xff,0xff,0x00,0x00,0xD1,0x00,0x01,0x00};
uint8_t MpptPara2[8] = {0xff,0xff,0x00,0x00,0xD5,0x00,0x01,0x00};

// PCS，AC功率为0；MPPT先共负载，MPPT功率大时，同时也共电池。
// MPPT功率不大时，负载功率大,电池也共负载

extern Struct_InvParameter Para;
extern Struct_SystemFault Sys_Fault;
extern struct_MPPT Mppt;
extern CAN_TxHeaderTypeDef Tx2Message;
extern uint32_t Tx2Mailbox;
extern uint8_t can2_rxdata[8],can2_txdata[8];
extern CAN_RxHeaderTypeDef CAN2_Header;
extern uint8_t mppt_state_back; 
extern Struct_FParameter ActualValue;
char MpptOnOff = 1;
/*******************************************************************************
* Function Name  : void  SetMppt(uint8_t Oder)
* Description    : Mppt设置处理
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetMppt(uint8_t Oder)
{
    //if( Mppt.MpptRx_End_Flag == 1)                // MPPT状态机接收完成
        {
        
        Tx2Message.IDE = CAN_ID_EXT;   //  扩展帧
            if(Oder == MPPT_POWER_ON)             // 开机(9A设置所有模块关机MpptData,1为关机) 
            {
                    
                    MpptOnOff = 1;
                    Tx2Message.IDE = CAN_ID_EXT;   //  扩展帧
                    Tx2Message.ExtId = DCDC_FRAME;
                    Tx2Message.DLC = 8;                                           // Increment transmit data
                    HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptOn, &Tx2Mailbox);
                    osDelay(20);
                    HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,MpptConf,&Tx2Mailbox);
            }
            else if(Oder == MPPT_POWER_OFF)       // 关机(9A设置所有模块关机MpptData1,0为开机)   
            {
                    if(MpptOnOff == 1)
                    {
                        Tx2Message.ExtId = DCDC_FRAME;
                        Tx2Message.DLC = 8;                                           // Increment transmit data
                        HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptOff, &Tx2Mailbox);
                        MpptOnOff = 0;
                     }
            }
            else if(Oder == MPPT_SET_POWER)                   // 设置Mppt参数 800V
            {// 02 9C 3F F0 00 0B 71 B0 00 00 3A98 所有模块都输出 750V 15A
                    Mppt.Curr = 1000 * Mppt.Pwr / (uint16_t)ActualValue.PcsDcVoltage;  // 单位A W/V
                if(Mppt.Curr > 15000)
                    {
                        Mppt.Curr = 15000;
                    }
                    Tx2Message.ExtId = DCDC_FRAME;                // (设置所有模块电压*1000,电流*1000)
                    
                    
                
                    can2_txdata[0] = 0x00;//Para.PcsDcVH>>8;
                    can2_txdata[1] = 0x0c;//Para.PcsDcVH;
                    can2_txdata[2] = 0x35;//Para.PcsDcVL>>8;
                    can2_txdata[3] = 0x00;//Para.PcsDcVL;
                    can2_txdata[4] = Mppt.Curr>>24;
                    can2_txdata[5] = Mppt.Curr>>16;
                    can2_txdata[6] = Mppt.Curr>>8;
                    can2_txdata[7] = Mppt.Curr;                         
                
                    Tx2Message.DLC = 8;                                           
                    HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,can2_txdata, &Tx2Mailbox);
            }
      //      Mppt.MpptRx_End_Flag = 0;
}

}
 #if 0
void MpptVoltGet(void)
{
    int channelnum = 0;
    Tx2Message.IDE = CAN_ID_EXT;   //  扩展帧
    Tx2Message.ExtId = DCDC_FRAME;
    Tx2Message.DLC = 8;// Increment transmit data
    for(channelnum = 0; channelnum < 6; channelnum++)
    {
        memset(can2_rxdata,0,8);
        memset(CAN2_RX_BUF,0,8);
        CAN2_Header.ExtId = 0;
        HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptPara2, &Tx2Mailbox);
        MpptPara2[DCDC_PGN] +=1;
        osSignalWait(0x01,osWaitForever);
        if()
        Para.MpptSingleV
    }
}
#endif
/*******************************************************************************
* Function Name  : void  MpptDataGet(void)
* Description    : Mppt数据读取  按充电模块 CAN 通讯协议 V1.08
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MpptDataGet(void)
{
    //      Mppt.SysSta = mppt_state_back;
    Tx2Message.IDE = CAN_ID_EXT;   //  扩展帧
    switch(Mppt.SysSta)
    {
        case MTTP_READ_STEP0:
        {
            MpptPara1[DCDC_PGN] = 0xD1;
            Tx2Message.ExtId = DCDC_FRAME;   // 读取系统信息
            memset(can2_rxdata,0,8);
            memset(CAN2_RX_BUF,0,8);
            CAN2_Header.ExtId = 0;
            Mppt.SysSta = MTTP_READ_STEP1;
            Tx2Message.DLC = 8;// Increment transmit data
            HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptPara1, &Tx2Mailbox);
            Mppt.TimeOut = MPPTTINT;
            while(Mppt.Reissued == 0)
            {
                if(Mppt.TimeOut <= 0)
                {
                    Mppt.SysSta = MTTP_READ_STEP0;
                    return;
                }
            }
            Mppt.Reissued = 0;
            break;
        }
        case MTTP_READ_STEP1:
        {
            CAN2_Header.ExtId= CAN2_rxHeader.ExtId;
            if(CAN2_Header.ExtId == DCDC_DATA_GET1 )               // 模块回复系统电压电流
            {
                Para.MpptDcBusVolt              = ByteCom(can2_rxdata[1],can2_rxdata[0]);  // 系统总电压高16位 
                Para.MpptDcBusCurr              = ByteCom(can2_rxdata[3],can2_rxdata[2]);  // 系统总电流高16位 
                Para.MpptDcPwr               = ByteCom(can2_rxdata[5],can2_rxdata[4]);  // 系统总电流低16位
                Para.MpptSingleT[DCDC_MODEL1] = ByteCom(can2_rxdata[7],can2_rxdata[6]);  // 模块IGBT1温度
                Mppt.SysSta = MTTP_READ_STEP2;
            }
            Mppt.MpptRx_End_Flag = 1;
            break;
        }
            
        case MTTP_READ_STEP2:
        {
            Tx2Message.ExtId = DCDC_FRAME;                  // 读取系统模块温度
            memset(can2_rxdata,0,8);
            memset(CAN2_RX_BUF,0,8);
            CAN2_Header.ExtId = 0;
            Mppt.SysSta = MTTP_READ_STEP3;
            Tx2Message.DLC = 8;                                           
            MpptPara1[DCDC_PGN] = 0xd2;
            HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptPara1, &Tx2Mailbox);
            Mppt.TimeOut = MPPTTINT;
            while(Mppt.Reissued == 0)
            {
                if(Mppt.TimeOut <= 0)
                {
                    Mppt.SysSta = MTTP_READ_STEP2;
                    return;
                }
            }
            Mppt.Reissued = 0;
            break;
        }
        
        case MTTP_READ_STEP3:
        {
            CAN2_Header.ExtId= CAN2_rxHeader.ExtId;
            if(CAN2_Header.ExtId == DCDC_DATA_GET2)               //  模块2-5温度
            {
                Para.MpptSingleT[DCDC_MODEL2] = ByteCom(can2_rxdata[1],can2_rxdata[0]);  // 模块IGBT2温度
                Para.MpptSingleT[DCDC_MODEL3] = ByteCom(can2_rxdata[3],can2_rxdata[2]);  // 模块IGBT3温度
                Para.MpptSingleT[DCDC_MODEL4] = ByteCom(can2_rxdata[5],can2_rxdata[4]);  // 模块IGBT4温度
                Para.MpptSingleT[DCDC_MODEL5] = ByteCom(can2_rxdata[7],can2_rxdata[6]);  // 模块IGBT5温度
                Mppt.SysSta = MTTP_READ_STEP4;
            }
            Mppt.MpptRx_End_Flag = 1;
            break;
        }
        
        case MTTP_READ_STEP4:
        {
            Tx2Message.ExtId = DCDC_FRAME;                       // 读取温度
            memset(can2_rxdata,0,8);
            memset(CAN2_RX_BUF,0,8);
            CAN2_Header.ExtId = 0;
            Mppt.SysSta = MTTP_READ_STEP5;
            Tx2Message.DLC = 8;
            MpptPara1[DCDC_PGN] = 0xd3;
            HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptPara1, &Tx2Mailbox);
            Mppt.TimeOut = MPPTTINT;
            while(Mppt.Reissued == 0)
            {
                if(Mppt.TimeOut <= 0)
                {
                    Mppt.SysSta = MTTP_READ_STEP4;
                    return;
                }
            }
            Mppt.Reissued = 0;
            break;
        }
        
        case MTTP_READ_STEP5:
        {
            CAN2_Header.ExtId= CAN2_rxHeader.ExtId;
            if(CAN2_Header.ExtId == DCDC_DATA_GET3) //  回复电压电流  
            {
                Para.MpptSingleT[DCDC_MODEL6] = ByteCom(can2_rxdata[1],can2_rxdata[0]);  // 模块IGBT6温度
                Para.MpptInTemp               = ByteCom(can2_rxdata[3],can2_rxdata[2]);  //进风口温度
                Para.MpptoutTemp              = ByteCom(can2_rxdata[5],can2_rxdata[4]);  //出风口温度
                Mppt.SysSta = MTTP_READ_STEP6;
            }
            Mppt.MpptRx_End_Flag = 1;
            break;
        }
        
        case MTTP_READ_STEP6:
        {
            MpptPara1[DCDC_PGN] =0xDB ;//故障字
            Tx2Message.ExtId = DCDC_FRAME;                               // 
            memset(can2_rxdata,0,8);
            memset(CAN2_RX_BUF,0,8);
            CAN2_Header.ExtId = 0;
            Mppt.SysSta = MTTP_READ_STEP7;
            Tx2Message.DLC = 8;                                            
            HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptPara1, &Tx2Mailbox);
            Mppt.TimeOut = MPPTTINT;
            while(Mppt.Reissued == 0)
            {
                if(Mppt.TimeOut <= 0)
                {
                    Mppt.SysSta = MTTP_READ_STEP6;
                    return;
                }
            }
            Mppt.Reissued = 0;
            break;
        }
        
        case MTTP_READ_STEP7:
        {
            CAN2_Header.ExtId= CAN2_rxHeader.ExtId;
            if(CAN2_Header.ExtId == DCDC_DATA_GET11)
            {
                Para.MpptDevStutas[0] = ByteCom(can2_rxdata[1],can2_rxdata[0]);     // 模块组号
                Para.MpptDevStutas[1] = ByteCom(can2_rxdata[3],can2_rxdata[2]);
                Para.MpptDevStutas[2] = ByteCom(can2_rxdata[5],can2_rxdata[4]);
                Para.MpptDevStutas[3] = ByteCom(can2_rxdata[7],can2_rxdata[6]);
                if(((Para.MpptDevStutas[0] & 0x01) != 0) || ((Para.MpptDevStutas[1] & 0x842) != 0) \
                    || ((Para.MpptDevStutas[2] & 0x842) != 0))
                {
                    Sys_Fault.mttp_fault = MPPT_SYSTEM_FAULT;
                }
                else
                {
                    Sys_Fault.mttp_fault = MPPT_SYSTEM_NO_FAULT;
                }
                Mppt.SysSta = MTTP_READ_STEP8;
            }
            Mppt.MpptRx_End_Flag = 1;
            break;
        }
        
        case MTTP_READ_STEP8:
        {
            MpptPara1[DCDC_PGN] = 0xDC;
            Tx2Message.ExtId = DCDC_FRAME;                               // 故障字
            memset(can2_rxdata,0,8);
            memset(CAN2_RX_BUF,0,8);
            CAN2_Header.ExtId = 0;
            Mppt.SysSta = MTTP_READ_STEP9;
            Tx2Message.DLC = 8;                                            
            HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptPara1, &Tx2Mailbox);
            Mppt.TimeOut = MPPTTINT;
            while(Mppt.Reissued == 0)
            {
                if(Mppt.TimeOut <= 0)
                {
                    Mppt.SysSta = MTTP_READ_STEP8;
                    return;
                }
            }
            Mppt.Reissued = 0;
            break;
        }
        
        case MTTP_READ_STEP9:
        {
            CAN2_Header.ExtId= CAN2_rxHeader.ExtId;
            if(CAN2_Header.ExtId == DCDC_DATA_GET12)
            {
                Para.MpptBFault                 = ByteCom(can2_rxdata[1],can2_rxdata[0]);
                Para.MpptSysFault               = ByteCom(can2_rxdata[3],can2_rxdata[2]);
                Para.MpptCurrFault[DCDC_MODEL1] = ByteCom(can2_rxdata[5],can2_rxdata[4]);
                Para.MpptCurrFault[DCDC_MODEL2] = ByteCom(can2_rxdata[7],can2_rxdata[6]);
                if(((Para.MpptBFault & 0x3C) != 0) || ((Para.MpptSysFault & 0x6FFF) != 0) \
                    || ((Para.MpptCurrFault[DCDC_MODEL1] & 0x10) != 0) || ((Para.MpptCurrFault[DCDC_MODEL2] & 0x10) != 0))
                {
                    Sys_Fault.mttp_fault = MPPT_SYSTEM_FAULT;
                }
                else
                {
                    Sys_Fault.mttp_fault = MPPT_SYSTEM_NO_FAULT;
                }
                Mppt.SysSta = MTTP_READ_STEP10;
            }
            Mppt.MpptRx_End_Flag = 1;
            break;
        }
        
        case MTTP_READ_STEP10:
        {
            
            MpptPara1[DCDC_PGN] = 0xDD;
            Tx2Message.ExtId = DCDC_FRAME;                      // 读取模块信息(模块电压电流限值信息)
            memset(can2_rxdata,0,8);
            memset(CAN2_RX_BUF,0,8);
            CAN2_Header.ExtId = 0;
            Mppt.SysSta = MTTP_READ_STEP11;
            Tx2Message.DLC = 8;                                           // Increment transmit data
            HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptPara1, &Tx2Mailbox);
            Mppt.TimeOut = MPPTTINT;
            while(Mppt.Reissued == 0)
            {
                if(Mppt.TimeOut <= 0)
                {
                    Mppt.SysSta = MTTP_READ_STEP10;
                    return;
                }
            }
            Mppt.Reissued = 0;
            break;
        }
        case MTTP_READ_STEP11:
        {
            CAN2_Header.ExtId= CAN2_rxHeader.ExtId;
            if(CAN2_Header.ExtId == DCDC_DATA_GET13)
            {
                Para.MpptCurrFault[DCDC_MODEL3] = ByteCom(can2_rxdata[1],can2_rxdata[0]);        // MPPT
                Para.MpptCurrFault[DCDC_MODEL4] = ByteCom(can2_rxdata[3],can2_rxdata[2]);        // MPPT
                Para.MpptCurrFault[DCDC_MODEL5] = ByteCom(can2_rxdata[5],can2_rxdata[4]);        // MPPT
                Para.MpptCurrFault[DCDC_MODEL6] = ByteCom(can2_rxdata[7],can2_rxdata[6]);  // MPPT
                if(((Para.MpptCurrFault[DCDC_MODEL3] & 0x10) != 0) || ((Para.MpptCurrFault[DCDC_MODEL4] & 0x10) != 0) \
                    || ((Para.MpptCurrFault[DCDC_MODEL5] & 0x10) != 0) || ((Para.MpptCurrFault[DCDC_MODEL6] & 0x10) != 0))
                {
                    Sys_Fault.mttp_fault = MPPT_SYSTEM_FAULT;
                }
                else
                {
                    Sys_Fault.mttp_fault = MPPT_SYSTEM_NO_FAULT;
                }
            
                Mppt.SysSta = MTTP_READ_STEP12;
            }
            Mppt.MpptRx_End_Flag = 1;
            break;
        }
        case MTTP_READ_STEP12:
        {
            MpptPara1[DCDC_PGN] = 0xD5;
            Tx2Message.ExtId = DCDC_FRAME;                      // 读取模块信息(模块电压电流限值信息)
            memset(can2_rxdata,0,8);
            memset(CAN2_RX_BUF,0,8);
            CAN2_Header.ExtId = 0;
            Mppt.SysSta = MTTP_READ_STEP13;
            Tx2Message.DLC = 8;                                           // Increment transmit data
            HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,MpptPara2, &Tx2Mailbox);
            Mppt.TimeOut = MPPTTINT;
            while(Mppt.Reissued == 0)
            {
                if(Mppt.TimeOut <= 0)
                {
                    Mppt.SysSta = MTTP_READ_STEP12;
                    return;
                }
            }
            Mppt.Reissued = 0;
            break;
        }
        case MTTP_READ_STEP13:
        {
            CAN2_Header.ExtId= CAN2_rxHeader.ExtId;
            if(CAN2_Header.ExtId == DCDC_DATA_GET5)
            {
                Para.MpptSingleV[DCDC_MODEL1] = ByteCom(can2_rxdata[1],can2_rxdata[0]);
                Para.MpptSingleC[DCDC_MODEL1] = ByteCom(can2_rxdata[3],can2_rxdata[2]);
                Para.MpptSingleP[DCDC_MODEL1] = ByteCom(can2_rxdata[5],can2_rxdata[4]);
                Para.MpptSysVolt              = ByteCom(can2_rxdata[1],can2_rxdata[0]);
                Para.MpptSysCurr              = ByteCom(can2_rxdata[3],can2_rxdata[2]);
                Para.MpptSysPwr               = ByteCom(can2_rxdata[5],can2_rxdata[4]);
                Mppt.SysSta = MTTP_READ_STEP0;
            }
        }
    }
}


