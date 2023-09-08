#include "main.h"
#include "EMS.h"

typedef struct{
    uint16_t BmsBattFault0;
    uint16_t BmsBattFault1;
    uint16_t BmsBattFault2;
}BMS;
typedef struct{
    uint32_t CPwrUp;  //Charging power upper limit
    uint32_t DPwrUp;  //Discharging power upper limit
}PCS;

BMS batt;
PCS pcs;

//PCS Power set
#define PCS_CPWR_MAX 300000u
#define PCS_DPWR_MAX 300000u
#define PCS_CPWR_RT (PCS_CPWR_MAX >> 1)
#define PCS_DPWR_RT (PCS_DPWR_MAX >> 1)

//BMS fault check
#define BMS_FAULT (BMS_FAULT0 || BMS_FAULT1 || BMS_FAULT2)
#define BMS_FAULT0 batt.BmsBattFault0
#define BMS_FAULT1 batt.BmsBattFault1
#define BMS_FAULT2 batt.BmsBattFault2

/* BMS fault 0 segment */
//Charging over current
#define BMS_FAULT_COC        (BMS_FAULT0 & 0xc000u)  
#define BMS_FAULT_COC_MILD   !(BMS_FAULT_COC ^ 0x4000u)
#define BMS_FAULT_COC_MEDIUM !(BMS_FAULT_COC ^ 0x8000u) 
#define BMS_FAULT_COC_SEVERE !(BMS_FAULT_COC ^ 0xc000u)
//Discharging over current
#define BMS_FAULT_DOC        (BMS_FAULT0 & 0x3000u)  
#define BMS_FAULT_DOC_MILD   !(BMS_FAULT_DOC ^ 0x1000u)
#define BMS_FAULT_DOC_MEDIUM !(BMS_FAULT_DOC ^ 0x2000u) 
#define BMS_FAULT_DOC_SEVERE !(BMS_FAULT_DOC ^ 0x3000u)
//Cell voltage difference
#define BMS_FAULT_EVD        (BMS_FAULT0 & 0x0c00u)  
#define BMS_FAULT_EVD_MILD   !(BMS_FAULT_EVD ^ 0x0800u)
#define BMS_FAULT_EVD_MEDIUM !(BMS_FAULT_EVD ^ 0x0400u) 
#define BMS_FAULT_EVD_SEVERE !(BMS_FAULT_EVD ^ 0x0c00u)
//Cell over voltage
#define BMS_FAULT_EOV        (BMS_FAULT0 & 0x0300u)  
#define BMS_FAULT_EOV_MILD   !(BMS_FAULT_EOV ^ 0x0100u)
#define BMS_FAULT_EOV_MEDIUM !(BMS_FAULT_EOV ^ 0x0200u) 
#define BMS_FAULT_EOV_SEVERE !(BMS_FAULT_EOV ^ 0x0300u)
//Cell under voltage
#define BMS_FAULT_EUV        (BMS_FAULT0 & 0x00c0u)  
#define BMS_FAULT_EUV_MILD   !(BMS_FAULT_EUV ^ 0x0080u)
#define BMS_FAULT_EUV_MEDIUM !(BMS_FAULT_EUV ^ 0x0040u) 
#define BMS_FAULT_EUV_SEVERE !(BMS_FAULT_EUV ^ 0x00c0u)
//Cell temperature difference
#define BMS_FAULT_ETD        (BMS_FAULT0 & 0x0030u)  
#define BMS_FAULT_ETD_MILD   !(BMS_FAULT_ETD ^ 0x0010u)
#define BMS_FAULT_ETD_MEDIUM !(BMS_FAULT_ETD ^ 0x0020u) 
#define BMS_FAULT_ETD_SEVERE !(BMS_FAULT_ETD ^ 0x0030u)
//Cell over temperature
#define BMS_FAULT_EOT        (BMS_FAULT0 & 0x000cu)  
#define BMS_FAULT_EOT_MILD   !(BMS_FAULT_EOT ^ 0x0008u)
#define BMS_FAULT_EOT_MEDIUM !(BMS_FAULT_EOT ^ 0x0004u) 
#define BMS_FAULT_EOT_SEVERE !(BMS_FAULT_EOT ^ 0x000cu)
//Cell under temperature
#define BMS_FAULT_EUT        (BMS_FAULT0 & 0x0003u)  
#define BMS_FAULT_EUT_MILD   !(BMS_FAULT_EUT ^ 0x0001u)
#define BMS_FAULT_EUT_MEDIUM !(BMS_FAULT_EUT ^ 0x0002u) 
#define BMS_FAULT_EUT_SEVERE !(BMS_FAULT_EUT ^ 0x0003u)

/* BMS fault 1 segment */
//Charging under temperature
#define BMS_FAULT_CUT        (BMS_FAULT1 & 0xc000u)  
#define BMS_FAULT_CUT_MILD   !(BMS_FAULT_CUT ^ 0x4000u)
#define BMS_FAULT_CUT_MEDIUM !(BMS_FAULT_CUT ^ 0x8000u) 
#define BMS_FAULT_CUT_SEVERE !(BMS_FAULT_CUT ^ 0xc000u)
//Discharging under temperature
#define BMS_FAULT_DUT        (BMS_FAULT1 & 0x3000u)  
#define BMS_FAULT_DUT_MILD   !(BMS_FAULT_DUT ^ 0x1000u)
#define BMS_FAULT_DUT_MEDIUM !(BMS_FAULT_DUT ^ 0x2000u) 
#define BMS_FAULT_DUT_SEVERE !(BMS_FAULT_DUT ^ 0x3000u)
//Insulation impedence low
#define BMS_FAULT_IIL        (BMS_FAULT1 & 0x0c00u)  
//Relay fault
#define BMS_FAULT_RLY        (BMS_FAULT1 & 0x0300u)  
//Balance fault
#define BMS_FAULT_BLC        (BMS_FAULT1 & 0x00c0u)  
//Current detection fault
#define BMS_FAULT_CDT        (BMS_FAULT1 & 0x0030u)  
//Voltage detection fault
#define BMS_FAULT_VDT        (BMS_FAULT1 & 0x000cu)  
//Temperature detection fault
#define BMS_FAULT_TDT        (BMS_FAULT1 & 0x0003u) 

/* BMS fault 2 segment */
//Battery system over voltage
#define BMS_FAULT_SOV        (BMS_FAULT2 & 0xc000u)  
#define BMS_FAULT_SOV_MILD   !(BMS_FAULT_SOV ^ 0x4000u)
#define BMS_FAULT_SOV_MEDIUM !(BMS_FAULT_SOV ^ 0x8000u) 
#define BMS_FAULT_SOV_SEVERE !(BMS_FAULT_SOV ^ 0xc000u)
//Battery system under voltage
#define BMS_FAULT_SUV        (BMS_FAULT2 & 0x3000u)  
#define BMS_FAULT_SUV_MILD   !(BMS_FAULT_SUV ^ 0x1000u)
#define BMS_FAULT_SUV_MEDIUM !(BMS_FAULT_SUV ^ 0x2000u) 
#define BMS_FAULT_SUV_SEVERE !(BMS_FAULT_SUV ^ 0x3000u)
//Cell Soc too high
#define BMS_FAULT_ESH        (BMS_FAULT2 & 0x0c00u)  
#define BMS_FAULT_ESH_MILD   !(BMS_FAULT_ESH ^ 0x0400u)
#define BMS_FAULT_ESH_MEDIUM !(BMS_FAULT_ESH ^ 0x0800u) 
#define BMS_FAULT_ESH_SEVERE !(BMS_FAULT_ESH ^ 0x0c00u)
//Cell Soc too low
#define BMS_FAULT_ESL        (BMS_FAULT2 & 0x0300u)  
#define BMS_FAULT_ESL_MILD   !(BMS_FAULT_ESL ^ 0x0100u)
#define BMS_FAULT_ESL_MEDIUM !(BMS_FAULT_ESL ^ 0x0200u) 
#define BMS_FAULT_ESL_SEVERE !(BMS_FAULT_ESL ^ 0x0300u)
//Reserved

//Battery module communication fault
#define BMS_FAULT_BMC        (BMS_FAULT2 & 0x0030u)
//PCS communication fault
#define BMS_FAULT_PCS        (BMS_FAULT2 & 0x000cu)
//Battry rack communication fault
#define BMS_FAULT_BRC        (BMS_FAULT2 & 0x0003u)

/* BMS status macro */
// BMS charging restricted
#define BMS_STATUS_CRT (BMS_FAULT_COC_MILD   || BMS_FAULT_DOC_MILD   || BMS_FAULT_EVD_MILD   ||                        \
BMS_FAULT_EUV_MILD   || BMS_FAULT_ETD_MILD   || BMS_FAULT_EOT_MILD   || BMS_FAULT_EUT_MILD   || BMS_FAULT_CUT_MILD   ||\
BMS_FAULT_DUT_MILD   || BMS_FAULT_EUT_MILD   ||                         BMS_FAULT_SUV_MILD  )
// BMS charging forbidden
#define BMS_STATUS_CFB (BMS_FAULT_COC_MEDIUM || BMS_FAULT_DOC_MEDIUM || BMS_FAULT_EVD_MEDIUM ||                        \
BMS_FAULT_EUV_MEDIUM || BMS_FAULT_ETD_MEDIUM || BMS_FAULT_EOT_MEDIUM || BMS_FAULT_EUT_MEDIUM || BMS_FAULT_CUT_MEDIUM ||\
BMS_FAULT_DUT_MEDIUM || BMS_FAULT_EUT_MEDIUM ||                         BMS_FAULT_SUV_MEDIUM)
// BMS discharging restricted
#define BMS_STATUS_DRT (BMS_FAULT_COC_MILD   || BMS_FAULT_DOC_MILD   || BMS_FAULT_EVD_MILD   || BMS_FAULT_EOV_MILD   ||\
                        BMS_FAULT_ETD_MILD   || BMS_FAULT_EOT_MILD   || BMS_FAULT_EUT_MILD   || BMS_FAULT_CUT_MILD   ||\
BMS_FAULT_DUT_MILD   || BMS_FAULT_EUT_MILD   || BMS_FAULT_SOV_MILD                          )
// BMS discharging forbidden
#define BMS_STATUS_DFB (BMS_FAULT_COC_MEDIUM || BMS_FAULT_DOC_MEDIUM || BMS_FAULT_EVD_MEDIUM || BMS_FAULT_EOV_MEDIUM ||\
                        BMS_FAULT_ETD_MEDIUM || BMS_FAULT_EOT_MEDIUM || BMS_FAULT_EUT_MEDIUM || BMS_FAULT_CUT_MEDIUM ||\
BMS_FAULT_DUT_MEDIUM || BMS_FAULT_EUT_MEDIUM || BMS_FAULT_SOV_MEDIUM                        )
// BMS battery system stoped
#define BMS_STATUS_SST (BMS_FAULT_IIL || BMS_FAULT_RLY || BMS_FAULT_BLC || BMS_FAULT_CDT || BMS_FAULT_VDT || BMS_FAULT_TDT ||\
BMS_FAULT_BMC || BMS_FAULT_PCS || BMS_FAULT_BRC)
// BMS battery system break
#define BMS_STATUS_SBR (BMS_FAULT_COC_SEVERE || BMS_FAULT_DOC_SEVERE || BMS_FAULT_EVD_SEVERE || BMS_FAULT_EOV_SEVERE ||\
BMS_FAULT_EUV_SEVERE || BMS_FAULT_ETD_SEVERE || BMS_FAULT_EOT_SEVERE || BMS_FAULT_EUT_SEVERE || BMS_FAULT_CUT_SEVERE ||\
BMS_FAULT_DUT_SEVERE || BMS_FAULT_EUT_SEVERE || BMS_FAULT_SOV_SEVERE || BMS_FAULT_SUV_SEVERE)

// BMS discharging normal set
void BmsDNM(void);
// BMS discharging restricted set
void BmsDRT(void);
// BMS discharging forbidden set
void BmsDFB(void);
// BMS charging normal set
void BmsCNM(void);
// BMS charging restricted set
void BmsCRT(void);
// BMS charging forbidden set
void BmsCFB(void);
// BMS battery system stoped set
void BmsSST(void);
// BMS battery system break set
void BmsSBR(void);
void PcsSetLimit(void);
void BmsGetFault(void);

void Actual_Value_Cal(void);
void Pcs_Mppt_Bms_Stare(void);
Struct_InvParameter Para;
Struct_FParameter ActualValue;
Struct_SystemFault Sys_Fault;
struct_MPPT Mppt;
struct_PCSDATA PcsData;
struct_BMS Bms; 
Struct_SysControl SysControl;

uint32_t Tx1Mailbox;
uint8_t PcsWriteBuff[100];
uint8_t mppt_state_back;
uint8_t pcs_state_back;

/*******************************************************************************
* Function Name  : void  SysStateManage(void)
* Description    : 系统状态机控制
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysStateManage(void)
{
	  Actual_Value_Cal();                      // 实际值计算
	  Pcs_Mppt_Bms_Stare();                    // Pcs/Mppt/Bms状态监测
    switch( SysControl.SysFlag )
    {
        case STA_STOP:                        // 停机模式
        {
            StopState();
            break;
        }
		case STA_START:
		{
			//PcsData.Value[10] = PCS_POWER_ON;     // 开机PCS 
		    SetPcs(PCS_MODE_POWER_ON);            // 先PCS开机,后MPPT开机
			SysControl.WaitCnt = 20000;           // 20s
              break;
		}
        case STA_PCSCHECK:                      // PCS开机完成确认
        {
            PcsPupDeal();                       // 先PCS开机,后MPPT开机
            break;
        }
        case STA_RUN:                           // 运行
        {
            RunState();
            break;
        }
        case STA_FAILURE:                      // 故障模式
        {
            FailureState();
            break;
        }
        case STA_EMSTOP:                      // 紧急停机
        {
					 if(EMERGENCY_STOP_INPUT == GPIO_PIN_RESET)     // 急停输入
					 {
							SetMppt(MPPT_POWER_OFF);                    // 关MPPT
							PcsData.Value[10] = PCS_POWER_OFF;          // 关PCS
							SetPcs(PCS_MODE_POWER_ON);      						 
					 }
					
            break;
        }
        default:
        {
            break;
        }
    }
    PowerOff();
}

/*******************************************************************************
* Function Name  : void  StopState(void)
* Description    : 停机状态处理
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void StopState(void)
{
		if(Para.OnOff == 1)//开机
		{
//				if(Para.Grid_onoff == 0)           // 并网运行
//				{
//					 PcsData.Value[0] = 0x0000;
//					 PcsData.Value[1] = Para.PcsRunMode; // 0x0040;      // 0X40-->Ac恒功率运行
//					 PcsData.Value[2] = 0x0000;
//					 PcsData.Value[3] = 0x0000;
//					 PcsData.Value[4] = 0x0000;
//					 PcsData.Value[5] = 0x0000;
//					 PcsData.Value[6] = 0x0000;
//					 PcsData.Value[7] = 0x0000;
//					 SetPcs(PCS_MODE_INVERTER);                                 //设置PCS为AC恒功率,PCS功率为0
//				}
				SysControl.SysFlag  = STA_START;                              // PCS开机
		}
		else
		{
				SetMppt(MPPT_POWER_OFF);                                       // 关MPPT
				PcsData.Value[10] = PCS_POWER_OFF;                             // 关PCS
				SetPcs(PCS_MODE_POWER_ON);                        
		}
}

 
/*******************************************************************************
* Function Name  : void RunState(void)
* Description    : 运行模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RunState(void)
{
		if(Para.Grid_onoff == 0)   // 并网
		{
//			 Para.GridOnMode = 1;
			 switch(Para.GridOnMode)
			 {
					case EMS_STG_LS:
						 Local_scheduling();
						 break;
					case EMS_STG_RS:
						 Remote_scheduling();
						 break;
					case EMS_STG_LF:
						 LoadFirst();
						 break;
					case EMS_STG_LR:
						 LoadFirst_PowerConservation();
						 break; 
					case EMS_STG_BF:
						 BattFirst();
						 break;
					case EMS_STG_ECO:		
						 Economic_Models();
						 break;
					case EMS_STG_PLS:
						 PeakLload_Shifting();
						 break;
					case EMS_STG_BKP:
						 GridOff();
						 break;
					case EMS_STG_EM:
						 External_Electric_Meters();
						 break;
					default:
						 LoadFirst();
						 break;
			 }
		}
		if(Para.Grid_onoff == 1)           // 离网
		{
	    	GridOff();
		}
}

/*******************************************************************************
* Function Name  : void  PowerOff(void)
* Description    : 关机处理
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PowerOff(void)
{
  if(Para.OnOff == 0)                              // 关机状态
  {
     SysControl.SysFlag = STA_STOP;
  }

}

 

 
/*******************************************************************************
* Function Name  : void  FailureState(void)
* Description    : 故障状态处理
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FailureState(void)
{
	if(Para.PcsFaultClear == 1)
	{
		 SysControl.SysFlag = PCS_MODE_FAULT_CLEAR;   
		 SetPcs(PCS_MODE_FAULT_CLEAR);                 // 设置PCS故障复位指令
	}
	   
}


/*******************************************************************************
* Function Name  : void  Actual_Value_Cal(void)
* Description    : 变量转化为实际值,浮点形式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Actual_Value_Cal()
{
float Volt;
float Curr;
	
	 ActualValue.PcsLoadActivePower = (float)((Para.PcsLoadPH << 16) + Para.PcsLoadPL);    // 当前负载功率W
	 Volt = (float)Para.MpptSysVolt * 0.1f;                                                 // MPPT电压单位V
	 Curr = (float) Para.MpptSysCurr * 0.1f;               // MPPT电流单位A
	 ActualValue.MpptSysPower = Volt * Curr;                                               // 当前光伏功率W
	 ActualValue.PcsChargePowerMax = 10000.0f;                                              // PCS充电最大功率W
	 ActualValue.BattPowerMax = 10000.0f;                                                   // 电池最大功率W
	 ActualValue.PcsDcVoltage = (float)((Para.PcsDcVH << 16) + Para.PcsDcVL) * 0.001f;     // PCS直流电压V
	 ActualValue.MpptSysPowerMax = 30000.0f;                                                // MPPT最大功率W
	 ActualValue.TrickleCurr = 1.0f;                                                       // 充电涓流电流A
	 ActualValue.MpptSysVoltage = (float) Para.MpptSysVolt * 0.1f; // MPPT系统总电压V
	 ActualValue.MpptSysCurrent = (float)(+ Para.MpptSysCurr) * 0.1f; // MPPT系统总电流A
	 ActualValue.GridLimitUp = (float)Para.PLSup * 1000.0f;                                                    // 电网上限功率，削峰填谷时的取电限值
	 ActualValue.GridLimitLow = (float)Para.PLSlow * 1000.0f;
	 ActualValue.AgcPower = 2000.0f;   // (float)Para.AgcPower;                                          // AGC模式功率 
}


/*******************************************************************************
* Function Name  : void  Pcs_Mppt_Bms_Stare(void)
* Description    : PCS/MPPT/BMS状态监测
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/	
void Pcs_Mppt_Bms_Stare()
{
	if((Sys_Fault.pcs_fault == PCS_SYSTEM_FAULT) ||
		(Sys_Fault.mttp_fault == MPPT_SYSTEM_FAULT)||
	  (Sys_Fault.bms3_fault == BMS_SYSTEM_FAULT))
	{
	   SysControl.SysFlag = STA_FAILURE;
	}
  PcsSetLimit();	
	BmsGetFault();
}

void PcsSetLimit(){
    if (BMS_FAULT){
        if (BMS_STATUS_SBR)
            BmsSBR();
        else if (BMS_STATUS_SST)
            BmsSST();
        else{
            if (BMS_STATUS_CFB)
                BmsCFB();
            else if (BMS_STATUS_CRT)
                BmsCRT();
            else
                BmsCNM();
            if (BMS_STATUS_DFB)
                BmsDFB();
            else if (BMS_STATUS_DRT)
                BmsDRT();
            else
                BmsDNM();
        }
    }
    else{
        BmsCNM();
        BmsDNM();
    }
}

void BmsGetFault(void){
    batt.BmsBattFault0 = 0x1010;
    batt.BmsBattFault1 = 0x00;
    batt.BmsBattFault1 = 0x00;
}


void BmsDNM(void){
    pcs.DPwrUp = PCS_DPWR_MAX;
    return ;
}
void BmsCNM(void){
    pcs.CPwrUp = PCS_CPWR_MAX;
    return ;
}
void BmsDRT(void){
    pcs.DPwrUp = PCS_DPWR_RT;
    return ;
}
void BmsCRT(void){
    pcs.CPwrUp = PCS_CPWR_RT;
	  return ;
}
void BmsDFB(void){
    pcs.DPwrUp = 0u;
    return ;
}
void BmsCFB(void){
    pcs.CPwrUp = 0u;
    return ;
}
void BmsSST(void){
    //PCS stop
    return ;
}
void BmsSBR(void){
    //BMS stop
    return ;
}

/*******************************************************************************
* Function Name  : uint16_t  ByteCom(uint8_t DataA,uint8_t DataB)
* Description    : 两个字节合并为一个字
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

uint16_t  ByteCom(uint8_t DataA,uint8_t DataB)
{
  return ((uint16_t)DataA << 8) + DataB;
}

uint16_t  J_ByteCom(uint8_t DataA,uint8_t DataB)
{
  return ((uint16_t)DataB << 8) + DataA;
}


/* CRC 高位字节值表 */
const uint16_t auchCRCHi[] = {
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;

/* CRC低位字节值表*/
const uint16_t auchCRCLo[] = {
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
		0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
		0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
		0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
		0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
		0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
		0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
		0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
		0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
		0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
		0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
		0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
		0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
		0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
		0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
		0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
		0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
		0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
		0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
		0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;


 


/*******************************************************************************
* Function Name  : uint16_t Crc16(uint16_t *puchMsg, uint16_t usDataLen)
* Description    : CRC效验
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t Crc16(uint16_t *puchMsg, uint16_t usDataLen)
{
    uint16_t uchCRCHi = 0xFF ;              /* 高CRC字节初始化  */
    uint16_t uchCRCLo = 0xFF ;              /* 低CRC 字节初始化 */
    uint32_t uIndex ;                       /* CRC循环中的索引  */
    while (usDataLen--)                     /* 传输消息缓冲区   */
    {
        uIndex = uchCRCHi ^ *puchMsg++ ;    /* 计算CRC          */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo = auchCRCLo[uIndex] ;
    }
    return (uchCRCHi << 8 | uchCRCLo) ;
}

/*******************************************************************************
* Function Name  : uint16_t Crc8(uint8_t *puchMsg, uint16_t usDataLen)
* Description    : CRC效验
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t Crc8(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint16_t uchCRCHi = 0xFF ;              /* 高CRC字节初始化  */
    uint16_t uchCRCLo = 0xFF ;              /* 低CRC 字节初始化 */
    uint32_t uIndex ;                       /* CRC循环中的索引  */
    while (usDataLen--)                     /* 传输消息缓冲区   */
    {
        uIndex = uchCRCHi ^ *puchMsg++ ;    /* 计算CRC          */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo = auchCRCLo[uIndex] ;
    }
    return (uchCRCHi << 8 | uchCRCLo) ;
}
