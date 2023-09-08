#include "main.h"

void dicontrol_cal(void);
void docontrol_cal(void);
extern Struct_InvParameter Para;
extern Struct_SystemFault Sys_Fault;


void dido_task(void *argument)
{
	while(1)
	{
		  osDelay(10);
		  //osSignalWait(0x08,osWaitForever);  Wait for what?
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);         //喂硬件狗
		  dicontrol_cal();
		  docontrol_cal();
	}
}	

/*******************************************************************************
* Function Name  : void  dicontrol_cal(void)
* Description    : 读取外部I点状态
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void dicontrol_cal(void)
{
	Para.Din = (EMERGENCY_STOP_INPUT ? DI_EM_STOP : 0u) | (SURGE_SERIES_FEEDBACK ? DI_SPD_FAILURE : 0u) | (GRID_CIRCUIT_BREAK_FEEDBACK ? 0u : DI_GRID_MCCB) |\
	(LOAD_CIRCUIT_BREAK_FEEDBACK ? 0u : DI_LOAD_MCCB) | (EXTERN_FAN_FEEDBACK ? 0u : DI_EX_FAN) | (BYPASS_CIRCUIT_BREAK_FEEDBACK ? 0u : DI_BYPASS_MCCB);
		 if(EMERGENCY_STOP_INPUT == GPIO_PIN_SET)
		 {
			  SysControl.SysFlag = STA_EMSTOP;        // 急停输入,系统处于急停状态
			  Para.OnOff = 0;                         // 运行命令复位
		 }
		 else
		 {
			  SYSTEM_FAILURE_INDICATION_OFF;
		 }
		 
		 if(SysControl.SysFlag == STA_RUN)          // 系统运行
		 {
		    SYSTEM_OPERATION_INDICATION_ON;
		 }
		 else
		 {
			  SYSTEM_OPERATION_INDICATION_OFF;
		 }
}


/*******************************************************************************
* Function Name  : void  docontrol_cal(void)
* Description    : 输出O点,控制风机,空调供电等
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void docontrol_cal(void)
{
	 		if(Para.PcsIgbtOutletTemp > 850)         // int16 0.1℃
			{
      	FAN_IN_POWER_CABINET_ON;               // 电源柜风机开
				EXTERN_FAN_POWER_CONTROL_ON;           // 柜外风机供电控制开
			}
			else if(Para.PcsIgbtOutletTemp < 800)
			{
				FAN_IN_POWER_CABINET_OFF;
				EXTERN_FAN_POWER_CONTROL_OFF;
			}
			if(SysControl.SysFlag == STA_RUN)
			{
				 SYSTEM_OPERATION_INDICATION_ON;     // 系统运行指示亮
			}
			else
			{				
				 SYSTEM_OPERATION_INDICATION_OFF;    // 系统运行指示熄
			}
				
			if((Sys_Fault.pcs_fault == PCS_SYSTEM_FAULT) ||
				 (Sys_Fault.mttp_fault == MPPT_SYSTEM_FAULT)||
				 (Sys_Fault.bms3_fault == BMS_SYSTEM_FAULT))
	   {
	      SYSTEM_FAILURE_INDICATION_ON;       // 系统故障指示亮
	   }	
     else
	   {
			 SYSTEM_FAILURE_INDICATION_OFF;       // 系统故障指示熄
		 }
}	
 

