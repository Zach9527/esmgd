#include "main.h"
#include "EMS.h"
#include "rtc.h"

int32_t Surplus;
int32_t Data32_Temp;
void Restrictive_Batt(void);
extern Struct_InvParameter Para;
 
extern struct_MPPT Mppt;
extern uint8_t mppt_state_back;
extern uint8_t pcs_state_back;
extern Struct_FParameter ActualValue;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef DateToUpdate;
extern osTimerId EMTimerHandle;


/*******************************************************************************
* Function Name  : uint16_t  PcsCheck(void)
* Description    : PCS运行状态检测
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t  PcsCheck(void)
{
  return Para.PcsStatus;
}

/*******************************************************************************
* Function Name  : uint16_t  BMSCheck(void)
* Description    : 电池状态，0可充可放 1 不可充可放  2 不可放可充 3 不可充不可放
* Input          : None
* Output         : None
* Return         : None
过温、欠温（不可充不可放）、欠压（不可放）、过压（不可充）、压差（不可充不可放）
*******************************************************************************/
uint16_t  BMSCheck(void)   //  电池状态检查要修改,电池可能同时存在几种状态
{
uint16_t bme_state;	
	 if((Para.BmsBattStatus0 & 0xc000) == 0xc000)              //  BMS状态
	 {
		 if((Para.BmsBattStatus0 & 0x1000) == 0x1000)            //  充电使能
	   {
			 if((Para.BmsBattStatus0 & 0x100)== 0x100)             // 电池充满      
	     {
		     bme_state =  CANNOT_CHARGE_CAN_DISCHARGE;
	     }
			 else
			 {
				 bme_state =  CAN_CHARGE_CAN_DIDCHARGEE;                 // 0可充可放  
			 }
	 
	   }	 
		 if((Para.BmsBattStatus0 & 0x400) == 0x400)          // 放电使能
	   {		    
			  if((Para.BmsBattStatus0 & 0x40) == 0x40)           // 电池放空            
	      {
		      bme_state =  CAN_CHARGE_CANNOT_DISCHARGE;           // 2 不可放可充 
	      }
				else
				{
					  bme_state =  CAN_CHARGE_CAN_DIDCHARGEE;     // 0可充可放  
				}			 
	   }
	 }	 
	 else  
	 {
		   bme_state = CANNOT_CHARGE_CANNOT_DISCHARGE;        // 3 不可充不可放;
	 }	 
	 return bme_state;
}	 

 
/*******************************************************************************
* Function Name  : uint16_t  Restrictive_Batt(void)
* Description    : 根据电池故障类型,对电池的充放电限制
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void Restrictive_Batt(void)	 
{

	 
   Data32_Temp = Data32_Temp;// / 2;       
	 	 
	 	 
	 	 
 
}

/*******************************************************************************
* Function Name  : void  LoadFirst(void)
* Description    : 负载优先
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  LoadFirst(void)
{

// =============================================================	 
// 电流\功率\电阻设定为负值，为充电模式，设定为正值，为放电模式	
// =============================================================	 

   Surplus = ActualValue.MpptSysPower - ActualValue.PcsLoadActivePower;
	
	
//	 Data32_Temp = (int32_t)(0);// ActualValue.BattPowerMax;   
//	 Mppt.Pwr = (uint16_t)(ActualValue.PcsLoadActivePower + ActualValue.BattPowerMax);  
	
	 if(Surplus > 0)                                                          // 光伏充足时,PV优先供应负载，余电充电池
	 { 
			if((BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE) || (BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE))  // 可充可放,不可放可充 
			{
				if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)                   // 防逆流,AC恒功率模式,功率设为零或负
				{
					 PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
					 Data32_Temp = -(int32_t)ActualValue.BattPowerMax;  
					 Mppt.Pwr = (uint16_t)(ActualValue.PcsLoadActivePower + ActualValue.BattPowerMax);            
				}
				else
				{ // PV能量充足时，PV优先供应负载，余电充电池
					 if(Surplus > ActualValue.BattPowerMax)                        // 供负载后剩余功率大于充电最大功率
					 {
						  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
						  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;          // 按最大充电功率充电池,值为负   
					 }
					 else
					 {
						  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
						  Data32_Temp = -(int32_t)Surplus;                           // 供负载后剩余功率充电池,值为负 
					 }

					 Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;             // PV全功率
				}
			}
			else if(BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE)                 // 已充满,不可充可放
			{
				if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)                // 防逆流
				{
					 PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
					 Data32_Temp = -(int32_t)(0);   
					 Mppt.Pwr = (uint16_t)ActualValue.PcsLoadActivePower; 
				}
				else
				{
					 if(ActualValue.MpptSysPower > ActualValue.PcsChargePowerMax)     // 光伏功率大于PCS最大功率
					 {	// AC恒功率控制不了电池放电,DC恒功率 = 0,MPPT功率很足,电池充满后不放 		
              PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 						 
						  Data32_Temp  = 0;                                             // 按MPPT当前功率供负载和回馈电网 
							Mppt.Pwr = (uint16_t)ActualValue.PcsChargePowerMax;           // 限制MPPT功率为PCS最大功率
					 }
					 else
					 {
						  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
						  Data32_Temp = 0;                                              // 按MPPT最大功率供负载和回馈电网        
							Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;             // 限制MPPT功率为PCS最大功率
					 }
				}
			}
		}
		else     // 光伏不足时,PV功率不满足负载时，电池自动放电
		{
				if(BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE || BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE)        // 电池未放截止
				{
					  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
					  Data32_Temp = -(int32_t)Surplus;                                 // 电池补充MPPT不足部分，值为正
				
				}
				else if(BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE)                   // 已放截止,涓流电流充
				{   // A * V = W
					  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
					  Data32_Temp = -(int32_t)(ActualValue.TrickleCurr * ActualValue.PcsDcVoltage);
				}	
		
				Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
		}
    Restrictive_Batt();
		Data32_Temp = Data32_Temp * 1000;
		PcsData.Value[2] = Data32_Temp >> 16;
		PcsData.Value[3] = Data32_Temp;		
		SetPcs(PCS_MODE_INVERTER);  
		SetMppt(MPPT_SET_POWER); 		
}

/*******************************************************************************
* Function Name  : void  PermanentFailureMode(void)
* Description    : 负载优先-保电
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LoadFirst_PowerConservation(void)
{
	Surplus = ActualValue.MpptSysPower - ActualValue.PcsLoadActivePower;
	if(Surplus > 0)                                                        // 光伏充足时
			 { 
					if(BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE)
					{
							if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)               // 防逆流
							{
								  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;
								  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
									Mppt.Pwr = (uint16_t)(ActualValue.PcsLoadActivePower + ActualValue.BattPowerMax); 
							}
							else
							{
								  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;
									if(Surplus > Para.BattMaxPwr)                             // 剩余功率大于充电最大功率
									{
										 Data32_Temp = -(int32_t)Para.BattMaxPwr;
									}
									else
									{
										 Data32_Temp = -(int32_t)Surplus;
									}
									Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
						 }
							
				}
				else if(BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE)                  // 已充满
				{
					 PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;
					 if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)                // 防逆流
					 {
						  Data32_Temp = -(int32_t)0;
							Mppt.Pwr = (int16_t)ActualValue.PcsLoadActivePower;
					 }
					 else
					 {
							if(ActualValue.MpptSysPower > ActualValue.PcsChargePowerMax)   // 光伏功率大于PCS最大功率
							{
								 Data32_Temp = -(int32_t)0;
								 Mppt.Pwr = (uint16_t)ActualValue.PcsChargePowerMax;
							}
							else
							{
								 Data32_Temp = -(int32_t)0;
								 Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
							}
				   }
				}
			}
			else //光伏不足时,不对电池充电
			{
				  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;
			 	  Data32_Temp = -(int32_t)0;     
					Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
			}
			Restrictive_Batt();
		Data32_Temp = Data32_Temp * 1000;
		PcsData.Value[2] = Data32_Temp >> 16;
		PcsData.Value[3] = Data32_Temp;		
		SetPcs(PCS_MODE_INVERTER);  
		SetMppt(MPPT_SET_POWER); 		
}

/*******************************************************************************
* Function Name  : void  BattFirst(void)
* Description    : 电池优先
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  BattFirst(void)
{

	  Surplus = ActualValue.MpptSysPower - ActualValue.BattPowerMax;
	
		if(Surplus > 0)                                                     // 光伏充足时,优先充电池，余电供负载
		{ 
				if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)               // 防逆流
				{
						 if((BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE) || (BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE))  // 可充可放,不可放可充 
						 {							 
							  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;   
							  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
	
								Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax; 
						 }
						 else
						 {
							  PcsData.Value[1] = PCS_AC_CONS_POWER_RUN;                              //  AC模式   							 
							  Data32_Temp = 0;         
								Mppt.Pwr = (uint16_t)ActualValue.PcsLoadActivePower; 						 
						 }
				}
				else
				{
					  if((BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE) || (BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE))  // 可充可放,不可放可充 
						{
							  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;   
					      Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
								Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax; 							
						}
						else
						{
							  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;    
							  Data32_Temp = 0; 
//							  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
								Mppt.Pwr = (uint16_t)ActualValue.MpptSysPower; 						 
						}
				
				}
		}
		else                                                                 // 光伏不足时,PV优先给电池充电
		{
			if((BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE) || (BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE))  // 可充可放,不可放可充 
			{
				if(Para.PVGridBothChargeEnable == 1)                             // 电网光伏同时充电
				{
					  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;   
					  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;                       
						Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
				}
				else                                                             // 电网光伏不能同时充电
				{
					  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;   
					  Data32_Temp = -(int32_t)ActualValue.MpptSysPower;              
						Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
				}
								
			}
			else
			{
				    if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)           // 防逆流
						{
							  PcsData.Value[1] = PCS_AC_CONS_POWER_RUN;                              //  AC模式   
				        Data32_Temp = 0;                      
						    Mppt.Pwr = (uint16_t)ActualValue.PcsLoadActivePower;			
						}
						else
						{
							  PcsData.Value[1] = PCS_AC_CONS_POWER_RUN;                              //  AC模式   							
				        Data32_Temp = 0;                        
						    Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;									
						}
			}
    }
		Restrictive_Batt();
		Data32_Temp = Data32_Temp * 1000;
		PcsData.Value[2] = Data32_Temp >> 16;
		PcsData.Value[3] = Data32_Temp;		
		SetPcs(PCS_MODE_INVERTER);  
		SetMppt(MPPT_SET_POWER); 		
}

/*******************************************************************************
* Function Name  : void  ECO(void)
* Description    : 经济模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  Economic_Models(void)
{
	  uint16_t prd, seg;
	  uint16_t date = (DateToUpdate.Month << 8) | DateToUpdate.Date, time = (sTime.Hours << 8) | sTime.Minutes ;
	  int i;
	  for (i = 0; i < 4; i++)
			if (date < Para.ECOCldSet[i].Date)
				break;
		prd = Para.ECOCldSet[i].Prd;
		for (i = 0; i < 6; i++)
			if (time < Para.ECOSegSet[prd][i].Time)
				break;
		if (!i)
			i = 6;
		else
			i--;
		seg = Para.ECOSegSet[prd][i].Seg;
    Surplus = ActualValue.MpptSysPower - ActualValue.PcsLoadActivePower;
    switch(seg)
		{
			case ECO_SEG_PEDI:
			case ECO_SEG_PEAK:
				LoadFirst();
				break;
			case ECO_SEG_PLAIN:
				LoadFirst_PowerConservation();
				break;
			case ECO_SEG_VALLEY:
				BattFirst();
				break;
		}	
}

/*******************************************************************************
* Function Name  : void  PeakLload_Shifting(void)
* Description    : 削峰填谷
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  PeakLload_Shifting(void)
{
	/*
		float Surplus1,Surplus2;

	
    Surplus = ActualValue.MpptSysPower - ActualValue.PcsLoadActivePower - ActualValue.BattPowerMax;
		Surplus1 = Surplus + ActualValue.GridLimitUp;
		Surplus2 = Surplus1 + ActualValue.BattPowerMax;
	
	
		if(Surplus > 0)                  // PV功率大于负载和充电功率时，不从电网取电，PV带负载并对电池充电
		{
			  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
		}
		else if(Surplus1 > 0)           // (PV功率+电网上限功率)大于(负载功率+充电功率时)时,电网和PV同时供电负载并对电池充电
		{
			  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
		}
		else if(Surplus2 > 0)           // (PV功率+电网上限功率)大于负载功率时,电网和PV优先供负载,余电充电池.
		{
			  Data32_Temp = -(int32_t)(ActualValue.MpptSysPower + ActualValue.GridLimitUp - ActualValue.PcsLoadActivePower);
			  if(Data32_Temp < -(uint32_t)ActualValue.BattPowerMax)
				{
					 Data32_Temp = -(uint32_t)ActualValue.BattPowerMax;
				}
		}
		else if(Surplus2 < 0)           // (PV功率+电网上限功率)小于负载功率时,电网,PV和电池同时供负载
		{
			  Data32_Temp = (int32_t)ActualValue.PcsLoadActivePower;    // 电池要放电,值为正
		}
*/
		float Surplus = ActualValue.MpptSysPower - ActualValue.PcsLoadActivePower;
		float Surplus1 = Surplus + ActualValue.GridLimitLow;
		//float Surplus1 = ActualValue.MpptSysPower + ActualValue.GridLimitLow - ActualValue.PcsLoadActivePower;
		float Surplus2 = Surplus + ActualValue.GridLimitUp;
		//float Surplus2 = ActualValue.MpptSysPower + ActualValue.GridLimitUp - ActualValue.PcsLoadActivePower;
		if (Surplus > 0) //光伏可满足负载，余量充电
			Data32_Temp = -(Surplus > ActualValue.BattPowerMax?ActualValue.BattPowerMax:Surplus);
		else if (Surplus1 > 0) //光伏+电网下限可满足负载，余量充电
			Data32_Temp = -Surplus1;
		else if (Surplus2 > 0) //光伏+电网下限不满足但不超过电网上限，光伏和电网供负载，不充电
			Data32_Temp = 0;
		else //负载超过光伏+电网上限，使用电池供电
			Data32_Temp = -Surplus2;
		PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
		Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
		Restrictive_Batt();
		Data32_Temp = Data32_Temp * 1000;
		PcsData.Value[2] = Data32_Temp >> 16;
		PcsData.Value[3] = Data32_Temp;		
		SetPcs(PCS_MODE_INVERTER);  
		SetMppt(MPPT_SET_POWER); 				
}

/*******************************************************************************
* Function Name  : void  EMS(void)
* Description    : EMS模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  EMS(void)
{

}

/*******************************************************************************
* Function Name  : void  AlonePVMode(void)
* Description    : 单PV模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//1. 无电网时，将启停旋钮转至ON,不点LCD开机键，储能控制器将开机进入单PV模
//   式，此时PV只对电池充电（最大30A），不会逆变输出到交流侧。
//2. 无电网时，在单PV模式时手动在LCD屏上开机，储能控制器进入离网模式。
//3. 离网模式放电至电池欠压告警点时，储能控制器关闭逆变，自动切换到单PV模
//   式，待电池充到设定电压(单PV转离网)时自动转入离网。
//4. 单PV模式检测到电网恢复正常，储能控制器自动进入并网模式。
void  AlonePVMode(void)
{

	 
		if(BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE)
		{
			  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
			  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
				Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;

		}
		else if(BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE)        // 已充满,转离网
		{
				GridOff();
		}
	  Restrictive_Batt();
		Data32_Temp = Data32_Temp * 1000;
		PcsData.Value[2] = Data32_Temp >> 16;
		PcsData.Value[3] = Data32_Temp;		
		SetPcs(PCS_MODE_INVERTER);  
		SetMppt(MPPT_SET_POWER); 	 
	 
}


/*******************************************************************************
* Function Name  : void  GridOff(void)
* Description    : 离网模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  GridOff(void)
{
		Data32_Temp = Para.BKPVolt * 1000;
	  int32_t Data32_Temp2 = Para.BKPFreq * 1000;
	  if(BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE) //不可充，MPPT功率写0
			Mppt.Pwr = 0.0f;
		else if(BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE && Para.PcsDcPL > 100) //电池不可放且负载功率大于光伏功率，系统停机
			SysControl.SysFlag = STA_STOP;
	  else //可充可放，MPPT正常运行
			Mppt.Pwr = ActualValue.MpptSysPowerMax;
		PcsData.Value[1] = PCS_MODE_OFF;
		PcsData.Value[2] = Data32_Temp >> 16;
		PcsData.Value[3] = Data32_Temp & 0xffff;		
	  PcsData.Value[4] = Data32_Temp2 >> 16;
	  PcsData.Value[5] = Data32_Temp2 & 0xffff;
		SetPcs(PCS_MODE_INVERTER); 
		SetMppt(MPPT_SET_POWER);
}



/*******************************************************************************
* Function Name  : void  DieselEngineMode(void)
* Description    : 油机模式(要完善)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  DieselEngineMode(void)
{
	
    Surplus = ActualValue.MpptSysPower - ActualValue.BattPowerMax;


		if((BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE) || (BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE))              // 电池状态，0可充可放 1 不可充可放  2 不可放可充 3 不可充不可放
		{ 
			    PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;
					if(ActualValue.MpptSysPower > ActualValue.BattPowerMax)       // PV功率大于充电功率
					{
							Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
					}
					else                                                          // PV功率小于充电功率
					{
							Data32_Temp = (int32_t)Surplus;                           // 这时Surplus为负
					}	
									
					Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
		}
    else
		{
	    	//STOPDE();                                              // 停止油机并转入离网模式
			  GridOff();
		}
		Restrictive_Batt();
		Data32_Temp = Data32_Temp * 1000;
		PcsData.Value[2] = Data32_Temp >> 16;
		PcsData.Value[3] = Data32_Temp;		
		SetPcs(PCS_MODE_INVERTER);  
		SetMppt(MPPT_SET_POWER); 	

}







/*******************************************************************************
* Function Name  : void  GridAutoSwitchingMode(void)
* Description    : 并离网自动切换模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  GridAutoSwitchingMode(void)
{

}


/*******************************************************************************
* Function Name  : void  FailureMode(void)
* Description    : 故障模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  FailureMode(void)
{
		SYSTEM_FAILURE_INDICATION_ON;    // 系统故障指示亮
}


/*******************************************************************************
* Function Name  : void  PermanentFailureMode(void)
* Description    : 永久故障模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  PermanentFailureMode(void)
{
		SYSTEM_FAILURE_INDICATION_ON;    // 系统故障指示亮
}



/*******************************************************************************
* Function Name  : void  PermanentFailureMode(void)
* Description    : 本地调度
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Local_scheduling(void)
{
	  int32_t Data32_Temp2;
	  switch (Para.LSMode)
		{
		  case LS_MODE_ACCP:
			  Para.PcsRunMode = PCS_MODE_ACCP;
			  Data32_Temp = Para.LSActiveP * 1000;
			  Data32_Temp2 = 0;
			  break;
		  case LS_MODE_DCCP:
			  Para.PcsRunMode = PCS_MODE_DCCP;
			  Data32_Temp = Para.LSActiveP * 1000;
		    Data32_Temp2 = Para.LSReactiveP * 1000;
			  break;
		}
		Mppt.Pwr = (uint16_t)ActualValue.PcsChargePowerMax;
		PcsData.Value[1] = Para.PcsRunMode;     // PCS运行模式
		Restrictive_Batt();
		Data32_Temp = Data32_Temp * 1000;
		Data32_Temp2 = Data32_Temp2 * 1000;
		PcsData.Value[2] = Data32_Temp >> 16;
		PcsData.Value[3] = Data32_Temp;		
		PcsData.Value[4] = Data32_Temp2 >> 16;
		PcsData.Value[5] = Data32_Temp2;
		SetPcs(PCS_MODE_INVERTER); 
		SetMppt(MPPT_SET_POWER); 
}


/*******************************************************************************
* Function Name  : void  PermanentFailureMode(void)
* Description    : 远程调度
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Remote_scheduling(void)
{

	  int32_t Data32_Temp2;
		switch (Para.LSMode)
		{
		  case LS_MODE_ACCP:
			  Para.PcsRunMode = PCS_MODE_ACCP;
			  Data32_Temp = Para.LSActiveP;
			  Data32_Temp2 = 0;
			  break;
		  case LS_MODE_DCCP:
			  Para.PcsRunMode = PCS_MODE_DCCP;
			  Data32_Temp = Para.LSActiveP;
		      Data32_Temp2 = Para.LSReactiveP;
			  break;
		}
		Mppt.Pwr = (uint16_t)ActualValue.PcsChargePowerMax;
		PcsData.Value[1] = Para.PcsRunMode;     // PCS运行模式
		Restrictive_Batt();
		Data32_Temp = Data32_Temp * 1000;
		Data32_Temp2 = Data32_Temp2 * 1000;
		PcsData.Value[2] = Data32_Temp >> 16;
		PcsData.Value[3] = Data32_Temp;		
		PcsData.Value[4] = Data32_Temp2 >> 16;
		PcsData.Value[5] = Data32_Temp2;
		SetPcs(PCS_MODE_INVERTER); 
		SetMppt(MPPT_SET_POWER);  			
	
}


/*******************************************************************************
* Function Name  : void  PermanentFailureMode(void)
* Description    : 外置电表
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void External_Electric_Meters(void)
{
	  if (Para.reservedx[0]++ > 60)
		{
			Para.reservedx[0] = 0;
			int32_t temp =  (int32_t)Para.Total_Active_Ppower * (int32_t)Para.CurrRatio * (int32_t)Para.VoltRatio - (int32_t)Para.EMActiveP;
			int32_t present = ((int32_t)PcsData.Value[2] << 16) + (int32_t)PcsData.Value[3];
			Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
			Restrictive_Batt();
			temp = temp * 1000;
			present += temp;
			PcsData.Value[1] = PCS_AC_CONS_POWER_RUN; 
			PcsData.Value[2] = present >> 16;
			PcsData.Value[3] = present;		
			SetPcs(PCS_MODE_INVERTER);  
			SetMppt(MPPT_SET_POWER); 
		}
		return ;
}

void EM_TimerCallback(const void * arg)
{
		int32_t temp =  ((int32_t)Para.Total_Active_Ppower | (int32_t)0xffff0000) * (int32_t)Para.CurrRatio * (int32_t)Para.VoltRatio - (int32_t)Para.EMActiveP;
	  int32_t present = ((int32_t)PcsData.Value[2] << 16) + (int32_t)PcsData.Value[3];
		Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
		Restrictive_Batt();
		temp = temp * 1000;
	  present += temp;
		PcsData.Value[1] = PCS_AC_CONS_POWER_RUN; 
		PcsData.Value[2] = present >> 16;
		PcsData.Value[3] = present;		
		SetPcs(PCS_MODE_INVERTER);  
		SetMppt(MPPT_SET_POWER); 
		return ;
}



