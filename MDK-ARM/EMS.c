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
* Description    : PCS����״̬���
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
* Description    : ���״̬��0�ɳ�ɷ� 1 ���ɳ�ɷ�  2 ���ɷſɳ� 3 ���ɳ䲻�ɷ�
* Input          : None
* Output         : None
* Return         : None
���¡�Ƿ�£����ɳ䲻�ɷţ���Ƿѹ�����ɷţ�����ѹ�����ɳ䣩��ѹ����ɳ䲻�ɷţ�
*******************************************************************************/
uint16_t  BMSCheck(void)   //  ���״̬���Ҫ�޸�,��ؿ���ͬʱ���ڼ���״̬
{
uint16_t bme_state;	
	 if((Para.BmsBattStatus0 & 0xc000) == 0xc000)              //  BMS״̬
	 {
		 if((Para.BmsBattStatus0 & 0x1000) == 0x1000)            //  ���ʹ��
	   {
			 if((Para.BmsBattStatus0 & 0x100)== 0x100)             // ��س���      
	     {
		     bme_state =  CANNOT_CHARGE_CAN_DISCHARGE;
	     }
			 else
			 {
				 bme_state =  CAN_CHARGE_CAN_DIDCHARGEE;                 // 0�ɳ�ɷ�  
			 }
	 
	   }	 
		 if((Para.BmsBattStatus0 & 0x400) == 0x400)          // �ŵ�ʹ��
	   {		    
			  if((Para.BmsBattStatus0 & 0x40) == 0x40)           // ��طſ�            
	      {
		      bme_state =  CAN_CHARGE_CANNOT_DISCHARGE;           // 2 ���ɷſɳ� 
	      }
				else
				{
					  bme_state =  CAN_CHARGE_CAN_DIDCHARGEE;     // 0�ɳ�ɷ�  
				}			 
	   }
	 }	 
	 else  
	 {
		   bme_state = CANNOT_CHARGE_CANNOT_DISCHARGE;        // 3 ���ɳ䲻�ɷ�;
	 }	 
	 return bme_state;
}	 

 
/*******************************************************************************
* Function Name  : uint16_t  Restrictive_Batt(void)
* Description    : ���ݵ�ع�������,�Ե�صĳ�ŵ�����
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
* Description    : ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  LoadFirst(void)
{

// =============================================================	 
// ����\����\�����趨Ϊ��ֵ��Ϊ���ģʽ���趨Ϊ��ֵ��Ϊ�ŵ�ģʽ	
// =============================================================	 

   Surplus = ActualValue.MpptSysPower - ActualValue.PcsLoadActivePower;
	
	
//	 Data32_Temp = (int32_t)(0);// ActualValue.BattPowerMax;   
//	 Mppt.Pwr = (uint16_t)(ActualValue.PcsLoadActivePower + ActualValue.BattPowerMax);  
	
	 if(Surplus > 0)                                                          // �������ʱ,PV���ȹ�Ӧ���أ�������
	 { 
			if((BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE) || (BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE))  // �ɳ�ɷ�,���ɷſɳ� 
			{
				if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)                   // ������,AC�㹦��ģʽ,������Ϊ���
				{
					 PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
					 Data32_Temp = -(int32_t)ActualValue.BattPowerMax;  
					 Mppt.Pwr = (uint16_t)(ActualValue.PcsLoadActivePower + ActualValue.BattPowerMax);            
				}
				else
				{ // PV��������ʱ��PV���ȹ�Ӧ���أ�������
					 if(Surplus > ActualValue.BattPowerMax)                        // �����غ�ʣ�๦�ʴ��ڳ�������
					 {
						  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
						  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;          // ������繦�ʳ���,ֵΪ��   
					 }
					 else
					 {
						  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
						  Data32_Temp = -(int32_t)Surplus;                           // �����غ�ʣ�๦�ʳ���,ֵΪ�� 
					 }

					 Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;             // PVȫ����
				}
			}
			else if(BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE)                 // �ѳ���,���ɳ�ɷ�
			{
				if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)                // ������
				{
					 PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
					 Data32_Temp = -(int32_t)(0);   
					 Mppt.Pwr = (uint16_t)ActualValue.PcsLoadActivePower; 
				}
				else
				{
					 if(ActualValue.MpptSysPower > ActualValue.PcsChargePowerMax)     // ������ʴ���PCS�����
					 {	// AC�㹦�ʿ��Ʋ��˵�طŵ�,DC�㹦�� = 0,MPPT���ʺ���,��س����󲻷� 		
              PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 						 
						  Data32_Temp  = 0;                                             // ��MPPT��ǰ���ʹ����غͻ������� 
							Mppt.Pwr = (uint16_t)ActualValue.PcsChargePowerMax;           // ����MPPT����ΪPCS�����
					 }
					 else
					 {
						  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
						  Data32_Temp = 0;                                              // ��MPPT����ʹ����غͻ�������        
							Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;             // ����MPPT����ΪPCS�����
					 }
				}
			}
		}
		else     // �������ʱ,PV���ʲ����㸺��ʱ������Զ��ŵ�
		{
				if(BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE || BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE)        // ���δ�Ž�ֹ
				{
					  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
					  Data32_Temp = -(int32_t)Surplus;                                 // ��ز���MPPT���㲿�֣�ֵΪ��
				
				}
				else if(BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE)                   // �ѷŽ�ֹ,���������
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
* Description    : ��������-����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LoadFirst_PowerConservation(void)
{
	Surplus = ActualValue.MpptSysPower - ActualValue.PcsLoadActivePower;
	if(Surplus > 0)                                                        // �������ʱ
			 { 
					if(BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE)
					{
							if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)               // ������
							{
								  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;
								  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
									Mppt.Pwr = (uint16_t)(ActualValue.PcsLoadActivePower + ActualValue.BattPowerMax); 
							}
							else
							{
								  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;
									if(Surplus > Para.BattMaxPwr)                             // ʣ�๦�ʴ��ڳ�������
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
				else if(BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE)                  // �ѳ���
				{
					 PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;
					 if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)                // ������
					 {
						  Data32_Temp = -(int32_t)0;
							Mppt.Pwr = (int16_t)ActualValue.PcsLoadActivePower;
					 }
					 else
					 {
							if(ActualValue.MpptSysPower > ActualValue.PcsChargePowerMax)   // ������ʴ���PCS�����
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
			else //�������ʱ,���Ե�س��
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
* Description    : �������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  BattFirst(void)
{

	  Surplus = ActualValue.MpptSysPower - ActualValue.BattPowerMax;
	
		if(Surplus > 0)                                                     // �������ʱ,���ȳ��أ���繩����
		{ 
				if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)               // ������
				{
						 if((BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE) || (BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE))  // �ɳ�ɷ�,���ɷſɳ� 
						 {							 
							  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;   
							  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
	
								Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax; 
						 }
						 else
						 {
							  PcsData.Value[1] = PCS_AC_CONS_POWER_RUN;                              //  ACģʽ   							 
							  Data32_Temp = 0;         
								Mppt.Pwr = (uint16_t)ActualValue.PcsLoadActivePower; 						 
						 }
				}
				else
				{
					  if((BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE) || (BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE))  // �ɳ�ɷ�,���ɷſɳ� 
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
		else                                                                 // �������ʱ,PV���ȸ���س��
		{
			if((BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE) || (BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE))  // �ɳ�ɷ�,���ɷſɳ� 
			{
				if(Para.PVGridBothChargeEnable == 1)                             // �������ͬʱ���
				{
					  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;   
					  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;                       
						Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
				}
				else                                                             // �����������ͬʱ���
				{
					  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;   
					  Data32_Temp = -(int32_t)ActualValue.MpptSysPower;              
						Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
				}
								
			}
			else
			{
				    if(Para.Anti_Reflux_Enable == ANTI_REFLUX_ENABLE)           // ������
						{
							  PcsData.Value[1] = PCS_AC_CONS_POWER_RUN;                              //  ACģʽ   
				        Data32_Temp = 0;                      
						    Mppt.Pwr = (uint16_t)ActualValue.PcsLoadActivePower;			
						}
						else
						{
							  PcsData.Value[1] = PCS_AC_CONS_POWER_RUN;                              //  ACģʽ   							
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
* Description    : ����ģʽ
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
* Description    : �������
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
	
	
		if(Surplus > 0)                  // PV���ʴ��ڸ��غͳ�繦��ʱ�����ӵ���ȡ�磬PV�����ز��Ե�س��
		{
			  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
		}
		else if(Surplus1 > 0)           // (PV����+�������޹���)����(���ع���+��繦��ʱ)ʱ,������PVͬʱ���縺�ز��Ե�س��
		{
			  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
		}
		else if(Surplus2 > 0)           // (PV����+�������޹���)���ڸ��ع���ʱ,������PV���ȹ�����,������.
		{
			  Data32_Temp = -(int32_t)(ActualValue.MpptSysPower + ActualValue.GridLimitUp - ActualValue.PcsLoadActivePower);
			  if(Data32_Temp < -(uint32_t)ActualValue.BattPowerMax)
				{
					 Data32_Temp = -(uint32_t)ActualValue.BattPowerMax;
				}
		}
		else if(Surplus2 < 0)           // (PV����+�������޹���)С�ڸ��ع���ʱ,����,PV�͵��ͬʱ������
		{
			  Data32_Temp = (int32_t)ActualValue.PcsLoadActivePower;    // ���Ҫ�ŵ�,ֵΪ��
		}
*/
		float Surplus = ActualValue.MpptSysPower - ActualValue.PcsLoadActivePower;
		float Surplus1 = Surplus + ActualValue.GridLimitLow;
		//float Surplus1 = ActualValue.MpptSysPower + ActualValue.GridLimitLow - ActualValue.PcsLoadActivePower;
		float Surplus2 = Surplus + ActualValue.GridLimitUp;
		//float Surplus2 = ActualValue.MpptSysPower + ActualValue.GridLimitUp - ActualValue.PcsLoadActivePower;
		if (Surplus > 0) //��������㸺�أ��������
			Data32_Temp = -(Surplus > ActualValue.BattPowerMax?ActualValue.BattPowerMax:Surplus);
		else if (Surplus1 > 0) //���+�������޿����㸺�أ��������
			Data32_Temp = -Surplus1;
		else if (Surplus2 > 0) //���+�������޲����㵫�������������ޣ�����͵��������أ������
			Data32_Temp = 0;
		else //���س������+�������ޣ�ʹ�õ�ع���
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
* Description    : EMSģʽ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  EMS(void)
{

}

/*******************************************************************************
* Function Name  : void  AlonePVMode(void)
* Description    : ��PVģʽ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//1. �޵���ʱ������ͣ��ťת��ON,����LCD�����������ܿ��������������뵥PVģ
//   ʽ����ʱPVֻ�Ե�س�磨���30A���������������������ࡣ
//2. �޵���ʱ���ڵ�PVģʽʱ�ֶ���LCD���Ͽ��������ܿ�������������ģʽ��
//3. ����ģʽ�ŵ������Ƿѹ�澯��ʱ�����ܿ������ر���䣬�Զ��л�����PVģ
//   ʽ������س䵽�趨��ѹ(��PVת����)ʱ�Զ�ת��������
//4. ��PVģʽ��⵽�����ָ����������ܿ������Զ����벢��ģʽ��
void  AlonePVMode(void)
{

	 
		if(BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE)
		{
			  PcsData.Value[1] = PCS_DC_CONS_CUR_RUN; 
			  Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
				Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;

		}
		else if(BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE)        // �ѳ���,ת����
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
* Description    : ����ģʽ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  GridOff(void)
{
		Data32_Temp = Para.BKPVolt * 1000;
	  int32_t Data32_Temp2 = Para.BKPFreq * 1000;
	  if(BMSCheck() == CANNOT_CHARGE_CAN_DISCHARGE) //���ɳ䣬MPPT����д0
			Mppt.Pwr = 0.0f;
		else if(BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE && Para.PcsDcPL > 100) //��ز��ɷ��Ҹ��ع��ʴ��ڹ�����ʣ�ϵͳͣ��
			SysControl.SysFlag = STA_STOP;
	  else //�ɳ�ɷţ�MPPT��������
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
* Description    : �ͻ�ģʽ(Ҫ����)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  DieselEngineMode(void)
{
	
    Surplus = ActualValue.MpptSysPower - ActualValue.BattPowerMax;


		if((BMSCheck() == CAN_CHARGE_CANNOT_DISCHARGE) || (BMSCheck() == CAN_CHARGE_CAN_DIDCHARGEE))              // ���״̬��0�ɳ�ɷ� 1 ���ɳ�ɷ�  2 ���ɷſɳ� 3 ���ɳ䲻�ɷ�
		{ 
			    PcsData.Value[1] = PCS_DC_CONS_CUR_RUN;
					if(ActualValue.MpptSysPower > ActualValue.BattPowerMax)       // PV���ʴ��ڳ�繦��
					{
							Data32_Temp = -(int32_t)ActualValue.BattPowerMax;
					}
					else                                                          // PV����С�ڳ�繦��
					{
							Data32_Temp = (int32_t)Surplus;                           // ��ʱSurplusΪ��
					}	
									
					Mppt.Pwr = (uint16_t)ActualValue.MpptSysPowerMax;
		}
    else
		{
	    	//STOPDE();                                              // ֹͣ�ͻ���ת������ģʽ
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
* Description    : �������Զ��л�ģʽ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  GridAutoSwitchingMode(void)
{

}


/*******************************************************************************
* Function Name  : void  FailureMode(void)
* Description    : ����ģʽ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  FailureMode(void)
{
		SYSTEM_FAILURE_INDICATION_ON;    // ϵͳ����ָʾ��
}


/*******************************************************************************
* Function Name  : void  PermanentFailureMode(void)
* Description    : ���ù���ģʽ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  PermanentFailureMode(void)
{
		SYSTEM_FAILURE_INDICATION_ON;    // ϵͳ����ָʾ��
}



/*******************************************************************************
* Function Name  : void  PermanentFailureMode(void)
* Description    : ���ص���
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
		PcsData.Value[1] = Para.PcsRunMode;     // PCS����ģʽ
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
* Description    : Զ�̵���
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
		PcsData.Value[1] = Para.PcsRunMode;     // PCS����ģʽ
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
* Description    : ���õ��
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



