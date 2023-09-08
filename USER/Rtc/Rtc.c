#include "main.h"
 


void RTC_Control(void);
void Electric_Quantity1_Cal(void);
 
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef DateToUpdate;
extern Struct_FParameter ActualValue; 
extern Struct_InvParameter Para;
extern osThreadId DIDOTASKHandle;
calendar_obj calendar;
 
 
uint8_t  Last_Time_Seconds = 0;
uint8_t  Last_Time_Minutes = 0;
uint8_t  Last_Time_Hours = 0;

void rtc_task(void *argument)
{
//void RTC_Control(void) 
//{
uint8_t  time_deta;
	  
   
	 
	 for(;;)
   {
		if (Para.CorrectTime){
			DateToUpdate.Year = Para.Year % 100;
		  DateToUpdate.Month = Para.Month;
		  DateToUpdate.Date = Para.Day ;
			DateToUpdate.WeekDay = Para.Weekday;
		  sTime.Hours = Para.Hour;
		  sTime.Minutes = Para.Minute;
		  sTime.Seconds = Para.Second;
			HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
		}
		else{
	  HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);              //喂硬件狗
		osDelay(100);	 
	//  一定要要先读Time然后再度Date，否者会出错,即使不需要Date数据，也要读Date否则也会出错
	//		HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);    // 设置时间
			 //使用BCD码，便于做判断和移位
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); 
			HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN); 
	//	  calendar.Year = DateToUpdate.Year;
	//	  calendar.Month = DateToUpdate.Month;
	//	  calendar.WeekDay = DateToUpdate.WeekDay;
	//	  calendar.Date = DateToUpdate.Date;
	//	  calendar.Hours = sTime.Hours;
	//	  calendar.Minutes = sTime.Minutes;
	//	  calendar.Seconds = sTime.Seconds;
			Para.Year = DateToUpdate.Year + 2000;
		  Para.Month = DateToUpdate.Month;
		  Para.Day = DateToUpdate.Date;
			Para.Weekday = DateToUpdate.WeekDay;
		  Para.Hour = sTime.Hours;
		  Para.Minute = sTime.Minutes;
		  Para.Second = sTime.Seconds;
		}
	  time_deta = calendar.Seconds - Last_Time_Seconds;
	  if(time_deta > 30)
		{
//			 	 osSignalSet(EEPROMTASKHandle,0x01);            // 数据修改,发送任务通知存储
			   Last_Time_Seconds = calendar.Seconds;
		}
	  //Electric_Quantity1_Cal();
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);                 //喂硬件狗
	}
}


void Electric_Quantity1_Cal()
{
uint8_t  time_deta;	
	
	time_deta = calendar.Seconds - Last_Time_Seconds;
	if(time_deta >= 1)  // 一秒钟用电量
	{
	   Para.Mppt_electric_quantity1_s += (uint16_t)(ActualValue.MpptSysPower * 0.0027777778f * 0.0001f);  // KW*h
	   Last_Time_Seconds = calendar.Seconds;
	   osSignalSet(DIDOTASKHandle,0x07); 		
	}
	
	time_deta = calendar.Minutes - Last_Time_Minutes;
	if(time_deta >= 1)  // 一分钟用电量
	{
		Para.Mppt_electric_quantity1_m += Para.Mppt_electric_quantity1_s ;
		Last_Time_Minutes = calendar.Minutes;
	}
 
	
	time_deta = calendar.Hours - Last_Time_Hours;
	if(time_deta >= 1)  // 一小时钟用电量
	{
		 Para.Mppt_electric_quantity1_h += Para.Mppt_electric_quantity1_s;
		 Wirte_Over_Flag = 0;                      // 写24C2456标志,一小时存储一次
		 Last_Time_Hours = calendar.Hours;
	}
	
}




