#ifndef  RTC_H
#define  RTC_H

#include "main.h"
#include "cmsis_os.h"
#include "string.h"
 typedef struct 
{
	uint8_t Hours;
	uint8_t Minutes;
	uint8_t Seconds;			
	//公历日月年周
	uint16_t Year;
	uint8_t  Month;
	uint8_t  WeekDay;
	uint8_t  Date;		 
}calendar_obj;					 
//extern calendar_obj calendar;	//日历结构体



#endif



