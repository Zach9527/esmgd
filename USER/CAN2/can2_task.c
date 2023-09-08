#include "main.h"
 

CAN_RxHeaderTypeDef CAN2_rxHeader;
CAN_RxHeaderTypeDef CAN2_txHeader;
uint8_t CAN2_RX_BUF[8];	
uint8_t can2_rxdata[8],can2_txdata[8];
CAN_RxHeaderTypeDef CAN2_Header;
CAN_TxHeaderTypeDef Tx2Message; 
uint32_t Tx2Mailbox;

extern void MpptDataGet(void);

/******************************************
*****  预留另外一组MPPT(3个)通讯 *****
******************************************/
void can2_task(void *argument)
{
   while(1)
   {	 
        osDelay(100);	
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);         //喂硬件狗
        memcpy(can2_rxdata,CAN2_RX_BUF,8);		
        MpptDataGet();
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);         //喂硬件狗

  }
}

