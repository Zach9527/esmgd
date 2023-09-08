#include "main.h"
 

void BmsData_Rx(void);
CAN_RxHeaderTypeDef CAN1_txHeader;
CAN_RxHeaderTypeDef CAN1_rxHeader;
CAN_TxHeaderTypeDef Tx1Message; 

uint8_t CAN1_RX_BUF[8];	
uint8_t can1_rxdata[8],can1_txdata[8];
uint32_t TxMailbox;

extern struct_MPPT Mppt;
/*
      按BMS CAN 通信协议 2.0
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
  {         
     if(hcan->Instance == CAN1)
     {
         if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_rxHeader, CAN1_RX_BUF)==HAL_OK)//此处的fifo0也要注意
          {
               HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
               osSignalSet(CAN1TASKHandle,0x01);
          }              
     }
		 if(hcan->Instance == CAN2)
     {
         if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_rxHeader, CAN2_RX_BUF)==HAL_OK)//此处的fifo0也要注意
          {
               HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
               //osSignalSet(CAN2TASKHandle,0x01);
               Mppt.Reissued = 1;
          }              
     }
 }
void can1_task(void *argument)
{
  while(1)
  {		 		  
  		 osSignalWait(0x01,osWaitForever);
       memcpy(can1_rxdata,CAN1_RX_BUF,8);
//		   BmsData_Rx();
	}
}


