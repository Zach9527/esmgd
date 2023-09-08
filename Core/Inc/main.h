/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "SysControl.h"
#include "Eeprom.h"
#include "EMS.h"
#include "dido.h"
#include "dealusart_task.h"
#include "rtc.h"
//#include "phy.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define  USARTC_TX_EN   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)  // BMS
#define  USARTC_RX_EN   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)
#define  USARTB_TX_EN   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET)
#define  USARTB_RX_EN   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET)


#define  USARTA_TX_EN   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET)
#define  USARTA_RX_EN   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET)  //  PCS
#define  ADDR_24LCxx_Write 0xA0
#define  ADDR_24LCxx_Read 0xA1 
#define  ADC1_CHANNEL_CNT 6// 																		
#define  ADC1_CHANNEL_FRE 100	//

#define PCSRTINT  2000 // PCS数据读取时间为2秒
#define TOUT      1000// 超时时间
#define MPPTTINT  2000  //2000 // MPPT读取间隔时间


/* static IP address: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   60
#define IP_ADDR3   10
/* net mask */
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0
/* gateway address */
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   60
#define GW_ADDR3   1
/* net port */
#define NETPORT    502

/* DCDC FRAME*/
#define DCDC_FRAME      0x18D001FA

#define DCDC_DATA_GET1  0x18D1FA01
#define DCDC_DATA_GET2  0x18D2FA01
#define DCDC_DATA_GET3  0x18D3FA01
#define DCDC_DATA_GET4  0x18D4FA01
#define DCDC_DATA_GET5  0x18D5FA01
#define DCDC_DATA_GET6  0x18D6FA01
#define DCDC_DATA_GET7  0x18D7FA01
#define DCDC_DATA_GET8  0x18D8FA01
#define DCDC_DATA_GET9  0x18D9FA01
#define DCDC_DATA_GET10 0x18DAFA01
#define DCDC_DATA_GET11 0x18DBFA01
#define DCDC_DATA_GET12 0x18DCFA01
#define DCDC_DATA_GET13 0x18DDFA01



#define DCDC_PGN       4
typedef enum
{
    DCDC_MODEL1 = 0,
    DCDC_MODEL2,
    DCDC_MODEL3,
    DCDC_MODEL4,
    DCDC_MODEL5,
    DCDC_MODEL6,
}DCDC_MODEL;





typedef struct {
    uint32_t Ia;
    uint32_t Ib;   
    uint32_t Ic;   
}Struct_Current;


typedef struct {
    uint32_t Ua;
    uint32_t Ub;   
    uint32_t Uc;   
}Struct_Voltage;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern osThreadId DEALUASRTHandle;
extern osThreadId USARTCHandle;
extern osThreadId CAN1TASKHandle;
extern osThreadId CAN2TASKHandle;
extern osThreadId DITASKHandle;
extern osThreadId DOTASKHandle;
extern osThreadId EEPROMTASKHandle;
extern osThreadId RTCTASKHandle;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern CAN_RxHeaderTypeDef CAN2_rxHeader;
extern CAN_RxHeaderTypeDef CAN2_txHeader;

extern uint8_t CAN2_RX_BUF[8];





#define TCPSIGNAL_TIMEOUT           5000





/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
