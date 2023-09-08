/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/

#include "main.h"
 #include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/sockets.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId EEPROMTASKHandle;
osThreadId RTCTASKHandle;
osThreadId DIDOTASKHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId DEALUASRTHandle;
osThreadId CAN1TASKHandle;
osThreadId CAN2TASKHandle;
osThreadId PCSPHYTASKHandle;
osThreadId BSMPHYTASKHandle;
osTimerId EMTimerHandle;

extern void rtc_task(void const *argument);
extern void eeprom_task(void const *argument);
extern void dido_task(void const *argument);
extern void LcdDataGet(void);
extern void StartDefaultTask(void const * argument);
extern void BsmDatPut(void);
extern void PcsDatPut(void);

void dealusart_task(void const * argument);
extern void can1_task(void const * argument);
extern void can2_task(void const * argument);
extern void PcsPhyTask(void const *argument);
extern void BsmPhyTask(void const *argument);
extern void EM_TimerCallback(void const *arg);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

//	uint32_t exec;
//	osTimerDef(em_timer, EM_TimerCallback);
//  EMTimerHandle = osTimerCreate(osTimer(em_timer), osTimerPeriodic, &exec);
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 200);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of DEALUASRT */
  osThreadDef(DEALUASRT, dealusart_task, osPriorityNormal, 0, 128);
  DEALUASRTHandle = osThreadCreate(osThread(DEALUASRT), NULL);


  /* definition and creation of CAN2TASK */
  osThreadDef(CAN2TASK, can2_task, osPriorityNormal, 0, 128);
  CAN2TASKHandle = osThreadCreate(osThread(CAN2TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
 /* definition and creation of DEALUASRT */
  osThreadDef(EEPROMTASK, eeprom_task, osPriorityNormal, 0, 128);
  EEPROMTASKHandle = osThreadCreate(osThread(EEPROMTASK), NULL);
	
 /* definition and creation of DEALUASRT */
  osThreadDef(RTCTASK, rtc_task, osPriorityNormal, 0, 128);
  RTCTASKHandle = osThreadCreate(osThread(RTCTASK), NULL);
	
 /* definition and creation of DEALUASRT */
//  osThreadDef(DIDOTASK, dido_task, osPriorityNormal, 0, 64);
//  DIDOTASKHandle = osThreadCreate(osThread(DIDOTASK), NULL);	

 /* definition and creation of DEALUASRT */
 /* definition and creation of DEALUASRT */
  osThreadDef(PCSPHYTASK, PcsPhyTask, osPriorityNormal, 0, 512);
  PCSPHYTASKHandle = osThreadCreate(osThread(PCSPHYTASK), NULL);
  /* USER CODE END RTOS_THREADS */
  /* definition and creation of DEALUASRT */
  osThreadDef(BSMPHYTASK, BsmPhyTask, osPriorityNormal, 0, 512);
  BSMPHYTASKHandle = osThreadCreate(osThread(BSMPHYTASK), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
//  MX_LWIP_Init();

	USARTB_RX_EN;
	USARTA_RX_EN;
	USARTC_RX_EN;

			
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12);//¹¤×÷ÉÁµÆ
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);//硬件喂狗
		LcdDataGet();
		osDelay(100);	        // lxf
		SysStateManage();
        PcsDatPut();
		BsmDatPut();

  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

