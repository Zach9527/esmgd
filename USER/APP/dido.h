#ifndef  DIDO_H
#define  DIDO_H

#include "main.h"
 

#define EMERGENCY_STOP_INPUT            HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)   // ��ͣ���룬����
#define SURGE_SERIES_FEEDBACK           HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14)   // ��ӿ�����������룬����
#define GRID_CIRCUIT_BREAK_FEEDBACK     HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)   // ������·���������룬����
#define LOAD_CIRCUIT_BREAK_FEEDBACK     HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)    // ���ض�·���������룬����
#define EXTERN_FAN_FEEDBACK             HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)    // �ⲿ���ȷ���������
#define BYPASS_CIRCUIT_BREAK_FEEDBACK   HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)    // ��·��·���������룬����
#define RESERVER_INPUT1                 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)    // Ԥ������1
#define RESERVER_INPUT2                 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)    // Ԥ������2

#define FAN_IN_POWER_CABINET_ON         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET)   // ��Դ������
#define FAN_IN_POWER_CABINET_OFF        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET) // ��Դ������

#define SYSTEM_FAILURE_INDICATION_ON    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET)   // ϵͳ����ָʾ��
#define SYSTEM_FAILURE_INDICATION_OFF   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET) // ϵͳ����ָʾϨ

#define SYSTEM_OPERATION_INDICATION_ON  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET)   // ϵͳ����ָʾ��
#define SYSTEM_OPERATION_INDICATION_OFF HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET) // ϵͳ����ָʾϨ

#define AIR_POWER_CONTROL_ON            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET)   // �յ�������ƿ�
#define AIR_POWER_CONTROL_OFF           HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET) // �յ�������ƹ�

#define HIGH_POWER_BOX_CONTROL_ON       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET)    // ��ѹ�乩����ƿ�
#define HIGH_POWER_BOX_CONTROL_OFF      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET)  // ��ѹ�乩����ƹ�

#define EXTERN_FAN_POWER_CONTROL_ON     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET)    // ������������ƿ�
#define EXTERN_FAN_POWER_CONTROL_OFF    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET)  // ������������ƹ�

#define RESERVER_OUTPUT1_ON             HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET)    // �������1
#define RESERVER_OUTPUT1_OFF            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET)  // �������1

#define RESERVER_OUTPUT2_ON             HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET)    // �������2
#define RESERVER_OUTPUT2_OFF            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET)  // �������2




#endif
