#ifndef  DIDO_H
#define  DIDO_H

#include "main.h"
 

#define EMERGENCY_STOP_INPUT            HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)   // 急停输入，常闭
#define SURGE_SERIES_FEEDBACK           HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14)   // 浪涌串联反馈输入，常闭
#define GRID_CIRCUIT_BREAK_FEEDBACK     HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)   // 电网断路器反馈输入，常开
#define LOAD_CIRCUIT_BREAK_FEEDBACK     HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)    // 负载断路器反馈输入，常开
#define EXTERN_FAN_FEEDBACK             HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)    // 外部风扇反馈，常开
#define BYPASS_CIRCUIT_BREAK_FEEDBACK   HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)    // 旁路断路器反馈输入，常开
#define RESERVER_INPUT1                 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)    // 预留输入1
#define RESERVER_INPUT2                 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)    // 预留输入2

#define FAN_IN_POWER_CABINET_ON         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET)   // 电源柜风机开
#define FAN_IN_POWER_CABINET_OFF        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET) // 电源柜风机关

#define SYSTEM_FAILURE_INDICATION_ON    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET)   // 系统故障指示亮
#define SYSTEM_FAILURE_INDICATION_OFF   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET) // 系统故障指示熄

#define SYSTEM_OPERATION_INDICATION_ON  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET)   // 系统运行指示亮
#define SYSTEM_OPERATION_INDICATION_OFF HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET) // 系统运行指示熄

#define AIR_POWER_CONTROL_ON            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET)   // 空调供电控制开
#define AIR_POWER_CONTROL_OFF           HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET) // 空调供电控制关

#define HIGH_POWER_BOX_CONTROL_ON       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET)    // 高压箱供电控制开
#define HIGH_POWER_BOX_CONTROL_OFF      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET)  // 高压箱供电控制关

#define EXTERN_FAN_POWER_CONTROL_ON     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET)    // 柜外风机供电控制开
#define EXTERN_FAN_POWER_CONTROL_OFF    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET)  // 柜外风机供电控制关

#define RESERVER_OUTPUT1_ON             HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET)    // 保留输出1
#define RESERVER_OUTPUT1_OFF            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET)  // 保留输出1

#define RESERVER_OUTPUT2_ON             HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET)    // 保留输出2
#define RESERVER_OUTPUT2_OFF            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET)  // 保留输出2




#endif
