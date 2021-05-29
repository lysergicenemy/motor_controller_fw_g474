/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_lptim.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx.h"
#include "stm32g4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "digitalFilters.h"
#include "utils.h"
#include "model_bldc.h"
#include "bldcpwm.h"
#include "bldc.h"
#include "foc.h"
#include "dataLog.h"
#include "stm32g4xx_it.h"
#include "ntc.h"              // Include header for NTC object
#include "hall.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC_10MHz_Pin LL_GPIO_PIN_0
#define OSC_10MHz_GPIO_Port GPIOF
#define OSC_10MHzF1_Pin LL_GPIO_PIN_1
#define OSC_10MHzF1_GPIO_Port GPIOF
#define I_PHASE_A_Pin LL_GPIO_PIN_0
#define I_PHASE_A_GPIO_Port GPIOA
#define I_PHASE_B_Pin LL_GPIO_PIN_1
#define I_PHASE_B_GPIO_Port GPIOA
#define I_PHASE_C_Pin LL_GPIO_PIN_2
#define I_PHASE_C_GPIO_Port GPIOA
#define V_PHASE_A_Pin LL_GPIO_PIN_4
#define V_PHASE_A_GPIO_Port GPIOC
#define V_PHASE_B_Pin LL_GPIO_PIN_7
#define V_PHASE_B_GPIO_Port GPIOA
#define V_PHASE_C_Pin LL_GPIO_PIN_6
#define V_PHASE_C_GPIO_Port GPIOA
#define V_INPUT_Pin LL_GPIO_PIN_3
#define V_INPUT_GPIO_Port GPIOA
#define I_INPUT_Pin LL_GPIO_PIN_0
#define I_INPUT_GPIO_Port GPIOB
#define TEMP_SENSOR_Pin LL_GPIO_PIN_5
#define TEMP_SENSOR_GPIO_Port GPIOA
#define OCP_Pin LL_GPIO_PIN_11
#define OCP_GPIO_Port GPIOB
#define PWM_BH_Pin LL_GPIO_PIN_12
#define PWM_BH_GPIO_Port GPIOB
#define PWM_BL_Pin LL_GPIO_PIN_13
#define PWM_BL_GPIO_Port GPIOB
#define PWM_CH_Pin LL_GPIO_PIN_14
#define PWM_CH_GPIO_Port GPIOB
#define PWM_CL_Pin LL_GPIO_PIN_15
#define PWM_CL_GPIO_Port GPIOB
#define ENABLE_Pin LL_GPIO_PIN_11
#define ENABLE_GPIO_Port GPIOC
#define PWM_AH_Pin LL_GPIO_PIN_8
#define PWM_AH_GPIO_Port GPIOA
#define PWM_AL_Pin LL_GPIO_PIN_9
#define PWM_AL_GPIO_Port GPIOA
#define BRAKE_PWM_Pin LL_GPIO_PIN_15
#define BRAKE_PWM_GPIO_Port GPIOA
#define LED_FAULT_Pin LL_GPIO_PIN_13
#define LED_FAULT_GPIO_Port GPIOC
#define LED_STATUS_Pin LL_GPIO_PIN_14
#define LED_STATUS_GPIO_Port GPIOC
#define HALL_PHASE_A_Pin LL_GPIO_PIN_7
#define HALL_PHASE_A_GPIO_Port GPIOB
#define HALL_PHASE_B_Pin LL_GPIO_PIN_8
#define HALL_PHASE_B_GPIO_Port GPIOB
#define HALL_PHASE_C_Pin LL_GPIO_PIN_9
#define HALL_PHASE_C_GPIO_Port GPIOB
#define SPEED_PWM_Pin LL_GPIO_PIN_5
#define SPEED_PWM_GPIO_Port GPIOB
#define DIR_Pin LL_GPIO_PIN_4
#define DIR_GPIO_Port GPIOA

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

#define FLASH_CONFIG_ADR_START 0x0807F000
#define FLASH_CONFIG_ADR_END (FLASH_CONFIG_ADR_START + sizeof(flashData_t))
#define FLASH_CONFIG_PG_NMB 127U
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
