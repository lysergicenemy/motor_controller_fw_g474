/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
// ----- sensor type defs -------------//
#define HALL 0
#define ENCODER_ABZ 1
#define SENSORLESS 2

uint8_t sensorType = HALL;

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA15   ------> S_TIM2_CH1
     PB9   ------> S_TIM8_CH3
*/
void MX_GPIO_Init(void)
{

  //LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

  /**/
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BRAKE_PWM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(BRAKE_PWM_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_FAULT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_FAULT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SPEED_PWM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_10;
  LL_GPIO_Init(SPEED_PWM_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // HALL
  GPIO_InitStruct.Pin = HALL_PHASE_A_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(HALL_PHASE_A_GPIO_Port, &GPIO_InitStruct);
  /**/
  GPIO_InitStruct.Pin = HALL_PHASE_B_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(HALL_PHASE_B_GPIO_Port, &GPIO_InitStruct);
  /**/
  GPIO_InitStruct.Pin = HALL_PHASE_C_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(HALL_PHASE_C_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  // /* External interrupts */
  // LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE7);
  // LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE8);
  // LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE9);
  // /**/
  // EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  // EXTI_InitStruct.LineCommand = ENABLE;
  // EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  // EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  // LL_EXTI_Init(&EXTI_InitStruct);
  // /**/
  // EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_8;
  // EXTI_InitStruct.LineCommand = ENABLE;
  // EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  // EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  // LL_EXTI_Init(&EXTI_InitStruct);
  // /**/
  // EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_9;
  // EXTI_InitStruct.LineCommand = ENABLE;
  // EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  // EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  // LL_EXTI_Init(&EXTI_InitStruct);
  // /* EXTI interrupt init*/
  // NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  // NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 2 */
// if (sensorType == HALL)
// {
//   /**/
//   GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
//   GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//   LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//   /**/
//   GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
//   GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//   LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//   /**/
//   GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
//   GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//   LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
// }
// if (sensorType == ENCODER_ABZ)
// {
//   /**/

// }
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
