/**
  ******************************************************************************
  * File Name          : LPTIM.c
  * Description        : This file provides code for the configuration
  *                      of the LPTIM instances.
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
#include "lptim.h"

/* USER CODE BEGIN 0 */
void lptim_start(uint32_t period)
{
  LL_LPTIM_Enable(LPTIM1);
  LL_LPTIM_SetAutoReload(LPTIM1, period);
  LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
}

/* USER CODE END 0 */

/* LPTIM1 init function */
void MX_LPTIM1_Init(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**LPTIM1 GPIO Configuration  
  PB5   ------> LPTIM1_IN1
  PB7   ------> LPTIM1_IN2 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV1);
  LL_LPTIM_SetPolarity(LPTIM1, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
  LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_EXTERNAL);
  LL_LPTIM_TrigSw(LPTIM1);
  LL_LPTIM_SetInput1Src(LPTIM1, LL_LPTIM_INPUT1_SRC_GPIO);
  LL_LPTIM_SetInput2Src(LPTIM1, LL_LPTIM_INPUT2_SRC_GPIO);
  LL_LPTIM_SetEncoderMode(LPTIM1, LL_LPTIM_ENCODER_MODE_RISING_FALLING);
  LL_LPTIM_EnableEncoderMode(LPTIM1);
}

/* USER CODE BEGIN 1 */

// LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
// LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV1);
// LL_LPTIM_SetPolarity(LPTIM1, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
// LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
// LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_EXTERNAL);
// LL_LPTIM_SetAutoReload(LPTIM1, 4095);
// LL_LPTIM_TrigSw(LPTIM1);
// LL_LPTIM_SetInput1Src(LPTIM1, LL_LPTIM_INPUT1_SRC_GPIO);
// LL_LPTIM_SetInput2Src(LPTIM1, LL_LPTIM_INPUT2_SRC_GPIO);

// LL_LPTIM_Enable(LPTIM1);
// LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
// LL_LPTIM_EnableEncoderMode(LPTIM1);
// LL_LPTIM_SetEncoderMode(LPTIM1, LL_LPTIM_ENCODER_MODE_RISING_FALLING);

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
