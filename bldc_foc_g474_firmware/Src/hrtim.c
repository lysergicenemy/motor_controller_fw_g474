/**
  ******************************************************************************
  * File Name          : HRTIM.c
  * Description        : This file provides code for the configuration
  *                      of the HRTIM instances.
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
#include "hrtim.h"

/* USER CODE BEGIN 0 */

void hrtim_start(volatile float pwmFreq, volatile float deadtime_ns, uint32_t pwmPeriod_adcTrig_ratio, volatile float *halfPeriod)
{
  *halfPeriod = (340000000.0f / pwmFreq) / 4.0f;
  /* PWM frequency = HRTIM_CLK / AutoReloadVal * 2(up-down mode) */
  //foc1.data.halfPwmPeriod = (340000000.0f / foc1.config.pwmFreq) / 4.0f; // 4 - upDown mode = 2, halfPeriod = 2
  /* PWM period per ADC trigger event ratio. 0 - means 1 PWM period per 1 ADC trigger */
  LL_HRTIM_SetADCPostScaler(HRTIM1, LL_HRTIM_ADCTRIG_2, pwmPeriod_adcTrig_ratio); // how many PWM periods for 1 ADC sampling
  /* PWM period */
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)*halfPeriod * 2.0f);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)*halfPeriod * 2.0f);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)*halfPeriod * 2.0f);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)*halfPeriod * 2.0f);
  /* DeadTime. if DTPRSC - div2(mul2), t_DTG = 11.76 ns */
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(deadtime_ns / 11.76f));
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(deadtime_ns / 11.76f));
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(deadtime_ns / 11.76f));
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(deadtime_ns / 11.76f));
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(deadtime_ns / 11.76f));
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(deadtime_ns / 11.76f));

  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_C | LL_HRTIM_TIMER_D);
  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | LL_HRTIM_OUTPUT_TC1 |
                                    LL_HRTIM_OUTPUT_TC2 | LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2);
  /* Set duty to 0 */
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, 0);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, 0);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, 0);
  /* Set blanking for CBC current limmiting is 200ns */
  float blankingTime = 200.f * 1e-9 / ((1.f / pwmFreq) / (*halfPeriod * 2.f));
  LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)blankingTime);
  LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)blankingTime);
  LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)blankingTime);

  LL_HRTIM_ClearFlag_FLT4(HRTIM1);
}

/* USER CODE END 0 */

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_HRTIM1);
  
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /* HRTIM1 interrupt Init */
  NVIC_SetPriority(HRTIM1_FLT_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(HRTIM1_FLT_IRQn);

  LL_HRTIM_ConfigDLLCalibration(HRTIM1, LL_HRTIM_DLLCALIBRATION_MODE_CONTINUOUS, LL_HRTIM_DLLCALIBRATION_RATE_0);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  /** Hardware current limiting routing
   * PHASE_A -> PB1  -> CMPR1 -> EE4 -> TA -> TA1_OUT_RST_SRC
   * PHASE_B -> PB11 -> CMPR6 -> EE3 -> TC -> TC1_OUT_RST_SRC
   * PHASE_C -> PB0  -> CMPR4 -> EE2 -> TD -> TD1_OUT_RST_SRC
   * */
  LL_HRTIM_EE_SetPrescaler(HRTIM1, LL_HRTIM_EE_PRESCALER_DIV1);
  LL_HRTIM_EE_SetSrc(HRTIM1, LL_HRTIM_EVENT_2, LL_HRTIM_EEV2SRC_COMP4_OUT);
  LL_HRTIM_EE_SetPolarity(HRTIM1, LL_HRTIM_EVENT_2, LL_HRTIM_EE_POLARITY_HIGH);
  LL_HRTIM_EE_SetSensitivity(HRTIM1, LL_HRTIM_EVENT_2, LL_HRTIM_EE_SENSITIVITY_LEVEL);
  LL_HRTIM_EE_SetFastMode(HRTIM1, LL_HRTIM_EVENT_2, LL_HRTIM_EE_FASTMODE_ENABLE);
  LL_HRTIM_EE_SetSrc(HRTIM1, LL_HRTIM_EVENT_3, LL_HRTIM_EEV3SRC_COMP6_OUT);
  LL_HRTIM_EE_SetPolarity(HRTIM1, LL_HRTIM_EVENT_3, LL_HRTIM_EE_POLARITY_HIGH);
  LL_HRTIM_EE_SetSensitivity(HRTIM1, LL_HRTIM_EVENT_3, LL_HRTIM_EE_SENSITIVITY_LEVEL);
  LL_HRTIM_EE_SetFastMode(HRTIM1, LL_HRTIM_EVENT_3, LL_HRTIM_EE_FASTMODE_ENABLE);
  LL_HRTIM_EE_SetSrc(HRTIM1, LL_HRTIM_EVENT_4, LL_HRTIM_EEV4SRC_COMP1_OUT);
  LL_HRTIM_EE_SetPolarity(HRTIM1, LL_HRTIM_EVENT_4, LL_HRTIM_EE_POLARITY_HIGH);
  LL_HRTIM_EE_SetSensitivity(HRTIM1, LL_HRTIM_EVENT_4, LL_HRTIM_EE_SENSITIVITY_LEVEL);
  LL_HRTIM_EE_SetFastMode(HRTIM1, LL_HRTIM_EVENT_4, LL_HRTIM_EE_FASTMODE_ENABLE);

  LL_HRTIM_TIM_SetEventFilter(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_EVENT_4, LL_HRTIM_EEFLTR_BLANKINGCMP2);
  LL_HRTIM_TIM_SetEventFilter(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_EVENT_3, LL_HRTIM_EEFLTR_BLANKINGCMP2);
  LL_HRTIM_TIM_SetEventFilter(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_EVENT_2, LL_HRTIM_EEFLTR_BLANKINGCMP2);

  LL_HRTIM_ConfigADCTrig(HRTIM1, LL_HRTIM_ADCTRIG_4, LL_HRTIM_ADCTRIG_UPDATE_TIMER_C, LL_HRTIM_ADCTRIG_SRC24_TIMCPER);
  LL_HRTIM_SetADCPostScaler(HRTIM1, LL_HRTIM_ADCTRIG_2, 0);

  LL_HRTIM_ConfigADCTrig(HRTIM1, LL_HRTIM_ADCTRIG_2, LL_HRTIM_ADCTRIG_UPDATE_TIMER_C, LL_HRTIM_ADCTRIG_SRC24_TIMCRST);
  LL_HRTIM_SetADCPostScaler(HRTIM1, LL_HRTIM_ADCTRIG_2, 0); // how many PWM periods for 1 ADC sampling
  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_PRESCALERRATIO_MUL2);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_MASTER, 4095);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_MASTER, 0x00);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_UPDATETRIG_NONE);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_MASTER);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_PRESCALERRATIO_MUL2);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, 4095);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_A, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_COUNTING_MODE_UP_DOWN);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetFaultEventRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetBMRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetADCRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetOutputRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_EnableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_A); // update duty when update event ocurs
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_UPDATETRIG_NONE | LL_HRTIM_UPDATETRIG_RESET);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, 2047);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DT_PRESCALER_DIV2); // DIV - MUL vice versa. see RM and PSC defines
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_A, 13);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_A, 13);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUTPUTSET_TIMCMP1);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUTPUTRESET_EEV_2 | LL_HRTIM_OUTPUTRESET_EEV_3 | LL_HRTIM_OUTPUTRESET_EEV_4);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUTPUTSET_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUTPUTRESET_EEV_2 | LL_HRTIM_OUTPUTRESET_EEV_3 | LL_HRTIM_OUTPUTRESET_EEV_4);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_PRESCALERRATIO_MUL2);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_C, 4095);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_C, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_COUNTING_MODE_UP_DOWN);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetFaultEventRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetBMRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetADCRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_RST); // when trigger occurs
  LL_HRTIM_TIM_SetOutputRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_EnableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_UPDATETRIG_NONE | LL_HRTIM_UPDATETRIG_RESET);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, 0);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DT_PRESCALER_DIV2);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_C, 13);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_C, 13);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUTPUTSET_TIMCMP1);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUTPUTRESET_EEV_2 | LL_HRTIM_OUTPUTRESET_EEV_3 | LL_HRTIM_OUTPUTRESET_EEV_4);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUTPUTSET_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUTPUTRESET_EEV_2 | LL_HRTIM_OUTPUTRESET_EEV_3 | LL_HRTIM_OUTPUTRESET_EEV_4);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_PRESCALERRATIO_MUL2);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_D, 4095);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_D, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_COUNTING_MODE_UP_DOWN);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetFaultEventRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetBMRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetADCRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetOutputRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_EnableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_UPDATETRIG_NONE | LL_HRTIM_UPDATETRIG_RESET);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, 0);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_DT_PRESCALER_DIV2);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_D, 13);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_D, 13);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUTPUTSET_TIMCMP1);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUTPUTRESET_EEV_2 | LL_HRTIM_OUTPUTRESET_EEV_3 | LL_HRTIM_OUTPUTRESET_EEV_4);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUTPUTSET_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUTPUTRESET_EEV_2 | LL_HRTIM_OUTPUTRESET_EEV_3 | LL_HRTIM_OUTPUTRESET_EEV_4);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**HRTIM1 GPIO Configuration    
    PB12     ------> HRTIM1_CHC1
    PB13     ------> HRTIM1_CHC2
    PB14     ------> HRTIM1_CHD1
    PB15     ------> HRTIM1_CHD2
    PA8     ------> HRTIM1_CHA1
    PA9     ------> HRTIM1_CHA2 
    */
  GPIO_InitStruct.Pin = PWM_BH_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(PWM_BH_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_BL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(PWM_BL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_CH_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(PWM_CH_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_CL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(PWM_CL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_AH_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(PWM_AH_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_AL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(PWM_AL_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 1 */

//LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_COUNTING_MODE_UP_DOWN);

//LL_HRTIM_ConfigDLLCalibration(HRTIM1, LL_HRTIM_DLLCALIBRATION_MODE_SINGLESHOT, LL_HRTIM_DLLCALIBRATION_RATE_0);
//LL_HRTIM_StartDLLCalibration(HRTIM1);

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
