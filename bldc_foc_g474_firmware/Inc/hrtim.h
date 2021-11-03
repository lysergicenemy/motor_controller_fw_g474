/**
  ******************************************************************************
  * File Name          : HRTIM.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __hrtim_H
#define __hrtim_H
#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

  void MX_HRTIM1_Init(void);

  /* USER CODE BEGIN Prototypes */
  void hrtim_start(volatile float pwmFreq, volatile float deadtime_ns, uint32_t pwmPeriod_adcTrig_ratio, volatile float *halfPeriod);

  /**
 * @brief Disable PWM signals
 * 
 */
  static inline void hrtim_pwm_disable(void)
  {
    LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | LL_HRTIM_OUTPUT_TC1 |
                                       LL_HRTIM_OUTPUT_TC2 | LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2);
  }

  /**
 * @brief Enable PWM signals
 * 
 */
  static inline void hrtim_pwm_enable(void)
  {
    LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | LL_HRTIM_OUTPUT_TC1 |
                                      LL_HRTIM_OUTPUT_TC2 | LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2);
  }

  /**
 * @brief Update PWM signals
 * Input:
 * ta, tb, tc - referance duty cycles, range(-1.0f : 1.0f)
 * halfPwmPeriod - half of counter update value
 */
  static inline void hrtim_pwm_update(float ta, float tb, float tc, float halfPwmPeriod)
  {
    LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(ta * halfPwmPeriod + halfPwmPeriod));
    LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(tb * halfPwmPeriod + halfPwmPeriod));
    LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(tc * halfPwmPeriod + halfPwmPeriod));
  }

  /**
 * @brief Hardware-dependent PWM macro for BLDC motor commutation logic
 * 
 * Config (#define PWM_METHOD):
 * Switching mode 1: terminal 1 - disable
 *                   terminal 2 - PWM
 *                   terminal 3 - PWM
 * Switching mode 2: terminal 1 - disable
 *                   terminal 2 - PWM
 *                   terminal 3 - force low side
 */

  // Configure driver in this section:

#define AH (LL_HRTIM_OUTPUT_TA1)
#define AL (LL_HRTIM_OUTPUT_TA2)
#define BH (LL_HRTIM_OUTPUT_TC1)
#define BL (LL_HRTIM_OUTPUT_TC2)
#define CH (LL_HRTIM_OUTPUT_TD1)
#define CL (LL_HRTIM_OUTPUT_TD2)

  static inline void hrtim_bldcpwm_update(float halfPwmPeriod, float duty, uint8_t sector)
  {
    switch (sector)
    {
    case 0:
      /* State s1: current flows to motor windings from phase A->B, de-energized phase = C */
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(halfPwmPeriod + duty * halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(halfPwmPeriod - duty * halfPwmPeriod));
      LL_HRTIM_DisableOutput(HRTIM1, CH | CL);
      LL_HRTIM_EnableOutput(HRTIM1, AH | AL | BH | BL);
      break;
    case 1:
      /* State s2: current flows to motor windings from phase A->C, de-energized phase = B */
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(halfPwmPeriod + duty * halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(halfPwmPeriod - duty * halfPwmPeriod));
      LL_HRTIM_DisableOutput(HRTIM1, BH | BL);
      LL_HRTIM_EnableOutput(HRTIM1, AH | AL | CH | CL);
      break;
    case 2:
      /* State s3: current flows to motor windings from phase B->C, de-energized phase = A */
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(halfPwmPeriod + duty * halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(halfPwmPeriod - duty * halfPwmPeriod));
      LL_HRTIM_DisableOutput(HRTIM1, AH | AL);
      LL_HRTIM_EnableOutput(HRTIM1, BH | BL | CH | CL);
      break;
    case 3:
      /* State s4: current flows to motor windings from phase B->A, de-energized phase = C */
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(halfPwmPeriod - duty * halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(halfPwmPeriod + duty * halfPwmPeriod));
      LL_HRTIM_DisableOutput(HRTIM1, CH | CL);
      LL_HRTIM_EnableOutput(HRTIM1, BH | BL | AH | AL);
      break;
    case 4:
      /* State s5: current flows to motor windings from phase C->A, de-energized phase = B */
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(halfPwmPeriod - duty * halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(halfPwmPeriod + duty * halfPwmPeriod));
      LL_HRTIM_DisableOutput(HRTIM1, BH | BL);
      LL_HRTIM_EnableOutput(HRTIM1, CH | CL | AH | AL);
      break;
    case 5:
      /* State s6: current flows to motor windings from phase C->B, de-energized phase = A */
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(halfPwmPeriod - duty * halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(halfPwmPeriod + duty * halfPwmPeriod));
      LL_HRTIM_DisableOutput(HRTIM1, AH | AL);
      LL_HRTIM_EnableOutput(HRTIM1, CH | CL | BH | BL);
      break;
    default:
      break;
    }
  }

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ hrtim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
