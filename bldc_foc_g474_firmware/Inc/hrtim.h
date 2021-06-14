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
  static inline void pwm_disable(void)
  {
    LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | LL_HRTIM_OUTPUT_TC1 |
                                       LL_HRTIM_OUTPUT_TC2 | LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2);
  }
  static inline void pwm_enable(void)
  {
    LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | LL_HRTIM_OUTPUT_TC1 |
                                      LL_HRTIM_OUTPUT_TC2 | LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2);
  }
  /* USER CODE END Private defines */

  void MX_HRTIM1_Init(void);

  /* USER CODE BEGIN Prototypes */
  void hrtim_start(void);
  void MX_HRTIM1_Init_BLDC(void);

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
