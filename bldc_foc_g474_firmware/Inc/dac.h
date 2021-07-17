/**
  ******************************************************************************
  * @file    dac.h
  * @brief   This file contains all the function prototypes for
  *          the dac.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#ifndef __DAC_H__
#define __DAC_H__

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

  void MX_DAC2_Init(void);
  void MX_DAC3_Init(void);

  /* USER CODE BEGIN Prototypes */
  void dac_start(void);

  /**
 * @brief 
 * Update hardware current control thresholds
 * 
 * @param iMax - max current (A)
 * @param offset - offset factor to convert current in (A) to DAC (0-4095) value
 */
  static inline void dac_cmpr_update(float iMax, float offset)
  {
    /* Set the data to be loaded in the data holding register */
    LL_DAC_ConvertData12RightAligned(DAC2, LL_DAC_CHANNEL_1, (uint32_t)(iMax * offset + offset) - 1);
    LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_1, (uint32_t)(iMax * offset + offset) - 1);
    LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_2, (uint32_t)(iMax * offset + offset) - 1);

    /* Trig DAC conversion by software */
    LL_DAC_TrigSWConversion(DAC2, LL_DAC_CHANNEL_1);
    LL_DAC_TrigSWConversion(DAC3, LL_DAC_CHANNEL_1);
    LL_DAC_TrigSWConversion(DAC3, LL_DAC_CHANNEL_2);
  }
  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DAC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
