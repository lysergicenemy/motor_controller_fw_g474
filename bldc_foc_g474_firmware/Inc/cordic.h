/**
  ******************************************************************************
  * @file    cordic.h
  * @brief   This file contains all the function prototypes for
  *          the cordic.c file
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
#ifndef __CORDIC_H__
#define __CORDIC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  void Cordic_Init(void);

  /** Calc sinCos using hardware CORDIC
   *  Input: angle (must be scaled to -1 : 1 range)
   *  Average execution time: ~98 CPU cycles
   **/
  static inline void cordic_sincos_calc(volatile float angle, volatile float *s, volatile float *c)
  {
    if (LL_CORDIC_GetFunction(CORDIC) != LL_CORDIC_FUNCTION_SINE)
    {
      LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_SINE, /* cosine function */
                       LL_CORDIC_PRECISION_6CYCLES,     /* max precision for q1.31 cosine */
                       LL_CORDIC_SCALE_0,               /* no scale */
                       LL_CORDIC_NBWRITE_1,             /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
                       LL_CORDIC_NBREAD_2,              /* Two output data: sine, then cosine */
                       LL_CORDIC_INSIZE_32BITS,         /* q1.31 format for input data */
                       LL_CORDIC_OUTSIZE_32BITS);       /* q1.31 format for output data */
    }

    LL_CORDIC_WriteData(CORDIC, _IQ31(angle * ONE_BY_PI)); // cordic input data format q31, angle range (1 : -1)
    *s = _IQ31toF((int32_t)LL_CORDIC_ReadData(CORDIC));
    *c = _IQ31toF((int32_t)LL_CORDIC_ReadData(CORDIC));
  }

  // Calc atan2 using hardware CORDIC
  static inline void cordic_atan2_calc(volatile float s, volatile float c, volatile float *angle)
  {
    if (LL_CORDIC_GetFunction(CORDIC) != LL_CORDIC_FUNCTION_PHASE)
    {
      LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_PHASE, /* phase/atan2 function */
                       LL_CORDIC_PRECISION_6CYCLES,     /* max precision for q1.31 cosine */
                       LL_CORDIC_SCALE_0,               /* no scale */
                       LL_CORDIC_NBWRITE_2,             /* Two input data: sin, cos */
                       LL_CORDIC_NBREAD_1,              /* One output data: phase */
                       LL_CORDIC_INSIZE_32BITS,         /* q1.31 format for input data */
                       LL_CORDIC_OUTSIZE_32BITS);       /* q1.31 format for output data */
    }

    LL_CORDIC_WriteData(CORDIC, _IQ31(s * ONE_BY_PI));
    LL_CORDIC_WriteData(CORDIC, _IQ31(c * ONE_BY_PI));
    *angle = _IQ31toF((int32_t)LL_CORDIC_ReadData(CORDIC));
  }

#ifdef __cplusplus
}
#endif

#endif /* __CORDIC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
