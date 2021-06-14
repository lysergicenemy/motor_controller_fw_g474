/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "cordic.h"

void Cordic_Init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
  /* Set default func - sinCos */ 
  LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_SINE, /* sine function */
                   LL_CORDIC_PRECISION_6CYCLES,     /* max precision for q1.31 cosine */
                   LL_CORDIC_SCALE_0,               /* no scale */
                   LL_CORDIC_NBWRITE_1,             /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
                   LL_CORDIC_NBREAD_2,              /* Two output data: sine, then cosine */
                   LL_CORDIC_INSIZE_32BITS,         /* q1.31 format for input data */
                   LL_CORDIC_OUTSIZE_32BITS);       /* q1.31 format for output data */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
