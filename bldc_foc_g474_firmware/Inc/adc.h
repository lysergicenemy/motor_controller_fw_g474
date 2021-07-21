/**
  ******************************************************************************
  * File Name          : ADC.h
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* USER CODE BEGIN Private defines */
  struct adcData_s
  {
    int32_t ph_u;
    int32_t ph_v;
    int32_t ph_w;
    int32_t v_u;
    int32_t v_v;
    int32_t v_w;
    int32_t v_dc;
    int32_t pcb_temp;
  };
  typedef volatile struct adcData_s adcData_t;

  /** Get ADC input data in SI units
 *  Note: offsets must be calcualted before
 *  Output:
 *  iPhaseA, iPhaseB, iPhaseC - phase currents (A)
 *  vPhaseA, vPhaseB, vPhaseC - phase voltages (V)
 *  DcBusVolt - DC-Link voltage (V)
 *  isa, isb - alpha/beta components of phase currents (A)
 *  vAlpha, vBeta - alpha/beta components of phase voltages (V)
 */
  static inline void adc_get_inputs_SI(adcData_t *padc, foc_t *p, modelBLDC_t *pbldc)
  {
    const float one_by_halfADC = 0.00048828125f; // (1 / 2048)
    const float one_by_maxADC = 0.000244140625f; // (1 / 4096)
    if (p->config.sim == 1)
    {
      p->data.angle = pbldc->tetaR;
      // clarke transform for phase currents
      p->data.isa = pbldc->isPhaseA;
      p->data.isb = _1DIV_SQRT3 * pbldc->isPhaseA + _2DIV_SQRT3 * pbldc->isPhaseB;
      p->volt.DcBusVolt = pbldc->udc;
    }
    else if (p->config.sim == 0)
    {
      // scaling ADC data
      p->data.iPhaseA = ((float)padc->ph_u - p->data.offsetCurrA) * (one_by_halfADC * p->config.adcFullScaleCurrent);
      p->data.iPhaseB = ((float)padc->ph_v - p->data.offsetCurrB) * (one_by_halfADC * p->config.adcFullScaleCurrent);
      p->data.iPhaseC = ((float)padc->ph_w - p->data.offsetCurrC) * (one_by_halfADC * p->config.adcFullScaleCurrent);
      p->volt.DcBusVolt = (float)padc->v_dc * one_by_maxADC * p->config.adcFullScaleVoltage;
      p->data.vPhaseA = ((float)padc->v_u - p->data.offsetVoltA) * (one_by_maxADC * p->config.adcFullScaleVoltage);
      p->data.vPhaseB = ((float)padc->v_v - p->data.offsetVoltB) * (one_by_maxADC * p->config.adcFullScaleVoltage);
      p->data.vPhaseC = ((float)padc->v_w - p->data.offsetVoltC) * (one_by_maxADC * p->config.adcFullScaleVoltage);
      // Clarke transform for phase currents and voltages
      p->data.isa = p->data.iPhaseA;
      p->data.isb = _1DIV_SQRT3 * p->data.iPhaseA + _2DIV_SQRT3 * p->data.iPhaseB;
      p->data.vAlpha = (2.f / 3.f) * p->cmtn.lpf_bemfA.out - (1.f / 3.f) * (p->cmtn.lpf_bemfB.out - p->cmtn.lpf_bemfC.out);
      p->data.vBeta = ONE_BY_SQRT3 * (p->cmtn.lpf_bemfB.out - p->cmtn.lpf_bemfC.out);
    }
  }
  /* USER CODE END Private defines */

  void MX_ADC1_Init(void);
  void MX_ADC2_Init(void);
  void MX_ADC3_Init(void);

  /* USER CODE BEGIN Prototypes */
  void adc_start(void);
  void adc_start_bldc(void);
  void ADC1_Init_BLDC(void);
  void ADC2_Init_BLDC(void);

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
