/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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

#include "main.h"
#include "stm32g4xx_it.h"

void slowCalc(void);

extern dataLogVars_t dataLog;
extern hall_t hall;
extern canData_t can;
extern void can_receive_massege(void);

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
// void ADC1_2_IRQHandler(void)
// {
//   /* USER CODE BEGIN ADC1_2_IRQn 0 */
//     if (LL_ADC_IsActiveFlag_JEOS(ADC1))
//     {
//      LL_ADC_ClearFlag_JEOS(ADC1);
//     }

//   /* USER CODE END ADC1_2_IRQn 0 */

//   /* USER CODE BEGIN ADC1_2_IRQn 1 */

//   /* USER CODE END ADC1_2_IRQn 1 */
// }

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  // if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) != RESET)
  // {
  //   LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
  //   hall.A = (hall.A == 0) ? 1 : 0;
  //   hall.cyclesCntPr = hall.cyclesCnt;
  //   hall.cyclesCnt = DWT->CYCCNT;
  // }
  // if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) != RESET)
  // {
  //   LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
  //   hall.B = (hall.B == 0) ? 1 : 0;
  //   hall.cyclesCntPr = hall.cyclesCnt;
  //   hall.cyclesCnt = DWT->CYCCNT;
  // }
  // if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET)
  // {
  //   LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
  //   hall.C = (hall.C == 0) ? 1 : 0;
  //   hall.cyclesCntPr = hall.cyclesCnt;
  //   hall.cyclesCnt = DWT->CYCCNT;
  // }
}

/**
  * @brief This function handles TIM7 global interrupt, DAC2 and DAC4 channel underrun error interrupts.
  */
void TIM7_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC_IRQn 0 */
  if (LL_TIM_IsActiveFlag_UPDATE(TIM7))
  {
    LL_TIM_ClearFlag_UPDATE(TIM7);
    slowCalc();
  }

  /* USER CODE END TIM7_DAC_IRQn 0 */

  /* USER CODE BEGIN TIM7_DAC_IRQn 1 */

  /* USER CODE END TIM7_DAC_IRQn 1 */
}

/**
  * @brief This function handles HRTIM fault global interrupt.
  */
void HRTIM1_FLT_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_FLT_IRQn 0 */
  LL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
  LL_mDelay(100);

  /* USER CODE END HRTIM1_FLT_IRQn 0 */

  /* USER CODE BEGIN HRTIM1_FLT_IRQn 1 */

  /* USER CODE END HRTIM1_FLT_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
  if (LL_DMA_IsActiveFlag_TC6(DMA1))
  {
    LL_DMA_ClearFlag_GI6(DMA1);
    dataLog.tcFlag = 1;
  }
  if (LL_DMA_IsActiveFlag_TE6(DMA1))
  {
    LL_DMA_ClearFlag_TE6(DMA1);
  }
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  // new message received
  if ((FDCAN1->IR & FDCAN_IR_RF0N) != 0)
  {
    // clear flag
    FDCAN1->IR = FDCAN_IR_RF0N | FDCAN_IR_RF0L | FDCAN_IR_RF0F;
    can.RxFlag = 1;
    //can_receive_massege();
  }
  // else
  // {
  //   while (1){}
  // }
}

/**
  * @brief This function handles USART2 global interrupt
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
