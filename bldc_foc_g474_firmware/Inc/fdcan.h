/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  /* USER CODE BEGIN Includes */
  // struct canData_s
  // {
  //   FDCAN_HandleTypeDef hfdcan1;
  //   FDCAN_FilterTypeDef sFilterConfig;
  //   FDCAN_TxHeaderTypeDef TxHeader;
  //   FDCAN_RxHeaderTypeDef RxHeader;
  //   uint8_t TxData[8];
  //   uint8_t RxData[8];
  // };
  // typedef struct canData_s canData_t;
  /* USER CODE END Includes */

  //extern FDCAN_HandleTypeDef hfdcan1;

  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

  void MX_FDCAN1_Init(void);

  /* USER CODE BEGIN Prototypes */
  void can_filters_config(void);
  void can_transmit_massege(FDCAN_TxHeaderTypeDef TxHeader, uint32_t id, uint8_t *message, uint8_t length, uint8_t messageId);
  void can_receive_massege(void);
  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
