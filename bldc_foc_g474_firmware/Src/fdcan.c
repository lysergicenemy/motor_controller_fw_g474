/**
  ******************************************************************************
  * @file    fdc
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

// FDCAN_FilterTypeDef sFilterConfig;
// FDCAN_TxHeaderTypeDef TxHeader;
// FDCAN_RxHeaderTypeDef RxHeader;
// uint8_t TxData[8];
// uint8_t RxData[8];
canData_t can;

/* */
void can_filters_config(void)
{
  /* Configure extended ID reception filter to Rx FIFO 0 */
  can.sFilterConfig.IdType = FDCAN_STANDARD_ID;
  can.sFilterConfig.FilterIndex = 0;
  can.sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  can.sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE; //FDCAN_FILTER_TO_RXFIFO0;
  can.sFilterConfig.FilterID1 = 0x111;
  can.sFilterConfig.FilterID2 = 0x7FF;
  HAL_FDCAN_ConfigFilter(&can.hfdcan1, &can.sFilterConfig);
  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  HAL_FDCAN_ConfigGlobalFilter(&can.hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

/* */
void can_transmit_massege(FDCAN_TxHeaderTypeDef TxHeader, uint32_t id, uint8_t *message, uint8_t DLC, uint8_t messageId)
{
  // /* Add message to Tx FIFO */
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  //TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.DataLength = (uint32_t)(DLC << 16);
  TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = messageId;
  HAL_FDCAN_AddMessageToTxFifoQ(&can.hfdcan1, &TxHeader, message);
}
void can_receive_massege(void)
{
  if (can.RxFlag != 0)
  {
    HAL_FDCAN_GetRxMessage(&can.hfdcan1, FDCAN_RX_FIFO0, &can.RxHeader, can.RxData);
    can.RxFlag = 0;
  }
}

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{
  can.hfdcan1.Instance = FDCAN1;
  can.hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
  can.hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  can.hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  can.hfdcan1.Init.AutoRetransmission = DISABLE;
  can.hfdcan1.Init.TransmitPause = DISABLE;
  can.hfdcan1.Init.ProtocolException = DISABLE;
  can.hfdcan1.Init.NominalPrescaler = 2;
  can.hfdcan1.Init.NominalSyncJumpWidth = 2;
  can.hfdcan1.Init.NominalTimeSeg1 = 71;
  can.hfdcan1.Init.NominalTimeSeg2 = 13;
  can.hfdcan1.Init.DataPrescaler = 2;
  can.hfdcan1.Init.DataSyncJumpWidth = 2;
  can.hfdcan1.Init.DataTimeSeg1 = 71;
  can.hfdcan1.Init.DataTimeSeg2 = 13;
  can.hfdcan1.Init.StdFiltersNbr = 1;
  can.hfdcan1.Init.ExtFiltersNbr = 0;
  can.hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&can.hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  can_filters_config();
  HAL_FDCAN_Start(&can.hfdcan1);
  HAL_FDCAN_ActivateNotification(&can.hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  /* USER CODE END FDCAN1_Init 2 */
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (fdcanHandle->Instance == FDCAN1)
  {
    /* GPIO Ports Clock Enable */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
    /**/
    LL_GPIO_ResetOutputPin(FDCAN_RX_GPIO_Port, FDCAN_RX_Pin);
    LL_GPIO_ResetOutputPin(FDCAN_TX_GPIO_Port, FDCAN_TX_Pin);
    /**/
    GPIO_InitStruct.Pin = FDCAN_RX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /**/
    GPIO_InitStruct.Pin = FDCAN_TX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    NVIC_SetPriority(FDCAN1_IT0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    /* USER CODE BEGIN FDCAN1_MspInit 1 */

    /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{

  if (fdcanHandle->Instance == FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

    /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_FDCAN);

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */

    /* FDCAN1 interrupt Deinit */
    NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

    /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
