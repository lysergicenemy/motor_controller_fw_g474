/**
  ******************************************************************************
  * File Name          : SYS.h
  * Description        : This file provides code for the configuration
  *                      of the SYS instances.
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
#ifndef __sys_H
#define __sys_H
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

  void MX_SYS_Init(void);

  /* USER CODE BEGIN Prototypes */
  void flashUnlock(void);
  void flashLock(void);
  uint32_t flashReadData(uint32_t address);
  void flashWriteData(uint32_t address, uint64_t data);
  void Flash_Write(unsigned char *data, unsigned int address, unsigned int count);
  void FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data);
  void FLASH_PageErase(uint32_t Page, uint32_t Bank);
  void FLASH_UpdateConfig(foc_t *p, hall_t *hp);
  void FLASH_LoadConfig(foc_t *p, hall_t *hp);
  uint32_t GetBank(uint32_t Addr);
  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ sys_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
