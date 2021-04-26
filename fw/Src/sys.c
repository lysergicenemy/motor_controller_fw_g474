/**
  ******************************************************************************
  * File Name          : SYS.c
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

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* SYS init function */
void MX_SYS_Init(void)
{

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral 
  */
  LL_PWR_DisableDeadBatteryPD();
}

/* USER CODE BEGIN 1 */
void flashUnlock(void)
{
  FLASH->KEYR = 0x45670123U;
  FLASH->KEYR = 0xCDEF89ABU;
}

void flashLock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;
}

uint32_t flashReadData(uint32_t address)
{
  return *(__IO uint32_t *)address;
}

void FLASH_PageErase(uint32_t Page)
{
  while (FLASH->SR & FLASH_SR_BSY)
    ;
  SET_BIT(FLASH->SR, FLASH_SR_PROGERR);
  SET_BIT(FLASH->SR, FLASH_SR_PGAERR);
  SET_BIT(FLASH->SR, FLASH_SR_PGSERR);

  CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICEN);
  CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCEN);
  /* Proceed to erase the page */
  MODIFY_REG(FLASH->CR, FLASH_CR_PNB, ((Page & 0xFFU) << FLASH_CR_PNB_Pos));
  SET_BIT(FLASH->CR, FLASH_CR_PER);
  SET_BIT(FLASH->CR, FLASH_CR_STRT);

  while (FLASH->SR & FLASH_SR_BSY)
    ;
  CLEAR_BIT(FLASH->CR, (FLASH_CR_PER | FLASH_CR_PNB));
}

void FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data)
{
  while (FLASH->SR & FLASH_SR_BSY)
    ;
  /* Set PG bit */
  SET_BIT(FLASH->CR, FLASH_CR_PG);
  /* Program first word */
  *(uint32_t *)Address = (uint32_t)Data;
  /* Barrier to ensure programming is performed in 2 steps, in right order
    (independently of compiler optimization behavior) */
  __ISB();
  /* Program second word */
  *(uint32_t *)(Address + 4U) = (uint32_t)(Data >> 32U);
  /* wait end of operation */
  while (FLASH->SR & FLASH_SR_BSY)
    ;
  /* Clear PG bit */
  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
}
void FLASH_UpdateConfig(foc_t *p, hall_t *hp)
{
  __disable_irq();
  /* Clear OPTVERR bit set on virgin samples */
  SET_BIT(FLASH->SR, FLASH_SR_OPTVERR);
  flashUnlock();
  FLASH_PageErase(FLASH_CONFIG_PG_NMB);
  uint32_t Address = FLASH_CONFIG_ADR_START;
  // Stator resistance, DQ inductance *(uint32_t*)
  FLASH_Program_DoubleWord(Address, *(uint32_t *)&p->config.Rs);
  FLASH_Program_DoubleWord(Address + 8, *(uint32_t *)&p->config.Ld);
  FLASH_Program_DoubleWord(Address + 8 * 2, *(uint32_t *)&p->config.Lq);
  // Hall table offset angles
  FLASH_Program_DoubleWord(Address + 8 * 3, *(uint32_t *)&hp->offsetAvg[0]);
  FLASH_Program_DoubleWord(Address + 8 * 4, *(uint32_t *)&hp->offsetAvg[1]);
  FLASH_Program_DoubleWord(Address + 8 * 5, *(uint32_t *)&hp->offsetAvg[2]);
  FLASH_Program_DoubleWord(Address + 8 * 6, *(uint32_t *)&hp->offsetAvg[3]);
  FLASH_Program_DoubleWord(Address + 8 * 7, *(uint32_t *)&hp->offsetAvg[4]);
  FLASH_Program_DoubleWord(Address + 8 * 8, *(uint32_t *)&hp->offsetAvg[5]);
  FLASH_Program_DoubleWord(Address + 8 * 9, *(uint32_t *)&hp->offset);
  // End
  flashLock();
  p->paramIdState = Enter;
  hp->offsetState = 0;
  p->driveState = STOP;
  p->data.flashUpdateFlag = 1;
  __enable_irq();
}

void FLASH_LoadConfig(foc_t *p, hall_t *hp)
{
  uint32_t Address = FLASH_CONFIG_ADR_START;
  
  uint32_t u = flashReadData(Address);
  p->config.Rs = *(float *)&u;

  u = flashReadData(Address + 8);
  p->config.Ld = *(float *)&u;

  u = flashReadData(Address + 8*2);
  p->config.Lq = *(float *)&u;

  u = flashReadData(Address + 8*3);
  hp->offsetAvg[0] = *(float *)&u;

  u = flashReadData(Address + 8*4);
  hp->offsetAvg[1] = *(float *)&u;

  u = flashReadData(Address + 8*5);
  hp->offsetAvg[2] = *(float *)&u;

  u = flashReadData(Address + 8*6);
  hp->offsetAvg[3] = *(float *)&u;

  u = flashReadData(Address + 8*7);
  hp->offsetAvg[4] = *(float *)&u;

  u = flashReadData(Address + 8*8);
  hp->offsetAvg[5] = *(float *)&u;

  u = flashReadData(Address + 8*9);
  hp->offset = *(float *)&u;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
