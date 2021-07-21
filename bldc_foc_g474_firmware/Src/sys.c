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
#define FLASH_SIZE_DATA_REGISTER FLASHSIZE_BASE

#if defined(FLASH_OPTR_DBANK)
#define FLASH_SIZE ((((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0xFFFFU)) ? (0x200UL << 10U) : (((*((uint32_t *)FLASH_SIZE_DATA_REGISTER)) & 0xFFFFUL) << 10U))
#define FLASH_BANK_SIZE (FLASH_SIZE >> 1)
#define FLASH_PAGE_NB 128U
#define FLASH_PAGE_SIZE_128_BITS 0x1000U /* 4 KB */
#else
#define FLASH_SIZE ((((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0xFFFFU)) ? (0x80UL << 10U) : (((*((uint32_t *)FLASH_SIZE_DATA_REGISTER)) & 0xFFFFUL) << 10U))
#define FLASH_BANK_SIZE (FLASH_SIZE)
#define FLASH_PAGE_NB ((FLASH_SIZE == 0x00080000U) ? 256U : 64U)
#endif

#define FLASH_PAGE_SIZE 0x800U /* 2 KB */

#define FLASH_TIMEOUT_VALUE 1000U /* 1 s  */

#define FLASH_BANK_1 0x00000001U /*!< Bank 1   */
#if defined(FLASH_OPTR_DBANK)
#define FLASH_BANK_2 0x00000002U                      /*!< Bank 2   */
#define FLASH_BANK_BOTH (FLASH_BANK_1 | FLASH_BANK_2) /*!< Bank1 and Bank2  */
#else
#define FLASH_BANK_BOTH FLASH_BANK_1 /*!< Bank 1   */
#endif
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

void FLASH_PageErase(uint32_t Page, uint32_t Bank)
{
  while (FLASH->SR & FLASH_SR_BSY)
    ;
   SET_BIT(FLASH->SR, FLASH_SR_PROGERR);
   SET_BIT(FLASH->SR, FLASH_SR_PGAERR);
   SET_BIT(FLASH->SR, FLASH_SR_PGSERR);
  // SET_BIT(FLASH->ECCR, (FLASH_SR_OPTVERR & (FLASH_ECCR_ECCC | FLASH_ECCR_ECCD)));
  //CLEAR_BIT(FLASH->CR, FLASH_CR_BKER);

  CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICEN);
  CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCEN);
/* Proceed to erase the page */
#if defined(FLASH_OPTR_DBANK)
  if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) == 0U)
  {
    CLEAR_BIT(FLASH->CR, FLASH_CR_BKER);
  }
  else
  {
    if ((Bank & FLASH_BANK_1) != 0U)
    {
      CLEAR_BIT(FLASH->CR, FLASH_CR_BKER);
    }
    else
    {
      SET_BIT(FLASH->CR, FLASH_CR_BKER);
    }
  }
#endif

  MODIFY_REG(FLASH->CR, FLASH_CR_PNB, ((Page & 0xFFU) << FLASH_CR_PNB_Pos));
  SET_BIT(FLASH->CR, FLASH_CR_PER);
  SET_BIT(FLASH->CR, FLASH_CR_STRT);
  /* wait operation */
  while (FLASH->SR & FLASH_SR_BSY)
    ;
  CLEAR_BIT(FLASH->CR, (FLASH_CR_PER | FLASH_CR_PNB));

  /* Reset instruction cache */
  do
  {
    SET_BIT(FLASH->ACR, FLASH_ACR_ICRST);
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICRST);
  } while (0);
  /* Enable instruction cache */
  SET_BIT(FLASH->ACR, FLASH_ACR_ICEN);
  /* Flush the instruction and data caches */
  /* Reset data cache */
  do
  {
    SET_BIT(FLASH->ACR, FLASH_ACR_DCRST);
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCRST);
  } while (0);
  /* Enable data cache */
  SET_BIT(FLASH->ACR, FLASH_ACR_DCEN);
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

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */

uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}

void FLASH_UpdateConfig(foc_t *p, hall_t *hp)
{
  if (p->driveState == STOP)
  {
    __disable_irq();
    uint32_t Address = FLASH_CONFIG_ADR_START;
    /* Clear OPTVERR bit set on virgin samples */
    SET_BIT(FLASH->SR, FLASH_SR_OPTVERR);
    flashUnlock();
    FLASH_PageErase(FLASH_CONFIG_PG_NMB, GetBank(Address));
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
    FLASH_Program_DoubleWord(Address + 8 * 10, *(uint32_t *)&p->config.Kv);
    // End
    flashLock();
    p->paramIdState = ID_ENTER;
    p->paramIdRunState = ID_RUN_HALL_FWD;
    hp->offsetState = 0;
    p->driveState = STOP;
    p->data.flashUpdateFlag = 1;
    __enable_irq();
  }
}

void FLASH_LoadConfig(foc_t *p, hall_t *hp)
{
  uint32_t Address = FLASH_CONFIG_ADR_START;

  uint32_t u = flashReadData(Address);
  p->config.Rs = *(float *)&u;

  u = flashReadData(Address + 8);
  p->config.Ld = *(float *)&u;

  u = flashReadData(Address + 8 * 2);
  p->config.Lq = *(float *)&u;

  u = flashReadData(Address + 8 * 3);
  hp->offsetAvg[0] = *(float *)&u;

  u = flashReadData(Address + 8 * 4);
  hp->offsetAvg[1] = *(float *)&u;

  u = flashReadData(Address + 8 * 5);
  hp->offsetAvg[2] = *(float *)&u;

  u = flashReadData(Address + 8 * 6);
  hp->offsetAvg[3] = *(float *)&u;

  u = flashReadData(Address + 8 * 7);
  hp->offsetAvg[4] = *(float *)&u;

  u = flashReadData(Address + 8 * 8);
  hp->offsetAvg[5] = *(float *)&u;

  u = flashReadData(Address + 8 * 9);
  hp->offset = *(float *)&u;

  u = flashReadData(Address + 8 * 10);
  p->config.Kv = *(float *)&u;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
