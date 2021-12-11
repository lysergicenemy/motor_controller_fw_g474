/**
  ******************************************************************************
  * File Name          : 
  * Description        : 
  ******************************************************************************
  *
  ******************************************************************************
  */

#include "flash.h"

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
    return (*(__IO uint32_t *)address);
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

        uint64_t array[(sizeof(p->config) / sizeof(uint64_t))];
        memcpy(array, (const void *)&p->config, sizeof(p->config));

        /* Clear OPTVERR bit set on virgin samples */
        SET_BIT(FLASH->SR, FLASH_SR_OPTVERR);
        flashUnlock();
        FLASH_PageErase(FLASH_CONFIG_PG_NMB, GetBank(Address));

        for (uint32_t i = 0; i < (sizeof(p->config) / sizeof(uint64_t)); i++, Address += sizeof(uint64_t))
        {
            FLASH_Program_DoubleWord(Address, array[i]);
        }
        Address = FLASH_CONFIG_ADR_START + sizeof(p->config);
        for (uint32_t i = 0; i < (sizeof(hp->offsetAvg) / sizeof(hp->offsetAvg[0])); i++, Address += sizeof(uint64_t))
        {
            //uint64_t hData = ((uint32_t)hp->offsetAvg[i] << 32 ) | ((uint32_t)hp->offsetAvg[i+1] & 0xffffffff);
            FLASH_Program_DoubleWord(Address, *(uint32_t *)&hp->offsetAvg[i]);
        }
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
    /* Check if flash data is empty */
    if (flashReadData(FLASH_CONFIG_ADR_START) != 0xFFFFFFFF)
    {
        uint32_t Address = FLASH_CONFIG_ADR_START;
        memcpy((void *)&p->config, (const void *)Address, sizeof(p->config));

        Address = FLASH_CONFIG_ADR_START + sizeof(p->config);
        for (uint32_t i = 0; i < (sizeof(hp->offsetAvg) / sizeof(hp->offsetAvg[0])); i++, Address += sizeof(uint64_t))
        {
            uint32_t u = flashReadData(Address);
            hp->offsetAvg[i] = *(float*)&u;
        }
    }
}

/****************************/
