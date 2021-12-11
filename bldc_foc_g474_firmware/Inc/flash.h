/**
  ******************************************************************************
  * File Name          : 
  * Description        : 
  *
  ******************************************************************************
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __flash_H
#define __flash_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"


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

#ifdef __cplusplus
}
#endif
#endif /*__ flash_H */

/****************************/
