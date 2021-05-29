/**
 ***********************************************************************
 * File name: dataLog.h
 * Discription: DataLoger for Freemaster/UART
 *  
 *  */

#ifndef DATALOG_H
#define DATALOG_H

#define DATALOG_STERAM_SIZE 1024

// Buffer transfer fnc
void USART2_StartTransfer(void);

/*---------- DataLog struct --------------*/
struct dataLogVars_s
{
  uint8_t trigger;
  uint8_t recordCmplt;
  uint8_t autoTrigEn;
  uint16_t psc;
  uint32_t frameCntr;
  uint32_t isrCntr;
  uint8_t txData[18]; // 4 32-bit channels + 16-bit start frame = 18 bytes
  uint8_t tcFlag;     // transfer CMPLT flag
  float trackedValue; // tracked value for datalog triggering
  float buffChannel1[DATALOG_STERAM_SIZE];
  float buffChannel2[DATALOG_STERAM_SIZE];
  float buffChannel3[DATALOG_STERAM_SIZE];
  float buffChannel4[DATALOG_STERAM_SIZE];
  float channel1;
  float channel2;
  float channel3;
  float channel4;
  float in1;
  float in2;
  float in3;
  float in4;
};
typedef volatile struct dataLogVars_s dataLogVars_t;

static inline void datalogCalc(dataLogVars_t *p)
{
  if (p->trigger >= 1)
  {
    p->buffChannel1[p->frameCntr] = p->in1; // var 1
    p->buffChannel2[p->frameCntr] = p->in2; // var 2
    p->buffChannel3[p->frameCntr] = p->in3; // var 3
    p->buffChannel4[p->frameCntr] = p->in4; // var 4

    p->frameCntr++;
    if (p->frameCntr >= (DATALOG_STERAM_SIZE - 1))
    {
      p->recordCmplt = 1;
      p->trigger = 0;
      p->frameCntr = 0;
    }
  }

  if (p->recordCmplt >= 1)
  {
    if (p->isrCntr >= p->psc)
    {
      p->channel1 = p->buffChannel1[p->frameCntr];
      p->channel2 = p->buffChannel2[p->frameCntr];
      p->channel3 = p->buffChannel3[p->frameCntr];
      p->channel4 = p->buffChannel4[p->frameCntr];

      p->isrCntr = 0;
      p->frameCntr++;
    }
    p->isrCntr++;
    if (p->frameCntr >= (DATALOG_STERAM_SIZE - 1))
    {
      p->recordCmplt = 0;
      p->frameCntr = 0;
      p->isrCntr = 0;
      if (p->autoTrigEn != 0)
      {
        p->trigger = 1;
      }
    }
  }
}

/** DataLog transmit fnc using UART
 *  Hardware dependent elements:
 *  1. Fnc which transmit packet using UART
 *  2. transmited packet flag - tcFlag, must be set in TC ISR
 *  Packet discription:
 *  |frameStartByte1|frameStartByte2|ch1Byte1|ch1Byte2|ch1Byte2|ch1Byte2|...
 *  ...|ch4Byte4| - total bytes count is 18.
 */
static inline void datalogCalcUART(dataLogVars_t *p)
{
  if (p->trigger >= 1)
  {
    p->buffChannel1[p->frameCntr] = p->in1; // var 1
    p->buffChannel2[p->frameCntr] = p->in2; // var 2
    p->buffChannel3[p->frameCntr] = p->in3; // var 3
    p->buffChannel4[p->frameCntr] = p->in4; // var 4

    p->frameCntr++;
    if (p->frameCntr >= (DATALOG_STERAM_SIZE - 1))
    {
      p->recordCmplt = 1;
      p->trigger = 0;
      p->frameCntr = 0;
    }
  }
  if (p->recordCmplt >= 1)
  {
    p->channel1 = p->buffChannel1[p->frameCntr];
    p->channel2 = p->buffChannel2[p->frameCntr];
    p->channel3 = p->buffChannel3[p->frameCntr];
    p->channel4 = p->buffChannel4[p->frameCntr];
    /* Packing data for transmit */
    p->txData[0] = 0x45; // 'E' 0100 0101
    p->txData[1] = 0x5A; // 'Z'
    p->txData[2] = *((uint8_t *)&p->channel1);
    p->txData[3] = *((uint8_t *)&p->channel1 + 1);
    p->txData[4] = *((uint8_t *)&p->channel1 + 2);
    p->txData[5] = *((uint8_t *)&p->channel1 + 3);
    p->txData[6] = *((uint8_t *)&p->channel2);
    p->txData[7] = *((uint8_t *)&p->channel2 + 1);
    p->txData[8] = *((uint8_t *)&p->channel2 + 2);
    p->txData[9] = *((uint8_t *)&p->channel2 + 3);
    p->txData[10] = *((uint8_t *)&p->channel3);
    p->txData[11] = *((uint8_t *)&p->channel3 + 1);
    p->txData[12] = *((uint8_t *)&p->channel3 + 2);
    p->txData[13] = *((uint8_t *)&p->channel3 + 3);
    p->txData[14] = *((uint8_t *)&p->channel4);
    p->txData[15] = *((uint8_t *)&p->channel4 + 1);
    p->txData[16] = *((uint8_t *)&p->channel4 + 2);
    p->txData[17] = *((uint8_t *)&p->channel4 + 3);
    /* Start transfer data frame */
    if (p->tcFlag == 1)
    {
      USART2_StartTransfer();
      p->frameCntr++;
      p->tcFlag = 0;
    }
    if (p->frameCntr >= (DATALOG_STERAM_SIZE - 1))
    {
      p->recordCmplt = 0;
      p->frameCntr = 0;
      if (p->autoTrigEn != 0)
      {
        p->trigger = 1;
      }
    }
  }
}

#endif