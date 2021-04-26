/**
 ***********************************************************************
 * File name: dataLog.h
 * Discription: DataLoger for Freemaster
 *  
 *  */

#ifndef DATALOG_H
#define DATALOG_H

#define DATALOG_STERAM_SIZE 256

/*---------- DataLog struct --------------*/
struct dataLogVars_s
{
  uint8_t trigger;
  uint8_t recordCmplt;
  uint8_t autoTrigEn;
  uint16_t psc;
  uint32_t frameCntr;
  uint32_t isrCntr;
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

#endif