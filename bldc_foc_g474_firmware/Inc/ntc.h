/* =================================================================================
File name:        NTC.H                  
                    
Description: 
=====================================================================================
 
------------------------------------------------------------------------------*/
#ifndef __NTC_H__
#define __NTC_H__

#include "math.h"

//------------------------------------------------

struct ntc_s
{
	float r_balance;
	float r_ntc_0;
	float ta_0;
	float betta;
	float temp;
	float r_ntc;
	float u;
};
typedef volatile struct ntc_s ntc_t;

/**
 * input: ntc_t - NTC data type
 *            u - scaled adcData to range 0.0f - 1.0f
*/
static inline void ntc_temperature(ntc_t *ntc)
{
	/* if Vcc -> Rb -> sensePoint -> NTC -> GND */
	//ntc->r_ntc = ntc->r_balance * ntc->u / (1.f - ntc->u);

	/* if Vcc -> NTC -> sensePoint -> Rb -> GND */
	ntc->r_ntc = (ntc->r_balance - (ntc->u * ntc->r_balance)) / ntc->u;
	ntc->temp = 1.f / (1.f / (ntc->ta_0 + 273.f) + logf(ntc->r_ntc / ntc->r_ntc_0) / ntc->betta) - 273.f;
}

#endif