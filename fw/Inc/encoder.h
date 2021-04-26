/* =================================================================================
File name:       PARK.H 
===================================================================================*/

#ifndef __ENCODER_H__
#define __ENCODER_H__

typedef volatile struct
{
	_iq theta;				/* Output: encoder output angle (rad) */
	_iq angleOffset;		/* Input: angle offset betwen encoder 0 and phase A */
	_iq thetaRaw;			/* Var: encoder output angle in ticks */
	uint8_t index;			/* Input: index flag */
	uint32_t counter;   	/* Input: encoder TIM counter value */
	uint8_t resolution; 	/* Parameter: encoder counter resolution in number of bits */
	uint8_t dir:1;			/* Parameter: 0 - 1. direction (for easy inverse angle) */
	uint8_t calibFlag:1;	/* Output: 0 - 1. setting to 1 when calibration is done */
} encoder_t;

static inline void encoder_calc(encoder_t *p)
{
	p->thetaRaw = p->counter + p->angleOffset; // theta = theta + theta_offset

	if (p->thetaRaw < 0)
		p->thetaRaw = p->thetaRaw + (1 << p->resolution);
	else if (p->thetaRaw > (1 << p->resolution))
		p->thetaRaw = p->thetaRaw - (1 << p->resolution);

	if (p->dir > 0)
		p->theta = ((1 << p->resolution) - p->thetaRaw) - (1 << (p->resolution - 1));
	else
		p->theta = (p->thetaRaw) - (1 << (p->resolution - 1));
	p->theta = _IQmpy((p->theta << (GLOBAL_Q - p->resolution)), _IQ(M_2PI));
}

#endif // __ENCODER_H__
