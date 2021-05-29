/**
 * Digital 1-st order filters 
 * if f >> f_cut: Amp = sqrt(1 + w^2*Tf^2). In f_cut Amp = -3db = 0.70794578
 * Phase = -arctg(w*T). In f_cut, Phase = 45deg = PI/4 = 0.78539816
 * for LPF: T = Ts / (Tf + Ts), Ts - sample time, Tf - filter time constant
 * for HPF: T = Tf/(Tf + Ts)
 * 
 * cast example:
 * Init: lpf1.T = T_S / (1.0f / (M_2PI * F_CUT) + T_S);
 * Update: lpf1.in = adcData;
 *  	   LPF_calc(&lpf1);
 *		   filteredData = lpf1.out;
 */
#ifndef __DIGITALFILTERS_H__
#define __DIGITALFILTERS_H__


struct filterData_s
{
	float in;
	float inPr;
	float out;
	float T; 
};
typedef volatile struct filterData_s filterData_t;

#define FILTER_DEFAULTS \
	{                   \
		0.0f, 1e-20f, 0.0f, 0.0f     \
	}

static inline void LPF_calc(filterData_t *p)
{
	p->out = (p->T * (p->in - p->out)) + p->out;
}

static inline void HPF_calc(filterData_t *p)
{
	p->out = p->T * (p->out + (p->in - p->inPr));
	p->inPr = p->in;
}

#endif

/***************************** END OF FILE ****/
