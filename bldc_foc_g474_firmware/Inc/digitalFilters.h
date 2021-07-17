/**
 * @file 
 * @brief Digital 1-st order filters 
 * Time constant computation:
 * for LPF: T = Ts / (Tf + Ts), Ts - sample time, Tf - filter time constant
 * for HPF: T = Tf/(Tf + Ts)
 * 
 * @param[in] src Исходная область памяти
 * @param[in] n Количество байтов, которые необходимо скопировать
 * 
 * @example:
 * Init: @code lpf1.T = T_S / (1.0f / (M_2PI * F_CUT) + T_S); @endcode
 * Update:
 * @code lpf1.in = adcData;
 *  	 LPF_calc(&lpf1);
 *		 filteredData = lpf1.out;
 * @endcode
 */
#ifndef __DIGITALFILTERS_H__
#define __DIGITALFILTERS_H__

struct filterData_s
{
	float in;						   ///< Internal: raw data[n]
	float inPr;						   ///< Internal: raw data[n-1]
	float out;						   ///< Output: filtered data[n]
	float T;						   ///< Internal: Filter time constant
};
typedef volatile struct filterData_s filterData_t;

#define FILTER_DEFAULTS           \
	{                             \
		0.0f, 1e-20f, 0.0f, 0.0f, \
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
