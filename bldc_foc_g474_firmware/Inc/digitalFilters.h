/**
 * @file 
 * @brief Digital 1-st order filters 
 * Time constant computation:
 * for LPF: T = Ts / (Tf + Ts), Ts - sample time, Tf - filter time constant
 * for HPF: T = Tf / (Tf + Ts)
 * 
 * @param[in] 
 * @param[in] 
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

#define DF_SQ(x) ((x) * (x)) // sqare

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
/** 1-st order low-pass filter
 * 
*/
static inline void LPF_calc(filterData_t *p)
{
	p->out = (p->T * (p->in - p->out)) + p->out;
}

/** 1-st order high-pass filter
 * 
*/
static inline void HPF_calc(filterData_t *p)
{
	p->out = p->T * (p->out + (p->in - p->inPr));
	p->inPr = p->in;
}

/** 1-st order low-pass filter
 * with compensation phase lag and
 * attenuation
 * 
*/
struct lpf_phAmpComp_s
{
	float ws, wf;
	float attLPF, attHPF;
	float wfHPF, phaseLPF;
	float ratio;
	filterData_t lpf;
	filterData_t hpf;
};
typedef volatile struct lpf_phAmpComp_s lpf_phAmpComp_t;

static inline void LPF_compPhaseAmp_calc(lpf_phAmpComp_t *p)
{
	p->ratio = (p->ws / p->wf);
	// calc low-pass filter attenuation
	p->attLPF = 1.f / sqrtf(1.f + DF_SQ(p->ws / p->wf));
	p->phaseLPF = -atanf(p->ws / p->wf);
	
}

#endif

/***************************** END OF FILE ****/
