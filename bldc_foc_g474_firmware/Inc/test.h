/**
 * @brief Digital 1-st order filters 
 * Time constant computation:
 * for LPF: T = Ts / (Tf + Ts), Ts - sample time, Tf - filter time constant
 * for HPF: T = Tf/(Tf + Ts)
 * 
 * 
 * Init: @code lpf1.T = T_S / (1.0f / (M_2PI * F_CUT) + T_S); @endcode
 * Update:
 * @code lpf1.in = adcData;
 * LPF_calc(&lpf1);
 * filteredData = lpf1.out;
 * @endcode
 */

#ifndef __TEST_H__
#define __TEST_H__

struct test_s
{
    float in;   ///< Internal: raw data[n]
    float inPr; ///< Internal: raw data[n-1]
    float out;  ///< Output: filtered data[n]
    float T;    ///< Internal: Filter time constant
    void (*calc)(struct test_s *);
};
//typedef volatile struct test_s test_t;

#define TEST_DEFAULTS             \
    {                             \
        0.0f, 1e-20f, 0.0f, 0.0f, \
            &test_calc,           \
    }

static inline void test_calc(struct test_s *p)
{
    p->out = (p->T * (p->in - p->out)) + p->out;
}

#endif

/***************************** END OF FILE ****/
