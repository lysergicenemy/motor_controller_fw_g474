/* =================================================================================
File name:        HALL.H                  
                    
Description: 
=====================================================================================
 
------------------------------------------------------------------------------*/
#ifndef __HALL_H__
#define __HALL_H__

#include "main.h"

#define READ_HALL1 (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_7))
#define READ_HALL2 (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_8))
#define READ_HALL3 (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_9))

//------------------------------------------------
struct hall_s
{
    uint8_t A, B, C, state, statePr, stateNext;
    uint32_t isrCntr;
    uint8_t dir, offsetState, detectFlag, offsetFlag;
    float offset, angle, angleTmp, angleRaw, time, delta, angleInc;
    float speedSector, speedE, speedE_filtered; // Motor electrical speed in rad/s
    float diff, diffNext, ts;
    float offsetFwd[6]; // Angle value in Hall sensor rising edge
    float offsetRev[6]; // Angle value in Hall sensor falling edge
    float offsetAvg[6]; // Angle value in Hall sensor middle
};
typedef volatile struct hall_s hall_t;

#define HALL_DEFAULTS            \
    {                            \
        0, 0, 0, 0, 0, 0,        \
            0,                   \
            0, 0, 0, 0,          \
            0, 0, 0, 0, 0, 0, 0, \
            0, 0, 0,             \
            0, 0, 0,             \
            {0}, {0}, {0},       \
    }

static inline void Hall_read(hall_t *p, int samples)
{
    samples = 1 + 2 * samples;

    int h1 = 0, h2 = 0, h3 = 0;
    int tres = samples / 2;

    while (samples--)
    {
        h1 += READ_HALL1;
        h2 += READ_HALL2;
        h3 += READ_HALL3;
    }
    p->A = (h1 > tres);
    p->B = (h2 > tres);
    p->C = (h3 > tres);
}
/** Update angle position using Hall sensors table and interpolation
 * beetwen midle of points **/
static inline void Hall_update(hall_t *p)
{
    /* Increment ISR counter */
    p->isrCntr++;
    p->time += p->ts;
    /* Calc sensor state and check validity */
    // if (p->A == 0 && p->B == 1 && p->C == 0)
    // {
    //     if (p->statePr == 5)
    //     {
    //         p->state = 0;
    //         p->dir = 0;
    //     }
    //     if (p->statePr == 1)
    //     {
    //         p->state = 0;
    //         p->dir = 1;
    //     }
    // }
    // else if (p->A == 0 && p->B == 1 && p->C == 1)
    // {
    //     if (p->statePr == 0)
    //     {
    //         p->state = 1;
    //         p->dir = 0;
    //     }
    //     if (p->statePr == 2)
    //     {
    //         p->state = 1;
    //         p->dir = 1;
    //     }
    // }
    // else if (p->A == 0 && p->B == 0 && p->C == 1)
    // {
    //     if (p->statePr == 1)
    //     {
    //         p->state = 2;
    //         p->dir = 0;
    //     }
    //     if (p->statePr == 3)
    //     {
    //         p->state = 2;
    //         p->dir = 1;
    //     }
    // }
    // else if (p->A == 1 && p->B == 0 && p->C == 1)
    // {
    //     if (p->statePr == 2)
    //     {
    //         p->state = 3;
    //         p->dir = 0;
    //     }
    //     if (p->statePr == 4)
    //     {
    //         p->state = 3;
    //         p->dir = 1;
    //     }
    // }
    // else if (p->A == 1 && p->B == 0 && p->C == 0)
    // {
    //     if (p->statePr == 3)
    //     {
    //         p->state = 4;
    //         p->dir = 0;
    //     }
    //     if (p->statePr == 5)
    //     {
    //         p->state = 4;
    //         p->dir = 1;
    //     }
    // }
    // else if (p->A == 1 && p->B == 1 && p->C == 0)
    // {
    //     if (p->statePr == 4)
    //     {
    //         p->state = 5;
    //         p->dir = 0;
    //     }
    //     if (p->statePr == 0)
    //     {
    //         p->state = 5;
    //         p->dir = 1;
    //     }
    // }
    /* Calc sensor state */
    if (p->A == 0 && p->B == 1 && p->C == 0)
        p->state = 0;
    else if (p->A == 0 && p->B == 1 && p->C == 1)
        p->state = 1;
    else if (p->A == 0 && p->B == 0 && p->C == 1)
        p->state = 2;
    else if (p->A == 1 && p->B == 0 && p->C == 1)
        p->state = 3;
    else if (p->A == 1 && p->B == 0 && p->C == 0)
        p->state = 4;
    else if (p->A == 1 && p->B == 1 && p->C == 0)
        p->state = 5;
    /* Calc angle 60deg resolution */
    p->angleRaw = p->offsetAvg[p->state];
    /* Check new state and reset counter */
    if (p->state != p->statePr)
    {
        /* Calc time between  states and angle increment value */
        //p->time = (float)p->isrCntr * p->ts;
        //p->time = (float)(TIM6->CNT) / 2982456.f;//* 0.00000033529f;
        //TIM6->CNT = 0;
        p->diff = fabsf(utils_angle_difference_rad(p->offsetAvg[p->statePr], p->offsetAvg[p->state]));
        p->speedSector = p->diff / p->time;
        UTILS_LP_FAST(p->speedE_filtered, p->speedSector, 0.01f);
        //p->time = (float)p->isrCntr;
        //p->angle = p->angleRaw;
        //p->angle = p->angleRaw + (utils_angle_difference_rad(p->offsetAvg[p->state], p->offsetAvg[p->statePr]) * 0.5f);
        p->isrCntr = 0;
        p->delta = 0;
        p->time = 0;

        if (p->state > p->statePr)
        {
            p->dir = (p->state != 5) ? 0 : p->dir;
        }
        if (p->state < p->statePr)
        {
            p->dir = (p->state != 0) ? 1 : p->dir;
        }
    }
    p->stateNext = (p->dir == 0) ? p->state + 1 : p->state - 1;
    p->stateNext = (p->dir == 0 && p->state == 5) ? 0 : p->stateNext;
    p->stateNext = (p->dir == 1 && p->state == 0) ? 5 : p->stateNext;
    /* Calc angle interpolation */
    p->diffNext = fabsf(utils_angle_difference_rad(p->offsetAvg[p->state], p->offsetAvg[p->stateNext]));
    p->delta = fabsf(p->speedE_filtered * ((float)p->isrCntr * p->ts));
    UTILS_NAN_ZERO(p->delta);
    //float phaseCorrect = p->diff * 0.5f;
    p->delta = SAT(p->delta, p->diffNext, 0);
    //p->angle = (p->dir == 0) ? p->angleRaw + p->delta : p->angleRaw - p->delta;
    p->angle = (p->dir == 0) ? p->angleRaw + p->delta : p->angleRaw - p->delta;

    p->statePr = p->state;
    /* Normalize to range -PI to PI */
    if (p->angle < -MF_PI)
        p->angle = p->angle + M_2PI;
    else if (p->angle > MF_PI)
        p->angle = p->angle - M_2PI;
    /* If speed < speedMin use raw angle(60 deg resolution) */
    p->angle = (fabsf(p->speedE) <= 50.f) ? p->angleRaw : p->angle;
}

/** Update angle position using Hall sensors estimated offset 
 * and interpolation **/
static inline void Hall_updateNoTable(hall_t *p)
{
    /* Calc sensor state */
    if (p->A == 0 && p->B == 1 && p->C == 0)
    {
        p->angleRaw = -MF_PI;
        p->state = 0;
    }
    else if (p->A == 0 && p->B == 1 && p->C == 1)
    {
        p->angleRaw = -(M_PI3 * 2.f);
        p->state = 1;
    }
    else if (p->A == 0 && p->B == 0 && p->C == 1)
    {
        p->angleRaw = -M_PI3;
        p->state = 2;
    }
    else if (p->A == 1 && p->B == 0 && p->C == 1)
    {
        p->angleRaw = 0.f;
        p->state = 3;
    }
    else if (p->A == 1 && p->B == 0 && p->C == 0)
    {
        p->angleRaw = M_PI3;
        p->state = 4;
    }
    else if (p->A == 1 && p->B == 1 && p->C == 0)
    {
        p->angleRaw = M_PI3 * 2.f;
        p->state = 5;
    }
    /* Normalize to range -PI to PI */
    if (p->angleRaw < -MF_PI)
        p->angleRaw = p->angleRaw + M_2PI;
    else if (p->angleRaw > MF_PI)
        p->angleRaw = p->angleRaw - M_2PI;
    /* Check new state and reset counter */
    if (p->state != p->statePr)
    {
        /* Calc time between  states and angle increment value */
        p->time = (float)p->isrCntr;
        p->angleTmp = p->angleRaw;
        p->isrCntr = 0;
        if (p->state > p->statePr)
        {
            p->dir = (p->state != 5) ? 0 : p->dir;
        }
        if (p->state < p->statePr)
        {
            p->dir = (p->state != 0) ? 1 : p->dir;
        }
    }
    p->statePr = p->state;
    /* Calc angle interpolation */
    p->delta = M_PI3 * ((float)p->isrCntr / p->time);
    p->delta = SAT(p->delta, M_PI3, 0.f);
    p->angleTmp = (p->dir == 0) ? p->angleRaw + p->delta : p->angleRaw - p->delta;
    p->angle = p->angleTmp + p->offset;
    /* Normalize to range -PI to PI */
    if (p->angle < -MF_PI)
        p->angle = p->angle + M_2PI;
    else if (p->angle > MF_PI)
        p->angle = p->angle - M_2PI;
    /* If speed < speedMin use raw angle(60 deg resolution) */
    p->angle = (fabsf(p->speedE) <= 25.f) ? p->angleRaw : p->angle;
}

#endif