/* =================================================================================
File name:        HALL.H                  
                    
Description: 
=====================================================================================
 
------------------------------------------------------------------------------*/
#ifndef __HALL_H__
#define __HALL_H__

#include "main.h"

//------------------------------------------------
struct hall_s
{
    uint8_t A, B, C, state, statePr, stateNext;
    uint32_t isrCntr;
    uint8_t dir, offsetState, detectFlag, offsetFlag;
    float offset, angle, angleTmp, angleRaw, time, delta, angleInc, speedE;
    float offsetFwd[6]; // Angle value in Hall sensor rising edge
    float offsetRev[6]; // Angle value in Hall sensor falling edge
    float offsetAvg[6]; // Angle value in Hall sensor middle
};
typedef volatile struct hall_s hall_t;

#define HALL_DEFAULTS               \
    {                               \
        0, 0, 0, 0, 0, 0,           \
            0,                      \
            0, 0, 0, 0,             \
            0, 0, 0, 0, 0, 0, 0, 0, \
            {0}, {0}, {0},          \
    }

/** Update angle position using Hall sensors table and interpolation
 * beetwen midle of points **/
static inline void Hall_update(hall_t *p)
{
    /* Increment ISR counter */
    p->isrCntr++;
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
        p->angle = p->angleRaw;
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
    p->stateNext = (p->dir == 0) ? p->state+1 : p->state-1;
    p->stateNext = (p->dir == 0 && p->state == 5) ? 0 : p->stateNext;
    p->stateNext = (p->dir == 1 && p->state == 0) ? 5 : p->stateNext;
    float diff = fabsf(utils_angle_difference_rad(p->offsetAvg[p->state], p->offsetAvg[p->stateNext]));
    p->delta = diff * ((float)p->isrCntr / p->time);
    p->delta = SAT(p->delta, diff, 0);
    p->angle = (p->dir == 0) ? p->angleRaw + p->delta : p->angleRaw - p->delta;

    p->statePr = p->state;
    /* Calc angle interpolation */
    // p->delta = M_PI3 * ((float)p->isrCntr / p->time);
    // p->delta = SAT(p->delta, M_PI3, 0.f);
    // p->angle = (p->dir == 0) ? p->angleRaw + p->delta : p->angleRaw - p->delta;
    /* Normalize to range -PI to PI */
    if (p->angle < -MF_PI)
        p->angle = p->angle + M_2PI;
    else if (p->angle > MF_PI)
        p->angle = p->angle - M_2PI;
    /* If speed < speedMin use raw angle(60 deg resolution) */
    p->angle = (fabsf(p->speedE) <= 25.f) ? p->angleRaw : p->angle;
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