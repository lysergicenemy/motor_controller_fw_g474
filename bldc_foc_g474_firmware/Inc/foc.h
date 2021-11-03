/**
 ***********************************************************************
 * @file : foc.h
 * @brief: Implementation math library for DC/BLDC/PMSM
 *  6step/field-oriented control.
 * @example:
 * @code: bldc1.pi_id.Ref = 1.0f;
 *        bldc1.calc.pi_reg(&bldc1.pi_id); 
 * @endcode
 *  */

#ifndef FOC_H
#define FOC_H

/**
 * @brief Function pointers to hardware drivers
 * @param: ta, tb, tc, duty, *s, *c, x, y - range -1.f : 1.f
 * @param: angle - range -pi : pi
 */
void (*foc_pwm_init)(volatile float pwmFreq, volatile float deadtime_ns, uint32_t pwmPeriod_adcTrig_ratio, volatile float *halfPeriod);
void (*foc_pwm_off)(void);
void (*foc_pwm_on)(void);
void (*foc_pwm_update)(float ta, float tb, float tc, float halfPwmPeriod);
void (*foc_bldcpwm_update)(float halfPwmPeriod, float duty, uint8_t sector);
void (*foc_sincos)(volatile float angle, volatile float *s, volatile float *c);
void (*foc_atan2)(volatile float x, volatile float y, volatile float *angle);

/*********************************************************************
 * Space-vector PWM generator.
 * Input: usa, usb - voltages in alpha/beta reference frame
 * Output: Ta, Tb, Tc - switching functions for half-bridges
 ****************************************************************** */
struct svgen_s
{
    float usa, usb;    // Input: reference voltages alfa-beta ref.frame
    float udc;         // Input: DC-Bus voltage
    float Ta;          // Output: reference phase-a switching function
    float Tb;          // Output: reference phase-b switching function
    float Tc;          // Output: reference phase-c switching function
    float tmp1;        // Variable: temp variable
    float tmp2;        // Variable: temp variable
    float tmp3;        // Variable: temp variable
    uint8_t VecSector; // Variable: Space vector sector
};
typedef volatile struct svgen_s svgen_t;
#define SVGEN_DEFAULTS  \
    {                   \
        0, 0, 0, 0, 0,  \
            0, 0, 0, 0, \
    }

static inline void Svgen_calc(svgen_t *p)
{
    float_t Vmax_pu = 0, Vmin_pu = 0, Vcom_pu;
    float_t oneOverDcBus_invV = 1.f / p->udc;

    float_t Va_pu = p->usa * oneOverDcBus_invV;
    float_t Vbeta_pu = p->usb * oneOverDcBus_invV;

    float_t Va_tmp = 0.5f * (-Va_pu);
    float_t Vb_tmp = SQRT3_BY_2 * Vbeta_pu;
    float_t Vb_pu = Va_tmp + Vb_tmp; // -0.5*Valpha + sqrt(3)/2 * Vbeta;
    float_t Vc_pu = Va_tmp - Vb_tmp; // -0.5*Valpha - sqrt(3)/2 * Vbeta;

    // find Vmax and Vmin
    if (Va_pu > Vb_pu)
    {
        Vmax_pu = Va_pu;
        Vmin_pu = Vb_pu;
    }
    else
    {
        Vmax_pu = Vb_pu;
        Vmin_pu = Va_pu;
    }

    if (Vc_pu > Vmax_pu)
    {
        Vmax_pu = Vc_pu;
    }
    else if (Vc_pu < Vmin_pu)
    {
        Vmin_pu = Vc_pu;
    }
    else
    {
    }

    // compute Vcom
    Vcom_pu = 0.5f * (Vmax_pu + Vmin_pu); // 0.5*(Vmax+Vmin)

    // Subtract common-mode term to achieve SV modulation
    p->Ta = (Va_pu - Vcom_pu);
    p->Tb = (Vb_pu - Vcom_pu);
    p->Tc = (Vc_pu - Vcom_pu);
}

/*********************************************************************
 * Digital PI(D) controller
 * Input: Ref, Fdb - reference and feadback
 * Output: Out - output from controller
 ****************************************************************** */
struct pidReg_s
{
    float Ref;       // Input: Reference input
    float Fdb;       // Input: Feedback input
    float Fdfwd;     // Input: Feedforward input
    float Err;       // Variable: Error
    float Kp;        // Parameter: Proportional gain
    float Up;        // Variable: Proportional output
    float Ui;        // Variable: Integral output
    float Ud;        // Variable: Derivative output
    float OutPreSat; // Variable: Pre-saturated output
    float OutMax;    // Parameter: Maximum output
    float OutMin;    // Parameter: Minimum output
    float Out;       // Output: PID output
    float SatErr;    // Variable: Saturated difference
    float Ki;        // Parameter: Integral gain
    float Kc;        // Parameter: Integral correction gain
    float Kd;        // Parameter: Derivative gain
    float Up1;       // History: Previous proportional output
};
typedef volatile struct pidReg_s pidReg_t;
#define PIDREG_DEFAULTS \
    {                   \
        0, 0, 0, 0, 0,  \
            0, 0, 0, 0, \
            0, 0, 0, 0, \
            0, 0, 0, 0, \
    }

/* Digital PI controller. Parallel form. Antiwindup: dynamic integrator clamp */
static inline void PI_calc(pidReg_t *p)
{
    p->Err = p->Ref - p->Fdb;                         /* Compute the error */
    p->Up = p->Kp * p->Err;                           /* Compute the proportional output */
    p->Ui += p->Ki * p->Err;                          /* Compute the integral output */
    p->Ud = p->Kd * (p->Err - p->Up1);                /* Derivative term */
    p->SatErr = p->OutMax - fabsf(p->Up + p->Ud);     /* Inegrator saturation value */
    p->Ui = SAT(p->Ui, p->SatErr, -p->SatErr);        /* Saturate the integral output */
    p->OutPreSat = p->Up + p->Ui + p->Ud + p->Fdfwd;  /* Compute the pre-saturated output */
    p->Out = SAT(p->OutPreSat, p->OutMax, p->OutMin); /* Saturate the output */
    p->Up1 = p->Err;
}

/* Digital PI controller. Series form. Antiwindup: back-calculation */
static inline void PI_antiwindup_calc(pidReg_t *p)
{
    p->Err = p->Ref - p->Fdb;                         /* Compute the error */
    p->Up = p->Kp * p->Err;                           /* Compute the proportional output */
    p->Ui += p->Ki * p->Up + p->Kc * p->SatErr;       /* Compute the integral output */
    p->Ud = p->Kd * (p->Err - p->Up1);                /* Derivative term */
    p->OutPreSat = p->Up + p->Ui + p->Ud + p->Fdfwd;  /* Compute the pre-saturated output */
    p->Out = SAT(p->OutPreSat, p->OutMax, p->OutMin); /* Saturate the output */
    p->SatErr = p->Out - p->OutPreSat;
    p->Up1 = p->Err;
}

/*********************************************************************
 * @brief Phase voltages calculation
 * @input: DcBusVolt, MfuncV1, MfuncV2, MfuncV3
 * @output: VphaseA, VphaseB, VphaseC
 ****************************************************************** */
struct voltCalc_s
{
    float DcBusVolt;                 // Input: DC-bus voltage
    float MfuncV1;                   // Input: Modulation voltage phase A (pu)
    float MfuncV2;                   // Input: Modulation voltage phase B (pu)
    float MfuncV3;                   // Input: Modulation voltage phase C (pu)
    uint8_t OutOfPhase;              // Parameter: Out of Phase adjustment (0 or 1) (Q0) - independently with global Q
    float VphaseA;                   // Output: Phase voltage phase A
    float VphaseB;                   // Output: Phase voltage phase B
    float VphaseC;                   // Output: Phase voltage phase C
    float usa;                       // Output: Stationary alpha-axis phase voltage
    float usb;                       // Output: Stationary beta-axis phase voltage
    float usd, usq;                  // Output: DQ-axis phase voltages
    float magMaxD, magMaxQ, dutyMax; // Output: DQ-axis phase voltages
    float dutyQ, dutyD;              // Output: DQ-axis duty cycles
    float temp;                      // Variable: temp variable
};
typedef volatile struct voltCalc_s voltCalc_t;
#define VOLTCALC_DEFAULTS \
    {                     \
        0,                \
            0, 0, 0,      \
            0,            \
            0, 0, 0,      \
            0, 0,         \
            0, 0,         \
            0, 0, 0,      \
            0, 0,         \
            0,            \
    }

static inline void Volt_calc(voltCalc_t *p)
{
    /* Scale the incomming Modulation functions with the DC bus voltage value*/
    /* and calculate the 3 Phase voltages */
    p->temp = p->DcBusVolt * 0.3333333f;
    p->VphaseA = p->temp * ((p->MfuncV1 * 2) - p->MfuncV2 - p->MfuncV3);
    p->VphaseB = p->temp * ((p->MfuncV2 * 2) - p->MfuncV1 - p->MfuncV3);
    p->VphaseC = p->temp * ((p->MfuncV3 * 2) - p->MfuncV2 - p->MfuncV1);

    if (p->OutOfPhase != 0)
    {
        p->VphaseA = -p->VphaseA;
        p->VphaseB = -p->VphaseB;
    }
    /* Voltage transformation (a,b,c)  ->  (Alpha,Beta)	*/
    p->usa = p->VphaseA;
    p->usb = ONE_BY_SQRT3 * p->VphaseA + TWO_BY_SQRT3 * p->VphaseB;
}

/*********************************************************************
 * @brief Flux observer
 * @input: alfa-beta currents and voltages
 * @parameters: fluxLinkage, gamma
 * @output: phase
 ****************************************************************** */
struct fluxObs_s
{
    float i_alpha;
    float i_beta;
    float v_alpha;
    float v_beta;
    float fluxLinkage; // Magnets flux linkage, Wb
    float R;
    float L;
    float x1, x1_prev;
    float x2, x2_prev;
    float err;
    float dt;
    float phase;
    float gamma;
};
typedef volatile struct fluxObs_s fluxObs_t;
#define FLUX_OBS_DEFAULTS           \
    {                               \
        0, 0, 0, 0, 0, 0, 0,        \
            0, 0, 0, 0, 0, 0, 0, 0, \
    }

// See http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
static inline void FluxObs_calc(fluxObs_t *p)
{
    float L_ia = p->L * p->i_alpha;
    float L_ib = p->L * p->i_beta;
    float R_ia = p->R * p->i_alpha;
    float R_ib = p->R * p->i_beta;
    float lambda_2 = SQ(p->fluxLinkage);

    p->err = lambda_2 - (SQ(p->x1 - L_ia) + SQ(p->x2 - L_ib));
    float x1_dot = -R_ia + p->v_alpha + p->gamma * (p->x1 - L_ia) * p->err;
    float x2_dot = -R_ib + p->v_beta + p->gamma * (p->x2 - L_ib) * p->err;
    p->x1 += x1_dot * p->dt;
    p->x2 += x2_dot * p->dt;

    UTILS_NAN_ZERO(p->x1);
    UTILS_NAN_ZERO(p->x2);

    /* 1: Original */
    foc_atan2(p->x1 - L_ia, p->x2 - L_ib, &p->phase);
    /* 2: Like VESC  */
    //p->phase = utils_fast_atan2(p->x2_prev + p->x2, p->x1_prev + p->x1);
    //p->x1_prev = p->x1;
    //p->x1_prev = p->x2;
}
/*********************************************************************
 * @brief Sliding mode rotor position observer
 * with low-pass filter phase shift compensation.
 * @input: Valpha, Vbeta, Ialpha, Ibeta - phase voltages and currents in A-B reference frame
 * @output: Theta - estimated angle
 ****************************************************************** */
struct smopos_s
{
    float Valpha;      // input: Stationary alfa-axis stator voltage
    float Vbeta;       // Input: Stationary beta-axis stator voltage
    float Ialpha;      // Input: Stationary alfa-axis stator current
    float Ibeta;       // Input: Stationary beta-axis stator current
    float Ealpha;      // Variable: Stationary alfa-axis back EMF
    float Ebeta;       // Variable: Stationary beta-axis back EMF
    float Zalpha;      // Output: Stationary alfa-axis sliding control
    float Zbeta;       // Output: Stationary beta-axis sliding control
    float EstIalpha;   // Variable: Estimated stationary alfa-axis stator current
    float EstIbeta;    // Variable: Estimated stationary beta-axis stator current
    float IalphaError; // Variable: Stationary alfa-axis current error
    float IbetaError;  // Variable: Stationary beta-axis current error
    float Theta;       // Output: Compensated rotor angle
    float ThetaLag;    // Output: Compensated rotor angle
    float speedE;      // Output: Compensated rotor angle
    float Gsmopos;     // Parameter: Motor dependent control gain
    float Fsmopos;     // Parameter: Motor dependent plant matrix
    float lpfWc;       // Parameter: Sliding control filter gain
    float lpfTf;       // Parameter: Sliding control filter gain
    float vdc;         // Input: Input angular frequency
};
typedef volatile struct smopos_s smopos_t;
#define SMOPOS_DEFAULTS                  \
    {                                    \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
            0, 0, 0, 0, 0, 0, 0, 0, 0,   \
    }

static inline void Smopos_calc(smopos_t *p)
{
    /*	Sliding mode current observer	*/
    p->EstIalpha = p->Fsmopos * p->EstIalpha + p->Gsmopos * (p->Valpha - p->Ealpha - p->Zalpha);
    p->EstIbeta = p->Fsmopos * p->EstIbeta + p->Gsmopos * (p->Vbeta - p->Ebeta - p->Zbeta);
    UTILS_NAN_ZERO(p->EstIalpha);
    UTILS_NAN_ZERO(p->EstIbeta);
    /*	Current errors	*/
    p->IalphaError = p->EstIalpha - p->Ialpha;
    p->IbetaError = p->EstIbeta - p->Ibeta;
    /*  Sliding control calculator	*/
    p->Zalpha = SAT(p->IalphaError, 0.5f, -0.5f) * (p->vdc);
    p->Zbeta = SAT(p->IbetaError, 0.5f, -0.5f) * (p->vdc);
    /* Low-pass filtering output of sliding controller */
    p->Ealpha = p->Ealpha + (p->lpfTf * (p->Zalpha - p->Ealpha));
    p->Ebeta = p->Ebeta + (p->lpfTf * (p->Zbeta - p->Ebeta));
    UTILS_NAN_ZERO(p->Ealpha);
    UTILS_NAN_ZERO(p->Ebeta);
    /*	Rotor angle calculation	*/
    foc_atan2(p->Ebeta, -p->Ealpha, &p->Theta);
    /* Phase lag compensation */
    foc_atan2(p->speedE, p->lpfWc, &p->ThetaLag);
    p->Theta += p->ThetaLag;
    if (p->Theta < -MF_PI)
        p->Theta = p->Theta + M_2PI;
    else if (p->Theta > MF_PI)
        p->Theta = p->Theta - M_2PI;
}

/*********************************************************************
 * @brief BEMF observer for DC motor
 * @input: Ua, Ia - Armature voltage and current
 * @parameters: fluxLinkage - Magnets or field winding flux linlage
 * @output: BEMF
 ****************************************************************** */
struct bemf_s
{
    float ia;
    float ua;
};
typedef volatile struct bemf_s bemf_t;
#define BEMF_DEFAULTS \
    {                 \
        0, 0,         \
    }

static inline void Bemf_calc(fluxObs_t *p)
{
}

/*********************************************************************
 * Enum with fault codes
 ****************************************************************** */
enum faultCode_e
{
    FLT_NONE,      // Fault code: No fault
    FLT_OCP_U,     // Fault code: Overcurrent phase A
    FLT_OCP_V,     // Fault code: Overcurrent phase B
    FLT_OCP_W,     // Fault code: Overcurrent phase C
    FLT_OVP,       // Fault code: Overvoltage DC-Link
    FLT_OVT,       // Fault code: PCB Overtemperature
    FLT_UVLO,      // Fault code: Undervoltage DC-Link
    FLT_ID_NC,     // Fault code: Param ID - Motor not connected
    FLT_ID_OUT_MAG // Fault code: Param ID - referance mag > maximal mag
};
typedef volatile enum faultCode_e faultCode_t;

/*********************************************************************
 * Struct with Protection data
 ****************************************************************** */
struct prot_s
{
    faultCode_t fltCode;
    float ocpThld, ovpThld, uvloThld, pcbTempThld; // Protection thresholds, OCP for phase currents (A), DC_Link voltage protect OVP/UVLO (V)
    float currPhU, currPhV, currPhW, udc, tempPcb;
    uint8_t protFlag;
    uint8_t prechargeFlag;
    uint8_t enable;
};
typedef volatile struct prot_s prot_t;
#define PROTECTION_DEFAULTS \
    {                       \
        0,                  \
            0, 0, 0, 0,     \
            0, 0, 0, 0, 0,  \
            0, 0, 0,        \
    }

static inline void Prot_calc(prot_t *p)
{
    if (p->enable != 0)
    {
        if (p->currPhU >= p->ocpThld || p->currPhU <= -p->ocpThld)
        {
            p->fltCode = FLT_OCP_U;
            p->protFlag = 1;
        }
        if (p->currPhV >= p->ocpThld || p->currPhV <= -p->ocpThld)
        {
            p->fltCode = FLT_OCP_V;
            p->protFlag = 1;
        }
        if (p->currPhW >= p->ocpThld || p->currPhW <= -p->ocpThld)
        {
            p->fltCode = FLT_OCP_W;
            p->protFlag = 1;
        }
        if (p->udc >= p->ovpThld)
        {
            p->fltCode = FLT_OVP;
            p->protFlag = 1;
        }
        if (p->udc <= p->uvloThld)
        {
            if (p->prechargeFlag != 0)
            {
                p->fltCode = FLT_UVLO;
                p->protFlag = 1;
            }
        }
        if (p->tempPcb >= p->pcbTempThld)
        {
            if (p->prechargeFlag != 0)
            {
                p->fltCode = FLT_OVT;
                p->protFlag = 1;
            }
        }
    }
}

/*********************************************************************
 * Struct with PLL based angle and speed estimation
 ****************************************************************** */
struct pll_s
{
    float AngleRaw, dt;                 // Input: Raw electrical angle(rad), sample time(s)
    float Err;                          // Internal: Electrical angle error (delta Theta)
    float SpeedPll, AnglePll, SpeedAbs; // Output: Filtered Speed(rad/s), Angle(rad), module of filtered speed
    float Kp, Ki;                       // Parameter: PLL gains
};
typedef volatile struct pll_s pll_t;

/*-----------------------------------------------------------------------------
Default initalizer for the SPEED_MEAS_QEP object.
-----------------------------------------------------------------------------*/
#define PLL_DEFAULTS         \
    {                        \
        0, 0,                \
            0,               \
            0, 0,            \
            2000.f, 30000.f, \
    }

static inline void Pll_calc(pll_t *p)
{
    UTILS_NAN_ZERO(p->AnglePll);
    p->Err = (p->AngleRaw - p->AnglePll); // for avoid QNAN
    /* Normalize to range -PI to PI */
    if (p->Err < -MF_PI)
        p->Err = p->Err + M_2PI;
    else if (p->Err > MF_PI)
        p->Err = p->Err - M_2PI;
    /* calc pll angle */
    UTILS_NAN_ZERO(p->SpeedPll);
    p->AnglePll += (p->SpeedPll + p->Err * p->Kp) * p->dt;
    /* Normalize to range -PI to PI */
    if (p->AnglePll < -MF_PI)
        p->AnglePll = p->AnglePll + M_2PI;
    else if (p->AnglePll > MF_PI)
        p->AnglePll = p->AnglePll - M_2PI;
    /* calc pll speed */
    p->SpeedPll += (p->Err * p->Ki * p->dt);
    p->SpeedAbs = fabsf(p->SpeedPll);
}

/*********************************************************************
 * Struct with Maximim torque per amper (MTPA)
 ****************************************************************** */
struct mtpa_s
{
    uint8_t en;
    float iMax;
    float beta, k_mtpa, sinBeta, cosBeta;
    float Ld, Lq, Ldq_diff, lambdaPM;
    float iqRef, id_MTPA, iq_MTPA;
};
typedef volatile struct mtpa_s mtpa_t;

/*-----------------------------------------------------------------------------
Default initalizer for the MTPA object.
-----------------------------------------------------------------------------*/
#define MTPA_DEFAULTS   \
    {                   \
        0, 0,           \
            0, 0, 0, 0, \
            0, 0, 0, 0, \
            0, 0, 0,    \
    }

/* https://www.ti.com/lit/pdf/spracf3 */
static inline void Mtpa_calc(mtpa_t *p)
{
    p->id_MTPA = (p->lambdaPM - sqrtf(SQ(p->lambdaPM) + 8.f * SQ(p->Ldq_diff) * SQ(p->iqRef))) / (4.f * p->Ldq_diff);
    UTILS_NAN_ZERO(p->id_MTPA);
    p->iq_MTPA = SIGN(p->iqRef) * sqrtf(SQ(p->iqRef) - SQ(p->id_MTPA));
    UTILS_NAN_ZERO(p->iq_MTPA);
}

/*********************************************************************
 * Struct with Anti-cogging torque compensation
 ****************************************************************** */
struct antiCogg_s
{
    float coggingMap[100];
    uint8_t en;
    uint8_t tabNmbr;
    float udZero, udFltrd, uRef, uCogg;
    float angle; // Input: actual electrical angle
    float gain;  // compensation gain
    float scale; // 100 / (2 * pi)
};
typedef volatile struct antiCogg_s antiCogg_t;

/*-----------------------------------------------------------------------------
Default initalizer for the MTPA object.
-----------------------------------------------------------------------------*/
#define ANTICOGG_DEFAULTS           \
    {                               \
        {0}, 0, 0, 0, 0, 0,         \
            0, 0, 1.f, 15.9154943f, \
    }

/* require more testing! */
static inline void AntiCogg_calc(antiCogg_t *p)
{
    p->tabNmbr = (uint8_t)((p->angle + MF_PI) * p->scale);
    p->uCogg = (p->en == 1) ? (p->uRef * p->coggingMap[p->tabNmbr]) * p->gain : 0;
}

/* **********************************************************************************
Description: 
Commutation trigger logic in sensorless operation for BLDC mode
1) Find zero-cross poin.
2) Starting inegrate BEMF.
3) When BEMF integral grater than threshold value - switch to next step 
*********************************************************************************** */
struct cmtn_s
{
    uint8_t Trigger; // Output: commutation trigger flag
    float fluxBuff;
    float cmtnTh;
    float As, Bs, Cs;
    float An, Bn, Cn;
    uint8_t dir;
    uint8_t cmtnState;
    float halfVdc;
    float Neutral;
    filterData_t lpf_bemfA; // Data struct: bemf A low-pass filter data
    filterData_t lpf_bemfB; // Data struct: bemf B low-pass filter data
    filterData_t lpf_bemfC; // Data struct: bemf C low-pass filter data
};
typedef volatile struct cmtn_s cmtn_t;
#define CMTN_DEFAULTS        \
    {                        \
        0,                   \
            0,               \
            1.f,             \
            0, 0, 0,         \
            0, 0, 0,         \
            0,               \
            0,               \
            0,               \
            0,               \
            FILTER_DEFAULTS, \
            FILTER_DEFAULTS, \
            FILTER_DEFAULTS, \
    }

static inline void CMTN_run(volatile cmtn_t *obj)
{
    // clear flags on entry if state is changed:
    obj->Trigger = 0;
    obj->Neutral = (obj->As + obj->Bs + obj->Cs) * (1.f / 3.f);
    obj->An = obj->As - obj->Neutral;
    obj->Bn = obj->Bs - obj->Neutral;
    obj->Cn = obj->Cs - obj->Neutral;

    if (obj->dir == 1)
    {
        // Commutation State table Tasks; must correspond to bldcpwm.h table!
        switch (obj->cmtnState)
        {
        case 0: // State s1: current flows to motor windings from voltage A->B, de-energized voltage = C

            //  As we are interested in the zero crossing of the Bemf it is possible to check only for
            //  the Bemf sign change
            if (obj->Cn > 0.f)
            {
                obj->fluxBuff += obj->Cn;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 1: // State s2: current flows to motor windings from voltage A->C, de-energized voltage = B

            if (obj->Bn < 0.f)
            {
                obj->fluxBuff += obj->Bn;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 2: // State s3: current flows to motor windings from voltage B->C, de-energized voltage = A

            if (obj->An > 0.f)
            {
                obj->fluxBuff += obj->An;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 3: // State s4: current flows to motor windings from voltage B->A, de-energized voltage = C

            if (obj->Cn < 0.f)
            {
                obj->fluxBuff += obj->Cn;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 4: // State s5: current flows to motor windings from voltage C->A, de-energized voltage = B

            if (obj->Bn > 0.f)
            {
                obj->fluxBuff += obj->Bn;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 5: // State s6: current flows to motor windings from voltage C->B, de-energized voltage = A

            if (obj->An < 0.f)
            {
                obj->fluxBuff += obj->An;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        }
    }
    if (obj->dir == 0)
    {
        switch (obj->cmtnState)
        {
        case 0: // State s1: current flows to motor windings from voltage A->B, de-energized voltage = C
            if (obj->Cn < 0.f)
            {
                obj->fluxBuff += obj->Cn;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 1: // State s2: current flows to motor windings from voltage A->C, de-energized voltage = B

            if (obj->Bn > 0.f)
            {
                obj->fluxBuff += obj->Bn;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 2: // State s3: current flows to motor windings from voltage B->C, de-energized voltage = A

            if (obj->An < 0.f)
            {
                obj->fluxBuff += obj->An;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 3: // State s4: current flows to motor windings from voltage B->A, de-energized voltage = C

            if (obj->Cn > 0.f)
            {
                obj->fluxBuff += obj->Cn;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 4: // State s5: current flows to motor windings from voltage C->A, de-energized voltage = B

            if (obj->Bn < 0.f)
            {
                obj->fluxBuff += obj->Bn;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        case 5: // State s6: current flows to motor windings from voltage C->B, de-energized voltage = A

            if (obj->An > 0.f)
            {
                obj->fluxBuff += obj->An;             // integrate BEMF
                if (ABS(obj->fluxBuff) > obj->cmtnTh) // compare integral of BEMF with threshold
                {
                    obj->fluxBuff = 0.f; // reset integral
                    obj->Trigger = 1;    // set "switch to next state" trigger
                }
            }
            break;
        }
    }
}

/*********************************************************************
 * Struct with pointers for "calc" functions
 ****************************************************************** */
struct calc_s
{
    void (*pi_reg)(pidReg_t *p); /* Pointer to the PI-controller funcion           */
    void (*svgen)(svgen_t *p);   /* Pointer to the Space-vector generator funcion           */
    void (*smo)(smopos_t *p);    /* Pointer to the Observer funcion           */
    void (*volt)(voltCalc_t *p); /* Pointer to the volt calc function */
    void (*prot)(prot_t *p);     /* Pointer to the protection calc function */
    void (*cmtn)(cmtn_t *obj);   /* Pointer to the 6-step commutation calc function */
    void (*flux)(fluxObs_t *p);  /* Pointer to the flux observer calc function */
    void (*pll)(pll_t *p);       /* Pointer to the PLL calc function */
    void (*mtpa)(mtpa_t *p);     /* Pointer to the MTPA calc function */
    void (*cgtc)(antiCogg_t *p); /* Pointer to the cogging torque compensation calc function */
};
typedef volatile struct calc_s calc_t;
#define CALC_DEFAULTS      \
    {                      \
        PI_calc,           \
            Svgen_calc,    \
            Smopos_calc,   \
            Volt_calc,     \
            Prot_calc,     \
            CMTN_run,      \
            FluxObs_calc,  \
            Pll_calc,      \
            Mtpa_calc,     \
            AntiCogg_calc, \
    }

/*********************************************************************
 * Struct with general data(internal variables)
 ****************************************************************** */
struct data_s
{
    float udRef, uqRef;                            // Input: reference D/Q-axis voltages (V)
    float idRef, iqRef, iqRmp;                     // Input: reference D/Q-axis currents, id ramp rate (A/s)
    float speedRef, thetaRef, spdRmp, speedRpm;    // Input: reference speed, angle, speed ramp, measured speed in RPM
    float iPhaseA, iPhaseB, iPhaseC;               // Input: ABC currents (A)
    float vPhaseA, vPhaseB, vPhaseC;               // Input: measured phase voltages (V)
    float vAlpha, vBeta;                           // Input: measured alpha-beta components of phase voltages (V)
    float vd, vq;                                  // Input: measured D/Q components of phase voltages (V)
    float iAvg, iAvgFiltered, iStrtp, hwCurrLim;   // Input: Total current (A)
    float isa, isb, isd, isq;                      // Variable: D/Q-axis currents (A)
    float udDec, uqDec;                            // Variable: D/Q-axis voltages decoupling components
    float Pem, Pin, Pe_loss, efficiency;           // Variable: Motor power (shaft, input, losses) (W)
    float Te, Tmag, Trel;                          // Variable: Electrical torque (full, magnetic, reluctance) (N.M)
    float wBase, wMinSensorless, vMax;             // Variable: Base electrical frequency (rad/s), minimal stable observer speed, maximal voltage
    float sinTheta, cosTheta;                      // Variable: angle components - sin/cos
    float id_Rs, id_Ls, id_Ld, id_Lq;              // Variable: observed motor parameters
    float id_Zs, id_Xs;                            // Variable: observed motor parameters
    float id_Ud, id_Uq;                            // Variable: observed motor parameters
    float id_UdAmpl, id_UqAmpl;                    // Variable: observed motor parameters
    float id_IdAmpl, id_IqAmpl, id_uMag, id_iMag;  // Variable: observed motor parameters
    float id_LsBank, id_RsBank;                    // Variable: observed motor parameters
    float id_UdErr, hallAngle;                     // Variable: observed motor parameters
    uint32_t isrCntr0, isrCntr1, isrCntr2;         // Variable: ISR counter
    uint8_t spdLoopCntr;                           // Variable: counting ISR calls for speed loop prescaller
    uint8_t invEnable, flashUpdateFlag;            // Variable: Inverter enable
    float offsetCurrA, offsetCurrB, offsetCurrC;   // Variable: offset value for phase current sensing
    float offsetVoltA, offsetVoltB, offsetVoltC;   // Variable: offset value for phase current sensing
    float freq, freqRef, freqRmp, freqStep, angle; // Variable: angle generator parameters
    float halfPwmPeriod;                           // Variable: half of PWM period in timer tics
    float bldc_commCntr, bldc_commCntrRef;
    float duty, dutyRef, dutyRmp, dutyStart;
};
typedef volatile struct data_s data_t;
#define DATA_DEFAULTS      \
    {                      \
        0, 0,              \
            0, 0, 0,       \
            0, 0, 0, 0,    \
            0, 0, 0,       \
            0, 0, 0,       \
            0, 0,          \
            0, 0,          \
            0, 0, 0, 0,    \
            0, 0, 0, 0,    \
            0, 0,          \
            0, 0, 0, 0,    \
            0, 0, 0,       \
            0, 0,          \
            0, 0, 0, 0,    \
            0, 0,          \
            0, 0,          \
            0, 0,          \
            0, 0,          \
            0, 0, 0, 0,    \
            0, 0,          \
            0, 0,          \
            0, 0, 0,       \
            0,             \
            0, 0,          \
            0, 0, 0,       \
            0, 0, 0, 0, 0, \
            0,             \
            0, 0,          \
            0, 0, 0, 0,    \
    }

/*********************************************************************
 * Struct with flags
 ****************************************************************** */
struct flags_s
{
    uint8_t bldcStartup;
    uint8_t fluxDetectedOpenLoop;
    uint8_t fluxDetectedClosedLoop;
    uint8_t directionDetect;
    uint8_t sensorlessStartup;
    uint8_t pwmInputEn;
    uint8_t inductanceRatio;
    uint8_t sensorlessInjectionD; // Injection D-axis current at low speed in sensorless mode
    uint8_t overmodulation;       // Enable overmodulation: voltage vector on hexagon borders
};
typedef volatile struct flags_s flags_t;

#define FLAGS_DEFAULTS       \
    {                        \
        0, 0, 0, 0, 0, 0, 0, \
    }

/*********************************************************************
 * Struct with general config(internal variables)
 ****************************************************************** */

enum decMode_e
{
    DEC_DISABLE,   // Decoupling mode: No decoupling
    DEC_CROSS,     // Decoupling mode: cross links
    DEC_BEMF,      // Decoupling mode: bemf
    DEC_CROSS_BEMF // Decoupling mode: cross links + bemf
};
typedef volatile enum decMode_e decMode_t;

enum driveState_e
{
    STOP,         // Motor stoped
    RUN_DUTY,     // Duty cycle control mode
    RUN_CURRENT,  // Current control mode
    RUN_SPD,      // Speed control mode
    PARAM_ID_RL,  // Identify motor parameters (Rs, Ld, Lq)
    PARAM_ID_RUN, // Identify motor running dependet parameters (Hall table, magnets flux, inductance anysatropy map)
    FAULT         // Fault case
};
typedef volatile enum driveState_e driveState_t;

enum paramIdState_e
{
    ID_ENTER, // First enter flag
    ID_RS,    // Stator resistance identification
    ID_LD,    // Stator inductance identification
    ID_LQ,    // Stator inductance identification
    ID_CMPLT, // Identification completed
};
typedef volatile enum paramIdState_e paramIdState_t;

enum paramIdRunState_e
{
    ID_RUN_HALL_ENTER, // Hall table calc - enter
    ID_RUN_HALL_FWD,   // Hall table calc - forward dir
    ID_RUN_HALL_RVS,   // Hall table calc - reverse dir
    ID_RUN_HALL_CMPLT, // Hall table calc - completed
    ID_RUN_COG_FORCE,  // Cogging torque map - force angle
    ID_RUN_COG_CALC,   // Cogging torque map - calc
    ID_RUN_FLUX_OL,    // Cogging torque map - completed
    ID_RUN_FLUX_CL,    // Cogging torque map - completed
    ID_RUN_CMPLT,      // Cogging torque map - completed
};
typedef volatile enum paramIdRunState_e paramIdRunState_t;

enum focMode_e
{
    DC,   // Brushed DC-motor control
    BLDC, // 6-step trapezoidal commutation
    FOC,  // Field-oriented control
};
typedef volatile enum focMode_e focMode_t;

enum sensorType_e
{
    MANUAL,     // Angle source: generated by user
    HALL,       // Angle source: Hall sensors
    SENSORLESS, // Angle source: flux observer
    HYBRYD,     // Angle source: low speed - hall, high - flux observer
};
typedef volatile enum sensorType_e sensorType_t;

struct config_s
{
    focMode_t mode;                     // FOC or BLDC mode
    sensorType_t sensorType;            // Type of rotor position sensor
    uint8_t sim;                        // 0 - control real motor, 1 - control motor model
    decMode_t decMode;                  // Axis decoupling mode
    float currentMaxPos, currentMaxNeg; // Current limits (POS: ESC -> MOTOR) (NEG: ESC -> SUPPLY)
    float speedMax;                     // Motor maximal electrical speed rad/s
    float pwmFreq, smpFreq, tS;         // Inverter PWM frequency. In most cases samplingFreq = pwmFreq, tS - inv value (1/sampFreq)
    float adcFullScaleCurrent;          // (Vadc/2) / R_CurrSense / OP_AMP_GAIN
    float adcFullScaleVoltage;          // Vadc * K_VOLT_DIV
    float deadTime, Rds_on;             // deadtime and Mosfet Rds(on). nS, Ohm
    float Rs, Ld, Lq, Kv, pp;           // Motor phase resistance(Ohm), inductance(H) and pole pairs.
    uint32_t adcPostScaler;             // Determines how many PWM periods used for 1 ADC conversion. It configured in Foc_Init()
};
typedef volatile struct config_s config_t;
#define CONFIG_DEFAULTS    \
    {                      \
        0, 0,              \
            0, 0,          \
            0, 0,          \
            0,             \
            0, 0, 0,       \
            0,             \
            0,             \
            0, 0,          \
            0, 0, 0, 0, 0, \
            0,             \
    }
/*********************************************************************
 * General control struct
 ****************************************************************** */
struct foc_s
{
    driveState_t driveState;           // drive state enum for FOC mode
    paramIdState_t paramIdState;       // parameters identification state
    paramIdRunState_t paramIdRunState; // parameters identification state
    flags_t flags;                     // Flags
    data_t data;                       // Internal variables
    svgen_t svgen;                     // space-vector generator data
    pidReg_t pi_id;                    // D-axis current controller data
    pidReg_t pi_iq;                    // Q-axis current controller data
    pidReg_t pi_spd;                   // Speed controller data
    smopos_t smo;                      // Sliding-mode position observer data
    fluxObs_t flux;                    // Flux observer data
    voltCalc_t volt;                   // Phase voltages calculator data
    config_t config;                   // FOC configuration
    prot_t prot;                       // Protection data
    cmtn_t cmtn;                       // 6-step trapezoidal control using BEMF integration
    pll_t pll;                         // PLL angle and speed estimation
    mtpa_t mtpa;                       // MTPA algorithm
    antiCogg_t cgtc;                   // Cogging torque compensation
    filterData_t lpf_Iavg;             // Low-pass filter for total current calc
    filterData_t lpf_id;               // Low-pass filter for id current
    filterData_t lpf_iq;               // Low-pass filter for iq current
    filterData_t lpf_vdc;              // Low-pass filter for DC-bus voltage
    filterData_t lpf_Pem;              // Low-pass filter for motor power
    filterData_t lpf_NTC;              // Low-pass filter for board temperature
    filterData_t lpf_offsetIa;         // Low-pass filter for phase A current offset
    filterData_t lpf_offsetIb;         // Low-pass filter for phase B current offset
    filterData_t lpf_offsetIc;         // Low-pass filter for phase B current offset
    filterData_t lpf_offsetVa;         // Low-pass filter for phase A current offset
    filterData_t lpf_offsetVb;         // Low-pass filter for phase B current offset
    filterData_t lpf_offsetVc;         // Low-pass filter for phase B current offset
    calc_t calc;                       // Struct with methods of FOC class (calc functions)
};
typedef volatile struct foc_s foc_t;
#define FOC_DEFAULTS             \
    {                            \
        0,                       \
            0,                   \
            0,                   \
            FLAGS_DEFAULTS,      \
            DATA_DEFAULTS,       \
            SVGEN_DEFAULTS,      \
            PIDREG_DEFAULTS,     \
            PIDREG_DEFAULTS,     \
            PIDREG_DEFAULTS,     \
            SMOPOS_DEFAULTS,     \
            FLUX_OBS_DEFAULTS,   \
            VOLTCALC_DEFAULTS,   \
            CONFIG_DEFAULTS,     \
            PROTECTION_DEFAULTS, \
            CMTN_DEFAULTS,       \
            PLL_DEFAULTS,        \
            MTPA_DEFAULTS,       \
            ANTICOGG_DEFAULTS,   \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            FILTER_DEFAULTS,     \
            CALC_DEFAULTS,       \
    }

#endif