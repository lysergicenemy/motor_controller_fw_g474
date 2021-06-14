/**
 ***********************************************************************
 * File name: foc.h
 * Discription: Implementation math library for BLDC/PMSM
 *  field-oriented control.
 * Cast exmpl: bldc1.pi_id.Ref = 1.0f;
 *             bldc1.calc.pi_reg(&bldc1.pi_id); 
 *  */

#ifndef FOC_H
#define FOC_H

#define FOC_TS 1.0f / 32000.0f // Calc frequency

#define SQRT3_DIV2 0.8660254f
#define SAT(A, Pos, Neg) (((A) > (Pos)) ? (Pos) : (((A) < (Neg)) ? (Neg) : (A)))

/*********************************************************************
 * Space-vector PWM generator.
 * Input: usa, usb - voltages in alpha/beta reference frame
 * Output: Ta, Tb, Tc - switching functions for half-bridges
 ****************************************************************** */
struct svgen_s
{
    float usa, usb;    // Input: reference voltages alfa-beta ref.frame
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
        0, 0, 0, 0,     \
            0, 0, 0, 0, \
    }

static inline void Svgen_calc(svgen_t *p)
{
    p->tmp1 = p->usb;
    p->tmp2 = p->usb * 0.5f + p->usa * SQRT3_DIV2;
    p->tmp3 = p->tmp2 - p->tmp1;

    p->VecSector = 3;
    p->VecSector = (p->tmp2 > 0) ? (p->VecSector - 1) : p->VecSector;
    p->VecSector = (p->tmp3 > 0) ? (p->VecSector - 1) : p->VecSector;
    p->VecSector = (p->tmp1 < 0) ? (7 - p->VecSector) : p->VecSector;

    if (p->VecSector == 1 || p->VecSector == 4)
    {
        p->Ta = p->tmp2;
        p->Tb = p->tmp1 - p->tmp3;
        p->Tc = -p->tmp2;
    }

    else if (p->VecSector == 2 || p->VecSector == 5)
    {
        p->Ta = p->tmp3 + p->tmp2;
        p->Tb = p->tmp1;
        p->Tc = -p->tmp1;
    }

    else
    {
        p->Ta = p->tmp3;
        p->Tb = -p->tmp3;
        p->Tc = -(p->tmp1 + p->tmp2);
    }
}

/*********************************************************************
 * Digital PI controller (Antiwindup: back calculation)
 * Input: Ref, Fdb - reference and feadback
 * Output: Out - output from controller
 ****************************************************************** */
struct pidReg_s
{
    float Ref;       // Input: Reference input
    float Fdb;       // Input: Feedback input
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
        0, 0, 0, 0,     \
            0, 0, 0, 0, \
            0, 0, 0, 0, \
            0, 0, 0, 0, \
    }

static inline void PI_calc(pidReg_t *p)
{
    // p->Err = p->Ref - p->Fdb;                         /* Compute the error */
    // p->Up = p->Kp * p->Err;                           /* Compute the proportional output */
    // UTILS_NAN_ZERO(p->Ui);
    // p->Ui += p->Ki * p->Err;                          /* Compute the integral output */
    // p->Ui = SAT(p->Ui, p->OutMax, p->OutMin);         /* Saturate the integral output */
    // p->OutPreSat = p->Up + p->Ui;                     /* Compute the pre-saturated output */
    // p->Out = SAT(p->OutPreSat, p->OutMax, p->OutMin); /* Saturate the output */

    p->Err = p->Ref - p->Fdb; /* Compute the error */
    p->Up = p->Kp * p->Err;   /* Compute the proportional output */
    p->Ui += p->Ki * p->Up + p->Kc * p->SatErr; /* Compute the integral output */
    UTILS_NAN_ZERO(p->Ui);
    p->OutPreSat = p->Up + p->Ui;                     /* Compute the pre-saturated output */
    p->Out = SAT(p->OutPreSat, p->OutMax, p->OutMin); /* Saturate the output */
    p->SatErr = p->Out - p->OutPreSat;                /* Compute the saturate difference */
}
/*********************************************************************
 * Phase voltages calculation
 * Input: 
 * Output: 
 ****************************************************************** */
struct voltCalc_s
{
    float DcBusVolt;        // Input: DC-bus voltage
    float MfuncV1;          // Input: Modulation voltage phase A (pu)
    float MfuncV2;          // Input: Modulation voltage phase B (pu)
    float MfuncV3;          // Input: Modulation voltage phase C (pu)
    uint8_t OutOfPhase;     // Parameter: Out of Phase adjustment (0 or 1) (Q0) - independently with global Q
    float VphaseA;          // Output: Phase voltage phase A
    float VphaseB;          // Output: Phase voltage phase B
    float VphaseC;          // Output: Phase voltage phase C
    float usa;              // Output: Stationary alpha-axis phase voltage
    float usb;              // Output: Stationary beta-axis phase voltage
    float usd, usq;         // Output: DQ-axis phase voltages
    float vMagMax, dutyMax; // Output: DQ-axis phase voltages
    float dutyQ, dutyD;     // Output: DQ-axis duty cycles
    float temp;             // Variable: temp variable
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
            0, 0,         \
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

    if (p->OutOfPhase != 0)
    {
        p->VphaseA = -p->VphaseA;
        p->VphaseB = -p->VphaseB;
    }
    /* Voltage transformation (a,b,c)  ->  (Alpha,Beta)	*/
    p->usa = p->VphaseA;
    p->usb = 0.577350269f * p->VphaseA + 1.154700538f * p->VphaseB;
}

/*********************************************************************
 * Flux observer
 * Input: alfa-beta currents and voltages
 * Parameters: fluxLeakage, gamma
 * Output: phase
 ****************************************************************** */
struct fluxObs_s
{
    float i_alpha;
    float i_beta;
    float v_alpha;
    float v_beta;
    float fluxLeakage;
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
    float lambda_2 = SQ(p->fluxLeakage);

    p->err = lambda_2 - (SQ(p->x1 - L_ia) + SQ(p->x2 - L_ib));
    float x1_dot = -R_ia + p->v_alpha + p->gamma * (p->x1 - L_ia) * p->err;
    float x2_dot = -R_ib + p->v_beta + p->gamma * (p->x2 - L_ib) * p->err;
    p->x1 += x1_dot * p->dt;
    p->x2 += x2_dot * p->dt;

    UTILS_NAN_ZERO(p->x1);
    UTILS_NAN_ZERO(p->x2);

    /* 1: Original */
    p->phase = utils_fast_atan2(p->x2 - L_ib, p->x1 - L_ia);
    /* 2: Like VESC  */
    //p->phase = utils_fast_atan2(p->x2_prev + p->x2, p->x1_prev + p->x1);
    //p->x1_prev = p->x1;
    //p->x1_prev = p->x2;
}
/*********************************************************************
 * Sliding mode rotor position observer
 * with zero phase delay filter for estimated BEMF signal:
 * Current Error -> SMC -> LPF(Tf) -> HPF(Tf)
 * Tf dynamically adjustment depends on field frequency for provide filter f_cut = f_signal
 * Input: Valpha, Vbeta, Ialpha, Ibeta - phase voltages and currents in A-B reference frame
 * Output: Theta - observed angle
 ****************************************************************** */
struct smopos_s
{
    float Valpha;           // Input: Stationary alfa-axis stator voltage
    float Ealpha;           // Variable: Stationary alfa-axis back EMF
    float Zalpha;           // Output: Stationary alfa-axis sliding control
    float Gsmopos;          // Parameter: Motor dependent control gain
    float EstIalpha;        // Variable: Estimated stationary alfa-axis stator current
    float Fsmopos;          // Parameter: Motor dependent plant matrix
    float Vbeta;            // Input: Stationary beta-axis stator voltage
    float Ebeta;            // Variable: Stationary beta-axis back EMF
    float Zbeta;            // Output: Stationary beta-axis sliding control
    float EstIbeta;         // Variable: Estimated stationary beta-axis stator current
    float Ialpha;           // Input: Stationary alfa-axis stator current
    float IalphaError;      // Variable: Stationary alfa-axis current error
    float Kslide;           // Parameter: Sliding control gain
    float Ibeta;            // Input: Stationary beta-axis stator current
    float IbetaError;       // Variable: Stationary beta-axis current error
    float Kslf;             // Parameter: Sliding control filter gain
    float Theta;            // Output: Compensated rotor angle
    float E0;               // Parameter: 0.5
    float freq;             // Input: Input angular frequency
    float tmp_Tf;           // Variable: Time constant for given frequency
    float tmp_Ealpha;       // Variable: Time constant for given frequency
    float tmp_Ebeta;        // Variable: Time constant for given frequency
    float tmp_Theta;        // Variable: Time constant for given frequency
    filterData_t lpf_alpha; // Data struct: bemf Alpha low-pass filter data
    filterData_t hpf_alpha; // Data struct: bemf Alpha high-pass filter data
    filterData_t lpf_beta;  // Data struct: bemf Beta low-pass filter data
    filterData_t hpf_beta;  // Data struct: bemf Beta high-pass filter data
};
typedef volatile struct smopos_s smopos_t;
#define SMOPOS_DEFAULTS                   \
    {                                     \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  \
            0, 0, 0, 0, 0, 0, 0.5f, 0, 0, \
            0, 0, 0,                      \
            FILTER_DEFAULTS,              \
            FILTER_DEFAULTS,              \
            FILTER_DEFAULTS,              \
            FILTER_DEFAULTS               \
    }

static inline void Smopos_calc(smopos_t *p)
{
    /*	Sliding mode current observer	*/
    p->EstIalpha = p->Fsmopos * p->EstIalpha + p->Gsmopos * (p->Valpha - p->Ealpha - p->Zalpha);
    p->EstIbeta = p->Fsmopos * p->EstIbeta + p->Gsmopos * (p->Vbeta - p->Ebeta - p->Zbeta);
    /*	Current errors	*/
    p->IalphaError = p->EstIalpha - p->Ialpha;
    p->IbetaError = p->EstIbeta - p->Ibeta;
    /*  Sliding control calculator	*/
    /* p->Zalpha=p->IalphaError*p->Kslide/p->E0) where E0=0.5 here*/
    p->Zalpha = SAT(p->IalphaError, p->E0, -p->E0) * (p->Kslide * 2);
    p->Zbeta = SAT(p->IbetaError, p->E0, -p->E0) * (p->Kslide * 2);
    /*	Sliding control filter -> back EMF calculator	*/
    p->tmp_Tf = 1.0f / (p->freq); // recalculate filter time constant (continuous time)
    /*------------- calc BEMF alpha ----------------------------*/
    p->lpf_alpha.in = p->Zalpha;
    p->lpf_alpha.T = FOC_TS / (p->tmp_Tf + FOC_TS); // time constant in descrete time dominian for LPF
    LPF_calc(&p->lpf_alpha);
    p->hpf_alpha.T = p->tmp_Tf / (p->tmp_Tf + FOC_TS); // time constant in descrete time dominian for HPF
    p->hpf_alpha.in = p->lpf_alpha.out + 1e-20f;       // 1e-20f for avoid QNAN
    HPF_calc(&p->hpf_alpha);
    p->tmp_Ealpha = p->hpf_alpha.out * 2.f; // final value
    // /*------------- calc BEMF beta ----------------------------*/
    p->lpf_beta.in = p->Zbeta;
    p->lpf_beta.T = p->lpf_alpha.T; // time constant in descrete time dominian for LPF
    LPF_calc(&p->lpf_beta);
    p->hpf_beta.T = p->hpf_alpha.T;            // time constant in descrete time dominian for HPF
    p->hpf_beta.in = p->lpf_beta.out + 1e-20f; // 1e-20f for avoid QNAN
    HPF_calc(&p->hpf_beta);
    p->tmp_Ebeta = p->hpf_beta.out * 2.f; // final value (1.41*1.41=2)

    p->Ealpha = p->Ealpha + (p->Kslf * (p->Zalpha - p->Ealpha));
    p->Ebeta = p->Ebeta + (p->Kslf * (p->Zbeta - p->Ebeta));
    /*	Rotor angle calculator -> Theta = atan(-Ealpha,Ebeta)	*/
    //p->Theta = utils_fast_atan2(-p->Ebeta, -p->Ealpha);
    //p->tmp_Theta = utils_fast_atan2(-p->tmp_Ebeta, -p->tmp_Ealpha);
    //p->tmp_Theta = utils_fast_atan2(-p->Ealpha, p->Ebeta);
    p->Theta = utils_fast_atan2(-p->tmp_Ealpha, p->tmp_Ebeta);
}

/*********************************************************************
 * Enum with decoupling modes
 ****************************************************************** */
enum decMode_e
{
    DEC_DISABLE,   // Decoupling mode: No decoupling
    DEC_CROSS,     // Decoupling mode: cross links
    DEC_BEMF,      // Decoupling mode: bemf
    DEC_CROSS_BEMF // Decoupling mode: cross links + bemf
};
typedef volatile enum decMode_e decMode_t;

/*********************************************************************
 * Enum with fault codes
 ****************************************************************** */
enum faultCode_e
{
    FLT_NONE,  // Fault code: No fault
    FLT_OCP_U, // Fault code: Overcurrent phase A
    FLT_OCP_V, // Fault code: Overcurrent phase B
    FLT_OVP,   // Fault code: Overvoltage DC-Link
    FLT_OVT,   // Fault code: PCB Overtemperature
    FLT_UVLO   // Fault code: Undervoltage DC-Link
};
typedef volatile enum faultCode_e faultCode_t;

/*********************************************************************
 * Struct with Protection data
 ****************************************************************** */
struct prot_s
{
    faultCode_t fltCode;
    float ocpThld, ovpThld, uvloThld, pcbTempThld; // Protection thresholds, OCP for phase currents (A), DC_Link voltage protect OVP/UVLO (V)
    float currPhU, currPhV, udc, tempPcb;
    uint8_t protFlag;
    uint8_t prechargeFlag;
    uint8_t enable;
};
typedef volatile struct prot_s prot_t;
#define PROTECTION_DEFAULTS \
    {                       \
        0,                  \
            0, 0, 0, 0,     \
            0, 0, 0, 0,     \
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
    float AngleRaw, dt;       // Input: Raw electrical angle(rad), sample time(s)
    float Err;                // Internal: Electrical angle error (delta Theta)
    float SpeedPll, AnglePll; // Output: Filtered Speed(rad/s), Angle(rad)
    float Kp, Ki;             // Parameter: PLL gains
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
}

/* **********************************************************************************
Description: 
Commutation trigger logic in sensorless operation.
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
    obj->Neutral = (obj->As + obj->Bs + obj->Cs) * 0.333333f;
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
 * Ramp func
 ****************************************************************** */
static inline void ramp_calc(volatile float *in, volatile float *out, volatile float rampRate, volatile float Ts)
{
    float delta = Ts * rampRate;
    if (*out < (*in - delta)) // add some hysteresys
    {
        *out += delta;
    }
    if (*out > (*in + delta))
    {
        *out -= delta;
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
    void (*cmtn)(cmtn_t *obj);
    void (*flux)(fluxObs_t *p);
    void (*pll)(pll_t *p);
};
typedef volatile struct calc_s calc_t;
#define CALC_DEFAULTS     \
    {                     \
        PI_calc,          \
            Svgen_calc,   \
            Smopos_calc,  \
            Volt_calc,    \
            Prot_calc,    \
            CMTN_run,     \
            FluxObs_calc, \
            Pll_calc,     \
    }
/*********************************************************************
 * Enum for drive states and angle source
 ****************************************************************** */
enum driveState_e
{
    STOP,               // Motor stoped
    RUN_OPENLOOP_VHZ,   // Control frequency and magnitude of rotating voltage vector
    ALIGN,              // Motor stoped
    RUN_OPENLOOP_IHZ,   // Control frequency and magnitude of rotating current vector
    RUN_CLOSEDLOOP_DQ,  // Control D/Q-axis currents in closed loop mode(angle for rotating
                        // reference frame arrived from sensor/observer)
    RUN_CLOSEDLOOP_SPD, // Control rotor speed in closed loop mode
    PARAM_ID,           // Identify motor parameters
    FAULT               // Fault case
};
typedef volatile enum driveState_e driveState_t;

enum driveStateBLDC_e
{
    STOP_BLDC,           // Motor stoped
    STARTUP_BLDC,        // Rotor align
    RUN_OPENLOOP_BLDC,   // Rump ON
    RUN_CLOSEDLOOP_BLDC, // Normal operation
    RUN_SPD_BLDC,        // Normal operation speed control mode
    FAULT_BLDC           // Fault case
};
typedef volatile enum driveStateBLDC_e driveStateBLDC_t;

enum paramIdState_e
{
    Enter, // First enter flag
    Rs,    // Stator resistance identification
    Ld,    // Stator inductance identification
    Lq,    // Stator inductance identification
    Cmplt, // Identification completed
};
typedef volatile enum paramIdState_e paramIdState_t;
/*********************************************************************
 * Struct with general data(internal variables)
 ****************************************************************** */
struct data_s
{
    float udRef, uqRef;                            // Input: reference D/Q-axis voltages (V)
    float idRef, iqRef, iqRmp;                     // Input: reference D/Q-axis currents, id ramp rate (A/s)
    float speedRef, thetaRef, spdRmp, speedRpm;    // Input: reference speed, angle, speed ramp, measured speed in RPM
    float iPhaseA, iPhaseB, iPhaseC;               // Input: ABC currents (A)
    float iAvg, iAvgFiltered, iStrtp;              // Input: Total current (A)
    float isa, isb, isd, isq;                      // Variable: D/Q-axis currents (A)
    float udDec, uqDec;                            // Variable: D/Q-axis voltages decoupling components
    float Te, Pe;                                  // Variable: Electrical torque (N.M) and Power (W)
    float isb_tmp;                                 // Variable: Tmp Phase B current
    float sinTheta, cosTheta;                      // Variable: angle components - sin/cos
    float id_Rs, id_Ls, id_Ld, id_Lq;              // Variable: observed motor parameters
    float id_Zs, id_Xs;                            // Variable: observed motor parameters
    float id_Ud, id_Uq;                            // Variable: observed motor parameters
    float id_UdAmpl, id_UqAmpl;                    // Variable: observed motor parameters
    float id_IdAmpl, id_IqAmpl, id_IdPr, id_IqPr;  // Variable: observed motor parameters
    float id_LsBank, id_RsBank;                    // Variable: observed motor parameters
    float id_UdErr;                                // Variable: observed motor parameters
    uint32_t isrCntr0, isrCntr1, isrCntr2;         // Variable: ISR counter
    uint8_t spdLoopCntr;                           // Variable: counting ISR calls for speed loop prescaller
    uint8_t invEnable, flashUpdateFlag;            // Variable: Inverter enable
    float offsetCurrA, offsetCurrB, offsetCurrC;   // Variable: offset value for phase current sensing
    float freq, freqRef, freqRmp, freqStep, angle; // Variable: angle generator parameters
    float halfPwmPeriod;                           // Variable: half of PWM period in timer tics
    float bldc_commCntr, bldc_commCntrRef;
    float bldc_duty, bldc_dutyRef, bldc_dutyStart;
};
typedef volatile struct data_s data_t;
#define DATA_DEFAULTS      \
    {                      \
        0, 0,              \
            0, 0, 0,       \
            0, 0, 0, 0,    \
            0, 0, 0,       \
            0, 0, 0,       \
            0, 0, 0, 0,    \
            0, 0,          \
            0, 0,          \
            0,             \
            0, 0,          \
            0, 0, 0, 0,    \
            0, 0,          \
            0, 0,          \
            0, 0,          \
            0, 0, 0, 0,    \
            0, 0,          \
            0,             \
            0, 0, 0,       \
            0,             \
            0, 0,          \
            0, 0, 0,       \
            0, 0, 0, 0, 0, \
            0,             \
            0, 0,          \
            0, 0, 0,       \
    }

/*********************************************************************
 * Struct with flags
 ****************************************************************** */
struct flags_s
{
    uint8_t driveStateSwitched;
    uint8_t bldcStartup;
};
typedef volatile struct flags_s flags_t;

#define FLAGS_DEFAULTS \
    {                  \
        0, 0,          \
    }

/*********************************************************************
 * Struct with general config(internal variables)
 ****************************************************************** */
enum focMode_e
{
    BLDC, // 6-step trapezoidal commutation
    FOC,  // Field-oriented control
};
typedef volatile enum focMode_e focMode_t;

enum sensorType_e
{
    SENSORLESS, // 6-step trapezoidal commutation
    HALL,       // Field-oriented control
};
typedef volatile enum sensorType_e sensorType_t;

struct config_s
{
    focMode_t mode;                 // Control real motor or simulate it
    sensorType_t sensorType;        // Type of rotor position sensor
    uint8_t sim;                    // 0 - control real motor, 1 - control motor model
    decMode_t decMode;              // Axis decoupling mode
    float baseCurrent, baseVoltage; // Base values in most cases = nominal values. Amps, Volts  !NOT USED
    float baseFreq;                 // Base electrical frequency. Hz
    float pwmFreq, tS;              // Inverter PWM frequency. In most cases samplingFreq = pwmFreq, tS - inv value (1/sampFreq)
    float adcFullScaleCurrent;      // (Vadc/2) / R_CurrSense / OP_AMP_GAIN
    float adcFullScaleVoltage;      // Vadc * K_VOLT_DIV
    float deadTime, Rds_on;         // deadtime and Mosfet Rds(on). nS, Ohm
    float Rs, Ld, Lq, Kv, pp;       // Motor phase resistance(Ohm), inductance(H) and pole pairs.
    uint32_t adcPostScaler;         // Determines how many PWM periods used for 1 ADC conversion. It configured in Foc_Init()
};
typedef volatile struct config_s config_t;
#define CONFIG_DEFAULTS    \
    {                      \
        0, 0,              \
            0, 0,          \
            0, 0,          \
            0,             \
            0, 0,          \
            0,             \
            0,             \
            0, 0,          \
            0, 0, 0, 0, 0, \
            0,             \
    }
/*********************************************************************
 * General control struct (must be last struct)
 ****************************************************************** */
struct foc_s
{
    driveState_t driveState;         // drive state enum for FOC mode
    driveStateBLDC_t driveStateBLDC; // drive state enum for BLDC mode
    paramIdState_t paramIdState;     // parameters identification state
    flags_t flag;                    // Flags
    data_t data;                     // Internal variables
    svgen_t svgen;                   // space-vector generator data
    pidReg_t pi_id;                  // D-axis current controller data
    pidReg_t pi_iq;                  // Q-axis current controller data
    pidReg_t pi_spd;                 // Speed controller data
    smopos_t smo;                    // Sliding-mode position observer data
    fluxObs_t flux;                  // Flux observer data
    voltCalc_t volt;                 // Phase voltages calculator data
    config_t config;                 // FOC configuration
    prot_t prot;                     // Protection data
    cmtn_t cmtn;                     // 6-step trapezoidal control using BEMF integration
    pll_t pll;                       // PLL angle and speed estimation
    filterData_t lpf_Iavg;           // Low-pass filter for total current calc
    filterData_t lpf_id;             // Low-pass filter for id current
    filterData_t lpf_iq;             // Low-pass filter for iq current
    filterData_t lpf_vdc;            // Low-pass filter for DC-bus voltage
    filterData_t lpf_Pe;             // Low-pass filter for motor power
    filterData_t lpf_NTC;            // Low-pass filter for board temperature
    filterData_t lpf_offsetIa;       // Low-pass filter for phase A current offset
    filterData_t lpf_offsetIb;       // Low-pass filter for phase B current offset
    calc_t calc;                     // Struct with methods of FOC class (calc functions)
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
static inline void Foc_Init(foc_t *p)
{
    /** Init timeBase values
     *  20000 Hz is maximal FOC execution rate for any PWM freq
     *  Example: ADC postScaler = 1, means we calc FOC loop 1 times for 2 PWM cycles
     */
    p->config.adcPostScaler = (uint32_t)(p->config.pwmFreq / 20000.1f);
    p->config.tS = (1.f / p->config.pwmFreq) * (float)(p->config.adcPostScaler + 1);
    p->data.freqStep = M_2PI * p->config.tS;
    /* Max duty cycle*/
    p->volt.dutyMax = 0.95f;
    /* Init ramps */
    p->data.freqRmp = 20.f;  // Hz/s
    p->data.iqRmp = 10.f;    // A/s
    p->data.spdRmp = 3000.f; // (rad/s)/s
    /* Init D-axis current PI controller */
    p->pi_id.Kp = 0.125f;
    p->pi_id.Ki = 0.025f;
    p->pi_id.Kc = 0.5f;
    p->pi_id.OutMin = (p->config.mode == FOC) ? -p->config.adcFullScaleVoltage : 0; // -1.f
    p->pi_id.OutMax = (p->config.mode == FOC) ? p->config.adcFullScaleVoltage : 1.f;
    /* Init Q-axis current PI controller */
    p->pi_iq.Kp = 0.125f;
    p->pi_iq.Ki = 0.025f;
    p->pi_iq.Kc = 0.5f;
    p->pi_iq.OutMin = (p->config.mode == FOC) ? -p->config.adcFullScaleVoltage : 0; // -1.f
    p->pi_iq.OutMax = (p->config.mode == FOC) ? p->config.adcFullScaleVoltage : 1.f;
    /** Automatic calc PI gains depends on motor RL parameters
     *  wcc is cutoff frequency of controller
     *  if current sampling 1 times for 1 PWM period use wcc 5%. In any cases try from 0.025 to 0.1
     */
    float wcc = (0.065f * (1.f / p->config.tS)) * M_2PI;
    p->pi_id.Kp = p->pi_iq.Kp = (p->config.Ld * wcc);
    p->pi_id.Ki = p->pi_iq.Ki = p->config.Rs * wcc * p->config.tS; // Rs * bw, bw = 1.0 / tc, Ki = 𝜔𝑐𝑅 × 𝑇𝑆
    p->pi_id.Kc = 1.f / p->pi_id.Kp;
    p->pi_iq.Kc = 1.f / p->pi_iq.Kp;
    /* Init speed PI controller */
    p->pi_spd.Kp = (p->config.mode == FOC) ? 0.025f : 0.01f;
    p->pi_spd.Ki = (p->config.mode == FOC) ? 0.001f : 0.0005f;
    p->pi_spd.Kc = 1.f / p->pi_spd.Kp;
    p->pi_spd.OutMin = (p->config.mode == FOC) ? (-p->prot.ocpThld * 0.9f) : 0.f;
    p->pi_spd.OutMax = (p->config.mode == FOC) ? (p->prot.ocpThld * 0.9f) : 0.99f;
    /* Init SMOPOS constants */
    p->smo.Fsmopos = exp((-p->config.Rs / p->config.Ld) * p->config.tS);
    p->smo.Gsmopos = (1 / p->config.Rs) * (1 - p->smo.Fsmopos);
    p->smo.Kslide = 0.5308703613f;
    /* Init Flux observer constants */
    p->flux.R = p->config.Rs * 1.5f;
    p->flux.L = p->config.Ld * 1.5f;
    p->flux.fluxLeakage = 60.0f / (SQRT3 * M_2PI * p->config.Kv * p->config.pp);
    p->flux.dt = p->config.tS;
    p->flux.gamma = (1000.f / (p->flux.fluxLeakage * p->flux.fluxLeakage)) * 0.5f; //300000.f;
    /* Init PLL */
    p->pll.dt = p->config.tS;
    p->pll.Kp = 2000.f;
    p->pll.Ki = 30000.f;
    /* saturation protection thresholds */
    p->prot.ocpThld = (p->prot.ocpThld > p->config.adcFullScaleCurrent) ? p->config.adcFullScaleCurrent : p->prot.ocpThld;
    p->prot.ovpThld = (p->prot.ovpThld > p->config.adcFullScaleVoltage) ? p->config.adcFullScaleVoltage : p->prot.ovpThld;
    /** BEMF filters init
     * Tf = 1 / (Fc * 2 * pi)
     *   */
    p->cmtn.lpf_bemfA.T = p->config.tS / ((1.f / (100.f * M_2PI)) + p->config.tS);
    p->cmtn.lpf_bemfB.T = p->cmtn.lpf_bemfA.T;
    p->cmtn.lpf_bemfC.T = p->cmtn.lpf_bemfA.T;
    p->cmtn.cmtnTh = 0.0001f;
    // BLDC mode settings
    p->data.bldc_commCntrRef = 350.f;
    p->data.bldc_dutyStart = 0.1f; // 10% of max value
    // Filters data init which execute in main ADC-ISR
    p->lpf_id.T = p->config.tS / (0.005f + p->config.tS);     // time constant in descrete time dominian for D-axis current LPF
    p->lpf_iq.T = p->config.tS / (0.005f + p->config.tS);     // time constant in descrete time dominian for Q-axis current LPF
    p->lpf_vdc.T = p->config.tS / (0.1f + p->config.tS);      // time constant in descrete time dominian for DC-Bus voltage LPF
    p->lpf_offsetIa.T = p->config.tS / (0.2f + p->config.tS); // time constant in descrete time dominian for D-axis current LPF
    p->lpf_offsetIb.T = p->config.tS / (0.2f + p->config.tS); // time constant in descrete time dominian for Q-axis current LPF
    // Filters data init which execute in 1ms SYSTICK-Callback
    p->lpf_Iavg.T = 0.001f / (0.02f + 0.001f); // time constant in descrete time dominian for total current LPF
    p->lpf_Pe.T = 0.001f / (0.02f + 0.001f);   // time constant in descrete time dominian for total current LPF
    p->lpf_NTC.T = 0.001f / (0.002f + 0.001f); // time constant in descrete time dominian for total current LPF
}

#endif