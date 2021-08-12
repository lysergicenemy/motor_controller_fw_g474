/**
 ***********************************************************************
 * @file : foc.h
 * @brief: Implementation math library for BLDC/PMSM
 *  6step/field-oriented control.
 * @example: bldc1.pi_id.Ref = 1.0f;
 *           bldc1.calc.pi_reg(&bldc1.pi_id); 
 *  */

#ifndef FOC_H
#define FOC_H

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
 * Digital PI(D) controller (Antiwindup: dynamyc integrator saturation)
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

    // p->Err = p->Ref - p->Fdb; /* Compute the error */
    // p->Up = p->Kp * p->Err;   /* Compute the proportional output */
    // p->Ui += p->Ki * p->Up + p->Kc * p->SatErr; /* Compute the integral output */
    // p->OutPreSat = p->Up + p->Ui;                     /* Compute the pre-saturated output */
    // p->Out = SAT(p->OutPreSat, p->OutMax, p->OutMin); /* Saturate the output */
    // p->SatErr = p->Out - p->OutPreSat;                /* Compute the saturate difference */
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
 * @input: DcBusVolt, MfuncV1-3
 * @output: VphaseA-C
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
 * @parameters: fluxLeakage, gamma
 * @output: phase
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
 * @brief Sliding mode rotor position observer
 * with zero phase delay filter for estimated BEMF signal:
 * Current Error -> SMC -> LPF(Tf) -> HPF(Tf)
 * Tf dynamically adjustment depends on field frequency for provide filter f_cut = f_signal
 * @input: Valpha, Vbeta, Ialpha, Ibeta - phase voltages and currents in A-B reference frame
 * @output: Theta - estimated angle
 ****************************************************************** */
struct smopos_s
{
    float Valpha;           // input: Stationary alfa-axis stator voltage
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
    float dt;               // Variable: Time constant 1/execution frequency
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
            0, 0, 0, 0,                   \
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
    p->lpf_alpha.T = p->dt / (p->tmp_Tf + p->dt); // time constant in descrete time dominian for LPF
    LPF_calc(&p->lpf_alpha);
    p->hpf_alpha.T = p->tmp_Tf / (p->tmp_Tf + p->dt); // time constant in descrete time dominian for HPF
    p->hpf_alpha.in = p->lpf_alpha.out + 1e-20f;      // 1e-20f for avoid QNAN
    HPF_calc(&p->hpf_alpha);
    p->tmp_Ealpha = p->hpf_alpha.out * 2.f; // final value
    // /*------------- calc BEMF beta ----------------------------*/
    p->lpf_beta.in = p->Zbeta;
    p->lpf_beta.T = p->lpf_alpha.T; // time constant in descrete time dominian for LPF
    LPF_calc(&p->lpf_beta);
    p->hpf_beta.T = p->hpf_alpha.T;            // time constant in descrete time dominian for HPF
    p->hpf_beta.in = p->lpf_beta.out + 1e-20f; // 1e-20f for avoid QNAN
    HPF_calc(&p->hpf_beta);
    p->tmp_Ebeta = p->hpf_beta.out * 2.f; // final value (sqrt2*sqrt2=2)

    p->Ealpha = p->Ealpha + (p->Kslf * (p->Zalpha - p->Ealpha));
    p->Ebeta = p->Ebeta + (p->Kslf * (p->Zbeta - p->Ebeta));
    /*	Rotor angle calculation -> Theta = atan(-Ealpha,Ebeta)	*/
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
    FLT_NONE,      // Fault code: No fault
    FLT_OCP_U,     // Fault code: Overcurrent phase A
    FLT_OCP_V,     // Fault code: Overcurrent phase B
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

    // float g_mtpa = p->k_mtpa / p->iqRef;
    // p->beta = acosf(g_mtpa - sqrtf(SQ(g_mtpa) + 0.5f));

    // LL_CORDIC_WriteData(CORDIC, _IQ31(p->beta * ONE_BY_PI)); // cordic input data format q31, angle range (1 : -1)
    // p->sinBeta = _IQ31toF((int32_t)LL_CORDIC_ReadData(CORDIC));
    // p->cosBeta = _IQ31toF((int32_t)LL_CORDIC_ReadData(CORDIC));

    // p->id_MTPA = p->iqRef * p->cosBeta;
    // UTILS_NAN_ZERO(p->id_MTPA);
    // p->iq_MTPA = p->iqRef * p->sinBeta;
    // UTILS_NAN_ZERO(p->iq_MTPA);

    p->id_MTPA = p->k_mtpa - sqrtf(SQ(p->k_mtpa) + (0.5f * SQ(p->iqRef)));
    p->iq_MTPA = sqrtf(SQ(p->iqRef) - SQ(p->id_MTPA));
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

/* https://www.ti.com/lit/pdf/spracf3 */
static inline void AntiCogg_calc(antiCogg_t *p)
{
    p->tabNmbr = (uint8_t)((p->angle + MF_PI) * p->scale);
    p->uCogg = (p->en == 1) ? (p->uRef * p->coggingMap[p->tabNmbr]) * p->gain : 0;
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

/** Ramp function (trapezoidal profile)
 * Input: in - referance value (rad/s, hz, rpm)
 * Output: out - pointer for actual value (rad/s, hz, rpm)
 * Param: rampRate - maximal acceleration (rad/s^2, hz^2, rpm^2)
 *        Ts - sample time
 */
static inline void ramp_calc(volatile float *out, float in, float rampRate, float Ts)
{
    float delta = Ts * rampRate;
    if (*out < in)
    {
        *out += delta;
    }
    if (*out > in)
    {
        *out -= delta;
    }
}
/** Ramp function (s-curve profile)
 * Input: in - referance value (rad/s, hz, rpm)
 * Output: out - pointer for actual value (rad/s, hz, rpm)
 * Param: rampRate - maximal acceleration in linear region (rad/s^2, hz^2, rpm^2)
 *        Ts - sample time
 */
static inline void ramp_s_calc(volatile float *out, float in, float rampRate, float Ts)
{
    // if acceleration/decceleration phases is 25% actual rampRate mast be multiplyed by 1.5
    rampRate *= 1.5f;
    static float acc, inPr, diff;

    if (in != inPr)
    {
        diff = fabsf(in - inPr);
        acc = 0.f;
    }
    float time = (diff * 0.25f) / rampRate;
    float jerk = (rampRate / time) * 0.5f;
    float freqDiff = fabsf(*out - in);

    // ramp-up
    if (*out < in)
    {
        // phase 1: increase acceleration
        if (freqDiff > (diff * 0.75f))
        {
            ramp_calc(&acc, rampRate, jerk, Ts);
        }
        // phase 3: decrease acceleration
        else if (freqDiff < (diff * 0.25f))
        {
            ramp_calc(&acc, 0.f, jerk, Ts);
        }
        // phase 2: constant acceleration
        else
        {
            acc = rampRate;
        }
    }
    // ramp-down
    else if (*out > in)
    {
        // phase 1: increase acceleration
        if (freqDiff > (diff * 0.75f))
        {
            ramp_calc(&acc, rampRate, jerk, Ts);
        }
        // phase 3: decrease acceleration
        else if (freqDiff < (diff * 0.25f))
        {
            ramp_calc(&acc, 0.f, jerk, Ts);
        }
        // phase 2: constant acceleration
        else
        {
            acc = rampRate;
        }
    }
    ramp_calc(&*out, in, fabsf(acc), Ts);
    inPr = in;
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
    void (*mtpa)(mtpa_t *p);
    void (*cgtc)(antiCogg_t *p);
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
 * Enum for drive states and angle source
 ****************************************************************** */
enum driveState_e
{
    STOP,               // Motor stoped
    RUN_OPENLOOP_VHZ,   // Control frequency and magnitude of rotating voltage vector
    PARAM_ID_RUN,       // Identify motor running dependet parameters (Hall table, magnets flux, inductance anysatropy map)
    RUN_OPENLOOP_IHZ,   // Control frequency and magnitude of rotating current vector
    RUN_CLOSEDLOOP_DQ,  // Control D/Q-axis currents in closed loop mode
    RUN_CLOSEDLOOP_SPD, // Control motor speed in closed loop mode
    PARAM_ID_RL,        // Identify motor parameters (Rs, Ld, Lq)
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
    float wBase, vMax;                             // Variable: Base electrical frequency (rad/s), maximal voltage
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
            0, 0, 0,       \
    }

/*********************************************************************
 * Struct with flags
 ****************************************************************** */
struct flags_s
{
    uint8_t driveStateSwitched;
    uint8_t bldcStartup;
    uint8_t fluxDetectedOpenLoop;
    uint8_t fluxDetectedClosedLoop;
    uint8_t directionDetect;
    uint8_t sensorlessStartup;
    uint8_t pwmInputEn;
    uint8_t inductanceRatio;
};
typedef volatile struct flags_s flags_t;

#define FLAGS_DEFAULTS       \
    {                        \
        0, 0, 0, 0, 0, 0, 0, \
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
    SENSORLESS, // Angle souece: flux observer
    HALL,       // Angle souece: Hall sensors
    HYBRYD,     // Angle souece: low speed - hall, high - flux observer
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
    driveStateBLDC_t driveStateBLDC;   // drive state enum for BLDC mode
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

/** Angle generator
 */
static inline void angle_gen(volatile float *angle, float freq, float Ts)
{
    float delta = M_2PI * Ts;
    *angle += delta * freq;
    if (*angle < -MF_PI)
        *angle = *angle + M_2PI;
    else if (*angle > MF_PI)
        *angle = *angle - M_2PI;
}

/** Open loop startup in sensorless mode
 * add timeout
 */
static inline void Foc_update_openLoopStart(foc_t *p)
{
    static volatile int32_t ol_timeout, ol_dir;
    volatile float iqRefShdw;

    // detect low speed
    if (fabsf(p->pll.SpeedPll) < (p->config.speedMax * 0.05f))
    {
        // detect run comand
        if (fabsf(p->data.speedRef) > 0.f || fabsf(p->data.iqRef) > 0.f)
        {
            p->flags.sensorlessStartup = 0;
        }
    }
    // openLoop startUp
    if (p->flags.sensorlessStartup == 0)
    {
        ol_dir = (p->driveState == RUN_CLOSEDLOOP_SPD) ? SIGN(p->data.speedRef) : ol_dir; // direction if speed control mode
        ol_dir = (p->driveState == RUN_CLOSEDLOOP_DQ) ? SIGN(p->data.iqRef) : ol_dir;     // direction if current control mode
        iqRefShdw = p->config.currentMaxPos * 0.5f * (float)ol_dir;                       // startUp current 50% of max
        p->data.freqRef = (p->config.speedMax * 0.1f) * ONE_BY_2PI * (float)ol_dir;       // startUp speed 10% of max speed
        p->data.freqRmp = fabsf(p->data.freqRef * 5.f);                                   // 0.2s startUp time
        ol_timeout++;
        // Align process
        if (ol_timeout < (uint32_t)(1.f * p->config.smpFreq))
        {
            p->data.angle = 0.f;
            ramp_calc(&p->pi_iq.Ref, iqRefShdw, fabsf(iqRefShdw), p->config.tS);
        }
        // Ramp up speed
        else
        {
            ramp_calc(&p->data.freq, p->data.freqRef, p->data.freqRmp, p->config.tS);
            // Angle generator
            p->data.angle += p->data.freqStep * p->data.freq;
            if (p->data.angle < -MF_PI)
                p->data.angle = p->data.angle + M_2PI;
            else if (p->data.angle > MF_PI)
                p->data.angle = p->data.angle - M_2PI;
        }
        // detect succesfull startup
        if (fabsf(p->pll.SpeedPll) > (fabsf(p->data.freqRef) * M_2PI * 0.9f))
        {
            //ramp_calc(&p->data.idRef, 0.f, 2.f, p->config.tS);
            p->flags.sensorlessStartup = 1;
            p->pi_iq.Ref = (p->driveState == RUN_CLOSEDLOOP_DQ) ? p->data.iqRef : 0.f;
            p->pi_spd.Ui = (p->driveState == RUN_CLOSEDLOOP_SPD) ? iqRefShdw : 0.f;
            p->data.freq = 0.f;
            p->data.freqRef = 0.f;
            ol_timeout = 0;
        }
        // detect no succesfull startup
        if (ol_timeout > (uint32_t)(2.f * p->config.smpFreq) && fabsf(p->pll.SpeedPll) < (fabsf(p->data.freqRef) * M_2PI * 0.9f))
        {
            // restart process
            p->data.freq = 0.f;
            p->pi_iq.Ref = 0.f;
            ol_timeout = 0;
        }
    }
}

/** Run D/Q currents controller
 * PARK -> DECOUPLING -> PI -> DQ_Limiter -> IPARK
 * Input(must be updated before call):
 * idRef, iqRef, isa, isb, sinTheta, cosTheta
 * Output:
 * usa, usb - referance alpha/beta components of phase voltage
 */
static inline void Foc_update_cc(foc_t *p)
{
    // PARK transform for phase currents
    p->data.isd = p->data.isa * p->data.cosTheta + p->data.isb * p->data.sinTheta;
    p->data.isq = -p->data.isa * p->data.sinTheta + p->data.isb * p->data.cosTheta;
    // Filtered data
    p->lpf_id.in = p->data.isd;
    p->lpf_iq.in = p->data.isq;
    p->lpf_vdc.in = p->volt.DcBusVolt;
    LPF_calc(&p->lpf_id);
    LPF_calc(&p->lpf_iq);
    LPF_calc(&p->lpf_vdc);
    p->volt.DcBusVolt = p->lpf_vdc.out;
    // calc PI current controllers for D/Q axis
    // calc MTPA
    if (p->mtpa.en == 1)
    {
        p->mtpa.iqRef = p->pi_iq.Ref;
        p->calc.mtpa(&p->mtpa);
    }

    // calc Cogging torque compensation
    if (p->cgtc.en == 1)
    {
        p->cgtc.angle = p->data.angle;
        p->cgtc.uRef = p->pi_iq.Out;
        p->calc.cgtc(&p->cgtc);
    }
    // calc decoupling
    switch (p->config.decMode)
    {
    case DEC_DISABLE:
        p->data.udDec = 0.f;
        p->data.uqDec = 0.f;
        break;
    case DEC_CROSS:
        p->data.udDec = p->data.isq * p->pll.SpeedPll * p->config.Lq;
        p->data.uqDec = p->data.isd * p->pll.SpeedPll * p->config.Ld;
        break;
    case DEC_BEMF:
        p->data.udDec = 0.f;
        p->data.uqDec = p->pll.SpeedPll * p->flux.fluxLeakage;
        break;
    case DEC_CROSS_BEMF:
        p->data.udDec = p->data.isq * p->pll.SpeedPll * p->config.Lq;
        p->data.uqDec = p->pll.SpeedPll * (p->data.isd * p->config.Ld + p->flux.fluxLeakage);
        break;
    default:
        break;
    }
    // calc PI
    p->pi_iq.Ref = (p->mtpa.en != 0) ? p->mtpa.iq_MTPA : p->pi_iq.Ref;
    p->pi_id.Ref = (p->mtpa.en != 0) ? p->mtpa.id_MTPA : p->pi_id.Ref;
    p->pi_id.Fdb = p->data.isd;
    p->pi_iq.Fdb = p->data.isq;
    p->pi_id.Fdfwd = -p->data.udDec;
    p->pi_iq.Fdfwd = p->data.uqDec + (p->cgtc.uCogg * p->cgtc.uRef);
    p->pi_id.OutMax = p->volt.magMaxD;
    p->pi_id.OutMin = -p->pi_id.OutMax;
    p->pi_iq.OutMax = p->volt.magMaxQ;
    p->pi_iq.OutMin = -p->pi_iq.OutMax;
    p->calc.pi_reg(&p->pi_id);
    p->calc.pi_reg(&p->pi_iq);

    p->volt.usd = p->pi_id.Out;
    p->volt.usq = p->pi_iq.Out;
    p->volt.magMaxD = p->volt.dutyMax * ONE_BY_SQRT3 * p->volt.DcBusVolt;
    p->volt.magMaxQ = sqrtf(SQ(p->volt.magMaxD) - SQ(p->volt.usd));
    // Saturation
    utils_saturate_vector_2d((float *)&p->volt.usd, (float *)&p->volt.usq, p->volt.magMaxD);
    // calc IPARK transform (IPARK output -> SVGEN input)
    p->svgen.usa = p->volt.usd * p->data.cosTheta - p->volt.usq * p->data.sinTheta;
    p->svgen.usb = p->volt.usq * p->data.cosTheta + p->volt.usd * p->data.sinTheta;
}

/** Run angle and speed estimation/calculation
 * Hall, flux observer, pll
 * Input(must be updated before call):
 * usa, usb, isa, isb
 * Output:
 * angle - angle of rotor flux
 * SpeedPll, speedRpm - electrical(rad/s) and mechanical(rpm) rotor speed
 */
static inline void Foc_update_angle_speed(foc_t *p)
{
    if (p->config.sensorType == HALL)
    {
        // calc speed PLL
        p->pll.AngleRaw = p->data.hallAngle;
        p->calc.pll(&p->pll);
        p->data.angle = (fabsf(p->pll.SpeedPll) > 50.f) ? p->pll.AnglePll : p->data.hallAngle;
    }
    else if (p->config.sensorType == SENSORLESS)
    {
        //Foc_update_openLoopStart(p);
        p->flags.sensorlessStartup = 1;
        // calc flux observer
        p->flux.i_alpha = p->data.isa;
        p->flux.i_beta = p->data.isb;
        p->flux.v_alpha = p->volt.usa;
        p->flux.v_beta = p->volt.usb;
        p->calc.flux(&p->flux);
        // calc speed PLL
        p->pll.AngleRaw = p->flux.phase;
        p->data.angle = (p->flags.sensorlessStartup == 1) ? p->flux.phase : p->data.angle;
        p->calc.pll(&p->pll);
    }
    else if (p->config.sensorType == HYBRYD)
    {
        // calc flux observer
        p->flux.i_alpha = p->data.isa;
        p->flux.i_beta = p->data.isb;
        p->flux.v_alpha = p->volt.usa;
        p->flux.v_beta = p->volt.usb;
        p->calc.flux(&p->flux);
        // calc speed PLL
        volatile static uint8_t hybryd_state;
        volatile float speedAbs = fabsf(p->pll.SpeedPll);
        hybryd_state = (speedAbs > 310.f) ? 1 : hybryd_state;
        hybryd_state = (speedAbs < 290.f) ? 0 : hybryd_state;
        if (hybryd_state == 0)
        {
            p->pll.AngleRaw = p->data.hallAngle;
            p->calc.pll(&p->pll);
            p->data.angle = (speedAbs > 50.f) ? p->pll.AnglePll : p->data.hallAngle;
        }
        else
        {
            p->pll.AngleRaw = p->flux.phase;
            p->data.angle = p->flux.phase;
            p->calc.pll(&p->pll);
        }
    }
    p->data.speedRpm = (p->pll.SpeedPll * RADS2RPM) / p->config.pp;
}

static inline void Foc_Init(foc_t *p)
{
    /** Init timeBase values
     *  20000 Hz is maximal FOC execution rate for any PWM freq (TESTED: 170Mhz CORTEX-M4F)
     *  If FOC ISR placed on CCMRAM, disable virtual motor and optimization level >= O2 it can be increased up to 40000 Hz
     *  Example: ADC postScaler = 0, means we calc FOC loop 1 times for 1 PWM cycles
     *           ADC postScaler = 1, means we calc FOC loop 1 times for 2 PWM cycles
     */
    p->config.adcPostScaler = (uint32_t)(p->config.pwmFreq / 40001.f);
    p->config.smpFreq = p->config.pwmFreq / (float)(p->config.adcPostScaler + 1);
    p->config.tS = 1.f / p->config.smpFreq;
    p->data.freqStep = M_2PI * p->config.tS;
    /* Max duty cycle*/
    p->volt.dutyMax = 0.975f;
    /* Set flags */
    p->flags.sensorlessStartup = 1;
    /* Init ramps */
    p->data.freqRmp = 20.f;  // Hz/s
    p->data.iqRmp = 50.f;    // A/s
    p->data.spdRmp = 5000.f; // (rad/s)/s
    /** Automatic calc PI gains depends on motor RL parameters
     *  wcc is cutoff frequency of controller
     *  RULE: if current sampling 1 times for 1 PWM period
     *  use wcc 5% of sampling frequency. Also try from 0.025 to 0.1
     */
    float wcc = (0.05f * p->config.smpFreq) * M_2PI;
    p->pi_id.Kp = p->pi_iq.Kp = (p->config.Ld * wcc);
    p->pi_id.Ki = p->pi_iq.Ki = p->config.Rs * wcc * p->config.tS;
    p->pi_id.Kc = p->pi_id.Ki / p->pi_id.Kp;
    p->pi_iq.Kc = p->pi_iq.Ki / p->pi_iq.Kp;
    p->pi_id.OutMin = p->pi_iq.OutMin = (p->config.mode == FOC) ? -p->config.adcFullScaleVoltage : 0;
    p->pi_id.OutMax = p->pi_iq.OutMax = (p->config.mode == FOC) ? p->config.adcFullScaleVoltage : 1.f;
    /* Init speed PI controller */
    p->pi_spd.Kp = 0.025f;
    p->pi_spd.Ki = 1.f * p->config.tS;
    p->pi_spd.Kd = 0.5f;
    p->config.currentMaxNeg = (p->config.currentMaxNeg > 0.f) ? -p->config.currentMaxNeg : p->config.currentMaxNeg;
    p->pi_spd.OutMin = (p->config.mode == FOC) ? p->config.currentMaxNeg : 0.f;
    p->pi_spd.OutMax = (p->config.mode == FOC) ? p->config.currentMaxPos : 0.99f;
    /* Init SMOPOS constants */
    p->smo.Fsmopos = expf((-p->config.Rs / p->config.Ld) * p->config.tS);
    p->smo.Gsmopos = (1.f / p->config.Rs) * (1.f - p->smo.Fsmopos);
    p->smo.Kslide = 0.53f;
    /* Init Flux observer constants */
    p->flux.R = p->config.Rs * 1.5f; // div 2 untill fixed voltage measurment scale
    p->flux.L = p->config.Ld * 1.5f;
    p->flux.fluxLeakage = 60.f / (SQRT3 * M_2PI * p->config.Kv * p->config.pp);
    p->flux.dt = p->config.tS;
    p->flux.gamma = (1000.f / (p->flux.fluxLeakage * p->flux.fluxLeakage)) * 0.5f; //300000.f;
    /* Init MTPA */
    p->mtpa.en = 0;
    p->mtpa.Ld = p->config.Ld;
    p->mtpa.Lq = p->config.Lq;
    p->mtpa.Ldq_diff = fabsf(p->config.Lq - p->config.Ld);
    p->mtpa.lambdaPM = p->flux.fluxLeakage;
    p->mtpa.iMax = p->prot.ocpThld * 0.9f; // 90% of overcurrent threshold
    p->mtpa.k_mtpa = 0.25f * (p->mtpa.lambdaPM / p->mtpa.Ldq_diff);
    /* Init PLL */
    p->pll.dt = p->config.tS;
    p->pll.Kp = 2000.f;
    p->pll.Ki = 30000.f;
    /* saturation protection thresholds */
    p->prot.ocpThld = (p->prot.ocpThld > p->config.adcFullScaleCurrent) ? p->config.adcFullScaleCurrent : p->prot.ocpThld;
    p->prot.ovpThld = (p->prot.ovpThld > p->config.adcFullScaleVoltage) ? p->config.adcFullScaleVoltage : p->prot.ovpThld;
    p->prot.enable = (p->config.sim == 1) ? 0 : p->prot.enable;
    p->data.hwCurrLim = p->prot.ocpThld * 0.95;
    /** BEMF filters init
     * Tf = 1 / (Fc * 2 * pi)
     *   */
    if (p->config.mode == BLDC)
    {
        p->cmtn.lpf_bemfA.T = p->config.tS / ((0.5f * p->config.speedMax) + p->config.tS);
        p->cmtn.lpf_bemfB.T = p->cmtn.lpf_bemfA.T;
        p->cmtn.lpf_bemfC.T = p->cmtn.lpf_bemfA.T;
    }
    if (p->config.mode == FOC)
    {
        p->cmtn.lpf_bemfA.T = p->config.tS / ((1.f / (1.5f * p->config.speedMax)) + p->config.tS);
        p->cmtn.lpf_bemfB.T = p->cmtn.lpf_bemfA.T;
        p->cmtn.lpf_bemfC.T = p->cmtn.lpf_bemfA.T;
    }
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
    p->lpf_offsetIc.T = p->config.tS / (0.2f + p->config.tS); // time constant in descrete time dominian for Q-axis current LPF
    p->lpf_offsetVa.T = p->config.tS / (0.2f + p->config.tS); // time constant in descrete time dominian for D-axis current LPF
    p->lpf_offsetVb.T = p->config.tS / (0.2f + p->config.tS); // time constant in descrete time dominian for Q-axis current LPF
    p->lpf_offsetVc.T = p->config.tS / (0.2f + p->config.tS); // time constant in descrete time dominian for Q-axis current LPF
    // Filters data init which execute in 1ms ISR
    float Ts_1ms = 1.f / 1000.f;
    p->lpf_Iavg.T = Ts_1ms / (0.02f + Ts_1ms); // time constant in descrete time dominian for total current LPF
    p->lpf_Pem.T = Ts_1ms / (0.02f + Ts_1ms);  // time constant in descrete time dominian for total current LPF
    p->lpf_NTC.T = Ts_1ms / (0.01f + Ts_1ms);  // time constant in descrete time dominian for total current LPF
}

#endif