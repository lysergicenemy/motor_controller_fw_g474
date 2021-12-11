/**
 ***********************************************************************
 * @file : foc_ctrl.h
 * @brief: Control functions for FOC library
 *  */

#ifndef FOC_CTRL_H
#define FOC_CTRL_H

/** Ramp function (trapezoidal profile)
 * @input:  in - referance value (rad/s, hz, rpm)
 * @output: out - pointer for actual value (rad/s, hz, rpm)
 * @param:  rampRate - maximal acceleration (rad/s^2, hz^2, rpm^2)
 *          Ts - sample time
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
 * @input:  in - referance value (rad/s, hz, rpm)
 * @output: out - pointer for actual value (rad/s, hz, rpm)
 * @param:  rampRate - maximal acceleration in linear region (rad/s^2, hz^2, rpm^2)
 *          Ts - sample time
 */
static inline void ramp_s_calc(volatile float *out, float in, float rampRate, float Ts)
{
    // if acceleration/decceleration phases is 25% of actual rampRate mast be multiplyed by 1.5
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

/** Angle generator
 * @input:  freq - referance value (rad/s, hz, rpm)
 * @output: angle - pointer for actual value (rad, deg, P.U.)
 * @param:  Ts - sample time
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
    if (p->pll.SpeedAbs < (p->config.speedMax * 0.05f))
    {
        // detect run comand
        if (p->pll.SpeedAbs > 0.f || fabsf(p->data.iqRef) > 0.f)
        {
            p->flags.sensorlessStartup = 0;
        }
    }
    // openLoop startUp
    if (p->flags.sensorlessStartup == 0)
    {
        ol_dir = (p->driveState == RUN_SPD) ? SIGN(p->data.speedRef) : ol_dir;      // direction if speed control mode
        ol_dir = (p->driveState == RUN_CURRENT) ? SIGN(p->data.iqRef) : ol_dir;     // direction if current control mode
        iqRefShdw = p->config.currentMaxPos * 0.5f * (float)ol_dir;                 // startUp current 50% of max
        p->data.freqRef = (p->config.speedMax * 0.1f) * ONE_BY_2PI * (float)ol_dir; // startUp speed 10% of max speed
        p->data.freqRmp = fabsf(p->data.freqRef * 5.f);                             // 0.2s startUp time
        ol_timeout++;
        // Align process
        if (ol_timeout < (uint32_t)(1.f * p->data.smpFreq))
        {
            p->data.angle = 0.f;
            ramp_calc(&p->pi_iq.Ref, iqRefShdw, fabsf(iqRefShdw), p->data.tS);
        }
        // Ramp up speed
        else
        {
            ramp_calc(&p->data.freq, p->data.freqRef, p->data.freqRmp, p->data.tS);
            // Angle generator
            p->data.angle += p->data.freqStep * p->data.freq;
            if (p->data.angle < -MF_PI)
                p->data.angle = p->data.angle + M_2PI;
            else if (p->data.angle > MF_PI)
                p->data.angle = p->data.angle - M_2PI;
        }
        // detect succesfull startup
        if (p->pll.SpeedAbs > (fabsf(p->data.freqRef) * M_2PI * 0.9f))
        {
            //ramp_calc(&p->data.idRef, 0.f, 2.f, p->data.tS);
            p->flags.sensorlessStartup = 1;
            p->pi_iq.Ref = (p->driveState == RUN_CURRENT) ? p->data.iqRef : 0.f;
            p->pi_spd.Ui = (p->driveState == RUN_SPD) ? iqRefShdw : 0.f;
            p->data.freq = 0.f;
            p->data.freqRef = 0.f;
            ol_timeout = 0;
        }
        // detect no succesfull startup
        if (ol_timeout > (uint32_t)(2.f * p->data.smpFreq) && p->pll.SpeedAbs < (fabsf(p->data.freqRef) * M_2PI * 0.9f))
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
 * @input:  idRef, iqRef, isa, isb, sinTheta, cosTheta (must be updated before call)
 * @output: usa, usb - referance alpha/beta components of phase voltages
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
        p->mtpa.iqRef = p->data.iqRef;
        p->calc.mtpa(&p->mtpa);
        p->pi_id.Ref = p->mtpa.id_MTPA;
        p->pi_iq.Ref = p->mtpa.iq_MTPA;
    }
    // calc D-axis current injection in low speed region
    else if (p->config.sensorType == SENSORLESS && p->flags.sensorlessInjectionD == 1)
    {
        //p->pi_id.Ref = (p->pll.SpeedAbs < p->data.wMinSensorless) ? p->pi_iq.Ref - ((p->pi_iq.Ref / p->data.wMinSensorless) * p->pll.SpeedPll) : 0.f;
        p->pi_id.Ref = (p->pll.SpeedAbs < (p->data.wMinSensorless)) ? p->pi_iq.Ref : p->pi_id.Ref;
        p->pi_id.Ref = (p->pll.SpeedAbs > (p->data.wMinSensorless * 1.5f)) ? 0.f : p->pi_id.Ref;
    }
    // calc Cogging torque compensation. EXPERIMENTAL
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
        p->data.udDec = p->pi_iq.Ref * p->pll.SpeedPll * p->config.Lq;
        p->data.uqDec = p->pi_id.Ref * p->pll.SpeedPll * p->config.Ld;
        break;
    case DEC_BEMF:
        p->data.udDec = 0.f;
        p->data.uqDec = p->pll.SpeedPll * p->flux.fluxLinkage;
        break;
    case DEC_CROSS_BEMF:
        p->data.udDec = p->pi_iq.Ref * p->pll.SpeedPll * p->config.Lq;
        p->data.uqDec = p->pll.SpeedPll * (p->pi_id.Ref * p->config.Ld + p->flux.fluxLinkage);
        break;
    default:
        break;
    }
    // calc PI
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
    float voltage_scale = (p->flags.overmodulation == 1) ? (2.f / 3.f) : ONE_BY_SQRT3;
    p->volt.magMaxD = p->volt.dutyMax * voltage_scale * p->volt.DcBusVolt;
    p->volt.magMaxQ = sqrtf(SQ(p->volt.magMaxD) - SQ(p->volt.usd));
    // Saturation
    utils_saturate_vector_2d((float *)&p->volt.usd, (float *)&p->volt.usq, p->volt.magMaxD);
    // calc IPARK transform (IPARK output -> SVGEN input)
    p->svgen.usa = p->volt.usd * p->data.cosTheta - p->volt.usq * p->data.sinTheta;
    p->svgen.usb = p->volt.usq * p->data.cosTheta + p->volt.usd * p->data.sinTheta;
}

/** Run angle and speed estimation/calculation
 * Hall, flux observer, pll
 * @input:  usa, usb, isa, isb (must be updated before call):
 * @output: angle - angle of rotor flux
 *          SpeedPll, speedRpm - electrical(rad/s) and mechanical(rpm) rotor speed
 */
static inline void Foc_update_angle_speed(foc_t *p)
{
    switch (p->config.sensorType)
    {
    case MANUAL:
        // ramp generator
        ramp_s_calc(&p->data.freq, p->data.freqRef, p->data.freqRmp, p->data.tS);
        // Angle generator
        angle_gen(&p->data.angle, p->data.freq, p->data.tS);
        p->pll.SpeedPll = p->data.freq * M_2PI;
        break;
    case HALL:
        // calc speed PLL
        p->pll.AngleRaw = p->data.hallAngle;
        p->calc.pll(&p->pll);
        p->data.angle = (p->pll.SpeedAbs > 50.f) ? p->pll.AnglePll : p->data.hallAngle;
        break;
    case SENSORLESS:
        // calc flux observer
        p->flux.i_alpha = p->data.isa;
        p->flux.i_beta = p->data.isb;
        p->flux.v_alpha = p->volt.usa;
        p->flux.v_beta = p->volt.usb;
        p->calc.flux(&p->flux);
        // calc speed PLL
        p->pll.AngleRaw = p->flux.phase;
        p->data.angle = p->flux.phase;
        p->calc.pll(&p->pll);
        break;
    case HYBRYD:
        // calc flux observer
        p->flux.i_alpha = p->data.isa;
        p->flux.i_beta = p->data.isb;
        p->flux.v_alpha = p->volt.usa;
        p->flux.v_beta = p->volt.usb;
        p->calc.flux(&p->flux);
        // calc speed PLL
        volatile static uint8_t hybryd_state;
        hybryd_state = (p->pll.SpeedAbs > 325.f) ? 1 : hybryd_state;
        hybryd_state = (p->pll.SpeedAbs < 275.f) ? 0 : hybryd_state;
        if (hybryd_state == 0)
        {
            p->pll.AngleRaw = p->data.hallAngle;
            p->calc.pll(&p->pll);
            p->data.angle = (p->pll.SpeedAbs > 50.f) ? p->pll.AnglePll : p->data.hallAngle;
        }
        else
        {
            p->pll.AngleRaw = p->data.hallAngle;
            p->data.angle = p->flux.phase;
            p->calc.pll(&p->pll);
        }
        break;

    default:
        break;
    }
    p->data.speedRpm = (p->pll.SpeedPll * RADS2RPM) / p->config.pp;
}

/** Run BLDC commutation logic
 * Hall, flux observer, pll
 * @input:  usa, usb, isa, isb (must be updated before call):
 * @output: angle - angle of rotor flux
 *          SpeedPll, speedRpm - electrical(rad/s) and mechanical(rpm) rotor speed
 */
static inline void Bldc_update_commutation(foc_t *p, hall_t *ph)
{
    switch (p->config.sensorType)
    {
    case MANUAL:
        // ramp generator
        ramp_s_calc(&p->data.freq, p->data.freqRef, p->data.freqRmp, p->data.tS);
        // Angle generator
        angle_gen(&p->data.angle, p->data.freq, p->data.tS);
        p->cmtn.cmtnState = (uint8_t)(p->data.angle + MF_PI);
        p->cmtn.cmtnState = SAT(p->cmtn.cmtnState, 5, 0);
        break;
    case HALL:
        // calc Hall sensor position
        Hall_read(ph, 4);
        Hall_update(ph);
        p->data.hallAngle = ph->angleRaw;
        p->data.angle = MF_PI - (M_PI3 + ((float)ph->state * M_PI3));
        p->cmtn.cmtnState = ph->state;
        break;
    case SENSORLESS:
        p->calc.cmtn(&p->cmtn);
        p->data.angle = MF_PI - (M_PI3 + ((float)p->cmtn.cmtnState * M_PI3));
        break;
    case HYBRYD:

        break;

    default:
        break;
    }
    p->pll.AngleRaw = p->data.angle;
    p->calc.pll(&p->pll);
    p->data.speedRpm = (p->pll.SpeedPll * RADS2RPM) / p->config.pp;
    /* if first enter, make 1 commutation blindly */
    if (p->cmtn.Trigger > 0 || p->flags.bldcStartup == 1)
    {
        p->data.duty = (p->flags.bldcStartup == 1) ? p->data.dutyRef : p->data.duty;
        p->flags.bldcStartup = 0;
        p->data.isrCntr1 = 0;
        p->cmtn.dir = (p->data.duty < 0.f) ? 1 : 0;
        if (p->cmtn.dir == 1)
            p->cmtn.cmtnState = (p->cmtn.cmtnState >= 5) ? 0 : p->cmtn.cmtnState + 1;
        else
            p->cmtn.cmtnState = (p->cmtn.cmtnState <= 0) ? 5 : p->cmtn.cmtnState - 1;
    }
    /* Check stacked rotor. 50ms timeout  */
    if (p->data.isrCntr1 >= (uint32_t)(0.05f * p->data.smpFreq) && p->config.sensorType == SENSORLESS)
    {
        p->flags.bldcStartup = 1;
        p->data.isrCntr1 = 0;
    }
}

/** Run D/Q currents controller
 * PARK -> DECOUPLING -> PI -> DQ_Limiter -> IPARK
 * @input:  idRef, iqRef, isa, isb, sinTheta, cosTheta (must be updated before call)
 * @output: usa, usb - referance alpha/beta components of phase voltages
 */
static inline void Foc_main_task(foc_t *p)
{
    
}

/** Run D/Q currents controller
 * PARK -> DECOUPLING -> PI -> DQ_Limiter -> IPARK
 * @input:  idRef, iqRef, isa, isb, sinTheta, cosTheta (must be updated before call)
 * @output: usa, usb - referance alpha/beta components of phase voltages
 */
static inline void Foc_slow_task(foc_t *p)
{
    
}

static inline void Foc_Init(foc_t *p)
{
    /** Init timeBase values
     *  20000 Hz is maximal FOC execution rate for any PWM freq (TESTED: 170Mhz CORTEX-M4F)
     *  If FOC ISR placed on CCMRAM, disable virtual motor and optimization level >= O2 it can be increased up to 40000 Hz
     *  Example: ADC postScaler = 0, means we calc FOC loop 1 times for 1 PWM cycles
     *           ADC postScaler = 1, means we calc FOC loop 1 times for 2 PWM cycles
     */
    p->data.adcPostScaler = (uint32_t)(p->config.pwmFreq / 40001.f);
    p->data.smpFreq = p->config.pwmFreq / (float)(p->data.adcPostScaler + 1);
    p->data.tS = 1.f / p->data.smpFreq;
    p->data.freqStep = M_2PI * p->data.tS;
    /* Max duty cycle */
    p->volt.dutyMax = 0.975f;
    /* Set flags */
    p->flags.sensorlessStartup = 1;
    /* Init ramps */
    p->data.freqRmp = 20.f;  // Hz/s
    p->data.iqRmp = 50.f;    // A/s
    p->data.spdRmp = 5000.f; // (rad/s)/s
    p->data.dutyRmp = 0.5f;  // P.U./s
    /** Automatic calc PI gains depends on motor RL parameters
     *  wcc is cutoff frequency of controller
     *  RULE: if current sampling 1 times for 1 PWM period
     *  use wcc 5% of sampling frequency. Also try from 0.025 to 0.1
     */
    float wcc = (0.05f * p->data.smpFreq) * M_2PI;
    p->pi_id.Kp = p->pi_iq.Kp = (p->config.Ld * wcc);
    p->pi_id.Ki = p->pi_iq.Ki = p->config.Rs * wcc * p->data.tS;
    p->pi_id.Kc = p->pi_id.Ki / p->pi_id.Kp;
    p->pi_iq.Kc = p->pi_iq.Ki / p->pi_iq.Kp;
    p->pi_id.OutMin = p->pi_iq.OutMin = (p->config.mode == FOC) ? -p->config.adcFullScaleVoltage : 0;
    p->pi_id.OutMax = p->pi_iq.OutMax = (p->config.mode == FOC) ? p->config.adcFullScaleVoltage : 1.f;
    /* Init speed PI controller */
    p->pi_spd.Kp = 0.025f;
    p->pi_spd.Ki = 1.f * p->data.tS;
    p->pi_spd.Kd = 0.5f;
    p->config.currentMaxNeg = (p->config.currentMaxNeg > 0.f) ? -p->config.currentMaxNeg : p->config.currentMaxNeg;
    p->pi_spd.OutMin = (p->config.mode == FOC) ? -(fabsf(p->config.currentMaxNeg)) : 0.f;
    p->pi_spd.OutMax = (p->config.mode == FOC) ? p->config.currentMaxPos : 0.99f;
    /* Init SMOPOS constants */
    p->smo.Fsmopos = expf((-p->config.Rs / p->config.Lq) * p->data.tS);
    p->smo.Gsmopos = (1.f / p->config.Rs) * (1.f - p->smo.Fsmopos);
    p->smo.lpfWc = M_2PI * 5.f; // low-pass filter cutoff frequency
    p->smo.lpfTf = p->data.tS / ((1.f / p->smo.lpfWc) + p->data.tS);
    /* Init Flux observer constants */
    p->flux.R = p->config.Rs * 1.5f;
    p->flux.L = ((p->config.Ld + p->config.Lq) * 0.5f) * 1.5f; // take average inductance of Ld and Lq
    p->flux.fluxLinkage = 60.f / (SQRT3 * M_2PI * p->config.Kv * p->config.pp);
    p->flux.dt = p->data.tS;
    p->flux.gamma = (1000.f / (p->flux.fluxLinkage * p->flux.fluxLinkage)) * 0.5f; //300000.f;
    p->data.wMinSensorless = 0.25f * p->flux.gamma * SQ(p->flux.fluxLinkage);
    /* Init MTPA */
    p->mtpa.en = 0;
    p->mtpa.Ld = p->config.Ld;
    p->mtpa.Lq = p->config.Lq;
    p->mtpa.Ldq_diff = fabsf(p->config.Lq - p->config.Ld);
    p->mtpa.lambdaPM = p->flux.fluxLinkage;
    p->mtpa.iMax = p->prot.ocpThld * 0.9f; // 90% of overcurrent threshold
    p->mtpa.k_mtpa = 0.25f * (p->mtpa.lambdaPM / p->mtpa.Ldq_diff);
    /* Init PLL */
    p->pll.dt = p->data.tS;
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
    p->cmtn.lpf_bemfA.T = p->data.tS / ((1.f / (1.5f * p->config.speedMax)) + p->data.tS);
    p->cmtn.lpf_bemfB.T = p->cmtn.lpf_bemfA.T;
    p->cmtn.lpf_bemfC.T = p->cmtn.lpf_bemfA.T;
    p->cmtn.cmtnTh = 0.01f;
    // BLDC mode settings
    p->data.bldc_commCntrRef = 350.f;
    p->data.dutyStart = 0.1f; // 10% of max value
    // Filters data init which execute in main ADC-ISR
    p->lpf_id.T = p->data.tS / (0.005f + p->data.tS);     // time constant in descrete time dominian for D-axis current LPF
    p->lpf_iq.T = p->data.tS / (0.005f + p->data.tS);     // time constant in descrete time dominian for Q-axis current LPF
    p->lpf_vdc.T = p->data.tS / (0.1f + p->data.tS);      // time constant in descrete time dominian for DC-Bus voltage LPF
    p->lpf_offsetIa.T = p->data.tS / (0.2f + p->data.tS); // time constant in descrete time dominian for D-axis current LPF
    p->lpf_offsetIb.T = p->data.tS / (0.2f + p->data.tS); // time constant in descrete time dominian for Q-axis current LPF
    p->lpf_offsetIc.T = p->data.tS / (0.2f + p->data.tS); // time constant in descrete time dominian for Q-axis current LPF
    p->lpf_offsetVa.T = p->data.tS / (0.2f + p->data.tS); // time constant in descrete time dominian for D-axis current LPF
    p->lpf_offsetVb.T = p->data.tS / (0.2f + p->data.tS); // time constant in descrete time dominian for Q-axis current LPF
    p->lpf_offsetVc.T = p->data.tS / (0.2f + p->data.tS); // time constant in descrete time dominian for Q-axis current LPF
    // Filters data init which execute in 1ms ISR
    float Ts_1ms = 1.f / 1000.f;
    p->lpf_Iavg.T = Ts_1ms / (0.02f + Ts_1ms); // time constant in descrete time dominian for total current LPF
    p->lpf_Pem.T = Ts_1ms / (0.02f + Ts_1ms);  // time constant in descrete time dominian for total current LPF
    p->lpf_NTC.T = Ts_1ms / (0.01f + Ts_1ms);  // time constant in descrete time dominian for total current LPF
}

#endif