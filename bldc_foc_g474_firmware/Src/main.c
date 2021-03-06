/**
 * BLDC FOC
 *   Operation modes:
 * 1. DQ-axis current control
 * 2. Speed control
 * 3. Sensorless operation, sensored (Hall sensors signals extrapolation)
 * 4. Motor parameter identification (Rs,Ld,Lq, Kv(Flux Linkage), Hall sensors table)
 * 5. Open-loop modes (V/Hz, I/Hz)
 * 6. Sensorless 6-step bldc mode (zc + bemf integration) + hardware PCMC
 * 7. Simulation virtual motor
 * 8. BDCM control + hardware PCMC
 * */

/**
 * TODO:
 * 1. Add CANOpen. CiA 402 (IEC 61800-7-201/301)
 * 8. SpeedRef - ERADS, replace to M/ERPM
 * 10. Separated hwconfig and focconfig
 * 11. Add Motor inertia (J) estimation for auto tune speed controller:
 *     https://e2e.ti.com/blogs_/b/industrial_strength/posts/teaching-your-pi-controller-to-behave-part-viii
 * 12. Add PPM protocol support
 * 14. Add DC/DC converter mode (Buck, Boost, Interleaved)
 * 
 * */

#include "main.h"
#include "adc.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "cordic.h"
#include "dac.h"
#include "comp.h"
#include "flash.h"
#include "fdcan.h"
#include "sys.h"

void CoreInit(void);
void CoreStart(void);
void SystemClock_Config(void);
void Foc_connect_hardware(void);

dataLogVars_t dataLog;       /* DataLogger struct */
adcData_t adcData;           /* ADC data struct */
ntc_t ntcPcb;                /* NTC temp sensing struct */
modelBLDC_t bldc1;           /* BLDC motor mathematical model */
hall_t hall = HALL_DEFAULTS; /* Hall sensors module */
foc_t foc1 = FOC_DEFAULTS;   /* Field-oriented control algorithms */
extern canData_t can;

volatile uint32_t start_cc;       /* Var. Data Watchpoint and Trace Unit */
volatile uint32_t stop_cc;        /* Var. Data Watchpoint and Trace Unit */
volatile uint32_t execTime0;      /* Var. Data Watchpoint and Trace Unit */
volatile uint32_t execTime1;      /* Var. Data Watchpoint and Trace Unit */
volatile uint8_t e = 0;           /* First enter flag */
volatile uint8_t systemReset = 0; /* MCU reset request */

// Auxiliary globals
volatile float ledDelay = 0;      /* Delay betwen toggle led, s */
volatile uint32_t pwmInputPeriod; /* Period of PWM-input signal */
volatile uint32_t pwmInputValue;  /* Duty of PWM-input signal */
volatile uint32_t dac_value;
volatile uint32_t angleIncDelay = 0;

int main(void)
{
  /* Configure the system clock, flash, interrupts and periph */
  CoreInit();
  /* Init BLDC model */
  ModelBLDC_Init(&bldc1);
  /* set FOC config parameters */
  foc1.config.mode = FOC;                  // FOC or BLDC(6-step) or BDC
  foc1.config.sim = 0;                     // 0 - real motor, 1 - motor model
  foc1.config.sensorType = HYBRYD;         // Motor position sensor (HALL, SENSORLESS)
  foc1.config.pwmFreq = 40000.f;           // set PWM frequency
  foc1.config.adcFullScaleCurrent = 50.f;  // set current wich means 4095 ADC value: adcRef / R_CSR / opAmpGain = 1.65 / 0.002 / 16.5 = 50.0
  foc1.config.adcFullScaleVoltage = 69.3f; // set voltage wich means 4095 ADC value: 3.3 * 21 = 69.3
  foc1.config.deadTime = 160.0f;           // set deadTime, nS (NOTE: 120ns for Rg+Rgdrv+Rgint=1.6+1.1+1=3.6 Ohm, Qg = 100nC)
  foc1.config.Rds_on = 0.0019f;            // set MOSFET Rds(on), Ohm (used for Rs,Ld/Lq identification)
  foc1.config.pp = 4.f;                    // set motor pole pairs
  foc1.config.Rs = 0.085f;                 // set motor phase resistance. Can be measured
  foc1.config.Ld = 0.000045f;              // set motor D-axis inductance. Can be measured
  foc1.config.Lq = foc1.config.Ld;         // set motor Q-axis inductance. Can be measured
  foc1.config.Kv = 270.f;                  // set motor Kv. Can be measured
  foc1.config.decMode = DEC_BEMF;          // feedForward compensation (DQ axis decoupling)
  foc1.config.currentMaxPos = 40.f;        // Motor maximal current
  foc1.config.currentMaxNeg = 10.f;        // Motor maximal breaking current
  foc1.config.speedMax = 3000.f;           // Motor maximal speed, erads/s
  /* Set protection thresholds */
  foc1.prot.enable = false;     // 1 - enable protection, 0 - disable
  foc1.prot.ocpThld = 50.0f;    // set Over Current Protection threshold (peak p-p)
  foc1.prot.ovpThld = 65.0f;    // set Over Voltage Protection threshold (DC-link)
  foc1.prot.uvloThld = 9.99f;   // set Under Voltage LockOut threshold (DC-link)
  foc1.prot.pcbTempThld = 80.f; // set PCB temperature threshold
  /* Initialize PCB temperature sensing module */
  ntcPcb.r_balance = 10000.f; // balance resistor value
  ntcPcb.r_ntc_0 = 4700.f;    // NTC base resistance
  ntcPcb.ta_0 = 25.f;         // NTC base temperature
  ntcPcb.betta = 3950.f;      // NTC beta coefficient
  /* Load parameters from flash */
  FLASH_LoadConfig(&foc1, &hall);
  /* Init FOC */
  Foc_Init(&foc1);
  Foc_connect_hardware();
  /* Start hardware */
  CoreStart();

  while (1)
  {
    /* Check CPU reset request */
    if (systemReset != 0)
    {
      //FLASH_UpdateConfig(&foc1, &hall);
      NVIC_SystemReset();
    }
    /* Check motor parameters identification complete */
    if (foc1.paramIdState == ID_CMPLT && foc1.paramIdRunState == ID_RUN_CMPLT)
    {
      /* Save parameters into flash after identification */
      FLASH_UpdateConfig(&foc1, &hall);
      //NVIC_SystemReset();
    }
  }
}

void CoreInit(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_PWR_DisableDeadBatteryPD();

  /* config clock */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_8);
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while (LL_RCC_HSE_IsReady() != 1)
  {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 68, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 68, LL_RCC_PLLP_DIV_3);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_EnableDomain_ADC();
  LL_RCC_PLL_Enable();
  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady() != 1)
  {
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }
  /* Insure 1???s transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while (DWT->CYCCNT < 100)
    ;
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(170000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(170000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_SYSCLK);
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_PCLK1);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_PLL);
  LL_RCC_SetADCClockSource(LL_RCC_ADC345_CLKSOURCE_PLL);
  LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);
}

void Foc_connect_hardware(void)
{
  foc_pwm_init = hrtim_start;
  foc_pwm_off = hrtim_pwm_disable;
  foc_pwm_on = hrtim_pwm_enable;
  foc_pwm_update = hrtim_pwm_update;
  foc_bldcpwm_update = hrtim_bldcpwm_update;
  foc_sincos = cordic_sincos_calc;
  foc_atan2 = cordic_atan2_calc;
}

void CoreStart(void)
{
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  Cordic_Init();
  MX_DAC2_Init();
  MX_DAC3_Init();
  MX_COMP1_Init();
  MX_COMP4_Init();
  MX_COMP6_Init();
  foc_pwm_init(foc1.config.pwmFreq, foc1.config.deadTime, foc1.data.adcPostScaler, &foc1.data.halfPwmPeriod);
  tim2_start();
  tim7_start();
  tim6_start();
  dac_start();
  cmpr_start(); // TODO: before PWM
  adc_start();
}

/* ISR for ADC sampling complited */
void ADC1_2_IRQHandler(void)
{
  start_cc = DWT->CYCCNT;

  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_ClearFlag_JEOS(ADC2);

  /* heartbit */
  foc1.data.isrCntr0++;                                               // ISR counter for toggle LED
  foc1.data.isrCntr1++;                                               // ISR counter for bldc startUP
  foc1.data.isrCntr2++;                                               // ISR counter for param identification case in FOC mode
  if (foc1.data.isrCntr0 >= (uint32_t)(ledDelay * foc1.data.smpFreq)) // Status LED control
  {
    LL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    foc1.data.isrCntr0 = 0;
  }
  /*******************************************************************
     * 6-step / FOC section
     ******************************************************************/

  /* take data from ADC */
  adcData.ph_u = ADC1->JDR3;     // current phase A
  adcData.ph_v = ADC1->JDR1;     // current phase B
  adcData.ph_w = ADC1->JDR2;     // current phase C
  adcData.v_dc = ADC1->JDR4;     // DC-Bus voltage
  adcData.v_u = ADC2->JDR3;      // BEMF phase A
  adcData.v_v = ADC2->JDR1;      // BEMF phase B
  adcData.v_w = ADC2->JDR2;      // BEMF phase C
  adcData.pcb_temp = ADC2->JDR4; // board temperature
  adc_get_inputs(&adcData, &foc1, &bldc1);

  switch (foc1.driveState)
  {
  case STOP:
    /* In STOP case: set dutyClycle to 50%, calc offset currents and reset all modules */
    // Reset if FAULT state before
    LL_GPIO_ResetOutputPin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
    // Reset controllers
    foc1.data.idRef = 0.f;
    foc1.data.iqRef = 0.f;
    foc1.pi_id.Ui = 0.f;
    foc1.pi_id.OutPreSat = 0.f;
    foc1.pi_iq.Ui = 0.f;
    foc1.pi_iq.OutPreSat = 0.f;
    foc1.pi_spd.Ui = 0.f;
    foc1.pi_spd.OutPreSat = 0.f;
    foc1.volt.usd = 0.f;
    foc1.volt.usq = 0.f;
    foc1.data.angle = 0.f;
    ledDelay = 0.f;
    foc1.flags.bldcStartup = 1;
    foc1.cmtn.cmtnState = 0;
    // calc current sensors offset
    foc1.lpf_vdc.in = (float)adcData.v_dc * (1.f / 4096.f) * foc1.config.adcFullScaleVoltage;
    foc1.lpf_offsetIa.in = (float)adcData.ph_u;
    foc1.lpf_offsetIb.in = (float)adcData.ph_v;
    foc1.lpf_offsetIc.in = (float)adcData.ph_w;
    foc1.lpf_offsetVa.in = (float)adcData.v_u * ((1.f / 4096.f) * foc1.config.adcFullScaleVoltage);
    foc1.lpf_offsetVb.in = (float)adcData.v_v * ((1.f / 4096.f) * foc1.config.adcFullScaleVoltage);
    foc1.lpf_offsetVc.in = (float)adcData.v_w * ((1.f / 4096.f) * foc1.config.adcFullScaleVoltage);
    // Filtered data
    LPF_calc(&foc1.lpf_vdc);
    LPF_calc(&foc1.lpf_offsetIa);
    LPF_calc(&foc1.lpf_offsetIb);
    LPF_calc(&foc1.lpf_offsetIc);
    LPF_calc(&foc1.lpf_offsetVa);
    LPF_calc(&foc1.lpf_offsetVb);
    LPF_calc(&foc1.lpf_offsetVc);
    foc1.volt.DcBusVolt = (foc1.config.sim == 0) ? foc1.lpf_vdc.out : bldc1.udc;
    float invDC = 1.f / foc1.volt.DcBusVolt;
    foc1.data.offsetCurrA = foc1.lpf_offsetIa.out;
    foc1.data.offsetCurrB = foc1.lpf_offsetIb.out;
    foc1.data.offsetCurrC = foc1.lpf_offsetIc.out;
    foc1.data.offsetVoltA = foc1.lpf_offsetVa.out * invDC;
    foc1.data.offsetVoltB = foc1.lpf_offsetVb.out * invDC;
    foc1.data.offsetVoltC = foc1.lpf_offsetVc.out * invDC;
    // set duty to 50%
    switch (foc1.config.mode)
    {
    case DC:
      foc_pwm_update(0, 0, 0, foc1.data.halfPwmPeriod);
      break;
    case BLDC:
      //foc_bldcpwm_update(foc1.data.halfPwmPeriod, 0.f, foc1.cmtn.cmtnState);
      foc_pwm_update(0, 0, 0, foc1.data.halfPwmPeriod);
      break;
    case FOC:
      // calc angle and speed
      Foc_update_angle_speed(&foc1);
      foc_pwm_update(0, 0, 0, foc1.data.halfPwmPeriod);
      break;

    default:
      break;
    }
    foc_pwm_on();
    // calc BLDC motor model
    ModelBLDC_Calc(&bldc1);
    // check enable pin
    // if (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_Pin) == 0 && foc1.volt.DcBusVolt >= 18.f && pwmInputValue < 100)
    // {
    //   foc1.driveState = RUN_CLOSEDLOOP_SPD;
    // }
    // Check protection and enable if precharge is done
    foc1.prot.prechargeFlag = (foc1.volt.DcBusVolt >= 20.f) ? 1 : 0;
    foc1.prot.currPhU = foc1.data.isa;
    foc1.prot.currPhV = foc1.data.isb;
    foc1.prot.udc = foc1.volt.DcBusVolt;
    foc1.prot.tempPcb = ntcPcb.temp;
    foc1.calc.prot(&foc1.prot);
    foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
    // call datalogger
    foc1.cgtc.tabNmbr++;
    foc1.cgtc.tabNmbr = (foc1.cgtc.tabNmbr >= 99) ? 0 : foc1.cgtc.tabNmbr;
    dataLog.in1 = foc1.cgtc.tabNmbr;
    dataLog.in2 = foc1.cgtc.coggingMap[foc1.cgtc.tabNmbr];
    dataLog.in3 = foc1.pi_id.Ref;
    dataLog.in4 = foc1.pi_iq.Ref;
    datalogCalcFast(&dataLog);
    break;

  case RUN_DUTY:
    switch (foc1.config.mode)
    {
    case DC:
      foc1.data.isq = foc1.data.iPhaseA - foc1.data.iPhaseC;
      // Filtered data
      foc1.lpf_iq.in = foc1.data.isq;
      foc1.lpf_vdc.in = foc1.volt.DcBusVolt;
      LPF_calc(&foc1.lpf_iq);
      LPF_calc(&foc1.lpf_vdc);
      foc1.lpf_id.out = 0.f;
      foc1.volt.DcBusVolt = foc1.lpf_vdc.out;
      // set duty
      ramp_calc(&foc1.data.duty, foc1.data.dutyRef, foc1.data.dutyRmp, foc1.data.tS);
      foc1.data.duty = SAT(foc1.data.duty, 1.f, -1.f);
      foc_pwm_update(foc1.data.duty, 0, -foc1.data.duty, foc1.data.halfPwmPeriod);
      // Check protection
      foc1.prot.currPhU = foc1.data.isa;
      foc1.prot.currPhV = foc1.data.isb;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // call datalogger
      if (foc1.data.iqRef != dataLog.trackedValue || foc1.prot.protFlag != 0)
        dataLog.trigger = 1;
      dataLog.trackedValue = foc1.data.iqRef;
      dataLog.in1 = foc1.pi_iq.Out;
      dataLog.in2 = foc1.data.isq;
      dataLog.in3 = foc1.data.iPhaseA;
      dataLog.in4 = foc1.data.iPhaseC;
      datalogCalcFast(&dataLog);
      break;
    case BLDC:
      foc1.cmtn.As = (float)(adcData.v_u) * (1.f / 4096.f) * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.Bs = (float)(adcData.v_v) * (1.f / 4096.f) * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.Cs = (float)(adcData.v_w) * (1.f / 4096.f) * foc1.config.adcFullScaleVoltage;
      // set duty
      ramp_calc(&foc1.data.duty, foc1.data.dutyRef, foc1.data.dutyRmp, foc1.data.tS);
      foc1.data.duty = SAT(foc1.data.duty, 1.f, -1.f);
      // calc commutation module
      Bldc_update_commutation(&foc1, &hall);
      // Update PWM values
      foc_bldcpwm_update(foc1.data.halfPwmPeriod, fabsf(foc1.data.duty), foc1.cmtn.cmtnState); //hall.state);
      // Check protection
      foc1.prot.currPhU = foc1.data.isa;
      foc1.prot.currPhV = foc1.data.isb;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // call datalogger
      dataLog.in1 = foc1.cmtn.An;
      dataLog.in2 = foc1.cmtn.Bn;
      dataLog.in3 = foc1.cmtn.Cn;
      dataLog.in4 = foc1.cmtn.fluxBuff;
      datalogCalcFast(&dataLog);
      break;
    case FOC:
      // calc sin/cos
      foc_sincos(foc1.data.angle, &foc1.data.sinTheta, &foc1.data.cosTheta);
      // PARK transform for phase currents
      foc1.data.isd = foc1.data.isa * foc1.data.cosTheta + foc1.data.isb * foc1.data.sinTheta;
      foc1.data.isq = -foc1.data.isa * foc1.data.sinTheta + foc1.data.isb * foc1.data.cosTheta;
      // Filtered data
      foc1.lpf_id.in = foc1.data.isd;
      foc1.lpf_iq.in = foc1.data.isq;
      foc1.lpf_vdc.in = foc1.volt.DcBusVolt;
      LPF_calc(&foc1.lpf_id);
      LPF_calc(&foc1.lpf_iq);
      LPF_calc(&foc1.lpf_vdc);
      foc1.volt.DcBusVolt = foc1.lpf_vdc.out;
      // calc duty cycle control
      float voltage_scale = (foc1.flags.overmodulation == 1) ? (2.f / 3.f) : ONE_BY_SQRT3;
      ramp_calc(&foc1.data.duty, foc1.data.dutyRef, foc1.data.dutyRmp, foc1.data.tS);
      foc1.volt.usd = foc1.data.udRef;
      foc1.volt.usq = foc1.data.duty * foc1.volt.DcBusVolt * voltage_scale;
      foc1.volt.magMaxD = foc1.volt.dutyMax * voltage_scale * foc1.volt.DcBusVolt;
      foc1.volt.magMaxQ = sqrtf(SQ(foc1.volt.magMaxD) - SQ(foc1.volt.usd));
      // Saturation
      utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.magMaxD);
      // calc IPARK transform (IPARK output -> SVGEN input)
      foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
      foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
      // calc space-vector generator and update PWM
      foc1.svgen.udc = foc1.volt.DcBusVolt;
      foc1.calc.svgen(&foc1.svgen);
      foc_pwm_update(foc1.svgen.Ta, foc1.svgen.Tb, foc1.svgen.Tc, foc1.data.halfPwmPeriod);
      // calc phase voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      // calc Hall sensor position
      Hall_read(&hall, 4);
      hall.speedE = foc1.pll.SpeedPll;
      Hall_update(&hall);
      foc1.data.hallAngle = hall.angleRaw;
      // calc flux observer
      Foc_update_angle_speed(&foc1);
      // Check protection
      foc1.prot.currPhU = foc1.data.iPhaseA;
      foc1.prot.currPhV = foc1.data.iPhaseB;
      foc1.prot.currPhW = foc1.data.iPhaseC;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // call datalogger
      if (foc1.data.iqRef != dataLog.trackedValue || foc1.prot.protFlag != 0)
        dataLog.trigger = 1;
      dataLog.trackedValue = foc1.data.iqRef;
      dataLog.in1 = foc1.data.isa;
      dataLog.in2 = foc1.data.isb;
      dataLog.in3 = foc1.data.vAlpha;
      dataLog.in4 = foc1.data.vBeta;
      datalogCalcFast(&dataLog);
      break;

    default:
      break;
    }
    break;

  case RUN_CURRENT:
    switch (foc1.config.mode)
    {
    case DC:
      /* code */
      break;
    case BLDC:
      /* code */
      break;
    case FOC:
      // calc sin/cos
      foc_sincos(foc1.data.angle, &foc1.data.sinTheta, &foc1.data.cosTheta);
      // calc current controller
      ramp_calc(&foc1.pi_iq.Ref, foc1.data.iqRef, foc1.data.iqRmp, foc1.data.tS);
      foc1.pi_id.Ref = foc1.data.idRef;
      foc1.pi_iq.Ref = (fabsf(foc1.data.iqRef) > 0.1f) ? foc1.pi_iq.Ref : 0.f;
      Foc_update_cc(&foc1);
      // calc phase voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      // calc space-vector generator and update PWM
      foc1.svgen.udc = foc1.volt.DcBusVolt;
      foc1.calc.svgen(&foc1.svgen);
      foc_pwm_update(foc1.svgen.Ta, foc1.svgen.Tb, foc1.svgen.Tc, foc1.data.halfPwmPeriod);
      // estimete Hall sensor position
      Hall_read(&hall, 4);
      hall.speedE = foc1.pll.SpeedPll;
      Hall_update(&hall);
      foc1.data.hallAngle = hall.angleRaw;
      // calc flux observer
      Foc_update_angle_speed(&foc1);
      // Check protection
      foc1.prot.currPhU = foc1.data.iPhaseA;
      foc1.prot.currPhV = foc1.data.iPhaseB;
      foc1.prot.currPhW = foc1.data.iPhaseC;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // call datalogger
      if (foc1.data.iqRef != dataLog.trackedValue || foc1.prot.protFlag != 0)
        dataLog.trigger = 1;
      dataLog.trackedValue = foc1.data.iqRef;
      dataLog.in1 = foc1.data.isa;
      dataLog.in2 = foc1.data.isb;    //foc1.svgen.Ta;
      dataLog.in3 = foc1.data.vAlpha; //foc1.volt.usq;
      dataLog.in4 = foc1.data.vBeta;  //foc1.data.angle;
      datalogCalcFast(&dataLog);
      break;

    default:
      break;
    }
    break;

  case RUN_SPD:
    switch (foc1.config.mode)
    {
    case DC:
      /* code */
      break;
    case BLDC:
      /* code */
      break;
    case FOC:
      // calc sin/cos
      foc_sincos(foc1.data.angle, &foc1.data.sinTheta, &foc1.data.cosTheta);
      // calc speed controller
      ramp_s_calc(&foc1.pi_spd.Ref, foc1.data.speedRef, foc1.data.spdRmp, foc1.data.tS); // S-curve profile ramp
      //ramp_calc(&foc1.pi_spd.Ref, foc1.data.speedRef, foc1.data.spdRmp, foc1.data.tS); // trapezoidal profile ramp
      foc1.data.spdLoopCntr++;
      if (foc1.data.spdLoopCntr >= 10)
      {
        foc1.data.spdLoopCntr = 0;
        foc1.pi_spd.Fdb = (foc1.config.sim == 1) ? bldc1.omega_rpm : foc1.pll.SpeedPll;
        foc1.calc.pi_reg(&foc1.pi_spd);
      }
      // calc current controller
      foc1.pi_id.Ref = foc1.data.idRef;
      foc1.pi_iq.Ref = foc1.pi_spd.Out;
      Foc_update_cc(&foc1);
      // calc phase voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      // calc space-vector generator and update PWM
      foc1.svgen.udc = foc1.volt.DcBusVolt;
      foc1.calc.svgen(&foc1.svgen);
      foc_pwm_update(foc1.svgen.Ta, foc1.svgen.Tb, foc1.svgen.Tc, foc1.data.halfPwmPeriod);
      // estimete Hall sensor position
      Hall_read(&hall, 4);
      hall.speedE = foc1.pll.SpeedPll;
      Hall_update(&hall);
      // calc angle and speed
      foc1.data.hallAngle = hall.angleRaw;
      Foc_update_angle_speed(&foc1);
      // Check protection
      foc1.prot.currPhU = foc1.data.iPhaseA;
      foc1.prot.currPhV = foc1.data.iPhaseB;
      foc1.prot.currPhW = foc1.data.iPhaseC;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // call datalogger
      if (foc1.data.speedRef != dataLog.trackedValue || foc1.prot.protFlag != 0)
        dataLog.trigger = 1;
      dataLog.trackedValue = foc1.data.speedRef;
      dataLog.in1 = hall.angle;        //foc1.data.isd;
      dataLog.in2 = hall.angleRaw;     //foc1.data.isq;
      dataLog.in3 = foc1.pll.AnglePll; //foc1.lpf_id.out;
      dataLog.in4 = foc1.flux.phase;   //foc1.lpf_iq.out;
      datalogCalcFast(&dataLog);
      break;

    default:
      break;
    }
    break;

  case PARAM_ID_RL:
    // Angle generator
    angle_gen(&foc1.data.angle, foc1.data.freq, foc1.data.tS);
    // PARK transform for phase currents
    volatile float s = 0.f;
    volatile float c = 1.f;
    foc1.data.isd = foc1.data.isa * c + foc1.data.isb * s;
    foc1.data.isq = -foc1.data.isa * s + foc1.data.isb * c;
    // Park transform for D/Q voltages
    foc1.data.vd = foc1.data.vAlpha * c + foc1.data.vBeta * s;
    foc1.data.vq = -foc1.data.vAlpha * s + foc1.data.vBeta * c;
    // Filtered data
    foc1.lpf_id.in = foc1.data.isd;
    foc1.lpf_iq.in = foc1.data.isq;
    foc1.lpf_vdc.in = foc1.volt.DcBusVolt;
    LPF_calc(&foc1.lpf_id);
    LPF_calc(&foc1.lpf_iq);
    LPF_calc(&foc1.lpf_vdc);
    foc1.volt.DcBusVolt = (foc1.config.sim == 1) ? bldc1.udc : foc1.lpf_vdc.out;
    switch (foc1.paramIdState)
    {
    case ID_ENTER:
      foc1.data.isrCntr2 = 0;
      foc1.data.idRef = foc1.config.currentMaxPos * 0.2f; // measuring current is 20% of max
      foc1.data.id_IdAmpl = 0.0f;
      foc1.data.id_UdAmpl = 0.0f;
      foc1.data.freq = foc1.config.pwmFreq * 0.1f;
      foc1.pi_id.Kp = foc1.pi_iq.Kp = 0.001f;
      foc1.pi_id.Ki = foc1.pi_iq.Ki = 0.1f;
      foc1.pi_id.Kc = foc1.pi_iq.Kc = 0.f;
      foc1.pi_id.Kd = foc1.pi_iq.Kc = 0.f;
      dataLog.trigger = 1;
      foc1.paramIdState = ID_RS;
      ledDelay = 0.05f;
      foc_sincos(0.f, &foc1.data.sinTheta, &foc1.data.cosTheta);
      break;
    case ID_RS:
      // calc PI current controllers for D/Q axis
      ramp_calc(&foc1.pi_id.Ref, foc1.data.idRef, foc1.data.idRef * 10.f, foc1.data.tS);
      foc1.pi_iq.Ref = 0.f;
      foc1.pi_id.Fdb = foc1.data.isd;
      foc1.pi_iq.Fdb = foc1.data.isq;
      foc1.pi_id.OutMax = foc1.volt.magMaxD;
      foc1.pi_id.OutMin = -foc1.pi_id.OutMax;
      foc1.pi_iq.OutMax = foc1.volt.magMaxD; // - foc1.data.uqDec;
      foc1.pi_iq.OutMin = -foc1.pi_iq.OutMax;
      foc1.calc.pi_reg(&foc1.pi_id);
      foc1.calc.pi_reg(&foc1.pi_iq);
      foc1.volt.usd = foc1.pi_id.Out;
      foc1.volt.usq = foc1.pi_iq.Out;
      // calc PI current controllers limits and decoupling
      foc1.volt.magMaxD = foc1.volt.dutyMax * foc1.volt.DcBusVolt * (2.f / 3.f);
      // Saturation
      utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.magMaxD);
      // calc phase voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      //calc IPARK transform (IPARK output -> SVGEN input)
      foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
      foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
      // calc space-vector generator
      foc1.svgen.udc = foc1.volt.DcBusVolt;
      foc1.calc.svgen(&foc1.svgen);
      // calc Rs
      if (foc1.data.isrCntr2 >= (uint32_t)(0.5f * foc1.data.smpFreq)) // wait 500 ms, until current is stable
      {
        // mosfetRds * Idc + deadtime * udc - produce additional voltage drop
        //float Vdt = (foc1.config.deadTime * 1e-9f) * foc1.config.pwmFreq * foc1.volt.DcBusVolt;
        //foc1.data.id_UdErr = foc1.config.Rds_on * fabsf(foc1.lpf_id.out) + Vdt;
        //foc1.data.id_UdErr = (foc1.config.sim == 0) ? foc1.data.id_UdErr : 0.f;
        //foc1.data.id_RsBank = ((foc1.volt.usd * 0.5f) - foc1.data.id_UdErr) / foc1.lpf_id.out;
        foc1.data.id_RsBank = foc1.data.vd / foc1.data.isd;
        UTILS_LP_FAST(foc1.data.id_Rs, foc1.data.id_RsBank, foc1.data.tS / (0.05f + foc1.data.tS));
        if (foc1.data.isrCntr2 >= (uint32_t)(1.f * foc1.data.smpFreq))
        {
          foc1.pi_id.Ref = 0.f;
          foc1.data.isrCntr2 = 0;
          foc1.svgen.Ta = foc1.svgen.Tb = foc1.svgen.Tc = 0.f;
          foc1.paramIdState = ID_LD;
          if (foc1.lpf_id.out < 0.1f)
          {
            foc1.prot.fltCode = FLT_ID_NC;
            foc1.prot.protFlag = 1;
            foc1.data.id_Rs = -1.f;
            foc1.data.isrCntr2 = 0;
          }
        }
        dataLog.trigger = 1;
      }
      break;
    case ID_LD:
      if (foc1.flags.inductanceRatio == 0)
      {
        foc1.data.udRef = foc1.data.idRef * foc1.data.id_Rs;
      }
      else
      {
        foc1.data.udRef = foc1.data.idRef * foc1.data.id_Zs;
      }
      // calc UdRef
      foc_sincos(foc1.data.angle, &foc1.data.sinTheta, &foc1.data.cosTheta);
      foc1.data.id_Ud = (foc1.data.isrCntr2 >= (uint32_t)(0.5f * foc1.data.smpFreq)) ? foc1.data.sinTheta * foc1.data.udRef : 0.f;
      foc1.data.id_Uq = 0.f;
      // Saturation
      foc1.volt.usd = foc1.data.id_Ud;
      foc1.volt.usq = foc1.data.id_Uq;
      foc1.volt.magMaxD = foc1.volt.dutyMax * (2.f / 3.f) * foc1.volt.DcBusVolt;
      // Saturation
      utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.magMaxD);
      /* Check errors */
      if (foc1.data.udRef > foc1.volt.magMaxD)
      {
        foc1.data.id_Ld = -1.f;
        foc1.data.id_Lq = -1.f;
        foc1.prot.protFlag = 1;
        foc1.prot.fltCode = FLT_ID_OUT_MAG;
      }
      // calc voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      //calc IPARK transform (IPARK output -> SVGEN input)
      foc1.svgen.usa = foc1.volt.usd * c - foc1.volt.usq * s;
      foc1.svgen.usb = foc1.volt.usq * c + foc1.volt.usd * s;
      // calc space-vector generator
      foc1.svgen.udc = foc1.volt.DcBusVolt;
      foc1.calc.svgen(&foc1.svgen);
      // identify Ls
      if (foc1.data.isrCntr2 >= (uint32_t)(1.f * foc1.data.smpFreq))
      {
        //UTILS_LP_FAST(foc1.data.id_UdAmpl, fabsf(foc1.volt.usd * 0.5f), foc1.data.tS / (0.02f + foc1.data.tS));
        UTILS_LP_FAST(foc1.data.id_UdAmpl, fabsf(foc1.data.vd), foc1.data.tS / (0.05f + foc1.data.tS));
        UTILS_LP_FAST(foc1.data.id_IdAmpl, fabsf(foc1.data.isd), foc1.data.tS / (0.05f + foc1.data.tS));
        UTILS_NAN_ZERO(foc1.data.id_UdAmpl);
        UTILS_NAN_ZERO(foc1.data.id_IdAmpl);
        foc1.data.id_uMag = (foc1.data.id_UdAmpl > foc1.data.id_uMag) ? foc1.data.id_UdAmpl : foc1.data.id_uMag;
        foc1.data.id_iMag = (foc1.data.id_IdAmpl > foc1.data.id_iMag) ? foc1.data.id_IdAmpl : foc1.data.id_iMag;
        dataLog.trigger = 1;
        foc1.data.id_Zs = foc1.data.id_uMag / foc1.data.id_iMag;            // Z = Ud / Id
        foc1.data.id_Xs = sqrtf(SQ(foc1.data.id_Zs) - SQ(foc1.data.id_Rs)); // XL = sqrt(Z^2 - R^2)
        foc1.data.id_Ls = foc1.data.id_Xs / (M_2PI * foc1.data.freq);       // L = XL / (2*pi*f)
        foc1.data.id_Ld = foc1.data.id_Ls;
        UTILS_NAN_ZERO(foc1.data.id_Ld);
      }
      if (foc1.data.isrCntr2 > (uint32_t)(1.5f * foc1.data.smpFreq))
      {
        if (foc1.flags.inductanceRatio == 0)
        {
          foc1.paramIdState = ID_LD;
          foc1.flags.inductanceRatio = 1;
          foc1.data.isrCntr2 = 0;
          foc1.data.id_UdAmpl = 0.f;
          foc1.data.id_IdAmpl = 0.f;
          foc1.data.id_uMag = 0.f;
          foc1.data.id_iMag = 0.f;
        }
        else
        {
          foc1.paramIdState = ID_LQ;
          foc1.flags.inductanceRatio = 0;
          foc1.data.isrCntr2 = 0;
          foc1.data.id_UdAmpl = 0.f;
          foc1.data.id_IdAmpl = 0.f;
          foc1.data.id_uMag = 0.f;
          foc1.data.id_iMag = 0.f;
        }
      }
      break;
    case ID_LQ:
      foc1.data.udRef = foc1.data.idRef * foc1.data.id_Zs;
      // calc UdRef
      foc_sincos(foc1.data.angle, &foc1.data.sinTheta, &foc1.data.cosTheta);
      foc1.data.id_Ud = 0.f;
      foc1.data.id_Uq = (foc1.data.isrCntr2 >= (uint32_t)(0.5f * foc1.data.smpFreq)) ? foc1.data.sinTheta * foc1.data.udRef : 0.f;
      // Saturation
      foc1.volt.usd = foc1.data.id_Ud;
      foc1.volt.usq = foc1.data.id_Uq;
      foc1.volt.magMaxD = foc1.volt.dutyMax * (2.f / 3.f) * foc1.volt.DcBusVolt;
      // Saturation
      utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.magMaxD);
      /* Check errors */
      if (foc1.data.udRef > foc1.volt.magMaxD)
      {
        foc1.data.id_Ld = -1.f;
        foc1.data.id_Lq = -1.f;
        foc1.prot.protFlag = 1;
        foc1.prot.fltCode = FLT_ID_OUT_MAG;
      }
      // calc voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      //calc IPARK transform (IPARK output -> SVGEN input)
      foc1.svgen.usa = foc1.volt.usd * c - foc1.volt.usq * s;
      foc1.svgen.usb = foc1.volt.usq * c + foc1.volt.usd * s;
      // calc space-vector generator
      foc1.svgen.udc = foc1.volt.DcBusVolt;
      foc1.calc.svgen(&foc1.svgen);
      // identify Ls
      if (foc1.data.isrCntr2 >= (uint32_t)(1.f * foc1.data.smpFreq))
      {
        //UTILS_LP_FAST(foc1.data.id_UdAmpl, fabsf(foc1.data.id_Uq), 0.0005f);
        UTILS_LP_FAST(foc1.data.id_UdAmpl, fabsf(foc1.data.vq), foc1.data.tS / (0.05f + foc1.data.tS));
        UTILS_LP_FAST(foc1.data.id_IdAmpl, fabsf(foc1.data.isq), foc1.data.tS / (0.05f + foc1.data.tS));
        if (foc1.data.isrCntr2 >= (uint32_t)(1.4f * foc1.config.pwmFreq * foc1.data.adcPostScaler))
        {
          foc1.data.id_uMag = (foc1.data.id_UdAmpl > foc1.data.id_uMag) ? foc1.data.id_UdAmpl : foc1.data.id_uMag;
          foc1.data.id_iMag = (foc1.data.id_IdAmpl > foc1.data.id_iMag) ? foc1.data.id_IdAmpl : foc1.data.id_iMag;
        }
        // mosfetRds * Idc + deadtime * udc - produce additional voltage drop
        // float Vdt = (foc1.config.deadTime * 1e-9f) * foc1.config.pwmFreq * foc1.volt.DcBusVolt;
        // foc1.data.id_UdErr = foc1.config.Rds_on * fabsf(foc1.lpf_iq.out) + Vdt;
        // foc1.data.id_UdErr = (foc1.config.sim == 0) ? foc1.data.id_UdErr : 0.f;
        dataLog.trigger = 1;
        //foc1.data.id_Zs = (foc1.data.id_uMag - foc1.data.id_UdErr) / foc1.data.id_iMag;                 // Z = Ud / Id
        foc1.data.id_Zs = foc1.data.id_uMag / foc1.data.id_iMag;                                        // Z = Ud / Id
        foc1.data.id_Xs = sqrtf(foc1.data.id_Zs * foc1.data.id_Zs - foc1.data.id_Rs * foc1.data.id_Rs); // XL = sqrt(Z^2 - R^2)
        foc1.data.id_Ls = foc1.data.id_Xs / (M_2PI * foc1.data.freq);                                   // L = XL / (2*pi*f)
        foc1.data.id_Lq = foc1.data.id_Ls;
      }
      foc1.paramIdState = (foc1.data.isrCntr2 >= (uint32_t)(1.5f * foc1.data.smpFreq)) ? ID_CMPLT : ID_LQ;
      break;
    case ID_CMPLT:
      /* Clear PWM signals, calc DQ-cureents PI gains, calc observer gains */
      foc1.svgen.Ta = 0.0f;
      foc1.svgen.Tb = 0.0f;
      foc1.svgen.Tc = 0.0f;
      foc1.data.idRef = 0.f;
      foc1.data.freq = 0.0f;
      foc1.data.freqRef = 0.0f;
      foc1.data.freqRmp = 0.0f;
      foc1.data.udRef = 0.f;
      foc1.data.isrCntr2 = 0;
      // calc D-Q axis current PI controllers gains
      float wcc = (0.05f * (1.f / foc1.data.tS)) * M_2PI;
      foc1.pi_id.Kp = foc1.pi_iq.Kp = (foc1.data.id_Ld * wcc);
      foc1.pi_id.Ki = foc1.pi_iq.Ki = foc1.data.id_Rs * wcc * foc1.data.tS;
      foc1.pi_id.Kc = 1.f / foc1.pi_id.Kp;
      foc1.pi_iq.Kc = 1.f / foc1.pi_iq.Kp;
      // observer gains calc
      foc1.smo.Fsmopos = expf((-foc1.data.id_Rs / foc1.data.id_Ld) * foc1.data.tS);
      foc1.smo.Gsmopos = (1.0f / foc1.data.id_Rs) * (1.0f - foc1.smo.Fsmopos);
      foc1.flux.R = foc1.data.id_Rs * 1.5f;
      foc1.flux.L = foc1.data.id_Ld * 1.5f;
      foc1.config.Rs = foc1.data.id_Rs;
      foc1.config.Ld = foc1.data.id_Ld;
      foc1.config.Lq = foc1.data.id_Lq;
      // go to IDLE state
      foc1.driveState = STOP;
      break;

    default:
      break;
    }
    // update PWM
    foc_pwm_update(foc1.svgen.Ta, foc1.svgen.Tb, foc1.svgen.Tc, foc1.data.halfPwmPeriod);
    // Check protection
    foc1.prot.currPhU = foc1.data.iPhaseA;
    foc1.prot.currPhV = foc1.data.iPhaseB;
    foc1.prot.currPhW = foc1.data.iPhaseC;
    foc1.prot.udc = foc1.volt.DcBusVolt;
    foc1.prot.tempPcb = ntcPcb.temp;
    foc1.calc.prot(&foc1.prot);
    foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
    // calc BLDC motor model
    if (foc1.config.sim == 1)
    {
      bldc1.cmpr0 = foc1.svgen.Ta;
      bldc1.cmpr1 = foc1.svgen.Tb;
      bldc1.cmpr2 = foc1.svgen.Tc;
      ModelBLDC_Calc(&bldc1);
    }
    // call datalogger
    dataLog.in1 = foc1.paramIdState;
    dataLog.in2 = foc1.data.vd;
    dataLog.in3 = foc1.data.isd;
    dataLog.in4 = foc1.data.angle;
    datalogCalcFast(&dataLog);
    break;
  case PARAM_ID_RUN:
    /* This case used for detect Hall sensors table and offset */
    if (foc1.paramIdRunState == ID_RUN_HALL_ENTER)
    {
      foc1.data.idRef = foc1.config.currentMaxPos * 0.2f; // measuring current is 20% of max
      foc1.data.freqRmp = 0.2f;
      foc1.data.freqRef = 0.2f;
      foc1.data.freq = 0.2f;
      foc1.data.angle = 0.f;
      foc1.data.iqRmp = 20.f;
      angleIncDelay++;
      foc1.paramIdRunState = (angleIncDelay >= (uint32_t)(2.f * foc1.data.smpFreq)) ? ID_RUN_HALL_FWD : foc1.paramIdRunState;
    }
    // Angle generator
    if (foc1.paramIdRunState == ID_RUN_HALL_FWD || foc1.paramIdRunState == ID_RUN_HALL_RVS || foc1.paramIdRunState == ID_RUN_FLUX_OL)
    {
      if (foc1.paramIdRunState == ID_RUN_FLUX_OL)
        ramp_calc(&foc1.data.freq, foc1.data.freqRef, foc1.data.freqRmp, foc1.data.tS);

      foc1.data.angle += foc1.data.freqStep * foc1.data.freq;
      if (foc1.data.angle < -MF_PI)
        foc1.data.angle = foc1.data.angle + M_2PI;
      else if (foc1.data.angle > MF_PI)
        foc1.data.angle = foc1.data.angle - M_2PI;
    }
    // Angle generator for cogging map
    if (foc1.paramIdRunState == ID_RUN_COG_FORCE)
    {
      foc1.data.idRef = foc1.config.currentMaxPos * 0.2f; // measuring current is 20% of max
      foc1.data.angle = 0.f;
      UTILS_LP_FAST(foc1.cgtc.udZero, foc1.pi_id.Out, 0.005f);
      angleIncDelay++;
      if (angleIncDelay >= (uint32_t)(2.0f * foc1.data.smpFreq))
      {
        foc1.paramIdRunState = ID_RUN_COG_CALC;
        angleIncDelay = 0;
        foc1.cgtc.tabNmbr = 0;
      }
    }
    if (foc1.paramIdRunState == ID_RUN_COG_CALC)
    {
      angleIncDelay++;
      if (angleIncDelay >= (uint32_t)(0.05f * foc1.data.smpFreq))
      {
        foc1.data.angle += (M_2PI / 100.f);
        if (foc1.data.angle < -MF_PI)
          foc1.data.angle = foc1.data.angle + M_2PI;
        else if (foc1.data.angle > MF_PI)
          foc1.data.angle = foc1.data.angle - M_2PI;

        angleIncDelay = 0;
        foc1.cgtc.tabNmbr++;
        dataLog.trigger = 1;
      }
    }
    // calc sin/cos
    foc_sincos(foc1.data.angle, &foc1.data.sinTheta, &foc1.data.cosTheta);
    // Filtered data
    foc1.lpf_id.in = foc1.data.isd;
    foc1.lpf_iq.in = foc1.data.isq;
    foc1.lpf_vdc.in = foc1.volt.DcBusVolt;
    LPF_calc(&foc1.lpf_id);
    LPF_calc(&foc1.lpf_iq);
    LPF_calc(&foc1.lpf_vdc);
    foc1.volt.DcBusVolt = foc1.lpf_vdc.out;
    // PARK transform for phase currents
    foc1.data.isd = foc1.data.isa * foc1.data.cosTheta + foc1.data.isb * foc1.data.sinTheta;
    foc1.data.isq = -foc1.data.isa * foc1.data.sinTheta + foc1.data.isb * foc1.data.cosTheta;
    // calc PI current controllers for D/Q axis
    ramp_calc(&foc1.pi_id.Ref, foc1.data.idRef, foc1.data.iqRmp, foc1.data.tS);
    //foc1.pi_id.Ref = foc1.data.idRef;
    foc1.pi_iq.Ref = foc1.data.iqRef;
    foc1.pi_id.Fdb = foc1.data.isd;
    foc1.pi_iq.Fdb = foc1.data.isq;
    foc1.calc.pi_reg(&foc1.pi_id);
    foc1.calc.pi_reg(&foc1.pi_iq);

    foc1.volt.usd = foc1.pi_id.Out;
    foc1.volt.usq = foc1.pi_iq.Out;
    UTILS_LP_FAST(foc1.cgtc.udFltrd, foc1.volt.usd, 0.001f);
    // Saturation
    foc1.volt.magMaxD = foc1.volt.dutyMax * SQRT3_BY_2 * foc1.volt.DcBusVolt;
    foc1.volt.magMaxQ = sqrtf(SQ(foc1.volt.magMaxD) - SQ(foc1.volt.usd));
    utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.magMaxD);
    // calc phase voltages
    foc1.volt.MfuncV1 = foc1.svgen.Ta;
    foc1.volt.MfuncV2 = foc1.svgen.Tb;
    foc1.volt.MfuncV3 = foc1.svgen.Tc;
    foc1.calc.volt(&foc1.volt);
    //calc IPARK transform (IPARK output -> SVGEN input)
    foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
    foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
    // calc space-vector generator (1/Vdc) * sqrt(3)
    foc1.svgen.udc = foc1.volt.DcBusVolt;
    foc1.calc.svgen(&foc1.svgen);
    // estimete Hall sensor position
    Hall_read(&hall, 4);
    // estimete Hall sensor position offset
    if (hall.A == 0 && hall.B == 1 && hall.C == 0)
      hall.state = 0; // 0(360) deg. 2PI
    else if (hall.A == 0 && hall.B == 1 && hall.C == 1)
      hall.state = 1; // 60 deg.  2PI/3 / 2
    else if (hall.A == 0 && hall.B == 0 && hall.C == 1)
      hall.state = 2; // 120 deg. 2PI/3
    else if (hall.A == 1 && hall.B == 0 && hall.C == 1)
      hall.state = 3; // 180 deg. PI
    else if (hall.A == 1 && hall.B == 0 && hall.C == 0)
      hall.state = 4; // 240 deg. PI - (2PI/3 * 2)
    else if (hall.A == 1 && hall.B == 1 && hall.C == 0)
      hall.state = 5; // 300 deg. PI - 2PI/3
    // detect direction
    if (foc1.flags.directionDetect == 0)
    {
      if (hall.state > hall.statePr && hall.state != 5)
      {
        hall.dir = 0;
        foc1.flags.directionDetect = 1;
      }
      if (hall.state < hall.statePr && hall.state != 0)
      {
        hall.dir = 1;
        foc1.flags.directionDetect = 1;
      }
    }
    // Calc Angle offset
    if (hall.state != hall.statePr)
    {
      if (hall.dir == 0)
      {
        hall.offsetFwd[hall.state] = (foc1.paramIdRunState == ID_RUN_HALL_FWD) ? foc1.data.angle : hall.offsetFwd[hall.state];
        hall.offsetRev[hall.state] = (foc1.paramIdRunState == ID_RUN_HALL_RVS) ? foc1.data.angle : hall.offsetRev[hall.state];
      }
      if (hall.dir == 1)
      {
        hall.offsetRev[hall.state] = (foc1.paramIdRunState == ID_RUN_HALL_FWD) ? foc1.data.angle : hall.offsetRev[hall.state];
        hall.offsetFwd[hall.state] = (foc1.paramIdRunState == ID_RUN_HALL_RVS) ? foc1.data.angle : hall.offsetFwd[hall.state];
      }
      hall.offset = foc1.data.angle;
      hall.offsetFlag = 1;
    }
    hall.statePr = hall.state;
    // calc avg
    hall.angleInc += foc1.data.freqStep * foc1.data.freq;
    if (hall.angleInc > (M_2PI * 0.999f) && foc1.paramIdRunState == ID_RUN_HALL_FWD)
    {
      foc1.data.freq = -foc1.data.freq;
      hall.angleInc = M_2PI;
      foc1.paramIdRunState = ID_RUN_HALL_RVS;
    }
    if (hall.angleInc < (0.001f) && foc1.paramIdRunState == ID_RUN_HALL_RVS)
    {
      foc1.data.freq = 0;
      foc1.paramIdRunState = ID_RUN_HALL_CMPLT;
    }
    if (foc1.paramIdRunState == ID_RUN_HALL_CMPLT)
    {
      for (uint8_t i = 0; i < 6; i++)
      {
        hall.offsetAvg[i] = utils_avg_angle_rad_fast(hall.offsetFwd[i], hall.offsetRev[i]);
        if (hall.offsetAvg[i] < -MF_PI)
          hall.offsetAvg[i] = hall.offsetAvg[i] + M_2PI;
        else if (hall.offsetAvg[i] > MF_PI)
          hall.offsetAvg[i] = hall.offsetAvg[i] - M_2PI;
      }
      /* Reset and end */
      foc1.data.idRef = 0.f;
      foc1.pi_id.Ref = 0.f;
      foc1.data.iqRef = 0.f;
      foc1.data.freqRef = 0.f;
      foc1.data.freq = 0.f;
      //foc1.driveState = STOP;
      angleIncDelay = 0;
      foc1.paramIdRunState = ID_RUN_COG_FORCE;
    }
    if (foc1.paramIdRunState == ID_RUN_COG_CALC)
    {
      foc1.cgtc.coggingMap[foc1.cgtc.tabNmbr] = -((foc1.cgtc.udFltrd - foc1.cgtc.udZero) / foc1.data.idRef);
      if (foc1.cgtc.tabNmbr >= (100 - 1))
      {
        foc1.pi_id.Ref = 0.f;
        foc1.data.freq = 0.f;
        foc1.data.idRef = foc1.config.currentMaxPos * 0.25f; // measuring speed is 20% of max
        foc1.data.iqRmp = 20.f;
        foc1.data.freqRef = (foc1.config.speedMax * 0.1f) / M_2PI; // measuring speed is 10% of max
        foc1.data.freqRmp = 10.f;
        foc1.flags.fluxDetectedOpenLoop = 0;
        angleIncDelay = 0;
        foc1.paramIdRunState = ID_RUN_FLUX_OL;
      }
    }
    if (foc1.paramIdRunState == ID_RUN_FLUX_OL)
    {
      if (angleIncDelay < (uint32_t)(1.f * foc1.data.smpFreq))
      {
        angleIncDelay++;
        foc1.data.freqRef = 0.f;
        foc1.data.freq = 0.f;
      }
      if (angleIncDelay >= (uint32_t)(1.f * foc1.data.smpFreq))
      {
        foc1.data.freqRef = (foc1.config.speedMax * 0.1f) / M_2PI; // measuring speed is 10% of max
      }
      if (foc1.data.freq > (foc1.data.freqRef * 0.95f))
      {
        angleIncDelay++;
        /* Calc Kv. Note: return value has to be divided by half the number of motor poles */
        if (foc1.flags.fluxDetectedOpenLoop == 0)
        {
          /* Calc duty */
          foc1.volt.dutyD = (foc1.volt.usd - (foc1.lpf_id.out * foc1.config.Rs)) / (foc1.volt.DcBusVolt * TWO_BY_SQRT3);
          foc1.volt.dutyQ = (foc1.volt.usq - (foc1.lpf_iq.out * foc1.config.Rs)) / (foc1.volt.DcBusVolt * TWO_BY_SQRT3);
          foc1.data.duty = sqrtf(SQ(foc1.volt.dutyD) + SQ(foc1.volt.dutyQ));
          foc1.pll.SpeedPll = foc1.data.freq * M_2PI;
          foc1.data.speedRpm = (foc1.pll.SpeedPll * RADS2RPM) / foc1.config.pp;
          volatile float ol_kv_raw = (foc1.data.speedRpm / (foc1.data.duty * foc1.volt.DcBusVolt)) / (foc1.config.pp / 2.f);
          UTILS_LP_FAST(foc1.config.Kv, ol_kv_raw, foc1.data.tS / (0.1f + foc1.data.tS));
          UTILS_NAN_ZERO(foc1.config.Kv);
        }
        if (angleIncDelay >= (uint32_t)(3.f * foc1.data.smpFreq) && foc1.flags.fluxDetectedOpenLoop == 0)
        {
          foc1.flux.fluxLinkage = 60.f / (SQRT3 * M_2PI * foc1.config.Kv * foc1.config.pp);
          foc1.flux.gamma = (1000.f / (SQ(foc1.flux.fluxLinkage))) * 0.5f;
          foc1.data.freqRmp = foc1.data.freqRef;
          foc1.data.iqRmp = foc1.data.idRef;
          foc1.data.freqRef = 0.f;
          foc1.data.idRef = 0.f;
          foc1.flags.fluxDetectedOpenLoop = 1;
          angleIncDelay = 0;
        }
      }
      if (foc1.flags.fluxDetectedOpenLoop == 1)
      {
        angleIncDelay = 0;
        foc1.data.freq = 0.f;
        foc1.data.idRef = 0.f;
        foc1.data.freqRef = 0.f;
        foc1.data.freqRmp = 10.f;
        foc1.data.speedRef = 0.3f * foc1.config.speedMax; // measuring speed is 30% of max
        foc1.data.spdRmp = 300.f;
        foc1.flags.fluxDetectedClosedLoop = 0;
        foc1.config.decMode = DEC_BEMF;
        foc1.config.sensorType = SENSORLESS;
        foc1.paramIdRunState = ID_RUN_FLUX_CL;
        foc1.driveState = RUN_SPD;
      }
    }
    // calc speed PLL
    foc1.pll.AngleRaw = foc1.flux.phase;
    foc1.calc.pll(&foc1.pll);
    // Check protection
    foc1.prot.currPhU = foc1.data.isa;
    foc1.prot.currPhV = foc1.data.isb;
    foc1.prot.udc = foc1.volt.DcBusVolt;
    foc1.prot.tempPcb = ntcPcb.temp;
    foc1.calc.prot(&foc1.prot);
    foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
    // apply voltage to motor and pwm driver
    foc_pwm_update(foc1.svgen.Ta, foc1.svgen.Tb, foc1.svgen.Tc, foc1.data.halfPwmPeriod);
    // calc BLDC motor model

    // call datalogger
    if (foc1.data.idRef != 0 || foc1.data.iqRef != 0 || foc1.prot.protFlag != 0)
      dataLog.trigger = 1;
    dataLog.in1 = foc1.cgtc.tabNmbr;
    dataLog.in2 = foc1.cgtc.coggingMap[foc1.cgtc.tabNmbr];
    dataLog.in3 = foc1.cgtc.udZero;
    dataLog.in4 = foc1.cgtc.udFltrd;
    datalogCalcFast(&dataLog);
    break;

  case FAULT:
    // Disable PWM and blink fault LED
    foc_pwm_off();
    // Enable fault LED
    LL_GPIO_SetOutputPin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
    // Record fault conditions
    switch (foc1.prot.fltCode)
    {
    case FLT_OCP_U:
      dataLog.in1 = adcData.ph_u;
      dataLog.in2 = foc1.data.offsetCurrA;
      dataLog.in3 = foc1.data.isa;
      dataLog.in4 = foc1.svgen.Ta;
      break;
    case FLT_OCP_V:
      dataLog.in1 = adcData.ph_v;
      dataLog.in2 = foc1.data.offsetCurrB;
      dataLog.in3 = foc1.data.isb;
      dataLog.in4 = foc1.svgen.Tb;
      break;
    case FLT_OVP:
      dataLog.in1 = foc1.svgen.usa;
      dataLog.in2 = foc1.svgen.usb;
      dataLog.in3 = foc1.volt.DcBusVolt;
      dataLog.in4 = foc1.lpf_Iavg.out;
      break;
    case FLT_OVT:
      dataLog.in1 = foc1.data.duty;
      dataLog.in2 = ntcPcb.temp;
      dataLog.in3 = foc1.volt.DcBusVolt;
      dataLog.in4 = foc1.lpf_Iavg.out;
      break;
    case FLT_UVLO:
      dataLog.in1 = foc1.svgen.usa;
      dataLog.in2 = foc1.svgen.usb;
      dataLog.in3 = foc1.volt.DcBusVolt;
      dataLog.in4 = foc1.lpf_Iavg.out;
      break;
    default:
      break;
    }
    datalogCalcFast(&dataLog);
    break;

  default:
    break;
  }

  stop_cc = DWT->CYCCNT;          /* hold CPU cycles counter */
  execTime1 = stop_cc - start_cc; /* calc ISR execution time */
}

/* ISR for 1ms calc */
void slowCalc(void)
{
  /* Set hardware current limiting */
  foc1.data.hwCurrLim = (foc1.data.hwCurrLim > foc1.config.adcFullScaleCurrent) ? foc1.config.adcFullScaleCurrent : foc1.data.hwCurrLim;
  foc1.data.hwCurrLim = (foc1.data.hwCurrLim < 0) ? 0 : foc1.data.hwCurrLim;
  dac_cmpr_update((foc1.data.hwCurrLim / foc1.config.adcFullScaleCurrent), 2048.f);
  dac_value = (uint32_t)((foc1.data.hwCurrLim / foc1.config.adcFullScaleCurrent) * 2048 + 2048) - 1;

  /* Total current */
  foc1.data.iAvg = sqrtf(SQ(foc1.lpf_id.out) + SQ(foc1.lpf_iq.out));
  UTILS_NAN_ZERO(foc1.data.iAvg);

  /* Torque equation. Magnetic, reluctance, on shaft */
  foc1.data.Tmag = 1.5f * foc1.config.pp * (foc1.flux.fluxLinkage * foc1.lpf_iq.out);
  foc1.data.Trel = 1.5f * foc1.config.pp * (foc1.config.Lq - foc1.config.Ld) * foc1.lpf_id.out * foc1.lpf_iq.out;
  foc1.data.Te = foc1.data.Tmag + foc1.data.Trel;

  /* Input power. 2/sqrt(3) its SVPWM scale factor */
  float Pin_raw = 1.5f * TWO_BY_SQRT3 * (foc1.volt.usd * foc1.lpf_id.out + foc1.volt.usq * foc1.lpf_iq.out);
  //float Pin_raw = 1.5f * TWO_BY_SQRT3 * foc1.data.bldc_duty * foc1.volt.DcBusVolt * foc1.data.iAvg;
  UTILS_LP_FAST(foc1.data.Pin, Pin_raw, 0.01f);
  UTILS_NAN_ZERO(foc1.data.Pin);

  /* Electrical resistive power losses */
  foc1.data.Pe_loss = 1.5f * foc1.config.Rs * SQ(foc1.data.iAvg);

  /* Electromechanical power filtered */
  foc1.lpf_Pem.in = foc1.data.Te * (foc1.pll.SpeedPll / foc1.config.pp);
  LPF_calc(&foc1.lpf_Pem);
  foc1.data.Pem = foc1.lpf_Pem.out;

  /* Efficiency calculation */
  float eff_raw = foc1.data.Pem / foc1.data.Pin;
  UTILS_LP_FAST(foc1.data.efficiency, eff_raw, 0.01f);
  UTILS_NAN_ZERO(foc1.data.efficiency);

  /* Kv(FluxLinkage) observer */
  if (foc1.paramIdRunState != ID_RUN_FLUX_OL && foc1.paramIdRunState != ID_RUN_FLUX_CL && foc1.driveState != STOP)
  {
    /* Detect steady state no load conditions */
    if (foc1.pll.SpeedAbs > 300.f && foc1.data.iAvg < 2.f && fabsf(foc1.pi_iq.Ref) > 0.f)
    {
      /* Calc duty */
      foc1.volt.dutyD = (foc1.volt.usd - (foc1.lpf_id.out * foc1.config.Rs)) / (foc1.volt.DcBusVolt * TWO_BY_SQRT3);
      foc1.volt.dutyQ = (foc1.volt.usq - (foc1.lpf_iq.out * foc1.config.Rs)) / (foc1.volt.DcBusVolt * TWO_BY_SQRT3);
      foc1.data.duty = (foc1.driveState != RUN_DUTY) ? MAGF(foc1.volt.dutyD, foc1.volt.dutyQ) : foc1.data.duty;
      float kv_obs_raw = (foc1.data.speedRpm / (foc1.data.duty * foc1.volt.DcBusVolt)) / (foc1.config.pp / 2.f);
      //foc1.flux.fluxLinkage = 60.f / (SQRT3 * M_2PI * foc1.config.Kv * foc1.config.pp);
      UTILS_LP_FAST(foc1.config.Kv, kv_obs_raw, 0.01f);
      UTILS_NAN_ZERO(foc1.config.Kv);
    }
  }

  /* Flux detect closed loop case */
  if (foc1.paramIdRunState == ID_RUN_FLUX_CL && foc1.driveState == RUN_SPD)
  {
    if (foc1.pll.SpeedAbs > fabsf(foc1.data.speedRef * 0.95f) && foc1.flags.fluxDetectedClosedLoop == 0)
    {
      angleIncDelay++;
      /* Calc duty */
      foc1.volt.dutyD = (foc1.volt.usd - (foc1.lpf_id.out * foc1.config.Rs)) / (foc1.volt.DcBusVolt * TWO_BY_SQRT3);
      foc1.volt.dutyQ = (foc1.volt.usq - (foc1.lpf_iq.out * foc1.config.Rs)) / (foc1.volt.DcBusVolt * TWO_BY_SQRT3);
      foc1.data.duty = sqrtf(SQ(foc1.volt.dutyD) + SQ(foc1.volt.dutyQ));
      float kv_raw = (foc1.data.speedRpm / (foc1.data.duty * foc1.volt.DcBusVolt)) / (foc1.config.pp / 2.f);
      UTILS_LP_FAST(foc1.config.Kv, kv_raw, 0.01f);
      UTILS_NAN_ZERO(foc1.config.Kv);
      if (angleIncDelay >= (uint32_t)(2.0f * 1000.f))
      {
        foc1.flux.fluxLinkage = 60.f / (SQRT3 * M_2PI * foc1.config.Kv * foc1.config.pp);
        foc1.flux.gamma = (1000.f / (SQ(foc1.flux.fluxLinkage))) * 0.5f;
        angleIncDelay = 0;
        foc1.data.speedRef = 0.f;
        foc1.flags.fluxDetectedClosedLoop = 1;
      }
    }
    if (foc1.pll.SpeedAbs < 50.f && foc1.flags.fluxDetectedClosedLoop == 1)
    {
      foc1.data.iqRmp = 20.f;
      foc1.paramIdRunState = ID_RUN_CMPLT;
      foc1.driveState = STOP;
    }
  }

  /* board temperature calc */
  ntcPcb.u = ((float)adcData.pcb_temp * (1.f / 4096.f));
  ntc_temperature(&ntcPcb);
  foc1.lpf_NTC.in = ntcPcb.temp;
  LPF_calc(&foc1.lpf_NTC);
  ntcPcb.temp = (foc1.volt.DcBusVolt > 15.f) ? foc1.lpf_NTC.out : 0.f;

  /* LED blink control */
  ledDelay = (foc1.data.iAvg > 0.1f) ? 0.5f : ledDelay;
  ledDelay = (foc1.pll.SpeedAbs > 5.f) ? 0.25f : ledDelay;
  ledDelay = (foc1.driveState == PARAM_ID_RL || foc1.driveState == PARAM_ID_RUN) ? 0.05f : ledDelay;
  ledDelay = (foc1.driveState == RUN_SPD && foc1.paramIdRunState == ID_RUN_FLUX_CL) ? 0.05f : ledDelay;

  /* PWM input control */
  if (foc1.driveState == RUN_CURRENT && foc1.flags.pwmInputEn == 1)
  {
    float raw_pwmInput = (float)pwmInputValue * (foc1.config.currentMaxPos / 1024.f); // 40(A)/1024(PWM_MAX_VALUE)
    raw_pwmInput = (pwmInputValue < 25) ? 0 : raw_pwmInput;
    UTILS_LP_FAST(foc1.data.iqRef, raw_pwmInput, 0.01f);
  }
  if (foc1.driveState == RUN_SPD && foc1.flags.pwmInputEn == 1)
  {
    foc1.data.vMax = (foc1.volt.DcBusVolt * ONE_BY_SQRT3) - (foc1.data.iAvg * foc1.config.Rs);
    foc1.data.wBase = foc1.data.vMax / MAGF(foc1.config.Lq * foc1.lpf_iq.out, foc1.config.Ld * foc1.lpf_id.out + foc1.flux.fluxLinkage);
    float raw_pwmInput = (float)pwmInputValue * (foc1.data.wBase / 1024.f); // 0.0009765625 = 1 / 1024
    raw_pwmInput = (pwmInputValue < 25) ? 0 : raw_pwmInput;
    UTILS_LP_FAST(foc1.data.speedRef, raw_pwmInput, 0.01f);
  }
  /* CAN transmit section */
  static uint32_t isrCntrCAN;
  isrCntrCAN++;
  if (isrCntrCAN >= 50) // every 50 ms
  {
    isrCntrCAN = 0;
    uint8_t TxData[8];
    TxData[0] = (uint8_t)foc1.driveState;
    TxData[1] = (uint8_t)((foc1.volt.DcBusVolt / foc1.config.adcFullScaleVoltage) * 255.f);
    TxData[2] = (uint8_t)((foc1.data.iAvg / foc1.config.adcFullScaleCurrent) * 255.f);
    TxData[3] = (uint8_t)(foc1.data.speedRpm * 0.02f);
    TxData[4] = (uint8_t)(ntcPcb.temp);
    TxData[5] = (uint8_t)(foc1.data.Pem * 0.1f);
    TxData[6] = (uint8_t)(foc1.data.Te * 10.f);
    TxData[7] = (uint8_t)(foc1.data.duty * 255.f);
    can_transmit_massege(can.txh[0], 0x1F, TxData, 8, 0xCC);
    uint8_t *canDataP = (uint8_t *)&foc1.data.isrCntr1;
    can_transmit_massege(can.txh[0], 0x1A, canDataP, 4, 0xCC);
  }
  /* CAN receive section */
  can_receive_massege();
  if ((can.RxHeader.Identifier == 0x321) && (can.RxHeader.IdType == FDCAN_STANDARD_ID))
  {
    if (can.RxData[0] == 1)
    {
      ledDelay = (float)can.RxData[1] * (1.f / 256.f);
    }
    if (can.RxData[0] == 0)
    {
      LL_GPIO_ResetOutputPin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
      ledDelay = (float)can.RxData[1] * (1.f / 256.f);
    }
  }
}

/* ISR for PWM-input signal */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if (LL_TIM_IsActiveFlag_CC1(TIM2) == 1)
  {
    LL_TIM_ClearFlag_CC1(TIM2);
    /* Get the Input Capture value */
    pwmInputPeriod = LL_TIM_IC_GetCaptureCH2(TIM2);

    /* Duty cycle computation */
    pwmInputValue = (pwmInputPeriod * 1024) / LL_TIM_IC_GetCaptureCH1(TIM2);
  }
  else
  {
    pwmInputValue = 0;
  }
}

void Error_Handler(void)
{
}