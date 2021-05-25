/**
 * BLDC FOC
 *   Operation modes:
 * 1. DQ-axis current control
 * 2. Speed control
 * 3. Sensorless operation, sensored (Hall sensors signals extrapolation)
 * 4. Motor parameter identification (Rs,Ld,Lq, Hall sensors table)
 * 5. Open-loop modes (V/Hz, I/Hz)
 * 6. Sensorless 6-step bldc mode (zc + bemf integration)
 * 7. Simulation internal motor model
 * */

/**
 * TODO:
 * 1. Add CAN
 * 2. Save identified parameters to flash and restore after restart (TESTED: OK)
 * 3. Add PLL speed estimation (TESTED: OK)
 * 4. Add UART datalogger
 * 5. Blocking UVLO when DC-Link precharge not done
 * 6. add flux leakage estimation (need for observers and feedForward)
 * 
 * */

#include "main.h"
#include "adc.h"
#include "hrtim.h"
#include "lptim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sys.h"

void CoreInit(void);
void CoreStart(void);
void SystemClock_Config(void);

dataLogVars_t dataLog;       /* DataLogger struct */
adcData_t adcData;           /* ADC data struct */
ntc_t ntcPcb;                /* NTC temp sensing struct*/
modelBLDC_t bldc1;           /* BLDC motor mathematical model */
hall_t hall = HALL_DEFAULTS; /* Hall sensors module */
foc_t foc1 = FOC_DEFAULTS;   /* Field-oriented control algorithms */

volatile uint32_t start_cc;       /* Var. Data Watchpoint and Trace Unit */
volatile uint32_t stop_cc;        /* Var. Data Watchpoint and Trace Unit */
volatile uint32_t execTime0;      /* Var. Data Watchpoint and Trace Unit */
volatile uint32_t execTime1;      /* Var. Data Watchpoint and Trace Unit */
volatile float cpuLoad;           /* Processor load, % */
volatile uint8_t e = 0;           /* First enter flag */
volatile uint8_t systemReset = 0; /* MCU reset request */
volatile float ledDelay = 0;      /* Delay betwen toggle led, s */

volatile uint32_t pwmInputPeriod;
volatile uint32_t pwmInputValue;

int main(void)
{
  /* Configure the system clock, flash, interrupts and periph */
  CoreInit();
  /* Init BLDC model */
  ModelBLDC_Init(&bldc1);
  /* set FOC parameters */
  foc1.config.mode = FOC;                  // FOC or BLDC(6-step)
  foc1.config.sim = false;                 // 0 - real motor, 1 - motor model
  foc1.config.pwmFreq = 30000.f;           // set PWM and main(current) loop frequency
  foc1.config.adcFullScaleCurrent = 50.f;  // set current wich means 4095 ADC value: 1.65 / 0.002 / 16.5 = 50.0
  foc1.config.adcFullScaleVoltage = 69.3f; // set voltage wich means 4095 ADC value: 3.3 * 21 = 69.3
  foc1.config.deadTime = 180.0f;           // set deadTime, nS
  foc1.config.Rds_on = 0.0019f;            // set MOSFET Rds(on), Ohm (used for motor parameters identification)
  foc1.config.pp = 4.f;                    // set motor pole pairs
  foc1.config.Rs = 1.54f;                  // set motor phase resistance. Can be measured
  foc1.config.Ld = 0.002f;                 // set motor D-axis inductance. Can be measured
  foc1.config.Lq = foc1.config.Ld;         // set motor Q-axis inductance. Can be measured
  foc1.config.Kv = 560.f;                  // set motor Kv
  foc1.config.axisDecEn = true;            // 1 - enable feedForward compensation (dq axis decoupling), 0 - disable
  foc1.prot.enable = true;                 // 1 - enable protection, 0 - disable
  foc1.prot.ocpThld = 40.0f;               // set Over Current Protection threshold (peak p-p)
  foc1.prot.ovpThld = 65.0f;               // set Over Voltage Protection threshold (DC-link)
  foc1.prot.uvloThld = 9.99f;              // set Under Voltage LockOut threshold (DC-link)
  foc1.prot.pcbTempThld = 90.f;            // set PCB temperature threshold
  /* Initialize PCB temperature sensing module */
  ntcPcb.r_balance = 10000.f; // balance resistor value
  ntcPcb.r_ntc_0 = 4700.f;    // NTC base resistance
  ntcPcb.ta_0 = 25.f;         // NTC base temperature
  ntcPcb.betta = 3950.f;      // NTC beta coefficient
  /* Load parameters from flash */
  FLASH_LoadConfig(&foc1, &hall);
  /* Init FOC */
  Foc_Init(&foc1);
  /* Start hardware */
  CoreStart();

  while (1)
  {
    /* Check CPU reset request */
    if (systemReset != 0)
      NVIC_SystemReset();
    /* Check motor parameters identification complete */
    if (foc1.paramIdState == Cmplt && hall.offsetState == 2)
    {
      /* Save parameters into flash after identification */
      FLASH_UpdateConfig(&foc1, &hall);
    }
    /* calc CPU load, % */
    cpuLoad = ((float)execTime1 / (float)execTime0) * 100.f;
    /* board temperature calc */
    ntcPcb.u = ((float)adcData.pcb_temp * 0.0002442f); // 1/4095
    ntc_temperature(&ntcPcb);
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
  /* Insure 1�s transition state at intermediate medium speed clock based on DWT */
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
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_PCLK1);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_PLL);
  LL_RCC_SetADCClockSource(LL_RCC_ADC345_CLKSOURCE_PLL);
}

void CoreStart(void)
{
  /* Initialize all configured peripherals */
  if (foc1.config.mode == FOC)
  {
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_HRTIM1_Init();
    MX_USART2_UART_Init();
    MX_TIM7_Init();
    MX_TIM2_Init();
    hrtim_start();
    tim2_start();
    adc_start();
  }
  if (foc1.config.mode == BLDC)
  {
    MX_GPIO_Init();
    ADC1_Init_BLDC();
    ADC2_Init_BLDC();
    MX_HRTIM1_Init_BLDC();
    MX_USART2_UART_Init();
    MX_TIM7_Init();
    MX_TIM2_Init();
    hrtim_start();
    tim2_start();
    adc_start_bldc();
  }
}

void ADC1_2_IRQHandler(void)
{
  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_ClearFlag_JEOS(ADC2);
  /* compute time between ISR calls using cycle counter */
  if (e == 0)
  {
    e = 1;
    DWT->CYCCNT = 0;
    start_cc = DWT->CYCCNT;
  }
  else
  {
    e = 0;
    stop_cc = DWT->CYCCNT;
    execTime0 = stop_cc - start_cc;
  }
  /* heartbit */
  foc1.data.isrCntr0++;                                                 // ISR counter for toggle LED
  foc1.data.isrCntr1++;                                                 // ISR counter for bldc startUP
  foc1.data.isrCntr2++;                                                 // ISR counter for param identification case in FOC mode
  if (foc1.data.isrCntr0 >= (uint32_t)(ledDelay * foc1.config.pwmFreq)) // Status LED control
  {
    LL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    foc1.data.isrCntr0 = 0;
  }

  /*******************************************************************
     * 6-step / FOC section
     ******************************************************************/
  start_cc = DWT->CYCCNT; /* read CPU cycles counter */
  if (foc1.config.mode == BLDC)
  {
    /* take data from ADC */
    adcData.ph_v = ADC1->JDR1;
    adcData.ph_w = ADC1->JDR2;
    adcData.ph_u = ADC1->JDR3;
    adcData.v_dc = ADC1->JDR4;
    adcData.v_u = ADC2->JDR3;
    adcData.v_v = ADC2->JDR1;
    adcData.v_w = ADC2->JDR2;
    adcData.pcb_temp = ADC2->JDR4;

    switch (foc1.driveStateBLDC)
    {
    case STOP_BLDC:
      // Set LED_STATUS to turn on for indicate STOP case
      ledDelay = 0.01f;
      // Convert and scale ADC data
      foc1.cmtn.lpf_bemfA.in = (float)(adcData.v_u - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfB.in = (float)(adcData.v_v - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfC.in = (float)(adcData.v_w - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.data.offsetCurrA = (float)adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent; // Phase A curr.offset in P.U. (1/4095)
      foc1.data.offsetCurrB = (float)adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent; // Phase B curr.offset in P.U.
      foc1.data.offsetCurrC = (float)adcData.ph_w * 0.0002442f * foc1.config.adcFullScaleCurrent; // Phase B curr.offset in P.U.
      // filtered BEMF
      LPF_calc(&foc1.cmtn.lpf_bemfA);
      LPF_calc(&foc1.cmtn.lpf_bemfB);
      LPF_calc(&foc1.cmtn.lpf_bemfC);
      // clear flags and reset data
      foc1.data.bldc_commCntr = 0.f;
      foc1.flag.driveStateSwitched = 1;
      foc1.pi_spd.Ref = 0.f;
      foc1.pi_spd.Ui = 0.f;
      foc1.pi_spd.OutPreSat = 0.f;
      foc1.data.bldc_duty = 0.f;
      // Calc Angle
      foc1.data.angle = MF_PI - (M_PI3 + ((float)foc1.cmtn.cmtnState * M_PI3));
      // calc PLL
      foc1.pll.AngleRaw = foc1.data.angle;
      foc1.calc.pll(&foc1.pll);
      bldcpwm_update((uint32_t)foc1.data.halfPwmPeriod, (uint32_t)(foc1.data.halfPwmPeriod), foc1.cmtn.cmtnState);
      /* code */
      break;
    case STARTUP_BLDC:
      // Reset couners, controllers data
      foc1.data.isrCntr1 = 0;
      foc1.flag.bldcStartup = 0;
      // Scale ADC data
      foc1.cmtn.lpf_bemfA.in = (float)(adcData.v_u - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfB.in = (float)(adcData.v_v - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfC.in = (float)(adcData.v_w - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.data.iPhaseA = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
      foc1.data.iPhaseB = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
      foc1.data.iPhaseC = (adcData.ph_w * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrC;
      foc1.data.iAvg = sqrtf((SQ(foc1.data.iPhaseA) + SQ(foc1.data.iPhaseB) + SQ(foc1.data.iPhaseC))) * 0.666666f; // 2/3
      // filtered total current
      foc1.lpf_Iavg.in = foc1.data.iAvg;
      LPF_calc(&foc1.lpf_Iavg);
      foc1.data.iAvgFiltered = foc1.lpf_Iavg.out;
      // filtered BEMF
      LPF_calc(&foc1.cmtn.lpf_bemfA);
      LPF_calc(&foc1.cmtn.lpf_bemfB);
      LPF_calc(&foc1.cmtn.lpf_bemfC);
      // check ZCD and new commutation step
      foc1.cmtn.As = foc1.cmtn.lpf_bemfA.out;
      foc1.cmtn.Bs = foc1.cmtn.lpf_bemfB.out;
      foc1.cmtn.Cs = foc1.cmtn.lpf_bemfC.out;
      foc1.calc.cmtn(&foc1.cmtn);
      // Calc Angle
      foc1.data.angle = MF_PI - (M_PI3 + ((float)foc1.cmtn.cmtnState * M_PI3)); // angular size of 1 section - 60 deg or pi/3 rad (electrical)
      // calc PLL
      foc1.pll.AngleRaw = foc1.data.angle;
      foc1.calc.pll(&foc1.pll);
      // Indicate motor state
      if (foc1.data.iAvgFiltered > 0.2f || foc1.pll.SpeedPll < 10.f) // indicate non-zero current
        ledDelay = 0.5f;
      if (foc1.pll.SpeedPll > 10.f) // indicate non-zero speed
        ledDelay = 0.2f;
      else
        ledDelay = 1.f;
      // calc duty
      /* set startup duty if we has start conditions */
      foc1.data.bldc_duty = (foc1.flag.driveStateSwitched == 1) ? foc1.data.bldc_dutyStart : foc1.data.bldc_duty;
      if (foc1.data.bldc_duty < (foc1.data.bldc_dutyRef - foc1.config.tS)) // add some hysteresys
      {
        foc1.data.bldc_duty += foc1.config.tS * 0.25f; // ramp rate: 0.25/s
      }
      if (foc1.data.bldc_duty > (foc1.data.bldc_dutyRef + foc1.config.tS))
      {
        foc1.data.bldc_duty -= foc1.config.tS * 0.25f; // ramp rate: 0.25/s
      }
      // startup: 100 ms in duty cycle control mode and switch to speed control
      if (foc1.data.isrCntr1 > (uint32_t)(foc1.config.pwmFreq * 0.1f)) // 100 ms
      {
        foc1.data.isrCntr1 = 0;
        foc1.driveStateBLDC = RUN_SPD_BLDC;
      }
      // New commutation
      /* if first enter, make 1 commutation blindly */
      if (foc1.cmtn.Trigger > 0 || foc1.flag.driveStateSwitched == 1)
      {
        foc1.flag.driveStateSwitched = 0;

        if (foc1.cmtn.dir == 1)
          foc1.cmtn.cmtnState = (foc1.cmtn.cmtnState >= 5) ? 0 : foc1.cmtn.cmtnState + 1;
        else
          foc1.cmtn.cmtnState = (foc1.cmtn.cmtnState <= 0) ? 5 : foc1.cmtn.cmtnState - 1;
      }
      // Update PWM values
      bldcpwm_update((uint32_t)foc1.data.halfPwmPeriod, (uint32_t)(foc1.data.halfPwmPeriod * foc1.data.bldc_duty), foc1.cmtn.cmtnState);

      // call datalogger
      dataLog.in1 = foc1.cmtn.As;
      dataLog.in2 = foc1.cmtn.Trigger;
      dataLog.in3 = foc1.cmtn.Neutral;
      dataLog.in4 = foc1.cmtn.fluxBuff;
      datalogCalc(&dataLog);
      break;
    case RUN_OPENLOOP_BLDC:
      foc1.cmtn.lpf_bemfA.in = (float)(adcData.v_u - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfB.in = (float)(adcData.v_v - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfC.in = (float)(adcData.v_w - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.data.iPhaseA = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
      foc1.data.iPhaseB = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
      foc1.data.iPhaseC = (adcData.ph_w * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrC;
      foc1.data.iAvg = sqrtf((SQ(foc1.data.iPhaseA) + SQ(foc1.data.iPhaseB) + SQ(foc1.data.iPhaseC))) * 0.666666f; // 2/3
      // filtered total current
      foc1.lpf_Iavg.in = foc1.data.iAvg;
      LPF_calc(&foc1.lpf_Iavg);
      foc1.data.iAvgFiltered = foc1.lpf_Iavg.out;
      // filtered BEMF
      LPF_calc(&foc1.cmtn.lpf_bemfA);
      LPF_calc(&foc1.cmtn.lpf_bemfB);
      LPF_calc(&foc1.cmtn.lpf_bemfC);
      // check ZCD and new commutation step
      foc1.cmtn.As = foc1.cmtn.lpf_bemfA.out;
      foc1.cmtn.Bs = foc1.cmtn.lpf_bemfB.out;
      foc1.cmtn.Cs = foc1.cmtn.lpf_bemfC.out;
      foc1.calc.cmtn(&foc1.cmtn);
      // Calc Angle
      foc1.data.angle = MF_PI - (M_PI3 + ((float)foc1.cmtn.cmtnState * M_PI3));
      // calc PLL
      foc1.pll.AngleRaw = foc1.data.angle;
      foc1.calc.pll(&foc1.pll);
      // Indicate motor state
      if (foc1.data.iAvgFiltered > 0.2f || foc1.pll.SpeedPll < 10.f) // indicate non-zero current
        ledDelay = 0.5f;
      if (foc1.pll.SpeedPll > 10.f) // indicate non-zero speed
        ledDelay = 0.2f;
      else
        ledDelay = 1.f;
      /* decrease delay betwen commutation steps to commCntrRef value */
      if (foc1.data.bldc_commCntr >= foc1.data.bldc_commCntrRef)
      {
        foc1.data.bldc_commCntr -= (foc1.config.tS * 500.f);
      }

      if (foc1.data.isrCntr1 >= (uint32_t)foc1.data.bldc_commCntr)
      {
        foc1.data.isrCntr1 = 0;

        if (foc1.cmtn.dir == 1)
          foc1.cmtn.cmtnState = (foc1.cmtn.cmtnState >= 5) ? 0 : foc1.cmtn.cmtnState + 1;
        else
          foc1.cmtn.cmtnState = (foc1.cmtn.cmtnState <= 0) ? 5 : foc1.cmtn.cmtnState - 1;
      }
      // Update PWM values
      bldcpwm_update((uint32_t)foc1.data.halfPwmPeriod, (uint32_t)(foc1.data.halfPwmPeriod * foc1.data.bldc_duty), foc1.cmtn.cmtnState);
      // call datalogger
      dataLog.in1 = foc1.cmtn.As;
      dataLog.in2 = foc1.cmtn.Trigger;
      dataLog.in3 = foc1.cmtn.Neutral;
      dataLog.in4 = foc1.cmtn.fluxBuff;
      datalogCalc(&dataLog);
      break;
    case RUN_CLOSEDLOOP_BLDC:
      foc1.cmtn.lpf_bemfA.in = (float)(adcData.v_u - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfB.in = (float)(adcData.v_v - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfC.in = (float)(adcData.v_w - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.data.iPhaseA = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
      foc1.data.iPhaseB = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
      foc1.data.iPhaseC = (adcData.ph_w * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrC;
      foc1.data.iAvg = sqrtf((SQ(foc1.data.iPhaseA) + SQ(foc1.data.iPhaseB) + SQ(foc1.data.iPhaseC))) * 0.666666f; // 2/3
      // filtered total current
      foc1.lpf_Iavg.in = foc1.data.iAvg;
      LPF_calc(&foc1.lpf_Iavg);
      foc1.data.iAvgFiltered = foc1.lpf_Iavg.out;
      // filtered BEMF
      LPF_calc(&foc1.cmtn.lpf_bemfA);
      LPF_calc(&foc1.cmtn.lpf_bemfB);
      LPF_calc(&foc1.cmtn.lpf_bemfC);
      // check ZCD and new commutation step
      foc1.cmtn.As = foc1.cmtn.lpf_bemfA.out;
      foc1.cmtn.Bs = foc1.cmtn.lpf_bemfB.out;
      foc1.cmtn.Cs = foc1.cmtn.lpf_bemfC.out;
      foc1.calc.cmtn(&foc1.cmtn);
      // Calc Angle
      foc1.data.angle = MF_PI - (M_PI3 + ((float)foc1.cmtn.cmtnState * M_PI3));
      // calc PLL
      foc1.pll.AngleRaw = foc1.data.angle;
      foc1.calc.pll(&foc1.pll);
      // Indicate motor state
      if (foc1.data.iAvgFiltered > 0.2f || foc1.pll.SpeedPll < 10.f) // indicate non-zero current
        ledDelay = 0.5f;
      if (foc1.pll.SpeedPll > 10.f) // indicate non-zero speed
        ledDelay = 0.2f;
      else
        ledDelay = 1.f;
      // calc duty
      // NEW:
      foc1.data.bldc_duty = (foc1.flag.driveStateSwitched == 1) ? foc1.data.bldc_dutyStart : foc1.data.bldc_duty;
      foc1.pi_id.Ref = (foc1.flag.driveStateSwitched == 1) ? foc1.data.bldc_dutyStart + foc1.pi_id.Ref : foc1.pi_id.Ref;
      foc1.pi_id.Fdb = foc1.data.iAvgFiltered;
      foc1.calc.pi_reg(&foc1.pi_id);
      foc1.data.bldc_duty = foc1.pi_id.Out;
      /* set startup duty if we has start conditions */
      foc1.data.bldc_duty = (foc1.flag.driveStateSwitched == 1) ? foc1.data.bldc_dutyStart : foc1.data.bldc_duty;
      if (foc1.data.bldc_duty < (foc1.data.bldc_dutyRef - foc1.config.tS)) // add some hysteresys
      {
        foc1.data.bldc_duty += foc1.config.tS * 0.5f; // ramp rate: 0.25/s
      }
      if (foc1.data.bldc_duty > (foc1.data.bldc_dutyRef + foc1.config.tS))
      {
        foc1.data.bldc_duty -= foc1.config.tS * 0.5f; // ramp rate: 0.25/s
      }
      // New commutation
      if (foc1.data.isrCntr1 > (uint32_t)(0.3f * foc1.config.pwmFreq) && foc1.flag.bldcStartup == 0)
      {
        foc1.flag.driveStateSwitched = 1;
        foc1.data.isrCntr1 = 0;
        foc1.flag.bldcStartup = 1;
      }
      /* if first enter, make 1 commutation blindly */
      if (foc1.cmtn.Trigger > 0 || foc1.flag.driveStateSwitched == 1)
      {
        foc1.flag.driveStateSwitched = 0;
        if (foc1.cmtn.dir == 1)
          foc1.cmtn.cmtnState = (foc1.cmtn.cmtnState >= 5) ? 0 : foc1.cmtn.cmtnState + 1;
        else
          foc1.cmtn.cmtnState = (foc1.cmtn.cmtnState <= 0) ? 5 : foc1.cmtn.cmtnState - 1;
      }
      // Update PWM values
      bldcpwm_update((uint32_t)foc1.data.halfPwmPeriod, (uint32_t)(foc1.data.halfPwmPeriod * foc1.data.bldc_duty), foc1.cmtn.cmtnState);
      // call datalogger
      dataLog.in1 = foc1.cmtn.As;
      dataLog.in2 = foc1.cmtn.Trigger;
      dataLog.in3 = foc1.cmtn.Neutral;
      dataLog.in4 = foc1.cmtn.fluxBuff;
      datalogCalc(&dataLog);
      break;
    case RUN_SPD_BLDC:
      foc1.cmtn.lpf_bemfA.in = (float)(adcData.v_u - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfB.in = (float)(adcData.v_v - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.cmtn.lpf_bemfC.in = (float)(adcData.v_w - 2047) * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      foc1.data.iPhaseA = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
      foc1.data.iPhaseB = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
      foc1.data.iPhaseC = (adcData.ph_w * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrC;
      foc1.data.iAvg = sqrtf((SQ(foc1.data.iPhaseA) + SQ(foc1.data.iPhaseB) + SQ(foc1.data.iPhaseC))) * 0.666666f; // 2/3
      // filtered total current
      foc1.lpf_Iavg.in = foc1.data.iAvg;
      LPF_calc(&foc1.lpf_Iavg);
      foc1.data.iAvgFiltered = foc1.lpf_Iavg.out;
      // filtered BEMF
      LPF_calc(&foc1.cmtn.lpf_bemfA);
      LPF_calc(&foc1.cmtn.lpf_bemfB);
      LPF_calc(&foc1.cmtn.lpf_bemfC);
      // check ZCD and new commutation step
      foc1.cmtn.As = foc1.cmtn.lpf_bemfA.out;
      foc1.cmtn.Bs = foc1.cmtn.lpf_bemfB.out;
      foc1.cmtn.Cs = foc1.cmtn.lpf_bemfC.out;
      foc1.calc.cmtn(&foc1.cmtn);
      // Calc Angle
      foc1.data.angle = MF_PI - (M_PI3 + ((float)foc1.cmtn.cmtnState * M_PI3));
      // calc PLL
      foc1.pll.Ki = 10000.f;
      foc1.pll.AngleRaw = foc1.data.angle;
      foc1.calc.pll(&foc1.pll);
      // Indicate motor state
      if (foc1.data.iAvgFiltered > 0.2f || foc1.pll.SpeedPll < 10.f) // indicate non-zero current
        ledDelay = 0.5f;
      if (foc1.pll.SpeedPll > 10.f) // indicate non-zero speed
        ledDelay = 0.2f;
      else
        ledDelay = 1.f;
      // calc speed control
      // ramp speed reference
      if (foc1.pi_spd.Ref < (fabsf(foc1.data.speedRef) - foc1.config.tS)) // add some hysteresys
      {
        foc1.pi_spd.Ref += foc1.config.tS * 1250.f; // ramp rate: 1500.0 rad/s/s
      }
      if (foc1.pi_spd.Ref > (fabsf(foc1.data.speedRef) + foc1.config.tS))
      {
        foc1.pi_spd.Ref -= foc1.config.tS * 1250.f; // ramp rate: 1500.0 rad/s/s
      }
      foc1.pi_spd.Fdb = fabsf(foc1.pll.SpeedPll);
      foc1.calc.pi_reg(&foc1.pi_spd);
      foc1.data.bldc_duty = foc1.pi_spd.Out;
      /* set startup duty if we has start conditions */
      foc1.data.bldc_duty = (foc1.flag.driveStateSwitched == 1) ? foc1.data.bldc_dutyStart : foc1.data.bldc_duty;
      // New commutation
      /* if first enter, make 1 commutation blindly */
      if (foc1.cmtn.Trigger > 0 || foc1.flag.driveStateSwitched == 1)
      {
        foc1.flag.driveStateSwitched = 0;                   //(fabsf(foc1.pll.SpeedPll) < 5.f) ? 1 : 0;
        foc1.cmtn.dir = (foc1.data.speedRef < 0.f) ? 1 : 0; // change comm sequance if speedRef is nagative
        if (foc1.cmtn.dir == 1)
          foc1.cmtn.cmtnState = (foc1.cmtn.cmtnState >= 5) ? 0 : foc1.cmtn.cmtnState + 1;
        else
          foc1.cmtn.cmtnState = (foc1.cmtn.cmtnState <= 0) ? 5 : foc1.cmtn.cmtnState - 1;
      }
      // Update PWM values
      bldcpwm_update((uint32_t)foc1.data.halfPwmPeriod, (uint32_t)(foc1.data.halfPwmPeriod * foc1.data.bldc_duty), foc1.cmtn.cmtnState);
      // call datalogger
      dataLog.in1 = foc1.cmtn.As;
      dataLog.in2 = foc1.cmtn.Trigger;
      dataLog.in3 = foc1.cmtn.Neutral;
      dataLog.in4 = foc1.cmtn.fluxBuff;
      datalogCalc(&dataLog);
      break;
    case FAULT_BLDC:
      if (foc1.data.isrCntr0 == 1)
        LL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
      break;
    default:
      break;
    }
  }
  else if (foc1.config.mode == FOC)
  {
    /* take data from ADC */
    adcData.ph_v = ADC1->JDR1;
    adcData.ph_w = ADC1->JDR2;
    adcData.ph_u = ADC1->JDR3;
    adcData.v_dc = ADC1->JDR4;
    adcData.pcb_temp = ADC2->JDR1;
    /* PCBtemp channel used software trigger */
    LL_ADC_INJ_StartConversion(ADC2);

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
      foc1.data.angle = 0.f;
      hall.angleInc = 0.f;
      hall.state = hall.statePr;
      ledDelay = 0.f;
      // Angle generator
      // foc1.data.angle += foc1.data.freqStep * foc1.data.freq;
      // foc1.data.angle = (foc1.data.angle > MF_PI) ? -MF_PI : foc1.data.angle;
      // calc current sensors offset
      foc1.data.offsetCurrA = (float)adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent; // Phase A curr.offset in P.U. (1/4095)
      foc1.data.offsetCurrB = (float)adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent; // Phase B curr.offset in P.U.
      foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      // calc sin/cos
      foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(0.0f)));
      foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(0.0f)));
      // calc phase voltages
      if (foc1.config.sim == 1)
        foc1.volt.DcBusVolt = bldc1.udc;
      else
        foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      // Filtered data
      foc1.lpf_vdc.in = foc1.volt.DcBusVolt;
      LPF_calc(&foc1.lpf_vdc);
      foc1.volt.DcBusVolt = foc1.lpf_vdc.out;
      // volt calc
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      foc1.volt.usd = foc1.volt.usa * foc1.data.cosTheta + foc1.volt.usb * foc1.data.sinTheta;
      foc1.volt.usq = -foc1.volt.usa * foc1.data.sinTheta + foc1.volt.usb * foc1.data.cosTheta;
      // set duty to 50%
      LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | LL_HRTIM_OUTPUT_TC1 |
                                        LL_HRTIM_OUTPUT_TC2 | LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2);

      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_TIM_GetPeriod(HRTIM1, LL_HRTIM_TIMER_A) >> 1);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_TIM_GetPeriod(HRTIM1, LL_HRTIM_TIMER_C) >> 1);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_TIM_GetPeriod(HRTIM1, LL_HRTIM_TIMER_D) >> 1);
      // calc BLDC motor model
      ModelBLDC_Calc(&bldc1);
      // check enable pin
      // if (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_Pin) == 1 && foc1.volt.DcBusVolt >= 18.f)
      // {
      // foc1.driveState = (foc1.paramIdState != Cmplt) ? PARAM_ID : foc1.driveState;
      // foc1.driveState = (foc1.paramIdState == Cmplt) ? ALIGN : foc1.driveState;
      // foc1.driveState = (foc1.data.flashUpdateFlag != 0) ? RUN_CLOSEDLOOP_SPD : foc1.driveState;
      // }
      // if (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_Pin) == 0 && foc1.volt.DcBusVolt >= 18.f && pwmInputValue < 100)
      // {
      //   foc1.driveState = RUN_CLOSEDLOOP_SPD;
      // }
      // Check protection and enable if precharge is done
      foc1.prot.prechargeFlag = (foc1.volt.DcBusVolt >= 20.f) ? 1 : 0;
      foc1.prot.currPhU = foc1.data.isa;
      foc1.prot.currPhV = foc1.data.isb_tmp;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // call datalogger
      dataLog.in1 = bldc1.tetaR;
      dataLog.in2 = foc1.smo.Theta;
      dataLog.in3 = foc1.pi_id.Ref;
      dataLog.in4 = foc1.pi_iq.Ref;
      datalogCalc(&dataLog);
      break;
    case RUN_OPENLOOP_VHZ:
      // Angle generator
      foc1.data.angle += foc1.data.freqStep * foc1.data.freq;
      if (foc1.data.angle < -MF_PI)
        foc1.data.angle = foc1.data.angle + M_2PI;
      else if (foc1.data.angle > MF_PI)
        foc1.data.angle = foc1.data.angle - M_2PI;
      // calc sin/cos
      foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(foc1.data.angle)));
      foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(foc1.data.angle)));
      // clarke transform for phase currents
      if (foc1.config.sim == 1)
      {
        foc1.data.isa = bldc1.isPhaseA;
        foc1.data.isb = _1DIV_SQRT3 * bldc1.isPhaseA + _2DIV_SQRT3 * bldc1.isPhaseB;
        foc1.volt.DcBusVolt = bldc1.udc;
      }
      else if (foc1.config.sim == 0)
      {
        foc1.data.isa = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
        foc1.data.isb_tmp = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
        foc1.data.isb = _1DIV_SQRT3 * foc1.data.isa + _2DIV_SQRT3 * foc1.data.isb_tmp;
        foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      }
      // Filtered data
      foc1.lpf_id.in = foc1.data.isd;
      foc1.lpf_iq.in = foc1.data.isq;
      foc1.lpf_vdc.in = foc1.volt.DcBusVolt;
      LPF_calc(&foc1.lpf_id);
      LPF_calc(&foc1.lpf_iq);
      LPF_calc(&foc1.lpf_vdc);
      foc1.volt.DcBusVolt = foc1.lpf_vdc.out;
      // volt calc
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      foc1.volt.usd = foc1.volt.usa * foc1.data.cosTheta + foc1.volt.usb * foc1.data.sinTheta;
      foc1.volt.usq = -foc1.volt.usa * foc1.data.sinTheta + foc1.volt.usb * foc1.data.cosTheta;
      //calc IPARK transform (IPARK output -> SVGEN input)  !reference D/Q voltages
      foc1.svgen.usa = foc1.data.udRef * foc1.data.cosTheta - foc1.data.uqRef * foc1.data.sinTheta;
      foc1.svgen.usb = foc1.data.uqRef * foc1.data.cosTheta + foc1.data.udRef * foc1.data.sinTheta;
      // calc space-vector generator
      foc1.calc.svgen(&foc1.svgen);
      // Check protection
      foc1.prot.currPhU = foc1.data.isa;
      foc1.prot.currPhV = foc1.data.isb_tmp;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // apply voltage to motor and pwm driver
      bldc1.cmpr0 = foc1.svgen.Ta;
      bldc1.cmpr1 = foc1.svgen.Tb;
      bldc1.cmpr2 = foc1.svgen.Tc;
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(foc1.svgen.Ta * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(foc1.svgen.Tb * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(foc1.svgen.Tc * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      // calc BLDC motor model
      //ModelBLDC_Calc(&bldc1);
      // call datalogger
      dataLog.in1 = bldc1.tetaR;
      dataLog.in2 = foc1.smo.Theta;
      dataLog.in3 = foc1.pi_id.Ref;
      dataLog.in4 = foc1.pi_iq.Ref;
      datalogCalc(&dataLog);
      break;
    case ALIGN:
      ledDelay = 0.1f;
      foc1.data.idRef = 5.f;
      // Angle generator
      foc1.data.angle += foc1.data.freqStep * foc1.data.freq;
      if (foc1.data.angle < -MF_PI)
        foc1.data.angle = foc1.data.angle + M_2PI;
      else if (foc1.data.angle > MF_PI)
        foc1.data.angle = foc1.data.angle - M_2PI;
      // calc sin/cos
      foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(foc1.data.angle)));
      foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(foc1.data.angle)));
      // clarke transform for phase currents
      if (foc1.config.sim == 1)
      {
        foc1.data.isa = bldc1.isPhaseA;
        foc1.data.isb = _1DIV_SQRT3 * bldc1.isPhaseA + _2DIV_SQRT3 * bldc1.isPhaseB;
        foc1.volt.DcBusVolt = bldc1.udc;
      }
      else if (foc1.config.sim == 0)
      {
        foc1.data.isa = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
        foc1.data.isb_tmp = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
        foc1.data.isb = _1DIV_SQRT3 * foc1.data.isa + _2DIV_SQRT3 * foc1.data.isb_tmp;
        foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      }
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
      foc1.pi_id.Ref = foc1.data.idRef;
      foc1.pi_iq.Ref = foc1.data.iqRef;
      foc1.pi_id.Fdb = foc1.data.isd;
      foc1.pi_iq.Fdb = foc1.data.isq;
      foc1.calc.pi_reg(&foc1.pi_id);
      foc1.calc.pi_reg(&foc1.pi_iq);
      // feed-forward axis decoupling
      /* TODO */
      // Saturation
      foc1.volt.usd = foc1.pi_id.Out;
      foc1.volt.usq = foc1.pi_iq.Out;
      foc1.volt.vMagMax = 0.66666f * foc1.volt.dutyMax * SQRT3_BY_2 * foc1.volt.DcBusVolt; // 0.66666 => 2/3
      utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.vMagMax);
      foc1.volt.usd = foc1.volt.usd / (0.66666f * foc1.volt.DcBusVolt);
      foc1.volt.usq = foc1.volt.usq / (0.66666f * foc1.volt.DcBusVolt);
      // calc phase voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      //calc IPARK transform (IPARK output -> SVGEN input)
      foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
      foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
      // calc space-vector generator (1/Vdc) * sqrt(3)
      foc1.calc.svgen(&foc1.svgen);
      // estimete Hall sensor position
      hall.A = LL_GPIO_IsInputPinSet(HALL_PHASE_A_GPIO_Port, HALL_PHASE_A_Pin);
      hall.B = LL_GPIO_IsInputPinSet(HALL_PHASE_B_GPIO_Port, HALL_PHASE_B_Pin);
      hall.C = LL_GPIO_IsInputPinSet(HALL_PHASE_C_GPIO_Port, HALL_PHASE_C_Pin);
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
      // Calc Angle offset
      if (hall.state != hall.statePr)
      {
        hall.offsetFwd[hall.state] = (hall.offsetState == 0) ? foc1.data.angle : hall.offsetFwd[hall.state];
        hall.offsetRev[hall.state] = (hall.offsetState == 1) ? foc1.data.angle : hall.offsetRev[hall.state];
        hall.offset = foc1.data.angle;
        hall.offsetFlag = 1;
      }
      hall.statePr = hall.state;
      // calc avg
      hall.angleInc += foc1.data.freqStep * foc1.data.freq;
      if (hall.angleInc > (M_2PI * 0.999f) && hall.offsetState == 0)
      {
        foc1.data.freq = -foc1.data.freq;
        hall.angleInc = M_2PI;
        hall.offsetState = 1;
      }
      if (hall.angleInc < (0.001f) && hall.offsetState == 1)
      {
        foc1.data.freq = 0;
        hall.offsetState = 2;
      }
      if (hall.offsetState == 2)
      {
        for (uint8_t i = 0; i < 6; i++)
        {
          float diff = fabsf(utils_angle_difference_rad(hall.offsetFwd[i], hall.offsetRev[i]));
          /* addition half of difference between rising and falling edge of Hall signal will estimate middle of the pulse */
          hall.offsetAvg[i] = hall.offsetFwd[i] + (diff * 0.5f);
          if (hall.offsetAvg[i] < -MF_PI)
            hall.offsetAvg[i] = hall.offsetAvg[i] + M_2PI;
          else if (hall.offsetAvg[i] > MF_PI)
            hall.offsetAvg[i] = hall.offsetAvg[i] - M_2PI;
        }
        /* Reset and end */
        foc1.data.idRef = 0.f;
        foc1.data.iqRef = 0.f;
        foc1.data.freq = 0.f;
        foc1.driveState = STOP;
      }
      // calc flux observer
      foc1.flux.i_alpha = foc1.data.isa;
      foc1.flux.i_beta = foc1.data.isb;
      foc1.flux.v_alpha = foc1.svgen.usa;
      foc1.flux.v_beta = foc1.svgen.usb;
      foc1.calc.flux(&foc1.flux);
      // calc speed PLL
      foc1.pll.AngleRaw = foc1.smo.Theta; //foc1.flux.phase;
      foc1.calc.pll(&foc1.pll);
      // Check protection
      foc1.prot.currPhU = foc1.data.isa;
      foc1.prot.currPhV = foc1.data.isb_tmp;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // apply voltage to motor and pwm driver
      bldc1.cmpr0 = foc1.svgen.Ta;
      bldc1.cmpr1 = foc1.svgen.Tb;
      bldc1.cmpr2 = foc1.svgen.Tc;
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(foc1.svgen.Ta * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(foc1.svgen.Tb * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(foc1.svgen.Tc * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      // calc BLDC motor model
      //ModelBLDC_Calc(&bldc1);
      // call datalogger
      if (foc1.data.idRef != 0 || foc1.data.iqRef != 0)
        dataLog.trigger = 1;
      dataLog.in1 = foc1.data.isd;
      dataLog.in2 = foc1.data.isq;
      dataLog.in3 = foc1.smo.tmp_Ealpha;
      dataLog.in4 = foc1.smo.tmp_Ebeta;
      datalogCalc(&dataLog);
      break;
    case RUN_OPENLOOP_IHZ:
      // Angle generator
      foc1.data.angle += foc1.data.freqStep * foc1.data.freq;
      if (foc1.data.angle < -MF_PI)
        foc1.data.angle = foc1.data.angle + M_2PI;
      else if (foc1.data.angle > MF_PI)
        foc1.data.angle = foc1.data.angle - M_2PI;
      // calc sin/cos
      foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(foc1.data.angle)));
      foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(foc1.data.angle)));
      // clarke transform for phase currents
      if (foc1.config.sim == 1)
      {
        foc1.data.isa = bldc1.isPhaseA;
        foc1.data.isb = _1DIV_SQRT3 * bldc1.isPhaseA + _2DIV_SQRT3 * bldc1.isPhaseB;
        foc1.volt.DcBusVolt = bldc1.udc;
      }
      else if (foc1.config.sim == 0)
      {
        foc1.data.isa = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
        foc1.data.isb_tmp = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
        foc1.data.isb = _1DIV_SQRT3 * foc1.data.isa + _2DIV_SQRT3 * foc1.data.isb_tmp;
        foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      }
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
      foc1.data.isq = foc1.data.isb * foc1.data.cosTheta - foc1.data.isa * foc1.data.sinTheta;
      // calc PI current controllers for D/Q axis
      foc1.pi_id.Ref = foc1.data.idRef;
      foc1.pi_iq.Ref = foc1.data.iqRef;
      foc1.pi_id.Fdb = foc1.data.isd;
      foc1.pi_iq.Fdb = foc1.data.isq;
      foc1.calc.pi_reg(&foc1.pi_id);
      foc1.calc.pi_reg(&foc1.pi_iq);
      // Saturation
      // foc1.volt.usd = foc1.pi_id.Out;
      // foc1.volt.usq = foc1.pi_iq.Out;
      // foc1.volt.vMagMax = 0.66666f * foc1.volt.dutyMax * SQRT3_BY_2 * foc1.volt.DcBusVolt; // 0.66666 => 2/3
      // utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.vMagMax);
      // foc1.volt.usd = foc1.volt.usd / (0.66666f * foc1.volt.DcBusVolt);
      // foc1.volt.usq = foc1.volt.usq / (0.66666f * foc1.volt.DcBusVolt);

      // feed-forward axis decoupling
      if (foc1.config.axisDecEn == 1)
      {
        foc1.volt.usd = foc1.pi_id.Out;
        foc1.volt.usq = foc1.pi_iq.Out;
        foc1.data.udDec = foc1.data.isq * foc1.pll.SpeedPll * foc1.config.Lq;
        foc1.data.uqDec = foc1.pll.SpeedPll * (foc1.data.isd * foc1.config.Ld + foc1.flux.fluxLeakage);
        foc1.volt.usd -= foc1.data.udDec;
        foc1.volt.usq += foc1.data.uqDec;
      }
      else
      {
        foc1.volt.usd = foc1.pi_id.Out;
        foc1.volt.usq = foc1.pi_iq.Out;
      }
      // Saturation
      foc1.volt.vMagMax = foc1.volt.dutyMax * SQRT3_BY_2 * foc1.volt.DcBusVolt; // 0.66666 => 2/3
      utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.vMagMax);
      foc1.volt.usd = foc1.volt.usd / (foc1.volt.DcBusVolt * _2DIV_SQRT3);
      foc1.volt.usq = foc1.volt.usq / (foc1.volt.DcBusVolt * _2DIV_SQRT3);
      // calc phase voltages
      // calc phase voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      //calc IPARK transform (IPARK output -> SVGEN input)
      foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
      foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
      // calc space-vector generator
      foc1.calc.svgen(&foc1.svgen);
      // estimete Hall sensor position
      hall.A = LL_GPIO_IsInputPinSet(HALL_PHASE_A_GPIO_Port, HALL_PHASE_A_Pin);
      hall.B = LL_GPIO_IsInputPinSet(HALL_PHASE_B_GPIO_Port, HALL_PHASE_B_Pin);
      hall.C = LL_GPIO_IsInputPinSet(HALL_PHASE_C_GPIO_Port, HALL_PHASE_C_Pin);
      hall.isrCntr++;
      hall.speedE = foc1.pll.SpeedPll;
      Hall_update(&hall);
      // calc obsrver
      // if (foc1.config.sim == 1)
      //   foc1.smo.freq = (bldc1.omega * bldc1.pp) + 1e-20f; // BLDC motor electrical angular frequency, 1e-20 for avoid QNAN
      // else if (foc1.config.sim == 0)
      // {
      //   foc1.smo.freq = foc1.data.freq + 1e-20f; //foc1.spdCalc.Speed;
      // }
      // ABS(foc1.smo.freq);
      // foc1.smo.Ialpha = foc1.data.isa;
      // foc1.smo.Ibeta = foc1.data.isb;
      // foc1.smo.Valpha = foc1.svgen.usa;
      // foc1.smo.Vbeta = foc1.svgen.usb;
      // foc1.calc.smo(&foc1.smo);
      // calc flux observer
      foc1.flux.i_alpha = foc1.data.isa;
      foc1.flux.i_beta = foc1.data.isb;
      foc1.flux.v_alpha = foc1.svgen.usa;
      foc1.flux.v_beta = foc1.svgen.usb;
      foc1.calc.flux(&foc1.flux);
      // calc speed PLL
      foc1.pll.AngleRaw = hall.angle; //foc1.smo.Theta; //foc1.flux.phase;
      foc1.calc.pll(&foc1.pll);
      // Check protection
      foc1.prot.currPhU = foc1.data.isa;
      foc1.prot.currPhV = foc1.data.isb_tmp;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // apply voltage to motor and pwm driver
      bldc1.cmpr0 = foc1.svgen.Ta;
      bldc1.cmpr1 = foc1.svgen.Tb;
      bldc1.cmpr2 = foc1.svgen.Tc;
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(foc1.svgen.Ta * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(foc1.svgen.Tb * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(foc1.svgen.Tc * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      // calc BLDC motor model
      ModelBLDC_Calc(&bldc1);
      // call datalogger
      if (foc1.data.idRef != 0 || foc1.data.iqRef != 0)
        dataLog.trigger = 1;
      dataLog.in1 = foc1.data.isd;
      dataLog.in2 = foc1.data.isq;
      dataLog.in3 = foc1.smo.tmp_Ealpha;
      dataLog.in4 = foc1.smo.tmp_Ebeta;
      datalogCalc(&dataLog);
      break;
    case RUN_CLOSEDLOOP_DQ:
      // Angle generator
      foc1.data.angle += foc1.data.freqStep * foc1.data.freq;
      if (foc1.data.angle < -MF_PI)
        foc1.data.angle = foc1.data.angle + M_2PI;
      else if (foc1.data.angle > MF_PI)
        foc1.data.angle = foc1.data.angle - M_2PI;
      // calc sin/cos
      if (foc1.config.sim == 1)
      {
        foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(bldc1.tetaR)));
        foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(bldc1.tetaR)));
        // clarke transform for phase currents
        foc1.data.isa = bldc1.isPhaseA;
        foc1.data.isb = _1DIV_SQRT3 * bldc1.isPhaseA + _2DIV_SQRT3 * bldc1.isPhaseB;
        foc1.volt.DcBusVolt = bldc1.udc;
      }
      else if (foc1.config.sim == 0)
      {
        foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(foc1.pll.AnglePll)));
        foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(foc1.pll.AnglePll)));
        // clarke transform for phase currents
        foc1.data.isa = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
        foc1.data.isb_tmp = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
        foc1.data.isb = _1DIV_SQRT3 * foc1.data.isa + _2DIV_SQRT3 * foc1.data.isb_tmp;
        foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      }
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
      //foc1.data.isd = lpf_id.out;
      //foc1.data.isq = lpf_iq.out;
      foc1.volt.DcBusVolt = foc1.lpf_vdc.out;
      // calc PI current controllers for D/Q axis
      /* ramp for Iq current for smoothly transitions */
      if (foc1.pi_iq.Ref < (foc1.data.iqRef - foc1.config.tS)) // add some hysteresys
      {
        foc1.pi_iq.Ref += foc1.config.tS * foc1.data.iqRmp; // ramp rate: A/sec
      }
      if (foc1.pi_iq.Ref > (foc1.data.iqRef + foc1.config.tS))
      {
        foc1.pi_iq.Ref -= foc1.config.tS * foc1.data.iqRmp; // ramp rate: rad/s/s
      }
      foc1.pi_id.Ref = foc1.data.idRef;
      //foc1.pi_iq.Ref = foc1.data.iqRef;
      foc1.pi_iq.Ref = (foc1.data.iqRef < 0.1f && foc1.data.iqRef > -0.1f) ? 0.f : foc1.pi_iq.Ref; // for immediately stop
      foc1.pi_id.Fdb = foc1.data.isd;
      foc1.pi_iq.Fdb = foc1.data.isq;
      foc1.calc.pi_reg(&foc1.pi_id);
      foc1.calc.pi_reg(&foc1.pi_iq);
      // feed-forward axis decoupling
      if (foc1.config.axisDecEn == 1)
      {
        foc1.volt.usd = foc1.pi_id.Out;
        foc1.volt.usq = foc1.pi_iq.Out;
        foc1.data.udDec = foc1.data.isq * foc1.pll.SpeedPll * foc1.config.Lq;
        foc1.data.uqDec = foc1.pll.SpeedPll * (foc1.data.isd * foc1.config.Ld + foc1.flux.fluxLeakage);
        foc1.volt.usd -= foc1.data.udDec;
        foc1.volt.usq += foc1.data.uqDec;
      }
      else
      {
        foc1.volt.usd = foc1.pi_id.Out;
        foc1.volt.usq = foc1.pi_iq.Out;
      }
      // Saturation
      foc1.volt.vMagMax = foc1.volt.dutyMax * ONE_BY_SQRT3 * foc1.volt.DcBusVolt;
      utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.vMagMax);
      foc1.volt.usd = foc1.volt.usd / (foc1.volt.DcBusVolt);
      foc1.volt.usq = foc1.volt.usq / (foc1.volt.DcBusVolt);
      // Calc duty
      foc1.data.bldc_duty = sqrtf(foc1.volt.usd * foc1.volt.usd + foc1.volt.usq * foc1.volt.usq);
      // calc phase voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      //calc IPARK transform (IPARK output -> SVGEN input)
      foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
      foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
      // calc space-vector generator
      foc1.calc.svgen(&foc1.svgen);
      // estimete Hall sensor position
      hall.A = LL_GPIO_IsInputPinSet(HALL_PHASE_A_GPIO_Port, HALL_PHASE_A_Pin);
      hall.B = LL_GPIO_IsInputPinSet(HALL_PHASE_B_GPIO_Port, HALL_PHASE_B_Pin);
      hall.C = LL_GPIO_IsInputPinSet(HALL_PHASE_C_GPIO_Port, HALL_PHASE_C_Pin);
      hall.isrCntr++;
      hall.speedE = foc1.pll.SpeedPll;
      Hall_update(&hall);
      // calc bemf observer
      // if (foc1.config.sim == 1)
      //   foc1.smo.freq = (bldc1.omega * bldc1.pp) + 1e-20f; // BLDC motor electrical angular frequency, 1e-20 for avoid QNAN
      // else if (foc1.config.sim == 0)
      // {
      //   //foc1.smo.freq = fabsf(foc1.spdCalc.Speed) + 1e-20f;
      // }
      // ABS(foc1.smo.freq);
      // foc1.smo.Ialpha = foc1.data.isa;
      // foc1.smo.Ibeta = foc1.data.isb;
      // foc1.smo.Valpha = foc1.svgen.usa;
      // foc1.smo.Vbeta = foc1.svgen.usb;
      // foc1.calc.smo(&foc1.smo);
      // calc flux observer
      foc1.flux.i_alpha = foc1.data.isa;
      foc1.flux.i_beta = foc1.data.isb;
      foc1.flux.v_alpha = foc1.volt.usa;
      foc1.flux.v_beta = foc1.volt.usb;
      foc1.calc.flux(&foc1.flux);
      // calc speed PLL
      foc1.pll.AngleRaw = hall.angle; //foc1.smo.Theta; //foc1.flux.phase;
      foc1.calc.pll(&foc1.pll);
      foc1.data.speedRpm = (foc1.pll.SpeedPll * RADS2RPM) / foc1.config.pp;
      //foc1.config.Kv = foc1.data.speedRpm / (foc1.data.bldc_duty * foc1.volt.DcBusVolt);
      // calc Total current, Electrical torque and Power
      foc1.data.iAvg = sqrtf(SQ(foc1.lpf_id.out) + SQ(foc1.lpf_iq.out));
      //foc1.data.Te = 1.5f * foc1.config.pp * (foc1.flux.fluxLeakage * foc1.data.isq + (foc1.config.Ld - foc1.config.Lq) * foc1.data.isd * foc1.data.isq);
      foc1.data.Te = 1.5f * foc1.config.pp * (foc1.flux.fluxLeakage * foc1.data.isq);
      foc1.data.Pe = foc1.data.Te * (foc1.pll.SpeedPll / foc1.config.pp);
      // Check protection
      foc1.prot.currPhU = foc1.data.isa;
      foc1.prot.currPhV = foc1.data.isb_tmp;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // apply voltage to motor model and pwm driver
      bldc1.cmpr0 = foc1.svgen.Ta;
      bldc1.cmpr1 = foc1.svgen.Tb;
      bldc1.cmpr2 = foc1.svgen.Tc;
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(foc1.svgen.Ta * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(foc1.svgen.Tb * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(foc1.svgen.Tc * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      // calc BLDC motor model
      //ModelBLDC_Calc(&bldc1);
      // call datalogger
      dataLog.in1 = bldc1.tetaR;
      dataLog.in2 = foc1.smo.tmp_Theta; //foc1.smo.Theta;
      dataLog.in3 = foc1.smo.Theta;
      dataLog.in4 = foc1.smo.tmp_Ebeta;
      datalogCalc(&dataLog);
      break;
    case RUN_CLOSEDLOOP_SPD:
      // Angle generator
      foc1.data.angle += foc1.data.freqStep * foc1.data.freq;
      if (foc1.data.angle < -MF_PI)
        foc1.data.angle = foc1.data.angle + M_2PI;
      else if (foc1.data.angle > MF_PI)
        foc1.data.angle = foc1.data.angle - M_2PI;
      // calc sin/cos
      if (foc1.config.sim == 1)
      {
        foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(bldc1.tetaR)));
        foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(bldc1.tetaR)));
        // clarke transform for phase currents
        foc1.data.isa = bldc1.isPhaseA;
        foc1.data.isb = _1DIV_SQRT3 * bldc1.isPhaseA + _2DIV_SQRT3 * bldc1.isPhaseB;
        foc1.volt.DcBusVolt = bldc1.udc;
      }
      else if (foc1.config.sim == 0)
      {
        foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(foc1.pll.AnglePll)));
        foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(foc1.pll.AnglePll)));
        // clarke transform for phase currents
        foc1.data.isa = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
        foc1.data.isb_tmp = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
        foc1.data.isb = _1DIV_SQRT3 * foc1.data.isa + _2DIV_SQRT3 * foc1.data.isb_tmp;
        foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
      }
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
      //foc1.data.isd = lpf_id.out;
      //foc1.data.isq = lpf_iq.out;
      foc1.volt.DcBusVolt = foc1.lpf_vdc.out;

      // calc PI speed controller
      // ramp speed reference
      // foc1.data.speedRef = (pwmInputValue > 100) ? (float)pwmInputValue * 0.24414f : 0.f;
      // foc1.driveState = (pwmInputValue > 100) ? foc1.driveState : STOP;
      // foc1.data.speedRef = (LL_GPIO_IsInputPinSet(DIR_GPIO_Port, DIR_Pin) == 1) ? -foc1.data.speedRef : foc1.data.speedRef;

      if (foc1.pi_spd.Ref < (foc1.data.speedRef - foc1.config.tS)) // add some hysteresys
      {
        foc1.pi_spd.Ref += foc1.config.tS * foc1.data.spdRmp; // ramp rate: rad/s/s
      }
      if (foc1.pi_spd.Ref > (foc1.data.speedRef + foc1.config.tS))
      {
        foc1.pi_spd.Ref -= foc1.config.tS * foc1.data.spdRmp; // ramp rate: rad/s/s
      }
      foc1.data.spdLoopCntr++;
      if (foc1.data.spdLoopCntr >= 10)
      {
        foc1.data.spdLoopCntr = 0;
        foc1.pi_spd.Fdb = (foc1.config.sim == 1) ? bldc1.omega_rpm : foc1.pll.SpeedPll;
        foc1.calc.pi_reg(&foc1.pi_spd);
        foc1.pi_spd.Out = (foc1.pi_spd.Ref <= 5.f && foc1.pi_spd.Ref >= -5.f) ? 0.f : foc1.pi_spd.Out; // will test it!
      }
      // calc PI current controllers for D/Q axis
      foc1.pi_iq.Ref = foc1.pi_spd.Out;
      foc1.pi_id.Ref = foc1.data.idRef;
      //foc1.pi_iq.Ref = foc1.data.iqRef;

      /* TEST: use P-controller for Iq current control in speed mode */ 
      foc1.pi_iq.Kp = 10.f * foc1.config.Rs;
      foc1.pi_iq.Ki = 0.f;

      foc1.pi_id.Fdb = foc1.data.isd;
      foc1.pi_iq.Fdb = foc1.data.isq;
      foc1.calc.pi_reg(&foc1.pi_id);
      foc1.calc.pi_reg(&foc1.pi_iq);
      // feed-forward axis decoupling
      if (foc1.config.axisDecEn == 1)
      {
        foc1.volt.usd = foc1.pi_id.Out;
        foc1.volt.usq = foc1.pi_iq.Out;
        foc1.data.udDec = foc1.data.isq * foc1.pll.SpeedPll * foc1.config.Lq;
        foc1.data.uqDec = foc1.pll.SpeedPll * (foc1.data.isd * foc1.config.Ld + foc1.flux.fluxLeakage);
        foc1.volt.usd -= foc1.data.udDec;
        foc1.volt.usq += foc1.data.uqDec;
      }
      else
      {
        foc1.volt.usd = foc1.pi_id.Out;
        foc1.volt.usq = foc1.pi_iq.Out;
      }
      // Saturation
      foc1.volt.vMagMax = foc1.volt.dutyMax * ONE_BY_SQRT3 * foc1.volt.DcBusVolt;
      utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.vMagMax);
      foc1.volt.usd = foc1.volt.usd / (foc1.volt.DcBusVolt);
      foc1.volt.usq = foc1.volt.usq / (foc1.volt.DcBusVolt);
      // calc phase voltages
      foc1.volt.MfuncV1 = foc1.svgen.Ta;
      foc1.volt.MfuncV2 = foc1.svgen.Tb;
      foc1.volt.MfuncV3 = foc1.svgen.Tc;
      foc1.calc.volt(&foc1.volt);
      //calc IPARK transform (IPARK output -> SVGEN input)
      foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
      foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
      // calc space-vector generator
      foc1.calc.svgen(&foc1.svgen);
      // estimete Hall sensor position
      hall.A = LL_GPIO_IsInputPinSet(HALL_PHASE_A_GPIO_Port, HALL_PHASE_A_Pin);
      hall.B = LL_GPIO_IsInputPinSet(HALL_PHASE_B_GPIO_Port, HALL_PHASE_B_Pin);
      hall.C = LL_GPIO_IsInputPinSet(HALL_PHASE_C_GPIO_Port, HALL_PHASE_C_Pin);
      hall.isrCntr++;
      hall.speedE = foc1.pll.SpeedPll;
      Hall_update(&hall);
      // calc flux observer
      foc1.flux.i_alpha = foc1.data.isa;
      foc1.flux.i_beta = foc1.data.isb;
      foc1.flux.v_alpha = foc1.volt.usa;
      foc1.flux.v_beta = foc1.volt.usb;
      foc1.calc.flux(&foc1.flux);
      // calc speed PLL
      foc1.pll.AngleRaw = hall.angle; //foc1.smo.Theta; //foc1.flux.phase;
      foc1.calc.pll(&foc1.pll);
      // calc Total current, Electrical torque and Power
      foc1.data.iAvg = sqrtf(SQ(foc1.lpf_id.out) + SQ(foc1.lpf_iq.out));
      foc1.data.Te = 1.5f * foc1.config.pp * (foc1.flux.fluxLeakage * foc1.data.isq);
      //foc1.data.Te = 1.5f * foc1.config.pp * (foc1.flux.fluxLeakage * foc1.data.isq + (foc1.config.Ld - foc1.config.Lq) * foc1.data.isd * foc1.data.isq);
      foc1.data.Pe = foc1.data.Te * (foc1.pll.SpeedPll / foc1.config.pp);
      // Check protection
      foc1.prot.currPhU = foc1.data.isa;
      foc1.prot.currPhV = foc1.data.isb_tmp;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // apply voltage to motor and pwm driver
      bldc1.cmpr0 = foc1.svgen.Ta;
      bldc1.cmpr1 = foc1.svgen.Tb;
      bldc1.cmpr2 = foc1.svgen.Tc;
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(foc1.svgen.Ta * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(foc1.svgen.Tb * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(foc1.svgen.Tc * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      // calc BLDC motor model
      //ModelBLDC_Calc(&bldc1);
      // call datalogger
      dataLog.in1 = bldc1.tetaR;
      dataLog.in2 = foc1.smo.tmp_Theta; //foc1.smo.Theta;
      dataLog.in3 = foc1.smo.Theta;
      dataLog.in4 = foc1.smo.tmp_Ebeta;
      datalogCalc(&dataLog);
      break;
    case PARAM_ID:
      // Angle generator
      foc1.data.angle += foc1.data.freqStep * foc1.data.freq;
      foc1.data.angle = (foc1.data.angle > MF_PI) ? -MF_PI : foc1.data.angle;
      if (foc1.paramIdState == Enter)
      {
        foc1.data.isrCntr2 = 0;
        foc1.data.idRef = 5.0f;
        foc1.data.id_IdAmpl = 0.0f;
        foc1.data.id_UdAmpl = 0.0f;
        foc1.data.freq = 2000.0f;
        foc1.pi_id.Kp = foc1.pi_iq.Kp = 0.001f;
        foc1.pi_id.Ki = foc1.pi_iq.Ki = 1.0f;
        foc1.pi_id.Kc = foc1.pi_iq.Kc = 0.5f;
        dataLog.psc = 1000;
        dataLog.trigger = 1;
        foc1.paramIdState = Rs;
        ledDelay = 0.1f;
      }

      if (foc1.paramIdState == Rs)
      {
        // calc sin/cos
        foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(0.0f)));
        foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(0.0f)));
        // clarke transform for phase currents
        if (foc1.config.sim == 1)
        {
          foc1.data.isa = bldc1.isPhaseA;
          foc1.data.isb = _1DIV_SQRT3 * bldc1.isPhaseA + _2DIV_SQRT3 * bldc1.isPhaseB;
          foc1.volt.DcBusVolt = bldc1.udc;
        }
        else if (foc1.config.sim == 0)
        {
          foc1.data.isa = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
          foc1.data.isb_tmp = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
          foc1.data.isb = _1DIV_SQRT3 * foc1.data.isa + _2DIV_SQRT3 * foc1.data.isb_tmp;
          foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
        }
        // Filtered data
        foc1.lpf_id.in = foc1.data.isd;
        foc1.lpf_iq.in = foc1.data.isq;
        foc1.lpf_vdc.in = foc1.volt.DcBusVolt;
        LPF_calc(&foc1.lpf_id);
        LPF_calc(&foc1.lpf_iq);
        LPF_calc(&foc1.lpf_vdc);
        //foc1.data.isd = lpf_id.out;
        //foc1.data.isq = lpf_iq.out;
        foc1.volt.DcBusVolt = foc1.lpf_vdc.out;
        // PARK transform for phase currents
        foc1.data.isd = foc1.data.isa * foc1.data.cosTheta + foc1.data.isb * foc1.data.sinTheta;
        foc1.data.isq = -foc1.data.isa * foc1.data.sinTheta + foc1.data.isb * foc1.data.cosTheta;
        // calc PI current controllers for D/Q axis
        foc1.pi_id.Ref = foc1.data.idRef;
        foc1.pi_iq.Ref = 0.0f;
        foc1.pi_id.Fdb = foc1.data.isd;
        foc1.pi_iq.Fdb = foc1.data.isq;
        foc1.calc.pi_reg(&foc1.pi_id);
        foc1.calc.pi_reg(&foc1.pi_iq);
        // Saturation
        foc1.volt.usd = foc1.pi_id.Out;
        foc1.volt.usq = foc1.pi_iq.Out;
        foc1.volt.vMagMax = foc1.volt.dutyMax * SQRT3_BY_2 * foc1.volt.DcBusVolt; // 0.66666 => 2/3
        utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.vMagMax);
        foc1.volt.usd = foc1.volt.usd / (foc1.volt.DcBusVolt * _2DIV_SQRT3);
        foc1.volt.usq = foc1.volt.usq / (foc1.volt.DcBusVolt * _2DIV_SQRT3);
        // calc voltages
        foc1.volt.MfuncV1 = foc1.svgen.Ta;
        foc1.volt.MfuncV2 = foc1.svgen.Tb;
        foc1.volt.MfuncV3 = foc1.svgen.Tc;
        foc1.calc.volt(&foc1.volt);
        // foc1.volt.usd = foc1.volt.usa * foc1.data.cosTheta + foc1.volt.usb * foc1.data.sinTheta;
        // foc1.volt.usq = -foc1.volt.usa * foc1.data.sinTheta + foc1.volt.usb * foc1.data.cosTheta;
        //calc IPARK transform (IPARK output -> SVGEN input)
        // foc1.svgen.usa = foc1.pi_id.Out * foc1.data.cosTheta - foc1.pi_iq.Out * foc1.data.sinTheta;
        // foc1.svgen.usb = foc1.pi_iq.Out * foc1.data.cosTheta + foc1.pi_id.Out * foc1.data.sinTheta;
        foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
        foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
        // calc space-vector generator
        foc1.calc.svgen(&foc1.svgen);
        // calc Rs
        if (foc1.data.isrCntr2 >= 2000) // wait 100 ms, until current is stable, for avoid #QNAN
        {
          foc1.data.id_UdErr = foc1.config.Rds_on * fabsf(foc1.data.isd) + ((foc1.config.deadTime * 1e-9f) * foc1.volt.DcBusVolt); // fetDrop * Idc + deadtime * udc
          foc1.data.id_Rs = (foc1.pi_id.Out - foc1.data.id_UdErr) / foc1.data.isd;
          foc1.paramIdState = (foc1.data.isrCntr2 >= 30000) ? Ld : Rs;
          dataLog.trigger = 1;
        }
      }
      if (foc1.paramIdState == Ld)
      {
        foc1.data.udRef = (foc1.data.idRef * foc1.data.id_Rs * 1.05f);
        // calc sin/cos
        foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(0.f)));
        foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(0.f)));
        // calc UdRef
        foc1.data.id_Ud = _IQtoF(utSinAbs(_IQ(foc1.data.angle))) * foc1.data.udRef;
        foc1.data.id_Uq = 0.f;
        // clarke transform for phase currents
        if (foc1.config.sim == 1)
        {
          foc1.data.isa = bldc1.isPhaseA;
          foc1.data.isb = _1DIV_SQRT3 * bldc1.isPhaseA + _2DIV_SQRT3 * bldc1.isPhaseB;
          foc1.volt.DcBusVolt = bldc1.udc;
        }
        else if (foc1.config.sim == 0)
        {
          foc1.data.isa = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
          foc1.data.isb_tmp = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
          foc1.data.isb = _1DIV_SQRT3 * foc1.data.isa + _2DIV_SQRT3 * foc1.data.isb_tmp;
          foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
        }
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
        // Saturation
        foc1.volt.usd = foc1.data.id_Ud;
        foc1.volt.usq = foc1.data.id_Uq;
        foc1.volt.vMagMax = foc1.volt.dutyMax * SQRT3_BY_2 * foc1.volt.DcBusVolt; // 0.66666 => 2/3
        utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.vMagMax);
        foc1.volt.usd = foc1.volt.usd / (foc1.volt.DcBusVolt * _2DIV_SQRT3);
        foc1.volt.usq = foc1.volt.usq / (foc1.volt.DcBusVolt * _2DIV_SQRT3);
        // calc voltages
        foc1.volt.MfuncV1 = foc1.svgen.Ta;
        foc1.volt.MfuncV2 = foc1.svgen.Tb;
        foc1.volt.MfuncV3 = foc1.svgen.Tc;
        foc1.calc.volt(&foc1.volt);
        //calc IPARK transform (IPARK output -> SVGEN input)
        foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
        foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
        // calc space-vector generator
        foc1.calc.svgen(&foc1.svgen);
        // identify Ls
        if (foc1.data.isrCntr2 >= 50000)
        {
          foc1.data.id_UdAmpl = (foc1.data.id_Ud > foc1.data.id_UdAmpl) ? foc1.data.id_Ud : foc1.data.id_UdAmpl;
          foc1.data.id_IdAmpl = (foc1.data.isd > foc1.data.id_IdAmpl) ? foc1.data.isd : foc1.data.id_IdAmpl; // Calc Id DC component
          //foc1.data.id_UdErr = foc1.config.Rds_on * fabsf(foc1.data.id_IdAmpl) + (foc1.config.deadTime * foc1.volt.DcBusVolt); // fetDrop * Idc + deadtime * udc
          dataLog.trigger = 1;
          foc1.data.id_Zs = foc1.data.id_UdAmpl / foc1.data.id_IdAmpl;                                    // Z = Ud / Id
          foc1.data.id_Xs = sqrtf(foc1.data.id_Zs * foc1.data.id_Zs - foc1.data.id_Rs * foc1.data.id_Rs); // XL = sqrt(Z^2 - R^2)
          foc1.data.id_Ls = foc1.data.id_Xs / (M_2PI * foc1.data.freq);                                   // L = XL / (2*pi*f)
          foc1.data.id_Ld = foc1.data.id_Ls;
        }
        if (foc1.data.isrCntr2 >= 70000)
        {
          foc1.paramIdState = Lq;
          foc1.data.id_UdAmpl = foc1.data.id_IdAmpl = 0.f;
        }
      }
      if (foc1.paramIdState == Lq)
      {
        //foc1.data.udRef = (10.0f * foc1.data.id_Rs) / (foc1.volt.DcBusVolt * _2DIV_SQRT3); // (Imax/2 * Rs) / (Udc * 2/sqrt(3))
        foc1.data.udRef = (foc1.data.idRef * foc1.data.id_Rs * 1.05f);
        // calc sin/cos
        foc1.data.sinTheta = _IQtoF(utSinAbs(_IQ(0.f)));
        foc1.data.cosTheta = _IQtoF(utCosAbs(_IQ(0.f)));
        // calc UdRef
        foc1.data.id_Ud = 0.f;
        foc1.data.id_Uq = (foc1.data.isrCntr2 >= 75000) ? _IQtoF(utSinAbs(_IQ(foc1.data.angle))) * foc1.data.udRef : 0.f;
        // clarke transform for phase currents
        if (foc1.config.sim == 1)
        {
          foc1.data.isa = bldc1.isPhaseA;
          foc1.data.isb = _1DIV_SQRT3 * bldc1.isPhaseA + _2DIV_SQRT3 * bldc1.isPhaseB;
          foc1.volt.DcBusVolt = bldc1.udc;
        }
        else if (foc1.config.sim == 0)
        {
          foc1.data.isa = (adcData.ph_u * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrA;
          foc1.data.isb_tmp = (adcData.ph_v * 0.0002442f * foc1.config.adcFullScaleCurrent) - foc1.data.offsetCurrB;
          foc1.data.isb = _1DIV_SQRT3 * foc1.data.isa + _2DIV_SQRT3 * foc1.data.isb_tmp;
          foc1.volt.DcBusVolt = (float)adcData.v_dc * 0.0002442f * foc1.config.adcFullScaleVoltage;
        }
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
        // Saturation
        foc1.volt.usd = foc1.data.id_Ud;
        foc1.volt.usq = foc1.data.id_Uq;
        foc1.volt.vMagMax = foc1.volt.dutyMax * SQRT3_BY_2 * foc1.volt.DcBusVolt; // 0.66666 => 2/3
        utils_saturate_vector_2d((float *)&foc1.volt.usd, (float *)&foc1.volt.usq, foc1.volt.vMagMax);
        foc1.volt.usd = foc1.volt.usd / (foc1.volt.DcBusVolt * _2DIV_SQRT3);
        foc1.volt.usq = foc1.volt.usq / (foc1.volt.DcBusVolt * _2DIV_SQRT3);
        // calc voltages
        foc1.volt.MfuncV1 = foc1.svgen.Ta;
        foc1.volt.MfuncV2 = foc1.svgen.Tb;
        foc1.volt.MfuncV3 = foc1.svgen.Tc;
        foc1.calc.volt(&foc1.volt);
        // foc1.volt.usd = foc1.volt.usa * foc1.data.cosTheta + foc1.volt.usb * foc1.data.sinTheta;
        // foc1.volt.usq = -foc1.volt.usa * foc1.data.sinTheta + foc1.volt.usb * foc1.data.cosTheta;
        //calc IPARK transform (IPARK output -> SVGEN input)
        foc1.svgen.usa = foc1.volt.usd * foc1.data.cosTheta - foc1.volt.usq * foc1.data.sinTheta;
        foc1.svgen.usb = foc1.volt.usq * foc1.data.cosTheta + foc1.volt.usd * foc1.data.sinTheta;
        // calc space-vector generator
        foc1.calc.svgen(&foc1.svgen);
        // identify Ls
        if (foc1.data.isrCntr2 >= 90000)
        {
          //foc1.data.id_UdAmpl = foc1.data.udRef * bldc1.udc * 1.154700538f;                                // Calc Ud DC component
          foc1.data.id_UdAmpl = (foc1.data.id_Uq > foc1.data.id_UdAmpl) ? foc1.data.id_Uq : foc1.data.id_UdAmpl;
          foc1.data.id_IdAmpl = (foc1.data.isq > foc1.data.id_IdAmpl) ? foc1.data.isq : foc1.data.id_IdAmpl; // Calc Id DC component
          //foc1.data.id_UdErr = foc1.config.Rds_on * fabsf(foc1.data.id_IdAmpl) + (foc1.config.deadTime * foc1.volt.DcBusVolt); // fetDrop * Idc + deadtime * udc
          dataLog.trigger = 1;
          //foc1.data.id_Zs = (foc1.data.id_UdAmpl - foc1.data.id_UdErr) / foc1.data.id_IdAmpl;             // Z = Ud / Id
          foc1.data.id_Zs = foc1.data.id_UdAmpl / foc1.data.id_IdAmpl;                                    // Z = Ud / Id
          foc1.data.id_Xs = sqrtf(foc1.data.id_Zs * foc1.data.id_Zs - foc1.data.id_Rs * foc1.data.id_Rs); // XL = sqrt(Z^2 - R^2)
          foc1.data.id_Ls = foc1.data.id_Xs / (M_2PI * foc1.data.freq);                                   // L = XL / (2*pi*f)
          foc1.data.id_Lq = foc1.data.id_Ls;
        }
        foc1.paramIdState = (foc1.data.isrCntr2 >= 110000) ? Cmplt : Lq;
      }
      if (foc1.paramIdState == Cmplt)
      {
        /* Clear PWM signals, calc DQ-cureents PI gains, calc observer gains */
        foc1.svgen.Ta = 0.0f;
        foc1.svgen.Tb = 0.0f;
        foc1.svgen.Tc = 0.0f;
        foc1.data.idRef = 0.f;
        foc1.data.freq = 0.1f;
        foc1.data.udRef = 0.f;
        // calc D-Q axis current PI controllers gains
        // float tc = 0.001f; // sec
        // foc1.pi_id.Kp = foc1.pi_iq.Kp = foc1.data.id_Ls * (1.f / tc); // Ls * bw
        // foc1.pi_id.Ki = foc1.pi_iq.Ki = foc1.data.id_Rs * (1.f / tc) * foc1.config.tS; // Rs * bw, bw = 1.0 / tc, Ki = 𝜔𝑐𝑅 × 𝑇𝑆
        float wcc = (0.05f * foc1.config.pwmFreq) * M_2PI;
        foc1.pi_id.Kp = foc1.pi_iq.Kp = (foc1.data.id_Ld * wcc) * 0.5f;
        foc1.pi_id.Ki = foc1.pi_iq.Ki = foc1.data.id_Rs * wcc * foc1.config.tS * 0.5f; // Rs * bw, bw = 1.0 / tc, Ki = 𝜔𝑐𝑅 × 𝑇𝑆
        foc1.pi_id.Kc = 1.f / foc1.pi_id.Kp;
        foc1.pi_iq.Kc = 1.f / foc1.pi_iq.Kp;
        /* z = 0.707, bw=500 rad/s, Kp = (2*z*bw*Ld-Rs), taui = Kp/(Ld*bw*bw), Ki = Kp/taui  */
        // float zeta = 0.707f; // -3db
        // float bw = 500.f;    // rad/s
        // foc1.pi_id.Kp = foc1.pi_iq.Kp = 2.f * zeta * bw * foc1.data.id_Ld - foc1.data.id_Rs;
        // foc1.pi_id.Ki = foc1.pi_iq.Ki = foc1.pi_id.Kp / (foc1.pi_id.Kp * foc1.config.tS / (foc1.data.id_Ld * bw * bw));
        // observer gains calc
        foc1.smo.Fsmopos = exp((-foc1.data.id_Rs / foc1.data.id_Ld) * foc1.config.tS);
        foc1.smo.Gsmopos = (1.0f / foc1.data.id_Rs) * (1.0f - foc1.smo.Fsmopos);
        foc1.smo.Kslide = 0.55f;
        foc1.flux.R = foc1.data.id_Rs * 1.5f;
        foc1.flux.L = foc1.data.id_Ld * 1.5f;
        foc1.config.Rs = foc1.data.id_Rs;
        foc1.config.Ld = foc1.data.id_Ld;
        foc1.config.Lq = foc1.data.id_Lq;

        /**
         * TODO: ADD BEEP */
        foc1.driveState = STOP;
      }
      // Check protection
      foc1.prot.currPhU = foc1.data.isa;
      foc1.prot.currPhV = foc1.data.isb_tmp;
      foc1.prot.udc = foc1.volt.DcBusVolt;
      foc1.prot.tempPcb = ntcPcb.temp;
      foc1.calc.prot(&foc1.prot);
      foc1.driveState = (foc1.prot.protFlag != 0) ? FAULT : foc1.driveState;
      // apply voltage to motor
      bldc1.cmpr0 = foc1.svgen.Ta;
      bldc1.cmpr1 = foc1.svgen.Tb;
      bldc1.cmpr2 = foc1.svgen.Tc;
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(foc1.svgen.Ta * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (uint32_t)(foc1.svgen.Tb * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (uint32_t)(foc1.svgen.Tc * foc1.data.halfPwmPeriod + foc1.data.halfPwmPeriod));
      // calc BLDC motor model
      ModelBLDC_Calc(&bldc1);
      // call datalogger
      dataLog.in1 = foc1.data.id_Ud;
      dataLog.in2 = foc1.data.id_Uq;
      dataLog.in3 = foc1.data.isd;
      dataLog.in4 = foc1.data.isq;
      datalogCalc(&dataLog);
      break;

    case FAULT:
      // Disable PWM and blink fault LED
      LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | LL_HRTIM_OUTPUT_TC1 |
                                         LL_HRTIM_OUTPUT_TC2 | LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2);

      if (foc1.data.isrCntr0 == 1)
        LL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
      break;

    default:
      break;
    }
  }
  stop_cc = DWT->CYCCNT;          /* hold CPU cycles counter */
  execTime1 = stop_cc - start_cc; /* calc CPU utilization time */
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
    //((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;

    //     /* uwFrequency computation
    //     TIM1 counter clock = (System Clock) */
    //     uwFrequency = ( HAL_RCC_GetSysClockFreq()  ) / uwIC2Value;
  }
  else
  {
    pwmInputValue = 0;
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}