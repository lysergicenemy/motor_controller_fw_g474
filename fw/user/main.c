/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "hrtim.h"
#include "lptim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//-------- BOARD PARAM -----------------------//
//#define USE_HSI 1
#define ADC_FULL_SCALE_DC_LINK_VOLT 82.5f // VCC * K_Vdiv = 3.3 * 25
#define DC_LINK_VOLT_SCALE 0.02014652f    // ( Vadc / adcResol ) * K_Vdiv = ( 3.3 / 4095 ) * 25
#define F_SMP (340000000.0f / 8192.0f)    // HRTIM_CLK / (HRTIM_PERIOD * 2) - if used upDown counting mode
#define T_S (1.0f / F_SMP)
#define ADC_FULL_SCALE_CURRENT 41.25f // (VCC / 2) / ( R_SENSE * OPAMP_GAIN ) = 1.65 / (0.002 * 20)
#define PH_CURRENT_SCALE 0.02015144f  // ADC_FULL_SCALE_CURRENT / (adcResol / 2) = 41.25 / 2047

#define SPD_LOOP_PSC 8
#define ALIGN_CURRENT 0.05f // set align current in P.U.
// ----- drive state defs ------------//
#define STOP 0
#define ALIGN 1
#define ENCODER_CALIB 2
#define RUN_CURR_LOOP 3
#define RUN_SPD_LOOP 4
// ----- sensor type defs -------------//
#define HALL 0
#define ENCODER_ABZ 1
#define SENSORLESS 2

//------- MOTOR PARAM -----------------//
#define POLES 2.0f      // magnets poles in rotor
#define R_S 0.12479f    // Ohm
#define L_S 0.00021033f // H
#define FLUX 0.5305f    // V/HZ
#define BASE_FREQ 100.0f
#define BASE_VOLT ADC_FULL_SCALE_DC_LINK_VOLT * 0.57735026918963f // DC_LINK_VOLT / sqrt(3)

#define DATALOG_STERAM_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile int32_t offsetCurrentA = 0;
volatile int32_t offsetCurrentB = 0;
volatile int32_t offsetCurrentC = 0;
volatile int32_t currentAbuff = 0;
volatile int32_t currentBbuff = 0;
volatile int32_t currentCbuff = 0;
volatile uint16_t currFilterNmb = 0;

volatile uint16_t dcLinkVolt = 0;

volatile const int32_t CORDIC_ZTBL[14] = {6434, 3798, 2007, 1019, 511, 256, 128, 64, 32, 16, 8, 4, 2, 1};

volatile _iq SpeedRef = _IQ(0.0);
volatile _iq VdTesting = _IQ(0.0);  // Vd reference (pu)
volatile _iq VqTesting = _IQ(0.05); // Vq reference (pu)
volatile uint8_t driveState = STOP;
volatile uint8_t sensorType = HALL;
volatile uint8_t resetRequest = 0;
volatile uint8_t currLpf_en = 1; // enable digital low-pass filter for phase currents

volatile _iq IdRef = 0;
volatile _iq IqRef = 0;

volatile _iq IdRefPr = 0;
volatile _iq IqRefPr = 0;

volatile uint8_t SpeedLoopCount = 1;
volatile _iq speedRampSlope = _IQ(0.00075);

volatile int32_t virtualTim0 = 0;
volatile int32_t virtualTim1 = 0;
volatile uint32_t pwmInputPeriod = 0;
volatile uint32_t pwmInputValue = 0;

volatile uint8_t hall_a;
volatile uint8_t hall_b;
volatile uint8_t hall_c;
volatile uint8_t hall_state;
volatile uint8_t hall_statePr;
volatile _iq hall_time;
volatile _iq hall_timeNew;
volatile _iq hall_timeOld;
volatile _iq hall_tmp;
volatile _iq hall_period;
volatile _iq hall_deltaTheta;
volatile _iq hall_angle;

/* Phase shift calc: (((2 * pi * 2^GLOBAL_Q) / 360) * shiftDeg) / 2^GLOBAL_Q = 
= (((6.2831 * 16777216) / 360) * 15) / 16777216 = 0.2617 */
volatile _iq phaseShift = _IQ(0.2617); // 0.2617 = 15 degree shift

/*---------- DataLog struct --------------*/
typedef struct
{
  volatile uint8_t trigger : 1;
  volatile uint8_t recordCmplt : 1;

  volatile uint32_t frameCntr;
  volatile uint32_t isrCntr;
  volatile int32_t buffChannel1[DATALOG_STERAM_SIZE];
  volatile int32_t buffChannel2[DATALOG_STERAM_SIZE];
  volatile int32_t buffChannel3[DATALOG_STERAM_SIZE];
  volatile int32_t buffChannel4[DATALOG_STERAM_SIZE];
  volatile int32_t channel1;
  volatile int32_t channel2;
  volatile int32_t channel3;
  volatile int32_t channel4;
} dataLogVars_t;

dataLogVars_t dataLog;
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;
SVGEN svgen1 = SVGEN_DEFAULTS;
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
RAMPGEN rg1 = RAMPGEN_DEFAULTS;
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;
SPEED_ESTIMATION speed3 = SPEED_ESTIMATION_DEFAULTS;
PI_CONTROLLER pi_id = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iq = PI_CONTROLLER_DEFAULTS;
TPidReg3 pi_spd = PIDREG3_DEFAULTS;
SMOPOS smo1 = SMOPOS_DEFAULTS;
SMOPOS_CONST smo1_const = SMOPOS_CONST_DEFAULTS;

FILTER_DATA lpf1 = FILTER_DEFAULTS;
FILTER_DATA lpf2 = FILTER_DEFAULTS;
FILTER_DATA lpf3 = FILTER_DEFAULTS;
FILTER_DATA lpf4 = FILTER_DEFAULTS;

encoder_t encoder;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void adc_start(void);
void hrtim_start(void);
void tim7_start(void);
void datalogCalc(dataLogVars_t *p, _iq *ch1, _iq *ch2, _iq *ch3, _iq *ch4);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  SET_BIT(FLASH->OPTR, FLASH_OPTR_nSWBOOT0); // decouple PB8 pin from BOOT0
  CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT0); // set BOOT0 to "0"

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral 
  */
  LL_PWR_DisableDeadBatteryPD();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_HRTIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  if (sensorType == ENCODER_ABZ)
    MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize the RAMPGEN module
  rg1.StepAngleMax = _IQ(BASE_FREQ * T_S);

  // Initialize RMPCNTL module
  /* Exmpl: Td = RampDelayMax * T_S = 50 * 0.000024 = 0.0012 s =>
    => Tramp = ((TargetValue - initial value ) / 0.0000305) * Td = (IQ(1) / 0.000305) * 0.0012 = 3.934 s */
  rc1.RampDelayMax = 1;

  // Initialize the PI module for Id
  /* Keep in mind sqrt(Id^2 + Iq^2) must be less then 1.0 !!!
   * PI current calc eq (absolute form):
   * bw = 1.0 / tc;
   * kp = l * bw
   * ki = r * bw */

  //pi_id.Kp = _IQ(((L_S * M_2PI) * (F_SMP / 20)) * (ADC_FULL_SCALE_CURRENT / ADC_FULL_SCALE_DC_LINK_VOLT));
  //pi_id.Ki = _IQ((R_S / L_S) * T_S * 5);

  pi_id.Kp = _IQ(7.5);
  pi_id.Ki = _IQ(0.0125);
  pi_id.Umax = _IQ(0.45);
  pi_id.Umin = _IQ(-0.45);

  // Initialize the PI module for Iq
  //pi_iq.Kp = _IQ(((L_S * M_2PI) * (F_SMP / 30)) * (ADC_FULL_SCALE_CURRENT / ADC_FULL_SCALE_DC_LINK_VOLT));
  //pi_iq.Ki = _IQ((R_S / L_S) * T_S * 5);
  pi_iq.Kp = _IQ(7.5);
  pi_iq.Ki = _IQ(0.0125);
  pi_iq.Umax = _IQ(0.8);
  pi_iq.Umin = _IQ(-0.8);

  // Initialize the PI module for speed
  pi_spd.Kp_reg3 = _IQ(0.5);
  pi_spd.Ki_reg3 = _IQ(0.001);
  pi_spd.Kc_reg3 = _IQ(0.5); //_IQ((0.001 * T_S) / 0.5);// ki*t/kp
  pi_spd.Kd_reg3 = _IQ(0.025);
  pi_spd.pid_out_max = _IQ(0.3);  //pi_iq.Umax;
  pi_spd.pid_out_min = _IQ(-0.3); //pi_iq.Umin;

  // Initialize the SPEED_FR module ENCODER based speed calculation
  speed1.K1 = _IQ(1 / (BASE_FREQ * 2 * M_PI * T_S));
  speed1.K2 = _IQ(1 / (1 + T_S * 2 * M_PI * 5)); // Low-pass cut-off frequency
  speed1.K3 = _IQ(1) - speed1.K2;
  speed1.BaseRpm = 120 * (BASE_FREQ / POLES);

  // Initialize the SPEED_EST module SMOPOS based speed calculation
  speed3.K1 = _IQ21(1 / (BASE_FREQ * T_S));
  speed3.K2 = _IQ(0.7); //_IQ(1 / (1 + T_S * 2 * M_PI * 30));  // Low-pass cut-off frequency
  speed3.K3 = _IQ(1) - speed3.K2;
  speed3.BaseRpm = 120 * (BASE_FREQ / POLES);

  // Initialize the SMOPOS constant module
  smo1_const.Rs = R_S;
  smo1_const.Ls = L_S;
  smo1_const.Ib = ADC_FULL_SCALE_CURRENT;
  smo1_const.Vb = 18.0f; //BASE_VOLT;
  smo1_const.Ts = T_S;
  SMO_CONST_MACRO(smo1_const);

  smo1.Fsmopos = _IQ(smo1_const.Fsmopos); //_IQ(0.5);//
  smo1.Gsmopos = _IQ(smo1_const.Gsmopos); //_IQ(-0.05);//
  smo1.Kslide = _IQ(0.05308703613);
  smo1.Kslf = _IQ(0.001057073975);

  // Initialize the Low-pass filter
  lpf1.Tf = 0.005f;
  lpf1.Ts = T_S;
  Filter_Init(&lpf1);
  // Initialize the Low-pass filter
  lpf2.Tf = 0.005f;
  lpf2.Ts = T_S;
  Filter_Init(&lpf2);
  // Initialize the Low-pass filter
  lpf3.Tf = 0.005f;
  lpf3.Ts = T_S;
  Filter_Init(&lpf3);
  // Initialize the Low-pass filter
  lpf4.Tf = 0.002f;
  lpf4.Ts = T_S;
  Filter_Init(&lpf4);

  // Initialize encoder module
  encoder.resolution = 12; // 1024 ppr * 4
  encoder.dir = 1;

  /* -----------------------------------------
   * Start all periph */
  hrtim_start();
  if (sensorType == ENCODER_ABZ)
    lptim_start((1 << encoder.resolution) - 1);

  MX_TIM2_Init();
  tim2_start();
  adc_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (resetRequest == 1)
      NVIC_SystemReset();
    /* BEEP */
    // if (driveState == STOP)
    // {
    //   LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_PRESCALERRATIO_DIV2);
    //   LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_PRESCALERRATIO_DIV2);
    //   LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_PRESCALERRATIO_DIV2);
    //   LL_mDelay(500);
    //   LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_PRESCALERRATIO_DIV4);
    //   LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_PRESCALERRATIO_DIV4);
    //   LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_PRESCALERRATIO_DIV4);
    //   LL_mDelay(500);
    // }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_8);
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while (LL_RCC_HSE_IsReady() != 1)
  {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLP_DIV_3);
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

/* USER CODE BEGIN 4 */

void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
  if (LL_ADC_IsActiveFlag_JEOS(ADC1))
  {
    LL_ADC_ClearFlag_JEOS(ADC1);
    virtualTim1++;
    if (virtualTim1 >= 415039)
      virtualTim1 = 0; // reset every 10 seconds (10 / T_S = 415039)

    switch (driveState)
    {
    case STOP:
    {
      /* In STOP case: set dutyClycle to 50%, calc offset currents and reset all modules */

      offsetCurrentA = _IQ13toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1)); // Phase A curr.offset
      offsetCurrentB = _IQ13toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2)); // Phase B curr.offset
      offsetCurrentC = _IQ13toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3)); // Phase C curr.offset

      lpf1.x = offsetCurrentA;
      lpf2.x = offsetCurrentB;
      lpf3.x = offsetCurrentC;

      Filter_Execute(&lpf1);
      Filter_Execute(&lpf2);
      Filter_Execute(&lpf3);

      offsetCurrentA = _IQtoIQ13(lpf1.y);
      offsetCurrentB = _IQtoIQ13(lpf2.y);
      offsetCurrentC = _IQtoIQ13(lpf3.y);

      rc1.TargetValue = rc1.SetpointValue = 0;
      rg1.Freq = rg1.Out = 0;
      pi_id.v1 = pi_id.ui = pi_id.i1 = 0;
      pi_iq.v1 = pi_iq.ui = pi_iq.i1 = 0;
      pi_spd.ui_reg3 = 0;
      LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);

      if (sensorType == HALL)
      {
        hall_a = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_5);
        hall_b = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_7);
        hall_c = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_6);

        if (hall_a == 0 && hall_b == 1 && hall_c == 0)
          hall_state = 0; // 0(360) deg. 2PI
        else if (hall_a == 0 && hall_b == 1 && hall_c == 1)
          hall_state = 1; // 60 deg.  2PI/3 / 2
        else if (hall_a == 0 && hall_b == 0 && hall_c == 1)
          hall_state = 2; // 120 deg. 2PI/3
        else if (hall_a == 1 && hall_b == 0 && hall_c == 1)
          hall_state = 3; // 180 deg. PI
        else if (hall_a == 1 && hall_b == 0 && hall_c == 0)
          hall_state = 4; // 240 deg. PI - (2PI/3 * 2)
        else if (hall_a == 1 && hall_b == 1 && hall_c == 0)
          hall_state = 5; // 300 deg. PI - 2PI/3

        if (hall_state != hall_statePr)
        {
          switch (hall_state)
          {
          case 0:
            hall_angle = _IQ(0.0);
            break;
          case 1:
            hall_angle = _IQ(1.047198);
            break;
          case 2:
            hall_angle = _IQ(2.094395);
            break;
          case 3:
            hall_angle = _IQ(M_PI);
            break;
          case 4:
            hall_angle = _IQ(-1.047198);
            break;
          case 5:
            hall_angle = _IQ(-2.094395);
            break;
          default:
            break;
          }
          hall_time = virtualTim1; // time in T_S
          hall_timeOld = hall_timeNew;
          hall_timeNew = hall_time;
          hall_tmp = hall_timeNew - hall_timeOld;
          hall_period = (hall_tmp < 0) ? 415039 + hall_tmp : hall_tmp;

          hall_deltaTheta = _IQ(1.047198) / hall_period; // delta(rad) = 60(deg) / peroid(T_S)
        }
        hall_statePr = hall_state;

        hall_angle += hall_deltaTheta;
        if (hall_angle < -_IQ(M_PI))
          hall_angle = hall_angle + _IQ(M_2PI);
        else if (hall_angle > _IQ(M_PI))
          hall_angle = hall_angle - _IQ(M_2PI);
      }

      // ------------------------------------------------------------------------------
      //  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
      // ------------------------------------------------------------------------------
      volt1.DcBusVolt = _IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1)); // DC Bus voltage meas.
      volt1.MfuncV1 = svgen1.Ta;
      volt1.MfuncV2 = svgen1.Tb;
      volt1.MfuncV3 = svgen1.Tc;
      PHASEVOLT_MACRO(volt1);

      /* go to ALIGN case if enable is done and V_DC grater than 15V (15/82.5 = 0.181) */
      if (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_Pin) == 1 && volt1.DcBusVolt >= _IQ(0.181))
      {
        if (encoder.calibFlag == 0)
        {
          driveState = ALIGN;
          lpf1.y = lpf2.y = lpf3.y = 0;
        }
        else
          driveState = RUN_SPD_LOOP;
      }

      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, 2047);
    }
    break;
    case ALIGN:
    {
      /* In ALIGN case set encoder TIM = 0, set reference frame angle = 0
         and slowly increase Idref to align rotor in fixed position */

      clarke1.As = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - offsetCurrentA)); // Phase A curr.
      clarke1.Bs = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - offsetCurrentB)); // Phase B curr.
      clarke1.Cs = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) - offsetCurrentC)); // Phase C curr.

      if (currLpf_en == 1)
      {
        lpf1.x = clarke1.As;
        lpf2.x = clarke1.Bs;
        lpf3.x = clarke1.Cs;

        Filter_Execute(&lpf1);
        Filter_Execute(&lpf2);
        Filter_Execute(&lpf3);

        clarke1.As = lpf1.y;
        clarke1.Bs = lpf2.y;
        clarke1.Cs = lpf3.y;
      }

      CLARKE_3CUR_MACRO(clarke1);

      /* reset math blocks */
      rc1.TargetValue = rc1.SetpointValue = 0;
      rg1.Freq = rg1.Out = rg1.Angle = 0;
      pi_spd.ui_reg3 = 0;

      encoder.thetaRaw = (LPTIM1->CNT); // just for view in freemaster
      park1.Angle = 0;
      LL_LPTIM_ResetCounter(LPTIM1);
      /* wait untill align process is done */
      virtualTim0++;
      if (virtualTim0 >= 29166) // 0.7s / T_S = 29166
      {
        virtualTim0 = 0;
        driveState = ENCODER_CALIB;
      }
      /* check conditions which give posibility to drive */
      if (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_Pin) == 0 || volt1.DcBusVolt < _IQ(0.181))
        driveState = STOP;
      // ------------------------------------------------------------------------------
      //  Connect inputs of the PARK module and call the park trans. macro
      // ------------------------------------------------------------------------------
      park1.Alpha = clarke1.Alpha;
      park1.Beta = clarke1.Beta;
      park1.Sine = _IQsin(park1.Angle);
      park1.Cosine = _IQcos(park1.Angle);
      PARK_MACRO(park1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PID_REG3 module and call the PID IQ controller macro
      // ------------------------------------------------------------------------------
      pi_iq.Ref = IqRef;
      pi_iq.Fdb = park1.Qs;
      PI_MACRO(pi_iq);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PI module and call the PID ID controller macro
      // ------------------------------------------------------------------------------
      IdRef = _IQ(ALIGN_CURRENT);
      pi_id.Ref = ramper(IdRef, pi_id.Ref, _IQ(0.0000025));
      pi_id.Fdb = park1.Ds;
      PI_MACRO(pi_id);

      //-------- Trigger for dataloger ------------------//

      if (IqRef != IqRefPr || IdRef != IdRefPr)
        dataLog.trigger = 1;

      IqRefPr = IqRef;
      IdRefPr = IdRef;

      // ------------------------------------------------------------------------------
      //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
      // ------------------------------------------------------------------------------
      ipark1.Ds = pi_id.Out;
      ipark1.Qs = pi_iq.Out;

      ipark1.Sine = park1.Sine;
      ipark1.Cosine = park1.Cosine;
      IPARK_MACRO(ipark1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
      // ------------------------------------------------------------------------------
      volt1.DcBusVolt = _IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1)); // DC Bus voltage meas.
      volt1.MfuncV1 = svgen1.Ta;
      volt1.MfuncV2 = svgen1.Tb;
      volt1.MfuncV3 = svgen1.Tc;
      PHASEVOLT_MACRO(volt1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
      // ------------------------------------------------------------------------------
      svgen1.Ualpha = ipark1.Alpha;
      svgen1.Ubeta = ipark1.Beta;
      SVGENDQ_MACRO(svgen1);

      /* check conditions which give posibility to drive */
      if (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_Pin) == 0 || volt1.DcBusVolt < _IQ(0.181))
      {
        driveState = STOP; // back to STOP case
        IdRef = IqRef = 0; // reset speedRef
      }

      //------------------------------------------------------------------------------
      // Convert IQ24 value to 12-bit PWM value
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (svgen1.Ta >> 13) + 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (svgen1.Tb >> 13) + 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (svgen1.Tc >> 13) + 2047);

      //------------------------------------------------------------------------------
      // Call datalogger
      datalogCalc(&dataLog, &pi_id.Ref, &pi_id.Fdb, &clarke1.Alpha, &clarke1.Beta);
    }
    break;

    case ENCODER_CALIB:
    {
      /* In ENCODER_CALIB case slowly rotate rotor until encoder index will be found
         after that save encoder counter value like "offsetAngle" and reset it */

      clarke1.As = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - offsetCurrentA)); // Phase A curr.
      clarke1.Bs = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - offsetCurrentB)); // Phase B curr.
      clarke1.Cs = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) - offsetCurrentC)); // Phase C curr.

      if (currLpf_en == 1)
      {
        lpf1.x = clarke1.As;
        lpf2.x = clarke1.Bs;
        lpf3.x = clarke1.Cs;

        Filter_Execute(&lpf1);
        Filter_Execute(&lpf2);
        Filter_Execute(&lpf3);

        clarke1.As = lpf1.y;
        clarke1.Bs = lpf2.y;
        clarke1.Cs = lpf3.y;
      }

      CLARKE_3CUR_MACRO(clarke1);

      // ------------------------------------------------------------------------------
      //  Calc encoder angle offset
      if (sensorType == ENCODER_ABZ)
      {

        LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);

        encoder.angleOffset = (LPTIM1->CNT) + (1 << (encoder.resolution - 1)); // offset + pi
        //posEncoderOffset = (LPTIM1->CNT) + 2047;
      }
      if (sensorType == HALL)
      {
        hall_a = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_5);
        hall_b = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_7);
        hall_c = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_6);
      }
      // ------------------------------------------------------------------------------
      //  Connect inputs of the RAMP GEN module and call the ramp generator macro
      // ------------------------------------------------------------------------------
      rg1.Freq = _IQ(0.05);
      RG_MACRO(rg1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PARK module and call the park trans. macro
      // ------------------------------------------------------------------------------
      park1.Angle = rg1.Out;

      park1.Alpha = clarke1.Alpha;
      park1.Beta = clarke1.Beta;
      park1.Sine = _IQsin(park1.Angle);
      park1.Cosine = _IQcos(park1.Angle);
      PARK_MACRO(park1);

      // Clear integral part of speed PI when speedRef = 0, or we not in speed loop mode
      pi_spd.ui_reg3 = 0;

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PID_REG3 module and call the PID IQ controller macro
      // ------------------------------------------------------------------------------
      pi_iq.Ref = IqRef;
      pi_iq.Fdb = park1.Qs;
      PI_MACRO(pi_iq);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PI module and call the PID ID controller macro
      // ------------------------------------------------------------------------------
      pi_id.Ref = IdRef; //ramper(IdRef, pi_id.Ref, _IQ(0.00001));
      pi_id.Fdb = park1.Ds;
      PI_MACRO(pi_id);

      //-------- Trigger for dataloger ------------------//

      if (IqRef != IqRefPr || IdRef != IdRefPr)
        dataLog.trigger = 1;

      IqRefPr = IqRef;
      IdRefPr = IdRef;

      // ------------------------------------------------------------------------------
      //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
      // ------------------------------------------------------------------------------
      ipark1.Ds = pi_id.Out;
      ipark1.Qs = pi_iq.Out;

      ipark1.Sine = park1.Sine;
      ipark1.Cosine = park1.Cosine;
      IPARK_MACRO(ipark1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
      // ------------------------------------------------------------------------------
      volt1.DcBusVolt = _IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1)); // DC Bus voltage meas.
      volt1.MfuncV1 = svgen1.Ta;
      volt1.MfuncV2 = svgen1.Tb;
      volt1.MfuncV3 = svgen1.Tc;
      PHASEVOLT_MACRO(volt1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
      // ------------------------------------------------------------------------------
      svgen1.Ualpha = ipark1.Alpha;
      svgen1.Ubeta = ipark1.Beta;
      SVGENDQ_MACRO(svgen1);

      /* check conditions which give posibility to drive */
      if (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_Pin) == 0 || volt1.DcBusVolt < _IQ(0.181))
      {
        driveState = STOP; // back to STOP case
        IdRef = IqRef = 0; // reset speedRef
      }

      //------------------------------------------------------------------------------
      // Convert IQ24 value to 12-bit PWM value
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (svgen1.Ta >> 13) + 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (svgen1.Tb >> 13) + 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (svgen1.Tc >> 13) + 2047);

      //------------------------------------------------------------------------------
      // Call datalogger
      datalogCalc(&dataLog, &pi_id.Ref, &pi_id.Fdb, &clarke1.Alpha, &clarke1.Beta);
    }
    break;

    case RUN_CURR_LOOP:
    {
      /* Basic current loop: control DQ-axis currents */

      clarke1.As = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - offsetCurrentA)); // Phase A curr.
      clarke1.Bs = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - offsetCurrentB)); // Phase B curr.
      clarke1.Cs = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) - offsetCurrentC)); // Phase C curr.

      if (currLpf_en == 1)
      {
        lpf1.x = clarke1.As;
        lpf2.x = clarke1.Bs;
        lpf3.x = clarke1.Cs;

        Filter_Execute(&lpf1);
        Filter_Execute(&lpf2);
        Filter_Execute(&lpf3);

        clarke1.As = lpf1.y;
        clarke1.Bs = lpf2.y;
        clarke1.Cs = lpf3.y;
      }

      CLARKE_3CUR_MACRO(clarke1);

      // ------------------------------------------------------------------------------
      //  Calc encoder angle
      // ------------------------------------------------------------------------------
      /* small debounce for remove jitter */
      if (encoder.index == 1)
        virtualTim0++;
      if (virtualTim0 >= 10)
      {
        LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);
        encoder.index = 0;
        virtualTim0 = 0;
      }

      encoder.counter = LPTIM1->CNT;
      encoder_calc(&encoder);

      // ------------------------------------------------------------------------------
      //  Calc speed from encoder
      // ------------------------------------------------------------------------------

      speed1.ElecTheta = encoder.theta; //posEncoder;
      SPEED_FR_MACRO(speed1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the RMP module and call the ramp control macro
      // ------------------------------------------------------------------------------
      rc1.TargetValue = SpeedRef;
      RC_MACRO(rc1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the RAMP GEN module and call the ramp generator macro
      // ------------------------------------------------------------------------------
      rg1.Freq = rc1.SetpointValue;
      RG_MACRO(rg1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PARK module and call the park trans. macro
      // ------------------------------------------------------------------------------
      park1.Angle = encoder.theta; //posEncoder;

      park1.Alpha = clarke1.Alpha;
      park1.Beta = clarke1.Beta;
      park1.Sine = _IQsin(park1.Angle);
      park1.Cosine = _IQcos(park1.Angle);
      PARK_MACRO(park1);

      // Clear integral part of speed PI when speedRef = 0, or we not in speed loop mode
      pi_spd.ui_reg3 = 0;

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PID_REG3 module and call the PID IQ controller macro
      // ------------------------------------------------------------------------------
      pi_iq.Ref = IqRef;
      pi_iq.Fdb = park1.Qs;
      PI_MACRO(pi_iq);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PI module and call the PID ID controller macro
      // ------------------------------------------------------------------------------
      pi_id.Ref = IdRef; //ramper(IdRef, pi_id.Ref, _IQ(0.00001));
      pi_id.Fdb = park1.Ds;
      PI_MACRO(pi_id);

      /* Decoupling cross links betwen Vd and Vq */
      // _iq speedRad = _IQmpy(SpeedRef, _IQ(15.91549));
      // _iq dec_vd = _IQmpy(pi_iq.Fbk, _IQmpy(speedRad, _IQ(L_S)));
      // _iq dec_vq = _IQmpy(pi_id.Fbk, _IQmpy(speedRad, _IQ(L_S))) + _IQmpy(speedRad, _IQmpy(_IQ(FLUX), _IQ(0.1591549)));

      // pi_id.Out -= dec_vd;
      // pi_iq.Out += dec_vq;

      //-------- Trigger for dataloger ------------------//

      if (IqRef != IqRefPr || IdRef != IdRefPr)
        dataLog.trigger = 1;

      IqRefPr = IqRef;
      IdRefPr = IdRef;

      // ------------------------------------------------------------------------------
      //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
      // ------------------------------------------------------------------------------
      ipark1.Ds = pi_id.Out;
      ipark1.Qs = pi_iq.Out;

      ipark1.Sine = park1.Sine;
      ipark1.Cosine = park1.Cosine;
      IPARK_MACRO(ipark1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
      // ------------------------------------------------------------------------------
      volt1.DcBusVolt = _IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1)); // DC Bus voltage meas.
      volt1.MfuncV1 = svgen1.Ta;
      volt1.MfuncV2 = svgen1.Tb;
      volt1.MfuncV3 = svgen1.Tc;
      PHASEVOLT_MACRO(volt1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
      // ------------------------------------------------------------------------------
      svgen1.Ualpha = ipark1.Alpha;
      svgen1.Ubeta = ipark1.Beta;
      SVGENDQ_MACRO(svgen1);

      /* check conditions which give posibility to drive */
      if (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_Pin) == 0 || volt1.DcBusVolt < _IQ(0.181))
      {
        driveState = STOP; // back to STOP case
        IdRef = IqRef = 0; // reset speedRef
      }

      //------------------------------------------------------------------------------
      // Convert IQ24 value to 12-bit PWM value
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (svgen1.Ta >> 13) + 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (svgen1.Tb >> 13) + 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (svgen1.Tc >> 13) + 2047);

      //------------------------------------------------------------------------------
      // Call datalogger
      datalogCalc(&dataLog, &pi_id.Ref, &pi_id.Fdb, &pi_iq.Ref, &pi_iq.Fdb);
    }
    break;

    case RUN_SPD_LOOP:
    {
      clarke1.As = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - offsetCurrentA)); // Phase A curr.
      clarke1.Bs = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - offsetCurrentB)); // Phase B curr.
      clarke1.Cs = _IQmpy2(_IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) - offsetCurrentC)); // Phase C curr.

      if (currLpf_en == 1)
      {
        lpf1.x = clarke1.As;
        lpf2.x = clarke1.Bs;
        lpf3.x = clarke1.Cs;

        Filter_Execute(&lpf1);
        Filter_Execute(&lpf2);
        Filter_Execute(&lpf3);

        clarke1.As = lpf1.y;
        clarke1.Bs = lpf2.y;
        clarke1.Cs = lpf3.y;
      }

      CLARKE_3CUR_MACRO(clarke1);

      // ------------------------------------------------------------------------------
      //  Calc encoder angle
      // ------------------------------------------------------------------------------
      /* small debounce for remove jitter */
      if (encoder.index == 1)
        virtualTim0++;
      if (virtualTim0 >= 10)
      {
        LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);
        encoder.index = 0;
        virtualTim0 = 0;
      }

      encoder.counter = LPTIM1->CNT;
      encoder_calc(&encoder);

      // if (posEncoderIndex == 1) virtualTim0++;
      // if (virtualTim0 >= 10)
      // {
      //   LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);
      //   posEncoderIndex = 0;
      //   virtualTim0 = 0;
      // }
      // posEncoderRaw = (LPTIM1->CNT) + posEncoderOffset; // theta = theta - theta_error

      // if (posEncoderRaw < 0)
      //   posEncoderRaw = posEncoderRaw + (LPTIM1->ARR);
      // else if (posEncoderRaw > (LPTIM1->ARR))
      //   posEncoderRaw = posEncoderRaw - (LPTIM1->ARR);

      // posEncoder = _IQmpy(_IQ12toIQ(( (LPTIM1->ARR) - posEncoderRaw) - 2047), _IQ(M_2PI));

      // ------------------------------------------------------------------------------
      //  Calc speed from encoder
      speed1.ElecTheta = encoder.theta; //posEncoder;
      SPEED_FR_MACRO(speed1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the RMP module and call the ramp control macro
      // ------------------------------------------------------------------------------
      rc1.TargetValue = SpeedRef;
      RC_MACRO(rc1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PARK module and call the park trans. macro
      // ------------------------------------------------------------------------------
      park1.Angle = encoder.theta; //posEncoder;

      park1.Alpha = clarke1.Alpha;
      park1.Beta = clarke1.Beta;
      park1.Sine = _IQsin(park1.Angle);
      park1.Cosine = _IQcos(park1.Angle);
      PARK_MACRO(park1);

      // ------------------------------------------------------------------------------
      //    Connect inputs of the PI module and call the PID speed controller macro
      // ------------------------------------------------------------------------------
      SpeedRef = _IQ10toIQ(pwmInputValue);
      if (SpeedLoopCount == SPD_LOOP_PSC)
      {
        pi_spd.pid_ref_reg3 = ramper(SpeedRef, pi_spd.pid_ref_reg3, speedRampSlope);
        pi_spd.pid_fdb_reg3 = speed1.Speed;
        pid_reg3_calc(&pi_spd);
        SpeedLoopCount = 1;
      }
      else
        SpeedLoopCount++;

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PID_REG3 module and call the PID IQ controller macro
      // ------------------------------------------------------------------------------
      pi_iq.Ref = pi_spd.pid_out_reg3;
      pi_iq.Fdb = park1.Qs;
      PI_MACRO(pi_iq);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PI module and call the PID ID controller macro
      // ------------------------------------------------------------------------------
      pi_id.Ref = IdRef; //ramper(IdRef, pi_id.Ref, _IQ(0.00001));
      pi_id.Fdb = park1.Ds;
      PI_MACRO(pi_id);

      /* Decoupling cross links betwen Vd and Vq */
      // _iq speedRad = _IQmpy(SpeedRef, _IQ(15.91549));
      // _iq dec_vd = _IQmpy(pi_iq.Fbk, _IQmpy(speedRad, _IQ(L_S)));
      // _iq dec_vq = _IQmpy(pi_id.Fbk, _IQmpy(speedRad, _IQ(L_S))) + _IQmpy(speedRad, _IQmpy(_IQ(FLUX), _IQ(0.1591549)));

      // pi_id.Out -= dec_vd;
      // pi_iq.Out += dec_vq;

      //-------- Trigger for dataloger ------------------//

      if (IqRef != IqRefPr || IdRef != IdRefPr)
        dataLog.trigger = 1;

      IqRefPr = IqRef;
      IdRefPr = IdRef;

      // ------------------------------------------------------------------------------
      //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
      // ------------------------------------------------------------------------------
      ipark1.Ds = pi_id.Out;
      ipark1.Qs = pi_iq.Out;

      ipark1.Sine = park1.Sine;
      ipark1.Cosine = park1.Cosine;
      IPARK_MACRO(ipark1);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
      // ------------------------------------------------------------------------------
      volt1.DcBusVolt = _IQ12toIQ(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1)); // DC Bus voltage meas.
      volt1.MfuncV1 = svgen1.Ta;
      volt1.MfuncV2 = svgen1.Tb;
      volt1.MfuncV3 = svgen1.Tc;
      PHASEVOLT_MACRO(volt1);

      // ------------------------------------------------------------------------------
      //    Connect inputs of the SMO_POS module and call the sliding-mode observer macro
      // ------------------------------------------------------------------------------
      // smo1.Ialpha = clarke1.Alpha;
      // smo1.Ibeta = clarke1.Beta;
      // smo1.Valpha = volt1.Valpha;
      // smo1.Vbeta = volt1.Vbeta;
      // SMO_MACRO(smo1);
      // smo1.Theta = _IQmpy2(_IQ14toIQ(smo1.Theta));
      // /* Phase shift compensation */
      // smo1.Theta = smo1.Theta - phaseShift; // 15 degree shift
      // if (smo1.Theta > _IQ(M_PI))
      // {
      //   _iq tmp1 = smo1.Theta - _IQ(M_PI);
      //   smo1.Theta = _IQ(-M_PI) + tmp1;
      // }
      // if (smo1.Theta < _IQ(-M_PI))
      // {
      //   _iq tmp2 = smo1.Theta - _IQ(-M_PI);
      //   smo1.Theta = _IQ(M_PI) + tmp2;
      // }

      // ------------------------------------------------------------------------------
      //    Connect inputs of the SPEED_EST module and call the estimated speed macro
      // ------------------------------------------------------------------------------
      // speed3.EstimatedTheta = smo1.Theta;
      // SE_MACRO(speed3);

      // ------------------------------------------------------------------------------
      //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
      // ------------------------------------------------------------------------------
      svgen1.Ualpha = ipark1.Alpha;
      svgen1.Ubeta = ipark1.Beta;
      SVGENDQ_MACRO(svgen1);

      /* check conditions which give posibility to drive */
      if (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_Pin) == 0 || volt1.DcBusVolt < _IQ(0.181))
      {
        driveState = STOP; // back to STOP case
        SpeedRef = 0;      // reset speedRef
      }

      //------------------------------------------------------------------------------
      // Convert IQ24 value to 12-bit PWM value
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (svgen1.Ta >> 13) + 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, (svgen1.Tb >> 13) + 2047);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, (svgen1.Tc >> 13) + 2047);

      //------------------------------------------------------------------------------
      // Call datalogger
      //------------------------------------------------------------------------------
      datalogCalc(&dataLog, &pi_id.Ref, &pi_id.Fdb, &pi_iq.Ref, &pi_iq.Fdb);
    }
    break;

    default:
      break;
    }
  }
  /* USER CODE END ADC1_2_IRQn 0 */
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
    pwmInputValue = (pwmInputPeriod * 1023) / LL_TIM_IC_GetCaptureCH1(TIM2); //((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;

    //     /* uwFrequency computation
    //     TIM1 counter clock = (System Clock) */
    //     uwFrequency = ( HAL_RCC_GetSysClockFreq()  ) / uwIC2Value;
  }
  else
  {
    pwmInputValue = 0;
  }
}

void datalogCalc(dataLogVars_t *p, _iq *ch1, _iq *ch2, _iq *ch3, _iq *ch4)
{
  if (p->trigger >= 1)
  {
    p->buffChannel1[p->frameCntr] = *ch1; // var 1
    p->buffChannel2[p->frameCntr] = *ch2; // var 2
    p->buffChannel3[p->frameCntr] = *ch3; // var 3
    p->buffChannel4[p->frameCntr] = *ch4; // var 4

    p->frameCntr++;
    if (p->frameCntr >= (DATALOG_STERAM_SIZE - 1))
    {
      p->recordCmplt = 1;
      p->trigger = 0;
      p->frameCntr = 0;
    }
  }

  if (p->recordCmplt >= 1)
  {
    if (p->isrCntr >= 80)
    {
      p->channel1 = p->buffChannel1[p->frameCntr];
      p->channel2 = p->buffChannel2[p->frameCntr];
      p->channel3 = p->buffChannel3[p->frameCntr];
      p->channel4 = p->buffChannel4[p->frameCntr];

      p->isrCntr = 0;
      p->frameCntr++;
    }
    p->isrCntr++;
    if (p->frameCntr >= (DATALOG_STERAM_SIZE - 1))
    {
      p->recordCmplt = 0;
      p->frameCntr = 0;
      p->isrCntr = 0;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
