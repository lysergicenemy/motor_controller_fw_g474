# Firmware for electronic speed controller (BLDC/PMSM)

# Operation modes:
* Field-Oriented control (current control, speed control)
* 6-step sensorless control (zero crossing + bemf integration) 
* Motor parameters identification (Rs, Ld, Lq, Hall sensors table, Kv / fluxLinkage)
* Virtual motor model simulation
* Sensored (Hall based software encoder) and sensorless (flux observer) operation
* DC-Motor mode

## STM32G474 MCU features:
* Precisely PWM generation using High-resolution timer (HRTIM1)
* Fast trigonometry computation using hardware CORDIC
* Hardware cycle-by-cycle current limiting in BLDC mode using internal comparators
* Accelerating control loop calculation time using CCMRAM

# Firmware config:
```c
// connect hardware specific fuctions to library specific functions (main.c)
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
// set all necessary parameters in configuration structure (main.c) 
  foc1.config
// set all remaining parameters in init function (foc_ctrl.h) 
  Foc_Init()
```