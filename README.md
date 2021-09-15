# Firmware for electronic speed controller (BLDC/PMSM)

# Operation modes:
* Field-Oriented control (current control, speed control)
* 6-step sensorless control (zero crossing + bemf integration) 
* Motor parameters identification (Rs, Ld, Lq, Hall sensors table, Kv / fluxLinkage)
* Virtual motor model simulation
* Sensored (Hall based software encoder) and sensorless (flux observer) operation

## STM32G474CE MCU features:
* Precisely PWM generation using High-resolution timer (HRTIM1)
* Fast trigonometry computation using hardware CORDIC
* Hardware cycle-by-cycle current limiting in BLDC mode using internal comparators
* Accelerating control loop calculation time using CCMRAM

# Firmware config:
```c
* //set all necessary parameters in configuration structure 
  foc1.config
* //set all remaining parameters in init function 
  Foc_Init()
```