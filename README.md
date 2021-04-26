# Firmware for BLDC motor controller
* STM32G474CE controller used HRTIM for precisely PWM phase currents generation and dual-ADC mode for fast currents sampling

# Modes:
* Field-Oriented control
* 6-step control
* Motor parameters identification
* Motor model simulation
* Sensored (Hall based software encoder) and sensorless (experimental) operation

# Firmware config:

* set all necessary parameters in main.c - foc1.config
* set all remaining (if need) parameters in foc.h -> Foc_Init() function