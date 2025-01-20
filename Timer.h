/*
 ---------------------------------------------------------
Purpose: Sets up timer0 CTC-mode and PWM at 2kHz
Uses: .h file for Timer.c, include in main.c
Author:Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1.2
Date and year:12/06-2024 (European calender)
Updated to current version: 15/01-2025
Updated to current version, adding the PWM - 09/01-2025
 ---------------------------------------------------------
 */
 #ifndef TIMER_H_
 #define TIMER_H_
 void initTimer(unsigned int compareValue);
 void setup_pwm_2khz();
 #endif /* TIMER_H_ */