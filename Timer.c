 /*
 --------------------------------------------------------
Purpose: Purpose: Sets up timer0 CTC-mode and PWM at 2kHz
 Input: The compare value is initialized in main
 Uses: includes Timer.h
 Author: Mathias Columbus Dyrløv Madsen
 University: DTU
 Version: 1.2
 Creation Date and year:12/06-2024 (European calender)
 Updated to this version, adding the PWM - 09/01-2025
 -----------------------------------------------------
 */
 #include <avr/io.h>
void initTimer(unsigned int compareValue) {
    TCCR0B |= (1 << CS01);    // Prescaler set to 8
    TCCR0A |= (1 << WGM01);   // CTC mode
    OCR0A = compareValue;     // Compare value
    TIMSK0 |= (1 << OCIE0A);  // Enable interrupt on Compare Match A
    TCNT0 = 0;
    
}

void setup_pwm_2khz() {
    // Set pin OC1A (PB1) as output
    DDRB |= (1 << PB1);  // Set PB1 (pin 9 on Arduino) as output

    // Configure Timer1 for Fast PWM mode
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Non-inverting mode, WGM bits for Fast PWM
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // WGM bits, Prescaler = 8

    // Set TOP value for 2 kHz PWM frequency
    ICR1 = 999;

    // Set initial duty cycle to 50%
    OCR1A = 499;  // 50% duty cycle

    TIMSK1 |= (1 << OCIE1A);  // Enable interrupt on OCR1A compare match
}
 