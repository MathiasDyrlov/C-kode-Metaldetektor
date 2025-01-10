 /*--------------------------------------------------------
Purpose: Sets up timer
 Input: The compare value is initialized in main
 Uses: includes Timer1.h
 Author: Mathias Columbus Dyrløv Madsen
 University: DTU
 Version: 1.2
 Date and year:12/06-2024 (European calender)-----------------------------------------------------*/
 #include <avr/io.h>
void initTimer(unsigned int compareValue) {
    TCCR0B |= (1 << CS01);    // Prescaler set to 8
    TCCR0A |= (1 << WGM01);   // CTC mode
    OCR0A = compareValue;     // Compare value
    TIMSK0 |= (1 << OCIE0A);  // Enable interrupt on Compare Match A
    TCNT0 = 0;
    
}

 void init_timer0(unsigned int compareValue){
	 TCCR0B |= (1<<CS01); //Pre-scaling 8
	 TCCR0A |= (1<<WGM01); //CTC mode top value = OCR0A
	 OCR0A = compareValue; //compare match use 124 as a parameter if max frequency is 8kHz and 16MHz internal clock.
	 TIMSK0 |=(1<<OCIE0A); //Interrupt when TCNT0 = OCR0A value
	 TCNT0 = 0; // Reset the counter to ensure rising edge alignment
 }
 