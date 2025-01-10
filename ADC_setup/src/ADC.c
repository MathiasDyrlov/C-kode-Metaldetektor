/*---------------------------------------------------------
Purpose: Sets up desired ADC and contains a function that can scale from resolution to resolution
Input: channel for the ADC
Output: the number scaled via the scalefunction
Uses: includes ADC.h
Author: Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1
Date and year:02/05-2024 (European calender)
-----------------------------------------------------*/

#include <avr/io.h>
#include "ADC.h"

void initADC(uint8_t channel) {
    // Select the ADC input channel (e.g., ADC0 for A0 pin)
    ADMUX = (ADMUX & ~(0x1F)) | (channel);  // Set channel (0-7 for ADC0-ADC7)
    
    // Set the reference voltage to AVCC (5V)
    ADMUX = (ADMUX & ~(1 << REFS1)) | (1 << REFS0);

    // Set ADC prescaler to 128 for 125 kHz ADC clock (16 MHz / 128)
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // Enable ADC, Auto Triggering, and ADC Interrupt
    ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADIE);

    // Set the auto trigger source to Timer0 Compare Match A (ADTS1 = 1, ADTS0 = 1)
    ADCSRB |= (1 << ADTS1) | (1 << ADTS0);  // Select Timer0 Compare Match A as the trigger source

    // Start the ADC conversion (will trigger automatically)
    ADCSRA |= (1 << ADSC);
    DIDR0 = (1<<channel);
}

