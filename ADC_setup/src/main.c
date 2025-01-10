//Includes libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ADC.h"
#include "Timer.h"
#include <math.h>
#include "I2C.h"  //include library for i2c driver
#include "ssd1306.h" //include display driver
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define N 32  // Size of the array
volatile bool buffer_full = false;

uint8_t adc_samples[N];  // Array to store ADC samples



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
}

ISR(TIMER1_COMPA_vect) {
    // Reset Timer0 counter, should be running simultanously, this is a safeguard
    TCNT0 = 0;
  
}
ISR(TIMER0_COMPA_vect){
    

}


ISR(ADC_vect) {
    static int adc_index = 0; 
    if (buffer_full==false)
    {
    // Store the ADC result into the array
    adc_samples[adc_index] = ADCH;  // Read ADC value (ADCL + ADCH)
    
    // Increment the index
    adc_index++;
    }
    // If we've filled the array with N samples
    if (adc_index == N) {
        // Reset the index to start overwriting the samples
        adc_index = 0;
        buffer_full = true;
        
        // Optionally, process the array here (e.g., compute average, etc.)
       // process_adc_samples();  // A custom function to process the array
    }
    
}

void uint8_to_string(uint8_t value, char *str) {
    // Convert an unsigned 8-bit integer to a string
    char temp[4];  // Buffer for up to 3 digits + null terminator
    int i = 0;

    do {
        temp[i++] = (value % 10) + '0';  // Extract the least significant digit
        value /= 10;                    // Remove the least significant digit
    } while (value > 0);

    // Reverse the string to get the correct order
    int j = 0;
    while (i > 0) {
        str[j++] = temp[--i];
    }
    str[j] = '\0';  // Null-terminate the string
}

int main(){
     DDRB|= (1<<PB5);
    initTimer(124);
    setup_pwm_2khz();
    initADC(0);
    _i2c_address = 0X78; // write address for i2c interface
	I2C_Init();  //initialize i2c interface to display
	InitializeDisplay(); //initialize  display
	//print_fonts();  //for test and then exclude the  clear_display(); call
	clear_display();   //use this before writing you own text
    sei();
    
    char test[] = "test";
    ADCSRA |= (1 << ADSC);  // Start conversion

 while (1) {
        if(buffer_full==true){
            PORTB ^= (1<<PB5);
        sendStrXY(test,0,0);
        // Delay for 1000ms
        _delay_ms(1000);  // Adjust delay as needed
        buffer_full = false;
        }
    }
}