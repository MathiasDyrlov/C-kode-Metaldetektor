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
  //this should be set to OCR0A if the PA-step flips the output signal. 
}

//only used to test adc timing
ISR(TIMER0_COMPA_vect){
     PORTB ^= (1 << PB5);  // Toggle PB5 to indicate processing

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

void float_to_string(float value, char *str, uint8_t precision) {
    // Extract integer part
    int int_part = (int)value;

    // Extract fractional part
    float fractional_part = value - int_part;

    // Convert integer part to string
    char temp[12];  // Temporary buffer for the integer part
    int i = 0;
    do {
        temp[i++] = (int_part % 10) + '0';
        int_part /= 10;
    } while (int_part > 0);

    // Reverse the integer part into the output string
    while (i > 0) {
        *str++ = temp[--i];
    }

    // Add decimal point
    if (precision > 0) {
        *str++ = '.';
    }

    // Convert fractional part to string
    for (uint8_t p = 0; p < precision; p++) {
        fractional_part *= 10;
        int digit = (int)fractional_part;
        *str++ = digit + '0';
        fractional_part -= digit;
    }

    // Null-terminate the string
    *str = '\0';
}







// Function to calculate amplitude and phase using DFT on a uint8 buffer
void calculate_dft(uint8_t *buffer, uint8_t size, uint32_t *amplitude, float *phase) {
    float real_sum = 0.0;
    float imag_sum = 0.0;
    const int cosine[] =  {1,0,-1,0};
    const int sine[] =  {0,1,0,-1};
    // Compute DFT for each frequency bin N/4R
    
        for (uint8_t n = 0; n < size; n++) {
            // Calculate the real and imaginary sums for each frequency bin
            
            real_sum += buffer[n] * cosine[n%4];
            imag_sum -= buffer[n] * sine[n%4];  // Negative due to complex conjugate
        }

    // Compute amplitude and phase for the first frequency bin (k=1)
    *amplitude = (uint32_t)sqrt(real_sum * real_sum + imag_sum * imag_sum);
    *phase = atan2(imag_sum, real_sum);
}


int main(){
     DDRB|= (1<<PB5);
    initTimer(124);
    setup_pwm_2khz();
    initADC(1);
    _i2c_address = 0X78; // write address for i2c interface
	I2C_Init();  //initialize i2c interface to display
	InitializeDisplay(); //initialize  display
	//print_fonts();  //for test and then exclude the  clear_display(); call
	clear_display();   //use this before writing you own text
    sei();
    char buffer[10];  // Buffer to store ADC value as a string

    
    
    uint32_t amplitude1 = 0;  // To store the calculated amplitude (for DFT approximation)
    float phase1 = 0.0;       // To store the calculated phase (for DFT approximation)

    //test cases
    uint8_t test_values[N] = {42, 183, 91, 7, 154, 33, 212, 5, 98, 17, 57, 103, 206, 239, 71, 29, 158, 46, 201, 114, 34, 77, 185, 220, 14, 88, 129, 160, 250, 9, 35, 142};




    

 while (1) {
        if (buffer_full == true) {
           



            // Calculate amplitude and phase using DFT
            calculate_dft(test_values, N, &amplitude1, &phase1);

            // Display amplitude
            float_to_string(amplitude1, buffer, 2);  // Convert amplitude to string
            sendStrXY("Amp:", 0, 0);
            sendStrXY(buffer, 1, 0);  // Display amplitude on OLED at (x=1, y=0)

            // Display phase in degrees
            float_to_string(phase1 * 180 / M_PI, buffer, 2);  // Convert radians to degrees
            sendStrXY("Phase:", 3, 0);
            sendStrXY(buffer, 4, 0);  // Display phase on OLED at (x=4, y=0)

            buffer_full = false;        // Reset buffer full flag
            ADCSRA |= (1 << ADSC);      // Restart ADC conversion
        }
    }
}