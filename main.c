/*
 --------------------------------------------------------------
Purpose: This Program is used to both run and compute results
on a metal detector. A Pwm output will run through the PA-step
into the coils and back into the ADC input. 
Both the output and ADC-sampling frequency is in phase.
Here the MCU will run a simplified DTF on the ADC-input and
output result to a screen via I2C.

Input: AC input at 2kHz from filter past the coils
Output: PWM-signal for running the coils. 

Uses: "UART.h", "ADC.h", "Timer.h", "I2C.h", "ssd1306.h"

Author: Mathias Columbus Dyrløv Madsen, 
I2C.h and ssd1306 are both written by Ole Schultz

University: DTU
Version: 1.3
Creation Date and year:07/01-2025 (European calender)
Updated to this version 17/01-2025
 --------------------------------------------------------------
 */

//Includes libraries

#include "ADC.h"
#include "Timer.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <math.h> // Used to call sqrt and atan2

#include "I2C.h"  //include library for i2c driver. 
#include "ssd1306.h" //include display driver

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define N 32  // Size of the ADC window. IE number of tabs

//volatile variables to be processed 
volatile bool buffer_full = false;
volatile bool do_sample = false;
volatile bool button_pressed = false;
volatile bool first_press = false;



uint8_t adc_samples[N];  // Array to store ADC samples

//init function for external interrupt
void initButton() {
    // Configure INT1 to trigger on a falling edge
    EICRA |= (1 << ISC11);    // Falling edge on INT1
    EICRA &= ~(1 << ISC10);

    // Enable external interrupt for INT1
    EIMSK |= (1 << INT1);

    // Enable pull-up resistor on PD3 (INT1)
    PORTD |= (1 << PD3);

    // Default set to zero, just added for clarity. PD3 creates a PullUp resistor
    DDRD &= ~(1 << PD3);
}

//interrupt for the calbration button
ISR(INT1_vect){
   
    button_pressed = true; // interrupt to be processed in main scope
    PORTB ^= (1 << PB5);  // Toggle PB5 to indicate processing, LED will toggle
    //alternativily toggle in ADC_vect ISR to get an indication for when each sample is done
}

//PWM timer interrupt, used to Phase allign ADC sampling
ISR(TIMER1_COMPA_vect) {
    
TCNT0 = 0; // Reset Timer0 counter, ensuring phase allignment with pwm output 
    if (buffer_full==false)
    {
        do_sample = true; //This ensures that a new adc ample window starts at the same point every time
        
        
    } 

    

 

   
}

//only used to test adc timing
ISR(TIMER0_COMPA_vect){
     //PORTB ^= (1 << PB5);  // Toggle PB5 to indicate processing
    
}

//Interrupt for ADC, activates once 1 sample is written in ADCH
ISR(ADC_vect) {
    static int adc_index = 0; 
    if (do_sample==true)
    {
    // Store the ADC result into the array
    adc_samples[adc_index] = ADCH;  // Read ADC value (ADCL + ADCH)
    // Increment the index
    adc_index++;
    }
    // If index is full, buffer flag will be flipped
    if (adc_index == N) {
        adc_index = 0;// Reset the index to start overwriting the samples
        buffer_full = true;
        do_sample = false;
    }
    
}

//not used
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

//Used instead of sprintf, faster and uses less memory
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
void calculate_dft(uint8_t *buffer, uint8_t size, float *amp_point, float *phase_point) {
    float real_sum = 0.0;
    float imag_sum = 0.0;
    //static cosine and sine since 8kHz sampling a 2kHz input with phase allignment
    const int cosine[] =  {1,0,-1,0};
    const int sine[] =  {0,1,0,-1};
    // Compute DFT for each frequency bin N/4R
    
        for (uint8_t n = 0; n < size; n++) {
            // Calculate the real and imaginary sums for each frequency bin
            
            real_sum += buffer[n] * cosine[n%4];
            imag_sum -= buffer[n] * sine[n%4];  // Negative due to Eulers identity
        }

    // Compute amplitude and phase for the frequency bin
    *amp_point = sqrt(real_sum * real_sum + imag_sum * imag_sum)/size;
    *phase_point = (atan2(imag_sum, real_sum))* 180 / M_PI;
    
}

int main(){
     DDRB|= (1<<PB5); //output for LED for testing
     //init functions
    initTimer(124);
    setup_pwm_2khz();
    initButton();
    initADC(1);
    //i2c to run display
    _i2c_address = 0X78; // write address for i2c interface
	I2C_Init();  //initialize i2c interface to display
	InitializeDisplay(); //initialize  display
	clear_display();   //use this before writing you own text
    sei();
    char buffer_amp[10], buffer_phase[10];  // Buffer to store ADC value as a string

    
    
    float amplitude = 0;  // To store the calculated amplitude
    float phase = 0.0;       // To store the calculated phase 
    float amplitude_calibrate = 0;  // To store the calibrated amplitude
    float phase_calibrate = 0.0;       // To store the calibrated phase

    //test case - same array used in matlab simulation
    uint8_t test_values[N] = {42, 183, 91, 7, 154, 33, 212, 5, 98, 17, 57, 103, 206, 239, 71, 29, 158, 46, 201, 114, 34, 77, 185, 220, 14, 88, 129, 160, 250, 9, 35, 142};

    sendStrXY("Press button to calibrate",0,0);


    

 while (1) {
    if (button_pressed==true)
    {   
        
        amplitude_calibrate=amplitude;
        phase_calibrate=phase;
        first_press = true;
        button_pressed = false;
        
    }
    
        if (buffer_full == true) {
           
            // Calculate amplitude and phase using DFT
            calculate_dft(test_values, N, &amplitude, &phase);

            // Display amplitude
            amplitude=amplitude-amplitude_calibrate; // Calibration subtracted from current result
            float_to_string(amplitude, buffer_amp, 2);  // Convert amplitude to string
            

            // Display phase in degrees
            phase=phase-phase_calibrate;
            float_to_string(phase, buffer_phase, 2);  // Convert phase to string

            if (1)//first_press==true)// Only prints to screen once the calibration button has been pressed once
            {
            sendStrXY("Amp:", 3, 0);
            sendStrXY(buffer_amp, 4, 0);  // Display amplitude on OLED at (x=1, y=0)
            sendStrXY("Phase:", 6, 0);
            sendStrXY(buffer_phase, 7, 0);  // Display phase on OLED at (x=4, y=0)
            }

            buffer_full = false;        // Flips flag, do_sample is flipped in timer1 the 
        }
    }
}