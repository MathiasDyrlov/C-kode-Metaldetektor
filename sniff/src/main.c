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

//Includes libraries created for the project
#include "ADC.h"
#include "Timer.h"
#include "Test_debugging.h"
//avr libraries for using register names matching the 328p datasheet
#include <avr/io.h>
#include <avr/interrupt.h>

//include libraries created by Ole Schultz for driving the OLED-screen
#include "I2C.h"  //include library for I2C driver. 
#include "ssd1306.h" //include display driver

//include standard libraries
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h> // Used to call sqrt and atan2

#define N 128  // Size of the ADC window. IE number of tabs


// Global variables initialised
//volatile variables to be processed 
volatile bool buffer_full = false;
volatile bool button_pressed = false;
volatile bool do_sample = false;

volatile uint16_t timer_counter = 0; // Timer0 counter for debounce
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
ISR(INT1_vect) {
    static uint16_t last_debounce_time = 0;

    // Check debounce interval
    if ((timer_counter - last_debounce_time) > 2000) { // debounce
        button_pressed = true;
        PORTB ^= (1<<PB5);
        last_debounce_time = timer_counter; // Update last debounce time
    }
}

    //test
    ISR(TIMER0_COMPA_vect) {
    static uint8_t toggle_count = 0; // Counter for downscaling 8 kHz to 2 kHz
    ADCSRA |= (1 << ADSC);
    toggle_count++;
    
    // Toggle PB1 at 2 kHz (every 4 ISR calls)
    if (toggle_count == 2) { // Rising edge
        PORTB |= (1 << PB1); // Set PB1 high
        do_sample = true;    // Trigger sampling on the rising edge
    } else if (toggle_count == 4) { // Falling edge
        PORTB &= ~(1 << PB1); // Set PB1 low
        toggle_count = 0;     // Reset the counter
    }

    timer_counter++; //for debounce
}
    


//Interrupt for ADC, activates once 1 sample is written in ADCH
ISR(ADC_vect) {
    
    static int adc_index = 0; 
    // Store the ADC result into the array
    if (do_sample==true)
    {
    
    adc_samples[adc_index] = ADCH;  // Read ADC value (ADCL + ADCH)
    
    // Increment the index
    adc_index++;
    }
    //adc_index++;
    // If index is full, buffer flag will be flipped
    if (adc_index == N) {
        do_sample = false;
        buffer_full = true;
        
        adc_index = 0;// Reset the index to start overwriting the samples
        
    }
}
    
void clear_buffer(uint8_t *buffer, size_t size) {
    memset(buffer, '\0', size); // Set all elements to null terminator
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
    *amp_point = sqrt(real_sum * real_sum + imag_sum * imag_sum);
    *phase_point = (atan2(imag_sum, real_sum))* 180 / M_PI;
    
}

int main(){
     DDRB|= (1<<PB5); //output for LED for testing
     DDRB |= (1 << PB1); // PWM output
     clear_buffer(adc_samples,N);
     //init functions
    initTimer(124);
    //setup_pwm_2khz();
    initButton();
    initADC(1);
    //i2c to run display
    _i2c_address = 0X78; // write address for i2c interface
	I2C_Init();  //initialize i2c interface to display
	InitializeDisplay(); //initialize  display
	clear_display();   //use this before writing you own text
    sei();
    char buffer_amp[10] = {0,0,0,0,0,0,0,0,0,0};
    char buffer_phase[10] = {0,0,0,0,0,0,0,0,0,0};  // Buffer to store ADC value as a string

    
    
    float amplitude_raw = 0.0;  // To store the calculated amplitude
    float phase_raw = 0.0;       // To store the calculated phase 
    float amplitude_calibrateval = 0.0, amp_calibrated = 0.0;  // To store the calibrated amplitude
    float phase_calibrateval = 0.0, phase_calibrated = 0.0;       // To store the calibrated phase
    float phase_acc = 0.0, amp_acc = 0.0;

    //test case - same array used in matlab simulation
    //uint8_t test_values[N] = {42, 183, 91, 7, 154, 33, 212, 5, 98, 17, 57, 103, 206, 239, 71, 29, 158, 46, 201, 114, 34, 77, 185, 220, 14, 88, 129, 160, 250, 9, 35, 142};

    sendStrXY("Press button to calibrate",0,0);
    sendStrXY("Amp:", 3, 0);
    sendStrXY("Phase:", 6, 0);
    

 while (1) {
    
    
    if (button_pressed==true)
    {   
        
        amplitude_calibrateval=amplitude_raw;
        phase_calibrateval=phase_raw;
        button_pressed = false;
        
    }
    
        if (buffer_full == true) {
            
            //apply Blackman Window
            for (uint8_t n = 0; n < N; n++) {
                float window = 0.42 
                   - 0.5 * cos(2 * M_PI * n / (N - 1)) 
                   + 0.08 * cos(4 * M_PI * n / (N - 1)); // Blackman window
                adc_samples[n] = adc_samples[n] * window;
            }
            
            // Calculate amplitude and phase using DFT
            calculate_dft(adc_samples, N, &amplitude_raw, &phase_raw);
            amp_calibrated=amplitude_raw-amplitude_calibrateval; // Calibration subtracted from current result
            phase_calibrated=phase_raw-phase_calibrateval;

            amp_acc = (0.8 * amp_acc) + (0.2 * (amp_calibrated)); // moving average filter
            phase_acc = (0.8 * phase_acc) + (0.2 * (phase_calibrated));
            // Display amplitude
            
            dtostrf(amp_acc, 6, 1, buffer_amp); // Converts to "123.45"
            dtostrf(phase_acc, 6, 2, buffer_phase); // Converts to "123.45"
            
            // Display phase in degrees
            sendStrXY(buffer_amp, 4, 0);  // Display amplitude on OLED at (x=4, y=0)
            sendStrXY(buffer_phase, 7, 0);  // Display phase on OLED at (x=7, y=0)
            
            buffer_full = false;        // Flips flag, do_sample is flipped in timer1 the 
        }
    }
}