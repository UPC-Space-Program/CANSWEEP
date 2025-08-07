/*
 * stepper.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Pau
 */

#include "stepper.h"
#include "encoder.h"
#include "main.h"
#include <math.h>
#include <strings.h>  // for ffs() function
// Motor parameters
// Motor configuration (change these for each motor)
#define STEPS_PER_REV       200         // 1.8° per step motor
#define MICROSTEPS          8           // Must match DRV8711 setting
#define GEAR_RATIO          1.0f        // Direct drive = 1.0

// Timer configuration
#define TIMER_CLOCK_HZ      16000000
#define MAX_FREQ_HZ         100000
#define MIN_FREQ_HZ         1

// Calculated constant
#define STEPS_PER_RADIAN    ((STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / (2.0f * M_PI))

// External hardware
extern TIM_HandleTypeDef htim1;

// State
static float current_velocity = 0;
static uint8_t current_microsteps = MICROSTEPS;

// Set timer frequency
static void set_timer_frequency(uint32_t freq_hz) {

    if (freq_hz > MAX_FREQ_HZ) freq_hz = MAX_FREQ_HZ;
    if (freq_hz < MIN_FREQ_HZ) { //velocitat 0
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        return;
    }
    
    uint16_t prescaler;
	uint32_t period;

	// Use fixed prescalers based on frequency range
	if (freq_hz < 100) {
		prescaler = 255;  // For 1-99 Hz
	} else if (freq_hz < 1000) {
		prescaler = 15;   // For 100-999 Hz
	} else {
		prescaler = 0;    // For 1000+ Hz
	}

	// Calculate period with the fixed prescaler
	period = (TIMER_CLOCK_HZ / ((prescaler + 1) * freq_hz)) - 1;

	// Clamp period to 16-bit
	if (period > 65535) period = 65535;

	__HAL_TIM_SET_PRESCALER(&htim1, prescaler);
	__HAL_TIM_SET_AUTORELOAD(&htim1, (uint16_t)period);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(period / 2));

	htim1.Instance->EGR = TIM_EGR_UG;
}

void stepper_init(void) {
    // DRV8711 already initialized in main.c
    // Just start PWM but with no pulses
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    set_timer_frequency(0);
}

void stepper_set_velocity(float rad_per_sec) {
    current_velocity = rad_per_sec;

    // Set direction
    HAL_GPIO_WritePin(DIR_AIN2_GPIO_Port, DIR_AIN2_Pin,
                      (rad_per_sec >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Calculate frequency with current microstep setting
    float steps_per_rad = (STEPS_PER_REV * current_microsteps * GEAR_RATIO) / (2.0f * M_PI);
    uint32_t freq_hz = (uint32_t)(fabsf(rad_per_sec) * steps_per_rad);

    set_timer_frequency(freq_hz);
}

float stepper_get_velocity(void) {
    return current_velocity;
}

void stepper_stop(void) {
    current_velocity = 0;
    set_timer_frequency(0);
}

void stepper_set_microsteps(uint8_t divider) {
	uint16_t ctrl_value;

	    switch(divider) {
	        case 1:   ctrl_value = 0x0C01; break;  // Full step 11 00 0 0000 001
	        case 2:   ctrl_value = 0x0C09; break;  // Half step 11 00 0 0001 001
	        case 4:   ctrl_value = 0x0C11; break;  // 1/4 step 11 00 0 0010 001
	        case 8:   ctrl_value = 0x0C19; break;  // 1/8 step 11 00 0 0011 001
	        case 16:  ctrl_value = 0x0C21; break;  // 1/16 step 11 00 0 0100 001
	        case 32:  ctrl_value = 0x0C29; break;  // 1/32 step 11 00 0 0101 001
	        case 64:  ctrl_value = 0x0C31; break;  // 1/64 step 11 00 0 0110 001
	        case 128: ctrl_value = 0x0C39; break;  // 1/128 step 11 00 0 0111 001
	        case 256: ctrl_value = 0x0C41; break;  // 1/256 step 11 00 0 1000 001
	        default: return;  // Invalid value, do nothing
	    }

	    current_microsteps = divider;
	    DRV8711_WriteRegister(DRV8711_REG_CTRL, ctrl_value);

	    // Recalculate frequency for current velocity
	    if (current_velocity != 0) {
	        stepper_set_velocity(current_velocity);
	    }
}
