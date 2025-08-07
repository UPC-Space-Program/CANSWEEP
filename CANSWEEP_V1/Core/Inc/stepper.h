/*
 * stepper.h
 *
 *  Created on: Aug 6, 2025
 *      Author: Pau
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include <stdint.h>
#include <stdbool.h>

// Initialize stepper driver
void stepper_init(void);

// Set velocity in rad/s (negative = reverse)
void stepper_set_velocity(float rad_per_sec);

// Get current velocity
float stepper_get_velocity(void);

// Stop immediately
void stepper_stop(void);

// Optional: Change microsteps at runtime if needed
void stepper_set_microsteps(uint8_t divider);

#endif /* INC_STEPPER_H_ */
