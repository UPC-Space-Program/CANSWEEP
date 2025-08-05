/*
 * encoder.h
 *
 *  Created on: Aug 5, 2025
 *      Author: Pau
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32g4xx_hal.h"

// AS5600 I2C Address
#define ENCODER_I2C_ADDR         (0x36 << 1)

// AS5600 Registers
#define ENCODER_RAW_ANGLE_REG    0x0C
#define ENCODER_ANGLE_REG        0x0E
#define ENCODER_STATUS_REG       0x0B
#define ENCODER_CONF_REG         0x07

// Status bits
#define ENCODER_MAGNET_DETECTED  0x20

// Encoder data structure
typedef struct {
    volatile uint16_t angle;          // Current angle (0-4095)
    volatile int32_t  turns;          // Turn counter
    volatile int32_t  position;       // Total position (turns * 4096 + angle)
    uint16_t          last_angle;     // For overflow detection
    uint32_t          last_update;    // Timestamp of last update
    uint8_t           magnet_status;  // Magnet detection status
    uint32_t          error_count;    // I2C error counter
} Encoder_Handle_t;

// Function prototypes
void Encoder_Init(Encoder_Handle_t* encoder, I2C_HandleTypeDef* hi2c);
void Encoder_UpdatePosition(Encoder_Handle_t* encoder, I2C_HandleTypeDef* hi2c);
uint16_t Encoder_GetAngle(Encoder_Handle_t* encoder);
int32_t Encoder_GetPosition(Encoder_Handle_t* encoder);
int32_t Encoder_GetTurns(Encoder_Handle_t* encoder);
uint8_t Encoder_IsMagnetDetected(Encoder_Handle_t* encoder);

#endif /* INC_ENCODER_H_ */
