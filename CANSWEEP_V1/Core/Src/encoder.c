/*
 * encoder.c
 *
 *  Created on: Aug 5, 2025
 *      Author: Pau
 */


#include "encoder.h"

void Encoder_Init(Encoder_Handle_t* encoder, I2C_HandleTypeDef* hi2c)
{
    // Initialize structure
    encoder->angle = 0;
    encoder->turns = 0;
    encoder->position = 0;
    encoder->last_angle = 0;
    encoder->error_count = 0;
    encoder->last_update = HAL_GetTick();

    // Read initial position
    uint8_t buffer[2];
    if (HAL_I2C_Mem_Read(hi2c, ENCODER_I2C_ADDR, ENCODER_RAW_ANGLE_REG,
                         I2C_MEMADD_SIZE_8BIT, buffer, 2, 10) == HAL_OK)
    {
        encoder->angle = ((uint16_t)buffer[0] << 8) | buffer[1];
        encoder->last_angle = encoder->angle;
    }

    // Check magnet status
    uint8_t status;
    if (HAL_I2C_Mem_Read(hi2c, ENCODER_I2C_ADDR, ENCODER_STATUS_REG,
                         I2C_MEMADD_SIZE_8BIT, &status, 1, 10) == HAL_OK)
    {
        encoder->magnet_status = status;
    }
}

void Encoder_UpdatePosition(Encoder_Handle_t* encoder, I2C_HandleTypeDef* hi2c)
{
    uint8_t buffer[2];

    // Read angle
    if (HAL_I2C_Mem_Read(hi2c, ENCODER_I2C_ADDR, ENCODER_RAW_ANGLE_REG,
                         I2C_MEMADD_SIZE_8BIT, buffer, 2, 1) == HAL_OK)
    {
        // Convert to 12-bit angle
        uint16_t new_angle = ((uint16_t)buffer[0] << 8) | buffer[1];
        encoder->angle = new_angle;

        // Detect overflow/underflow
        int16_t delta = (int16_t)new_angle - (int16_t)encoder->last_angle;

        if (delta > 2048) {
            encoder->turns--;  // Underflow
        } else if (delta < -2048) {
            encoder->turns++;  // Overflow
        }

        // Update position
        encoder->position = (encoder->turns * 4096) + new_angle;
        encoder->last_angle = new_angle;
        encoder->last_update = HAL_GetTick();

        // Reset error count on successful read
        /*if (encoder->error_count > 0) {
            encoder->error_count = 0;
        }*/
    }
    else
    {
        // Increment error count
        encoder->error_count++;
    }
}

uint16_t Encoder_GetAngle(Encoder_Handle_t* encoder)
{
    return encoder->angle;
}

int32_t Encoder_GetPosition(Encoder_Handle_t* encoder)
{
    return encoder->position;
}

int32_t Encoder_GetTurns(Encoder_Handle_t* encoder)
{
    return encoder->turns;
}

uint8_t Encoder_IsMagnetDetected(Encoder_Handle_t* encoder)
{
    return (encoder->magnet_status & ENCODER_MAGNET_DETECTED) ? 1 : 0;
}
