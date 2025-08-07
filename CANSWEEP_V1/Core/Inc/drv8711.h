/*
 * drv8711.h
 *
 *  Created on: Aug 7, 2025
 *      Author: Pau
 */

#ifndef INC_DRV8711_H_
#define INC_DRV8711_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

// DRV8711 Register Addresses
#define DRV8711_REG_CTRL    0x00
#define DRV8711_REG_TORQUE  0x01
#define DRV8711_REG_OFF     0x02
#define DRV8711_REG_BLANK   0x03
#define DRV8711_REG_DECAY   0x04
#define DRV8711_REG_STALL   0x05
#define DRV8711_REG_DRIVE   0x06
#define DRV8711_REG_STATUS  0x07

// Control Register Settings
#define DRV8711_CTRL_ENABLE_1_8_STEP    0x0C19  // Enable motor, 1/8 microstepping
#define DRV8711_TORQUE_MAX              0x0080  // Cuidado amb aquest valor, aixo controla la corrent maxima q passara per l'stepper. 0x80 és un 50%
#define DRV8711_OFF_TIME_3US            0x030  // 3us off time
#define DRV8711_DECAY_MIXED_10US        0x110  // Mixed decay, 10us

// Function Prototypes
void DRV8711_Init(SPI_HandleTypeDef *hspi);
void DRV8711_WriteRegister(uint8_t address, uint16_t data);
uint16_t DRV8711_ReadRegister(uint8_t address);
void DRV8711_Enable(void);
void DRV8711_Disable(void);
HAL_StatusTypeDef DRV8711_CheckFault(void);

#endif /* INC_DRV8711_H_ */
