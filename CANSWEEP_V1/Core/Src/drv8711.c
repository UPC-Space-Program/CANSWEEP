/*
 * drv8711.c
 *
 *  Created on: Aug 7, 2025
 *      Author: Pau
 */
#include "drv8711.h"
#include "main.h"  // For pin definitions

// Private variables
static SPI_HandleTypeDef *drv_spi = NULL;

// Initialize the DRV8711
void DRV8711_Init(SPI_HandleTypeDef *hspi) {
    drv_spi = hspi;

    // Enable the chip (nSLEEP high)
    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);
    HAL_Delay(10);  // Power-up delay

    // Configure registers with meaningful names
    DRV8711_WriteRegister(DRV8711_REG_CTRL, DRV8711_CTRL_ENABLE_1_8_STEP);
    DRV8711_WriteRegister(DRV8711_REG_TORQUE, DRV8711_TORQUE_MAX);
    DRV8711_WriteRegister(DRV8711_REG_OFF, DRV8711_OFF_TIME_3US);
    DRV8711_WriteRegister(DRV8711_REG_BLANK, 0x0800);  // 2.56us blank time
    DRV8711_WriteRegister(DRV8711_REG_DECAY, DRV8711_DECAY_MIXED_10US);
    DRV8711_WriteRegister(DRV8711_REG_STALL, 0x040);  // default
    //DRV8711_WriteRegister(DRV8711_REG_DRIVE, 0x0000);
    DRV8711_WriteRegister(DRV8711_REG_STATUS, 0x0000); // Clear faults
}

// Write to DRV8711 register
void DRV8711_WriteRegister(uint8_t address, uint16_t data) {
    if (drv_spi == NULL) return;  // Safety check

    uint16_t cmd = ((address & 0x07) << 12) | (data & 0xFFF);
    uint8_t tx[2] = {cmd >> 8, cmd & 0xFF};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // CS low
    HAL_SPI_Transmit(drv_spi, tx, 2, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    // CS high
}

// Read from DRV8711 register
uint16_t DRV8711_ReadRegister(uint8_t address) {
    if (drv_spi == NULL) return 0;  // Safety check

    uint16_t cmd = 0x8000 | ((address & 0x07) << 12);
    uint8_t tx[2] = {cmd >> 8, cmd & 0xFF};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // CS low
    HAL_SPI_TransmitReceive(drv_spi, tx, rx, 2, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    // CS high

    return ((rx[0] << 8) | rx[1]) & 0xFFF;
}

// Enable motor driver
void DRV8711_Enable(void) {
    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);
}

// Disable motor driver
void DRV8711_Disable(void) {
    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_RESET);
}

// Check for faults
HAL_StatusTypeDef DRV8711_CheckFault(void) {
    uint16_t status = DRV8711_ReadRegister(DRV8711_REG_STATUS);
    if (status & 0xFF00) {  // Any fault bits set
        return HAL_ERROR;
    }
    return HAL_OK;
}

