//
// Created by maxpayne on 06.12.16.
//

#include <stdint.h>
#include "stm32f0xx_hal.h"
#include <DS1307.h>

/**
 * Returns nth bit (zero-based) of specified number
 */
#define NTH_BIT(VALUE, BIT) ((VALUE >> BIT) & (uint8_t) 0x1)

I2C_HandleTypeDef *i2c_pointer;

void (*errorHandler)(void);

uint8_t i2c_writeBuffer[4];
uint8_t i2c_readBuffer[3];

DS1307_Time currentTime = {.hours = {0xFF, 0xFF}, .minutes = {0xFF, 0xFF}, .seconds = {0xFF, 0xFF}};

static void DS1307_RequestTimeReading();

void DS1307_Initialize(I2C_HandleTypeDef *i2c, void (*i2cErrorHandler)(void)) {
    i2c_pointer = i2c;
    errorHandler = i2cErrorHandler;

    DS1307_RequestTimeReading();
}

void DS1307_Handle_Transmit_Completed() {
    //If we get here, it means some data has been transferred to DS1307. Can be register pointer, new values for h/m/s or
    //init values for enabling oscillator and settings 24h mode.
    //Either way, we have to request three bytes from registers 0x00, 0x01 and 0x02
    if (HAL_I2C_Master_Receive_IT(i2c_pointer, DS1307_READ_ADDRESS, &i2c_readBuffer, 3) != HAL_OK) {
        (*errorHandler)();
    };
}

void DS1307_Handle_Receive_Completed() {
    uint8_t initRequired = 0;
    i2c_writeBuffer[0] = 0x00; //first register
    i2c_writeBuffer[1] = i2c_readBuffer[0];
    //check if oscillator is disabled
    if (NTH_BIT(i2c_readBuffer[0], 7)) {
        i2c_writeBuffer[0] &= 0x7f; //we have to set 7th bit to zero to enable oscillator
        initRequired = 1;
    }
    i2c_writeBuffer[2] = i2c_readBuffer[1];
    i2c_writeBuffer[3] = i2c_readBuffer[2];

    //in case hours in 12h format, read tens of hours
    uint8_t hours_1 = NTH_BIT(i2c_readBuffer[2], 4);

    //check if hours in 12h format
    if (NTH_BIT(i2c_readBuffer[2], 6)) {
        i2c_writeBuffer[3] &= 0xBF; //we have to set 6th bit to zero to enable 24h mode
        //12h mode. Now we need to recalculate hours value in 24h format.
        //check if hours is PM now
        if (NTH_BIT(i2c_readBuffer[2], 5)) {
            hours_1 += 12;
        }
        i2c_writeBuffer[3] &= ((hours_1 << 4) & 0xFF); //new hours value in bits 4 and 5
        initRequired = 1;
    } else {
        hours_1 = (i2c_readBuffer[2] >> 4) & (uint8_t) 0x3; //bits 4-5;
    }

    currentTime.seconds[0] = i2c_readBuffer[0] & (uint8_t) 0x0F;       //bits 0-3
    currentTime.seconds[1] = (i2c_readBuffer[0] >> 4) & (uint8_t) 0x7; //bits 4-6

    currentTime.minutes[0] = i2c_readBuffer[1] & (uint8_t) 0x0F;       //bits 0-3
    currentTime.minutes[1] = (i2c_readBuffer[1] >> 4) & (uint8_t) 0x7; //bits 4-6

    currentTime.hours[0] = i2c_readBuffer[2] & (uint8_t) 0x0F;       //bits 0-3
    currentTime.hours[1] = hours_1;

    if (initRequired) {
        if (HAL_I2C_Master_Transmit_IT(i2c_pointer, DS1307_WRITE_ADDRESS, &i2c_writeBuffer, 4) != HAL_OK) {
            (*errorHandler)();
        };
    }
}

DS1307_Time DS1307_GetCurrentTime() {
    DS1307_RequestTimeReading();

    return currentTime;
}

/**
 * Requests reading of current time from DS1307 by setting its register pointer to 0x00
 */
void DS1307_RequestTimeReading() {
    //set DS1307 internal register pointer to 0x00 - seconds register
    i2c_writeBuffer[0] = 0x00;
    if (HAL_I2C_Master_Transmit_IT(i2c_pointer, DS1307_WRITE_ADDRESS, &i2c_writeBuffer, 1) != HAL_OK) {
        (*errorHandler)();
    };
}

void DS1307_SetCurrentTime(DS1307_Time *newTime) {
    i2c_writeBuffer[0] = 0x00; //seconds register address
    i2c_writeBuffer[1] = (newTime->seconds[0] << 4) | newTime->seconds[1];
    i2c_writeBuffer[2] = (newTime->minutes[0] << 4) | newTime->minutes[1];
    i2c_writeBuffer[3] = (newTime->hours[0] << 4) | newTime->hours[1];

    if (HAL_I2C_Master_Transmit_IT(i2c_pointer, DS1307_WRITE_ADDRESS, &i2c_writeBuffer, 4) != HAL_OK) {
        (*errorHandler)();
    }
}

