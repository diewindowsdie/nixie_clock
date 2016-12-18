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

extern DS1307_Time currentTime;

volatile DS1307_State interactionState;

void DS1307_Initialize(I2C_HandleTypeDef *i2c, void (*i2cErrorHandler)(void)) {
    i2c_pointer = i2c;
    errorHandler = i2cErrorHandler;
    interactionState = DS1307_READY;

    DS1307_RequestTimeReading();
}

void DS1307_Handle_Transmit_Completed() {
    //If we get here, it means some data has been transferred to DS1307. Can be register pointer, new values for h/m/s or
    //init values for enabling oscillator and settings 24h mode.
    if (interactionState == DS1307_READING) {
        //that means we're in process of reading data from DS1307 and this event caused by writing register pointer
        //now we request data
        if (HAL_I2C_Master_Receive_IT(i2c_pointer, DS1307_READ_ADDRESS, &i2c_readBuffer, 3) != HAL_OK) {
            (*errorHandler)();
        };
    } else if (interactionState == DS1307_WRITING) {
        //that means we're in process of writing data to DS1307 and this event caused by writing data to DS1307 registers
        //now we send register pointer to read data on next iteration
        i2c_writeBuffer[0] = 0x00;
        if (HAL_I2C_Master_Transmit_IT(i2c_pointer, DS1307_WRITE_ADDRESS, &i2c_writeBuffer, 1) != HAL_OK) {
            (*errorHandler)();
        };
        interactionState = DS1307_READING;
    }
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

    uint8_t hours = i2c_readBuffer[2] & (uint8_t) 0x0F;       //bits 0-3

    //in case hours in 12h format, read tens of hours
    uint8_t tensHours = NTH_BIT(i2c_readBuffer[2], 4);

    //check if hours in 12h format
    if (NTH_BIT(i2c_readBuffer[2], 6)) {
        i2c_writeBuffer[3] &= 0xBF; //we have to set 6th bit to zero to enable 24h mode
        //12h mode. Now we need to recalculate hours value in 24h format.

        hours += 10 * tensHours;

        //check if hours is PM now
        if (NTH_BIT(i2c_readBuffer[2], 5)) {
            hours += 12;
        }
        i2c_writeBuffer[3] &= (((hours / 10) << 4) & 0xFF); //new hours value in bits 4 and 5
        initRequired = 1;
    } else {
        hours += 10 * ((i2c_readBuffer[2] >> 4) & (uint8_t) 0x3); //bits 4-5;
    }

    uint8_t seconds = i2c_readBuffer[0] & (uint8_t) 0x0F;       //bits 0-3
    seconds += 10 * ((i2c_readBuffer[0] >> 4) & (uint8_t) 0x7); //bits 4-6

    uint8_t minutes = i2c_readBuffer[1] & (uint8_t) 0x0F;       //bits 0-3
    minutes += 10 * ((i2c_readBuffer[1] >> 4) & (uint8_t) 0x7); //bits 4-6

    currentTime.seconds = seconds;
    currentTime.minutes = minutes;
    currentTime.hours = hours;

    interactionState = DS1307_READY;

    if (initRequired) {
        interactionState = DS1307_WRITING;
        if (HAL_I2C_Master_Transmit_IT(i2c_pointer, DS1307_WRITE_ADDRESS, &i2c_writeBuffer, 4) != HAL_OK) {
            (*errorHandler)();
        };
    }
}

void DS1307_RequestTimeReading() {
    //set DS1307 internal register pointer to 0x00 - seconds register
    i2c_writeBuffer[0] = 0x00;
    interactionState = DS1307_READING;
    if (HAL_I2C_Master_Transmit_IT(i2c_pointer, DS1307_WRITE_ADDRESS, &i2c_writeBuffer, 1) != HAL_OK) {
        (*errorHandler)();
    };
}

volatile DS1307_State DS1307_GetCurrentState() {
    return interactionState;
}

void DS1307_SetCurrentTime(DS1307_Time *newTime) {
    interactionState = DS1307_WRITING;

    i2c_writeBuffer[0] = 0x00; //seconds register address
    i2c_writeBuffer[1] = ((uint8_t)(newTime->seconds / 10) << 4) | (uint8_t)(newTime->seconds % 10);
    i2c_writeBuffer[2] = ((uint8_t)(newTime->minutes / 10) << 4) | (uint8_t)(newTime->minutes % 10);
    i2c_writeBuffer[3] = ((uint8_t)(newTime->hours / 10) << 4) | (uint8_t)(newTime->hours % 10);

    if (HAL_I2C_Master_Transmit_IT(i2c_pointer, DS1307_WRITE_ADDRESS, &i2c_writeBuffer, 4) != HAL_OK) {
        (*errorHandler)();
    }
}

