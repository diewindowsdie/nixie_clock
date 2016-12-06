//
// This header file contains methods to interact with DS1307 real-time clock chip
// Created by maxpayne on 06.12.2016.
//

//DS1307 chip address used in write I2C messages
#define DS1307_WRITE_ADDRESS 0xD0
//DS1307 chip address used in read I2C messages
#define DS1307_READ_ADDRESS 0xD1

/**
 * Representation of time received or being sent to DS1307. Each field contains a time unit where zero array element
 * is the number of units (0 to 9), and second byte is the number of tens of unit. For example, 57 represented as {7, 5}
 */
typedef struct DS1307_Time {
    uint8_t seconds[2];
    uint8_t minutes[2];
    uint8_t hours[2];
} DS1307_Time;

/**
 * Performs library initialization by storing I2C handle and requesting time from DS1307
 *
 * @param i2c Pointer to I2C handle
 */
void DS1307_Initialize(I2C_HandleTypeDef *i2c);

/**
 * Function to be called when MCU receives interrupt event related to completion of transmitting data to DS1307
 */
void DS1307_Handle_Transmit_Completed();

/**
 * Function to be called when MCU receives interrupt event related to completion of receiving data from DS1307.
 * Also it checks if DS1307 internal oscillator is disabled and time is in 12h format and requests DS1307 update if neccesary.
 *
 * @param readBuffer Array contains data read from I2C bus
 */
void DS1307_Handle_Receive_Completed();

/**
 * Returns the last read time from DS1307 and requests reading new one. If no data had been read from DS1307 yet, returns pairs of 0xFF.
 */
DS1307_Time DS1307_GetCurrentTime();

/**
 * Sets the current time stored in DS1307 to specified value
 * @param newTime Pointer to the structure holds new time to be stored in DS1307
 */
void DS1307_SetCurrentTime(DS1307_Time *newTime);