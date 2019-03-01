/*!
* @file fxas21002c.h
* @author Ethan Lew
* 
* This file defines methods for NXP's FXAS21002C 3-Axis gyroscope. The structures and 
* enumerations are taken from Adafruit's implementation: https://github.com/adafruit/Adafruit_FXAS21002C
* This implementation is done in embedded C and uses a generic i2c interface.
*/

#ifndef FXAS21002C_H
#define FXAS21002C_H

#include <stdlib.h>
#include "i2c_utils.h"

/* 7-bit address for this sensor */
#define FXAS21002C_ADDRESS       (0x21)       // 0100001
/* Device ID for this sensor (used as a sanity check during init) */
#define FXAS21002C_ID            (0xD7)       // 1101 0111
/* Gyroscope sensitivity at 250dps */
#define GYRO_SENSITIVITY_250DPS  (0.0078125F) // Table 35 of datasheet
/* Gyroscope sensitivity at 500dps */
#define GYRO_SENSITIVITY_500DPS  (0.015625F)
/* Gyroscope sensitivity at 1000dps */
#define GYRO_SENSITIVITY_1000DPS (0.03125F)
/* Gyroscope sensitivity at 2000dps */
#define GYRO_SENSITIVITY_2000DPS (0.0625F)
/* Gyroscope Buffer Size */
#define GYRO_BUFF_SIZE 8
/* Conversion factor for sensor */
#define SENSORS_DPS_TO_RADS (0.017453293F) /**< Degrees/s to rad/s multiplier */

#define GYRO_RANGE 250

/*!
    Raw register addresses used to communicate with the sensor.
*/
typedef enum {
    GYRO_REGISTER_STATUS              = 0x00, /**< 0x00 */
    GYRO_REGISTER_OUT_X_MSB           = 0x01, /**< 0x01 */
    GYRO_REGISTER_OUT_X_LSB           = 0x02, /**< 0x02 */
    GYRO_REGISTER_OUT_Y_MSB           = 0x03, /**< 0x03 */
    GYRO_REGISTER_OUT_Y_LSB           = 0x04, /**< 0x04 */
    GYRO_REGISTER_OUT_Z_MSB           = 0x05, /**< 0x05 */
    GYRO_REGISTER_OUT_Z_LSB           = 0x06, /**< 0x06 */
    GYRO_REGISTER_WHO_AM_I            = 0x0C, /**< 0x0C (default value = 0b11010111, read only) */
    GYRO_REGISTER_CTRL_REG0           = 0x0D, /**< 0x0D (default value = 0b00000000, read/write) */
    GYRO_REGISTER_CTRL_REG1           = 0x13, /**< 0x13 (default value = 0b00000000, read/write) */
    GYRO_REGISTER_CTRL_REG2           = 0x14, /**< 0x14 (default value = 0b00000000, read/write) */
} gyro_registers_t;

/*!
    Enum to define valid gyroscope range values
*/
typedef enum {
    GYRO_RANGE_250DPS  = 250,     /**< 250dps */
    GYRO_RANGE_500DPS  = 500,     /**< 500dps */
    GYRO_RANGE_1000DPS = 1000,    /**< 1000dps */
    GYRO_RANGE_2000DPS = 2000     /**< 2000dps */
} gyro_range_t;


/*!
    Struct to store a single raw (integer-based) gyroscope vector
*/
typedef struct gyro_int_data_s {
      int16_t x;    /**< Raw int16_t value for the x axis */
      int16_t y;    /**< Raw int16_t value for the y axis */
      int16_t z;    /**< Raw int16_t value for the z axis */
} gyro_int_data_t;

typedef struct gyro_float_data_s {
      float x;    /**< Raw int16_t value for the x axis */
      float y;    /**< Raw int16_t value for the y axis */
      float z;    /**< Raw int16_t value for the z axis */
} gyro_float_data_t;

typedef struct gyro_s {
    gyro_int_data_t raw;
    gyro_float_data_t converted;
    gyro_range_t range;
    int32_t id;
    i2c_peripheral_t i2c;
} gyro_t;


typedef enum {
    GYRO_SUCCESS = 0x0,
    GYRO_ID_FAIL = 0x1,   
    GYRO_BUS_FAIL = 0x2, 
    GYRO_NMALLOC = 0x3,
} gyro_err_t;


gyro_err_t gyro_init(gyro_t **gyro);

gyro_err_t gyro_update(gyro_t *gyro);

gyro_err_t gyro_destroy(gyro_t **gyro);


#endif