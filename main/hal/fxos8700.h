/*!
* @file fxos8700.h
* @author Ethan Lew
* 
* This file defines methods for NXP's FXOS8700 6-Axis Xtrinsic sensor. The structures and 
* enumerations are taken from Adafruit's implementation: https://github.com/adafruit/Adafruit_FXOS8700
* This implementation is done in embedded C and uses a generic i2c interface.
*/
#ifndef FXOS8700_H
#define FXOS8700_H

#include "i2c_utils.h"

/** 7-bit I2C address for this sensor */
#define FXOS8700_ADDRESS           (0x1F)     // 0011111
/** Device ID for this sensor (used as sanity check during init) */
#define FXOS8700_ID (0xC7) // 1100 0111

#define ACCEL_RANGE ACCEL_RANGE_4G

#define ACCEL_BUFF_SIZE 13
#define MAGN_BUFF_SIZE 13

#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)

/*!
    Raw register addresses used to communicate with the sensor.
*/
typedef enum
{
    FXOS8700_REGISTER_STATUS          = 0x00, /**< 0x00 */
    FXOS8700_REGISTER_OUT_X_MSB       = 0x01, /**< 0x01 */
    FXOS8700_REGISTER_OUT_X_LSB       = 0x02, /**< 0x02 */
    FXOS8700_REGISTER_OUT_Y_MSB       = 0x03, /**< 0x03 */
    FXOS8700_REGISTER_OUT_Y_LSB       = 0x04, /**< 0x04 */
    FXOS8700_REGISTER_OUT_Z_MSB       = 0x05, /**< 0x05 */
    FXOS8700_REGISTER_OUT_Z_LSB       = 0x06, /**< 0x06 */
    FXOS8700_REGISTER_WHO_AM_I        = 0x0D, /**< 0x0D (default value = 0b11000111, read only) */
    FXOS8700_REGISTER_XYZ_DATA_CFG    = 0x0E, /**< 0x0E */
    FXOS8700_REGISTER_CTRL_REG1       = 0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_CTRL_REG2       = 0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_CTRL_REG3       = 0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_CTRL_REG4       = 0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_CTRL_REG5       = 0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_MSTATUS         = 0x32, /**< 0x32 */
    FXOS8700_REGISTER_MOUT_X_MSB      = 0x33, /**< 0x33 */
    FXOS8700_REGISTER_MOUT_X_LSB      = 0x34, /**< 0x34 */
    FXOS8700_REGISTER_MOUT_Y_MSB      = 0x35, /**< 0x35 */
    FXOS8700_REGISTER_MOUT_Y_LSB      = 0x36, /**< 0x36 */
    FXOS8700_REGISTER_MOUT_Z_MSB      = 0x37, /**< 0x37 */
    FXOS8700_REGISTER_MOUT_Z_LSB      = 0x38, /**< 0x38 */
    FXOS8700_REGISTER_MCTRL_REG1      = 0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_MCTRL_REG2      = 0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_MCTRL_REG3      = 0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
} fxos8700Registers_t;

/*!
    Range settings for the accelerometer sensor.
*/
typedef enum
{
    ACCEL_RANGE_2G                    = 0x00, /**< +/- 2g range */
    ACCEL_RANGE_4G                    = 0x01, /**< +/- 4g range */
    ACCEL_RANGE_8G                    = 0x02  /**< +/- 8g range */
} fxos8700AccelRange_t;


/*!
    Struct to store a single raw (integer-based) gyroscope vector
*/
typedef struct raw_int_data_s {
      int16_t x;    /**< Raw int16_t value for the x axis */
      int16_t y;    /**< Raw int16_t value for the y axis */
      int16_t z;    /**< Raw int16_t value for the z axis */
} raw_int_data_t;

typedef struct raw_float_data_s {
      float x;    /**< Raw int16_t value for the x axis */
      float y;    /**< Raw int16_t value for the y axis */
      float z;    /**< Raw int16_t value for the z axis */
} raw_float_data_t;

typedef struct accel_s {
    raw_int_data_t raw;
    raw_float_data_t converted;
    fxos8700AccelRange_t range;
    int32_t id;
    i2c_peripheral_t i2c;
} accel_t;

typedef struct magn_s {
    raw_int_data_t raw;
    raw_float_data_t converted;
    int32_t id;
    i2c_peripheral_t i2c;
} magn_t;

typedef enum {
    ACCEL_SUCCESS = 0x0,
    ACCEL_BUS_FAIL = 0x1,
    ACCEL_ID_FAIL = 0x2,
} accel_err_t;

typedef enum {
    MAGN_SUCCESS = 0x0,
    MAGN_BUS_FAIL = 0x1,
    MAGN_ID_FAIL = 0x2,
} magn_err_t;

accel_err_t accel_init(accel_t *accel);

accel_err_t accel_update(accel_t *accel);

magn_err_t magn_init(magn_t *magn);

magn_err_t magn_update(magn_t *magn);

#endif