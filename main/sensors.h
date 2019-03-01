/*!
* @brief Structures to describe generic sensor data
* @author Ethan Lew
*/

#include <stdint.h>

#ifndef SENSORS_H
#define SENSORS_H

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

typedef struct accel_int_data_s {
      int16_t x;    /**< Raw int16_t value for the x axis */
      int16_t y;    /**< Raw int16_t value for the y axis */
      int16_t z;    /**< Raw int16_t value for the z axis */
} accel_int_data_t;

typedef struct accel_float_data_s {
      float x;    /**< Raw int16_t value for the x axis */
      float y;    /**< Raw int16_t value for the y axis */
      float z;    /**< Raw int16_t value for the z axis */
} accel_float_data_t;


typedef struct magn_int_data_s {
      int16_t x;    /**< Raw int16_t value for the x axis */
      int16_t y;    /**< Raw int16_t value for the y axis */
      int16_t z;    /**< Raw int16_t value for the z axis */
} magn_int_data_t;

typedef struct magn_float_data_s {
      float x;    /**< Raw int16_t value for the x axis */
      float y;    /**< Raw int16_t value for the y axis */
      float z;    /**< Raw int16_t value for the z axis */
} magn_float_data_t;

typedef enum {
    GYRO_SUCCESS = 0x0,
    GYRO_ID_FAIL = 0x1,   
    GYRO_BUS_FAIL = 0x2, 
    GYRO_NMALLOC = 0x3,
} gyro_err_t;

typedef enum {
    ACCEL_SUCCESS = 0x0,
    ACCEL_BUS_FAIL = 0x1,
    ACCEL_ID_FAIL = 0x2,
    ACCEL_NMALLOC = 0x3,
} accel_err_t;

typedef enum {
    MAGN_SUCCESS = 0x0,
    MAGN_BUS_FAIL = 0x1,
    MAGN_ID_FAIL = 0x2,
    MAGN_NMALLOC = 0x3,
} magn_err_t;

#endif 