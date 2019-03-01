/*!
* @brief implement the paper described here: http://folk.ntnu.no/skoge/prost/proceedings/cdc-ecc05/pdffiles/papers/1889.pdf
* @author Ethan Lew
*/

#ifndef MAHONY_H
#define MAHONY_H

#include <stdlib.h>
#include <math.h>
#include "../sensors.h"
#include "filter_utils.h"

#define DEFAULT_SAMPLE_FREQ	100.0f	// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.1f) // 2 * integral gain

typedef struct  mahony_filter_s {
    float invSampleFreq;
    quaternion_f_t q;
    euler_f_t eu;
    float twoKi, twoKp;
    float integralFBx, integralFBy, integralFBz;
    int anglesComputed;
} mahony_filter_t;

typedef enum {
    MAHONY_FILTER_SUCCESS = 0x00,
    MAHONY_FILTER_MFAIL = 0x01,
    MAHONY_FILTER_NMALLOC = 0x3,
} mahony_filter_err_t;

mahony_filter_err_t mahony_filter_init(mahony_filter_t **filt, float sampl_freq);

mahony_filter_err_t mahony_filter_update(mahony_filter_t* filt, accel_float_data_t accel, gyro_float_data_t gyro, magn_float_data_t magn);

mahony_filter_err_t mahony_filter_destroy(mahony_filter_t **filt);

#endif