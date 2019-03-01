#include "mahony.h"

static mahony_filter_err_t mahony_imu_update(mahony_filter_t* filt, accel_float_data_t accel, gyro_float_data_t gyro);


mahony_filter_err_t mahony_filter_init(mahony_filter_t **filt, float sampl_freq){
    if(filt != NULL){
        *filt = (mahony_filter_t *)malloc(sizeof(mahony_filter_t));
    }

    (*filt)->twoKi = twoKiDef;
    (*filt)->twoKp =  twoKpDef;
    (*filt)->q.q0 = 1.0f;
	(*filt)->q.q1 = 0.0f;
	(*filt)->q.q2 = 0.0f;
	(*filt)->q.q3 = 0.0f;
	(*filt)->integralFBx = 0.0f;
	(*filt)->integralFBy = 0.0f;
	(*filt)->integralFBz = 0.0f;
	(*filt)->anglesComputed = 0;
    (*filt)->invSampleFreq = 1.0f / sampl_freq;

    return MAHONY_FILTER_SUCCESS;
}

mahony_filter_err_t mahony_filter_update(mahony_filter_t* filt, accel_float_data_t accel, gyro_float_data_t gyro, magn_float_data_t magn){
    float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
    float qa, qb, qc;

    /* Use IMU algorithm if magnetometer measurement invalid */
	/* (avoids NaN in magnetometer normalisation) */
	if((magn.x == 0.0f) && (magn.y == 0.0f) && (magn.z == 0.0f)) {
		mahony_imu_update(filt, accel, gyro);
		return MAHONY_FILTER_MFAIL;
    }

    /* NOTE: gyro is already in rad/s */

    /* Compute feedback only if accelerometer measurement valid */
	/* (avoids NaN in accelerometer normalisation) */
    if(!((accel.x == 0.0f) && (accel.y == 0.0f) && (accel.z == 0.0f))) {
        
        /* Normalise accelerometer measurement */
		recipNorm = fast_inv_sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
		accel.x *= recipNorm;
		accel.y *= recipNorm;
        accel.z *= recipNorm;

        /* Normalise magnetometer measurement */
		recipNorm = fast_inv_sqrt(magn.x * magn.x + magn.y * magn.y + magn.z * magn.z);
		magn.x *= recipNorm;
		magn.y *= recipNorm;
        magn.z *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
		q0q0 = filt->q.q0 * filt->q.q0;
		q0q1 = filt->q.q0 * filt->q.q1;
		q0q2 = filt->q.q0 * filt->q.q2;
		q0q3 = filt->q.q0 * filt->q.q3;
		q1q1 = filt->q.q1 * filt->q.q1;
		q1q2 = filt->q.q1 * filt->q.q2;
		q1q3 = filt->q.q1 * filt->q.q3;
		q2q2 = filt->q.q2 * filt->q.q2;
		q2q3 = filt->q.q2 * filt->q.q3;
        q3q3 = filt->q.q3 * filt->q.q3;

        /* Reference direction of Earth's magnetic field */
		hx = 2.0f * (magn.x * (0.5f - q2q2 - q3q3) + magn.y * (q1q2 - q0q3) + magn.z * (q1q3 + q0q2));
		hy = 2.0f * (magn.x * (q1q2 + q0q3) + magn.y * (0.5f - q1q1 - q3q3) + magn.z * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (magn.x * (q1q3 - q0q2) + magn.y * (q2q3 + q0q1) + magn.z * (0.5f - q1q1 - q2q2));

        /* Estimated direction of gravity and magnetic field */
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        /* Error is sum of cross product between estimated direction */
		/* and measured direction of field vectors */
		halfex = (accel.y * halfvz - accel.z * halfvy) + (magn.y * halfwz - magn.z * halfwy);
		halfey = (accel.z * halfvx - accel.x * halfvz) + (magn.z * halfwx - magn.x * halfwz);
        halfez = (accel.x * halfvy - accel.y * halfvx) + (magn.x * halfwy - magn.y * halfwx);

        // Compute and apply integral feedback if enabled
		if(filt->twoKi > 0.0f) {
			// integral error scaled by Ki
			filt->integralFBx += filt->twoKi * halfex * filt->invSampleFreq;
			filt->integralFBy += filt->twoKi * halfey * filt->invSampleFreq;
			filt->integralFBz += filt->twoKi * halfez * filt->invSampleFreq;
			gyro.x += filt->integralFBx;	// apply integral feedback
			gyro.y += filt->integralFBy;
			gyro.z += filt->integralFBz;
		} else {
			filt->integralFBx = 0.0f;	// prevent integral windup
			filt->integralFBy = 0.0f;
			filt->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gyro.x += filt->twoKp * halfex;
		gyro.y += filt->twoKp * halfey;
        gyro.z += filt->twoKp * halfez;
    }    

    // Integrate rate of change of quaternion
	gyro.x *= (0.5f * filt->invSampleFreq);		// pre-multiply common factors
	gyro.y *= (0.5f * filt->invSampleFreq);
	gyro.z *= (0.5f * filt->invSampleFreq);
	qa = filt->q.q0;
	qb = filt->q.q1;
	qc = filt->q.q2;
	filt->q.q0 += (-qb * gyro.x - qc * gyro.y - filt->q.q3 * gyro.z);
	filt->q.q1 += (qa * gyro.x + qc * gyro.z - filt->q.q3 * gyro.y);
	filt->q.q2 += (qa * gyro.y - qb * gyro.z + filt->q.q3 * gyro.x);
	filt->q.q3 += (qa * gyro.z + qb * gyro.y - qc * gyro.x);

	// Normalise quaternion
	recipNorm = fast_inv_sqrt(filt->q.q0 * filt->q.q0 + filt->q.q1 * filt->q.q1 + filt->q.q2 * filt->q.q2 + filt->q.q3 * filt->q.q3);
	filt->q.q0 *= recipNorm;
	filt->q.q1 *= recipNorm;
	filt->q.q2 *= recipNorm;
	filt->q.q3 *= recipNorm;
    filt->anglesComputed = 0;

    return MAHONY_FILTER_SUCCESS;
}

mahony_filter_err_t mahony_imu_update(mahony_filter_t* filt, accel_float_data_t accel, gyro_float_data_t gyro){
    float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
    float qa, qb, qc;

    /* Compute feedback only if accelerometer measurement valid */
	/* (avoids NaN in accelerometer normalisation) */
    if(!((accel.x == 0.0f) && (accel.y == 0.0f) && (accel.z == 0.0f))) {
        /* Normalise accelerometer measurement */
		recipNorm = fast_inv_sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
		accel.x *= recipNorm;
		accel.y *= recipNorm;
        accel.z *= recipNorm;

        // Estimated direction of gravity
		halfvx = filt->q.q1 * filt->q.q3 - filt->q.q0 * filt->q.q2;
		halfvy = filt->q.q0 * filt->q.q1 + filt->q.q2 * filt->q.q3;
        halfvz = filt->q.q0 * filt->q.q0 - 0.5f + filt->q.q3 * filt->q.q3;

        // Error is sum of cross product between estimated
		// and measured direction of gravity
		halfex = (accel.y * halfvz - accel.z * halfvy);
		halfey = (accel.z * halfvx - accel.x * halfvz);
        halfez = (accel.x * halfvy - accel.y * halfvx);

        // Compute and apply integral feedback if enabled
		if(filt->twoKi > 0.0f) {
			// integral error scaled by Ki
			filt->integralFBx += filt->twoKi * halfex * filt->invSampleFreq;
			filt->integralFBy += filt->twoKi * halfey * filt->invSampleFreq;
			filt->integralFBz += filt->twoKi * halfez * filt->invSampleFreq;
			gyro.x += filt->integralFBx;	// apply integral feedback
			gyro.y += filt->integralFBy;
			gyro.z += filt->integralFBz;
		} else {
			filt->integralFBx = 0.0f;	// prevent integral windup
			filt->integralFBy = 0.0f;
			filt->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gyro.x += filt->twoKp * halfex;
		gyro.y += filt->twoKp * halfey;
        gyro.z += filt->twoKp * halfez;
    }

    // Integrate rate of change of quaternion
	gyro.x *= (0.5f * filt->invSampleFreq);		// pre-multiply common factors
	gyro.y *= (0.5f * filt->invSampleFreq);
	gyro.z *= (0.5f * filt->invSampleFreq);
	qa = filt->q.q0;
	qb = filt->q.q1;
	qc = filt->q.q2;
	filt->q.q0 += (-qb * gyro.x - qc * gyro.y - filt->q.q3 * gyro.z);
	filt->q.q1 += (qa * gyro.x + qc * gyro.z - filt->q.q3 * gyro.y);
	filt->q.q2 += (qa * gyro.y - qb * gyro.z + filt->q.q3 * gyro.x);
	filt->q.q3 += (qa * gyro.z + qb * gyro.y - qc * gyro.x);

	// Normalise quaternion
	recipNorm = fast_inv_sqrt(filt->q.q0 * filt->q.q0 + filt->q.q1 * filt->q.q1 + filt->q.q2 * filt->q.q2 + filt->q.q3 * filt->q.q3);
	filt->q.q0 *= recipNorm;
	filt->q.q1 *= recipNorm;
	filt->q.q2 *= recipNorm;
	filt->q.q3 *= recipNorm;
    filt->anglesComputed = 0;

    return MAHONY_FILTER_SUCCESS;
}

mahony_filter_err_t mahony_filter_destroy(mahony_filter_t **filt){
    if(filt != NULL){
        free(*filt);
        *filt = NULL;
        return MAHONY_FILTER_SUCCESS;
    } else {
        return MAHONY_FILTER_NMALLOC;
    }
}