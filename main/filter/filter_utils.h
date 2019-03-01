/*!
* @brief math structures and methods used for filtering
* @author Ethan Lew
*/
#ifndef FILTER_UTILS_H
#define FILTER_UTILS_H

typedef struct quaternion_f_s {
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion_f_t;

typedef struct euler_f_s {
    float roll;
    float pitch;
    float yaw;
} euler_f_t;

/*!
* @brief the legendary fast inv_sqrt
*/
float fast_inv_sqrt(float x);

/*!
* @brief Quaternion to Euler method
*/
euler_f_t quat_to_eul(quaternion_f_t q);

#endif

