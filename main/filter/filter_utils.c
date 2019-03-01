#include "filter_utils.h"
#include <math.h>

/*!
* See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
float fast_inv_sqrt(float x){
    float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
    return y;
}


euler_f_t quat_to_eul(quaternion_f_t q){
    euler_f_t eul;
    eul.roll = atan2f(q.q0 * q.q1 + q.q2 * q.q3, 0.5f - q.q1 * q.q1 - q.q2* q.q2);
	eul.pitch = asinf(-2.0f * (q.q1 * q.q3 - q.q0 * q.q2));
	eul.yaw = atan2f(q.q1 * q.q2 + q.q0 * q.q3, 0.5f - q.q2 * q.q2 - q.q3 * q.q3);
    return eul;
}