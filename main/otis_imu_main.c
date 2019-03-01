/*!
* @file otis_imu_main.c
* @author Ethan Lew
*
* For now, the main task grabs gyroscope data and prints it to the serial device.
*/
#include "hal/fxas21002c.h"
#include "hal/fxos8700.h"
#include "hal/time_utils.h"
#include "filter/mahony.h"

#define SAMPLE_PERIOD 10
#define SAMPLE_FREQUENCY 100.0f

static void mahony_test_task(void *arg)
{
    /* Timing parameters */
    const TickType_t init_wait = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS( SAMPLE_PERIOD );
    xLastWakeTime = xTaskGetTickCount();

    /* Create generic gyroscope */
    gyro_t* gyro = NULL;
    if(gyro_init(&gyro) != GYRO_SUCCESS){
        printf("Gyroscope initialization failed.\n");
    }
    vTaskDelay(init_wait);

    /* Create generic magnetometer */
    magn_t* magn = NULL;
    if(magn_init(&magn) != MAGN_SUCCESS){
        printf("Magnetometer initialization failed.\n");
    }

    /* Create generic accelerometer */
    accel_t* accel = NULL;
    if(accel_init(&accel) != ACCEL_SUCCESS){
        printf("Accelerometer initialization failed.\n");
    }

    /* Create mahony filter */
    mahony_filter_t* filt = NULL;
    if(mahony_filter_init(&filt, SAMPLE_FREQUENCY) != MAHONY_FILTER_SUCCESS){
        printf("Mahony initialization failed.\n");
    }

    /* Offsets applied to raw x/y/z mag values */
    float mag_offsets[3]            = { 11.04F, 6.57F, 126.08F };

    /* Soft iron error compensation matrix */
    float mag_softiron_matrix[3][3] = { {  0.978,  -0.018,  0.004 },
                                        {  -0.018,  1.052, -0.005 },
                                        {  0.004, 0.005,  0.972 } };
    float mag_field_strength = 47.40f;

    /* Offsets applied to compensate for gyro zero-drift error for x/y/z */
    float gyro_zero_offsets[3] = { 0.0F, 0.0F, 0.0F };

    float x, y, z;
    gyro_float_data_t g;
    magn_float_data_t m;
    accel_float_data_t a;
    euler_f_t eu;

    /* Print and update gyro mainloop */
    while(1){
        vTaskDelayUntil( &xLastWakeTime, xPeriod );
    
        gyro_update(gyro);
        accel_update(accel);
        magn_update(magn);

        /* Apply mag offset compensation (base values in uTesla) */
        x = magn->converted.x - mag_offsets[0];
        y = magn->converted.y - mag_offsets[1];
        z = magn->converted.z - mag_offsets[2];

        /* Apply mag soft iron error compensation */
        m.x = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
        m.y = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
        m.z = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
 
        /* Apply gyro zero-rate error compensation */
        g.x = gyro->converted.x + gyro_zero_offsets[0];
        g.y = gyro->converted.y + gyro_zero_offsets[1];
        g.z = gyro->converted.z + gyro_zero_offsets[2];

        a.x = accel->converted.x;
        a.y = accel->converted.y;
        a.z = accel->converted.z;

        /* Process the filter */
        mahony_filter_update(filt, a, g, m);

        /* Print to euler orientation */
        eu = quat_to_eul(filt->q);
        printf("%2.3f %2.3f %2.3f \n", eu.yaw, eu.pitch, eu.roll);
    }

    /* Deallocate the variables */
    gyro_destroy(&gyro);
    accel_destroy(&accel);
    magn_destroy(&magn);
    mahony_filter_destroy(&filt);
}

/*!
* main app entrance
*/
void app_main()
{
    xTaskCreate(mahony_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}
