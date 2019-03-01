/*!
* @file otis_imu_main.c
* @author Ethan Lew
*
* For now, the main task grabs gyroscope data and prints it to the serial device.
*/
#include "hal/fxas21002c.h"
#include "hal/fxos8700.h"
#include "hal/time_utils.h"

#define SAMPLE_PERIOD 10

static void gyro_test_task(void *arg)
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

    /* Print and update gyro mainloop */
    while(1){
        vTaskDelayUntil( &xLastWakeTime, xPeriod );
        printf("%2.3f %2.3f %2.3f ", accel->converted.x, accel->converted.y, accel->converted.z);
        printf("%2.3f %2.3f %2.3f ", gyro->converted.x, gyro->converted.y, gyro->converted.z);
        printf("%2.3f %2.3f %2.3f \n", magn->converted.x, magn->converted.y, magn->converted.z);
        gyro_update(gyro);
        accel_update(accel);
        magn_update(magn);
    }
    
    gyro_destroy(&gyro);
    accel_destroy(&accel);
    magn_destroy(&magn);
}

/*!
* main app entrance
*/
void app_main()
{
    xTaskCreate(gyro_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}
