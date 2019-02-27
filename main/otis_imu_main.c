/*!
* @file otis_imu_main.c
* @author Ethan Lew
*
* For now, the main task grabs gyroscope data and prints it to the serial device.
*/
#include "hal/fxas21002c.h"

#define SENSOR_BUFF_SIZE 1
#define GYRO_REGISTER_WHO_AM_I 0x0C

static void gyro_test_task(void *arg)
{
    /* Timing parameters */
    const TickType_t init_wait = 50 / portTICK_PERIOD_MS;
    const TickType_t sample_period = 10 / portTICK_PERIOD_MS;


    /* Create generic gyroscope */
    gyro_err_t ret;
    gyro_t* gyro = (gyro_t*)malloc(sizeof(gyro_t));
    ret = gyro_init(gyro);
    vTaskDelay(init_wait);

    /* Print and update gyro mainloop */
    while(1){
        vTaskDelay(sample_period);
        printf("%2.3f %2.3f %2.3f \n", gyro->converted.x, gyro->converted.y, gyro->converted.z);
        gyro_update(gyro);
    }
    
    free(gyro);
}

/*!
* main app entrance
*/
void app_main()
{
    xTaskCreate(gyro_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}