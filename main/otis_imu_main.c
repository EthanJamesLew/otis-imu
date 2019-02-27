/*!
* @file otis_imu_main.c
* @author Ethan Lew
*
* For now, the main task grabs gyroscope data and prints it to the serial device.
*/
#include "hal/fxas21002c.h"
#include "hal/fxos8700.h"

#define SENSOR_BUFF_SIZE 1
#define GYRO_REGISTER_WHO_AM_I 0x0C

static void gyro_test_task(void *arg)
{
    /* Timing parameters */
    const TickType_t init_wait = 100 / portTICK_PERIOD_MS;
    const TickType_t sample_period = 10 / portTICK_PERIOD_MS;


    /* Create generic gyroscope */
    gyro_err_t g_ret;
    gyro_t* gyro = (gyro_t*)malloc(sizeof(gyro_t));
    //g_ret = gyro_init(gyro);
    //vTaskDelay(init_wait);

    accel_err_t a_ret;
    accel_t* accel = (accel_t*)malloc(sizeof(accel_t));
    a_ret = accel_init(accel);
    vTaskDelay(init_wait);

    if(a_ret != ACCEL_SUCCESS)
    {
        while(1)
        {
            printf("Accel init failed with %d\n", a_ret);
        }
    }

    magn_err_t m_ret;
    magn_t* magn = (magn_t*)malloc(sizeof(magn_t));
    //m_ret = magn_init(magn);

    /* Print and update gyro mainloop */
    while(1){
        vTaskDelay(sample_period);
        printf("Data: %d %d %d \n", accel->raw.x, accel->raw.y, accel->raw.z);
        //gyro_update(gyro);
        accel_update(accel);
        //magn_update(magn);
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