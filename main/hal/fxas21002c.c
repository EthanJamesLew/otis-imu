#include "fxas21002c.h"

gyro_err_t gyro_init(gyro_t **gyro){
    if(gyro != NULL){
        *gyro = (gyro_t*)malloc(sizeof(gyro_t));
    }

    /* Setup the I2C device */
    i2c_err_t ret;
    uint8_t* data_rd = (uint8_t*)malloc(sizeof(uint8_t)*GYRO_BUFF_SIZE);
    uint8_t* data_wr = (uint8_t*)malloc(sizeof(uint8_t)*GYRO_BUFF_SIZE);
    (*gyro)->i2c.addr = FXAS21002C_ADDRESS;
    (*gyro)->i2c.clk_speed = I2C_MASTER_FAST_FREQ_HZ;
    (*gyro)->i2c.mode =I2C_MODE_TYPE_MASTER;
    (*gyro)->i2c.tx_buff_len = GYRO_BUFF_SIZE;
    (*gyro)->i2c.rx_buff_len = GYRO_BUFF_SIZE;
    ret = i2c_utils_setup((*gyro)->i2c);
    if(ret != I2C_SUCCESS)
        return GYRO_BUS_FAIL;

    /* Clear raw data */
    (*gyro)->raw.x = 0;
    (*gyro)->raw.y = 0;
    (*gyro)->raw.z = 0;

    /* Check device ID */
    ret = i2c_utils_read((*gyro)->i2c, GYRO_REGISTER_WHO_AM_I, data_rd, 8);
    if(data_rd[0] != FXAS21002C_ID)
        return GYRO_ID_FAIL;
    
    /* Setup the gyro range */
    (*gyro)->range = GYRO_RANGE;
    uint8_t ctrlReg0 = 0x00;
    switch((*gyro)->range)
    {
        case GYRO_RANGE_250DPS:
        ctrlReg0 = 0x03;
        break;
        case GYRO_RANGE_500DPS:
        ctrlReg0 = 0x02;
        break;
        case GYRO_RANGE_1000DPS:
        ctrlReg0 = 0x01;
        break;
        case GYRO_RANGE_2000DPS:
        ctrlReg0 = 0x00;
        break;
    }

    data_wr[0] = GYRO_REGISTER_CTRL_REG1;
    data_wr[1] = 0x00;
    ret = i2c_utils_write((*gyro)->i2c, data_wr, 2);

    data_wr[1] = (1 << 6);
    ret = i2c_utils_write((*gyro)->i2c, data_wr, 2);

    data_wr[0] = GYRO_REGISTER_CTRL_REG0;
    data_wr[1] = ctrlReg0;
    ret = i2c_utils_write((*gyro)->i2c, data_wr, 2);

    data_wr[0] = GYRO_REGISTER_CTRL_REG1;
    data_wr[1] = 0x0E;
    ret = i2c_utils_write((*gyro)->i2c, data_wr, 2);
    
    free(data_rd);
    free(data_wr);

    return GYRO_SUCCESS;

}

gyro_err_t gyro_update(gyro_t *gyro){
    if(!gyro) {
        return GYRO_NMALLOC;
    }

    i2c_err_t ret;
    uint8_t* data_rd = (uint8_t*)malloc(sizeof(uint8_t)*GYRO_BUFF_SIZE);
    uint8_t* data_wr = (uint8_t*)malloc(sizeof(uint8_t)*GYRO_BUFF_SIZE);

    /* Clear raw data */
    gyro->raw.x = 0;
    gyro->raw.y = 0;
    gyro->raw.z = 0;

    ret = i2c_utils_read(gyro->i2c, GYRO_REGISTER_STATUS | 0x80, data_rd, 7);
    if(ret != I2C_SUCCESS)
        return GYRO_BUS_FAIL;

    //uint8_t status = data_rd[0];
    uint8_t xhi = data_rd[1];
    uint8_t xlo = data_rd[2];
    uint8_t yhi = data_rd[3];
    uint8_t ylo = data_rd[4];
    uint8_t zhi = data_rd[5];
    uint8_t zlo = data_rd[6];

    gyro->converted.x = (int16_t)((xhi << 8) | xlo);
    gyro->converted.y = (int16_t)((yhi << 8) | ylo);
    gyro->converted.z = (int16_t)((zhi << 8) | zlo);

    gyro->raw.x = gyro->converted.x;
    gyro->raw.y =  gyro->converted.y;
    gyro->raw.z = gyro->converted.z;

    switch(gyro->range)
    {
        case GYRO_RANGE_250DPS:
        gyro->converted.x *= GYRO_SENSITIVITY_250DPS;
        gyro->converted.y *= GYRO_SENSITIVITY_250DPS;
        gyro->converted.z *= GYRO_SENSITIVITY_250DPS;
        break;
        case GYRO_RANGE_500DPS:
        gyro->converted.x *= GYRO_SENSITIVITY_500DPS;
        gyro->converted.y *= GYRO_SENSITIVITY_500DPS;
        gyro->converted.z *= GYRO_SENSITIVITY_500DPS;
        break;
        case GYRO_RANGE_1000DPS:
        gyro->converted.x *= GYRO_SENSITIVITY_1000DPS;
        gyro->converted.y *= GYRO_SENSITIVITY_1000DPS;
        gyro->converted.z *= GYRO_SENSITIVITY_1000DPS;
        break;
        case GYRO_RANGE_2000DPS:
        gyro->converted.x *= GYRO_SENSITIVITY_2000DPS;
        gyro->converted.y *= GYRO_SENSITIVITY_2000DPS;
        gyro->converted.z *= GYRO_SENSITIVITY_2000DPS;
        break;
    }

    gyro->converted.x *= SENSORS_DPS_TO_RADS;
    gyro->converted.y *= SENSORS_DPS_TO_RADS;
    gyro->converted.z *= SENSORS_DPS_TO_RADS;
        
    free(data_rd);
    free(data_wr);

    return GYRO_SUCCESS;
}

gyro_err_t gyro_destroy(gyro_t **gyro){
    if(gyro){
        free(*gyro);
        *gyro = NULL;
        return GYRO_SUCCESS;
    } else {
        return GYRO_NMALLOC;
    }
}