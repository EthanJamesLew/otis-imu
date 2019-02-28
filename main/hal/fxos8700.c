#include "fxos8700.h"

/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg) */
#define ACCEL_MG_LSB_2G (0.000244F)
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
#define ACCEL_MG_LSB_4G (0.000488F)
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg) */
#define ACCEL_MG_LSB_8G (0.000976F)
/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.1F)


accel_err_t accel_init(accel_t *accel){
    i2c_err_t ret;
    uint8_t* data_rd = (uint8_t*)malloc(sizeof(uint8_t)*ACCEL_BUFF_SIZE);
    uint8_t* data_wr = (uint8_t*)malloc(sizeof(uint8_t)*ACCEL_BUFF_SIZE);

    accel->i2c.addr = FXOS8700_ADDRESS;
    accel->i2c.clk_speed = I2C_MASTER_FAST_FREQ_HZ;
    accel->i2c.mode =I2C_MODE_TYPE_MASTER;
    accel->i2c.tx_buff_len = ACCEL_BUFF_SIZE;
    accel->i2c.rx_buff_len = ACCEL_BUFF_SIZE;

    ret = i2c_utils_setup(accel->i2c);
    if(ret != I2C_SUCCESS){
        while(1){
            printf("%d\n", ret );
        }
        return ACCEL_BUS_FAIL;
    }

    /* Setup the range */
    accel->range = ACCEL_RANGE;

    /* Clear raw data */
    accel->raw.x = 0;
    accel->raw.y = 0;
    accel->raw.z = 0;

    /* Check device ID */
    ret = i2c_utils_read(accel->i2c, FXOS8700_REGISTER_WHO_AM_I, data_rd, 1);
    if(data_rd[0] != FXOS8700_ID) {
        return ACCEL_ID_FAIL;
    }

    /* Place 0x00 into accel CTRL register to place device into standby*/
    data_wr[0] = FXOS8700_REGISTER_CTRL_REG1;
    data_wr[1] = 0x00;
    /* Set to standby mode (required to make changes to this register) */
    ret = i2c_utils_write(accel->i2c, data_wr, 2);
    
    /* Configure the accelerometer */
    switch (accel->range) {
        case (ACCEL_RANGE_2G):
        data_wr[0] = FXOS8700_REGISTER_XYZ_DATA_CFG;
        data_wr[1] = 0x00;
        ret = i2c_utils_write(accel->i2c, data_wr, 2);
        break;
        
        case (ACCEL_RANGE_4G):
        data_wr[0] = FXOS8700_REGISTER_XYZ_DATA_CFG;
        data_wr[1] = 0x01;
        ret = i2c_utils_write(accel->i2c, data_wr, 2);
        break;
        
        case (ACCEL_RANGE_8G):
        data_wr[0] = FXOS8700_REGISTER_XYZ_DATA_CFG;
        data_wr[1] = 0x02;
        ret = i2c_utils_write(accel->i2c, data_wr, 2);
        break;
    }

    /* High resolution */
    data_wr[0] = FXOS8700_REGISTER_CTRL_REG2;
    data_wr[1] = 0x02;
    ret = i2c_utils_write(accel->i2c, data_wr, 2);

    /* Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode */
    data_wr[0] = FXOS8700_REGISTER_CTRL_REG1;
    //data_wr[1] = 0x0D;
    data_wr[1] = 0x15;
    ret = i2c_utils_write(accel->i2c, data_wr, 2);

     /* Configure the magnetometer */
    /* Hybrid Mode, Over Sampling Rate = 16 */
    data_wr[0] = FXOS8700_REGISTER_MCTRL_REG1;
    data_wr[1] = 0x1F;
    ret = i2c_utils_write(accel->i2c, data_wr, 2);
    /* Jump to reg 0x33 after reading 0x06 */
    data_wr[0] = FXOS8700_REGISTER_MCTRL_REG2;
    data_wr[1] = 0x20;
    ret = i2c_utils_write(accel->i2c, data_wr, 2);


    free(data_rd);
    free(data_wr);

    return ACCEL_SUCCESS;
}

accel_err_t accel_update(accel_t *accel){
    i2c_err_t ret;
    uint8_t* data_rd = (uint8_t*)malloc(sizeof(uint8_t)*ACCEL_BUFF_SIZE);
    uint8_t* data_wr = (uint8_t*)malloc(sizeof(uint8_t)*ACCEL_BUFF_SIZE);

    /* clear the raw data */
    accel->raw.x = 0;
    accel->raw.y = 0;
    accel->raw.z = 0;

    ret = i2c_utils_read(accel->i2c, FXOS8700_REGISTER_STATUS, data_rd, 13);
    if(ret != I2C_SUCCESS)
        return ACCEL_BUS_FAIL;

    //uint8_t status = data_rd[0];
    uint8_t axhi = data_rd[1];
    uint8_t axlo = data_rd[2];
    uint8_t ayhi = data_rd[3];
    uint8_t aylo = data_rd[4];
    uint8_t azhi = data_rd[5];
    uint8_t azlo = data_rd[6];

    accel->converted.x = (int16_t)((axhi << 8) | axlo) >> 2;
    accel->converted.y = (int16_t)((ayhi << 8) | aylo) >> 2;
    accel->converted.z = (int16_t)((azhi << 8) | azlo) >> 2;

    accel->raw.x = accel->converted.x;
    accel->raw.y =  accel->converted.y;
    accel->raw.z = accel->converted.z;

    switch (accel->range) {
      case (ACCEL_RANGE_2G):
          accel->converted.x *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          accel->converted.y *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          accel->converted.z *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ACCEL_RANGE_4G):
          accel->converted.x *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          accel->converted.y *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          accel->converted.z *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ACCEL_RANGE_8G):
          accel->converted.x *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          accel->converted.y *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          accel->converted.z *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      break;
    }   

    free(data_rd);
    free(data_wr);

    return ACCEL_SUCCESS;
}

magn_err_t magn_init(magn_t *magn){
    i2c_err_t ret;
    uint8_t* data_rd = (uint8_t*)malloc(sizeof(uint8_t)*MAGN_BUFF_SIZE);
    uint8_t* data_wr = (uint8_t*)malloc(sizeof(uint8_t)*MAGN_BUFF_SIZE);

    magn->i2c.addr = FXOS8700_ADDRESS;
    magn->i2c.clk_speed = I2C_MASTER_FAST_FREQ_HZ;
    magn->i2c.mode =I2C_MODE_TYPE_MASTER;
    magn->i2c.tx_buff_len = ACCEL_BUFF_SIZE;
    magn->i2c.rx_buff_len = ACCEL_BUFF_SIZE;

    ret = i2c_utils_setup(magn->i2c);
    if(ret != I2C_SUCCESS)
        return MAGN_BUS_FAIL;

    /* Clear raw data */
    magn->raw.x = 0;
    magn->raw.y = 0;
    magn->raw.z = 0;

    /* Check device ID */
    ret = i2c_utils_read(magn->i2c, FXOS8700_REGISTER_WHO_AM_I, data_rd, 8);
    if(data_rd[0] != FXOS8700_ID)
        return MAGN_ID_FAIL;

    /* Set to standby mode (required to make changes to this register) */
    ret = i2c_utils_write(magn->i2c, data_wr, 2);

    /* High resolution */
    data_wr[0] = FXOS8700_REGISTER_CTRL_REG2;
    data_wr[1] = 0x02;
    ret = i2c_utils_write(magn->i2c, data_wr, 2);

    /* Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode */
    data_wr[0] = FXOS8700_REGISTER_CTRL_REG1;
    data_wr[1] = 0x15;
    ret = i2c_utils_write(magn->i2c, data_wr, 2);

    /* Configure the magnetometer */
    /* Hybrid Mode, Over Sampling Rate = 16 */
    data_wr[0] = FXOS8700_REGISTER_MCTRL_REG1;
    data_wr[1] = 0x1F;
    ret = i2c_utils_write(magn->i2c, data_wr, 2);
    /* Jump to reg 0x33 after reading 0x06 */
    data_wr[0] = FXOS8700_REGISTER_MCTRL_REG2;
    data_wr[1] = 0x20;
    ret = i2c_utils_write(magn->i2c, data_wr, 2);

    free(data_rd);
    free(data_wr);

    return MAGN_SUCCESS;

}

magn_err_t magn_update(magn_t *magn){
    //i2c_err_t ret;
    uint8_t* data_rd = (uint8_t*)malloc(sizeof(uint8_t)*ACCEL_BUFF_SIZE);
    uint8_t* data_wr = (uint8_t*)malloc(sizeof(uint8_t)*ACCEL_BUFF_SIZE);

    /* clear the raw data */
    magn->raw.x = 0;
    magn->raw.y = 0;
    magn->raw.z = 0;

    i2c_utils_read(magn->i2c, FXOS8700_REGISTER_STATUS, data_rd, 13);

    uint8_t mxhi = data_rd[7];
    uint8_t mxlo = data_rd[8];
    uint8_t myhi = data_rd[9];
    uint8_t mylo = data_rd[10];
    uint8_t mzhi = data_rd[11];
    uint8_t mzlo = data_rd[12];

    magn->converted.x = (int16_t)((mxhi << 8) | mxlo);
    magn->converted.y = (int16_t)((myhi << 8) | mylo);
    magn->converted.z = (int16_t)((mzhi << 8) | mzlo);

    magn->raw.x = magn->converted.x;
    magn->raw.y =  magn->converted.y;
    magn->raw.z = magn->converted.z;

    /* Convert mag values to uTesla */
    magn->converted.x *= MAG_UT_LSB;
    magn->converted.y *= MAG_UT_LSB;
    magn->converted.z *= MAG_UT_LSB;

    free(data_rd);
    free(data_wr);

    return MAGN_SUCCESS;
}