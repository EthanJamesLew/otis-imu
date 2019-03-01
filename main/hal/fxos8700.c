#include "fxos8700.h"

/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg) */
#define ACCEL_MG_LSB_2G (0.000244F)
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
#define ACCEL_MG_LSB_4G (0.000488F)
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg) */
#define ACCEL_MG_LSB_8G (0.000976F)
/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.1F)
/** Resample threshold */
#define FXOS_US_THRESHOLD 2000

/* Static ("singleton") device context */
static fxos8700_t* fxos8700; 
/* Resample trigger mechanism */
static timer_hal_t* fxos_timer;
/* How many sensors are programmer accessible */
static uint8_t fxos_reference_count = 0;

static fxos8700_err_t fxos8700_init(fxos8700_t *fxos);

static fxos8700_err_t fxos8700_update(fxos8700_t *fxos);

static fxos8700_err_t fxos8700_destroy(fxos8700_t **fxos);

accel_err_t accel_init(accel_t **accel){
    if(accel != NULL){
        *accel = (accel_t*)malloc(sizeof(accel_t));
    }

    /* If the static type hasn't been created, construct it*/
    fxos8700_err_t ret;
    if(fxos8700){
        ret = 0;
    } else {
        /* Startup sampling timer */
        fxos_timer = (timer_hal_t*)malloc(sizeof(timer_hal_t));
        start_hal_timer(fxos_timer);
        /* Create sensor context and update reference count */
        fxos8700 = (fxos8700_t*)malloc(sizeof(fxos8700_t));
        ret = fxos8700_init(fxos8700);
        fxos_reference_count++;
    }

    /* If error, return */
    if(ret != 0)
        return ret; 

    /*  Default out the values */
    (*accel)->fxos = fxos8700;
    (*accel)->raw.x = 0;
    (*accel)->raw.y = 0;
    (*accel)->raw.z = 0; 
    (*accel)->converted.x = 0;
    (*accel)->converted.y = 0;
    (*accel)->converted.z = 0;

    return ACCEL_SUCCESS;
}

accel_err_t accel_update(accel_t *accel){
    if(!accel){
        return ACCEL_NMALLOC;
    }
    /* Update the timer diff */
    update_hal_timer(fxos_timer);
    fxos8700_err_t ret;
    /* If diff time great anough, resample the data */
    if(fxos_timer->diff > FXOS_US_THRESHOLD) {
        /* Reset timer ewent -- exceed sample threshold */
        reset_hal_timer(fxos_timer);
        ret =  fxos8700_update(fxos8700);
    } else {
        ret =  ACCEL_SUCCESS;
    }

    /* Update in structure */
    accel->raw.x = fxos8700->a_raw.x;
    accel->raw.y = fxos8700->a_raw.y;
    accel->raw.z = fxos8700->a_raw.z;
    accel->converted.x = fxos8700->a_converted.x;
    accel->converted.y = fxos8700->a_converted.y;
    accel->converted.z = fxos8700->a_converted.z;

    return ret;
}

accel_err_t accel_destroy(accel_t **accel){
    if(accel){
        /* Destroy entire device if no longer avilable to the programmer */
        fxos_reference_count--;
        if(fxos_reference_count == 0) {
            fxos8700_destroy(&((*accel)->fxos));
        }
        free(*accel);
        *accel = NULL;
        return ACCEL_SUCCESS;
    } else {
        return ACCEL_NMALLOC;
    }
}

magn_err_t magn_init(magn_t **magn){
    if (magn != NULL){
        *magn = (magn_t*)malloc(sizeof(magn_t));
    }
    /* If the static type hasn't been created, construct it*/
    fxos8700_err_t ret;
    if(fxos8700){
        ret = 0;
    } else {
        fxos_timer = (timer_hal_t*)malloc(sizeof(timer_hal_t));
        start_hal_timer(fxos_timer);
        fxos8700 = (fxos8700_t*)malloc(sizeof(fxos8700_t));
        ret = fxos8700_init(fxos8700);
        fxos_reference_count++;
    }

    if(ret != 0)
        return ret; 

    (*magn)->fxos = fxos8700;
    (*magn)->raw.x = 0;
    (*magn)->raw.y = 0;
    (*magn)->raw.z = 0; 
    (*magn)->converted.x = 0;
    (*magn)->converted.y = 0;
    (*magn)->converted.z = 0; 

    return MAGN_SUCCESS;
}

magn_err_t magn_update(magn_t *magn){
    if(!magn){
        return MAGN_NMALLOC;
    }
    /* Update the timer diff */
    update_hal_timer(fxos_timer);
    fxos8700_err_t ret;
    /* If diff time great anough, resample the data */
    if(fxos_timer->diff > FXOS_US_THRESHOLD) {
        reset_hal_timer(fxos_timer);
        ret =  fxos8700_update(fxos8700);
    } else {
        ret = MAGN_SUCCESS;
    }

     /* Update in structure */
    magn->raw.x = fxos8700->m_raw.x;
    magn->raw.y = fxos8700->m_raw.y;
    magn->raw.z = fxos8700->m_raw.z;

    magn->converted.x = fxos8700->m_converted.x;
    magn->converted.y = fxos8700->m_converted.y;
    magn->converted.z = fxos8700->m_converted.z;

    return ret;
}

magn_err_t magn_destroy(magn_t **magn){
    if(magn){
        /* Destroy entire device if no longer avilable to the programmer */
        fxos_reference_count--;
        if(fxos_reference_count == 0) {
            fxos8700_destroy(&((*magn)->fxos));
        }
        free(*magn);
        *magn = NULL;
        return MAGN_SUCCESS;
    } else {
        return MAGN_NMALLOC;
    }
}


static fxos8700_err_t fxos8700_init(fxos8700_t *fxos){

    i2c_err_t ret;
    uint8_t* data_rd = (uint8_t*)malloc(sizeof(uint8_t)*FXOS_BUFF_SIZE);
    uint8_t* data_wr = (uint8_t*)malloc(sizeof(uint8_t)*FXOS_BUFF_SIZE);

    /* Setup the i2c device from macros */
    fxos->i2c.addr = FXOS8700_ADDRESS;
    fxos->i2c.clk_speed = I2C_MASTER_FAST_FREQ_HZ;
    fxos->i2c.mode =I2C_MODE_TYPE_MASTER;
    fxos->i2c.tx_buff_len = ACCEL_BUFF_SIZE;
    fxos->i2c.rx_buff_len = ACCEL_BUFF_SIZE;

    ret = i2c_utils_setup(fxos->i2c);
    if(ret != I2C_SUCCESS){
        while(1){
            printf("%d\n", ret );
        }
        return ACCEL_BUS_FAIL;
    }

    /* Setup the range */
    fxos->range = ACCEL_RANGE;

    /* Check device ID */
    ret = i2c_utils_read(fxos->i2c, FXOS8700_REGISTER_WHO_AM_I, data_rd, 1);
    if(data_rd[0] != FXOS8700_ID) {
        return FXOS8700_ID_FAIL;
    }

    /* Place 0x00 into accel CTRL register to place device into standby*/
    data_wr[0] = FXOS8700_REGISTER_CTRL_REG1;
    data_wr[1] = 0x00;
    /* Set to standby mode (required to make changes to this register) */
    ret = i2c_utils_write(fxos->i2c, data_wr, 2);
    
    /* Configure the accelerometer */
    switch (fxos->range) {
        case (ACCEL_RANGE_2G):
        data_wr[0] = FXOS8700_REGISTER_XYZ_DATA_CFG;
        data_wr[1] = 0x00;
        ret = i2c_utils_write(fxos->i2c, data_wr, 2);
        break;
        
        case (ACCEL_RANGE_4G):
        data_wr[0] = FXOS8700_REGISTER_XYZ_DATA_CFG;
        data_wr[1] = 0x01;
        ret = i2c_utils_write(fxos->i2c, data_wr, 2);
        break;
        
        case (ACCEL_RANGE_8G):
        data_wr[0] = FXOS8700_REGISTER_XYZ_DATA_CFG;
        data_wr[1] = 0x02;
        ret = i2c_utils_write(fxos->i2c, data_wr, 2);
        break;
    }

    /* High resolution */
    data_wr[0] = FXOS8700_REGISTER_CTRL_REG2;
    data_wr[1] = 0x02;
    ret = i2c_utils_write(fxos->i2c, data_wr, 2);

    /* Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode */
    data_wr[0] = FXOS8700_REGISTER_CTRL_REG1;
    data_wr[1] = 0x0D;
    //data_wr[1] = 0x15;
    ret = i2c_utils_write(fxos->i2c, data_wr, 2);

     /* Configure the magnetometer */
    /* Hybrid Mode, Over Sampling Rate = 16 */
    data_wr[0] = FXOS8700_REGISTER_MCTRL_REG1;
    data_wr[1] = 0x1F;
    ret = i2c_utils_write(fxos->i2c, data_wr, 2);
    /* Jump to reg 0x33 after reading 0x06 */
    data_wr[0] = FXOS8700_REGISTER_MCTRL_REG2;
    data_wr[1] = 0x20;
    ret = i2c_utils_write(fxos->i2c, data_wr, 2);


    free(data_rd);
    free(data_wr);

    return FXOS8700_SUCCESS;

}

static fxos8700_err_t fxos8700_update(fxos8700_t *fxos){
    if(!fxos) {
        return FXOS8700_NMALLOC;
    }

    i2c_err_t ret;
    uint8_t* data_rd = (uint8_t*)malloc(sizeof(uint8_t)*ACCEL_BUFF_SIZE);
    uint8_t* data_wr = (uint8_t*)malloc(sizeof(uint8_t)*ACCEL_BUFF_SIZE);

    /* clear the raw data */
    fxos->a_raw.x = 0;
    fxos->a_raw.y = 0;
    fxos->a_raw.z = 0;

    fxos->m_raw.x = 0;
    fxos->m_raw.y = 0;
    fxos->m_raw.z = 0;

    /* Read in 13 bytes:
    * [0] dev status
    * [1] upper accelerometer x-axis byte
    * [2] lower accelerometer x-axis byte
    * [3] upper accelerometer y-axis byte
    * [4] lower accelerometer y-axis byte
    * [5] upper accelerometer z-axis byte
    * [6] lower accelerometer z-axis byte
    * [7] upper magnetometer x-axis byte
    * [8] lower magnetometer x-axis byte
    * [9] upper magnetometer y-axis byte
    * [10] lower magnetometer y-axis byte
    * [11] upper magnetometer z-axis byte
    * [12] lower magnetometer z-axis byte
    */
    ret = i2c_utils_read(fxos->i2c, FXOS8700_REGISTER_STATUS, data_rd, 13);
    if(ret != I2C_SUCCESS)
        return FXOS8700_BUS_FAIL;

    uint8_t axhi = data_rd[1];
    uint8_t axlo = data_rd[2];
    uint8_t ayhi = data_rd[3];
    uint8_t aylo = data_rd[4];
    uint8_t azhi = data_rd[5];
    uint8_t azlo = data_rd[6];

    uint8_t mxhi = data_rd[7];
    uint8_t mxlo = data_rd[8];
    uint8_t myhi = data_rd[9];
    uint8_t mylo = data_rd[10];
    uint8_t mzhi = data_rd[11];
    uint8_t mzlo = data_rd[12];

    /* form accelerometer values */
    fxos->a_converted.x = (int16_t)((axhi << 8) | axlo) >> 2;
    fxos->a_converted.y = (int16_t)((ayhi << 8) | aylo) >> 2;
    fxos->a_converted.z = (int16_t)((azhi << 8) | azlo) >> 2;

    fxos->a_raw.x = fxos->a_converted.x;
    fxos->a_raw.y =  fxos->a_converted.y;
    fxos->a_raw.z = fxos->a_converted.z;

    /* Apply range correction */
    switch (fxos->range) {
      case (ACCEL_RANGE_2G):
          fxos->a_converted.x *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          fxos->a_converted.y *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          fxos->a_converted.z *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ACCEL_RANGE_4G):
          fxos->a_converted.x *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          fxos->a_converted.y *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          fxos->a_converted.z *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ACCEL_RANGE_8G):
          fxos->a_converted.x *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          fxos->a_converted.y *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          fxos->a_converted.z *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      break;
    }   

    /* form magnetometer values */
    fxos->m_converted.x = (int16_t)((mxhi << 8) | mxlo);
    fxos->m_converted.y = (int16_t)((myhi << 8) | mylo);
    fxos->m_converted.z = (int16_t)((mzhi << 8) | mzlo);

    fxos->m_raw.x = fxos->m_converted.x;
    fxos->m_raw.y =  fxos->m_converted.y;
    fxos->m_raw.z = fxos->m_converted.z;

    /* Convert mag values to uTesla */
    fxos->m_converted.x *= MAG_UT_LSB;
    fxos->m_converted.y *= MAG_UT_LSB;
    fxos->m_converted.z *= MAG_UT_LSB;

    free(data_rd);
    free(data_wr);

    return FXOS8700_SUCCESS;
}

static fxos8700_err_t fxos8700_destroy(fxos8700_t **fxos){
    if(fxos) {
        free(*fxos);
        *fxos = NULL;
        return FXOS8700_SUCCESS;
    } else {
        return FXOS8700_NMALLOC;
    }
}