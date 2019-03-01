#include "i2c_utils.h"

static int I2C_DRIVER_INSTALLED = 0;

/*!
*  The i2c_setup for a ESP32 i2c peripheral works as follows
*   1. Setup a i2c_config_t struct
*   2. Setup the operation mode with a i2c_opmode_t struct (optional)
*   3. Setup the clock speed
*   4. Setup mode (master in this case)
* 
*   In this case, the clock speed is setup in the master configuration.
*/
i2c_err_t i2c_utils_setup(i2c_peripheral_t i2c_setup)
{
    esp_err_t ret = ESP_OK;

    /* Populate a i2c_config_t type*/
    i2c_config_t conf_dev;
    conf_dev.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_dev.scl_pullup_en = GPIO_PULLUP_ENABLE;

    /* Slave/Master Specific Configuration */
    if (i2c_setup.mode == I2C_MODE_TYPE_SLAVE){
        int i2c_port = I2C_SLAVE_NUM;
        conf_dev.sda_io_num = I2C_SLAVE_SDA_IO;
        conf_dev.scl_io_num = I2C_SLAVE_SCL_IO;
        conf_dev.mode = I2C_MODE_SLAVE;
        conf_dev.slave.addr_10bit_en = 0;
        conf_dev.slave.slave_addr = i2c_setup.addr;
        i2c_param_config(i2c_port, &conf_dev);
        if (I2C_DRIVER_INSTALLED == 0){
            ret =  i2c_driver_install(i2c_port, conf_dev.mode,
                                    i2c_setup.rx_buff_len,
                                    i2c_setup.tx_buff_len, 0);
            I2C_DRIVER_INSTALLED = 1;
        }
    } else {
        int i2c_port = I2C_MASTER_NUM;
        conf_dev.sda_io_num = I2C_MASTER_SDA_IO;
        conf_dev.scl_io_num = I2C_MASTER_SCL_IO;
        conf_dev.mode = I2C_MODE_MASTER;
        conf_dev.master.clk_speed = i2c_setup.clk_speed;

        i2c_param_config(i2c_port, &conf_dev);
        if (I2C_DRIVER_INSTALLED == 0){
            ret =  i2c_driver_install(i2c_port, conf_dev.mode,
                                    i2c_setup.rx_buff_len,
                                    i2c_setup.tx_buff_len, 0);
            I2C_DRIVER_INSTALLED = 1;
        }
    }

    /* Interpret Error Results */
    if(ret == ESP_OK){
        return I2C_SUCCESS;
    } else if (ret == ESP_ERR_INVALID_ARG) {
        return I2C_INVALID_SETUP;
    } else {
        return I2C_INSTALL_ERROR;
    }
}

/*!
* The ESP32 initiates as master read as
* 1. Create a command link
* 2. Address the peripheral by writing a byte
* 3. Read N-1 bytes with ACK
* 4. Read last byte with NACK
* 5. Send stop bit
* 6. Start the transmission
* 7. Delete the command link
*/
i2c_err_t i2c_utils_read(i2c_peripheral_t i2c_dev, uint8_t i2c_reg, uint8_t *data_rd, size_t size)
{
    esp_err_t ret;

    /* Nothing to read case */
    if (size == 0) {
        return I2C_SUCCESS;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    /* Add a start bit */
    i2c_master_start(cmd);
    /* Address the peripheral */
    i2c_master_write_byte(cmd, (i2c_dev.addr << 1), ACK_CHECK_EN); 
    /* Write register */
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    /* Send repeated start */
    i2c_master_start(cmd);
    /* Readdress the peripheral */
    i2c_master_write_byte(cmd, (i2c_dev.addr << 1) | READ_BIT, ACK_CHECK_EN); 
    /* ACK all but last byte */
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    /* NACK the last byte */
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    /* Add stop bit */
    i2c_master_stop(cmd);
    /* Start the transmission */
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 500 / portTICK_RATE_MS);
    /* delete the link */
    i2c_cmd_link_delete(cmd);

    /* Interpret output */
    switch(ret) {
        case ESP_OK:
            return I2C_SUCCESS;
            break;
        case I2C_INVALID_SETUP:
            return I2C_INVALID_SETUP;
            break;
        case ESP_ERR_TIMEOUT:
            return I2C_TIMEOUT;
            break;
        default:
            return I2C_FAIL;
    }
}

/*!
* 1. Create a command link
* 2. Add start bit
* 3. Address peripheral by write byte
* 4. Send N bytes of data
* 5. Add stop bit
* 6. Begin transmission
* 7. Delete the command link
*/
i2c_err_t i2c_utils_write(i2c_peripheral_t i2c_dev, uint8_t *data_wr, size_t size)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_dev.addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 500 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    /* Interpret output */
    switch(ret) {
        case ESP_OK:
            return I2C_SUCCESS;
            break;
        case I2C_INVALID_SETUP:
            return I2C_INVALID_SETUP;
            break;
        case ESP_ERR_TIMEOUT:
            return I2C_TIMEOUT;
            break;
        default:
            return I2C_FAIL;
    }
}