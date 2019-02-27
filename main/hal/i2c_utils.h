/*!
* @file i2c_utils.h
* @author Ethan Lew
* @brief Abstract esp32 i2c features into a generic feature set
*/

#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include <stdlib.h>
#include <stdio.h>
#include "driver/i2c.h"
#include "sdkconfig.h"


#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_FAST_FREQ_HZ 400000        /*!< I2C master clock frequency */
#define I2C_MASTER_NORMAL_FREQ_HZ 100000        /*!< I2C master clock frequency */

#define I2C_SLAVE_NUM I2C_NUMBER(1) /*!< I2C port number for slave dev */
#define I2C_MASTER_NUM I2C_NUMBER(0) /*!< I2C port number for master dev */

#define I2C_MASTER_SCL_IO 22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21               /*!< gpio number for I2C master data  */
#define I2C_SLAVE_SCL_IO 36               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 39               /*!< gpio number for i2c slave data */

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

/*!
*  Simple enum to select master/slave
*/
typedef enum {
    I2C_MODE_TYPE_MASTER = 0x0,
    I2C_MODE_TYPE_SLAVE = 0x1,
} i2c_mode_type_t;

/*!
* Parameters of the i2c peripheral 
* To setup the ESP32 peripheral, it is necessary to specify
*   addr (eg        I2C_ADDRESS)
*   mode (eg        I2C_MODE_MASTER/I2C_MODE_SLAVE)
*   clk_speed (eg   I2C_MASTER_FREQ_HZ)
*   tx_buffer_len (eg I2C_MASTER_TX_BUF_DISABLE/I2C_SLAVE_TX_BUF_LEN)
*   rx_buffer_len (eg  I2C_MASTER_RX_BUF_DISABLE/I2C_SLAVE_RX_BUF_LEN)
*/
typedef struct i2c_peripheral_s {
    uint8_t addr;
    i2c_mode_type_t mode;
    uint32_t clk_speed;
    size_t tx_buff_len;
    size_t rx_buff_len;
} i2c_peripheral_t;

/*!
* Generic i2c errors
*/
typedef enum {
    I2C_SUCCESS = 0x0,
    I2C_INSTALL_ERROR = 0x1,
    I2C_INVALID_SETUP = 0x2,
    I2C_TIMEOUT = 0x3,
    I2C_INVALID_STATE = 0x4,
    I2C_FAIL = 0x5,
} i2c_err_t;

/*!
* @brief setup an i2c peripheral before use in master mode
* @param i2c_setup a struct containing the necessary setup parameters
* @returns i2c status
*/
i2c_err_t i2c_utils_setup(i2c_peripheral_t i2c_setup);

/*!
* @brief read size bytes from i2c device (master mode)
* @param i2c_dev the target peripheral parameters
* @param data_rd the read data buffer
* @param size the number of bytes read
* @returns i2c status
*/
i2c_err_t i2c_utils_read(i2c_peripheral_t i2c_dev, uint8_t i2c_reg, uint8_t *data_rd, size_t size);

/*!
* @brief write size bytes to i2c device (master mode)
* @param i2c_dev the target peripheral structure
* @param data_wr data to write to the target device
* @param size the number of bytes to write
* @returns i2c status
*/
i2c_err_t i2c_utils_write(i2c_peripheral_t i2c_dev, uint8_t *data_wr, size_t size);

#endif