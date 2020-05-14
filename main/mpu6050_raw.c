/* i2c/mpu6050 - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/i2c.h"

#define BUFF_SIZE 6
#define DELAY_TIME_BETWEEN_ITEMS_MS   500 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO    18    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    19    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

#define MPU6050_ADDR  0x68    /*!< slave address for MPU6050 */
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define ACCE_START_ADD 0x3B    /*!< Register address to start reading raw acce values */
#define GYRO_START_ADD 0x43    /*!< Register address to start reading raw gyro values */

xSemaphoreHandle print_mux;
int MAXVALUE = 16384;
//uint8_t data_rd[BUFF_SIZE]; 

/**
 * @brief function to show buffer
 */
void disp_buf(uint8_t* buf, int len)
{
    int i;
    int16_t temp;
    for(i = 0; i < len; i++) {
        temp = ((buf[i] << 8) + buf[i +1]);
        printf("%d ", temp);
        i++;
    }
    printf("\n");
}


/**
 * @brief mpu6050_init, inittialize MPU6050
 */
esp_err_t mpu6050_init(i2c_port_t i2c_num)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x6B, ACK_CHECK_EN); //Power register address
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); //Power on internal chip 
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief mpu6050_read_acce Read raw acce values
 */
esp_err_t mpu6050_read_acce(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    //printf("mpu6050_read_acce ***********\n");
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ACCE_START_ADD, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    //disp_buf(data_rd, BUFF_SIZE);
    return ret;
}

/**
 * @brief impu6050_read_gyro Read raw gyro values
 */
esp_err_t mpu6050_read_gyro(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, GYRO_START_ADD, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief impu6050_read_gyro Read raw gyro values
 */
esp_err_t mpu6050_set4g(i2c_port_t i2c_num)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x1C, ACK_CHECK_EN);
    //i2c_master_write(cmd, 0b11101111, 1, ACK_CHECK_EN); // set +/- 4g
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
void i2c_master_init()
{
    printf("i2c master init ***********\n");
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


//void mpu6050_test_raw(void* arg)
void mpu6050_test_raw()
{
    printf("mpu6050_test_raw ***********\n");
    int ret;
    ret = mpu6050_init(I2C_MASTER_NUM);
    /*keep retrying until initialzation is successful*/
    while(ret != ESP_OK) {
        printf("INIT FAILED... Retry\n");
        vTaskDelay(100/ portTICK_RATE_MS);
        ret = mpu6050_init(I2C_MASTER_NUM);
    }
    printf("INIT SUCCESS...\n");
    vTaskDelay(100/ portTICK_RATE_MS); //Delay to init power on mpu6050
}

void mpu6050_myinit()
{
    //print_mux = xSemaphoreCreateMutex();
    i2c_master_init();
    mpu6050_test_raw();
    //mpu6050_set4g(I2C_MASTER_NUM);
    //xTaskCreate(mpu6050_test_raw, "mpu6050_test_raw", 1024 * 2, NULL, 10, NULL);
}