/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"


static const char *TAG = "main";

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a ADS1115 Analog to Digital converter for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP8266 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO2 is assigned as the data signal of i2c master port
 *    GPIO0 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of ADS1115 with GPIO2/GPIO0
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */

#define I2C_EXAMPLE_MASTER_SCL_IO           	2                	/*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           	0               	/*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              	I2C_NUM_0        	/*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   	0                	/*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   	0                	/*!< I2C master do not need buffer */

/**
 * Define the ESP8266 register address:
 */
#define ADS1115_SENSOR_ADDR 					0x48		     	/*!< slave address for ADS1115 sensor */
#define GENERAL_CALL							0x00				/*!< general call address */

#define CONVERSION_REG							0x00				/*!< address to conversion register */
#define CONFIG_REG								0x01				/*!< address to configuration register */

#define WRITE_BIT                           	I2C_MASTER_WRITE 	/*!< I2C master write */
#define READ_BIT                            	I2C_MASTER_READ  	/*!< I2C master read */
#define ACK_CHECK_EN                        	0x1              	/*!< I2C master will check ack from slave*/
#define LAST_NACK_VAL                       	0x2              	/*!< I2C last_nack value */




/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
	
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

/**
 * @brief test code to write ESP8266
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_example_master_ads1115_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_example_master_ads1115_write2(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data_msb, uint8_t *data_lsb, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data_msb, data_len, ACK_CHECK_EN);
    i2c_master_write(cmd, data_lsb, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief test code to read mpu6050
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_example_master_ads1115_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_example_master_ads1115_init(i2c_port_t i2c_num)
{
    uint8_t cmd_data;
    uint8_t cmd_data_msb;
	uint8_t cmd_data_lsb;

    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_example_master_init();
	
    cmd_data = 0x06;    // issue general call command to reset the ads1115
    ESP_ERROR_CHECK(i2c_example_master_ads1115_write(i2c_num, GENERAL_CALL, &cmd_data, 1));

    ESP_LOGI(TAG, "Setting up Config Register\n");
	
    cmd_data_msb = 0x45;    // Setup the configuration register; (0100010110000011)
    cmd_data_lsb = 0x83;
    ESP_ERROR_CHECK(i2c_example_master_ads1115_write2(i2c_num, CONFIG_REG, &cmd_data_msb, &cmd_data_lsb, 1));

    ESP_LOGI(TAG, "Executed\n");
    
	/*
			OS = 0
			MUX = 100
			PGA = 010
			MODE = 1
			DR = 100
			COMP_MODE = 0
			COMP_POL = 0
			COMP_LAT = 0
			COMP_QUE = 11
	*/
	

	
    return ESP_OK;
}

static void i2c_task_example(void *arg)
{
    ESP_LOGI(TAG, "Task Started\n");
    
    uint8_t cmd_data_msb;
	uint8_t cmd_data_lsb;
	uint8_t data[2];
    int ret = -1;
	double VOLTAGE;
    uint16_t voltage = 0;

    i2c_example_master_ads1115_init(I2C_EXAMPLE_MASTER_NUM);

    ESP_LOGI(TAG, "*******************\n");
			
    ESP_LOGI(TAG, "ADC Sensor values stored in CONEVRSION REGISTER\n");


    while (1) {
		//start a conversion by setting the OS bit
        cmd_data_msb = 0xC5;    // Setup the configuration register; (0100010110000011)
        cmd_data_lsb = 0x83;
        i2c_example_master_ads1115_write2(I2C_EXAMPLE_MASTER_NUM, CONFIG_REG, &cmd_data_msb, &cmd_data_lsb, 1);

		//delay a for while
		vTaskDelay(100 / portTICK_RATE_MS);
		  
		//read convserion
        memset(data, 0, 2);
        ret = i2c_example_master_ads1115_read(I2C_EXAMPLE_MASTER_NUM, CONVERSION_REG, data, 2);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Data: %d.%d\n\n", data[0], data[1]);

            voltage = (data[0] << 8 | data[1]);
			
			VOLTAGE = (double)(voltage/32768.0) * 3.3;
			
            ESP_LOGI(TAG, "VOLTAGE: %d.%d\n\n", (uint16_t)VOLTAGE, (uint16_t)(VOLTAGE * 10) % 10);
        } else {
            ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
        } 
    }
    
    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

void app_main(void)
{
    //start i2c task
    xTaskCreate(i2c_task_example, "i2c_task_example", 2048, NULL, 10, NULL);
}