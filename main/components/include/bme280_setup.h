/*
 * Adaptado do codigo de https://github.com/SFeli/ESP32_BME280_IDF.
 *
 * Adapted from SFelis' code available at the link above with changes documented.
 */

#ifndef BME280_SETUP_H_
#define BME280_SETUP_H_

#include "global_data.h"
#include "bme280.h"

#define I2C_MASTER_FREQ_HZ      CONFIG_I2C_MASTER_FREQUENCY
#define RTOS_DELAY_1SEC         ( 1 * 1000 / portTICK_PERIOD_MS)

void i2c_master_init ();
int8_t i2c_reg_read  ( uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr );
int8_t i2c_reg_write ( uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr );
void delay_us        ( uint32_t us, void *intf_ptr );
void print_rslt      ( const char api_name[], int8_t rslt );
void store_data      ( struct bme280_data *comp_data );
void bme280_task     ( void *pvParameters );

#endif /* BME280_SETUP_H_ */
