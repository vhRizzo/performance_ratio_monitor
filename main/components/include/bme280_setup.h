/*
 * Fonte: https://github.com/SFeli/ESP32_BME280_IDF.
 *
 * Adapted from SFelis' code available at the link above. All changes to source
 * code are documented with a "-N" identifier at the end.
 *
 * In the source code, everything is done inside main.c. Here we created this
 * header file to declare functions and include libraries and also created a
 * source file to implement only BME280-related functions.
 */

#ifndef BME280_SETUP_H_
#define BME280_SETUP_H_

// All libraries used throughout the project are included in this header file -N
#include "global_data.h"

/* Wrapped the code with this condition for easy deactivation of the device when
 * not using it for debug purposes. -N */
#ifdef BME280_SENSOR
#include "bme280.h"

#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY
#define RTOS_DELAY_1SEC (1 * 1000 / portTICK_PERIOD_MS)

void i2c_master_init();
int8_t i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length,
                    void *intf_ptr);
int8_t i2c_reg_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length,
                     void *intf_ptr);
void delay_us(uint32_t us, void *intf_ptr);
void print_rslt(const char api_name[], int8_t rslt);
void store_data(struct bme280_data *comp_data);
void bme280_task(void *pvParameters);

#endif /* BME280_SENSOR */
#endif /* BME280_SETUP_H_ */