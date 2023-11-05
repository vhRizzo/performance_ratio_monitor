#include "bme280_setup.h"
#ifdef BME280_SENSOR

/* Removed unused defines and moved used ones to either this component's header
 * or global_data header. -N */
struct bme280_data comp_data;
struct bme280_dev bme;
uint8_t settings_sel;
int8_t bmestatus = BME280_OK;
bme280_t bme_dados;       // Item to store sensor's data -N
QueueHandle_t bme_queue;  // Queue that will send the item above -N

void i2c_master_init() {
    /* We are using two i2c devices in this project, so i2c initialization was
     * moved to a global component. -N */
    //*************// Verify that the I2C slave is working properly
    esp_err_t f_retval;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR_PRIM << 1) | I2C_MASTER_WRITE,
                          true);
    i2c_master_stop(cmd);
    f_retval = i2c_master_cmd_begin(I2C_PORT, cmd, RTOS_DELAY_1SEC);
    if (f_retval != ESP_OK) {
#ifdef SERIAL_DEBUG
        // Changed print to log -N
        ESP_LOGE(
            __func__,
            "I2C slave NOT working or wrong I2C slave address - error (%i)",
            f_retval);
#endif
    }
    i2c_cmd_link_delete(cmd);
}

int8_t i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length,
                    void *intf_ptr) {
    int8_t iError;
    int8_t i2c_addr = *((int *)intf_ptr);
    esp_err_t esp_err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg_addr, true);
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd_handle, reg_data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd_handle, reg_data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd_handle);
    esp_err =
        i2c_master_cmd_begin(I2C_PORT, cmd_handle, 1000 / portTICK_PERIOD_MS);

    if (esp_err == ESP_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    i2c_cmd_link_delete(cmd_handle);
    return iError;
}

int8_t i2c_reg_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length,
                     void *intf_ptr) {
    int8_t i2c_addr = *((int *)intf_ptr);
    int8_t iError;
    esp_err_t esp_err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg_addr, true);
    i2c_master_write(cmd_handle, reg_data, length, true);
    i2c_master_stop(cmd_handle);
    esp_err =
        i2c_master_cmd_begin(I2C_PORT, cmd_handle, 1000 / portTICK_PERIOD_MS);
    if (esp_err == ESP_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    i2c_cmd_link_delete(cmd_handle);
    return iError;
}

/* Changed to microseconds to match Bosch's pattern, also changed to FreeRTOS'
 * delay method. -N */
void delay_us(uint32_t us, void *intf_ptr) {
    vTaskDelay((us / 1000) / portTICK_PERIOD_MS);
}

void print_rslt(const char api_name[], int8_t rslt) {
    // Same as before, changed all prints to logs -N
    if (rslt != BME280_OK) {
        if (rslt == BME280_E_NULL_PTR) {
            ESP_LOGE(api_name, "Error [%d] : Null pointer error", rslt);
        } else if (rslt == BME280_E_DEV_NOT_FOUND) {
            ESP_LOGE(api_name, "Error [%d] : Device not found", rslt);
        } else if (rslt == BME280_E_INVALID_LEN) {
            ESP_LOGE(api_name, "Error [%d] : Invalid Lenght", rslt);
        } else if (rslt == BME280_E_COMM_FAIL) {
            ESP_LOGE(api_name, "Error [%d] : Bus communication failed", rslt);
        } else if (rslt == BME280_E_SLEEP_MODE_FAIL) {
            ESP_LOGE(api_name, "Error [%d] : SLEEP_MODE_FAIL", rslt);
        } else {
            /* For more error codes refer "*_defs.h" */
            ESP_LOGE(api_name, "Error [%d] : Unknown error code", rslt);
        }
    } else {
        ESP_LOGI(api_name, " BME280 status [%d]", rslt);
    }
}

/* Read the sensor's data and send it through the queue -N. */
void store_data(struct bme280_data *comp_data) {
    bme_dados.temperatura = comp_data->temperature;
    bme_dados.umidade = comp_data->humidity;
    bme_dados.pressao = comp_data->pressure;
    xQueueOverwrite(bme_queue, &bme_dados);
}

/* Only forced mode will be used, so normal mode was removed, and the function
 * was changed to FreeRTOS task's pattern. -N */
void bme280_task(void *pvParameters) {
    // Stuff from app_main was moved here to be called before the loop -N
    i2c_master_init();

    uint8_t dev_addr = BME280_I2C_ADDR_PRIM;  // 0x76
    bme.intf_ptr = &dev_addr;
    bme.intf = BME280_I2C_INTF;
    bme.read = i2c_reg_read;
    bme.write = (bme280_write_fptr_t)i2c_reg_write;
    bme.delay_us = delay_us;

    bmestatus = bme280_init(&bme);
#ifdef SERIAL_DEBUG
    print_rslt("bme280_init status", bmestatus);
#endif

    /* Recommended mode of operation: Indoor navigation */
    bme.settings.osr_h = BME280_OVERSAMPLING_1X;
    bme.settings.osr_p = BME280_OVERSAMPLING_16X;
    bme.settings.osr_t = BME280_OVERSAMPLING_2X;
    bme.settings.filter = BME280_FILTER_COEFF_16;
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL |
                   BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    bmestatus = bme280_set_sensor_settings(settings_sel, &bme);
#ifdef SERIAL_DEBUG
    print_rslt("bme280_set_sensor_settings status", bmestatus);
#endif
    /* The measure time of this project will always be higher than the sensor's
     * minimum delay required between measurements, so the calculation of this
     * delay was removed. -N */
    while (1) {
        /* Won't print the sensor's readings anymore, and now delays the
         * project's measurement interval instead of the minimum time required.
         * -N */
        bmestatus = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme);
        vTaskDelay((TEMPO_ANALISE * 1000) / portTICK_PERIOD_MS);
        bmestatus = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);
        store_data(&comp_data);
    }
}
#endif /* BME280_SENSOR */
