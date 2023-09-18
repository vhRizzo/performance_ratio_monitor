#ifndef GLOBAL_DATA_H
#define GLOBAL_DATA_H

#ifdef __cplusplus
extern "C" {
#endif /* C++ Guard */

/* Geral */
#define TEMPO_ANALISE 30  // Tempo entre cada leitura de dados, em segundos
#define TEMPO_STARTUP 5   // Atraso até que o sistema inicie, em segundos
// Frequência na qual será feita a leitura das coordenadas GPS (uma vez a cada
// [CONTAGEM_GPS] leituras de [TEMPO_ANALISE]).
#define CONTAGEM_GPS 3

/* Comentar/remover os defines irá desabilitar os respectivos processos */
#define DSM501A_SENSOR
#define BME280_SENSOR
#define NEO6M_SENSOR
#define DS18B20_SENSOR
#define ADS1115_SENSOR
#define SERIAL_DEBUG

/* Pinout */
#define I2C_SDA_PIN GPIO_NUM_21  // D21
#define I2C_SCL_PIN GPIO_NUM_22  // D22
#define DSM_P10_PIN GPIO_NUM_34  // D34
#define DSM_P25_PIN GPIO_NUM_35  // D35
#define NEO_UTX_PIN GPIO_NUM_32  // D32
#define NEO_URX_PIN GPIO_NUM_33  // D33
#define LORA_TX_PIN GPIO_NUM_12  // D12
#define LORA_RX_PIN GPIO_NUM_13  // D13
#define DS18B20_PIN GPIO_NUM_27  // D27

/* Portas UART */
#define NEO_UART_PORT UART_NUM_1
#define LOR_UART_PORT UART_NUM_2

/* Porta I2C */
#define I2C_PORT I2C_NUM_0

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp32/rom/ets_sys.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

#if defined(BME280_SENSOR) || defined(ADS1115_SENSOR)
esp_err_t i2c_init();
#endif
void rcv_data_task(void *pvParameters);
void snd_data_task(void *pvParameters);

#ifdef DSM501A_SENSOR
// Struct/item com as variaveis necessarias para armazenar os dados de seu
// respectivo sensor
typedef struct __attribute__((__packed__)) {
    float poeira_pm_10;
    float poeira_pm_25;
} dsm501a_t;

// Handler para a fila de recebimento de itens, marcado como "extern" para que o
// compilador saiba que ele será inicializado por uma fonte externa.
extern QueueHandle_t dsm_queue;
#endif

#ifdef BME280_SENSOR
typedef struct __attribute__((__packed__)) {
    float temperatura;
    float umidade;
    uint32_t pressao;
} bme280_t;

extern QueueHandle_t bme_queue;
#endif

#ifdef NEO6M_SENSOR
typedef struct __attribute__((__packed__)) {
    float coord[2];  //[0] Lat , [1] Lng
} neo6m_t;

extern QueueHandle_t neo_queue;
#endif

#ifdef DS18B20_SENSOR
typedef struct __attribute__((__packed__)) {
    float dallas_temp;
} ds18b20_t;

extern QueueHandle_t ds18b20_queue;
#endif

#ifdef ADS1115_SENSOR
typedef struct __attribute__((__packed__)) {
    float irrad;
} ads_t;

extern QueueHandle_t ads_queue;
#endif

// __attribute__((__packed__)) informa ao compilador para utilizar o mínimo de
// memória para tipos dentro da struct ou da union
typedef struct __attribute__((__packed__)) {
    float temperatura;   // 4 bytes
    float umidade;       // 4 bytes
    uint32_t pressao;    // 2 bytes
    float dallas_temp;   // 4 bytes
    float irrad;         // 4 bytes
    float poeira_pm_10;  // 4 bytes
    float poeira_pm_25;  // 4 bytes
    float coord[2];      // 8 bytes
} dados_t;

#ifdef __cplusplus
}
#endif /* C++ Guard */
#endif /* GLOBAL_DATA_H */
