extern "C" {  // Converte o código .cpp para .c

#include "Modem_SmartModular.hpp"
#include "global_data.h"

#ifdef DSM501A_SENSOR
#include "dsm501a_setup.h"
#endif
#ifdef BME280_SENSOR
#include "bme280_setup.h"
#endif
#ifdef NEO6M_SENSOR
#include "neo6m_setup.h"
#endif
#ifdef DS18B20_SENSOR
#include "ds18b20_setup.h"
#endif
#ifdef ADS1115_SENSOR
#include "ads1115.h"
#endif

void app_main(void);
}

void app_main(void) {
    // Verifica por problemas na memória flash do ESP
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // Sinaliza que um pino do ESP será utilizado (pino 2 é o pino padrão do LED
    // do dev-kit ESP32S)
    esp_rom_gpio_pad_select_gpio(GPIO_NUM_2);
    // Inicializa o pino como output
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);  // Apaga o LED (define o sinal para baixo)
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Aguarda 1 segundo
    gpio_set_level(GPIO_NUM_2, 1);  // Acende o LED (define o sinal para alto)
    /* A função do LED é ser um controle de funcionamento. O LED deverá
     * permanecer aceso durante todo o funcionamento do dispositivo, se o LED
     * estiver apagado ou piscando, é sinal de que há problemas no sistema. */

    // Interrompe o uso do WiFi (não será utilizado neste projeto)
    esp_wifi_stop();

    /* O trecho de código comentado abaixo deve ser executado apenas quando for
     * necessário utilizar um novo módulo LoRa para configurá-lo e validá-lo.
     *
     * @warning EXECUTAR ESTE TRECHO DE CÓDIGO APENAS UMA VEZ PARA CONFIGURAR UM
     * NOVO MÓDULO, APÓS CONFIGURADO, COMENTE O CÓDIGO NOVAMENTE. */
    // Modem_SmartModular teste(UART_NUM_2);
    // teste.OTAA_APPEUI("00:00:00:00:00:00:00:00");
    // teste.OTAA_APPKEY("8C:17:01:D4:15:AB:D5:FB:09:58:CB:35:3E:F6:60:C1");
    // teste.salvar_config();

#if defined(BME280_SENSOR) || defined(ADS1115_SENSOR)
    // Se algum dos sensores I2C estiverem sendo usados, initializa o I2C
    ESP_ERROR_CHECK(i2c_init());
#endif

#ifdef ADS1115_SENSOR
    // Cria uma fila para enviar itens de uma tarefa para outra
    ads_queue = xQueueCreate(1, sizeof(ads_t));
    /* Cria a tarefa do conversor analógico-digital.
     *
     * Tarefas são funções que geralmente se mantém em um loop infinito para
     * realizar determinados procedimentos, semelhante ao loop do Arduino, porém
     * possui a capacidade de executar várias tarefas simultaneamente. */
    xTaskCreatePinnedToCore(ads1115_task, "ADS1115_Task", 4 * 1024, NULL,
                            configMAX_PRIORITIES - 3, NULL, 1);
#endif

    /* Daqui em diante apenas repete o mesmo processo acima para os demais
     * sensores ou protocolos necessários. */
#ifdef DS18B20_SENSOR
    ds18b20_queue = xQueueCreate(1, sizeof(ds18b20_t));
    xTaskCreatePinnedToCore(ds18b20_task, "DS18B20_Task", 4 * 1024, NULL,
                            configMAX_PRIORITIES - 4, NULL, 0);
#endif

#ifdef BME280_SENSOR
    bme_queue = xQueueCreate(1, sizeof(bme280_t));
    xTaskCreatePinnedToCore(bme280_task, "BME280_Task", 4 * 1024, NULL,
                            configMAX_PRIORITIES - 5, NULL, 0);
#endif

#ifdef DSM501A_SENSOR
    dsm_queue = xQueueCreate(1, sizeof(dsm501a_t));
    xTaskCreatePinnedToCore(dsm501a_task, "DSM501a_Task", 4 * 1024, NULL,
                            configMAX_PRIORITIES - 6, NULL, 0);
#endif

#ifdef NEO6M_SENSOR
    neo_queue = xQueueCreate(1, sizeof(neo6m_t));
    xTaskCreatePinnedToCore(neo6m_task, "NEO-6M_Task", 4 * 1024, NULL,
                            configMAX_PRIORITIES - 7, NULL, 0);
#endif
    /* As tarefas abaixo são de leitura e envio de dados, estas tarefas usarão
     * as filas geradas acima para receber os itens para então enviá-los via
     * LoRaWAN, portanto é feito um atraso aqui para garantir que as filas já
     * estejam todas operantes, pois caso estas tarefas tentem ler um item de
     * uma fila que ainda não existe, um erro fatal será gerado, reiniciando o
     * ESP. */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(rcv_data_task, "Data_Receive_Task", 4 * 1024, NULL,
                            configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(snd_data_task, "Data_Send_Task", 4 * 1024, NULL,
                            configMAX_PRIORITIES - 1, NULL, 1);
}
