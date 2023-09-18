extern "C" { /* Converte o codigo .cpp para .c */

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
    /* Verifica por problemas na memória flash do ESP */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // Sinaliza que um pino do ESP será utilizado (pino 2 e o pino padrao do LED
    // do dev-kit ESP32S)
    esp_rom_gpio_pad_select_gpio(GPIO_NUM_2);
    // Inicializa este como output
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);  // Apaga o LED (define o sinal para baixo)
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Aguarda 1 segundo
    gpio_set_level(GPIO_NUM_2, 1);  // Acende o LED (define o sinal para alto)
    /* O LED serve para indicar que o ESP esta funcionando corretamente, O LED
     * permanecera aceso, e se piscar e sinal de que o ESP reiniciou */

    esp_wifi_stop();  // Interrompe o uso do WiFi (não será utilizado neste
                      // projeto)

    // ***** Fazer apenas uma vez!!!! (Alterar as configurações da antena LoRa)
    // Modem_SmartModular teste(UART_NUM_2);
    // teste.OTAA_APPEUI("00:00:00:00:00:00:00:00");
    // teste.OTAA_APPKEY("8C:17:01:D4:15:AB:D5:FB:09:58:CB:35:3E:F6:60:C1");
    // teste.salvar_config();
    // ***** As configs ficarão salvas para o auto_join ser ativado!

#if defined(BME280_SENSOR) || defined(ADS1115_SENSOR)
    ESP_ERROR_CHECK(i2c_init());  // se algum dos sensores I2C estiverem sendo
                                  // usados, initializa o I2C
#endif

#ifdef ADS1115_SENSOR
    // Cria uma fila para enviar itens de uma tarefa para outra
    ads_queue = xQueueCreate(1, sizeof(ads_t));
    /*
     * Cria a tarefa do conversor analógico-digital.
     * Tarefas sao funções que geralmente se mantem em um loop infinito para
     * realizar determinadas tarefas. Semelhante ao loop do Arduino, com o
     * diferencial de que varias tarefas podem ser executadas simultaneamente.
     */
    xTaskCreatePinnedToCore(
        ads1115_task,    // Função da tarefa
        "ADS1115_Task",  // Nome para controle de depuração
        4 * 1024,  // Tamanho da pilha de execução que será alocado para a
                   // tarefa
        NULL,      // Parâmetros da tarefa (geralmente não utilizado)
        configMAX_PRIORITIES -
            3,  // Prioridade da tarefa (maior valor terá maior prioridade)
        NULL,  // Handler para esta tarefa caso seja necessário que outra tarefa
               // ou função se comunique/controle esta
        1);  // Nucleo do ESP que será utilizado para executar a tarefa
#endif

    /* Daqui em diante apenas repete o mesmo processo acima para os demais
     * sensores ou protocolos necessários */

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
    /*
     * As tarefas abaixo são de leitura e envio de dados, estas tarefas usarão
     * as filas geradas acima para receber os itens para então enviá-los via
     * LoRaWAN, portanto é feito um atraso aqui para garantir que as filas já
     * estejam todas operantes, pois caso estas tarefas tentem ler um item de
     * uma fila que ainda não existe, um erro fatal será gerado, reiniciando o
     * ESP.
     */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(rcv_data_task, "Data_Receive_Task", 4 * 1024, NULL,
                            configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(snd_data_task, "Data_Send_Task", 4 * 1024, NULL,
                            configMAX_PRIORITIES - 1, NULL, 1);
}
