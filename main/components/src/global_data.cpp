#include "global_data.h"

#include "Modem_SmartModular.hpp"

/* Cria um item para armazenar os dados dos sensores, inicializando todos os
 * dados em -1 */
dados_t dados = {
    .temperatura = -1,
    .umidade = -1,
    .pressao = (uint32_t)-1,
    .dallas_temp = -1,
    .irrad = -1,
    .poeira_pm_10 = -1,
    .poeira_pm_25 = -1,
    .coord = {-1, -1},
};
char rcv[1000];  // String para armazenar a resposta da LoRaWAN
/* Aqui deve-se escolher >=1 , sendo útil para indicar qual parte da aplicação
 * pertence o dado obtido!!!! */
uint32_t rcv_size = 0;   // Tamanho dos dados recebidos
uint32_t send_size = 0;  // Tamanho dos dados enviados
/* Inicializa o objeto da LoRaWAN utilizando um construtor com a porta UART a
 * ser utilizada pelo ESP */
Modem_SmartModular teste(LOR_UART_PORT);
/* Inicializa o contador de envio de coordenadas GPS em 2 / 3 do valor total,
 * desta forma as coordenadas não serão enviadas imediatamente no início (pois o
 * módulo GPS provavelmente não terá sincronizado com o satélite ainda), mas
 * também não espera um ciclo completo para enviar a primeira medida. */
// Contador de leituras para enviar as coordenadas GPS
int cont_gps = (int)floor(CONTAGEM_GPS * (2. / 3.));

// Declara os itens que serão recebidos através das filas
#ifdef DSM501A_SENSOR
dsm501a_t dsm_rcv;
#endif
#ifdef BME280_SENSOR
bme280_t bme_rcv;
#endif
#ifdef NEO6M_SENSOR
neo6m_t neo_rcv;
#endif
#ifdef DS18B20_SENSOR
ds18b20_t dallas_rcv;
#endif
#ifdef ADS1115_SENSOR
ads_t ads_rcv;
#endif

#if defined(BME280_SENSOR) || defined(ADS1115_SENSOR)
esp_err_t i2c_init() {
    i2c_config_t i2c_config;
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = I2C_SDA_PIN;
    i2c_config.scl_io_num = I2C_SCL_PIN;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = 100000;
    i2c_param_config(I2C_PORT, &i2c_config);
    return i2c_driver_install(I2C_PORT, i2c_config.mode, 0, 0, 0);
}
#endif

void rcv_data_task(void *pvParameters) {
    while (1) {
#ifdef DSM501A_SENSOR
        // Se receber um novo item pela fila
        if (xQueueReceive(dsm_queue, &dsm_rcv, portMAX_DELAY)) {
            // Armazena os dados recebidos
            dados.poeira_pm_10 = dsm_rcv.poeira_pm_10;
            dados.poeira_pm_25 = dsm_rcv.poeira_pm_25;
        }
#endif
#ifdef BME280_SENSOR
        if (xQueueReceive(bme_queue, &bme_rcv, portMAX_DELAY)) {
            dados.temperatura = bme_rcv.temperatura;
            dados.umidade = bme_rcv.umidade;
            dados.pressao = bme_rcv.pressao;
        }
#endif
#ifdef NEO6M_SENSOR
        if (xQueueReceive(neo_queue, &neo_rcv, portMAX_DELAY)) {
            /* Por vezes, ao tentar ler os valores do GPS antes do módulo ter
             * sincronizado com o satélite, ele enviará valores de lixo. Então
             * nas condições abaixo, os valores são filtrados para apenas
             * atualizar caso a latitude esteja entre -90 e 90 e a longitude
             * entre -180 e 180, excluindo a coordenada 0 para ambos devido a
             * também ser enviado como lixo pelo módulo, por mais que esta seja
             * uma coordenada válida. */
            if (fabs(neo_rcv.coord[0]) > 0 && fabs(neo_rcv.coord[0]) <= 90)
                dados.coord[0] = neo_rcv.coord[0];
            if (fabs(neo_rcv.coord[1]) > 0 && fabs(neo_rcv.coord[1]) <= 180)
                dados.coord[1] = neo_rcv.coord[1];
        }
#endif
#ifdef DS18B20_SENSOR
        if (xQueueReceive(ds18b20_queue, &dallas_rcv, portMAX_DELAY)) {
            dados.dallas_temp = dallas_rcv.dallas_temp;
        }
#endif
#ifdef ADS1115_SENSOR
        if (xQueueReceive(ads_queue, &ads_rcv, portMAX_DELAY)) {
            dados.irrad = ads_rcv.irrad;
        }
#endif
    }
}

void snd_data_task(void *pvParameters) {
    // Inicializa a string de resposta dos dados gerais da LoRa
    memset(rcv, 0, 1000);
    teste.OTAA_DEUI();  // Exibe o endereço MAC do módulo LoRa
#ifdef SERIAL_DEBUG
    // Exibe a chave de validação do módulo
    ESP_LOGI(__func__, "%s", teste.OTAA_APPKEY());
#endif
    // Verifica se o autojoin está ativado, se não estiver, o ativa
    if (strcmp(teste.auto_join(), "0") == 0) {
        // Esse log não será desativado fora de debug por acionar o auto_join
        ESP_LOGI("APP", "AutoJoin ativado: %s", teste.auto_join(1));
    } else {
#ifdef SERIAL_DEBUG
        ESP_LOGI("APP", "AutoJoin já ativado...");
#endif
    }
    // Atraso necessário para que alguns sensores fiquem estáveis
    vTaskDelay((TEMPO_STARTUP * 1000) / portTICK_PERIOD_MS);
    while (1) {
        // Atraso entre leituras
        vTaskDelay((TEMPO_ANALISE * 1000) / portTICK_PERIOD_MS);
        // String temporária para formatar os dados
        char tmp[100];
        // Gera a string com os dados do módulo fotovoltaico
        sprintf(tmp, "%.2f,%.2f;%.2f;%.3f", dados.poeira_pm_10,
                dados.poeira_pm_25, dados.dallas_temp, dados.irrad);
        send_size = strlen(tmp);  // Armazena o tamanho da string
#ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "ENVIOU = DADOS: %s | TAM: %i | PORTA: %d", tmp,
                 (int)send_size, PORTA_SOL);
#endif
        // Envia os dados na porta dos dados do piranômetro
        teste.enviar_receber(PORTA_SOL, tmp, send_size, rcv, &rcv_size);
#ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "RECEBEU = DADOS: %s | TAM: %i", rcv, (int)rcv_size);
#endif
        // Gera a string com os dados ambientais
        sprintf(tmp, "%.2f,%.2f,%d", dados.temperatura, dados.umidade,
                (int)dados.pressao);
        send_size = strlen(tmp);  // Armazena o tamanho da string
#ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "ENVIOU = DADOS: %s | TAM: %i | PORTA: %d", tmp,
                 (int)send_size, PORTA_AMB);
#endif
        // Envia os dados na porta de dados ambientais
        teste.enviar_receber(PORTA_AMB, tmp, send_size, rcv, &rcv_size);
#ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "RECEBEU = DADOS: %s | TAM: %i", rcv, (int)rcv_size);
#endif
#ifdef NEO6M_SENSOR
        cont_gps++;  // Incrementa o contados do gps
        // Se o contador for maior que a quantidade de contagem definida no
        // header
        if (cont_gps >= CONTAGEM_GPS) {
            // Gera a string com as coordenadas GPS
            sprintf(tmp, "%.6f,%.6f", dados.coord[0], dados.coord[1]);
            send_size = strlen(tmp);  // Armazena o tamanho da string
#ifdef SERIAL_DEBUG
            ESP_LOGI(__func__, "ENVIOU = DADOS: %s | TAM: %i | PORTA: %d", tmp,
                     (int)send_size, PORTA_GPS);
#endif
            // Envia os dados na porta do gps
            teste.enviar_receber(PORTA_GPS, tmp, send_size, rcv, &rcv_size);
#ifdef SERIAL_DEBUG
            ESP_LOGI(__func__, "RECEBEU = DADOS: %s | TAM: %i", rcv,
                     (int)rcv_size);
#endif
            cont_gps = 0;  // Reinicia o contador do gps
        }
#endif
    }
}
