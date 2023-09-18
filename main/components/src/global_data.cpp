#include "global_data.h"

#include "Modem_SmartModular.hpp"

/* Cria um item para armazenar os dados dos sensores inicializando todos os
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
char tmp[100];       // String temporaria para formatar os dados
char rcv[1000];      // String para armazenar a resposta da LoRaWAN
char rcv_gps[1000];  // Outra para a resposta do GPS
/* Aqui deve-se escolher >=1 , sendo útil para indicar qual parte da aplicação
 * pertence o dado obtido!!!! */
uint16_t porta = 20;
uint32_t rcv_size = 0;    // Variavel para armazenar o tamanho da resposta
uint32_t send_size = 0;   // E o tamanho dos dados enviados
uint16_t porta_gps = 19;  // Porta utilizada para as coordenadas GPS
// Porta utilizada para os sensores relacionados ao piranômetro
uint16_t porta_sol = 18;
// Variavel para armazenar o tamanho da resposta ao envio das coordenadas GPS
uint32_t rcv_gps_size = 0;
/* Inicializa o objeto da LoRaWAN utilizando um construtor com a porta UART a
 * ser utilizada pelo ESP */
Modem_SmartModular teste(LOR_UART_PORT);
/* Inicializa o contador de envio de coordenadas GPS em 2/3 do
 * valor total, desta forma as coordenadas nao serao enviadas
 * imediatamente no inicio (pois o modulo GPS provavelmente nao
 * tera sincronizado com o satelite ainda), mas tambem nao
 * espera um ciclo completo para enviar a primeira medida.*/
int cont_gps = (int)floor(CONTAGEM_GPS * (2. / 3.));

// Inicializa os itens que serao recebidos atraves das filas
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
#ifdef BPW34_SENSOR
bpw34_t bpw_rcv;
#endif
#ifdef ADS1115_SENSOR
ads_t ads_rcv;
#endif

#if defined(BME280_SENSOR) || defined(ADS1115_SENSOR)
esp_err_t i2c_init() {
    // Here, the way that i2c_config is initialized was changed to get rid of
    // compiler warnings about uninitialized values
    i2c_config_t i2c_config;
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = I2C_SDA_PIN;
    i2c_config.scl_io_num = I2C_SCL_PIN;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = 100000;
    i2c_param_config(I2C_PORT, &i2c_config);
    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}
#endif

/* Esta tarefa apenas se mantem em um loop recebendo os itens armazenados nas
 * filas */
void rcv_data_task(void *pvParameters) {
    while (1) {
#ifdef DSM501A_SENSOR
        if (xQueueReceive(
                dsm_queue, &dsm_rcv,
                portMAX_DELAY))  // Ao receber um novo item pela fila...
        {
            dados.poeira_pm_10 =
                dsm_rcv.poeira_pm_10;  // Armazena os dados do item recebido no
                                       // item de dados gerais
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
            /* Por vezes, ao tentar ler os valores do GPS antes do modulo ter
             * sincronizado com o satelite, ele enviara valores de lixo. Entao
             * nas condicoes abaixo os valores sao filtrados para apenas
             * atualizar caso a latitude esteja em -90 e 90 e a longitude entre
             * -180 e 180, excluindo a coordenada 0 para ambos. O motivo de
             * excluir o 0 se da por ele ser um dos valores de lixo enviado pelo
             * modulo. Esta coordenada seria valida, mas e muito improvavel que
             * seja necessaria, portanto foi decidido que e melhor desconsiderar
             * este valor. */
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

#ifdef BPW34_SENSOR
        if (xQueueReceive(bpw34_queue, &bpw_rcv, portMAX_DELAY)) {
            dados.irrad = bpw_rcv.irrad;
        }
#endif

#ifdef ADS1115_SENSOR
        if (xQueueReceive(ads_queue, &ads_rcv, portMAX_DELAY)) {
            dados.irrad = ads_rcv.irrad;
        }
#endif
    }
}

/* Esta tarefa inicializa o modulo LoRa, e envia os dados recebidos para uma
 * antena LoRa */
void snd_data_task(void *pvParameters) {
    memset(rcv, 0,
           1000);  // Inicializa a string de resposta dos dados gerais da LoRa
    memset(rcv_gps, 0, 1000);  // E a de dados do gps

    teste.OTAA_DEUI();  // Exibe o endereco MAC do modulo
#ifdef SERIAL_DEBUG
    ESP_LOGI(__func__, "%s",
             teste.OTAA_APPKEY());  // Exibe a chave de validacao do modulo
#endif

    if (strcmp(teste.auto_join(), "0") ==
        0) {  // Verifica se o autojoin esta ativado, se nao estiver, o ativa
#ifdef SERIAL_DEBUG
        ESP_LOGI("APP", "AutoJoin ativado: %s", teste.auto_join(1));
#endif
    } else {
#ifdef SERIAL_DEBUG
        ESP_LOGI("APP", "AutoJoin já ativado...");
#endif
    }

    vTaskDelay((TEMPO_STARTUP * 1000) /
               portTICK_PERIOD_MS);  // Delay necessario para que alguns
                                     // sensores estejam em seu estado estavel
    while (1) {
        vTaskDelay((TEMPO_ANALISE * 1000) /
                   portTICK_PERIOD_MS);  // Delay para cada leitura
        char tmp[100];  // String auxiliar para formatar os dados que serao
                        // enviados para a antena LoRa
        sprintf(tmp, "%.2f,%.2f,%d;%.2f,%.2f", dados.temperatura, dados.umidade,
                (int)dados.pressao, dados.poeira_pm_10,
                dados.poeira_pm_25);  // String que contera as informacoes
                                      // relacionadas ao ambiente
        send_size = strlen(tmp);      // Armazena o tamanho da string
#ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "ENVIOU = DADOS: %s | TAM: %i | PORTA: %d", tmp,
                 (int)send_size, porta);
#endif
        teste.enviar_receber(
            porta, tmp, send_size, rcv,
            &rcv_size);  // Envia os dados na porta dos dados ambientais
#ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "RECEBEU = DADOS: %s | TAM: %i", rcv, (int)rcv_size);
#endif

        sprintf(tmp, "%.2f,%.2f", dados.dallas_temp,
                dados.irrad);  // String que contera as informacoes relacionadas
                               // ao piranometro
        send_size = strlen(tmp);  // Armazena o tamanho da string
#ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "ENVIOU = DADOS: %s | TAM: %i | PORTA: %d", tmp,
                 (int)send_size, porta_sol);
#endif
        teste.enviar_receber(
            porta_sol, tmp, send_size, rcv,
            &rcv_size);  // Envia os dados na porta dos dados do piranometro
#ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "RECEBEU = DADOS: %s | TAM: %i", rcv, (int)rcv_size);
#endif

#ifdef NEO6M_SENSOR
        cont_gps++;  // Incrementa o contados do gps
        if (cont_gps >=
            CONTAGEM_GPS) {     // Se o contador for maior que a quantidade
                                // de contagem definida no header
            char tmp_gps[100];  // Cria uma string propria para armazenar os
                                // dados do gps
            sprintf(tmp_gps, "%.6f,%.6f", dados.coord[0],
                    dados.coord[1]);  // Formata as coordenadas com precisao de
                                      // 6 casas decimais
            send_size = strlen(tmp_gps);  // Armazena o tamanho da string
#ifdef SERIAL_DEBUG
            ESP_LOGI(__func__, "ENVIOU = DADOS: %s | TAM: %i | PORTA: %d",
                     tmp_gps, (int)send_size, porta_gps);
#endif
            teste.enviar_receber(
                porta_gps, tmp_gps, send_size, rcv_gps,
                &rcv_gps_size);  // Envia os dados na porta relacionada ao gps
#ifdef SERIAL_DEBUG
            ESP_LOGI(__func__, "RECEBEU = DADOS: %s | TAM: %i", rcv_gps,
                     (int)rcv_gps_size);
#endif
            cont_gps = 0;  // Reinicia o contador do gps
        }
#endif
    }
}
