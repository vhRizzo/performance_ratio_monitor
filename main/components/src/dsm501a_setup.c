#include "global_data.h"

#ifdef DSM501A_SENSOR

#include "dsm501a_setup.h"

QueueHandle_t dsm_queue;    // Handler da queue que os dados serao enviados

/* Um compilador para poupar memoria ira checar o valor de certa variavel em uma condicao apenas uma vez enquanto ele
 * nao ver que esta variavel foi atualizada dentro deste loop. Para exemplificar, imagine que voce tem em algum lugar
 * um codigo com algo do tipo:
 * if (alguma_condicao) {
 *     i++;
 * }
 *
 * E em outro lugar, outro codigo do tipo:
 * int i = 0;
 * while (i < 30) {
 *     printf("algo_para_exibir_na_tela");
 * }
 *
 * Mesmo que ambos os processos estejam sendo executado simultaneamente, o compilador percebe que a variavel 'i' nao
 * se atualiza dentro do loop while, portanto nao verifica mais a condicao e apenas prende o processo em um loop infinito.
 * Ao indicar para o compilador que a variavel e 'volatile', ele entende que ela pode ser atualizada por algum outro processo
 * qualquer, entao ele ira verificar a condicao do while a cada loop. */
/* Variaveis relacionadas ao sensor com sensibilidade de 1,0 μm */
volatile unsigned int estado_anterior_v2 = 1;   // Armazenar o estado anterior de leitura do sensor, inicializado em HIGH
volatile unsigned int pulsos_v2 = 0;            // Armazenar a duracao total dos pulsos do sensor
volatile unsigned long marca_falling_v2 = 0;    // Armazenar o momento da borda de descida do pulso

/* Variaveis relacionadas ao sensor com sensibilidade de 2,5 μm */
volatile unsigned int estado_anterior_v1 = 1;
volatile unsigned int pulsos_v1 = 0;
volatile unsigned long marca_falling_v1 = 0;

/* Funcao de controle de interrupcao (1,0 μm) */
void IRAM_ATTR change_v2_isr ( void *arg )
{
    uint32_t gpio_estado = gpio_get_level((gpio_num_t)(uint32_t) arg);  // Le o estado atual do pino de leitura
    if(gpio_estado == estado_anterior_v2) return;   // Se o estado anterior for igual ao estado atual, sai da funcao.
                                                    // Pode acontecer em momentos de transicao, como enquanto a media
                                                    // dos pulsos estiver sendo calculada.
    else if (gpio_estado == 0) {                    // Se o estado atual for LOW, borda de descida
        marca_falling_v2 = esp_timer_get_time();    // Armazena o momento da queda
        estado_anterior_v2 = 0;                     // E atualiza o estado anterior para o atual
    } else {                                        // Se o estado atual for HIGH, borda de subida
        estado_anterior_v2 = 1;                     // Atualiza o estado anterior para o estado atual
        pulsos_v2 += esp_timer_get_time() - marca_falling_v2; // Armazena a duracao que o pulso permaneceu em LOW,
                                                              // de forma cumulativa.
    }
}

/* Funcao de controle de interrupcao (2,5 μm) */
void IRAM_ATTR change_v1_isr ( void *arg )
{
    uint32_t gpio_estado = gpio_get_level((gpio_num_t)(uint32_t) arg);
    if(gpio_estado == estado_anterior_v1) return;
    else if (gpio_estado == 0) {
        marca_falling_v1 = esp_timer_get_time();
        estado_anterior_v1 = 0;
    } else {
        estado_anterior_v1 = 1;
        pulsos_v1 += esp_timer_get_time() - marca_falling_v1;
    }
}

void dsm501a_task( void *pvParameters )
{
    float ratio_v2 = 0;         // Taxa de tempo que a leitura permaneceu em LOW
    float concentration_v2 = 0; // Concentracao de particulas de poeira
    float ratio_v1 = 0;
    float concentration_v1 = 0;
    dsm501a_t dsm;              // Item que sera enviado para a fila

    gpio_config_t io_conf = {   // Configuracao dos pinos de leitura
        .pin_bit_mask = GPIO_INPUT_PIN_SEL, // Bit Mask dos pinos
        .mode = GPIO_MODE_INPUT,            // Modo de utilizacao dos pinos (Entrada)
        .pull_up_en = GPIO_PULLUP_DISABLE,  // Se Pull Up sera utilizado (Desabilitado)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Se Pull Down sera utilizado (Desabilitado)
        .intr_type = GPIO_INTR_ANYEDGE,     // Se interrupcao sera utilizado (Habilitado para qualquer borda)
    };
    gpio_config(&io_conf);      // Aplica as configuracoes
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);    // Intala o servico de interrupcao

    gpio_isr_handler_add(DSM_P10_PIN, change_v2_isr, (void*) DSM_P10_PIN);  // Ativa a interrupcao para o pino de 1,0 μm
    gpio_isr_handler_add(DSM_P25_PIN, change_v1_isr, (void*) DSM_P25_PIN);  // Ativa a interrupcao para o pino de 2,5 μm

    while(true)
    {
        vTaskDelay( (TEMPO_ANALISE*1000)/portTICK_PERIOD_MS );  // Delay entre cada leitura

        /* Desativa as interrupcoes */
        gpio_isr_handler_remove(DSM_P10_PIN);
        gpio_isr_handler_remove(DSM_P25_PIN);

        unsigned long pulsos_acumulados_v2 = pulsos_v2; // Armazena os pulsos acumulados em uma nova variavel
        pulsos_v2 = 0;                                  // Zera a variavel de acumulacao
        estado_anterior_v2 = 1;                         // Retorna o estado anterior para HIGH

        unsigned long pulsos_acumulados_v1 = pulsos_v1;
        pulsos_v1 = 0;
        estado_anterior_v1 = 1;

        /* Calcula a taxa de duracao dos pulsos: (pulsos/tempo_de_leitura) */
        ratio_v2 = (float)( (pulsos_acumulados_v2) / (float)(TEMPO_ANALISE * 10000) );
        ratio_v1 = (float)( (pulsos_acumulados_v1) / (float)(TEMPO_ANALISE * 10000) );

        /* No datasheet do DSM501A existe uma curva quase linear relacionando a taxa com a concentracao de particulas (pcs/283ml)
         * Os valores da curva media onde os valores permanecem lineares foram utilizados para calcular o coeficiente desta reta.
         * O valor encontrado foi de aproximadamente 615.55 */
        concentration_v2 = 615.55 * ratio_v2;
        concentration_v1 = 615.55 * ratio_v1;

        dsm.poeira_pm_10 = concentration_v2;    // Atualiza os dados do item que sera enviado para a fila
        dsm.poeira_pm_25 = concentration_v1;
        xQueueOverwrite(dsm_queue, &dsm);       // E envia o item para a fila

        /* Reativa a interrupcao dos pinos */
        gpio_isr_handler_add(DSM_P10_PIN, change_v2_isr, (void*) DSM_P10_PIN);
        gpio_isr_handler_add(DSM_P25_PIN, change_v1_isr, (void*) DSM_P25_PIN);
    }
}
#endif
