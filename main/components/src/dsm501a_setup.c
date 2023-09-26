#include "dsm501a_setup.h"
#ifdef DSM501A_SENSOR

QueueHandle_t dsm_queue;  // Handler da queue que os dados serão enviados

/* Para poupar memória, o compilador geralmente verifica se uma variável
 * utilizada como condição de um loop é atualizada dentro do próprio loop. Se
 * ele identificar que ela não é atualizada, ele não irá verificar novamente
 * esta condição nas próximas iterações, por exemplo, no trecho de código:
 *
 * if (j < 30) {
 *     i++;
 * }
 *
 * Como a variável j não é atualizada no loop, o compilador não irá mais
 * verificar o estado desta variável para as demais iterações. De um ponto de
 * vista lógico isso faz sentido, porém em sistemas embarcados, existem ocasiões
 * (como neste projeto) onde uma variável é alterada externamente, como uma
 * interrupção. Portanto, para deixar explícito para o compilador que, por mais
 * que uma variável condicional não seja atualizada dentro do corpo do loop, ela
 * ainda deverá ser verificada a cada iteração, deve-se utilizar a palavra-chave
 * "volatile". */

/* Variáveis relacionadas ao sensor com sensibilidade de 1,0 μm */
// Nível do estado anterior
volatile unsigned int estado_anterior_v2 = 1;
// Duração total dos pulsos
volatile unsigned int pulsos_v2 = 0;
// Instante da borda de descida do pulso
volatile unsigned long marca_falling_v2 = 0;

/* Variaveis relacionadas ao sensor com sensibilidade de 2,5 μm */

volatile unsigned int estado_anterior_v1 = 1;
volatile unsigned int pulsos_v1 = 0;
volatile unsigned long marca_falling_v1 = 0;

/**
 * @brief Callback de interrupção da saída do DSM501A com sensibilidade de 1,0
 * μm. Verifica se é a borda de subida ou de descida do pulso. Se for a borda de
 * descida, armazena o instante da queda. Se for uma borda de subida, armazena a
 * duração na qual o pulso permaneceu em nível baixo (instante da subida -
 * instante da descida).
 *
 * @param arg Pino no qual a saída V2 do DSM501A está conectada.
 */
void IRAM_ATTR change_v2_isr(void *arg) {
    // Estado atual do sensor
    uint32_t gpio_estado = gpio_get_level((gpio_num_t)(uint32_t)arg);
    /* Se o estado anterior for igual ao estado atual, encerra a função. Pode
     * acontecer na situação onde o valor de estado anterior é alterado
     * manualmente durante o cálculo da concentração de poeira. */
    if (gpio_estado == estado_anterior_v2)
        return;
    else if (gpio_estado == 0) {  // Se o estado atual for LOW, borda de descida
        marca_falling_v2 = esp_timer_get_time();  // Armazena o momento da queda
        estado_anterior_v2 = 0;  // E atualiza o estado anterior para o atual
    } else {                     // Se o estado atual for HIGH, borda de subida
        // Atualiza o estado anterior para o estado atual
        estado_anterior_v2 = 1;
        // Armazena a duração que o pulso permaneceu em LOW, de forma
        // cumulativa.
        pulsos_v2 += esp_timer_get_time() - marca_falling_v2;
    }
}

/**
 * @brief Callback de interrupção da saída do DSM501A com sensibilidade
 * ajustável. Verifica se é a borda de subida ou de descida do pulso. Se for a
 * borda de descida, armazena o instante da queda. Se for uma borda de subida,
 * armazena a duração na qual o pulso permaneceu em nível baixo (instante da
 * subida - instante da descida).
 *
 * @param arg Pino no qual a saída V1 do DSM501A está conectada.
 */
void IRAM_ATTR change_v1_isr(void *arg) {
    uint32_t gpio_estado = gpio_get_level((gpio_num_t)(uint32_t)arg);
    if (gpio_estado == estado_anterior_v1)
        return;
    else if (gpio_estado == 0) {
        marca_falling_v1 = esp_timer_get_time();
        estado_anterior_v1 = 0;
    } else {
        estado_anterior_v1 = 1;
        pulsos_v1 += esp_timer_get_time() - marca_falling_v1;
    }
}

void dsm501a_task(void *pvParameters) {
    float ratio_v2 = 0;  // Taxa de tempo que a leitura permaneceu em LOW
    float concentration_v2 = 0;  // Concentração de partículas de poeira
    float ratio_v1 = 0;
    float concentration_v1 = 0;
    dsm501a_t dsm;  // Item que será enviado pela fila
    // Configuração dos pinos de leitura
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,  // Bit Mask dos pinos
        .mode = GPIO_MODE_INPUT,  // Modo de utilização dos pinos (Entrada)
        // Se Pull Up será utilizado (Desabilitado)
        .pull_up_en = GPIO_PULLUP_DISABLE,
        // Se Pull Down sera utilizado (Desabilitado)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        // Se interrupção será utilizada (Habilitada para qualquer borda)
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);  // Aplica as configurações
    // Instala o serviço de interrupção
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // Ativa a interrupção para o pino de 1,0 μm
    gpio_isr_handler_add(DSM_P10_PIN, change_v2_isr, (void *)DSM_P10_PIN);
    // Ativa a interrupção para o pino de 2,5 μm
    gpio_isr_handler_add(DSM_P25_PIN, change_v1_isr, (void *)DSM_P25_PIN);
    while (true) {
        // Atraso entre as leituras
        vTaskDelay((TEMPO_ANALISE * 1000) / portTICK_PERIOD_MS);
        // Desativa as interrupções
        gpio_isr_handler_remove(DSM_P10_PIN);
        gpio_isr_handler_remove(DSM_P25_PIN);
        // Calcula a taxa de duração dos pulsos: (pulsos / tempo_de_leitura)
        ratio_v2 = (float)((pulsos_v2) / (float)(TEMPO_ANALISE * 10000));
        ratio_v1 = (float)((pulsos_v1) / (float)(TEMPO_ANALISE * 10000));
        /* No datasheet do DSM501A existe uma curva quase linear relacionando a
         * taxa com a concentração de partículas (pcs/283ml). Foram extraídos
         * arbritariamente 10 pontos da reta para gerar um gráfico de dispersão
         * em uma planilha e calcular o coeficiente da linha de tendência. O
         * valor encontrado foi de aproximadamente 615.55. */
        // Faz o cálculo da concentração.
        concentration_v2 = 615.55 * ratio_v2;
        concentration_v1 = 615.55 * ratio_v1;
        // Atualiza os dados do item que será enviado pela fila
        dsm.poeira_pm_10 = concentration_v2;
        dsm.poeira_pm_25 = concentration_v1;
        xQueueOverwrite(dsm_queue, &dsm);  // Envia o item
        pulsos_v2 = 0;                     // Zera a variável de acumulação
        estado_anterior_v2 = 1;  // Retorna os estados anteriores para HIGH
        pulsos_v1 = 0;
        estado_anterior_v1 = 1;
        // Reativa as interrupções
        gpio_isr_handler_add(DSM_P10_PIN, change_v2_isr, (void *)DSM_P10_PIN);
        gpio_isr_handler_add(DSM_P25_PIN, change_v1_isr, (void *)DSM_P25_PIN);
    }
}
#endif
