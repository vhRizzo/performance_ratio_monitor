/**
 * @file Modem_Smartmodular.cpp
 * @author Fernando Barreto (fbarreto@utfpr.edu.br)
 * @brief Classe que irá interfacear com o modem SmartModular utilizando UART 
 * @version 0.1
 * @date 2022-06-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "Modem_SmartModular.hpp"


/**
 * @brief Construct a new Modem_SmartModular::Modem_SmartModular object * 
 * Valores padrão dos parâmetros estão na definição da Classe Modem_SmartModular
 * 
 * @param uart_num Porta UART: Padrão UART_NUM_2
 * @param pin_tx_uart Pino TX do ESP32 p/ enviar ao Modem: Padrão #define TX_SERIAL_AT_PIN (GPIO_NUM_14)
 * @param pin_rx_uart Pino RX do ESP32 p/ receber do Modem: Padrão #define RX_SERIAL_AT_PIN (GPIO_NUM_13)
 * @param baud_rate Taxa sinalização Modem: Padrão 9600 bps (conforme datasheet)
 * @param data_bits Bits de dados por ciclo: Padrão UART_DATA_8_BITS (conforme datasheet)
 * @param parity Padrão: UART_PARITY_DISABLE (conforme datasheet)
 * @param stop_bits Padrão: UART_STOP_BITS_1 (conforme datasheet)
 * @param flow_control Padrão: UART_HW_FLOWCTRL_DISABLE (conforme datasheet)
 * @param tam_buffer Tamanho de memória para RX e TX. Padrão: 2048 bytes
 */
Modem_SmartModular::Modem_SmartModular(uart_port_t uart_num, uint8_t pin_tx_uart, uint8_t pin_rx_uart, uint32_t baud_rate, uint8_t data_bits, uint8_t parity, uint8_t stop_bits, uint8_t flow_control, uint32_t tam_buffer){
    
    uart_tx_rx = uart_num;
    tam_tx = tam_buffer; //tam_buffer é quantidade de bytes
    tx_buf = ( char *) malloc(tam_tx);
    memset(tx_buf,0,tam_tx);
    tam_rx = tam_buffer;
    rx_buf = ( char *) malloc(tam_rx);
    memset(rx_buf,0,tam_rx);
    resposta_AT = xQueueCreate( 5, tam_rx ); 

    uart_config_t uart_config = {
        .baud_rate = (int) baud_rate,
        .data_bits = (uart_word_length_t) data_bits,
        .parity = (uart_parity_t )parity,
        .stop_bits = (uart_stop_bits_t) stop_bits,
        .flow_ctrl = (uart_hw_flowcontrol_t) flow_control,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_tx_rx, tam_rx, tam_tx, 0, NULL, 0)); //ira automaticamente bloquear um TX até terminar a transmissão (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html#_CPPv419uart_driver_install11uart_port_tiiiP13QueueHandle_ti)
    ESP_ERROR_CHECK(uart_param_config(uart_tx_rx, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_tx_rx, pin_tx_uart, pin_rx_uart, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));    
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__, "UART: %u,%u,%u,%lu,%u,%u,%u,%u,%lu",uart_num,pin_tx_uart,pin_rx_uart,baud_rate,data_bits,parity,stop_bits,flow_control,tam_buffer);
    #endif

    //xTaskCreate(&process_wrapper, "rx_task", 2048, NULL, configMAX_PRIORITIES-5, &xHandle);(void*)&this
    xTaskCreate(&process_wrapper, "rx_task", 2048, (void *) this, configMAX_PRIORITIES-5, &xHandle);
}

//Modem_SmartModular::Modem_SmartModular(){
//}

/**
 * @brief Destroy the Modem_SmartModular::Modem_SmartModular object
 * 
 */
Modem_SmartModular::~Modem_SmartModular(){
    vTaskDelete( xHandle ); //fazer com task_handle a ajustar no xTaskCreate...
    uart_driver_delete(uart_tx_rx);
    vQueueDelete(resposta_AT);
    free(tx_buf);
    free(rx_buf);
}

/**
 * @brief Método genérico para DEUI, APPEUI e APPKEY
 * 
 * @param comando contém qual o comando a ser enviado para o Modem via AT
 * @return char* contém qual o valor do DEUI ou APPEUI ou APPKEY conforme comando enviado
 */
char *Modem_SmartModular::OTAA_enviar_comando(char *comando){
    uint8_t contar_erros=0;    
    int comando_ok=0;
    char *token=NULL;
    
    #ifdef SERIAL_DEBUG
    //ESP_LOGI(__func__, "Comando generico...");
    #endif
    while(comando_ok==0){
        uart_write_bytes(uart_tx_rx, comando, strlen(comando)); //reiniciar a MCU do modem para enviar comando de novo        
        #ifdef SERIAL_DEBUG
        ESP_LOGI(__func__,"Comando enviado! Esperando modem...");
        #endif
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        do{
            while(xQueueReceive( resposta_AT, ( void * ) rx_buf, ( TickType_t ) 100)!=pdTRUE){}
            #ifdef SERIAL_DEBUG
            ESP_LOGI(__func__,"Resposta...:%s", rx_buf);        
            #endif
        
            if(strstr(rx_buf, "OK")!=NULL){
                #ifdef SERIAL_DEBUG
                //ESP_LOGI(__func__, "COMANDO EXECUTADO COM SUCESSO...");
                #endif
                comando_ok=1;
            }
            else{
                contar_erros++;
                if(contar_erros>20)
                    esp_restart();
            }
        }while(uxQueueMessagesWaiting(resposta_AT)>0);

        if(comando_ok==1){
            xQueueReset(resposta_AT);
            token = strtok(rx_buf, "\r\n");
            return token; //retorna a resposta do comando (o próximo token seria o OK...)
        }
    }
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__, "Retornando null ???");
    #endif
    return NULL;
}
/**
 * @brief Enviar comando de AJOIN para realizar JOIN automaticamente após a inicialização do módulo 
 * Porém, é necessário ter definido as APPEUI (método OTAA_APPEUI) e APPKEY (método OTAA_APPKEY) e
 * depois salvá-las com com método save_config
 * @param valor -1 (default sem parâmetro retorna se está ou não com AJOIN atividado), 0 desativa e 1 ativa
 * @return char * Retorna o status do comando ou NULL se parâmetro incorreto
 */


char *Modem_SmartModular::auto_join(int8_t valor){
    char comando_consulta[13] = "AT+AJOIN=?\r\n";
    char comando_auto[13] = "AT+AJOIN=1\r\n";
    char comando_naoauto[13] = "AT+AJOIN=0\r\n";
    
    if(valor==-1){
        #ifdef SERIAL_DEBUG
        ESP_LOGI(__func__,"Comando AT+AJOIN=?...");
        #endif
        return OTAA_enviar_comando(comando_consulta);        
    }
    if (valor==1){        
        #ifdef SERIAL_DEBUG
        ESP_LOGI(__func__,"Comando AT+AJOIN=1...");
        #endif
        return OTAA_enviar_comando(comando_auto);
    }
    if (valor==0){
        #ifdef SERIAL_DEBUG
        ESP_LOGI(__func__,"Comando AT+AJOIN=0...");
        #endif
        return OTAA_enviar_comando(comando_naoauto);
    }
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__, "Parâmetro incorreto, deve ser 1 para ligar o auto join, ou 0 para desabilitar.");
    #endif
    return NULL;
}

/**
 * @brief Salva as configurações feitas para que possam ser usadas após eventual reinicialização
 * 
  * @return char* Retorna o status do comando
 */
char *Modem_SmartModular::salvar_config(){    
    char comando_salvar[13] = "AT+SAVE\r\n";
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__,"Comando AT+SAVE...");
    #endif
    return OTAA_enviar_comando(comando_salvar);
}

/**
 * @brief Obter o DEVEUI utilizado no OTAA. Não é alterável.
 * 
 * @return char* Retorna o DEVEUI gravado na placa
 */
char *Modem_SmartModular::OTAA_DEUI(){ //vetor contendo 8 bytes de DEVEUI separados por ':' e terminado por '\n'. Tamanho do vetor: 16 bytes
    char comando_listar[12] = "AT+DEUI=?\r\n";
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__,"Comando AT+DEUI...");
    #endif
    return OTAA_enviar_comando(comando_listar); 
}

/**
 * @brief Método para alterar ou obter a APPEUI da ativação OTAA
 * 
 * @param APPEUI string contendo 8 bytes do APPEUI separados por ':' , se for NULL, o método retorna o APPEUI gravado na placa do radio
 * @return char* retorna a APPEUI gravado na placa do radio
 */
char *Modem_SmartModular::OTAA_APPEUI(char *APPEUI){ //vetor contendo 8 bytes em 00 de APPEUI separados por ':' e terminado por '\n' (padrão ChirpStack). Tamanho do vetor: 16 bytes
    char comando_listar[14] = "AT+APPEUI=?\r\n";
    char comando_appeui[37] = "AT+APPEUI=00:00:00:00:00:00:00:00\r\n"; //padrão chirpstack...
    //para analise relacionado ao sprintf abaixo| "AT+APPEUI=00:00:00:00:00:00:00:00\r\n";
    //adotar padrão chirpstack.... mas se precisar....     
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__,"Comando AT+APPEUI...");
    #endif
    if (APPEUI==NULL){
        return OTAA_enviar_comando(comando_listar);
    }
    else{
        if(strlen(APPEUI)<=23){
            sprintf(comando_appeui,"%s%s\r\n","AT+APPEUI=",APPEUI);
            return OTAA_enviar_comando(comando_appeui);
        }
        else{
            #ifdef SERIAL_DEBUG
            ESP_LOGI(__func__,"APPEUI além 23 bytes de \"00:00:00:00:00:00:00:00\" , tamanho: %i...", strlen(APPEUI));
            #endif
            return NULL;
        }
    }
    //return NULL;
}

/**
 * @brief Método para alterar ou obter a APPKEY da ativação OTAA
 * 
 * @param APPKEY string contendo 16 bytes do APPKEY separados por ':' , se for NULL, o método retorna o APPKEY gravado na placa do radio
 * @return char* retorna o APPKEY gravado na placa do radio
 */
char *Modem_SmartModular::OTAA_APPKEY(char *APPKEY){ //vetor contendo 16 bytes de APPKEY separados por ':' e terminado por '\n'. Tamanho do vetor: 32 bytes
    char comando_listar[14] = "AT+APPKEY=?\r\n";
    char comando_appkey[61] = "";
    //para analise relacionado ao sprintf abaixo| "AT+APPKEY=00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00\r\n";
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__,"Comando AT+APPKEY...");
    #endif
    if (APPKEY==NULL){
        return OTAA_enviar_comando(comando_listar);
    }
    else{
        if(strlen(APPKEY)<=47){
            sprintf(comando_appkey,"%s%s\r\n","AT+APPKEY=",APPKEY);
            return OTAA_enviar_comando(comando_appkey);
        }
        else{
             #ifdef SERIAL_DEBUG
             ESP_LOGI(__func__,"APPKEY além 47 bytes de \"00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00\" , tamanho: %i...", strlen(APPKEY));
             #endif
            return NULL;
        }
    }
    //return NULL;
}

char *Modem_SmartModular::versao(){ //vetor contendo 16 bytes de DEVEUI separados por ':' e terminado por '\n'. Tamanho do vetor: 32 bytes
    char comando_listar[14] = "AT+VER=?\r\n";
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__,"Comando AT+VER...");
    #endif
    return OTAA_enviar_comando(comando_listar);
}

/**
 * @brief Método para enviar comando de RESET ao Modem. Se não retornar OK após 20 tentativas, força ESP restart...
 * 
 */
void Modem_SmartModular::reset(){
    uint8_t contar_erros=0;
    char cmd_reset[10]="ATZ?\r\n";
    int resetou=0;
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__, "RESET...");
    #endif
    while(resetou==0){        
        uart_write_bytes(uart_tx_rx, cmd_reset, strlen(cmd_reset)); //reiniciar a MCU do modem para enviar comando de novo        
        #ifdef SERIAL_DEBUG
        ESP_LOGI(__func__,"Esperando modem...");
        #endif
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        do{
            while(xQueueReceive( resposta_AT, ( void * ) rx_buf, ( TickType_t ) 100)!=pdTRUE){}
            #ifdef SERIAL_DEBUG
            ESP_LOGI(__func__,"Resposta...:%s", rx_buf);        
            #endif
        
            if(strstr(rx_buf, "OK")!=NULL){
                #ifdef SERIAL_DEBUG
                ESP_LOGI(__func__, "RESET FEITO, AGUARDAR 5s");            
                #endif
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                xQueueReset(resposta_AT);
                //ESP_ERROR_CHECK(uart_flush_input(uart_tx_rx));
                resetou=1;
            }
            else{
                contar_erros++;
                if(contar_erros>20)
                    esp_restart();
            }
        }while(uxQueueMessagesWaiting(resposta_AT)>0);
    }
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__, "RESET OK COM RX BUFFER ZERADO");
    #endif
}

/**
 * @brief Método para enviar comando de JOIN para o Modem. Faz uso do método this->join() para checar se
 * o JOIN foi efetivado com sucesso, devido ao tempo imprevisível que possa levar para fazer o JOIN.
 * Esse método aciona o método reset() após 5 tentativas de JOIN, uma vez que a comunicação ficou sem
 * acesso à rede LoraWAN...
 * 
 * @return uint8_t 1 indicando JOIN com sucesso
 */
uint8_t Modem_SmartModular::join(){
    uint8_t contar_erros=0;
    uint8_t tentativa=0;
    char cmd_join[10]="AT+JOIN\r\n";
    
    //this->reset();
    if(this->ver_join()==0){

        while(tentativa<10){
            #ifdef SERIAL_DEBUG
            ESP_LOGW(__func__, "Tentativa...: %i", (int)tentativa);
            #endif

            do{
                uart_write_bytes(uart_tx_rx, cmd_join, strlen(cmd_join));
                
                #ifdef SERIAL_DEBUG
                ESP_LOGI(__func__,"Esperando modem...");
                #endif
                vTaskDelay(100 / portTICK_PERIOD_MS);
            
                while(xQueueReceive( resposta_AT, ( void * ) rx_buf, ( TickType_t ) 1000)!=pdTRUE){}
                
                #ifdef SERIAL_DEBUG
                ESP_LOGI(__func__, "Resposta...:%s", rx_buf);
                #endif
                
                //if(strcmp(rx_buf,"\r\nAT_ERROR\r\n")==0)
                if(strstr(rx_buf, "AT_ERROR")!=NULL){
                    contar_erros++;
                    if(contar_erros>20)
                        esp_restart();                
                }
                else{
                    //if(strcmp(rx_buf,"\r\nOK\r\n")==0){
                    if(strstr(rx_buf, "OK")!=NULL){
                        #ifdef SERIAL_DEBUG
                        ESP_LOGI(__func__, "JOIN enviado...");
                        #endif
                        vTaskDelay(5000 / portTICK_PERIOD_MS); //tempo máximo para fazer Join...
                        xQueueReset(resposta_AT);//zerar vestigios de JOINED não documentado...

                        if(this->ver_join()==1){
                            return 1;
                        }
                    }
                    else{
                        #ifdef SERIAL_DEBUG
                        ESP_LOGI(__func__, "Resposta não prevista...: %s", rx_buf);
                        #endif
                    }
                    contar_erros=0;
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);                
            }while(uxQueueMessagesWaiting(resposta_AT)>0);

            tentativa++;
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            if(tentativa==5){
                this->reset();            
            }
        }
    }
    else{
        #ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "JOIN já realizado anteriormente.");
        #endif
        return 1;
    }
    return 0; //não mais executado, pois fará reset() para forçar novo JOIN
}

/**
 * @brief Envia comando ao Modem de verificação se fez JOIN na rede Lorawan. Há 20 tentativas, se não conseguir, 
 * reinicia o mode. Ainda, se por algum motivo o Modem travar e o ESP32 também, watchdog reiniciará ESP32...
 * 
 * @return uint8_t 1 se JOINED ou 0 se NOT JOINED
 */
uint8_t Modem_SmartModular::ver_join(){
    uint8_t exito=2;
    uint8_t contar_erros=0;
    char cmd_ver_join[11]="AT+NJS=?\r\n";
    while(exito==2){
        do{
            #ifdef SERIAL_DEBUG
            //ESP_LOGI(__func__,"Enviado...");    
            #endif
            uart_write_bytes(uart_tx_rx, cmd_ver_join, strlen(cmd_ver_join)); 
        
            #ifdef SERIAL_DEBUG
            //ESP_LOGI(__func__,"Esperando modem...");
            #endif
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            while(xQueueReceive( resposta_AT, ( void * ) rx_buf, ( TickType_t ) 100)!=pdTRUE){}
            #ifdef SERIAL_DEBUG
            ESP_LOGI(__func__,"Resposta...:%s", rx_buf);
            #endif
            if(strstr( rx_buf, "AT_ERROR")!=NULL){
                contar_erros++;
                if(contar_erros>20)
                    esp_restart();                
            }
            else{
                if(strstr(rx_buf,"OK")!=NULL){
                    if(strstr(rx_buf, "1")!=NULL)  //if(strcmp((char *)rx_buf,"1\r\n\r\nOK\r\n")==0)
                        exito=1;
                    if(strstr(rx_buf, "0")!=NULL) //if(strcmp((char *)rx_buf,"0\r\n\r\nOK\r\n")==0)
                        exito=0;
                }
                else{
                    #ifdef SERIAL_DEBUG
                    ESP_LOGI(__func__,"Resposta não prevista...:%s", rx_buf);
                    #endif
                }
                contar_erros=0;
            }
           
            
        }while(uxQueueMessagesWaiting(resposta_AT)>0);
    }
    return exito;
}


/**
 * @brief Task permanentemente em execução para monitorar o RX da UART por dados do Modem e atribuindo 
 * esses dados recebidos em String para a queue respostaAT
 * 
 * @param pvParameters Terá o ponteiro para o Objeto instanciado dessa classe (obtido do método static process_wrapper...)
 */
void Modem_SmartModular::rx_task(void *pvParameters){    
    Modem_SmartModular *ptr = static_cast<Modem_SmartModular*>(pvParameters);
    
    char *tmp = (char *) malloc(ptr->tam_rx);
   
    memset(tmp,0,ptr->tam_rx);
    while (1) {
        int i=0;
        int rxBytes=0;        

        while(rxBytes!=-1){
            rxBytes=uart_read_bytes(ptr->uart_tx_rx, ( void * ) &tmp[i], 1,  100 / portTICK_PERIOD_MS );            
            if(rxBytes>0){
                #ifdef SERIAL_DEBUG
                //ESP_LOGI(__func__,"Task - char: %2X",tmp[i]);
                #endif
                if(tmp[i]=='\n' || tmp[i]=='\r'){ //Segundo manual smartmodular, após '\r' o último é '\n'. Porém inconsistencia com JOINED nao documentado (por isso tmp[i]=='\r')
                    int tam_rx_ring = 0;
                    ESP_ERROR_CHECK(uart_get_buffered_data_len(ptr->uart_tx_rx, (size_t*)&tam_rx_ring));
                    #ifdef SERIAL_DEBUG
                    //ESP_LOGI(__func__,"Tamanho rx_ring: %i",tam_rx_ring);
                    #endif
                    if(tam_rx_ring==0){
                        i++;
                        tmp[i]='\0';
                        #ifdef SERIAL_DEBUG
                        //ESP_LOGI(__func__,"Recebeu: %s",tmp);
                        #endif
                        while(xQueueSend( ptr->resposta_AT, ( void * ) tmp, ( TickType_t ) 1000)!=pdPASS){}
                        #ifdef SERIAL_DEBUG
                        //ESP_LOGI(__func__,"Task - enfileirado...");
                        #endif
                        memset(tmp,0,ptr->tam_rx);
                        i=0;
                    }
                    else{
                        i++;
                    }
                }
                else {
                    i++;
                }
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    free(tmp);
}

/**
 * @brief Método para Enviar dados para a nuvem e Receber, se existir downlink, da nuvem. Os dados tx e rx são
 * vetores codigicados em caracteres ASCII e em formato string.
 * 
 * @param porta Indica qual a porta de uplink, que será referência para downlink.
 * @param dados_tx String a ser enviada via uplink
 * @param tam_dados_tx Tamanho da String a ser enviada
 * @param dados_rx String a ser recebida via downlink
 * @param tam_dados_rx Endereço de variável onde armazenará o Tamanho da String recebida. Se conter 0, não houve downlink...
 */
void Modem_SmartModular::enviar_receber(uint16_t porta, char *dados_tx, uint32_t tam_dados_tx, char *dados_rx, uint32_t *tam_dados_rx){
    uint8_t contar_erros=0;
    uint8_t contar_fila=0;
    uint32_t tam_msg_total = 0;
    uint8_t enviado=0;
    
    uint8_t recebido=0;
    char comando_recv[12]="AT+RECV=?\r\n";
    char *token=NULL;    
        
    sprintf(tx_buf,"%s%i:","AT+SEND=",porta);
    tam_msg_total = strlen(tx_buf);
    
    memcpy(&tx_buf[tam_msg_total],dados_tx,tam_dados_tx);
    tam_msg_total+=tam_dados_tx;

    tx_buf[tam_msg_total++] = '\r'; //montar a string padrão AT a enviar na UART    
    tx_buf[tam_msg_total++] = '\n';    
    tx_buf[tam_msg_total] = '\0';

    
    while(enviado==0){
        
        #ifdef SERIAL_DEBUG
        //ESP_LOGI(__func__, "---Cmd: %s", tx_buf);
        #endif
        uart_write_bytes(uart_tx_rx, tx_buf, tam_msg_total);
        vTaskDelay(5000 / portTICK_PERIOD_MS); //aguardar RX1 e RX2...
        contar_fila=0;
        do{
            while(xQueueReceive( resposta_AT, ( void * ) rx_buf, ( TickType_t ) 1000)!=pdTRUE){}
            #ifdef SERIAL_DEBUG
            //ESP_LOGI(__func__, "AT+SEND retornou: %s", rx_buf);        
            #endif
        
            if(strstr(rx_buf, "AT_ERROR")!=NULL){
                #ifdef SERIAL_DEBUG
                ESP_LOGI(__func__, "-AT_ERROR SEND: %s", rx_buf);
                #endif
                contar_erros++;
                if(contar_erros>20)
                    esp_restart();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else{
                if(strstr(rx_buf, "OK")!=NULL){ //BUG se +EVT , pode não vir OK e pode-se perder dados no RECV                    
                    #ifdef SERIAL_DEBUG
                    ESP_LOGI(__func__, "-OK SEND: %s", rx_buf);
                    #endif
                     vTaskDelay(1000 / portTICK_PERIOD_MS);
                    enviado=1;
                }
                else {
                        #ifdef SERIAL_DEBUG
                        ESP_LOGI(__func__, "Nem OK nem ERROR SEND: %s", rx_buf);
                        #endif
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                contar_erros=0;
            }
           #ifdef SERIAL_DEBUG
           ESP_LOGW(__func__, "Vez na filaS: %i", contar_fila);
           #endif
           contar_fila++;
        }while(uxQueueMessagesWaiting(resposta_AT)>0);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //xQueueReset(resposta_AT); //limpar qualquer lixo na fila vindo do SEND ou +EVT;
    memset(tx_buf,0,tam_msg_total);
    
    while(recebido==0){
        #ifdef SERIAL_DEBUG
        //ESP_LOGI(__func__, "---Cmd: %s", comando_recv);
        #endif
        uart_write_bytes(uart_tx_rx, comando_recv, strlen(comando_recv)); //pegar o que foi recebido RX1 e RX2
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        #ifdef SERIAL_DEBUG
        //ESP_LOGI(__func__, "AT+RECV transmitiu: %s", (char*) comando_recv);
        #endif
        contar_fila=0;
        do{
            while(xQueueReceive( resposta_AT, ( void * ) rx_buf, ( TickType_t ) 1000)!=pdTRUE){}
            #ifdef SERIAL_DEBUG
            //ESP_LOGI(__func__, "AT+RECV retornou: %s", rx_buf);
            #endif
            if(strstr(rx_buf, "AT_ERROR")!=NULL){
                #ifdef SERIAL_DEBUG
                ESP_LOGI(__func__, "-AT_ERROR RECV: %s", rx_buf);
                #endif
                contar_erros++;
                if(contar_erros>20)
                    esp_restart();
                vTaskDelay(1000 / portTICK_PERIOD_MS);                
            }
            else{
                if(strstr(rx_buf, "OK")!=NULL){
                    #ifdef SERIAL_DEBUG
                    ESP_LOGI(__func__, "-OK RECV: %s", rx_buf);
                    #endif
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    recebido=1;
                }
                else{
                    #ifdef SERIAL_DEBUG
                    ESP_LOGI(__func__, "-Nem OK nem ERROR RECV: %s", rx_buf);
                    #endif
                    vTaskDelay(1000 / portTICK_PERIOD_MS);  
                }
                contar_erros=0;
            }
            #ifdef SERIAL_DEBUG
            ESP_LOGW(__func__, "Vez na filaR: %i", contar_fila);
            #endif
            contar_fila++;
        }while(uxQueueMessagesWaiting(resposta_AT)>0);
    }
    
    *tam_dados_rx=0;
    dados_rx[0]='\0';
    token = strtok(rx_buf, "\r\n"); //separar "porta:dados\r\n\r\nOK\r\n"
    char *token2=NULL;
    char *tmp=NULL;
    while(token!=NULL){
        #ifdef SERIAL_DEBUG
        //ESP_LOGI(__func__, "Token: %s", token);
        #endif
        token2=strtok(token,":");
        #ifdef SERIAL_DEBUG
        //ESP_LOGI(__func__, "Token2 Porta: %s", token2);
        #endif
        if(token2!=NULL){
            if((uint8_t) strtol(token2,&tmp,10)==porta){ //só entra se achar a porta                
                token2 = strtok(NULL, ":"); //após a porta: pode conter os dados...
                if(token2!=NULL && strlen(token2) > 0) { //contem downlink...
                    #ifdef SERIAL_DEBUG
                    ESP_LOGI(__func__, "Token2 Dados: %s", token2);
                    #endif
                    *tam_dados_rx = strlen(token2);
                    memcpy(dados_rx,token2,*tam_dados_rx);
                    dados_rx[*tam_dados_rx]='\0';
                }
            }
        }
        token = strtok(NULL, "\r\n"); //pega o próximo token do rx_buf indicado antes...
    }
    //memcpy(dados_rx,rx_buf,*tam_dados_rx);
}

/*
void Modem_SmartModular::enviar_receber_bytes(uint16_t porta, uint8_t *dados_tx, uint32_t tam_dados_tx, uint8_t *dados_rx, uint32_t *tam_dados_rx){
    uint32_t tam_msg_total = 0;
    uint8_t enviado=0;
    
    uint8_t recebido=0;
    char comando_recv[15]="AT+RECVB=?\r\n";
    char *token=NULL;
        
    sprintf((char *)tx_buf,"%s%i:","AT+SENDB=",porta);
    tam_msg_total = strlen((char *)tx_buf);
    //for(int i=0;i<tam_dados_tx;i++){ //ajuste com base na tabela ASCII (byte para valor em hexa na string)
                                     //Ex: decimal 55, seria 37 em hexa. Ficaria 7 em string. Ficará em texto 3337, no ASCII p/ modem 37...
    for(int i=tam_dados_tx-1;i>=0;i--){
        uint8_t tmp_1o_nibble = dados_tx[i] & 0x0F;
        uint8_t tmp_2o_nibble = dados_tx[i] >> 4;

        if(tmp_2o_nibble >= 0x0 && tmp_2o_nibble <= 0x9) //0 até 9, ficará de 0x30 - 0x0x39
            tx_buf[tam_msg_total]=tmp_2o_nibble + 0x30; 
        else //A até F, ficará de 0x41 - 0x0x46
        //if(tmp_2o_nibble >= 0xa && tmp_2o_nibble <= 0xf)
            tx_buf[tam_msg_total]=tmp_2o_nibble + 0x37; 
        tam_msg_total++;
        if(tmp_1o_nibble >= 0x0 && tmp_1o_nibble <= 0x9) //0 até 9, ficará de 0x30 - 0x0x39
            tx_buf[tam_msg_total]=tmp_1o_nibble + 0x30; 
        else //A até F, ficará de 0x41 - 0x0x46
        //if(tmp_1o_nibble >= 0xa && tmp_1o_nibble <= 0xf)
            tx_buf[tam_msg_total]=tmp_1o_nibble + 0x37; 
        tam_msg_total++;
    }
    //memcpy(&tx_buf[tam_msg_total],dados_tx,tam_dados_tx);
    //tam_msg_total+=tam_dados_tx;

    tx_buf[tam_msg_total] = '\r'; //montar a string padrão AT a enviar na UART
    tam_msg_total+=1;
    tx_buf[tam_msg_total] = '\n';
    tam_msg_total+=1;
    tx_buf[tam_msg_total] = '\0';

    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__, "%s", (char*) tx_buf);
    #endif
    #ifdef SERIAL_DEBUG
    ESP_LOGI(__func__, "%X", (uint32_t) *dados_tx);
    #endif
    
    while(enviado==0){
        uart_write_bytes(uart_tx_rx, tx_buf, tam_msg_total);
        vTaskDelay(3000 / portTICK_PERIOD_MS); //tempo da janela RX1 (1s) e RX2 (2s)
        while(xQueueReceive( resposta_AT, ( void * ) rx_buf, ( TickType_t ) 1000)!=pdTRUE){}
        #ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "AT+SENDB retorno: %s", (char *) rx_buf);
        #endif
        if(strstr((char *)rx_buf, "AT_ERROR")!=NULL){
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        if(strstr((char *)rx_buf, "OK")!=NULL){
            #ifdef SERIAL_DEBUG
            ESP_LOGI(__func__, "Recebido após enviado:%s", (char*) rx_buf);
            #endif
            enviado=1;
        }
    }

    memset(tx_buf,0,tam_msg_total);

    while(recebido==0){
        uart_write_bytes(uart_tx_rx, comando_recv, strlen(comando_recv)); //não inclui o \0
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        while(xQueueReceive( resposta_AT, ( void * ) rx_buf, ( TickType_t ) 1000)!=pdTRUE){}
        #ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "AT+RECV retornou:%s", (char*) rx_buf);
        #endif
        if(strstr((char *)rx_buf, "AT_ERROR")!=NULL){
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        if(strstr((char *)rx_buf, "OK")!=NULL){
            recebido=1;
        }
    }

    *tam_dados_rx = strlen((char *) rx_buf); //o 1o token será porta:dados , que é o importante, depois vira \r\nOK\r\n não usaveis aqui.

    token = strtok((char *) rx_buf, "\r");
    while(token!=NULL){
        #ifdef SERIAL_DEBUG
        ESP_LOGI(__func__, "Token:%s", (char*) token);
        #endif
        token = strtok(NULL, "\r");
    }
    memcpy(dados_rx,rx_buf,*tam_dados_rx);
    
       
    memset(rx_buf,0,*tam_dados_rx);    
}
*/


