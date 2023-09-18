/**
 * @brief Definição da Classe Modem_SmartModular
 * 
 */

#ifndef MODEM_SMARTMODULAR_H
#define MODEM_SMARTMODULAR_H

#include "global_data.h"

//RX SmartModular 14 (será TX no esp32)
#define TX_SERIAL_AT_PIN (LORA_TX_PIN)
//TX SmartModular 13 (será RX no esp32)
#define RX_SERIAL_AT_PIN (LORA_RX_PIN)

class Modem_SmartModular{
    private:        
        uint8_t uart_tx_rx=0; //porta UART
        char *rx_buf=NULL; //buffer de RX
        char *tx_buf=NULL; //buffer de TX
        uint32_t tam_rx=0; //tamanho do buffer RX
        uint32_t tam_tx=0; //tamanho do buffer TX
        TaskHandle_t xHandle = NULL; //Armazenar a task para ser usada no método ~Modem_SmartModular

        QueueHandle_t resposta_AT; //fila preenchida pela task rx_task() com resposta do Modem e consumida pelos demais métodos

        void rx_task(void *pvParameters); //Tarefa instanciada dentro do Objeto, permitindo acesso aos atributos instanciados do objeto dessa classe

        static void process_wrapper(void* pvParameters){ //máscara para chamar rx_task() que não é static
            reinterpret_cast<Modem_SmartModular*>(pvParameters)->rx_task(pvParameters);
        }
        char *OTAA_enviar_comando(char *comando=NULL); //Método a ser usado pelo OTAA_DEUI, OTAA_APPEUI, OTAA_APPKEY

    public:
        Modem_SmartModular(uart_port_t uart_num=LOR_UART_PORT, uint8_t tx_uart=TX_SERIAL_AT_PIN, uint8_t rx_uart=RX_SERIAL_AT_PIN, uint32_t baud_rate=9600, uint8_t data_bits=UART_DATA_8_BITS, uint8_t parity=UART_PARITY_DISABLE, uint8_t stop_bits=UART_STOP_BITS_1, uint8_t flow_control=UART_HW_FLOWCTRL_DISABLE, uint32_t tam_buffer=2048);
        ~Modem_SmartModular();
        //OTAA
        char *auto_join(int8_t valor=-1);
        char *salvar_config();
        char *OTAA_DEUI(); //só leitura, conforme datasheet ???
        char *OTAA_APPEUI(char *APPEUI=NULL); //vetor contendo 8 bytes em 00 de APPEUI separados por ':' e terminado por '\n' (padrão ChirpStack). Tamanho do vetor: 16 bytes
        char *OTAA_APPKEY(char *APPKEY=NULL); //vetor contendo 16 bytes de DEVEUI separados por ':' e terminado por '\n'. Tamanho do vetor: 32 bytes
        char *versao();
        uint8_t join();
        uint8_t ver_join();
        //OTAA
        void reset();
        void enviar_receber(uint16_t porta, char *dados_tx, uint32_t tam_dados_tx, char *dados_rx, uint32_t *tam_dados_rx);
        //void enviar_receber_bytes(uint16_t porta, uint8_t *dados_tx, uint32_t tam_dados_tx, uint8_t *dados_rx, uint32_t *tam_dados_rx);
};

#endif