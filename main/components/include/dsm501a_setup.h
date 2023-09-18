#ifndef DSM501A_SETUP_H
#define DSM501A_SETUP_H

#include "global_data.h"

#define GPIO_INPUT_PIN_SEL      (uint64_t)((1ULL<<DSM_P10_PIN) | (1ULL<<DSM_P25_PIN)) // Define a Bit Mask de ambos os pinos
#define ESP_INTR_FLAG_DEFAULT   0

void dsm501a_task ( void *pvParameters );

#endif /* DSM501A_SETUP_H */
