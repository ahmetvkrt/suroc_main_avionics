#ifndef E22_LORA_H
#define E22_LORA_H

#include "main.h" 

extern UART_HandleTypeDef huart2;
#define E22_UART &huart2

// Mapped directly to your main.h definitions
#define E22_AUX_PORT    LORA_AUX_GPIO_Port
#define E22_AUX_PIN     LORA_AUX_Pin

void E22_Init(void);
uint8_t E22_IsBusy(void);
uint8_t E22_SendPayload_DMA(uint8_t *data, uint16_t size);

#endif // E22_LORA_H