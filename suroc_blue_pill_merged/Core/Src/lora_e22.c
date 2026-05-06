#include "lora_e22.h"

void E22_Init(void) {
    // Wait for the module to finish its power-on self-check
    while (HAL_GPIO_ReadPin(E22_AUX_PORT, E22_AUX_PIN) == GPIO_PIN_RESET) {}
    HAL_Delay(2); 
}

uint8_t E22_IsBusy(void) {
    return (HAL_GPIO_ReadPin(E22_AUX_PORT, E22_AUX_PIN) == GPIO_PIN_RESET) ? 1 : 0;
}

uint8_t E22_SendPayload_DMA(uint8_t *data, uint16_t size) {
    if (E22_IsBusy()) return 0; // Drop packet if LoRa is still broadcasting over the air

    if (HAL_UART_Transmit_DMA(E22_UART, data, size) == HAL_OK) {
        return 1; 
    }
    return 0; 
}