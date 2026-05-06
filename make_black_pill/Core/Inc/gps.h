#ifndef GPS_H
#define GPS_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart6; 
#define GPS_UART &huart6

typedef struct {
    float    latitude;     
    float    longitude;    
    float    altitude_m;   
    uint8_t  satellites;   
    uint8_t  fix_quality;  
    uint8_t  updated_flag;
} GPS_Data_t;

extern GPS_Data_t current_gps_data;

void GPS_Init(void);
void GPS_UART_RxCallback(uint8_t rx_byte);
void GPS_Process(void);

#endif // GPS_H