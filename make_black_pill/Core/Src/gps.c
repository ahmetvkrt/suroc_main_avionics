#include "gps.h"

GPS_Data_t current_gps_data = {0};

#define NMEA_MAX_LEN 82
static char rx_buffer[NMEA_MAX_LEN];
static uint8_t rx_index = 0;
static uint8_t rx_byte;
static uint8_t nmea_ready = 0;

static void Get_NMEA_Field(char *nmea_str, uint8_t field_num, char *output) {
    uint8_t commas = 0;
    while (*nmea_str) {
        if (*nmea_str == ',') {
            commas++;
            nmea_str++;
            if (commas == field_num) {
                uint8_t i = 0;
                while (*nmea_str != ',' && *nmea_str != '*' && *nmea_str != '\0') {
                    output[i++] = *nmea_str++;
                }
                output[i] = '\0';
                return;
            }
            continue;
        }
        nmea_str++;
    }
    output[0] = '\0'; 
}

void GPS_Init(void) {
    HAL_UART_Receive_IT(GPS_UART, &rx_byte, 1);
}

void GPS_UART_RxCallback(uint8_t incoming_byte) {
    if (incoming_byte == '\n') {
        rx_buffer[rx_index] = '\0'; 
        nmea_ready = 1;             
        rx_index = 0;               
    } else if (incoming_byte != '\r' && rx_index < NMEA_MAX_LEN - 1) {
        rx_buffer[rx_index++] = (char)incoming_byte;
    }
    HAL_UART_Receive_IT(GPS_UART, &rx_byte, 1);
}

void GPS_Process(void) {
    if (nmea_ready) {
        if (strstr(rx_buffer, "GGA") != NULL) {
            char temp[15];
            Get_NMEA_Field(rx_buffer, 6, temp);
            current_gps_data.fix_quality = atoi(temp);

            if (current_gps_data.fix_quality > 0) {
                Get_NMEA_Field(rx_buffer, 7, temp);
                current_gps_data.satellites = atoi(temp);
                Get_NMEA_Field(rx_buffer, 9, temp);
                current_gps_data.altitude_m = atof(temp);
                Get_NMEA_Field(rx_buffer, 2, temp);
                current_gps_data.latitude = atof(temp);
                Get_NMEA_Field(rx_buffer, 4, temp);
                current_gps_data.longitude = atof(temp);

                current_gps_data.updated_flag = 1;
            }
        }
        nmea_ready = 0; 
    }
}