#ifndef FLIGHT_LOGIC_H
#define FLIGHT_LOGIC_H

#include "stm32f4xx_hal.h"
#include "main.h" // For MOSFET pin definitions

typedef struct {
    uint8_t liftoff_detected;
    uint8_t apogee_detected;
    uint8_t drogue_deployed;
    uint8_t main_deployed;
    float   max_altitude;
} Flight_State_t;

extern Flight_State_t current_flight_state;

void Process_Flight_Logic(float filtered_alt, float filtered_vz, float filtered_pitch);

#endif // FLIGHT_LOGIC_H