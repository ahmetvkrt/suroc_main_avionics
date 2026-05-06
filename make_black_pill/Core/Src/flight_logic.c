#include "flight_logic.h"

Flight_State_t current_flight_state = {0};

void Process_Flight_Logic(float filtered_alt, float filtered_vz, float filtered_pitch) {
    
    // 1. Detect Liftoff (Alt > 10m AND Pitch < 80 deg) [KTR Algorithm]
    if (!current_flight_state.liftoff_detected && filtered_alt > 10.0f && filtered_pitch < 80.0f) {
        current_flight_state.liftoff_detected = 1;
    }

    // Track Max Altitude continuously
    if (current_flight_state.liftoff_detected && filtered_alt > current_flight_state.max_altitude) {
        current_flight_state.max_altitude = filtered_alt;
    }

    // 2. Apogee Detection / Drogue Deployment 
    // Condition: Vz <= 0 AND Pitch < 30 AND (MaxAlt - Alt <= 100) [KTR Algorithm]
    if (current_flight_state.liftoff_detected && !current_flight_state.drogue_deployed) {
        if (filtered_vz <= 0.0f && filtered_pitch < 30.0f && (current_flight_state.max_altitude - filtered_alt <= 100.0f)) {
            current_flight_state.apogee_detected = 1;
            current_flight_state.drogue_deployed = 1;
            
            // FIRE PYRO 1 (Drogue / Sürüklenme)
            // Note: Define PYRO1_MOSFET_Pin and Port in CubeMX!
            // HAL_GPIO_WritePin(PYRO1_MOSFET_GPIO_Port, PYRO1_MOSFET_Pin, GPIO_PIN_SET);
        }
    }

    // 3. Main Parachute Deployment (Alt < 500m) [KTR Algorithm]
    if (current_flight_state.drogue_deployed && !current_flight_state.main_deployed) {
        if (filtered_alt < 500.0f) {
            current_flight_state.main_deployed = 1;
            
            // FIRE PYRO 2 (Main / Ana Paraşüt)
            // Note: Define PYRO2_MOSFET_Pin and Port in CubeMX!
            // HAL_GPIO_WritePin(PYRO2_MOSFET_GPIO_Port, PYRO2_MOSFET_Pin, GPIO_PIN_SET);
        }
    }
}