#ifndef BME680_H
#define BME680_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Choose your I2C address based on SDO pin wiring.
#define BME680_I2C_ADDR_0x76 (0x76 << 1)
#define BME680_I2C_ADDR_0x77 (0x77 << 1)

// Pick one:
#ifndef BME680_I2C_ADDR
#define BME680_I2C_ADDR BME680_I2C_ADDR_0x77
#endif

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t dev_addr;

    // Calibration data
    uint16_t par_T1;
    int16_t  par_T2;
    int8_t   par_T3;

    uint16_t par_P1;
    int16_t  par_P2;
    int8_t   par_P3;
    int16_t  par_P4;
    int16_t  par_P5;
    int8_t   par_P6;
    int8_t   par_P7;
    int16_t  par_P8;
    int16_t  par_P9;
    uint8_t  par_P10;

    uint16_t par_H1;
    uint16_t par_H2;
    int8_t   par_H3;
    int8_t   par_H4;
    int8_t   par_H5;
    uint8_t  par_H6;
    int8_t   par_H7;

    uint8_t  par_GH1;
    int16_t  par_GH2;
    int8_t   par_GH3;

    int32_t  t_fine;

    // gas config
    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    uint8_t  range_sw_err;
} BME680_t;

typedef struct {
    float temperature_c;   // °C
    float pressure_pa;     // Pa
    float humidity_rh;     // %RH
    float gas_res_ohms;    // Ω (if gas enabled)
} BME680_Data_t;

// Public API
HAL_StatusTypeDef BME680_Init(BME680_t *dev, I2C_HandleTypeDef *hi2c, uint16_t i2c_addr);
HAL_StatusTypeDef BME680_Configure(BME680_t *dev,
                                   uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h,
                                   uint8_t iir_filter,
                                   uint16_t heater_temp_c, uint16_t heater_dur_ms);
HAL_StatusTypeDef BME680_ForceRead(BME680_t *dev, BME680_Data_t *out);

#ifdef __cplusplus
}
#endif

// ---- Oversampling & filter constants (public) ----
// oversampling encoding per Bosch (0..5)
#define OSRS_SKIP 0
#define OSRS_1X   1
#define OSRS_2X   2
#define OSRS_4X   3
#define OSRS_8X   4
#define OSRS_16X  5

// IIR filter (0=off, 1=2, 2=4, 3=8, 4=16)
#define IIR_OFF   0
#define IIR_2     1
#define IIR_4     2
#define IIR_8     3
#define IIR_16    4

#endif

