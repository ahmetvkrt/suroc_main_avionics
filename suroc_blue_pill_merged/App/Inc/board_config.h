#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32f1xx_hal.h"

// feature flags

#define USE_BME680        1
#define USE_BNO085        1
#define USE_GPS           1

#define USE_BNO085_SPI    1   /* 1: SPI, 0: I2C */

// bus config

#define I2C1_SPEED_HZ     400000U
#define SPI1_SPEED_HZ     2000000U
#define GPS_BAUDRATE     115200U


 // BME680 (I2C)


#define BME680_I2C        hi2c1
#define BME680_ADDR      (0x76 << 1)


 // BNO085 (SPI)


#define BNO085_SPI        hspi1

#define BNO085_CS_PORT   GPIOA
#define BNO085_CS_PIN    GPIO_PIN_4

#define BNO085_INT_PORT  GPIOB
#define BNO085_INT_PIN   GPIO_PIN_0

#define BNO085_RST_PORT  GPIOB
#define BNO085_RST_PIN   GPIO_PIN_1


 // GPS (UART + PPS)


#define GPS_UART         huart2

#define GPS_PPS_PORT    GPIOA
#define GPS_PPS_PIN     GPIO_PIN_8


//Debug


#define DEBUG_UART      huart1


 // App timing


#define APP_LOOP_HZ     100U

#endif /* BOARD_CONFIG_H */
