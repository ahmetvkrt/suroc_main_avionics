/*
 * BluePill (STM32F103C8) + BME680 via Bosch bme68x driver
 * I2C1: PB6=SCL, PB7=SDA (100kHz)
 * USART1: PA9=TX, PA10=RX (115200) for printf
 * LED: PC13 (active-low)
 */

#include "app.h"
#include "main.h"
#include "bme68x_hal.h"
#include "bme68x.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>   // for powf

/* Handles */
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart1;

/* printf over USART1 */
int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

/* LED helpers (PC13 active low) */
static inline void led_on(void)  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); } // active-low
static inline void led_off(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); }
static inline void led_toggle(void){ HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); }

/* Pressure → altitude conversion */
static const float P0_HPA = 1013.25f;  // standard atmosphere
static inline float altitude_from_hpa(float p_hpa) {
    // International barometric formula
    // Alt (m) = 44330 * (1 - (P/P0)^(1/5.255))
    return 44330.0f * (1.0f - powf(p_hpa / P0_HPA, 0.190294957f));
}

/* BME680 context */
static struct bme68x_dev dev;
static struct bme68x_conf bme_conf;
static struct bme68x_heatr_conf heat_off;

/* Main ---------------------------------------------------------------------*/
void main_avionic_init(void)
{
  led_on();
  printf("\r\nBME680 (bme68x) app init\r\n");

  /* --- Probe 0x76 / 0x77 --- */
  uint8_t addr7 = 0x77;
  uint8_t id=0;
  HAL_I2C_Mem_Read(&hi2c1, addr7<<1, 0xD0, I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
  printf("Probe chip ID at 0x%02X: 0x%02X (expect 0x61)\r\n", addr7, id);


  /* --- Init Bosch driver --- */
  if (bme68x_hal_init(&dev, &hi2c1, addr7) != BME68X_OK) {
      printf("bme68x init failed\r\n");
      Error_Handler();
  }
  printf("bme68x init OK, chip 0x%02X\r\n", dev.chip_id);

  // --- Config once ---
  bme_conf.os_hum  = BME68X_OS_1X;
  bme_conf.os_pres = BME68X_OS_1X;
  bme_conf.os_temp = BME68X_OS_1X;
  bme_conf.filter  = BME68X_FILTER_OFF;

  heat_off.enable = BME68X_DISABLE;

  bme68x_hal_configure(&dev, &bme_conf, &heat_off);
}

void main_avionic_loop(void)
{
    static uint32_t lastPrint = 0;
    struct bme68x_data d;

    if (bme68x_hal_forced_once(&dev, &bme_conf, &heat_off, &d) == 0) {
        if ((HAL_GetTick() - lastPrint) > 200) { // ~5 Hz print
            float p_hpa = d.pressure / 100.0f;
            float alt_m = altitude_from_hpa(p_hpa);
            printf("BME680: T=%.2f C, P=%.2f hPa, H=%.2f %%RH, Alt=%.1f m\r\n",
                   d.temperature, p_hpa, d.humidity, alt_m);
            lastPrint = HAL_GetTick();
            led_toggle();
        }
    }
}
