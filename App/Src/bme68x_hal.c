#include "bme68x_hal.h"

extern I2C_HandleTypeDef hi2c1;

// --- Small helpers ---
static int i2c_ready_retry(uint8_t addr7, uint32_t tries, uint32_t tout_ms) {
  for (uint32_t i=0; i<tries; ++i) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (addr7<<1), 1, tout_ms) == HAL_OK) return 0;
  }
  return -1;
}

static int8_t i2c_reg_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr)
{
  uint8_t addr7 = *(uint8_t*)intf_ptr;
  uint16_t dev  = (uint16_t)(addr7 << 1);
  if (i2c_ready_retry(addr7, 2, 10) != 0) return -1;

  // a couple of retries with a longer timeout
  for (int t=0; t<3; ++t) {
    if (HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t*)data, (uint16_t)len, 1000) == HAL_OK) return 0;
    HAL_Delay(2);
  }
  return -1;
}

static int8_t i2c_reg_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr)
{
  uint8_t addr7 = *(uint8_t*)intf_ptr;
  uint16_t dev  = (uint16_t)(addr7 << 1);
  if (i2c_ready_retry(addr7, 2, 10) != 0) return -1;

  for (int t=0; t<3; ++t) {
    if (HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT,
                         data, (uint16_t)len, 1000) == HAL_OK) return 0;
    HAL_Delay(2);
  }
  return -1;
}

// Bosch asks for µs; HAL has ms. Fine for forced mode.
static void delay_us(uint32_t us, void *intf_ptr) {
  (void)intf_ptr;
  HAL_Delay((us + 999) / 1000);
}

int bme68x_hal_init(struct bme68x_dev *dev, I2C_HandleTypeDef *unused, uint8_t addr7)
{
  static uint8_t addr_store;
  addr_store = addr7;

  dev->intf     = BME68X_I2C_INTF;
  dev->read     = i2c_reg_read;
  dev->write    = i2c_reg_write;
  dev->delay_us = delay_us;
  dev->intf_ptr = &addr_store;
  dev->amb_temp = 25;

  dev->delay_us(5000, dev->intf_ptr);
  return bme68x_init(dev);
}

// Configure once (call this once after init)
int bme68x_hal_configure(struct bme68x_dev *dev,
                         const struct bme68x_conf *conf,
                         const struct bme68x_heatr_conf *heatr_or_null)
{
  if (bme68x_set_conf((struct bme68x_conf*)conf, dev) != BME68X_OK) return -1;

  if (heatr_or_null && heatr_or_null->enable == BME68X_ENABLE) {
    if (bme68x_set_heatr_conf(BME68X_FORCED_MODE,
                              (struct bme68x_heatr_conf*)heatr_or_null, dev) != BME68X_OK) return -2;
  } else {
    struct bme68x_heatr_conf off = {0};
    if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &off, dev) != BME68X_OK) return -3;
  }
  return 0;
}

// Trigger one forced measurement + read it
int bme68x_hal_forced_once(struct bme68x_dev *dev,
                           const struct bme68x_conf *conf,    // only used for meas_dur
                           const struct bme68x_heatr_conf *heatr_or_null,
                           struct bme68x_data *out)
{
	  if (bme68x_set_op_mode(BME68X_FORCED_MODE, dev) != BME68X_OK) return -4;

	  // Bosch API returns duration in MICROSECONDS:
	  const uint32_t sens_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, (struct bme68x_conf*)conf, dev);
	  const uint32_t heat_us = (heatr_or_null && heatr_or_null->enable == BME68X_ENABLE)
	                           ? ((uint32_t)heatr_or_null->heatr_dur * 1000U)  // ms -> us
	                           : 0U;

	  // small margin ~2 ms
	  dev->delay_us(sens_us + heat_us + 2000U, dev->intf_ptr);

	  uint8_t n_fields = 0;
	  int8_t st = bme68x_get_data(BME68X_FORCED_MODE, out, &n_fields, dev);
	  return (st == BME68X_OK && n_fields) ? 0 : -5;
}
