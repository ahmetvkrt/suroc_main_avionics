#ifndef BME68X_HAL
#define BME68X_HAL

#include "bme68x.h"
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

int bme68x_hal_init(struct bme68x_dev *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr7);

// Configure once after init
int bme68x_hal_configure(struct bme68x_dev *dev,
                         const struct bme68x_conf *conf,
                         const struct bme68x_heatr_conf *heatr_or_null);

// Then call this each cycle
int bme68x_hal_forced_once(struct bme68x_dev *dev,
                           const struct bme68x_conf *conf,
                           const struct bme68x_heatr_conf *heatr_or_null,
                           struct bme68x_data *out);

#ifdef __cplusplus
}
#endif

#endif
