/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - BNO085 IMU Sensor
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Global değişkenler - sensör verilerini callback'te alacağız
volatile sh2_SensorEvent_t latestSensorEvent;
volatile uint8_t newDataReady = 0;
// Sensör değerleri - istediğiniz yerde kullanabilirsiniz
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
float quat_w, quat_x, quat_y, quat_z;
float roll, pitch, yaw;

uint32_t lastPrintTime = 0; // Son yazdırma zamanı
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ========== HAL FONKSIYONLARI ==========

int sh2_hal_open(sh2_Hal_t *self) {
    // I2C zaten CubeMX tarafından başlatıldı
    return SH2_OK;
}

void sh2_hal_close(sh2_Hal_t *self) {
    // Kapatma işlemi
}

int sh2_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Receive(&hi2c1, 0x4A << 1, pBuffer, len, 100);

    if (t_us) {
        *t_us = HAL_GetTick() * 1000; // Zaman damgası (mikrosaniye)
    }

    return (status == HAL_OK) ? len : 0;
}

int sh2_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c1, 0x4A << 1, pBuffer, len, 100);
    return (status == HAL_OK) ? len : 0;
}

uint32_t sh2_hal_getTimeUs(sh2_Hal_t *self) {
    return HAL_GetTick() * 1000; // ms'yi mikrosaniyeye çevir
}

// HAL yapısı
sh2_Hal_t hal = {
    .open = sh2_hal_open,
    .close = sh2_hal_close,
    .read = sh2_hal_read,
    .write = sh2_hal_write,
    .getTimeUs = sh2_hal_getTimeUs
};

// ========== CALLBACK FONKSIYONLARI ==========

// Sensör verisi geldiğinde otomatik çağrılır
void sensor_callback(void *cookie, sh2_SensorEvent_t *pEvent) {
    // Yeni veri geldi, kopyala
    latestSensorEvent = *pEvent;
    newDataReady = 1;
}

// Event callback (reset, hata vs için)
void event_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    if (pEvent->eventId == SH2_RESET) {
        // Sensör reset oldu
        // Gerekirse yeniden başlatma yapılabilir
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1); // timer'ı başlattım
  // I2C Tarama
  /*
    printf("I2C Scanning...\r\n");
    for(uint8_t addr = 0x01; addr < 0x7F; addr++) {
      if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 100) == HAL_OK) {
        printf("Found device at: 0x%02X\r\n", addr);
      }
    }

    for(uint8_t addr = 0x60; addr < 0x7F; addr++)
    {
        if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 100) == HAL_OK)
            printf("Found: 0x%02X\r\n", addr);
    }
    */

    //printf("Scan complete.\r\n");

  // UART'a başlangıç mesajı gönder
  printf("BNO085 Başlatılıyor...\r\n");
/*
  // BNO085 I2C adresi (7-bit)
  #define BNO085_ADDRESS 0x4A << 1  // HAL için 8-bit adres

  // I2C bağlantı testi
  uint8_t testData;
  HAL_StatusTypeDef status;

  status = HAL_I2C_IsDeviceReady(&hi2c1, BNO085_ADDRESS, 3, 100);
  if(status == HAL_OK) {
      printf("Başlatıldı!\r\n");
  }
*/
  //...
  // ========== BNO085 BAŞLATMA ==========

    int status = sh2_open(&hal, event_callback, NULL);

    if (status != SH2_OK) {
        // Hata! sh2_open başarısız
        while(1) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // LED yanıp sönsün
            HAL_Delay(200);
        }
    }

    HAL_Delay(100); // Sensörün başlaması için bekle

    // Sensor callback'i kaydet - BU ÇOK ÖNEMLİ!
    sh2_setSensorCallback(sensor_callback, NULL);

    // Sensör bilgilerini kontrol et
    sh2_ProductIds_t prodIds;
    status = sh2_getProdIds(&prodIds);

    if (status != SH2_OK) {
        // Sensörle iletişim kurulamadı
        while(1) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_Delay(100);
        }
    }

    // ========== SENSÖRLERI AKTİF ET ==========

    sh2_SensorConfig_t config;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = 10000;  // 10ms = 100Hz
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    // İstediğiniz tüm sensörleri aktif edin
    sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);           // Quaternion
    sh2_setSensorConfig(SH2_ACCELEROMETER, &config);             // Accelerometer
    sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config);      // Gyroscope
    sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED, &config); // Magnetometer
    sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config);       // Linear Accel
    sh2_setSensorConfig(SH2_GRAVITY, &config);                   // Gravity

    HAL_Delay(100); // Sensörlerin aktif olması için bekle

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    printf("Verileri okumaya geçiliyor...");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // sh2 servisini çalıştır - callback'leri tetikler
	      sh2_service();

	      // Yeni veri geldi mi kontrol et
	      if (newDataReady) {
	          newDataReady = 0; // Bayrağı temizle

	          sh2_SensorValue_t sensorValue;

	          // Ham event'i decode et
	          int rc = sh2_decodeSensorEvent(&sensorValue, &latestSensorEvent);

	          if (rc == SH2_OK) {

	              // Hangi sensör verisi geldi?
	              switch (sensorValue.sensorId) {

	                  case SH2_ACCELEROMETER:
	                      accel_x = sensorValue.un.accelerometer.x;
	                      accel_y = sensorValue.un.accelerometer.y;
	                      accel_z = sensorValue.un.accelerometer.z;

	                      // Burada kullanabilirsiniz (örn: UART ile gönderin)
	                      // printf("Accel: %.2f, %.2f, %.2f\n", accel_x, accel_y, accel_z);
	                      break;

	                  case SH2_GYROSCOPE_CALIBRATED:
	                      gyro_x = sensorValue.un.gyroscope.x; // rad/s
	                      gyro_y = sensorValue.un.gyroscope.y;
	                      gyro_z = sensorValue.un.gyroscope.z;
	                      break;

	                  case SH2_MAGNETIC_FIELD_CALIBRATED:
	                      mag_x = sensorValue.un.magneticField.x; // uTesla
	                      mag_y = sensorValue.un.magneticField.y;
	                      mag_z = sensorValue.un.magneticField.z;
	                      break;

	                  case SH2_ROTATION_VECTOR:
	                      quat_w = sensorValue.un.rotationVector.real;
	                      quat_x = sensorValue.un.rotationVector.i;
	                      quat_y = sensorValue.un.rotationVector.j;
	                      quat_z = sensorValue.un.rotationVector.k;

	                      // Quaternion'dan Euler açılarına çevirme
	                      roll = atan2f(2.0f*(quat_w*quat_x + quat_y*quat_z),
	                                    1.0f - 2.0f*(quat_x*quat_x + quat_y*quat_y));

	                      pitch = asinf(2.0f*(quat_w*quat_y - quat_z*quat_x));

	                      yaw = atan2f(2.0f*(quat_w*quat_z + quat_x*quat_y),
	                                   1.0f - 2.0f*(quat_y*quat_y + quat_z*quat_z));

	                      // Radyandan dereceye çevir
	                      roll = roll * 180.0f / M_PI;
	                      pitch = pitch * 180.0f / M_PI;
	                      yaw = yaw * 180.0f / M_PI;

	                      // Kullanım örneği
	                      // printf("Roll: %.1f, Pitch: %.1f, Yaw: %.1f\n", roll, pitch, yaw);
	                      break;

	                  case SH2_LINEAR_ACCELERATION:
	                      float lin_x = sensorValue.un.linearAcceleration.x;
	                      float lin_y = sensorValue.un.linearAcceleration.y;
	                      float lin_z = sensorValue.un.linearAcceleration.z;
	                      // Yerçekimi filtreli ivme
	                      break;

	                  case SH2_GRAVITY:
	                      float grav_x = sensorValue.un.gravity.x;
	                      float grav_y = sensorValue.un.gravity.y;
	                      float grav_z = sensorValue.un.gravity.z;
	                      break;

	                  default:
	                      // Diğer sensörler
	                      break;
	              }
	          }
	      }

	      // Her 100ms'de bir tüm verileri yazdır
	          if (HAL_GetTick() - lastPrintTime > 500) {
	              lastPrintTime = HAL_GetTick();

	              printf("\r\n=== BNO085 Sensor Data ===\r\n");
	              printf("Roll: %.1f  Pitch: %.1f  Yaw: %.1f\r\n", roll, pitch, yaw);
	              printf("Accel: X:%.2f Y:%.2f Z:%.2f m/s2\r\n", accel_x, accel_y, accel_z);
	              printf("Gyro:  X:%.2f Y:%.2f Z:%.2f rad/s\r\n", gyro_x, gyro_y, gyro_z);
	              printf("Mag:   X:%.1f Y:%.1f Z:%.1f uT\r\n", mag_x, mag_y, mag_z);
	              printf("==========================\r\n");
	          }

	      HAL_Delay(10); // Küçük gecikme
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// printf fonksiyonunu UART'a yönlendir
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
