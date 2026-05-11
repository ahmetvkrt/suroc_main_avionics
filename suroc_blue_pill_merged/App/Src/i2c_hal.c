#include "i2c_hal.h"
#include "main.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

#define RSTN_PORT GPIOA
#define RSTN_PIN  GPIO_PIN_15

#define INTN_PORT GPIOB
#define INTN_PIN GPIO_PIN_4

#define BOOTN_PORT GPIOB
#define BOOTN_PIN  GPIO_PIN_5

#define RESET_DELAY_US (10000)
#define START_DELAY_US (2000000)
#define READ_LEN (2)

enum BusState_e {
    BUS_INIT,
    BUS_IDLE,
    BUS_READING_LEN,
    BUS_GOT_LEN, 
    BUS_READING_TRANSFER,
    BUS_WRITING
};

static bool isOpen = false;
enum BusState_e i2cBusState;
volatile uint32_t rxTimestamp_uS;
static uint8_t rxBuf[SH2_HAL_MAX_TRANSFER_IN];
static uint32_t rxBufLen;
static uint16_t payloadLen;
static uint8_t txBuf[SH2_HAL_MAX_TRANSFER_OUT];
static bool rxDataReady;
static uint16_t i2cAddr;
static volatile bool inReset;
static sh2_Hal_t sh2Hal;

void enableInts(void) {
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void disableInts(void) {
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
}

static void enableI2cInts(void) {
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

static void disableI2cInts(void) {
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
}

static void sh2_gpio_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = INTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(INTN_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    GPIO_InitStruct.Pin = RSTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RSTN_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOOTN_PIN;
    HAL_GPIO_Init(BOOTN_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, GPIO_PIN_SET);
}

static uint32_t timeNowUs(void) {
    uint32_t msec = HAL_GetTick();
    uint32_t load = SysTick->LOAD + 1;
    uint32_t val = SysTick->VAL;
    return (msec * 1000) + (((load - val) * 1000) / load);
}

static void delay_us(uint32_t t) {
    uint32_t start = timeNowUs();
    while ((timeNowUs() - start) < t) {}
}

static void reset_delay_us(uint32_t t) {
    uint32_t start = timeNowUs();
    while (((timeNowUs() - start) < t) && (inReset)) {}
}

// --- MISSING I2C ERROR CALLBACKS RE-ADDED HERE ---
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        i2cBusState = BUS_IDLE; // Reset state machine!
        rxDataReady = true;     // Try again
    }
}
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        i2cBusState = BUS_IDLE;
        rxDataReady = true;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *pI2c)
{
    if (i2cBusState == BUS_READING_LEN)
    {
        uint16_t len = (rxBuf[0] + (rxBuf[1] << 8)) & ~0x8000;
        payloadLen = (len > sizeof(rxBuf)) ? sizeof(rxBuf) : len;

        i2cBusState = BUS_READING_TRANSFER;
        if (HAL_I2C_Master_Receive_IT(&hi2c1, i2cAddr, rxBuf, payloadLen) != HAL_OK) {
            i2cBusState = BUS_IDLE;
            rxDataReady = true;
        }
    }
    else if (i2cBusState == BUS_READING_TRANSFER)
    {
        rxBufLen = payloadLen;
        i2cBusState = BUS_IDLE;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *i2c)
{
    if (i2cBusState == BUS_WRITING) {
        i2cBusState = BUS_IDLE;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t n)
{
    if (n != INTN_PIN) return;
    if (i2cBusState == BUS_INIT) return;
    
    rxTimestamp_uS = timeNowUs();
    inReset = false;

    if (i2cBusState == BUS_IDLE)
    {
        if (hi2c1.State != HAL_I2C_STATE_READY) {
            rxDataReady = true;
            return;
        }

        if (rxBufLen > 0) { rxBufLen = 0; }

        i2cBusState = BUS_READING_LEN;
        if (HAL_I2C_Master_Receive_IT(&hi2c1, i2cAddr, rxBuf, READ_LEN) != HAL_OK) {
            i2cBusState = BUS_IDLE;
            rxDataReady = true;
        }
    }
    else
    {
        rxDataReady = true;
    }
}

void EXTI4_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4); }
void I2C1_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c1); }

static int sh2_i2c_hal_open(sh2_Hal_t *self)
{
    printf("[HAL] Starting sh2_i2c_hal_open()...\r\n");
    if (isOpen) return SH2_ERR;

    sh2_gpio_init();
    i2cBusState = BUS_INIT;
    isOpen = true;

    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN, GPIO_PIN_RESET);
    inReset = true;

    enableInts();
    delay_us(RESET_DELAY_US);
    
    i2cBusState = BUS_IDLE;
    rxBufLen = 0;
    rxDataReady = false;

    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN, GPIO_PIN_SET);

    reset_delay_us(START_DELAY_US);

    if (inReset) {
        printf("[HAL] ERROR: INTN was NOT received! Sensor not responding.\r\n");
        return SH2_ERR;
    }

    // --- NEW: AUTO-PROBE THE I2C ADDRESS SO IT NEVER FAILS! ---
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x4A << 1, 3, 100) == HAL_OK) {
        i2cAddr = 0x4A << 1;
        printf("[HAL] BNO085 Found at 0x4A!\r\n");
    } else if (HAL_I2C_IsDeviceReady(&hi2c1, 0x4B << 1, 3, 100) == HAL_OK) {
        i2cAddr = 0x4B << 1;
        printf("[HAL] BNO085 Found at 0x4B!\r\n");
    } else {
        printf("[HAL] ERROR: BNO085 not found on I2C bus!\r\n");
        return SH2_ERR;
    }

    printf("[HAL] sh2_i2c_hal_open() SUCCESS\r\n");
    return SH2_OK;
}

static void sh2_i2c_hal_close(sh2_Hal_t *self)
{
    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, GPIO_PIN_SET);
    i2cBusState = BUS_INIT;
    disableInts();
    isOpen = false;
}

static int sh2_i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    int retval = 0;
    
    disableInts();
    if (rxBufLen > 0)
    {
        if (len < rxBufLen) {
            rxBufLen = 0;
            retval = SH2_ERR_BAD_PARAM;
        } else {
            memcpy(pBuffer, rxBuf, rxBufLen);
            retval = rxBufLen;
            rxBufLen = 0;
            *t = rxTimestamp_uS;
        }
    }
    enableInts();

    if ((rxDataReady || (HAL_GPIO_ReadPin(INTN_PORT, INTN_PIN) == GPIO_PIN_RESET))
        && (i2cBusState == BUS_IDLE)
        && (hi2c1.State == HAL_I2C_STATE_READY))
    {
        rxDataReady = false;
        i2cBusState = BUS_READING_LEN;
        if (HAL_I2C_Master_Receive_IT(&hi2c1, i2cAddr, rxBuf, READ_LEN) != HAL_OK) {
            i2cBusState = BUS_IDLE;
            rxDataReady = true;
        }
    }
    
    return retval;
}

static int sh2_i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    int retval = 0;
    if ((pBuffer == 0) || (len == 0) || (len > sizeof(txBuf))) return SH2_ERR_BAD_PARAM;

    disableI2cInts();
    if (i2cBusState == BUS_IDLE)
    {
        i2cBusState = BUS_WRITING;
        memcpy(txBuf, pBuffer, len);
        HAL_I2C_Master_Transmit_IT(&hi2c1, i2cAddr, txBuf, len);
        retval = len;
    }
    enableI2cInts();
    
    return retval;
}

static uint32_t sh2_i2c_hal_getTimeUs(sh2_Hal_t *self) { return timeNowUs(); }

sh2_Hal_t *sh2_hal_init(void)
{
    sh2Hal.open = sh2_i2c_hal_open;
    sh2Hal.close = sh2_i2c_hal_close;
    sh2Hal.read = sh2_i2c_hal_read;
    sh2Hal.write = sh2_i2c_hal_write;
    sh2Hal.getTimeUs = sh2_i2c_hal_getTimeUs;
    return &sh2Hal;
}