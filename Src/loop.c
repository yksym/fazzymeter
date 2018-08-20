#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

static const uint32_t ACCEL_START = 0x3b;
static const uint32_t TIMEOUT = 1000;

typedef struct SensorData {
    uint16_t acc[3];
    uint16_t temp;
    uint16_t gyro[3];
} SensorData;

int _write(int file, char* ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, -1);
    return len;
}


void loop()
{
    SensorData data;
    while (1) {
        HAL_StatusTypeDef ret;
        ret= HAL_I2C_Mem_Read(&hi2c1, 0xd0, ACCEL_START, 1, (uint8_t*)&data, sizeof(data), TIMEOUT);
        if (ret != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
        //ret = HAL_I2C_Master_Receive(&hi2c1,(uint16_t)(0b1101000 << 1), (uint8_t*)&data, sizeof(data), TIMEOUT);
    }
}