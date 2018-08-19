#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

int _write(int file, char* ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, -1);
    return len;
}


void loop()
{
    uint8_t buf[24];
    while (1) {
        HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(&hi2c1,(uint16_t)(0b1101000 << 1), buf, sizeof(buf), 1000);
        if (ret == HAL_OK) {
            puts("success");
        }
    }
}