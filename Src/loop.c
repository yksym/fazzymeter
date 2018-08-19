#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

int _write(int file, char* ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, -1);
    return len;
}


void loop()
{
    puts("memo");
}