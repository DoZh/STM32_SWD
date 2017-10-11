#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

int stdout_putchar (int ch) {
  uint8_t buf[1]; 
  buf[0] = ch;

	HAL_UART_Transmit(&huart1, (uint8_t *)buf, 1, 10);
  return (ch);
}

int stderr_putchar (int ch) {
  uint8_t buf[1]; 
  buf[0] = ch;

	HAL_UART_Transmit(&huart1, (uint8_t *)buf, 1, 10);
  return (ch);
}

int stdin_getchar (void) {
	uint8_t buf[1]; 
	HAL_UART_Receive(&huart1, (uint8_t *)buf, 1, 10);
  return (buf[0]);
}
