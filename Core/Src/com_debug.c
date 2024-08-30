/*
 * com_debug.c
 *
 *  Created on: Aug 29, 2024
 *      Author: sajanduwal
 */

#include "com_debug.h"

void delay_us(uint32_t us) {
	uint32_t delay_counter_disp;
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		delay_counter_disp++;
	// wait for the counter to reach the us input in the parameter
}

void myDebug(const char *fmt, ...) {
	static char temp[100];
	va_list args;
	va_start(args, fmt);
	vsnprintf(temp, sizeof(temp), fmt, args);
	va_end(args);
	int len = bufferSize(temp);
	HAL_UART_Transmit(&huart1, (uint8_t*) temp, len, 1000);
}

int bufferSize(char *buffer) {
	int i = 0;
	while (*buffer++ != '\0')
		i++;
	return i;
}
