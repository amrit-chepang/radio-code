/*
 * com_debug.c
 *
 *  Created on: Feb 6, 2024
 *      Author: sajanduwal
 */

#include "com_debug.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"

void delay_us(uint32_t us) {
	uint32_t delay_counter_disp;
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		delay_counter_disp++;
	// wait for the counter to reach the us input in the parameter
}

void myPrintf(const char *fmt, ...) {
	static char temp[100];
	va_list args;
	va_start(args, fmt);
	vsnprintf(temp, sizeof(temp), fmt, args);
	va_end(args);
	int len = bufferSize(temp);
	HAL_UART_Transmit(&huart2, (uint8_t*) temp, len, 1000);
}

int bufferSize(char *buffer) {
	int i = 0;
	while (*buffer++ != '\0')
		i++;
	return i;
}
