/*
 * retarget_stdio.h
 *
 *  Created on: 17-Mar-2021
 *      Author: Raj.S
 */

#ifndef INC_RETARGET_STDIO_H_
#define INC_RETARGET_STDIO_H_
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>

//set you USART handler here
#define 	HUART	huart2

extern UART_HandleTypeDef HUART;

void RetargetInit(void);

#endif /* INC_RETARGET_STDIO_H_ */
