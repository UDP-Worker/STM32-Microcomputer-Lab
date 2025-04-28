/*
 * led.h
 *
 *  Created on: Jan 23, 2025
 *      Author: lenovo
 */

#ifndef BSP_LED_LED_H_
#define BSP_LED_LED_H_

#include "main.h"

#define led0(x)		x?	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET):\
						HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET)

#define led1(x)		x?	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET):\
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)

#define led2(x)		x?	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET):\
						HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)


#endif /* BSP_LED_LED_H_ */
