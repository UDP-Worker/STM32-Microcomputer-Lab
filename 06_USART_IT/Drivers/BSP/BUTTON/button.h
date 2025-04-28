/*
 * button.h
 *
 *  Created on: Jan 23, 2025
 *      Author: lenovo
 */

#ifndef BSP_BUTTON_BUTTON_H_
#define BSP_BUTTON_BUTTON_H_

#include "main.h"
#define BUTTON0	HAL_GPIO_ReadPin(BUTTON0_GPIO_Port, BUTTON0_Pin)
#define BUTTON1	HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin)

#define BUTTON0_PRES 1
#define BUTTON1_PRES 2

uint8_t button_scan(uint8_t mode);

#endif /* BSP_BUTTON_BUTTON_H_ */
