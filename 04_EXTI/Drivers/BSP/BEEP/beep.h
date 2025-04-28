/*
 * beep.h
 *
 *  Created on: Jan 23, 2025
 *      Author: lenovo
 */

#ifndef BSP_BEEP_BEEP_H_
#define BSP_BEEP_BEEP_H_

#define beep(x)		x?	HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET):\
						HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET)

#endif /* BSP_BEEP_BEEP_H_ */
