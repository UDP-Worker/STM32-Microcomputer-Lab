/*
 * button.c
 *
 *  Created on: Jan 23, 2025
 *      Author: lenovo
 */

#include "button.h"

//mode = 0:不支持连按，必须松开后才有下一次；mode = 1:支持连按
uint8_t button_scan(uint8_t mode){
	static uint8_t key_up = 1;
	uint8_t keyval = 0;

	if (mode) key_up = 1;

	if (key_up && (BUTTON0 == 0 || BUTTON1 == 0)){
		HAL_Delay(10);
		key_up = 0;

		if(BUTTON0 == 0){
			keyval = BUTTON0_PRES;
		}
		if(BUTTON1 == 0){
			keyval = BUTTON1_PRES;
		}
	}
	else if(BUTTON0==1 && BUTTON1==1){
		key_up = 1;
	}
	return keyval;

}
