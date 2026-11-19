/*
 * app.cpp
 *
 *  Created on: Nov 19, 2026
 *      Author: timotej-mestanek
 */

#include "main.h"
#include "uart.h"

extern "C" void cpp_main(void){
	while(1){
		debug_log("Test 1213 \r \n");
		HAL_Delay(1000);
	}
}
