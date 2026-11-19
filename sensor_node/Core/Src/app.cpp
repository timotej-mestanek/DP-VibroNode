
#include "main.h"
#include <cstring>


extern UART_HandleTypeDef huart1;

extern "C" void cpp_main(void) {

	const char test_msg[] = "uart test \r\n";

	    while (1) {
	        HAL_UART_Transmit(&huart1, (uint8_t*)test_msg, strlen(test_msg), 100);
	        HAL_Delay(1000);
	    }

}
