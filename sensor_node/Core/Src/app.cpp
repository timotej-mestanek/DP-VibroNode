
#include "main.h"
#include <cstring>
#include <cstdio>
#include "icm20948/icm20948.h"


extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;


extern "C" void cpp_main(void) {

	/* ACCL SETUP */

	IcmSpiManager manager(&hspi1);
    icm20948_Config config1;
    McuPin_typeDef pinCS = {GPIOA, GPIO_PIN_12};
    McuPin_typeDef pinINT = {GPIOB, GPIO_PIN_6};

    config1.pinCS = &pinCS;
    config1.int_source = interrupt_disable;
    config1.gyro.low_pass_filter = GYRO_lpf_196_6Hz;
    config1.gyro.sample_rate = GYRO_samplerate_281_3Hz;
    config1.accel.low_pass_filter = ACCEL_lpf_246Hz;
    config1.accel.sample_rate = ACCEL_samplerate_281_3Hz;
    config1.mag.mode = mag_mode_cont_measurement_100hz;

    Icm20948 sensor(&manager, &config1);

    char buffer[128];

    if (sensor.IsPresent()) {
         sensor.Wakeup();
         sensor.Start();
         HAL_Delay(100);
         const char ok_msg[] = "Senzor pripojen, zacinam merit...\r\n";
         HAL_UART_Transmit(&huart1, (uint8_t*)ok_msg, strlen(ok_msg), 100);
     } else {
         const char err[] = "Senzor nenalezen!\r\n";
         HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), 100);
         while(1);
     }

    while(1){
    	sensor.Read();
    	        axisesI *gyro  = sensor.gyroSensor->GetData();
    	        axisesI *accel = sensor.accSensor->GetData();
    	        axisesI *mag   = sensor.magSensor->GetData();

    	        int len = sprintf(buffer, "mag: [%d, %d, %d] | GYRO: [%d, %d, %d] | MAG: [%d, %d, %d]\t \n",
    	                          accel->x, accel->y, accel->z,
    	                          gyro->x, gyro->y, gyro->z,
    	                          mag->x, mag->y, mag->z);
    	        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
    	        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
    }


}
