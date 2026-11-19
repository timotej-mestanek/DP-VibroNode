
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
    config1.pinINT = &pinINT;
    config1.int_source = interrupt_RAW_DATA_0_RDY_EN;
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

         sensor.ResetFifo();
         sensor.EnableFifo(true, true);

         const char ok_msg[] = "Senzor rdy, FIFO sbe...\r\n";
                  HAL_UART_Transmit(&huart1, (uint8_t*)ok_msg, strlen(ok_msg), 100);
     } else {
         const char err[] = "Senzor NF \r\n";
         HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), 100);
         while(1);
     }

    while(1){
            uint16_t fifo_count = sensor.GetFifoCount();

            while (fifo_count >= 12) {

                uint8_t* fifo_data = sensor.ReadFifo(12);
                int16_t ax = (fifo_data[0] << 8) | fifo_data[1];
                int16_t ay = (fifo_data[2] << 8) | fifo_data[3];
                int16_t az = (fifo_data[4] << 8) | fifo_data[5];

                int16_t gx = (fifo_data[6] << 8) | fifo_data[7];
                int16_t gy = (fifo_data[8] << 8) | fifo_data[9];
                int16_t gz = (fifo_data[10] << 8) | fifo_data[11];

                int len = sprintf(buffer, "ACC: [%d, %d, %d] | GYRO: [%d, %d, %d]\r\n", ax, ay, az, gx, gy, gz);
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 10);

                fifo_count -= 12;
            }

            HAL_Delay(1000);
        }

}
