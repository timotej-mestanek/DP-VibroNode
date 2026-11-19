
#include "main.h"
#include <cstring>
#include <cstdio>
#include "icm20948/icm20948.h"


extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

uint8_t spi_tx_buf[13];
uint8_t spi_rx_buf[13];
char uart_tx_buf[128];

volatile uint8_t uart_tx_busy = 0;
volatile uint8_t sensor_data = 0;
volatile uint8_t spi_rx_complete = 0;


extern "C" void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_6) {
        sensor_data = 1;

    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

        spi_rx_complete = 1;
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_tx_busy = 0;
    }
}




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

    if (sensor.IsPresent()) {
         sensor.Wakeup();
         sensor.Start();
         HAL_Delay(100);

         sensor.ResetFifo();
         sensor.EnableFifo(true, true);
         uint8_t int_cfg =INT_PIN_CFG_2CLEAR;
         manager.write_single_reg(0, ub_0, B0_INT_PIN_CFG, int_cfg);
         manager.write_single_reg(0, ub_0, B0_INT_ENABLE_1, 0x01);

         const char ok_msg[] = "Senzor rdy, FIFO sbe...\r\n";
                  HAL_UART_Transmit(&huart1, (uint8_t*)ok_msg, strlen(ok_msg), 100);
     } else {
         const char err[] = "Senzor NF \r\n";
         HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), 100);
         while(1);
     }


    while(1){
            if (sensor_data == 1) {
                sensor_data = 0;

                uint16_t fifo_count = sensor.GetFifoCount();

                if (fifo_count >= 12) {
                    sensor.ReadFifo_DMA(spi_tx_buf, spi_rx_buf, 12);

                    fifo_count -= 12;
                }
            }
            if (spi_rx_complete == 1) {
                spi_rx_complete = 0;

                int16_t ax = (spi_rx_buf[1] << 8) | spi_rx_buf[2];
                int16_t ay = (spi_rx_buf[3] << 8) | spi_rx_buf[4];
                int16_t az = (spi_rx_buf[5] << 8) | spi_rx_buf[6];

                int16_t gx = (spi_rx_buf[7] << 8) | spi_rx_buf[8];
                int16_t gy = (spi_rx_buf[9] << 8) | spi_rx_buf[10];
                int16_t gz = (spi_rx_buf[11] << 8) | spi_rx_buf[12];


                    int len = sprintf(uart_tx_buf, "ACC: [%d, %d, %d] | GYRO: [%d, %d, %d]\r\n", ax, ay, az, gx, gy, gz);

                    HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buf, len, 100);

            }
        }
}
