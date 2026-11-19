
#include "main.h"
#include <cstring>
#include <cstdio>
#include "icm20948/icm20948.h"
#include "config.h"
#include "debug.h"
#include "process.h"


constexpr uint16_t FIFO_SIZE = 24;

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

uint8_t spi_tx_buf[FIFO_SIZE + 1]; /* CUZ OF address + 1byte */
uint8_t spi_rx_buf[FIFO_SIZE + 1];


static volatile uint8_t sensor_data = 0;
static volatile uint8_t spi_busy = 0;
static volatile uint8_t spi_rx_complete = 0;
static uint32_t overflow_cnt = 0;



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

static inline int16_t be16(const uint8_t *p) {
    return (int16_t)((p[0] << 8) | p[1]);
}

static Sample parse_sample(const uint8_t *p) {
    Sample s;
    s.ax = be16(p + 0);
    s.ay = be16(p + 2);
    s.az = be16(p + 4);
    s.gx = be16(p + 6);
    s.gy = be16(p + 8);
    s.gz = be16(p + 10);
    return s;
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

         debug_log("Sensor ready, FIFO...\r\n");
     } else {
        debug_log("Sensor not found");
         while(1);
     }

    uint8_t last_read_bytes = 0;

    while(1){
            if (sensor_data && !spi_busy) {
                sensor_data = 0;

                uint16_t fifo_count = sensor.GetFifoCount();

                if (fifo_count >= FIFO_OVERFLOW) {
                	sensor.ResetFifo();
                	overflow_cnt++;
                	debug_log("FIFO overflow, reset\r\n");
                } else {
                	uint16_t packets = fifo_count / PACKET_SIZE;
                	if (packets > MAX_PACKETS) packets = MAX_PACKETS;
                	if (packets > 0) {
                		last_read_bytes = packets * PACKET_SIZE;
                		spi_busy = 1;
                		sensor.ReadFifo_DMA(spi_tx_buf, spi_rx_buf, last_read_bytes);
                        }
                  }

            }
            if (spi_rx_complete) {
                spi_rx_complete = 0;
                spi_busy = 0;

                const uint8_t *p = &spi_rx_buf[1];
                for (uint16_t i = 0; i < last_read_bytes; i += PACKET_SIZE) {
                                process_sample(parse_sample(p + i));
                                debug_print_sample(process_last(), process_count());
                }
                sensor_data = 1;
            }
        }
}
