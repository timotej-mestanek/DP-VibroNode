/*
 * icm_datatypes.h
 *
 *  Created on: 8. 1. 2023
 *      Author: juraj
 */

#ifndef ICM_DATATYPES_H_
#define ICM_DATATYPES_H_


#include "stm32c0xx_hal.h"

#if defined(STM32F401xC) || defined(STM32F401xE)
#include "stm32f4xx_hal.h"
#endif

#if defined (STM32L432xx)
#include "stm32l4xx_hal.h"
#endif

#if defined (STM32L031xx)
#include "stm32l0xx_hal.h"
#endif


#define ICM20948_ACCEL					0x10
#define ICM20948_GYRO					0x20
#define ICM20948_MAG					0x30
#define ICM20948_TEMP					0x40

#define GET2BYTES(ptr,offset) 		((((uint16_t)*(ptr+(offset)))<<8) + *(ptr+(offset)+1))


typedef enum
{
	sensorAll = 0,
	sensorAccIndex = ICM20948_ACCEL >> 4,
	sensorGyroIndex = ICM20948_GYRO >> 4,
	sensorMagIndex =  ICM20948_MAG >> 4,
}Icm20948_sensorIndex;

typedef struct
{
	uint8_t type;
	float x;
	float y;
	float z;
} axisesF;

typedef struct
{
	uint8_t type;
	int16_t x;
	int16_t y;
	int16_t z;
} axisesI;


typedef enum
{
	GYRO_FS_250dps,
	GYRO_FS_500dps,
	GYRO_FS_1000dps,
	GYRO_FS_2000dps,
	GYRO_FS_None = 0xFF
} gyro_full_scale;

typedef enum
{
	ACCEL_FS_2g,
	ACCEL_FS_4g,
	ACCEL_FS_8g,
	ACCEL_FS_16g,
	ACCEL_FS_NoneG = 0xFF
} accel_full_scale;


/**
 * @brief Acceleroemter settings: register ACCEL_CONFIG_1 (Bank 2).
 */
typedef enum
{
	/** output rate = 4500Hz */
	ACCEL_lpf_OFF = 0xFE,
	/** output rate = determined by @ref  icm20948ACCEL_sample_rate_divider function, @ref accel_samplerate respectivelly */
	//ACCEL_lpf_246Hz = 0x0,
	ACCEL_lpf_246Hz = 0x1,
	ACCEL_lpf_114_4Hz = 0x2,
	ACCEL_lpf_050_4Hz = 0x3,
	ACCEL_lpf_023_9Hz = 0x4,
	ACCEL_lpf_011_5Hz = 0x5,
	ACCEL_lpf_005_7Hz = 0x6,
	ACCEL_lpf_473Hz = 0x7,
	ACCEL_lpf_None = 0xFF,
} accel_dlp_cfg;


typedef enum
{
	ACCEL_samplerate_562_5Hz = 0,
	ACCEL_samplerate_281_3Hz = 1,
	ACCEL_samplerate_187_5Hz = 2,
	ACCEL_samplerate_140_6Hz = 3,
	ACCEL_samplerate_102_3Hz = 4,
	ACCEL_samplerate_70_3Hz = 5,
	ACCEL_samplerate_48_9Hz = 6,
	ACCEL_samplerate_35_2Hz = 7,
	ACCEL_samplerate_17_6Hz = 8,
	ACCEL_samplerate_8_8Hz = 9,
	ACCEL_samplerate_4_4Hz = 10,
	ACCEL_samplerate_2_2Hz = 11,
	ACCEL_samplerate_1_1Hz = 12,
	ACCEL_samplerate_0_55Hz = 13,
	ACCEL_samplerate_0_27Hz = 14,
	ACCEL_samplerate_None = 255,

} accel_samplerate;


typedef enum
{
	GYRO_samplerate_562_5Hz = 1,
	GYRO_samplerate_375_0Hz = 2,
	GYRO_samplerate_281_3Hz = 3,
	GYRO_samplerate_225_0Hz = 4,
	GYRO_samplerate_187_5Hz = 5,
	GYRO_samplerate_140_6Hz = 7,
	GYRO_samplerate_125_0Hz = 8,
	GYRO_samplerate_102_3Hz = 10,
	GYRO_samplerate_070_3Hz = 15,
	GYRO_samplerate_066_2Hz = 16,
	GYRO_samplerate_048_9Hz = 22,
	GYRO_samplerate_035_2Hz = 31,
	GYRO_samplerate_034_1Hz = 32,
	GYRO_samplerate_017_6Hz = 63,
	GYRO_samplerate_017_3Hz = 64,
	GYRO_samplerate_004_4Hz = 255,
	GYRO_samplerate_None = 254,  //0xFE
} gyro_samplerate;


/**
 * @brief Gyroscope settings: register GYRO_CONFIG_1 (Bank 2).
 */
typedef enum
{
	/** output rate = 9000Hz */
	GYRO_low_pass_OFF = 0xFE,
	/** output rate = determined by @ref  icm20948_gyro_sample_rate_divider function, @ref accel_samplerate respectivelly */
	GYRO_lpf_196_6Hz = 0x0,
	GYRO_lpf_151_8Hz = 0x1,
	GYRO_lpf_119_5Hz = 0x2,
	GYRO_lpf_051_2Hz = 0x3,
	GYRO_lpf_023_9Hz = 0x4,
	GYRO_lpf_011_6Hz = 0x5,
	GYRO_lpf_005_7Hz = 0x6,
	GYRO_lpf_361_4Hz = 0x7,
	GYRO_lpf_None = 0xFF,
} gyro_dlp_cfg;


typedef enum
{
	mag_mode_power_down = 0,
	mag_mode_single_measurement = 1,
	mag_mode_cont_measurement_010hz = 2,
	mag_mode_cont_measurement_020hz = 4,
	mag_mode_cont_measurement_050hz = 6,
	mag_mode_cont_measurement_100hz = 8
} AK09916_operation_mode;

typedef enum
{
	interrupt_disable = 0,
	/** Enable DMP interrupt to propagate to interrupt pin 1. */
	interrupt_DMP_INT1_EN = 1 << 1,
	/** Enable PLL RDY interrupt (PLL RDY means PLL is running and in use as the clock source for the system) to propagate to interrupt pin 1. */
	interrupt_PLL_RDY_EN = 1 << 2,
	/** Enable interrupt for wake on motion to propagate to interrupt pin 1. */
	interrupt_WOM_INT_EN = 1 << 3,
	/** Enable wake on FSYNC interrupt. */
	interrupt_REG_WOF_EN = 1 << 7,
	/** Enable raw data ready interrupt from any sensor to propagate to interrupt pin 1.*/
	interrupt_RAW_DATA_0_RDY_EN = 1 << 8
}interrupt_source_enum;

typedef struct
{
  uint8_t I2C_MST_INT : 1;
  uint8_t DMP_INT1 : 1;
  uint8_t PLL_RDY_INT : 1;
  uint8_t WOM_INT : 1;
  uint8_t reserved_0 : 4;
} ICM_INT_STATUS_t;


typedef struct{
    GPIO_TypeDef *port;
    uint16_t pin;
}McuPin_typeDef;

typedef struct
{
	accel_dlp_cfg low_pass_filter;// = ACCEL_lpf_OFF;
	accel_samplerate sample_rate;// = ACCEL_samplerate_562_5Hz;
	accel_full_scale full_scale;// = _4g;
}Config_Accel_t;

typedef struct
{
	gyro_dlp_cfg low_pass_filter;// = GYRO_low_pass_OFF;
	gyro_samplerate sample_rate;// = GYRO_samplerate_375_0Hz;
	gyro_full_scale full_scale;// = _250dps;
}Config_Gyro_t;

typedef struct
{
	AK09916_operation_mode mode;// = mag_mode_power_down;
}Config_Mag_t;

typedef struct
{
	McuPin_typeDef *pinCS;// = NULL;
	McuPin_typeDef *pinINT;// = NULL;
	McuPin_typeDef *pinLED;// = NULL;
	Config_Accel_t accel;
	Config_Gyro_t gyro;
	Config_Mag_t mag;
	interrupt_source_enum int_source;// = interrupt_disable;
}icm20948_Config;


#endif /* ICM_DATATYPES_H_ */
