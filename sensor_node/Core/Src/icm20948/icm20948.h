/*
* @file icm20948.h
*
*  Created on: Dec 26, 2020
*      Author: mokhwasomssi
*
*  Modified: Juraj Dudak
*  Date: 30.12.2022
*/


#ifndef __ICM20948_H__
#define	__ICM20948_H__

#include "IcmSpiManager.h"

#include <stdbool.h>


class Sensor{
protected:
	int8_t _activeDevice;
	IcmSpiManager *_spi;
public:
	uint16_t _scaleFactor;
	axisesI data;
	int16_t offset_x;
	int16_t offset_y;
	int16_t offset_z;

	Sensor(IcmSpiManager *spi, int8_t _activeDevice);
	void SetScaleFactor(int16_t sf);
	void SetRange();
	void SetLowPassFilter();
	uint8_t SetSampleRate();
	uint8_t GetLowPassFilter(void);
	uint8_t GetSampleRate(void);
	uint8_t GetRange(void);
	uint8_t CheckLowPassInputValue(uint8_t);
	void Calibrate(void);
	uint8_t GetDataStatus(void);
	bool _read_without_scale_factor(axisesI* data);
	void SaveOffset(void);


	virtual axisesI *GetData(void) = 0;
	virtual bool Read(axisesI *) = 0;
	virtual bool ReadUnit(axisesF *)= 0;

};

class SensorAccel : public Sensor {
public:
	SensorAccel(IcmSpiManager *spi, int8_t _activeDevice);
	axisesI *GetData(void);
	bool Read(axisesI *);
	bool ReadUnit(axisesF*);

	void SetRange(accel_full_scale r);
	void SetLowPassFilter(accel_dlp_cfg config);
	uint8_t SetSampleRate(accel_samplerate divider);
	uint8_t CheckLowPassInputValue(uint8_t);
	void Calibrate(void);
	bool _read_without_scale_factor(axisesI* data);
	void SaveOffset(void);

	uint8_t GetLowPassFilter(void);
	uint8_t GetSampleRate(void);
	uint8_t GetRange(void);
};


class SensorGyro: public Sensor {
public:
	SensorGyro(IcmSpiManager *spi, int8_t _activeDevice);
	axisesI *GetData(void);
	bool Read(axisesI *);
	bool ReadUnit(axisesF*);

	void SetRange(gyro_full_scale r);
	void SetLowPassFilter(gyro_dlp_cfg config);
	uint8_t SetSampleRate(gyro_samplerate divider);
	uint8_t CheckLowPassInputValue(uint8_t);
	void Calibrate(void);
	void SaveOffset(void);

	uint8_t GetLowPassFilter(void);
	uint8_t GetSampleRate(void);
	uint8_t GetRange(void);
};

class SensorMag: public Sensor {
public:
	SensorMag(IcmSpiManager *spi, int8_t _activeDevice);
	axisesI *GetData(void);
	bool Read(axisesI *);
	bool ReadUnit(axisesF*);
};


class Icm20948{
private:
	uint8_t _numDevices;
	int8_t _activeDevice;
	IcmSpiManager *_spi_manager;
	McuPin_typeDef *_pinINT;
	bool _ak09916_enable;
	bool _sensor_ready;
	bool _is_present;

	bool icm20948_who_am_i();
	void icm20948_spi_slave_enable();
	void icm20948_i2c_master_reset();
	void icm20948_i2c_master_enable();
	void icm20948_i2c_master_clk_frq(uint8_t config);
	void icm20948_clock_source(uint8_t source);
	void icm20948_odr_align_enable();

    void ak09916_init(Config_Mag_t *config);
    bool ak09916_who_am_i();
    void ak09916_operation_mode_setting(AK09916_operation_mode mode);
    void ak09916_soft_reset();

public:
	SensorAccel *accSensor;
	SensorGyro *gyroSensor;
	SensorMag *magSensor;

    Icm20948();
    Icm20948(IcmSpiManager *spi, icm20948_Config *config);
    void SetInterruptSource(interrupt_source_enum int_source);
    void Calibrate(icm20948_Config *config);
    void Read(void);
	void Wakeup(void);
	void Sleep(void);
	uint8_t IsSleep(void);
	bool IsReady(void);
	uint8_t GetDataStatus(void);
	uint8_t GetIndex(void);
	uint8_t GetIndexZeroBased(void);
	void Reset(void);

	void Stop(void);
	void Start(void);

	uint8_t IsPresent(void);

	void EnableFifo(bool accel, bool gyro);
	void ResetFifo(void);
	uint16_t GetFifoCount(void);
	uint8_t* ReadFifo(uint16_t length);
	void ReadFifo_DMA(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t length);


};



#define ASSERT_SENSOR   {if (_is_present == false) {;}}


#endif	/* __ICM20948_H__ */
