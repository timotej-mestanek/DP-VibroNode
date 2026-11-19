/*
* @file icm20948.cpp
*
*  Created on: Dec 26, 2020
*      Author: mokhwasomssi (ANSII C version)
*
*  Modified: Juraj Dudak  (C++ version)
*  Date: 30.12.2022
*/


#include "icm20948.h"

uint16_t ACCEL_SampleRate_Table[15] = {
	1,
	3,
	5,
	7,
	10,
	15,
	22,
	31,
	34,
	127,
	255,
	513,
	1022,
	2044,
	4095,
};


Icm20948::Icm20948(IcmSpiManager *spi, icm20948_Config *config){

	_activeDevice = spi->addSlave(config->pinCS);
	_spi_manager = spi;
	_sensor_ready = false;

	accSensor = new SensorAccel(spi, _activeDevice);
	gyroSensor = new SensorGyro(spi, _activeDevice);
	magSensor = new SensorMag(spi, _activeDevice);

	_is_present = this->icm20948_who_am_i();
	if (_is_present == false){
		return;
	}
//	while(!this->icm20948_who_am_i());

	this->accSensor->data.type = ICM20948_ACCEL;		// + _activeDevice;
	this->gyroSensor->data.type = ICM20948_GYRO;		// + _activeDevice;
	this->magSensor->data.type = ICM20948_MAG;			// + _activeDevice;

	this->Reset();
	this->Wakeup();

	this->SetInterruptSource(config->int_source);

	this->gyroSensor->SetLowPassFilter(config->gyro.low_pass_filter);
	this->accSensor->SetLowPassFilter(config->accel.low_pass_filter);

	this->gyroSensor->SetSampleRate(config->gyro.sample_rate);
	this->accSensor->SetSampleRate(config->accel.sample_rate);

	this->accSensor->SetRange(config->accel.full_scale);
	this->gyroSensor->SetRange(config->gyro.full_scale);

	this->ak09916_init(&config->mag);

	_sensor_ready = true;
	this->Stop();

}
uint8_t Icm20948::IsPresent(void){
	return (int)_is_present;
}

void Icm20948::Calibrate(icm20948_Config *config)
{
	this->SetInterruptSource(config->int_source);

	if (config != NULL)
	{
		this->gyroSensor->SetLowPassFilter(config->gyro.low_pass_filter);
		this->accSensor->SetLowPassFilter(config->accel.low_pass_filter);

		this->gyroSensor->SetSampleRate(config->gyro.sample_rate);
		this->accSensor->SetSampleRate(config->accel.sample_rate);

		this->accSensor->SetRange(config->accel.full_scale);
		this->gyroSensor->SetRange(config->gyro.full_scale);

		this->ak09916_init(&config->mag);
	}

	this->accSensor->Calibrate();
	this->gyroSensor->Calibrate();

}

bool Icm20948::icm20948_who_am_i()
{
	uint8_t icm20948_id = _spi_manager->read_single_reg(_activeDevice, ub_0, B0_WHO_AM_I);

	if(icm20948_id == ICM20948_ID)
		return true;
	else
		return false;
}

void Icm20948::Reset(void)
{
	ASSERT_SENSOR;

	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_1, 0x81); // reset - 0x80, not sleep 0x40, source PLL auto 0x01
	HAL_Delay(10);  // min 2 ms delay have to be placed here

	this->icm20948_clock_source(1);
	this->icm20948_odr_align_enable();
	this->icm20948_spi_slave_enable();

}


void Icm20948::Wakeup(void)
{
//	uint8_t new_val = _spi_manager->read_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_1);
//	new_val &= 0xBF;
	uint8_t new_val = 0x01; // Auto selects the best available clock source

	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(50);
}

uint8_t Icm20948::IsSleep(void)
{
	uint8_t new_val = _spi_manager->read_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_1);
	return new_val & 0x40;
}

bool Icm20948::IsReady(void)
{
	return _sensor_ready;
}

void Icm20948::Stop(void)
{
	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_2, 0x3F);	// vypne vsetky senzory
	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_INT_ENABLE_1, 0x0);
	_sensor_ready = false;
}

void Icm20948::Start(void)
{
	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_2, 0x0);
	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_INT_ENABLE_1, 0x1);
	_sensor_ready = true;
}

void Icm20948::Sleep(void)
{
	uint8_t new_val = _spi_manager->read_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_1);
	new_val |= 0x40;

	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void Icm20948::icm20948_spi_slave_enable()
{
	uint8_t new_val = _spi_manager->read_single_reg(_activeDevice, ub_0, B0_USER_CTRL);
	new_val |= 0x10;

	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_USER_CTRL, new_val);
}

void Icm20948::icm20948_i2c_master_reset()
{
	uint8_t new_val = _spi_manager->read_single_reg(_activeDevice, ub_0, B0_USER_CTRL);
	new_val |= 0x02;

	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_USER_CTRL, new_val);
}

void Icm20948::icm20948_i2c_master_enable()
{
	uint8_t new_val = _spi_manager->read_single_reg(_activeDevice, ub_0, B0_USER_CTRL);
	new_val |= 0x20;

	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_USER_CTRL, new_val);
	HAL_Delay(100);
}


void Icm20948::icm20948_i2c_master_clk_frq(uint8_t config)
{
	uint8_t new_val = _spi_manager->read_single_reg(_activeDevice, ub_3, B3_I2C_MST_CTRL);
	new_val |= config;

	_spi_manager->write_single_reg(_activeDevice, ub_3, B3_I2C_MST_CTRL, new_val);
}

void Icm20948::icm20948_clock_source(uint8_t source)
{
	uint8_t new_val = _spi_manager->read_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_PWR_MGMT_1, new_val);
}

void Icm20948::icm20948_odr_align_enable()
{
	_spi_manager->write_single_reg(_activeDevice, ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void Icm20948::SetInterruptSource(interrupt_source_enum int_source)
{
	_spi_manager->write_single_reg(_activeDevice, ub_0, B0_INT_PIN_CFG, (INT_PIN_CFG_2CLEAR));  // ( INT_PIN_CFG_LATCH | ...
    uint8_t source1 = (uint8_t)(int_source & 0xFF);
    uint8_t source2 = (uint8_t)((int_source >> 8) & 0xFF);
    _spi_manager->write_single_reg(_activeDevice, ub_0, B0_INT_ENABLE, source1);
    _spi_manager->write_single_reg(_activeDevice, ub_0, B0_INT_ENABLE_1, source2);

}


void Icm20948::Read(void)
{
	uint8_t* temp =_spi_manager->read_multiple_reg(_activeDevice, ub_0, B0_ACCEL_XOUT_H, 12);

	this->accSensor->data.x = (int16_t)(temp[0] << 8 | temp[1]);
	this->accSensor->data.y = (int16_t)(temp[2] << 8 | temp[3]);
	this->accSensor->data.z = (int16_t)(temp[4] << 8 | temp[5]) + this->accSensor->_scaleFactor;
	// Add scale factor because calibration function offset gravity acceleration.

	this->gyroSensor->data.x = (int16_t)(temp[6] << 8 | temp[7]);
	this->gyroSensor->data.y = (int16_t)(temp[8] << 8 | temp[9]);
	this->gyroSensor->data.z = (int16_t)(temp[10] << 8 | temp[11]);

	if(this->_ak09916_enable){
		if(this->magSensor->Read(&this->magSensor->data) == false){
			this->magSensor->data.x = 0;
			this->magSensor->data.y = 0;
			this->magSensor->data.z = 0;
		}
	}
}


uint8_t Icm20948::GetDataStatus(void)
{
	// same register for ACC and GYRO sensor
	return this->accSensor->GetDataStatus();
}

/// ---------------------- MAGNETOEMTER -----------------------------------


void Icm20948::ak09916_init(Config_Mag_t *config)
{

	this->icm20948_i2c_master_reset();
	this->icm20948_i2c_master_enable();
	this->icm20948_i2c_master_clk_frq(7);

	this->_ak09916_enable = false;
	if(this->ak09916_who_am_i()){
		this->_ak09916_enable = true;
	}

	if(this->_ak09916_enable == true){
		this->ak09916_soft_reset();
		this->ak09916_operation_mode_setting(config->mode);

		if (config->mode == mag_mode_power_down) {
			this->_ak09916_enable = false;
		} else {
			this->_ak09916_enable = true;
		}
	}
}

void Icm20948::ak09916_soft_reset()
{
	_spi_manager->write_single_external_reg(_activeDevice, MAG_CNTL3, 0x01);
	HAL_Delay(100);
}


bool Icm20948::ak09916_who_am_i()
{
	uint8_t ak09916_id = _spi_manager->read_single_external_reg(_activeDevice, MAG_WIA2);

	if(ak09916_id == AK09916_ID)
		return true;
	else
		return false;
}

void Icm20948::ak09916_operation_mode_setting(AK09916_operation_mode mode)
{
	_spi_manager->write_single_external_reg(_activeDevice, MAG_CNTL2, mode);
	HAL_Delay(100);
}


uint8_t Icm20948::GetIndexZeroBased(void)
{
	return this->_activeDevice;
}


uint8_t Icm20948::GetIndex(void)
{
	return this->_activeDevice + 1;
}

/* ----------------- SENSORS --------------------
 * 1) Sensor Accel
 * 2) Sensor Gyro
 * 3) Sensor Mag
 * ---------------------------------------------- */

Sensor::Sensor(IcmSpiManager *spi, int8_t activeDevice){
	_spi = spi;
	_activeDevice = activeDevice;
	_scaleFactor = 1;
	offset_x = 0;
	offset_y = 0;
	offset_z = 0;
}

/**
 * @brief Interrupt status regiter - INT_STATUS_1
 * @return 1 – Sensor Register Raw Data, from all sensors, is updated and ready to be read.
 */
uint8_t Sensor::GetDataStatus(void)
{
	uint8_t new_val = _spi->read_single_reg(_activeDevice, ub_0, B0_INT_STATUS_1);
	return new_val & 0x01;
}

bool Sensor::_read_without_scale_factor(axisesI* data)
{
	return this->Read(data);
}

SensorGyro::SensorGyro(IcmSpiManager *spi, int8_t activeDevice):Sensor(spi, activeDevice){

}

SensorAccel::SensorAccel(IcmSpiManager *spi, int8_t activeDevice):Sensor(spi, activeDevice){

}

SensorMag::SensorMag(IcmSpiManager *spi, int8_t activeDevice):Sensor(spi, activeDevice){

}

void Sensor::SetScaleFactor(int16_t sf){
	_scaleFactor = sf;
}

uint8_t Sensor::GetLowPassFilter(void){
	return 0xFF;
}

uint8_t Sensor::GetSampleRate(void)
{
	return 0xFF;
}

uint8_t Sensor::GetRange(void){
	return 0;
}

//bool SensorGyro::Read(axisesI* data)
bool SensorGyro::Read(axisesI* localData)
{

	uint8_t* temp = this->_spi->read_multiple_reg(this->_activeDevice, ub_0, B0_GYRO_XOUT_H, 6);

	localData->x = (int16_t)(temp[0] << 8 | temp[1]);
	localData->y = (int16_t)(temp[2] << 8 | temp[3]);
	localData->z = (int16_t)(temp[4] << 8 | temp[5]);

	return true;
}


bool SensorAccel::Read(axisesI* localData)
{
	uint8_t* temp = this->_spi->read_multiple_reg(this->_activeDevice, ub_0, B0_ACCEL_XOUT_H, 6);

	localData->x = (int16_t)(temp[0] << 8 | temp[1]);
	localData->y = (int16_t)(temp[2] << 8 | temp[3]);
	localData->z = (int16_t)(temp[4] << 8 | temp[5]) + this->_scaleFactor;
	// Add scale factor because calibration function offset gravity acceleration.

	return true;
}

/**
 * Do not use this function to read final data. It is only for calibration of sensor.
 */
bool SensorAccel::_read_without_scale_factor(axisesI* localData)
{
	uint8_t* temp = this->_spi->read_multiple_reg(this->_activeDevice, ub_0, B0_ACCEL_XOUT_H, 6);

	localData->x = (int16_t)(temp[0] << 8 | temp[1]);
	localData->y = (int16_t)(temp[2] << 8 | temp[3]);
	localData->z = (int16_t)(temp[4] << 8 | temp[5]);
	// Add scale factor because calibraiton function offset gravity acceleration.

	return true;
}


bool SensorMag::Read(axisesI* localData)
{
	uint8_t* temp;
	uint8_t drdy;	// data ready
	uint8_t hofl;	// overflow

	drdy = this->_spi->read_single_external_reg(this->_activeDevice, MAG_ST1) & 0x01;
	if(!drdy)	return false;

	temp = this->_spi->read_multiple_external_reg(this->_activeDevice, MAG_HXL, 6);

	hofl = this->_spi->read_single_external_reg(this->_activeDevice, MAG_ST2) & 0x08;
	if(hofl)	return false;

	localData->x = (int16_t)(temp[1] << 8 | temp[0]);
	localData->y = (int16_t)(temp[3] << 8 | temp[2]);
	localData->z = (int16_t)(temp[5] << 8 | temp[4]);

	return true;
}

bool SensorGyro::ReadUnit(axisesF *result)
{
	this->Read(&this->data);

	result->x = this->data.x / this->_scaleFactor;
	result->y = this->data.y / this->_scaleFactor;
	result->z = this->data.z / this->_scaleFactor;

	return true;
}

bool SensorAccel::ReadUnit(axisesF *result)
{
	this->Read(&this->data);

	result->x = this->data.x / this->_scaleFactor;
	result->y = this->data.y / this->_scaleFactor;
	result->z = this->data.z / this->_scaleFactor;

	return true;
}

bool SensorMag::ReadUnit(axisesF *result)
{
	bool new_data = this->Read(&this->data);
	if(!new_data)	return false;

	result->x = (float)(data.x * 0.15);
	result->y = (float)(data.y * 0.15);
	result->z = (float)(data.z * 0.15);

	return true;
}


axisesI *SensorAccel::GetData()
{
	return &this->data;
}


axisesI *SensorGyro::GetData()
{
	return &this->data;
}

axisesI *SensorMag::GetData()
{
	return &this->data;
}


void SensorAccel::SetRange(accel_full_scale full_scale)
{
	uint8_t new_val = _spi->read_single_reg(_activeDevice, ub_2, B2_ACCEL_CONFIG);
	new_val = new_val & 0xF9;	// remova ACCEL_FS_SEL bites [1-2]
	int16_t accel_scale_factor;
	switch(full_scale)
	{
		case ACCEL_FS_2g :
			new_val |= 0x00;
			accel_scale_factor = 16384;
			break;
		case ACCEL_FS_4g :
			new_val |= 0x02;
			accel_scale_factor = 8192;
			break;
		case ACCEL_FS_8g :
			new_val |= 0x04;
			accel_scale_factor = 4096;
			break;
		case ACCEL_FS_16g :
			new_val |= 0x06;
			accel_scale_factor = 2048;
			break;
		default:
			new_val |= 0x00;
			accel_scale_factor = 16384; // 2g
	}

	this->SetScaleFactor(accel_scale_factor);

	_spi->write_single_reg(_activeDevice, ub_2, B2_ACCEL_CONFIG, new_val);
}

void SensorGyro::SetRange(gyro_full_scale full_scale)
{
	uint8_t new_val = _spi->read_single_reg(_activeDevice, ub_2, B2_GYRO_CONFIG_1);
	new_val &= 0xF9;	/// mask: xxxx x00x
	float gyro_scale_factor = 1;
	switch(full_scale)
	{
		case GYRO_FS_250dps:
			new_val |= 0x00;
			gyro_scale_factor = 131.0;
			break;
		case GYRO_FS_500dps:
			new_val |= 0x02;
			gyro_scale_factor = 65.5;
			break;
		case GYRO_FS_1000dps:
			new_val |= 0x04;
			gyro_scale_factor = 32.8;
			break;
		case GYRO_FS_2000dps:
			new_val |= 0x06;
			gyro_scale_factor = 16.4;
			break;
		default:
			new_val |= 0x00;
			gyro_scale_factor = 131.0;
	}

	this->SetScaleFactor(gyro_scale_factor);

	_spi->write_single_reg(_activeDevice, ub_2, B2_GYRO_CONFIG_1, new_val);
}

uint8_t SensorGyro::GetLowPassFilter(void)
{
	uint8_t new_val = _spi->read_single_reg(_activeDevice, ub_2, B2_GYRO_CONFIG_1);
	if( (new_val & 0x01) == 0x01)
	{
		return (new_val>>3) & 0x07;
	}
	return 0xFE;

}

void SensorGyro::SetLowPassFilter(gyro_dlp_cfg config)
{
	uint8_t new_val = _spi->read_single_reg(_activeDevice, ub_2, B2_GYRO_CONFIG_1);
	uint8_t gyro_fchoice = 1;
	uint8_t config_val = (uint8_t)config;
	uint8_t mask = 0xC7;
	if (config == GYRO_low_pass_OFF){
		gyro_fchoice = 0;
		config_val = 0;
		mask = 0xC6;
	}

	new_val = (new_val & mask) | (config_val<<3) | gyro_fchoice;	// mask Gyro range settings

	_spi->write_single_reg(_activeDevice, ub_2, B2_GYRO_CONFIG_1, new_val);
}


uint8_t SensorAccel::CheckLowPassInputValue(uint8_t value)
{
	uint8_t valid = 0;

	    switch(value) {
	        case ACCEL_lpf_246Hz:
	        case ACCEL_lpf_114_4Hz:
	        case ACCEL_lpf_050_4Hz:
	        case ACCEL_lpf_023_9Hz:
	        case ACCEL_lpf_011_5Hz:
	        case ACCEL_lpf_005_7Hz:
	        case ACCEL_lpf_473Hz:
	        case ACCEL_lpf_OFF:
	            valid = 1;
	            break;
	        default:
	        	valid = 0;
	    };

	    return valid;
}

uint8_t SensorGyro::CheckLowPassInputValue(uint8_t value)
{
	uint8_t valid = 0;

	    switch(value) {
	        case GYRO_low_pass_OFF:
	        case GYRO_lpf_005_7Hz:
	        case GYRO_lpf_011_6Hz:
	        case GYRO_lpf_023_9Hz:
	        case GYRO_lpf_051_2Hz:
	        case GYRO_lpf_119_5Hz:
	        case GYRO_lpf_151_8Hz:
	        case GYRO_lpf_196_6Hz:
	        case GYRO_lpf_361_4Hz:
	            valid = 1;
	            break;
	        default:
	        	valid = 0;
	    };

	    return valid;
}


/**
 * @brief Configure Low-pass filter:
 * @param config:
 */
void SensorAccel::SetLowPassFilter(accel_dlp_cfg config)
{
	uint8_t new_val = _spi->read_single_reg(_activeDevice, ub_2, B2_ACCEL_CONFIG);
	uint8_t accel_fchoice = 1;
	uint8_t mask = 0xC7;
//	uint8_t config_val = (uint8_t)config;
	if (config == ACCEL_lpf_OFF){
		accel_fchoice = 0;
//		config_val = 0;
		mask = 0xC6;
	}

	new_val = (new_val & mask) | (config<<3) | accel_fchoice;	// mask Accel range settings

	_spi->write_single_reg(_activeDevice, ub_2, B2_ACCEL_CONFIG, new_val);
}

uint8_t SensorAccel::GetLowPassFilter(void)
{
	uint8_t new_val = _spi->read_single_reg(_activeDevice, ub_2, B2_ACCEL_CONFIG);
	if((new_val & 0x01) == 0){
		//LPF is OFF
		return 0xFE;
	}

	new_val = (new_val>>3) & 0x07;
	if(new_val == 0){ // cofig value 0 and 1 has same LPF frequency: 246Hz
		new_val = 1;
	}

	return new_val;
}

uint8_t SensorGyro::SetSampleRate(gyro_samplerate divider)
{
	_spi->write_single_reg(_activeDevice, ub_2, B2_GYRO_SMPLRT_DIV, divider);
	return 1;
}

uint8_t SensorGyro::GetSampleRate(void)
{
	uint8_t value = _spi->read_single_reg(_activeDevice, ub_2, B2_GYRO_SMPLRT_DIV);
	return value;
}

uint8_t SensorAccel::SetSampleRate(accel_samplerate smplrt)
{
	if(smplrt < 0 || smplrt >= 15){
		return 0;
	}

	uint16_t sr = ACCEL_SampleRate_Table[smplrt];
	uint8_t data[2] = {(uint8_t)((sr>> 8) & 0x0F), (uint8_t)(sr & 0xFF)}; // MSB, LSB

	_spi->write_multiple_reg(_activeDevice, ub_2, B2_ACCEL_SMPLRT_DIV_1, data, 2);

	return 1;
}

uint8_t SensorAccel::GetSampleRate(void)
{
	uint8_t d1 =  _spi->read_single_reg(_activeDevice, ub_2, B2_ACCEL_SMPLRT_DIV_1);
	uint8_t d2 =  _spi->read_single_reg(_activeDevice, ub_2, B2_ACCEL_SMPLRT_DIV_2);
	uint16_t value = (d1<<8 | d2);
	uint8_t sr_index = 0;
	for(uint8_t i=0 ; i < 15 ; i++){
		if(value == ACCEL_SampleRate_Table[i]){
			sr_index = i;
		}
	}
	return sr_index;
}


uint8_t SensorAccel::GetRange(void){
	uint8_t new_val = _spi->read_single_reg(_activeDevice, ub_2, B2_ACCEL_CONFIG);
	return (new_val >> 1) & 0x03;
}

uint8_t SensorGyro::GetRange(void){
	uint8_t new_val = _spi->read_single_reg(_activeDevice, ub_2, B2_GYRO_CONFIG_1);
	return (new_val >> 1) & 0x03;
}

void SensorGyro::Calibrate(void)
{
	axisesI temp;
	int32_t gyro_bias[3] = {0};

	for(int i = 0; i < 100; i++)
	{
		while(this->GetDataStatus() == 0);
		this->Read(&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;
		HAL_Delay(2);
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	this->offset_x = gyro_bias[0];
	this->offset_y = gyro_bias[1];
	this->offset_z = gyro_bias[2];
	this->SaveOffset();
}

void SensorGyro::SaveOffset(void)
{
	if((this->offset_x+this->offset_y+this->offset_z)==0)
	{
		return;
	}
	uint8_t gyro_offset[6] = {0};
	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	gyro_offset[0] = (-this->offset_x / 4  >> 8) & 0xFF;
	gyro_offset[1] = (-this->offset_x / 4)       & 0xFF;
	gyro_offset[2] = (-this->offset_y / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-this->offset_y / 4)       & 0xFF;
	gyro_offset[4] = (-this->offset_z / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-this->offset_z / 4)       & 0xFF;

	_spi->write_multiple_reg(_activeDevice, ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}

void SensorAccel::Calibrate()
{
	axisesI temp;
	uint8_t* temp2;
	uint8_t* temp3;
	uint8_t* temp4;

	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};

	for(int q = 0; q < 3; q++)
	{
		for(int i = 0; i < 100; i++)
		{
			while(this->GetDataStatus() == 0);
			this->_read_without_scale_factor(&temp);
			accel_bias[0] += temp.x;
			accel_bias[1] += temp.y;
			accel_bias[2] += temp.z;
			HAL_Delay(2);
		}
		HAL_Delay(5);
	}

	accel_bias[0] /= 300;
	accel_bias[1] /= 300;
	accel_bias[2] /= 300;

	temp2 = _spi->read_multiple_reg(_activeDevice, ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);

	temp3 = _spi->read_multiple_reg(_activeDevice, ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);

	temp4 = _spi->read_multiple_reg(_activeDevice, ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);

	accel_bias_reg[0] -= (accel_bias[0]/8 );
	accel_bias_reg[1] -= (accel_bias[1]/8 );
	accel_bias_reg[2] -= (accel_bias[2]/8 );	// (accel_bias[2]/8)

	this->offset_x = accel_bias_reg[0];
	this->offset_y = accel_bias_reg[1];
	this->offset_z = accel_bias_reg[2];

	this->SaveOffset();
}

void SensorAccel::SaveOffset(void)
{
	if((this->offset_x+this->offset_y+this->offset_z)==0)
	{
		return;
	}
	uint8_t accel_offset[6] = {0};
	accel_offset[0] = (this->offset_x >> 8) & 0xFF;
  	accel_offset[1] = (this->offset_x)      & 0xFE;

	accel_offset[2] = (this->offset_y >> 8) & 0xFF;
  	accel_offset[3] = (this->offset_y)      & 0xFE;

	accel_offset[4] = (this->offset_z >> 8) & 0xFF;
	accel_offset[5] = (this->offset_z)      & 0xFE;


	_spi->write_multiple_reg(_activeDevice, ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	_spi->write_multiple_reg(_activeDevice, ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	_spi->write_multiple_reg(_activeDevice, ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}
