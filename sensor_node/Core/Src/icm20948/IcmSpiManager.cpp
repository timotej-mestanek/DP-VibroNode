/*
* @file IcmSpiManager.cpp
*  Created: Juraj Dudak  (C++ version)
*  Date: 30.12.2022
*/

#include "IcmSpiManager.h"

IcmSpiManager::IcmSpiManager(SPI_HandleTypeDef *hspi)
{
	_numDevices = 0;
	_activeDevice = -1;
	_spi = hspi;
	_active_bank = ub_NONE;
}


uint8_t IcmSpiManager::addSlave(McuPin_typeDef *pin)
{
	_pinCS[_numDevices] = pin;
	_numDevices++;
	return _numDevices-1;
}

IcmSpiManager* IcmSpiManager::getInstance(){
	return this;
}

inline void IcmSpiManager::cs_low(uint8_t index)
{
	_pinCS[index]->port->BRR = (uint32_t)_pinCS[index]->pin;
}


inline void IcmSpiManager::cs_high(uint8_t index)
{
	_pinCS[index]->port->BSRR = _pinCS[index]->pin;
}



void IcmSpiManager::select_user_bank(uint8_t sensorNum, userbank ub)
{
	// toto je v poriadku iba pri pouziti jedneho senzora
	// treba osetrit situaciu, ked je viacero senzorov.
	// zmena banky sa musi robit pre kazdy senzor

//	if(ub == _active_bank){
//		return;
//	}

	_active_bank = ub;

	uint8_t write_reg[2];
	write_reg[0] = WRITE | REG_BANK_SEL;
	write_reg[1] = ub;

	cs_low(sensorNum);
	HAL_SPI_Transmit(_spi, write_reg, 2, 10);
	cs_high(sensorNum);
}

void IcmSpiManager::takeBus(uint8_t index)
{
	while(_activeDevice >= 0){
		; // wait
	}
	_activeDevice = index;
}

void IcmSpiManager::releaseBus()
{
	_activeDevice = -1;
}

uint8_t* IcmSpiManager::read_multiple_reg(uint8_t sensorNum, userbank ub, uint8_t reg, uint8_t len)
{
	uint8_t read_reg = READ | reg;
	this->select_user_bank(sensorNum, ub);

	cs_low(sensorNum);
	HAL_SPI_Transmit(this->_spi, &read_reg, 1, 1000);
	HAL_SPI_Receive(this->_spi, this->_reg_val, len, 1000);
	cs_high(sensorNum);

	return this->_reg_val;
}


uint8_t IcmSpiManager::read_single_reg(uint8_t sensorNum, userbank ub, uint8_t reg)
{
	uint8_t read_reg = READ | reg;
	uint8_t reg_val;
	select_user_bank(sensorNum, ub);

	cs_low(sensorNum);
	HAL_SPI_Transmit(_spi, &read_reg, 1, 1000);
	HAL_SPI_Receive(_spi, &reg_val, 1, 1000);
	cs_high(sensorNum);

	return reg_val;
}


void IcmSpiManager::write_single_reg(uint8_t sensorNum, userbank ub, uint8_t reg, uint8_t val)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | reg;
	write_reg[1] = val;

	select_user_bank(sensorNum, ub);

	cs_low(sensorNum);
	HAL_SPI_Transmit(_spi, write_reg, 2, 1000);
	cs_high(sensorNum);
}


void IcmSpiManager::write_multiple_reg(uint8_t sensorNum, userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
	uint8_t write_reg = WRITE | reg;
	select_user_bank(sensorNum, ub);

	cs_low(sensorNum);
	HAL_SPI_Transmit(_spi, &write_reg, 1, 1000);
	HAL_SPI_Transmit(_spi, val, len, 1000);
	cs_high(sensorNum);
}


uint8_t IcmSpiManager::read_single_external_reg(uint8_t sensorNum, uint8_t reg)
{

	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_REG, reg);
	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_CTRL, 0x81);

	HAL_Delay(1);
	return this->read_single_reg(sensorNum, ub_0, B0_EXT_SLV_SENS_DATA_00);
}

void IcmSpiManager::write_single_external_reg(uint8_t sensorNum, uint8_t reg, uint8_t val)
{
	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_REG, reg);
	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_DO, val);
	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_CTRL, 0x81);
}

uint8_t* IcmSpiManager::read_multiple_external_reg(uint8_t sensorNum, uint8_t reg, uint8_t len)
{
	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_REG, reg);
	this->write_single_reg(sensorNum, ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);

	HAL_Delay(1);
	return this->read_multiple_reg(sensorNum, ub_0, B0_EXT_SLV_SENS_DATA_00, len);
}



