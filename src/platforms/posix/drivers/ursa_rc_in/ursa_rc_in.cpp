/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC
 *
 ****************************************************************************/

#include <string.h>
#include "DriverFramework.hpp"
#include "ADS1115.hpp"

using namespace DriverFramework;

#define POW2(_x) ((_x) * (_x))

// ADC driver comment

const uint16_t ADS1115::mux_table[ADS1115_CHANNEL_COUNT]= {
	    ADS1115_MUX_P1_N3,
	    ADS1115_MUX_P2_N3,
	    ADS1115_MUX_P0_NG,
	    ADS1115_MUX_P1_NG,
	    ADS1115_MUX_P2_NG,
	    ADS1115_MUX_P3_NG
	};

int ADS1115::ads1115_init()
{
	int result = _setSlaveConfig(ADS1115_SLAVE_ADDRESS,
				     ADS1115_BUS_FREQUENCY_IN_KHZ,
				     ADS1115_TRANSFER_TIMEOUT_IN_USECS);

	if (result < 0) {
		DF_LOG_ERR("could not set slave config");
	}

	/* Reset sensor and load calibration data into internal register */
	int32_t raw;
	uint16_t req = (ADS1115_OS_ACTIVE | m_gain | mux_table[ADC_BATTERY_VOLTAGE_CHANNEL] |
                         ADS1115_MODE_SINGLESHOT | ADS1115_COMP_QUE_DISABLE |
                         ADS1115_RATE_250);
	_configWrite(req);
	result = _collect(raw);

	if (result < 0) {
		DF_LOG_ERR("error: unable to communicate with the ADS1115 ADC");
		return -EIO;
	}

	return 0;
}

int ADS1115::start()
{
	int result = 0;
	result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Initialize the pressure sensor.*/
	result = ads1115_init();

	if (result != 0) {
		DF_LOG_ERR("error: pressure sensor initialization failed, sensor read thread not started");
		goto exit;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

exit:

	return result;
}

int ADS1115::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}

int ADS1115::_configWrite(uint16_t cmd)
{
	int ret;
	_retries = 0;
	uint8_t in_buffer[2];
	in_buffer[0] = (uint8_t)(cmd >> 8);
	in_buffer[1] = (uint8_t)(cmd & 0xFF);
	ret = _writeReg(ADS1115_ADC_CONFIG_ADDR  , in_buffer, 2);
	if (ret < 0) {
		DF_LOG_ERR("error: ADC conversion request failed");
	}

	return ret;
}

int ADS1115::_collect(int32_t &raw)
{
	int ret;

	union {
		uint8_t b[4];
		int32_t w;
	} cvt {};

	uint8_t buf[2];
	_retries = 0;

	ret = _writeReg(ADS1115_ADC_READ_ADDR  , nullptr, 0); //Select conversion address
	if (ret < 0) {
		DF_LOG_ERR("error: Could not select ADC conversion address");
	} else{
		ret = _simple_read(&buf[0], 2); //Select conversion address
		if (ret < 0) {
			DF_LOG_ERR("error: Could not read data");
		}
	}
	if (ret < 0) {
		raw = 0;
		return -1;
	}

	cvt.b[0] = buf[1];
	cvt.b[1] = buf[0];
	cvt.b[2] = 0;
	cvt.b[3] = 0;
	raw = cvt.w;

	return 0;
}

void ADS1115::_measure()
{
	int32_t raw;
	if (_collect(raw) < 0) DF_LOG_ERR("error: ADC read failed");
	_buf_adc[m_measure_channel].am_data = raw;

	/* Code specifically for battery measurement on URSA */
	m_measure_channel++;
	if (m_measure_channel>ADC_BATTERY_CURRENT_CHANNEL) m_measure_channel=ADC_BATTERY_VOLTAGE_CHANNEL;

	uint16_t req = (ADS1115_OS_ACTIVE | m_gain | mux_table[m_measure_channel] |
                         ADS1115_MODE_SINGLESHOT | ADS1115_COMP_QUE_DISABLE |
                         ADS1115_RATE_250);
	if (_configWrite(req)<0){
		DF_LOG_ERR("error: Starting ADC conversion failed");
	};
}

int ADS1115::devRead(void *buf, size_t count)
{	
	if (count > ADS1115_CHANNEL_COUNT * sizeof(adc_msg_s)){
		count = ADS1115_CHANNEL_COUNT * sizeof(adc_msg_s);
	}

	/* block interrupts while copying samples to avoid racing with an update */
	m_synchronize.lock();
	memcpy(buf, _buf_adc, sizeof(_buf_adc));
	m_synchronize.unlock();
	return count;
}