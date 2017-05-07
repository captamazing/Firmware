/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC
 *
 ****************************************************************************/

#pragma once

#include "DevObj.hpp"

namespace ursa_rc_in
{

#define RC_MEASURE_INTERVAL_US 20000 //50Hz
#define RC_MEM_PATH "/dev/mem"
#define RC_PAGEMAP "/proc/self/pagemap"
#define RC_CLASS_PATH "/dev/rc_in"
#define DRV_DF_DEVTYPE_RCIN 53

class RC_IN : public DevObj
{
public:
	RC_IN(const char *device_path) :
		DevObj("URSA_RC_IN", RC_MEM_PATH, RC_CLASS_PATH, RC_MEASURE_INTERVAL_US)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_ADS1115;
	}

	virtual ~RC_IN() = default;

	// @return 0 on success, -errno on failure
	int start();

	// @return 0 on success, -errno on failure
	int stop();

protected:
	void _measure();
	int devRead(void *buf, size_t count);
	struct adc_msg_s _buf_adc[ADS1115_CHANNEL_COUNT]; 
	uint32_t m_temperature_from_sensor;
	SyncObj 			m_synchronize;

	// Request to convert voltage or current data
	int _configWrite(uint16_t cmd);
	// Read out the requested sensor data
	int _collect(int32_t &raw);


	/* Channel 2 is voltage, 3 is current */
	static const uint16_t mux_table[ADS1115_CHANNEL_COUNT];

	// returns 0 on success, -errno on failure
	int ads1115_init();

	// Send reset to device
	int m_measure_channel;
	int m_gain;
};

}; // namespace DriverFramework
