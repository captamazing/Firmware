/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC
 *
 ****************************************************************************/

//Driver framework driver - PPM RC IN using DMA

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
	SyncObj 			m_synchronize;

	// returns 0 on success, -errno on failure
	int rc_in_init();
};

}; // namespace DriverFramework
