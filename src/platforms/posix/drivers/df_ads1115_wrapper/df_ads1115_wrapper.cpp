/****************************************************************************
 *
 * Copyright (c) 2017 LAOSAAC
 *
 ****************************************************************************/

/**
 * @file df_ads1115_wrapper.cpp
 * Lightweight driver to access the ADS1115 and publish output as battery status
 */

#include <px4_config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <px4_getopt.h>
#include <errno.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/battery_status.h>

#include <board_config.h>

#include <ads1115/ADS1115.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_ads1115_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfADS1115Wrapper : public ADS1115
{
public:
	DfADS1115Wrapper();
	~DfADS1115Wrapper();


	/**
	 * Start automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		start();

	/**
	 * Stop automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		stop();

private:
	int _publish(struct adc_sensor_data &data);

	orb_advert_t		_batt_topic;

	int			_batt_orb_class_instance;

	perf_counter_t		_batt_sample_perf;

};

DfADS1115Wrapper::DfADS1115Wrapper() :
	ADS1115(ADC_DEVICE_PATH),
	_batt_topic(nullptr),
	_batt_orb_class_instance(-1),
	_batt_sample_perf(perf_alloc(PC_ELAPSED, "df_batt_read"))
{
}

DfADS1115Wrapper::~DfADS1115Wrapper()
{
	perf_free(_batt_sample_perf);
}

int DfADS1115Wrapper::start()
{
	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("ADS1115 init fail: %d", ret);
		return ret;
	}

	ret = ADS1115::start();

	if (ret != 0) {
		PX4_ERR("ADS1115 start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfADS1115Wrapper::stop()
{
	/* Stop sensor. */
	int ret = ADS1115::stop();
	if (ret != 0) {
		PX4_ERR("ADC stop fail: %d", ret);
		return ret;
	}

	PX4_INFO("Stopping ADS1115 Wrapper...");

	return 0;
}


namespace df_ads1115_wrapper
{

DfADS1115Wrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
	g_dev = new DfADS1115Wrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfADS1115Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfADS1115Wrapper start failed");
		return ret;
	}

	// Test open the ADC
	DevHandle h;
	DevMgr::getHandle(ADC_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the ADC at: %s (%d)",
			   	ADC_DEVICE_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("DfADS1115Wrapper tried to stop but driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("DfADS1115Wrapper - driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	return 0;
}

void
usage()
{
	PX4_WARN("Usage: df_ads1115_wrapper 'start', 'info', 'stop'");
}

} // namespace df_ads1115_wrapper


int
df_ads1115_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		df_ads1115_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_ads1115_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_ads1115_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_ads1115_wrapper::info();
	}

	else {
		df_ads1115_wrapper::usage();
		return 1;
	}

	return ret;
}
