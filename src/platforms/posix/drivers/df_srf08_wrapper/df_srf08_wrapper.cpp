/****************************************************************************
 *
 * Copyright (c) 2017 LAOSAAC

 ****************************************************************************/

/**
 * @file df_srf08_wrapper.cpp
 * Lightweight driver to access the SRF08 of the DriverFramework.
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

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

#include <srf08/SRF08.hpp>
#include <DevMgr.hpp>

#define MAX_SRF08_RANGE 4
#define MIN_SRF08_RANGE 0.02


extern "C" { __EXPORT int df_srf08_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfSRF08Wrapper : public SRF08
{
public:
	DfSRF08Wrapper();
	~DfSRF08Wrapper();


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
	void _publish(double range);
	orb_advert_t		_sonar_topic;
	struct distance_sensor_s _sonar_data;
};

DfSRF08Wrapper::DfSRF08Wrapper() :
	SRF08(),
	_sonar_topic(nullptr)
{
}

DfSRF08Wrapper::~DfSRF08Wrapper()
{
}

int DfSRF08Wrapper::start()
{
	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("SRF08 init fail: %d", ret);
		return ret;
	}

	ret = SRF08::start();

	if (ret != 0) {
		PX4_ERR("SRF08 start fail: %d", ret);
		return ret;
	}

	_sonar_data.min_distance = MIN_SRF08_RANGE;
	_sonar_data.max_distance = MAX_SRF08_RANGE;
	_sonar_data.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	_sonar_data.id = 0;

	return 0;
}

int DfSRF08Wrapper::stop()
{
	/* Stop sensor. */
	int ret = SRF08::stop();

	if (ret != 0) {
		PX4_ERR("SRF08 stop fail: %d", ret);
		return ret;
	}

	return 0;
}

void DfSRF08Wrapper::_publish(double range)
{
	uint64_t ts = hrt_absolute_time();
	_sonar_data.timestamp = ts;
	//Smooth out major drops which can be experienced with dodgy measurements
	// if (range<_lastRange-0.2){
	// 	//range=_lastRange-0.05;
	// }
	// _lastRange=range;
	_sonar_data.current_distance=range;
	_sonar_data.covariance=0.0f;

	if (_sonar_topic == nullptr) {
		_sonar_topic = orb_advertise(ORB_ID(distance_sensor), &_sonar_data);
	} else {
		orb_publish(ORB_ID(distance_sensor), _sonar_topic, &_sonar_data);
	}
};


namespace df_srf08_wrapper
{

DfSRF08Wrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
	g_dev = new DfSRF08Wrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfSRF08Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfSRF08Wrapper start failed");
		return ret;
	}

	// Open the IMU sensor
	DevHandle h;
	DevMgr::getHandle(SRF08_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the sonar rangefinder at: %s (%d)",
			    SRF08_DEVICE_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
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
	PX4_WARN("Usage: df_SRF08_wrapper 'start', 'info', 'stop'");
}

} // namespace df_SRF08_wrapper


int df_srf08_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		df_srf08_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_srf08_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_srf08_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_srf08_wrapper::info();
	}

	else {
		df_srf08_wrapper::usage();
		return 1;
	}

	return ret;
}
