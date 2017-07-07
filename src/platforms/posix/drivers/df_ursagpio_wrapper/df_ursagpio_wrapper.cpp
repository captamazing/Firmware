/****************************************************************************
 *
 * Copyright (c) 2017 LAOSAAC
 *
 ****************************************************************************/

/**
 * @file df_ursagpio_wrapper.cpp
 * Lightweight wrapper to access the timed GPIO driver
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
#include <uORB/topics/input_rc.h>

#include <board_config.h>

#include <ursa_gpio/ursa_gpio.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_ursagpio_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;

#define LINUX_RC_INPUT_NUM_CHANNELS 16


class DfGPIOWrapper : public GPIO_TIMED
{
public:
	DfGPIOWrapper();
	~DfGPIOWrapper();


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

};

DfGPIOWrapper::DfGPIOWrapper() :
	GPIO_TIMED(GPIO_DEV_PATH)
{
}

DfGPIOWrapper::~DfGPIOWrapper()
{
}

int DfGPIOWrapper::start()
{
	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("GPIO init fail: %d", ret);
		return ret;
	}

	ret = GPIO_TIMED::start();

	if (ret != 0) {
		PX4_ERR("GPIO start fail: %d", ret);
		return ret;
	}

	// Test open the RC
	DevHandle h;
	DevMgr::getHandle(GPIO_DEV_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the GPIO at: %s (%d)",
			   	GPIO_DEV_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	PX4_INFO("Successfully started PIGPIO library");

	return 0;
}

int DfGPIOWrapper::stop()
{
	/* Stop sensor. */
	int ret = GPIO_TIMED::stop();
	if (ret != 0) {
		PX4_ERR("GPIO stop fail: %d", ret);
		return ret;
	}

	PX4_INFO("Stopping GPIO Wrapper and terminating PIGPIO library...");

	return 0;
}


namespace df_ursagpio_wrapper
{

DfGPIOWrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
	g_dev = new DfGPIOWrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfGPIOWrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfGPIOWrapper start failed");
		return ret;
	}

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("DfGPIOWrapper tried to stop but driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("DfGPIOWrapper - driver could not be stopped");
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
	PX4_WARN("Usage: df_ursagpio_wrapper 'start', 'info', 'stop'");
}

} // namespace df_ursagpio_wrapper


int
df_ursagpio_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		df_ursagpio_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_ursagpio_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_ursagpio_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_ursagpio_wrapper::info();
	}

	else {
		df_ursagpio_wrapper::usage();
		return 1;
	}

	return ret;
}
