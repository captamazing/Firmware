/****************************************************************************
 *
 * Copyright (c) 2017 LAOSAAC
 *
 ****************************************************************************/

/**
 * @file df_ursarcin_wrapper.cpp
 * Lightweight driver to access the RC rx PPM driver
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

#include <ursa_rcin/ursa_rcin.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_ursarcin_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfRCINWrapper : public RC_IN
{
public:
	DfRCINWrapper();
	~DfRCINWrapper();


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
};

DfRCINWrapper::DfRCINWrapper() :
	RC_IN(RC_DEV_PATH)
{
}

DfRCINWrapper::~DfRCINWrapper()
{
}

int DfRCINWrapper::start()
{
	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("RC init fail: %d", ret);
		return ret;
	}

	ret = RC_IN::start();

	if (ret != 0) {
		PX4_ERR("RC start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfRCINWrapper::stop()
{
	/* Stop sensor. */
	int ret = RC_IN::stop();
	if (ret != 0) {
		PX4_ERR("RC stop fail: %d", ret);
		return ret;
	}

	PX4_INFO("Stopping RC Wrapper...");

	return 0;
}


namespace df_ursarcin_wrapper
{

DfRCINWrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
	g_dev = new DfRCINWrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfRCINWrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfRCINWrapper start failed");
		return ret;
	}

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("dfURSARCINWrapper tried to stop but driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("dfURSARCINWrapper - driver could not be stopped");
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
	PX4_WARN("Usage: df_ursarcin_wrapper 'start', 'info', 'stop'");
}

} // namespace df_ursarcin_wrapper


int
df_ursarcin_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		df_ursarcin_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_ursarcin_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_ursarcin_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_ursarcin_wrapper::info();
	}

	else {
		df_ursarcin_wrapper::usage();
		return 1;
	}

	return ret;
}
