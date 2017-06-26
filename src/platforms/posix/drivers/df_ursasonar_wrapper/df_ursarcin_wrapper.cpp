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

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>

#include <board_config.h>

#include <ursa_rcin/ursa_rcin.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_ursarcin_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;

#define LINUX_RC_INPUT_NUM_CHANNELS 16


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

	int _process_rc_pulse(uint16_t width_usec);
	int _publish();
	struct input_rc_s _rcdata;
	//uint16_t _pulse_capt[LINUX_RC_INPUT_NUM_CHANNELS];
	uint8_t  _num_channels;
	int _channel_counter;
	orb_advert_t _rc_topic;

};

DfRCINWrapper::DfRCINWrapper() :
	RC_IN(RC_DEV_PATH),
	_channel_counter(0),
	_rc_topic(nullptr)
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

	// Test open the RC
	DevHandle h;
	DevMgr::getHandle(RC_DEV_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the RC at: %s (%d)",
			   	RC_DEV_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

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

int DfRCINWrapper::_process_rc_pulse(uint16_t width_usec){
	if (width_usec >= 2700) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0 and publish to uORB
        if (_channel_counter >= 0) {
            _rcdata.channel_count = _channel_counter;
            _publish();
        }
        _channel_counter = 0;
        return 0;
    }
    if (_channel_counter == -1) {
        // we are not synchronised
        return 0;
    }

    /*
      we limit inputs to between 700usec and 2300usec. This allows us
      to decode SBUS on the same pin, as SBUS will have a maximum
      pulse width of 100usec
     */
    if (width_usec > 700 && width_usec < 2300) {
        // take a reading for the current channel
        // buffer these
        _rcdata.values[_channel_counter] = width_usec;

        // move to next channel
        _channel_counter++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    if (_channel_counter >= LINUX_RC_INPUT_NUM_CHANNELS) {
        _rcdata.channel_count = _channel_counter;
        _channel_counter = -1;
    }

    return 0;
}

int DfRCINWrapper::_publish(){

	uint64_t ts = hrt_absolute_time();
	_rcdata.timestamp = ts;
	_rcdata.timestamp_last_signal = ts;
	_rcdata.rssi = 100;
	_rcdata.rc_lost_frame_count = 0;
	_rcdata.rc_total_frame_count = 1;
	_rcdata.rc_ppm_frame_length = 100;
	_rcdata.rc_failsafe = false;
	_rcdata.rc_lost = false;
	_rcdata.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

	if (_rc_topic == nullptr) {
		_rc_topic = orb_advertise(ORB_ID(input_rc), &_rcdata);

	} else {
		orb_publish(ORB_ID(input_rc), _rc_topic, &_rcdata);
	}
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
