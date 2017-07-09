/****************************************************************************
 *
 * Copyright (c) 2017 LAOSAAC
 *
 ****************************************************************************/

/**
 * @file ursa_rcin.cpp
 * Lightweight driver to access the RC rx via DMA timed GPIO
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


extern "C" { __EXPORT int ursa_rcin_main(int argc, char *argv[]); }

using namespace DriverFramework;

#define LINUX_RC_INPUT_NUM_CHANNELS 16
#define RCIN_RPI_GPIO_PIN 4


class UrsaRCINPub
{
public:
    UrsaRCINPub();
    ~UrsaRCINPub();


    /**
     * Setup callbacks etc
     *
     * @return 0 on success
     */
    int init(int gpio);

    /**
     * Stop automatic measurement.
     *
     * @return 0 on success
     */
    int stop();

    void rc_level_change(int gpio, int val, uint32_t tick);
    gpio_write_t callbackStruct;

private:
    int _publish();
    void _process_rc_pulse(uint32_t width_usec);
    struct input_rc_s _rcdata;
    uint8_t  _num_channels;
    int _channel_counter;
    uint32_t _startframe;
    orb_advert_t _rc_topic;
};

UrsaRCINPub::UrsaRCINPub() :
    _channel_counter(0),
    _startframe(0),
    _rc_topic(nullptr)
{
}

UrsaRCINPub::~UrsaRCINPub()
{
}

int UrsaRCINPub::init(int gpio)
{
    // Get a handle to our timed GPIO DMA magic
    DevHandle h;
    DevMgr::getHandle(GPIO_DEV_PATH, h); 

    if (!h.isValid()) {
        PX4_ERR("Failed to get handle to timed GPIO device");
        return -1;
    }

    // Setup the callback struct which we'll pass to the timed GPIO device
    callbackStruct.callback=std::bind(&UrsaRCINPub::rc_level_change, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    callbackStruct.gpio=gpio;
    callbackStruct.type=GPIO_CALLBACK;
    h.write((void*)&callbackStruct,sizeof(gpio_write_t));

    // Don't need this handle anymore
    DevMgr::releaseHandle(h);

    return 0;
}

int UrsaRCINPub::stop()
{
    /* Stop sensor. */
    // TODO add interface to release callbacks from GPIO

    PX4_INFO("Stopping RC Publisher...");

    return 0;
}

void UrsaRCINPub::rc_level_change(int gpio, int val, uint32_t tick){
    if (val==1){
        uint32_t time=tick-_startframe;
        _process_rc_pulse(time);
        _startframe=tick;
        return;
    }
}

void UrsaRCINPub::_process_rc_pulse(uint32_t width_usec){
    if (width_usec >= 2700) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0 and publish to uORB
        if (_channel_counter >= 0) {
            _rcdata.channel_count = _channel_counter;
            _publish();
        }
        _channel_counter = 0;
        return;
    }
    if (_channel_counter == -1) {
        // we are not synchronised
        return;
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

    return;
}

int UrsaRCINPub::_publish(){

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


namespace ursa_rcin
{
// Singleton since we only want/need one
UrsaRCINPub *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
    g_dev = new UrsaRCINPub();

    if (g_dev == nullptr) {
        PX4_ERR("failed instantiating UrsaRCINPub object");
        return -1;
    }

    int ret = g_dev->init(RCIN_RPI_GPIO_PIN);

    if (ret != 0) {
        PX4_ERR("UrsaRCINPub init failed");
        return ret;
    }

    return 0;
}

int stop()
{
    if (g_dev == nullptr) {
        PX4_ERR("UrsaRCINPub tried to stop but not running");
        return 1;
    }

    int ret = g_dev->stop();

    if (ret != 0) {
        PX4_ERR("UrsaRCINPub could not be stopped");
        return ret;
    }

    delete g_dev;
    g_dev = nullptr;
    return 0;
}

/**
 * Print a little info about the driver.
 */
int info()
{
    if (g_dev == nullptr) {
        PX4_ERR("driver not running");
        return 1;
    }

    PX4_DEBUG("state @ %p", g_dev);

    return 0;
}

void usage()
{
    PX4_WARN("Usage: ursa_rcin 'start', 'info', 'stop'");
}

} // namespace ursa_rcin


int ursa_rcin_main(int argc, char *argv[])
{
    int ret = 0;
    int myoptind = 1;

    if (argc <= 1) {
        ursa_rcin::usage();
        return 1;
    }

    const char *verb = argv[myoptind];


    if (!strcmp(verb, "start")) {
        ret = ursa_rcin::start();
    }

    else if (!strcmp(verb, "stop")) {
        ret = ursa_rcin::stop();
    }

    else if (!strcmp(verb, "info")) {
        ret = ursa_rcin::info();
    }

    else {
        ursa_rcin::usage();
        return 1;
    }

    return ret;
}
