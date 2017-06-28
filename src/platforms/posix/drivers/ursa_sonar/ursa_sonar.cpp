/****************************************************************************
 *
 * Copyright (c) 2017 LAOSAAC
 *
 ****************************************************************************/

/**
 * @file ursa_sonar.cpp
 * Lightweight driver to read sonar measurements via DMA timed GPIO and FS writes
 */

#include <px4_config.h>
#include <px4_workqueue.h>

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

#include <ursa_gpio/ursa_gpio.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int ursa_sonar_main(int argc, char *argv[]); }

using namespace DriverFramework;

#define MAX_STR 35
#define URSA_SONAR_PULSE_INTERVAL_US 100000

class UrsaSonarPub
{
public:
    UrsaSonarPub();
    ~UrsaSonarPub();


    /**
     * Setup callbacks etc
     *
     * @return 0 on success
     */
    int init(int gpio_Out, int gpio_In);

    /**
     * Stop automatic measurement.
     *
     * @return 0 on success
     */
    int stop();

    void process_pulse(uint32_t width_usec);
    gpio_callback_t callbackStruct;

    /* Trampoline for the work queue. */
    static void cycle_trampoline(void *arg);

private:
    void *_gpio_map;
    int _publish();
    void _cycle();
    struct distance_sensor_s _rcdata;
    uint8_t  _num_channels;
    int _channel_counter;
    orb_advert_t _rc_topic;
    int _gpio_Out;
    int _gpio_In;
    bool _shouldExit;
    struct work_s _work;
};

UrsaSonarPub::UrsaSonarPub() :
    _channel_counter(0),
    _rc_topic(nullptr)
{
}

UrsaSonarPub::~UrsaSonarPub()
{
}

int UrsaSonarPub::init(int gpio_Out, int gpio_In)
{
    _gpio_In=gpio_In;
    _gpio_Out=gpio_Out;
    _shouldExit=false;

    // Get a handle to our timed GPIO DMA magic
    DevHandle h;
    DevMgr::getHandle(GPIO_DEV_PATH, h); 

    if (!h.isValid()) {
        PX4_ERR("Failed to get handle to timed GPIO device");
        return -1;
    }

    // Setup the callback struct which we'll pass to the timed GPIO device
    callbackStruct.callback=std::bind(&UrsaSonarPub::process_pulse, this, std::placeholders::_1);
    callbackStruct.pin=gpio_In;
    callbackStruct.type=GPIO_CALLBACK_HIGHTIME;
    h.write((void*)&callbackStruct,sizeof(gpio_callback_t));

    // Don't need this handle anymore
    DevMgr::releaseHandle(h);

    // Export our GPIO output pin and get it ready for writing via the filesystem - also set input pin mode
    int gpio_fd;
    char buf[3];

    //Export pins
    if ((gpio_fd = open("/sys/class/gpio/export", O_RDWR | O_SYNC)) < 0) {
        PX4_ERR("Failed to open pin export file descriptor");
        return -1;
    }
    snprintf(buf, 3*sizeof(char), "%d", gpio_Out);
    write(gpio_fd, buf, 3);
    close(gpio_fd);

    // Direction is output
    char str_dir_file[MAX_STR];
    snprintf(str_dir_file, MAX_STR*sizeof(char), "/sys/class/gpio/gpio%d/direction", gpio_Out);
    if ((gpio_fd = open(str_dir_file, O_RDWR | O_SYNC)) < 0) {
        PX4_ERR("Failed to open pin direction file descriptor");
        return -1;
    }
    write(gpio_fd, "out", 4);
    close(gpio_fd);

    //Un-export input pin - just to be safe!
    if ((gpio_fd = open("/sys/class/gpio/unexport", O_RDWR | O_SYNC)) < 0) {
        PX4_ERR("Failed to open pin unexport file descriptor");
        return -1;
    }
    snprintf(buf, 3*sizeof(char), "%d", gpio_In);
    write(gpio_fd, buf, 3);
    close(gpio_fd);

    _cycle();

    return 0;
}

void UrsaSonarPub::cycle_trampoline(void *arg)
{
    UrsaSonarPub *dev = reinterpret_cast<UrsaSonarPub *>(arg);
    dev->_cycle();
}

void UrsaSonarPub::_cycle()
{
    // Send out a pulse on the GPIO
    int gpio_fd;
    char str_val_file[MAX_STR];
    snprintf(str_val_file, MAX_STR*sizeof(char), "/sys/class/gpio/gpio%d/value", _gpio_Out);
    if ((gpio_fd = open(str_val_file, O_RDWR | O_SYNC)) < 0) {
        PX4_ERR("Failed to open pin value file descriptor");
    }
    write(gpio_fd, "1", 2);
    write(gpio_fd, "0", 2);
    close(gpio_fd);

    if (!_shouldExit) {
        work_queue(HPWORK, &_work, (worker_t)&UrsaSonarPub::cycle_trampoline, this,
               USEC2TICK(URSA_SONAR_PULSE_INTERVAL_US));
    }
}

int UrsaSonarPub::stop()
{
    /* Stop sensor. */
    // TODO add interface to release callbacks from GPIO

    PX4_INFO("Stopping RC Publisher...");

    //Un-export output pin
    int gpio_fd;
    char buf[3];
    if ((gpio_fd = open("/sys/class/gpio/unexport", O_RDWR | O_SYNC)) < 0) {
        PX4_ERR("Failed to open pin unexport file descriptor");
        return -1;
    }
    snprintf(buf, 3*sizeof(char), "%d", _gpio_Out);
    write(gpio_fd, buf, 3);
    close(gpio_fd);

    _shouldExit=true;

    return 0;
}

void UrsaSonarPub::process_pulse(uint32_t width_usec){
    double range = width_usec/5882.35;
    PX4_INFO("RANGE: %f", range);
    // if (width_usec >= 2700) {
    //     // a long pulse indicates the end of a frame. Reset the
    //     // channel counter so next pulse is channel 0 and publish to uORB
    //     if (_channel_counter >= 0) {
    //         _rcdata.channel_count = _channel_counter;
    //         _publish();
    //     }
    //     _channel_counter = 0;
    //     return;
    // }
    // if (_channel_counter == -1) {
    //     // we are not synchronised
    //     return;
    // }

    
    //   we limit inputs to between 700usec and 2300usec. This allows us
    //   to decode SBUS on the same pin, as SBUS will have a maximum
    //   pulse width of 100usec
     
    // if (width_usec > 700 && width_usec < 2300) {
    //     // take a reading for the current channel
    //     // buffer these
    //     _rcdata.values[_channel_counter] = width_usec;

    //     // move to next channel
    //     _channel_counter++;
    // }

    // // if we have reached the maximum supported channels then
    // // mark as unsynchronised, so we wait for a wide pulse
    // if (_channel_counter >= LINUX_RC_INPUT_NUM_CHANNELS) {
    //     _rcdata.channel_count = _channel_counter;
    //     _channel_counter = -1;
    // }

    return;
}

int UrsaSonarPub::_publish(){

    // uint64_t ts = hrt_absolute_time();
    // _rcdata.timestamp = ts;
    // _rcdata.timestamp_last_signal = ts;
    // _rcdata.rssi = 100;
    // _rcdata.rc_lost_frame_count = 0;
    // _rcdata.rc_total_frame_count = 1;
    // _rcdata.rc_ppm_frame_length = 100;
    // _rcdata.rc_failsafe = false;
    // _rcdata.rc_lost = false;
    // _rcdata.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

    // if (_rc_topic == nullptr) {
    //     _rc_topic = orb_advertise(ORB_ID(input_rc), &_rcdata);

    // } else {
    //     orb_publish(ORB_ID(input_rc), _rc_topic, &_rcdata);
    // }
    return 0;
}


namespace ursa_sonar
{
// Singleton since we only want/need one right now
UrsaSonarPub *g_dev = nullptr;

int start(int gpio_Out, int gpio_In);
int stop();
int info();
void usage();

int start(int gpio_Out, int gpio_In)
{
    if (g_dev != nullptr) {
        PX4_ERR("Ursa sonar already running!");
        return -1;
    }

    g_dev = new UrsaSonarPub();

    if (g_dev == nullptr) {
        PX4_ERR("failed instantiating UrsaSonarPub object");
        return -1;
    }

    int ret = g_dev->init(gpio_Out, gpio_In);

    if (ret != 0) {
        PX4_ERR("UrsaSonarPub init failed");
        return ret;
    }
    PX4_INFO("Started ursa_sonar, output pin: %d, input pin: %d",gpio_Out,gpio_In);

    return 0;
}

int stop()
{
    if (g_dev == nullptr) {
        PX4_ERR("UrsaSonarPub tried to stop but not running");
        return 1;
    }

    int ret = g_dev->stop();

    if (ret != 0) {
        PX4_ERR("UrsaSonarPub could not be stopped");
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
    PX4_WARN("Usage: ursa_sonar 'start [gpio in] [gpio out]', 'info', 'stop'");
}

} // namespace ursa_sonar


int ursa_sonar_main(int argc, char *argv[])
{
    int ret = 0;

    if (argc <= 1) {
        ursa_sonar::usage();
        return 1;
    }

    const char *verb = argv[1];

    if (!strcmp(verb, "start")) {
        if (argc <= 3) {
            ursa_sonar::usage();
            return 1;
        }
        ret = ursa_sonar::start(atoi(argv[2]),atoi(argv[3]));
    }

    else if (!strcmp(verb, "stop")) {
        ret = ursa_sonar::stop();
    }

    else if (!strcmp(verb, "info")) {
        ret = ursa_sonar::info();
    }

    else {
        ursa_sonar::usage();
        return 1;
    }

    return ret;
}
