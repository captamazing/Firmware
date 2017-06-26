/****************************************************************************
 *
 * Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file df_erlebrain2_pwm_out_wrapper.cpp
 * Lightweight driver to access the PWM of the DriverFramework.
 */


#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/mixer/mixer_multirotor.generated.h>
#include <systemlib/param/param.h>
#include <systemlib/pwm_limit/pwm_limit.h>

#include <pca9685/PCA9685.hpp>
#include <DevMgr.hpp>

extern "C" { __EXPORT int df_ursapwm_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;

#define NUM_PWM 4
#define FREQUENCY_PWM 400
#define MIXER_FILENAME "ROMFS/px4fmu_common/mixers/quad_x.main.mix"

class DfPWMWrapper : public PCA9685 {
public:
    DfPWMWrapper():PCA9685(),
        _task_handle(-1),
        _task_should_exit(false),
        _is_running(false),
        _outputs_pub(nullptr),
        _rc_pub(nullptr),
        _mixer(nullptr)
    {}
    ~DfPWMWrapper(){};

    /**
     * Start PWM output
     *
     * @return 0 on success
     */
    int     start();

    /**
     * Stop PWM output
     *
     * @return 0 on success
     */
    int     stop();
    void task_main(int argc, char *argv[]);
    // threading variables
    px4_task_t _task_handle;
    volatile bool _task_should_exit;
    bool _is_running;

private:

    // subscriptions
    int     _controls_sub;
    int     _armed_sub;

    // publications
    orb_advert_t    _outputs_pub;
    orb_advert_t    _rc_pub;

    // topic structures
    actuator_controls_s _controls;
    actuator_outputs_s  _outputs;
    actuator_armed_s    _armed;

    pwm_limit_t     _pwm_limit;

    // esc parameters
    int32_t _pwm_disarmed;
    int32_t _pwm_min;
    int32_t _pwm_max;

    MultirotorMixer *_mixer;

    void send_outputs_pwm(const uint16_t *pwm);

    /* mixer initialization */
    int initialize_mixer(const char *mixer_filename);
    static int mixer_control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);

};

int DfPWMWrapper::start(){
    // gets the parameters for the esc's pwm
    param_get(param_find("PWM_DISARMED"), &_pwm_disarmed);
    param_get(param_find("PWM_MIN"), &_pwm_min);
    param_get(param_find("PWM_MAX"), &_pwm_max);

    /* Init device and start sensor. */
    int ret = init();

    if (ret != 0) {
        PX4_ERR("PWM register device fail: %d", ret);
        return ret;
    }

    ret = PCA9685::start();

    if (ret != 0) {
        PX4_ERR("PCA9685 start fail: %d", ret);
        return ret;
    }

    return 0;
}

int DfPWMWrapper::mixer_control_callback(uintptr_t handle,
               uint8_t control_group,
               uint8_t control_index,
               float &input)
{
    const actuator_controls_s *controls = (actuator_controls_s *)handle;
    input = controls[control_group].control[control_index];
    return 0;
}

int DfPWMWrapper::initialize_mixer(const char *mixer_filename)
{
    char buf[2048];
    size_t buflen = sizeof(buf);
    memset(buf, 0, 2048);

    PX4_INFO("Trying to initialize mixer from config file %s", mixer_filename);

    int fd_load = ::open(mixer_filename, O_RDONLY);

    if (fd_load != -1) {
        int nRead = ::read(fd_load, buf, buflen);
        close(fd_load);

        if (nRead > 0) {
            _mixer = MultirotorMixer::from_text(mixer_control_callback, (uintptr_t)&_controls, buf, buflen);

            if (_mixer != nullptr) {
                PX4_INFO("Successfully initialized mixer from config file");
                return 0;

            } else {
                PX4_ERR("Unable to parse from mixer config file");
                return -1;
            }

        } else {
            PX4_WARN("Unable to read from mixer config file");
            return -2;
        }

    } else {
        PX4_WARN("No mixer config file found, using default mixer.");

        /* Mixer file loading failed, fall back to default mixer configuration for
        * QUAD_X airframe. */
        float roll_scale = 1;
        float pitch_scale = 1;
        float yaw_scale = 1;
        float deadband = 0;

        _mixer = new MultirotorMixer(mixer_control_callback, (uintptr_t)&_controls,
                         MultirotorGeometry::QUAD_X,
                         roll_scale, pitch_scale, yaw_scale, deadband);

        // TODO: temporary hack to make this compile
        (void)_config_index[0];

        if (_mixer == nullptr) {
            PX4_ERR("Mixer initialization failed");
            return -1;
        }

        return 0;
    }
}

int DfPWMWrapper::stop()
{
    /* Stop sensor. */
    int ret = PCA9685::stop();
    if (ret != 0) {
        PX4_ERR("PWM stop fail: %d", ret);
        return ret;
    }

    PX4_INFO("Stopping PCA9685 Wrapper...");

    return 0;
}

void DfPWMWrapper::send_outputs_pwm(const uint16_t *pwm)
{
    //convert this to duty_cycle in ns
    for (unsigned i = 0; i < NUM_PWM; ++i) {
        write(i, pwm[i]);
    }
    push();
}

void DfPWMWrapper::task_main(int argc, char *argv[])
{
    _is_running = true;

    // Subscribe for orb topics
    _controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
    _armed_sub = orb_subscribe(ORB_ID(actuator_armed));

    // Start disarmed
    _armed.armed = false;
    _armed.prearmed = false;

    // Set up poll topic
    px4_pollfd_struct_t fds[1];
    fds[0].fd     = _controls_sub;
    fds[0].events = POLLIN;
    /* Don't limit poll intervall for now, 250 Hz should be fine. */
    //orb_set_interval(_controls_sub, 10);

    // Set up mixer
    if (initialize_mixer(MIXER_FILENAME) < 0) {
        PX4_ERR("Mixer initialization failed.");
        return;
    }

    pwm_limit_init(&_pwm_limit);

    // Main loop
    while (!_task_should_exit) {

        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 10);

        /* Timed out, do a periodic check for _task_should_exit. */
        if (pret == 0) {
            continue;
        }

        /* This is undesirable but not much we can do. */
        if (pret < 0) {
            PX4_WARN("poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;
        }

        if (fds[0].revents & POLLIN) {
            orb_copy(ORB_ID(actuator_controls_0), _controls_sub, &_controls);

            _outputs.timestamp = _controls.timestamp;

            /* do mixing */
            _outputs.noutputs = _mixer->mix(_outputs.output,
                            0 /* not used */,
                            NULL);


            /* disable unused ports by setting their output to NaN */
            for (size_t i = _outputs.noutputs;
                 i < sizeof(_outputs.output) / sizeof(_outputs.output[0]);
                 i++) {
                _outputs.output[i] = NAN;
            }

            const uint16_t reverse_mask = 0;
            uint16_t disarmed_pwm[4];
            uint16_t min_pwm[4];
            uint16_t max_pwm[4];

            for (unsigned int i = 0; i < 4; i++) {
                disarmed_pwm[i] = _pwm_disarmed;
                min_pwm[i] = _pwm_min;
                max_pwm[i] = _pwm_max;
            }

            uint16_t pwm[4];

            // TODO FIXME: pre-armed seems broken
            pwm_limit_calc(_armed.armed,
                       false/*_armed.prearmed*/,
                       _outputs.noutputs,
                       reverse_mask,
                       disarmed_pwm,
                       min_pwm,
                       max_pwm,
                       _outputs.output,
                       pwm,
                       &_pwm_limit);

            //pwm = 900 ~ 1800;
            send_outputs_pwm(pwm);

            if (_outputs_pub != nullptr) {
                orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

            } else {
                _outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
            }
        }

        bool updated;
        orb_check(_armed_sub, &updated);

        if (updated) {
            orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
        }
    }

    orb_unsubscribe(_controls_sub);
    orb_unsubscribe(_armed_sub);

    _is_running = false;

}

namespace df_ursapwm_wrapper
{

DfPWMWrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();
void task_main_trampoline(int argc, char *argv[]);

int start()
{
    g_dev = new DfPWMWrapper();

    if (g_dev == nullptr) {
        PX4_ERR("failed instantiating DfPWMWrapper object");
        return -1;
    }

    int ret = g_dev->start();

    if (ret != 0) {
        PX4_ERR("DfPWMWrapper start failed");
        return ret;
    }

    // Test open the PWM driver
    DevHandle h;
    DevMgr::getHandle(PWM_DEVICE_PATH, h);

    if (!h.isValid()) {
        DF_LOG_INFO("Error: unable to obtain a valid handle for the PWM output at: %s (%d)",
                PWM_DEVICE_PATH, h.getError());
        return -1;
    }

    DevMgr::releaseHandle(h);

    g_dev->_task_should_exit = false;
    /* start the task */
    g_dev->_task_handle = px4_task_spawn_cmd("pwm_out_main",
                      SCHED_DEFAULT,
                      SCHED_PRIORITY_MAX,
                      1500,
                      (px4_main_t)&task_main_trampoline,
                      nullptr);

    if (g_dev->_task_handle < 0) {
        warn("task start failed");
        return -1;
    }

    return 0;

}

void task_main_trampoline(int argc, char *argv[]){
    g_dev->task_main(argc, argv);
}

int stop()
{
    g_dev->_task_should_exit = true;

    while (g_dev->_is_running) {
        usleep(200000);
        PX4_INFO(".");
    }

    g_dev->_task_handle = -1;

    return OK;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
    if (g_dev == nullptr) {
        PX4_ERR("driver not running");
        return ERROR;
    }

    PX4_DEBUG("state @ %p", g_dev);

    return OK;
}

void
usage()
{
    PX4_WARN("Usage: df_erlebrain2_pwm_out_wrapper 'start', 'info', 'stop'");
}

} // namespace df_ursapwm_wrapper


int
df_ursapwm_wrapper_main(int argc, char *argv[])
{
    int ret = 0;
    int myoptind = 1;

    if (argc <= 1) {
        df_ursapwm_wrapper::usage();
        return 1;
    }

    const char *verb = argv[myoptind];

    if (!strcmp(verb, "start")) {
        ret = df_ursapwm_wrapper::start();
    }

    else if (!strcmp(verb, "stop")) {
        ret = df_ursapwm_wrapper::stop();
    }

    else if (!strcmp(verb, "info")) {
        ret = df_ursapwm_wrapper::info();
    }

    else {
        df_ursapwm_wrapper::usage();
        return 1;
    }

    return ret;
}
