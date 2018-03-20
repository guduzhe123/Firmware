/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file hil.cpp
 *
 * Driver/configurator for the virtual PWMSim port.
 *
 * This virtual driver emulates PWM / servo outputs for setups where
 * the connected hardware does not provide enough or no PWM outputs.
 *
 * Its only function is to take actuator_control uORB messages,
 * mix them with any loaded mixer and output the result to the
 * actuator_output uORB topic. PWMSim can also be performed with normal
 * PWM outputs, a special flag prevents the outputs to be operated
 * during PWMSim mode. If PWMSim is not performed with a standalone FMU,
 * but in a real system, it is NOT recommended to use this virtual
 * driver. Use instead the normal FMU or IO driver.
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_common.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <cmath>
#include <string.h>
#include <unistd.h>

#include <drivers/device/device.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include <systemlib/systemlib.h>
#include <lib/mixer/mixer.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <float.h>
#include <uORB/topics/mixer_switch.h>
#include <uORB/topics/mavlink_log.h>
#include <systemlib/mavlink_log.h>

#include <systemlib/err.h>

static orb_advert_t mavlink_log_pub = nullptr;

class PWMSim : public device::CDev
{
	const uint32_t PWM_SIM_DISARMED_MAGIC = 900;
	const uint32_t PWM_SIM_FAILSAFE_MAGIC = 600;
public:
	enum Mode {
		MODE_2PWM,
		MODE_4PWM,
		MODE_6PWM,
		MODE_8PWM,
		MODE_12PWM,
		MODE_16PWM,
		MODE_NONE
	};
	PWMSim();
	virtual ~PWMSim();

	virtual int     ioctl(device::file_t *filp, int cmd, unsigned long arg);

	virtual int	init();

	int		set_mode(Mode mode);
	int		set_pwm_rate(unsigned rate);
	int		_task;

private:
	static const unsigned _max_actuators = 8;

	Mode		_mode;
	int 		_update_rate;
	int 		_current_update_rate;
	int			_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	px4_pollfd_struct_t	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned	_poll_fds_num;
	int		_armed_sub;
	int     _vstatus_sub;
	int     actuator_controls_sub ;
	int     _mixer_switch_sub;
	int     _stop_num_pre;
	orb_advert_t	_outputs_pub;
	unsigned	_num_outputs;
	bool		_primary_pwm_device;

	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;

	volatile bool	_task_should_exit;
	static bool	_armed;
	static bool	_lockdown;
	static bool	_failsafe;

	MixerGroup	*_mixers;

	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main();
	void switch_mix(unsigned _rotor_count, float delta_out_max, float _thr_mdl_fac, float *outputs, int stop_num);

	bool updated = false;
	MultirotorMixer::saturation_status _saturation_status{};
	struct actuator_controls_s _actuator_controls;
	struct mixer_switch_s  _mixer_switch = {};

	float 				*_outputs_prev = nullptr;
	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);

	int		pwm_ioctl(device::file_t *filp, int cmd, unsigned long arg);
	void 	subscribe();

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	static const GPIOConfig	_gpio_tab[];
	static const unsigned	_ngpio;

	void		gpio_reset();
	void		gpio_set_function(uint32_t gpios, int function);
	void		gpio_write(uint32_t gpios, int function);
	uint32_t	gpio_read();
	int		gpio_ioctl(device::file_t *filp, int cmd, unsigned long arg);

};

namespace
{

PWMSim	*g_pwm_sim = nullptr;

} // namespace

bool PWMSim::_armed = false;
bool PWMSim::_lockdown = false;
bool PWMSim::_failsafe = false;

PWMSim::PWMSim() :
	CDev("pwm_out_sim", PWM_OUTPUT0_DEVICE_PATH),
	_task(-1),
	_mode(MODE_NONE),
	_update_rate(50),
	_current_update_rate(0),
	_poll_fds{},
	_poll_fds_num(0),
	_armed_sub(-1),
	_vstatus_sub(-1),
	actuator_controls_sub(-1),
	_mixer_switch_sub(-1),
	_stop_num_pre(-1),
	_outputs_pub(nullptr),
	_num_outputs(0),
	_primary_pwm_device(false),
	_groups_required(0),
	_groups_subscribed(0),
	_task_should_exit(false),
	_mixers(nullptr)
{
	memset(_controls, 0, sizeof(_controls));

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		_control_subs[i] = -1;
	}
}

PWMSim::~PWMSim()
{
	if (_task != -1) {
		/* tell the task we want it to go away */
		_task_should_exit = true;

		unsigned i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);

			/* if we have given up, kill it */
			if (--i == 0) {
				px4_task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	g_pwm_sim = nullptr;
}

int
PWMSim::init()
{
	int ret;

	ASSERT(_task == -1);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		return ret;

	} else {
		_primary_pwm_device = true;
	}

	/* start the PWMSim interface task */
	_task = px4_task_spawn_cmd("pwm_out_sim",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_DEFAULT,
				   1200,
				   (px4_main_t)&PWMSim::task_main_trampoline,
				   nullptr);

	if (_task < 0) {
		PX4_INFO("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
PWMSim::task_main_trampoline(int argc, char *argv[])
{
	g_pwm_sim->task_main();
}

int
PWMSim::set_mode(Mode mode)
{
	/*
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_2PWM:
		PX4_INFO("MODE_2PWM");
		/* multi-port with flow control lines as PWM */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 2;
		break;

	case MODE_4PWM:
		PX4_INFO("MODE_4PWM");
		/* multi-port as 4 PWM outs */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 4;
		break;

	case MODE_8PWM:
		PX4_INFO("MODE_8PWM");
		/* multi-port as 8 PWM outs */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 8;
		break;

	case MODE_12PWM:
		PX4_INFO("MODE_12PWM");
		/* multi-port as 12 PWM outs */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 12;
		break;

	case MODE_16PWM:
		PX4_INFO("MODE_16PWM");
		/* multi-port as 16 PWM outs */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 16;
		break;

	case MODE_NONE:
		PX4_INFO("MODE_NONE");
		/* disable servo outputs and set a very low update rate */
		_update_rate = 10;
		_num_outputs = 0;
		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

int
PWMSim::set_pwm_rate(unsigned rate)
{
	if ((rate > 500) || (rate < 10)) {
		return -EINVAL;
	}

	_update_rate = rate;
	return OK;
}

void
PWMSim::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			PX4_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			PX4_DEBUG("unsubscribe from actuator_controls_%d", i);
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] >= 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

void
PWMSim::switch_mix(unsigned _rotor_count, float delta_out_max, float _thrust_factor, float *outputs_add, int stop_num)
{
	/*for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
	    if (_control_subs[i] > 0) {
	        orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);
	//            PX4_INFO("count = %d", count);
	    }
	}*/
	orb_check(actuator_controls_sub, &updated);

	if (updated) {
		/* got command */
		orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_controls_sub, &_actuator_controls);
	}

//    PX4_INFO("roll = %.4f", (double)_actuator_controls.control[0]);

	float		roll    = _actuator_controls.control[0];
	float		pitch   = _actuator_controls.control[1];
	float		yaw     = _actuator_controls.control[2];
	float		thrust  = _actuator_controls.control[3];
	float		min_out = 1.0f;
	float		max_out = 0.0f;

//    PX4_INFO("thrust = %.4f", (double)thrust);
	/*mixers for hexarotor with one motor disabled.*/
	// hexarotor tilted mixer, by zhou hua;
	float hexa_tilted_data[6][4] = {	 -0.981584, -0.240501, -1.150884,  0.907673,
						 0.918681,  0.255922,  1.150884,  0.907673,
						 0.806960,  0.603947, -1.150884,  0.907673,
						 -0.379265, -0.953788,  1.150884,  0.907673,
						 -0.074945,  0.996971,  1.150884,  0.907673,
						 0.747348, -0.442012, -1.150884,  0.907673
				       };
	math::Matrix<6, 4> hexa_tilt;
	hexa_tilt.set(hexa_tilted_data);
//
	// hexarotor disable motor1, by 3 PHD.
	float hexa_dismotor1_data[6][4] = {  0.000000,  0.000000,  0.000000,  0.000000,
					     1.725300,  0.000000,  0.000000,  0.000000,
					     0.431300,  0.747100,  0.000000,  1.000000,
					     -0.431300, -0.747100,  0.000000,  1.000000,
					     -0.431300,  0.747100,  0.000000,  1.000000,
					     0.431300, -0.747100,  0.000000,  1.000000
					  };
	math::Matrix<6, 4> hexa_dismotor1;
	hexa_dismotor1.set(hexa_dismotor1_data);
//
//    // hexarotor disable motor2, by 3 PHD.
//    float hexa_dismotor2_data[6][4] = {	-1.725300,  0.000000,  0.000000,  0.000000 ,
//                                         0.000000,  0.000000,  0.000000,  0.000000 ,
//                                         0.431300,  0.747100,  0.000000,  1.200000 ,
//                                        -0.431300, -0.747100,  0.000000,  1.200000 ,
//                                        -0.431300,  0.747100,  0.000000,  1.200000 ,
//                                         0.431300, -0.747100,  0.000000,  1.200000 };
//    math::Matrix<6,4> hexa_dismotor2;
//    hexa_dismotor2.set(hexa_dismotor2_data);
//
	// hexarotor disable motor3, by 3 PHD.
//    float hexa_dismotor3_data[6][4] = {	-0.862700,  0.000000,  0.000000,  1.000000 ,
//                                         0.862700,  0.000000,  0.000000,  1.000000 ,
//                                         0.000000,  0.000000,  0.000000,  0.000000 ,
//                                        -0.862700, -1.494200,  0.000000,  0.000000 ,
//                                        -0.431300,  0.747100,  0.000000,  1.000000 ,
//                                         0.431300, -0.747100,  0.000000,  1.000000 };
//    math::Matrix<6,4> hexa_dismotor3;
//    hexa_dismotor3.set(hexa_dismotor3_data);
////
////    // hexarotor disable motor4, by 3 PHD.
//    float hexa_dismotor4_data[6][4] = {	   -0.862700,  0.000000,  0.000000,  1.200000 ,
//                                            0.862700,  0.000000,  0.000000,  1.200000 ,
//                                            0.862700,  1.494200,  0.000000,  0.000000 ,
//                                            0.000000,  0.000000,  0.000000,  0.000000 ,
//                                            -0.431300,  0.747100,  0.000000,  1.200000 ,
//                                            0.431300, -0.747100,  0.000000,  1.200000 };
//    math::Matrix<6,4> hexa_dismotor4;
//    hexa_dismotor4.set(hexa_dismotor4_data);
//
	// hexarotor disable motor5, by 3 PHD.
	/*    float hexa_dismotor5_data[6][4] = {	   -0.862700,  0.000000,  0.000000,  1.200000 ,
	                                            0.862700,  0.000000,  0.000000,  1.200000 ,
	                                            0.431300,  0.747100,  0.000000,  1.200000 ,
	                                           -0.431300, -0.747100,  0.000000,  1.200000 ,
	                                            0.000000,  0.000000,  0.000000,  0.000000 ,
	                                            0.862700, -1.494200,  0.000000,  0.000000 };
	    math::Matrix<6,4> hexa_dismotor5;
	    hexa_dismotor5.set(hexa_dismotor5_data);
	//
	    // hexarotor disable motor6, by 3 PHD.
	    float hexa_dismotor6_data[6][4] = {	 -0.862700,  0.000000,  0.000000,  1.200000 ,
	                                          0.862700,  0.000000,  0.000000,  1.200000 ,
	                                          0.431300,  0.747100,  0.000000,  1.200000 ,
	                                         -0.431300, -0.747100,  0.000000,  1.200000 ,
	                                          0.000000,  0.000000,  0.000000,  0.000000 ,
	                                          0.862700, -1.494200,  0.000000,  0.000000 };
	    math::Matrix<6,4> hexa_dismotor6;
	    hexa_dismotor6.set(hexa_dismotor6_data);*/

	// hexarotor x. test
	float hexa_x_data[6][4] = {
		-1.000000,  0.000000, -1.000000,  1.000000,
		1.000000,  0.000000,  1.000000,  1.000000,
		0.500000,  0.866025, -1.000000,  1.000000,
		-0.500000, -0.866025,  1.000000,  1.000000,
		-0.500000,  0.866025,  1.000000,  1.000000,
		0.500000, -0.866025, -1.000000,  1.000000
	};
	math::Matrix<6, 4> hexa_x;
	hexa_x.set(hexa_x_data);


	// clean out class variable used to capture saturation
	//    _saturation_status.value = 0;

	math::Matrix<6, 4> hexa_switch;

	if (stop_num == 1) {
		// switch to hexa tilted
		hexa_switch = hexa_tilt;

		if (_stop_num_pre != stop_num) {
			mavlink_log_critical(&mavlink_log_pub, "Switch to hexa tilt!");
		}

	} else if (stop_num == 2) {
		// switch to hexa disable motor 1
		hexa_switch = hexa_dismotor1;

		if (_stop_num_pre != stop_num) {
			mavlink_log_critical(&mavlink_log_pub, "Switch to hexa dismotor1!");
		}
	}

	_stop_num_pre = stop_num;

	// thrust boost parameters
	float thrust_increase_factor = 1.5f;
	float thrust_decrease_factor = 0.6f;

	//perform initial mix pass yielding unbounded outputs, ignore yaw
	float outputs[_rotor_count];
//    memcpy(outputs, 0, sizeof(outputs));

	for (unsigned i = 0; i < _rotor_count; i++) {
		float out = roll * hexa_switch(i, 0) +
			    pitch * hexa_switch(i, 1) +
			    thrust;

		out *= hexa_switch(i, 3);

//         calculate min and max output values
		if (out < min_out) {
			min_out = out;
		}

		if (out > max_out) {
			max_out = out;
		}

		outputs[i] = out;
	}

	float boost = 0.0f;		// value added to demanded thrust (can also be negative)
	float roll_pitch_scale = 1.0f;	// scale for demanded roll and pitch

	if (min_out < 0.0f && max_out < 1.0f && -min_out <= 1.0f - max_out) {
		float max_thrust_diff = thrust * thrust_increase_factor - thrust;

		if (max_thrust_diff >= -min_out) {
			boost = -min_out;

		} else {
			boost = max_thrust_diff;
			roll_pitch_scale = (thrust + boost) / (thrust - min_out);
		}

	} else if (max_out > 1.0f && min_out > 0.0f && min_out >= max_out - 1.0f) {
		float max_thrust_diff = thrust - thrust_decrease_factor * thrust;

		if (max_thrust_diff >= max_out - 1.0f) {
			boost = -(max_out - 1.0f);

		} else {
			boost = -max_thrust_diff;
			roll_pitch_scale = (1 - (thrust + boost)) / (max_out - thrust);
		}

	} else if (min_out < 0.0f && max_out < 1.0f && -min_out > 1.0f - max_out) {
		float max_thrust_diff = thrust * thrust_increase_factor - thrust;
		boost = math::constrain(-min_out - (1.0f - max_out) / 2.0f, 0.0f, max_thrust_diff);
		roll_pitch_scale = (thrust + boost) / (thrust - min_out);

	} else if (max_out > 1.0f && min_out > 0.0f && min_out < max_out - 1.0f) {
		float max_thrust_diff = thrust - thrust_decrease_factor * thrust;
		boost = math::constrain(-(max_out - 1.0f - min_out) / 2.0f, -max_thrust_diff, 0.0f);
		roll_pitch_scale = (1 - (thrust + boost)) / (max_out - thrust);

	} else if (min_out < 0.0f && max_out > 1.0f) {
		boost = math::constrain(-(max_out - 1.0f + min_out) / 2.0f, thrust_decrease_factor * thrust - thrust,
					thrust_increase_factor * thrust - thrust);
		roll_pitch_scale = (thrust + boost) / (thrust - min_out);
	}

	// capture saturation
	if (min_out < 0.0f) {
		_saturation_status.flags.motor_neg = true;
	}

	if (max_out > 1.0f) {
		_saturation_status.flags.motor_pos = true;
	}

	// Thrust reduction is used to reduce the collective thrust if we hit
	// the upper throttle limit
	float thrust_reduction = 0.0f;

	// mix again but now with thrust boost, scale roll/pitch and also add yaw
	for (unsigned i = 0; i < _rotor_count; i++) {
		float out = (roll * hexa_switch(i, 0) +
			     pitch * hexa_switch(i, 1)) * roll_pitch_scale +
			    yaw * hexa_switch(i, 2) +
			    thrust + boost;

		out *= hexa_switch(i, 3);

		// scale yaw if it violates limits. inform about yaw limit reached
		if (out < 0.0f) {
			if (fabsf(hexa_switch(i, 2)) <= FLT_EPSILON) {
				yaw = 0.0f;

			} else {
				yaw = -((roll * hexa_switch(i, 0) + pitch * hexa_switch(i, 1)) *
					roll_pitch_scale + thrust + boost) / hexa_switch(i, 2);
			}

		} else if (out > 1.0f) {
			// allow to reduce thrust to get some yaw response
			float prop_reduction = fminf(0.15f, out - 1.0f);
			// keep the maximum requested reduction
			thrust_reduction = fmaxf(thrust_reduction, prop_reduction);

			if (fabsf(hexa_switch(i, 2)) <= FLT_EPSILON) {
				yaw = 0.0f;

			} else {
				yaw = (1.0f - ((roll * hexa_switch(i, 0) + pitch * hexa_switch(i, 1)) *
					       roll_pitch_scale + (thrust - thrust_reduction) + boost)) / hexa_switch(i, 2);
			}
		}
	}

	// Apply collective thrust reduction, the maximum for one prop
	thrust -= thrust_reduction;

	// add yaw and scale outputs to range idle_speed...1
	for (unsigned i = 0; i < _rotor_count; i++) {
		outputs[i] = (roll * hexa_switch(i, 0) +
			      pitch * hexa_switch(i, 1)) * roll_pitch_scale +
			     yaw * hexa_switch(i, 2) +
			     thrust + boost;


//            implement simple model for static relationship between applied motor pwm and motor thrust
//            model: thrust = (1 - _thrust_factor) * PWM + _thrust_factor * PWM^2
//            this model assumes normalized input / output in the range [0,1] so this is the right place
//            to do it as at this stage the outputs are in that range.

		if (_thrust_factor > 0.0f) {
			outputs[i] = -(1.0f - _thrust_factor) / (2.0f * _thrust_factor) + sqrtf((1.0f - _thrust_factor) *
					(1.0f - _thrust_factor) / (4.0f * _thrust_factor * _thrust_factor) + (outputs[i] < 0.0f ? 0.0f : outputs[i] /
							_thrust_factor));
		}

		outputs[i] = math::constrain(-1.0f + (outputs[i] * 2.0f), -1.0f, 1.0f);
		outputs_add[i] = outputs[i];
	}

}

void
PWMSim::task_main()
{
	/* force a reset of the update rate */
	_current_update_rate = 0;

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	_mixer_switch_sub = orb_subscribe(ORB_ID(mixer_switch));
	/* advertise the mixed control outputs */
	actuator_outputs_s outputs = {};

	/* advertise the mixed control outputs, insist on the first group output */
	_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &outputs);


	/* loop until killed */
	while (!_task_should_exit) {

		if (_groups_subscribed != _groups_required) {
			subscribe();
			_groups_subscribed = _groups_required;
		}

		/* handle update rate changes */
		if (_current_update_rate != _update_rate) {
			int update_rate_in_ms = int(1000 / _update_rate);

			if (update_rate_in_ms < 2) {
				update_rate_in_ms = 2;
			}

			for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
				if (_control_subs[i] >= 0) {
					orb_set_interval(_control_subs[i], update_rate_in_ms);
				}
			}

			// up_pwm_servo_set_rate(_update_rate);
			_current_update_rate = _update_rate;
		}

		/* this can happen during boot, but after the sleep its likely resolved */
		if (_poll_fds_num == 0) {
			usleep(1000 * 1000);

			PX4_DEBUG("no valid fds");
			continue;
		}

		/* sleep waiting for data, but no more than a second */
		int ret = px4_poll(&_poll_fds[0], _poll_fds_num, 1000);

		/* this would be bad... */
		if (ret < 0) {
			DEVICE_LOG("poll error %d", errno);
			continue;
		}

		if (ret == 0) {
			// timeout
			continue;
		}

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] >= 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);
				}

				poll_id++;
			}
		}

		/* can we mix? */
		if (_armed && _mixers != nullptr) {

			size_t num_outputs;

			switch (_mode) {
			case MODE_2PWM:
				num_outputs = 2;
				break;

			case MODE_4PWM:
				num_outputs = 4;
				break;

			case MODE_6PWM:
				num_outputs = 6;
				break;

			case MODE_8PWM:
				num_outputs = 8;
				break;

			case MODE_16PWM:
				num_outputs = 16;
				break;

			default:
				num_outputs = 0;
				break;
			}

			orb_check(_mixer_switch_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(mixer_switch), _mixer_switch_sub, &_mixer_switch);
			}

			/* do mixing */
			if (_mixer_switch.mixer_switch_enable &&  _mixer_switch.mixer_switch_num != 0) {
				float outputs_add[6] = {};
				switch_mix(6, 0.0f, 0.0f, &outputs_add[0], _mixer_switch.mixer_switch_num);

				for (int i = 0; i < 6; i++) {
					outputs.output[i] = outputs_add[i];
				}

			} else {
				num_outputs = _mixers->mix(&outputs.output[0], num_outputs);

				if (_stop_num_pre != _mixer_switch.mixer_switch_num) {
					mavlink_log_critical(&mavlink_log_pub, "Using default hexa x mixer!");
				}

				_stop_num_pre = _mixer_switch.mixer_switch_num;
			}

			outputs.noutputs = num_outputs;
			outputs.timestamp = hrt_absolute_time();

			/*for (int i = 0; i < 6; i++){
			    PX4_INFO("outputs.output[%d] = %.4f", i, (double)outputs.output[i]);
			}*/

//			PX4_INFO("outputs[1] = %.4f", (double)outputs.output[1]);
//            PX4_INFO("outputs[2] = %.4f", (double)outputs.output[2]);
//            PX4_INFO("outputs[3] = %.4f", (double)outputs.output[3]);
//            PX4_INFO("outputs[4] = %.4f", (double)outputs.output[4]);
//            PX4_INFO("outputs[5] = %.4f", (double)outputs.output[5]);
			/* disable unused ports by setting their output to NaN */
			for (size_t i = 0; i < sizeof(outputs.output) / sizeof(outputs.output[0]); i++) {
				if (i >= num_outputs) {
					outputs.output[i] = NAN;//use NAN not 0
				}
			}

			/* iterate actuators */
			for (unsigned i = 0; i < num_outputs; i++) {
				/* last resort: catch NaN, INF and out-of-band errors */
				if (i < outputs.noutputs &&
				    PX4_ISFINITE(outputs.output[i]) &&
				    outputs.output[i] >= -1.0f &&
				    outputs.output[i] <= 1.0f) {
					/* scale for PWM output 1000 - 2000us */
					outputs.output[i] = 1500 + (500 * outputs.output[i]);

				} else {
					/*
					 * Value is NaN, INF or out of band - set to the minimum value.
					 * This will be clearly visible on the servo status and will limit the risk of accidentally
					 * spinning motors. It would be deadly in flight.
					 */
					//900 use this to disable motor
					outputs.output[i] = PWM_SIM_DISARMED_MAGIC;
				}
			}


#if CT_DISABLE_MOTOR
			/*
			 * QGroundControl Custom Command to disable motor
			 * QGCButton {
			 *      text: "VEHICLE_CMD_CT_MOTOR_STOP NO.3"
			 *      // Arguments to CustomCommandWidgetController::sendCommand (Mavlink COMMAND_LONG)
			 *      //   command id
			 *      //   component id
			 *      //   confirmadtion
			 *      //   param 1-7
			 *      onClicked: controller.sendCommand(701, 0, 0, 1, 3, 0, 0, 0, 0, 0)
			 * }
			 * QGCButton {
			 *      text: "VEHICLE_CMD_CT_MOTOR_STOP NO.3&4"
			 *      onClicked: controller.sendCommand(701, 0, 0, 1, 3, 1, 0, 0, 0, 0)
			 * }
			 */

			bool ct_updated;
			int motor_stop_num = 0;
			int motor_stop_num_oppo = 0;

			orb_check(_vstatus_sub, &ct_updated);
			static vehicle_status_s vehicle_status;

			if (ct_updated) {
				orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &vehicle_status);
			}

			motor_stop_num = vehicle_status.motor_stop_num;

			if (motor_stop_num > 10) {
				motor_stop_num_oppo = motor_stop_num / 10 - 1;
				motor_stop_num = motor_stop_num % 10 - 1;
				outputs.output[motor_stop_num_oppo] =  PWM_SIM_DISARMED_MAGIC;
				outputs.output[motor_stop_num] =  PWM_SIM_DISARMED_MAGIC;

			} else if (motor_stop_num > 0) {
				outputs.output[motor_stop_num - 1] =  PWM_SIM_DISARMED_MAGIC;
			}

#endif


			/* overwrite outputs in case of force_failsafe */
			if (_failsafe) {
				for (size_t i = 0; i < num_outputs; i++) {
					outputs.output[i] = PWM_SIM_FAILSAFE_MAGIC;//600
				}
			}

			/* overwrite outputs in case of lockdown */
			if (_lockdown) {
				for (size_t i = 0; i < num_outputs; i++) {
					outputs.output[i] = 0.0;
				}
			}

			/* and publish for anyone that cares to see */
			orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &outputs);
		}

		/* how about an arming update? */
//		bool updated;
		actuator_armed_s aa;
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &aa);
			/* do not obey the lockdown value, as lockdown is for PWMSim. Only obey manual lockdown */
			_armed = aa.armed;
			_failsafe = aa.force_failsafe;
			_lockdown = aa.manual_lockdown;
		}
	}

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] >= 0) {
			orb_unsubscribe(_control_subs[i]);
		}
	}

	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_vstatus_sub);

	/* make sure servos are off */
	// up_pwm_servo_deinit();

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_task = -1;
}

int
PWMSim::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	if (_armed) {
		input = controls[control_group].control[control_index];

	} else {
		/* clamp actuator to zero if not armed */
		input = 0.0f;
	}

	return 0;
}

int
PWMSim::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret;

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch (_mode) {
	case MODE_2PWM:
	case MODE_4PWM:
	case MODE_8PWM:
	case MODE_12PWM:
	case MODE_16PWM:
		ret = PWMSim::pwm_ioctl(filp, cmd, arg);
		break;

	default:
		ret = -ENOTTY;
		PX4_INFO("not in a PWM mode");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
PWMSim::pwm_ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;
	// int channel;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		// up_pwm_servo_arm(true);
		break;

	case PWM_SERVO_DISARM:
		// up_pwm_servo_arm(false);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		// PWMSim always outputs at the alternate (usually faster) rate
		g_pwm_sim->set_pwm_rate(arg);
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		// PWMSim always outputs at the alternate (usually faster) rate
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = 400;
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = 400;
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = 0;
		break;

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 850;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 900;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 1000;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 1500;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 2000;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_SET(2):
	case PWM_SERVO_SET(3):
		if (_mode != MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */

	case PWM_SERVO_SET(0):
	case PWM_SERVO_SET(1):
		if (arg < 2100) {
			// channel = cmd - PWM_SERVO_SET(0);
//			up_pwm_servo_set(channel, arg); XXX

		} else {
			ret = -EINVAL;
		}

		break;

	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
	case PWM_SERVO_GET(5):
	case PWM_SERVO_GET(4):
		if (_num_outputs < 8) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */

	case PWM_SERVO_GET(3):
	case PWM_SERVO_GET(2):
		if (_num_outputs < 4) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */

	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0): {
			*(servo_position_t *)arg = 1500;
			break;
		}

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {
			// no restrictions on output grouping
			unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

			*(uint32_t *)arg = (1 << channel);
			break;
		}

	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:
		if (_mode == MODE_16PWM) {
			*(unsigned *)arg = 16;

		} else if (_mode == MODE_8PWM) {

			*(unsigned *)arg = 8;

		} else if (_mode == MODE_4PWM) {

			*(unsigned *)arg = 4;

		} else {

			*(unsigned *)arg = 2;
		}

		break;

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCADDSIMPLE: {
			mixer_simple_s *mixinfo = (mixer_simple_s *)arg;

			SimpleMixer *mixer = new SimpleMixer(control_callback,
							     (uintptr_t)&_controls, mixinfo);

			if (mixer->check()) {
				delete mixer;
				_groups_required = 0;
				ret = -EINVAL;

			} else {
				if (_mixers == nullptr) {
					_mixers = new MixerGroup(control_callback,
								 (uintptr_t)&_controls);
				}

				_mixers->add_mixer(mixer);
				_mixers->groups_required(_groups_required);
			}

			break;
		}

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)&_controls);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					PX4_ERR("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {
					_mixers->groups_required(_groups_required);
				}
			}

			break;
		}


	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

namespace
{

enum PortMode {
	PORT_MODE_UNDEFINED = 0,
	PORT1_MODE_UNSET,
	PORT1_FULL_PWM,
	PORT2_MODE_UNSET,
	PORT2_8PWM,
	PORT2_12PWM,
	PORT2_16PWM,
};

PortMode g_port_mode = PORT_MODE_UNDEFINED;

int
hil_new_mode(PortMode new_mode)
{
	// uint32_t gpio_bits;


//	/* reset to all-inputs */
//	g_pwm_sim->ioctl(0, GPIO_RESET, 0);

	// gpio_bits = 0;

	PWMSim::Mode servo_mode = PWMSim::MODE_NONE;

	switch (new_mode) {
	case PORT_MODE_UNDEFINED:
	case PORT1_MODE_UNSET:
	case PORT2_MODE_UNSET:
		/* nothing more to do here */
		break;

	case PORT1_FULL_PWM:
		/* select 4-pin PWM mode */
		servo_mode = PWMSim::MODE_8PWM;
		break;

	case PORT2_8PWM:
		/* select 8-pin PWM mode */
		servo_mode = PWMSim::MODE_8PWM;
		break;

	case PORT2_12PWM:
		/* select 12-pin PWM mode */
		servo_mode = PWMSim::MODE_12PWM;
		break;

	case PORT2_16PWM:
		/* select 16-pin PWM mode */
		servo_mode = PWMSim::MODE_16PWM;
		break;
	}

//	/* adjust GPIO config for serial mode(s) */
//	if (gpio_bits != 0)
//		g_pwm_sim->ioctl(0, GPIO_SET_ALT_1, gpio_bits);

	/* (re)set the PWM output mode */
	g_pwm_sim->set_mode(servo_mode);

	return OK;
}

int
test()
{
	int	fd;

	fd = px4_open(PWM_OUTPUT0_DEVICE_PATH, 0);

	if (fd < 0) {
		puts("open fail");
		return -ENODEV;
	}

	px4_ioctl(fd, PWM_SERVO_ARM, 0);
	px4_ioctl(fd, PWM_SERVO_SET(0), 1000);

	px4_close(fd);

	return OK;
}

int
fake(int argc, char *argv[])
{
	if (argc < 5) {
		puts("pwm_out_sim fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)");
		return -EINVAL;
	}

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], nullptr, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], nullptr, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], nullptr, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], nullptr, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle == nullptr) {
		puts("advertise failed");
		return 1;
	}

	return 0;
}

} // namespace

extern "C" __EXPORT int pwm_out_sim_main(int argc, char *argv[]);

static void
usage()
{
	PX4_WARN("unrecognized command, try:");
	PX4_WARN("  mode_pwm, mode_port2_pwm8, mode_port2_pwm12, mode_port2_pwm16");
}

int
pwm_out_sim_main(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNDEFINED;
	const char *verb;
	int ret = OK;

	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	verb = argv[1];

	if (g_pwm_sim == nullptr) {
		g_pwm_sim = new PWMSim;

		if (g_pwm_sim == nullptr) {
			return -ENOMEM;
		}
	}

	/*
	 * Mode switches.
	 */

	// this was all cut-and-pasted from the FMU driver; it's junk
	if (!strcmp(verb, "mode_pwm")) {
		new_mode = PORT1_FULL_PWM;

	} else if (!strcmp(verb, "mode_port2_pwm8")) {
		new_mode = PORT2_8PWM;

	} else if (!strcmp(verb, "mode_port2_pwm12")) {
		new_mode = PORT2_8PWM;

	} else if (!strcmp(verb, "mode_port2_pwm16")) {
		new_mode = PORT2_8PWM;

	} else if (!strcmp(verb, "mode_pwm16")) {
		new_mode = PORT2_16PWM;
	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNDEFINED) {

		/* yes but it's the same mode */
		if (new_mode == g_port_mode) {
			return OK;
		}

		/* switch modes */
		ret = hil_new_mode(new_mode);

	} else if (!strcmp(verb, "test")) {
		ret = test();
	}

	else if (!strcmp(verb, "fake")) {
		ret = fake(argc - 1, argv + 1);
	}

	else {
		usage();
		ret = -EINVAL;
	}

	if (ret == OK && g_pwm_sim->_task == -1) {
		ret = g_pwm_sim->init();

		if (ret != OK) {
			warnx("failed to start the pwm_out_sim driver");
			delete g_pwm_sim;
			g_pwm_sim = nullptr;
		}
	}

	return ret;
}
