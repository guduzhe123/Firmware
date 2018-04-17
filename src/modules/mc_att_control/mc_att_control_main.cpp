/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <lib/mixer/mixer.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/pid_auto_tune.h>
#include <uORB/topics/mixer_switch.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/user_intention.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/uORB.h>
#include <systemlib/mavlink_log.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

#define MAX_GYRO_COUNT 3

#define PID_TUNE_START_TIME 500

static orb_advert_t mavlink_log_pub = nullptr;

class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~MulticopterAttitudeControl();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	int		_v_att_sub;		/**< vehicle attitude subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int		_motor_limits_sub;		/**< motor limits subscription */
	int		_battery_status_sub;	/**< battery status subscription */
	int		_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
	int		_sensor_correction_sub;	/**< sensor thermal correction subscription */
	int		_sensor_bias_sub;	/**< sensor in-run bias correction subscription */
	int     _mixer_switch_sub;  /* mixer switch subscription*/
	int     _vehicleLocalPosition_sub;
	int     _actuator_outputs_sub;
	int     _user_intention_sub;

	unsigned _gyro_count;
	int _selected_gyro;

	uint16_t _time;     /* calculate time one motion needed    */
	uint16_t _time_reached;    /*calculate time when real attitude reachs to att_sp  */
	uint16_t _time_overshot;   /*calculate time when real attitude at its max value   */

	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */
	orb_advert_t    _pid_tune_pub;         /* PID tuning publication  */
	orb_advert_t    _wind_estimate_pub;    /* */
	orb_advert_t    _debug_vect_pub;


	orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s		_v_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s		_actuators;		/**< actuator controls */
	struct vehicle_status_s			_vehicle_status;	/**< vehicle status */
	struct mc_att_ctrl_status_s 		_controller_status;	/**< controller status */
	struct battery_status_s			_battery_status;	/**< battery status */
	struct sensor_gyro_s			_sensor_gyro;		/**< gyro data before thermal correctons and ekf bias estimates are applied */
	struct sensor_correction_s		_sensor_correction;	/**< sensor thermal corrections */
	struct sensor_bias_s			_sensor_bias;		/**< sensor in-run bias corrections */
	struct pid_auto_tune_s          _pid_tune;          /*PID tuning data   */
	struct mixer_switch_s           _mixer_switch;      /* Mixer switch paramsters*/
	struct wind_estimate_s          _wind_estimate;     /*wind estimate*/
	struct vehicle_local_position_s				_vehicleLocalPosition;
	struct actuator_outputs_s       _actuator_outputs;
	struct user_intention_s             _user_intention_msg;
	struct debug_vect_s                 _debug_vect_msg;

	MultirotorMixer::saturation_status _saturation_status{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Matrix<3, 3>  _I;				/**< identity matrix */

	math::Matrix<3, 3>	_board_rotation = {};	/**< rotation matrix for the orientation that the board is mounted */
	math::Vector<3> _e_ang ;             /* error of angle */
	math::Vector<3> _e_rate ;            /* error of angle rate */
	math::Vector<3> _euler_angles;       /* euler angles of real */
	math::Vector<3> _euler_angles_sp;    /* euler angles setpoint */
	float _j_pid_tune;                   /* PID profermance judgement */
	float _ang_pre;                      /* angle of previous circle  */
	float _ang_act;                      /* angle of this circle */
	float _ang_max;                      /* max angle of real euler angle at one motion */
	float _ang_sum;                      /* sum value of real euler angle */
	math::Matrix<3, 3> _R_sp_pid_tune;   /* rotation matrix of angle_sp */
	math::Matrix<3, 3> _R_pid_tune;      /* rotation maxtrix of angle */
	float _roll_sp_pid;                  /* roll_sp */
	float _pitch_sp_pid;                 /* pitch_sp */
	float _yaw_sp_pid;                   /* yaw_sp  */
	float _control_input;                /* control input and energy */
	float _e_euler_ang;                  /* the needed error of euler angle when pid tuning begin*/
	float _e_euler_rate;                 /* the needed error of euler angle rate when pid tuning begin*/
	float _euler_angles_input;           /* the needed angle */
	float _angle_sp;                     /* the needed angle_sp */
	int w1 ;
	int w2 ;
	int w3 ;
	int w4 ;
	float w5 ;

	int64_t _time_start_wind;
	int _time_warn;

	int32_t _time_high_thrust;
	int64_t _time_high_pwm;

	enum PID_STATE {
		TUNE_NONE = 0,   /* don't tune */
		TUNE_ROLL_P,     /* tune roll at positive direction */
		TUNE_ROLL_N,     /* tune roll at negative direction*/
		TUNE_PITCH_P,
		TUNE_PITCH_N,
		TUNE_YAW_CW,
		TUNE_YAW_CCW,
	} _pid_tune_state;   /* which axis will be used for pid tuning */

	int pid_tune_stop;

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_integ_lim;
		param_t roll_rate_d;
		param_t roll_rate_ff;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_integ_lim;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t tpa_breakpoint_p;
		param_t tpa_breakpoint_i;
		param_t tpa_breakpoint_d;
		param_t tpa_rate_p;
		param_t tpa_rate_i;
		param_t tpa_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_integ_lim;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;
		param_t yaw_auto_max;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t acro_expo;
		param_t acro_superexpo;
		param_t rattitude_thres;

		param_t roll_tc;
		param_t pitch_tc;

		param_t vtol_type;
		param_t vtol_opt_recovery_enabled;
		param_t vtol_wv_yaw_rate_scale;

		param_t bat_scale_en;

		param_t board_rotation;

		param_t board_offset[3];
		param_t mc_pid_autotune;
		param_t mc_pid_adjust;
		param_t mc_pid_axis;
		param_t mc_pid_angle_rp;
		param_t mc_pid_angle_y;
		param_t mc_pid_time;

		param_t ht_rollrate_p;  // increase rollrate_p when one motor is disabled with hexa_tilt mixer.
		param_t ht_pitchrate_p; // increase pitchllrate_p when one motor is disabled with hexa_tilt mixer.
		param_t hts_rollrate_p;  // increase rollrate_p when one motor is disabled with hexa_tilt_s mixer.
		param_t hts_pitchrate_p; // increase pitchllrate_p when one motor is disabled with hexa_tilt_s mixer.
		param_t wind_detect;     // enable wind detect feature or not.
		param_t wind_start_num;  // wind level start number.
		param_t wind_thrust_max;  // wind disturbance warning while thrust is so big than this level.
		param_t sys_autostart;
		param_t max_rotation;

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_int_lim;			/**< integrator state limit for rate loop */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
		float yaw_ff;						/**< yaw control feed-forward */

		float tpa_breakpoint_p;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_i;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_d;				/**< Throttle PID Attenuation breakpoint */
		float tpa_rate_p;					/**< Throttle PID Attenuation slope */
		float tpa_rate_i;					/**< Throttle PID Attenuation slope */
		float tpa_rate_d;					/**< Throttle PID Attenuation slope */

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		float yaw_auto_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */
		math::Vector<3> auto_rate_max;		/**< attitude rate limits in auto modes */
		matrix::Vector3f acro_rate_max;		/**< max attitude rates in acro mode */
		float acro_expo;					/**< function parameter for expo stick curve shape */
		float acro_superexpo;				/**< function parameter for superexpo stick curve shape */
		float rattitude_thres;

		int32_t vtol_type;					/**< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
		bool vtol_opt_recovery_enabled;
		float vtol_wv_yaw_rate_scale;			/**< Scale value [0, 1] for yaw rate setpoint  */

		int32_t bat_scale_en;

		int32_t board_rotation;

		float board_offset[3];

		int mc_pid_autotune;
		int mc_pid_adjust;
		int mc_pid_axis;
		float mc_pid_angle_rp;
		float mc_pid_angle_y;
		int mc_pid_time;
		int wind_detect;
		float wind_start_num;
		float wind_thrust_max;
		int   sys_autostart;
		float max_rotation;

	}		_params;

	TailsitterRecovery *_ts_opt_recovery{nullptr};	/**< Computes optimal rates for tailsitter recovery */

	/**
	 * Update our local parameter cache.
	 */
	void			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		battery_status_poll();
	void		parameter_update_poll();
	void		sensor_bias_poll();
	void		sensor_correction_poll();
	void		vehicle_attitude_poll();
	void		vehicle_attitude_setpoint_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_motor_limits_poll();
	void		vehicle_rates_setpoint_poll();
	void		vehicle_status_poll();
	void        mixer_switch_pool();
	void        vehicleLocalPosition_pool();
	void        actuator_outputs_pool();
	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Throttle PID attenuation.
	 */
	math::Vector<3> pid_attenuations(float tpa_breakpoint, float tpa_rate);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();

	/* add PID justice function*/
	void       pid_justice(float control_u, float e_ang, float e_rate, float euler_angles, float angle_sp);

	/* do action while pid tuning */
	void       pid_tune_on_action();

	/* move to next action item*/
	void       pid_tune_advance_action();

	/* detect wind disturbance*/
	void       wind_detect();

	/* detect wind level*/
	void       wind_level();
};

namespace mc_att_control
{

MulticopterAttitudeControl	*g_control;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_vehicle_status_sub(-1),
	_motor_limits_sub(-1),
	_battery_status_sub(-1),
	_sensor_correction_sub(-1),
	_sensor_bias_sub(-1),
	_mixer_switch_sub(-1),
	_vehicleLocalPosition_sub(-1),
	_actuator_outputs_sub(-1),
	_user_intention_sub(-1),

	/* gyro selection */
	_gyro_count(1),
	_selected_gyro(0),
	_time(0),
	_time_reached(0),
	_time_overshot(0),

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
	_pid_tune_pub(nullptr),
	_wind_estimate_pub(nullptr),
	_debug_vect_pub(nullptr),
	_rates_sp_id(nullptr),
	_actuators_id(nullptr),

	_actuators_0_circuit_breaker_enabled(false),

	_v_att{},
	_v_att_sp{},
	_v_rates_sp{},
	_manual_control_sp{},
	_v_control_mode{},
	_actuators{},
	_vehicle_status{},
	_controller_status{},
	_battery_status{},
	_sensor_gyro{},
	_sensor_correction{},
	_sensor_bias{},
	_pid_tune{},
	_mixer_switch{},
	_wind_estimate{},
	_vehicleLocalPosition{},
	_actuator_outputs{},
	_user_intention_msg{},
	_debug_vect_msg{},
	_saturation_status{},
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_vehicle_status.is_rotary_wing = true;

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_int_lim.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.auto_rate_max.zero();
	_params.acro_rate_max.zero();
	_params.rattitude_thres = 1.0f;

	_params.bat_scale_en = 0;

	_params.board_rotation = 0;

	_params.board_offset[0] = 0.0f;
	_params.board_offset[1] = 0.0f;
	_params.board_offset[2] = 0.0f;

	_params.mc_pid_autotune = 0;
	_params.mc_pid_adjust = 0;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();
	_board_rotation.identity();
	_e_ang.zero();
	_e_rate.zero();
	_euler_angles.zero();
	_euler_angles_sp.zero();
	_j_pid_tune = 0.0f;
	_ang_pre = 0.0f;
	_ang_act = 0.0f;
	_ang_max = 0.0f;
	_ang_sum = 0.0f;
	_R_sp_pid_tune.zero();
	_R_pid_tune.zero();
	_roll_sp_pid = 0.0f;
	_pitch_sp_pid = 0.0f;
	_yaw_sp_pid = 0.0f;
	_control_input = 0.0f;
	_e_euler_ang = 0.0f;
	_e_euler_rate = 0.0f;
	_euler_angles_input = 0.0f;
	_angle_sp = 0.0f;
	w1 = 1;
	w2 = 10;
	w3 = 10;
	w4 = 10;
	w5 = 0.05;
	_time_start_wind = 0;
	_time_warn = 0;
	_time_high_thrust = 0;
	_time_high_pwm = 0;
	pid_tune_stop = 0;


	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_integ_lim	= 	param_find("MC_RR_INT_LIM");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff		= 	param_find("MC_ROLLRATE_FF");

	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p		= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i		= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_integ_lim	= 	param_find("MC_PR_INT_LIM");
	_params_handles.pitch_rate_d		= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 		= 	param_find("MC_PITCHRATE_FF");

	_params_handles.tpa_breakpoint_p 	= 	param_find("MC_TPA_BREAK_P");
	_params_handles.tpa_breakpoint_i 	= 	param_find("MC_TPA_BREAK_I");
	_params_handles.tpa_breakpoint_d 	= 	param_find("MC_TPA_BREAK_D");
	_params_handles.tpa_rate_p	 	= 	param_find("MC_TPA_RATE_P");
	_params_handles.tpa_rate_i	 	= 	param_find("MC_TPA_RATE_I");
	_params_handles.tpa_rate_d	 	= 	param_find("MC_TPA_RATE_D");

	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_integ_lim	= 	param_find("MC_YR_INT_LIM");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");

	_params_handles.roll_rate_max		= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max		= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max		= 	param_find("MC_YAWRATE_MAX");
	_params_handles.yaw_auto_max		= 	param_find("MC_YAWRAUTO_MAX");

	_params_handles.acro_roll_max		= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max		= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max		= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.acro_expo		= 	param_find("MC_ACRO_EXPO");
	_params_handles.acro_superexpo		= 	param_find("MC_ACRO_SUPEXPO");

	_params_handles.rattitude_thres 	= 	param_find("MC_RATT_TH");

	_params_handles.roll_tc			= 	param_find("MC_ROLL_TC");
	_params_handles.pitch_tc		= 	param_find("MC_PITCH_TC");

	_params_handles.bat_scale_en		=	param_find("MC_BAT_SCALE_EN");

	/* rotations */
	_params_handles.board_rotation		=	param_find("SENS_BOARD_ROT");

	/* rotation offsets */
	_params_handles.board_offset[0]		=	param_find("SENS_BOARD_X_OFF");
	_params_handles.board_offset[1]		=	param_find("SENS_BOARD_Y_OFF");
	_params_handles.board_offset[2]		=	param_find("SENS_BOARD_Z_OFF");

	_params_handles.mc_pid_autotune = param_find("MC_PID_AUTOTUNE");
	_params_handles.mc_pid_adjust = param_find("MC_PID_ADJUST");
	_params_handles.mc_pid_axis = param_find("MC_PID_AXIS");
	_params_handles.mc_pid_angle_rp = param_find("MC_PID_ANGLE_RP");
	_params_handles.mc_pid_angle_y = param_find("MC_PID_ANGLE_Y");
	_params_handles.mc_pid_time = param_find("MC_PID_TIME");
	_params_handles.wind_detect = param_find("MC_WIND_DET");
	_params_handles.wind_start_num = param_find("MC_START_NUM");
	_params_handles.wind_thrust_max = param_find("MC_WIND_THRUST");
	_params_handles.sys_autostart  =  param_find("SYS_AUTOSTART");
	_params_handles.max_rotation = param_find("MC_MAX_ROTATION");

	_params_handles.ht_pitchrate_p      =   param_find("HT_PITCHRATE_P");
	_params_handles.ht_rollrate_p      =   param_find("HT_ROLLRATE_P");
	_params_handles.hts_pitchrate_p      =   param_find("HTS_PITCHRATE_P");
	_params_handles.hts_rollrate_p      =   param_find("HTS_ROLLRATE_P");

	/* fetch initial parameter values */
	parameters_update();

	/* define which axis in needed for pid tuning at the beginning*/
	if (_params.mc_pid_axis == 0) {
		_pid_tune_state = TUNE_NONE;

	} else if (_params.mc_pid_axis == 1) {
		_pid_tune_state = TUNE_ROLL_P;

	} else if (_params.mc_pid_axis == 2) {
		_pid_tune_state = TUNE_ROLL_N;

	} else if (_params.mc_pid_axis == 3) {
		_pid_tune_state = TUNE_PITCH_P;

	} else if (_params.mc_pid_axis == 4) {
		_pid_tune_state = TUNE_PITCH_N;

	} else if (_params.mc_pid_axis == 5) {
		_pid_tune_state = TUNE_YAW_CCW;

	} else if (_params.mc_pid_axis == 6) {
		_pid_tune_state = TUNE_YAW_CW;
	}

	if (_params.vtol_type == 0 && _params.vtol_opt_recovery_enabled) {
		// the vehicle is a tailsitter, use optimal recovery control strategy
		_ts_opt_recovery = new TailsitterRecovery();
	}

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	delete _ts_opt_recovery;

	mc_att_control::g_control = nullptr;
}

void
MulticopterAttitudeControl::parameters_update()
{
	float v;

	float roll_tc, pitch_tc;

	param_get(_params_handles.roll_tc, &roll_tc);
	param_get(_params_handles.pitch_tc, &pitch_tc);

	if (_mixer_switch.mixer_switch_enable && _mixer_switch.motor_stop_num > 0 && _mixer_switch.motor_stop_num < 6) {
		if (_mixer_switch.motor_stop_num == 1) {
			param_get(_params_handles.ht_pitchrate_p, &v);
			_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
			param_get(_params_handles.ht_rollrate_p, &v);
			_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
		}

		if (_mixer_switch.motor_stop_num == 8) {
			param_get(_params_handles.hts_pitchrate_p, &v);
			_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
			param_get(_params_handles.hts_rollrate_p, &v);
			_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
		}

	} else {
		param_get(_params_handles.roll_rate_p, &v);
		_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
		param_get(_params_handles.pitch_rate_p, &v);
		_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	}

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_integ_lim, &v);
	_params.rate_int_lim(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_integ_lim, &v);
	_params.rate_int_lim(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	param_get(_params_handles.tpa_breakpoint_p, &_params.tpa_breakpoint_p);
	param_get(_params_handles.tpa_breakpoint_i, &_params.tpa_breakpoint_i);
	param_get(_params_handles.tpa_breakpoint_d, &_params.tpa_breakpoint_d);
	param_get(_params_handles.tpa_rate_p, &_params.tpa_rate_p);
	param_get(_params_handles.tpa_rate_i, &_params.tpa_rate_i);
	param_get(_params_handles.tpa_rate_d, &_params.tpa_rate_d);

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_integ_lim, &v);
	_params.rate_int_lim(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* auto angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
	_params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

	/* manual rate control acro mode rate limits and expo */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);
	param_get(_params_handles.acro_expo, &_params.acro_expo);
	param_get(_params_handles.acro_superexpo, &_params.acro_superexpo);

	/* stick deflection needed in rattitude mode to control rates not angles */
	param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

	if (_vehicle_status.is_vtol) {
		param_get(_params_handles.vtol_type, &_params.vtol_type);

		int32_t tmp;
		param_get(_params_handles.vtol_opt_recovery_enabled, &tmp);
		_params.vtol_opt_recovery_enabled = (tmp == 1);

		param_get(_params_handles.vtol_wv_yaw_rate_scale, &_params.vtol_wv_yaw_rate_scale);

	} else {
		_params.vtol_opt_recovery_enabled = false;
		_params.vtol_wv_yaw_rate_scale = 0.f;
	}

	param_get(_params_handles.bat_scale_en, &_params.bat_scale_en);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* rotation of the autopilot relative to the body */
	param_get(_params_handles.board_rotation, &(_params.board_rotation));

	/* fine adjustment of the rotation */
	param_get(_params_handles.board_offset[0], &(_params.board_offset[0]));
	param_get(_params_handles.board_offset[1], &(_params.board_offset[1]));
	param_get(_params_handles.board_offset[2], &(_params.board_offset[2]));

	/* get transformation matrix from sensor/board to body frame */
	get_rot_matrix((enum Rotation)_params.board_rotation, &_board_rotation);

	/* fine tune the rotation */
	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _params.board_offset[0],
					 M_DEG_TO_RAD_F * _params.board_offset[1],
					 M_DEG_TO_RAD_F * _params.board_offset[2]);
	_board_rotation = board_rotation_offset * _board_rotation;
	param_get(_params_handles.mc_pid_autotune, &(_params.mc_pid_autotune));
	param_get(_params_handles.mc_pid_adjust, &(_params.mc_pid_adjust));
	param_get(_params_handles.mc_pid_axis, &(_params.mc_pid_axis));
	param_get(_params_handles.mc_pid_angle_rp, &(_params.mc_pid_angle_rp));
	param_get(_params_handles.mc_pid_angle_y, &(_params.mc_pid_angle_y));
	param_get(_params_handles.mc_pid_time, &(_params.mc_pid_time));
	param_get(_params_handles.wind_detect, &(_params.wind_detect));
	param_get(_params_handles.wind_start_num, &(_params.wind_start_num));
	param_get(_params_handles.wind_thrust_max, &(_params.wind_thrust_max));
	param_get(_params_handles.sys_autostart, &(_params.sys_autostart));
	param_get(_params_handles.max_rotation, &(_params.max_rotation));
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_rates_sp_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

				_params_handles.vtol_type = param_find("VT_TYPE");
				_params_handles.vtol_opt_recovery_enabled = param_find("VT_OPT_RECOV_EN");
				_params_handles.vtol_wv_yaw_rate_scale = param_find("VT_WV_YAWR_SCL");

				parameters_update();

				if (_params.vtol_type == 0 && _params.vtol_opt_recovery_enabled) {
					// the vehicle is a tailsitter, use optimal recovery control strategy
					_ts_opt_recovery = new TailsitterRecovery();
				}

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterAttitudeControl::mixer_switch_pool()
{
	bool updated;
	orb_check(_mixer_switch_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(mixer_switch), _mixer_switch_sub, &_mixer_switch);
	}
}

void
MulticopterAttitudeControl::vehicleLocalPosition_pool()
{
	bool updated;
	orb_check(_vehicleLocalPosition_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _vehicleLocalPosition_sub, &_vehicleLocalPosition);
	}
}


void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		multirotor_motor_limits_s motor_limits = {};
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &motor_limits);

		_saturation_status.value = motor_limits.saturation_status;
	}
}

void
MulticopterAttitudeControl::actuator_outputs_pool()
{
	bool updated;
	orb_check(_actuator_outputs_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);
	}
}

void
MulticopterAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

void
MulticopterAttitudeControl::sensor_correction_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_correction_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
	}

	/* update the latest gyro selection */
	if (_sensor_correction.selected_gyro_instance < _gyro_count) {
		_selected_gyro = _sensor_correction.selected_gyro_instance;
	}
}

void
MulticopterAttitudeControl::sensor_bias_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_bias_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_bias), _sensor_bias_sub, &_sensor_bias);
	}

}

void
MulticopterAttitudeControl::pid_tune_advance_action()
{
	/* check which axis is needed for pid tuning and init some value*/
	switch (_pid_tune_state) {
	case TUNE_NONE:
		if (pid_tune_stop) {
			_time = 0;
			_time_reached = 0;
			_time_overshot = 0;
			pid_tune_stop = 0;
			_j_pid_tune = 0.0f;
			_params.mc_pid_autotune = 0;
			_ang_sum = 0.0f;

		}

		break;

	case TUNE_ROLL_P:
		if (pid_tune_stop) {
			_pid_tune_state = TUNE_ROLL_N;
			_time = 0;
			_time_reached = 0;
			_time_overshot = 0;
			pid_tune_stop = 0;
			_j_pid_tune = 0.0f;
			_ang_sum = 0.0f;

		}

		break;

	case TUNE_ROLL_N:
		if (pid_tune_stop) {
			_pid_tune_state = TUNE_ROLL_P;
			_time = 0;
			_time_reached = 0;
			_time_overshot = 0;
			pid_tune_stop = 0;
			_j_pid_tune = 0.0f;
			_ang_sum = 0.0f;

		}

		break;

	case TUNE_PITCH_P:
		if (pid_tune_stop) {
			_pid_tune_state = TUNE_PITCH_N;
			_time = 0;
			_time_reached = 0;
			_time_overshot = 0;
			pid_tune_stop = 0;
			_j_pid_tune = 0.0f;
			_ang_sum = 0.0f;

		}

		break;

	case TUNE_PITCH_N:
		if (pid_tune_stop) {
			_pid_tune_state = TUNE_PITCH_P;
			_time = 0;
			_time_reached = 0;
			_time_overshot = 0;
			pid_tune_stop = 0;
			_j_pid_tune = 0.0f;
			_ang_sum = 0.0f;

		}

		break;

	case TUNE_YAW_CW:
		if (pid_tune_stop) {
			_pid_tune_state = TUNE_YAW_CCW;
			_time = 0;
			pid_tune_stop = 0;
			_time_reached = 0;
			_time_overshot = 0;
			_j_pid_tune = 0.0f;
			_ang_sum = 0.0f;

		}

		break;

	case TUNE_YAW_CCW:
		if (pid_tune_stop) {
			_pid_tune_state = TUNE_YAW_CW;
			_time = 0;
			pid_tune_stop = 0;
			_time_reached = 0;
			_time_overshot = 0;
			_j_pid_tune = 0.0f;
			_ang_sum = 0.0f;
		}

		break;

	default:
		break;
	}
}

void
MulticopterAttitudeControl::pid_tune_on_action()
{
	switch (_pid_tune_state) {
	case TUNE_NONE:
		pid_tune_stop = 1;
		break;

	case TUNE_ROLL_N:
	case TUNE_ROLL_P:
		_time++;

		/* begin tuning*/
		if (_time == PID_TUNE_START_TIME) {
			mavlink_log_critical(&mavlink_log_pub, "tune roll start");
		}

		if (_time > PID_TUNE_START_TIME) {
			// give attitude setpoint.
			if (_pid_tune_state == TUNE_ROLL_P) {
				_roll_sp_pid = _params.mc_pid_angle_rp * 3.14f / 180.0f;

			} else {
				_roll_sp_pid = -_params.mc_pid_angle_rp * 3.14f / 180.0f;
			}

			_pitch_sp_pid = _euler_angles_sp(1);
			_yaw_sp_pid = _euler_angles_sp(2);

			_e_euler_ang = _e_ang(0);
			_e_euler_rate = _e_rate(0);
			_euler_angles_input = _euler_angles(0);
			_control_input = _actuators.control[0] * _actuators.control[0];
			_angle_sp = _roll_sp_pid;
		}

		break;

	case TUNE_PITCH_P:
	case TUNE_PITCH_N:
		_time++;

		if (_time == PID_TUNE_START_TIME) {
			mavlink_log_critical(&mavlink_log_pub, "tune pitch start");
		}

		if (_time > PID_TUNE_START_TIME) {
			_roll_sp_pid = _euler_angles_sp(0);

			if (_pid_tune_state == TUNE_PITCH_P) {
				_pitch_sp_pid = _params.mc_pid_angle_rp * 3.14f / 180.0f;

			} else {
				_pitch_sp_pid = -_params.mc_pid_angle_rp * 3.14f / 180.0f;
			}

			_yaw_sp_pid = _euler_angles_sp(2);

			_e_euler_ang = _e_ang(1);
			_e_euler_rate = _e_rate(1);
			_euler_angles_input = _euler_angles(1);
			_control_input = _actuators.control[1] * _actuators.control[1];
			_angle_sp = _pitch_sp_pid;
		}

		break;

	case TUNE_YAW_CW:
	case TUNE_YAW_CCW:
		_time++;

		if (_time == PID_TUNE_START_TIME) {
			mavlink_log_critical(&mavlink_log_pub, "tune yaw start");
		}

		if (_time > PID_TUNE_START_TIME) {
			_roll_sp_pid = _euler_angles_sp(0);
			_pitch_sp_pid = _euler_angles_sp(1);

			if (_pid_tune_state == TUNE_YAW_CW) {
				_yaw_sp_pid = _params.mc_pid_angle_y * 3.14f / 180.0f;

			} else {
				_yaw_sp_pid = _params.mc_pid_angle_y * 3.14f / 180.0f;
			}

			_e_euler_ang = _e_ang(2);
			_e_euler_rate = _e_rate(2);
			_euler_angles_input = _euler_angles(2);
			_control_input = _actuators.control[2] * _actuators.control[2];
			_angle_sp = _yaw_sp_pid;
		}

		break;

	}

	if (_time > PID_TUNE_START_TIME) {
		_R_sp_pid_tune.from_euler(_roll_sp_pid, _pitch_sp_pid, _yaw_sp_pid);

		// calculate PID performance value.
		pid_justice(_control_input, _e_euler_ang, _e_euler_rate, _euler_angles_input, _angle_sp);
	}

	if (_time > _params.mc_pid_time) {
		pid_tune_stop = 1;
	}
}

void
MulticopterAttitudeControl::pid_justice(float control_u, float e_ang, float e_rate, float euler_angles, float angle_sp)
{
	if (_time > _params.mc_pid_time) {
		return;
	}

	_ang_act = euler_angles;
	float error_ang_bet;
	error_ang_bet = _ang_act - _ang_pre;

	_j_pid_tune += w1 * fabsf(e_rate) + w2 * fabsf(e_ang) + w3 * control_u ;

	if (angle_sp >= 0) {

		if (_ang_max < _ang_act) {  //  max angle
			_ang_max = _ang_act;
		}

		if (_ang_act  > 0.9f * angle_sp && _ang_act  < 1.1f * angle_sp && !_time_reached) { //  reached time
			_time_reached = _time;
			_j_pid_tune += w5 * _time_reached;
			mavlink_log_critical(&mavlink_log_pub, "time reached = %d", _time_reached);
		}

		if (error_ang_bet  < 0.0f && _ang_act > angle_sp) {  // overshot;
			_j_pid_tune  += w4 * fabsf(error_ang_bet);

			if (!_time_overshot) {
				_time_overshot = _time;
				mavlink_log_critical(&mavlink_log_pub, "roll overshot = %.2f", (double)((_ang_act - angle_sp) / angle_sp));
			}
		}

	} else {

		if (_ang_max > _ang_act) {
			_ang_max = _ang_act;
		}

		if (_ang_act  < 0.9f * angle_sp && _ang_act  > 1.1f * angle_sp && !_time_reached) {
			_time_reached = _time;
			_j_pid_tune += w5 * _time_reached;
			mavlink_log_critical(&mavlink_log_pub, "time reached = %d", _time_reached);
		}

		if (error_ang_bet  > 0.0f && _ang_act < angle_sp) {  // overshot;
			if (!_time_overshot) {
				_time_overshot = _time;
				mavlink_log_critical(&mavlink_log_pub, "angle overshot = %.2f", (double)((_ang_act - angle_sp) / angle_sp));
			}

			_j_pid_tune  += w4 * fabsf(error_ang_bet);
		}
	}

	if (_time > _time_reached && _time < _params.mc_pid_time) {
		_ang_sum += _ang_act;
	}

	// some adjustments for pid tuning.
	if (_time == _params.mc_pid_time) {
		mavlink_log_critical(&mavlink_log_pub, "max= %.2f  average= %.2f J= %.2f", (double)(_ang_max * 180.0f / 3.14f),
				     (double)(_ang_sum / (_time - _time_reached) * 180.0f / 3.14f), (double)_j_pid_tune);

		if (fabsf(_ang_max) > 0.9f * fabsf(angle_sp) && fabsf(_ang_max) < 1.1f * fabsf(angle_sp) && _time_reached < 1000) {
			mavlink_log_critical(&mavlink_log_pub, "Nice P and nice D");

			if (fabsf(_ang_sum / (_time - _time_reached)) > 0.9f * fabsf(angle_sp)
			    && fabsf(_ang_sum / (_time - _time_reached)) < 1.1f * fabsf(angle_sp)) {
				mavlink_log_critical(&mavlink_log_pub, "Nice I");

			} else if (fabsf(_ang_sum / (_time - _time_reached)) < 0.9f * fabsf(angle_sp)) {
				mavlink_log_critical(&mavlink_log_pub, "Increase I");

			} else if (fabsf(_ang_sum / (_time - _time_reached)) > 1.1f * fabsf(angle_sp)) {
				mavlink_log_critical(&mavlink_log_pub, "Decrease I");
			}

		} else if (fabsf(_ang_max) < 0.9f * fabsf(angle_sp) || _time_reached > 1000) {
			mavlink_log_critical(&mavlink_log_pub, "increase angle rate p");

		} else if (fabsf(_ang_max) > 1.1f * fabsf(angle_sp) || _time_reached < 550) {
			mavlink_log_critical(&mavlink_log_pub, "decrease angle rate p");
		}
	}

	_ang_pre = _ang_act;

	_pid_tune.j_pid_tune = _j_pid_tune;
	_pid_tune.e_rate = e_rate;
	_pid_tune.e_ang = e_ang;
	_pid_tune.control_u = control_u;
	_pid_tune.error_ang_bet = error_ang_bet;
	_pid_tune.ang_act = _ang_act;
	_pid_tune.angle_sp = angle_sp;
	_pid_tune.ang_pre = _ang_pre;

	// publish pid profermance.
	if (_pid_tune_pub != nullptr) {
		orb_publish(ORB_ID(pid_auto_tune), _pid_tune_pub, &_pid_tune);

	} else {
		_pid_tune_pub = orb_advertise(ORB_ID(pid_auto_tune), &_pid_tune);
	}

}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();
	_euler_angles = q_att.to_euler();
	_euler_angles_sp = q_sp.to_euler();
	_R_pid_tune = R;
	/* construct attitude setpoint rotation matrix */
	math::Matrix<3, 3> R_sp;

	// failsafe program.  Reset all PID parameters to the default if turns to manual mode or RTL.
	if ((_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL
	     || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) && _params.mc_pid_adjust == 1) {
		PX4_INFO("run");
		_params.att_p(0) = 6.5f;
		_params.rate_p(0) = 0.15f;
		_params.rate_i(0) = 0.05f;
		_params.rate_d(0) = 0.003f;

		_params.att_p(1) = 6.5f;
		_params.rate_p(1) = 0.15f;
		_params.rate_i(1) = 0.05f;
		_params.rate_d(1) = 0.003f;

		_params.att_p(2) = 2.8f;
		_params.rate_p(2) = 0.2f;
		_params.rate_i(2) = 0.1f;
		_params.rate_d(2) = 0.00f;
		_params.rate_ff(2) = 0.0f;
	} // failsafe program.

	if (_params.mc_pid_autotune && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL
					|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL) && _v_control_mode.flag_armed) {
		pid_tune_advance_action();
		pid_tune_on_action();
		R_sp = _R_sp_pid_tune;

	} else {
		R_sp = q_sp.to_dcm();
		_time = 0;
	}


	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation (indexes: 0=pitch, 1=roll, 2=yaw).
	 * This is for roll/pitch only (tilt), e_R(2) is 0 */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length(); // == sin(tilt angle error)
	float e_R_z_cos = R_z * R_sp_z; // == cos(tilt angle error) == (R.transposed() * R_sp)(2, 2)

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp have the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));

	/* calculate the weight for yaw control
	 * Make the weight depend on the tilt angle error: the higher the error of roll and/or pitch, the lower
	 * the weight that we use to control the yaw. This gives precedence to roll & pitch correction.
	 * The weight is 1 if there is no tilt error.
	 */
	float yaw_w = e_R_z_cos * e_R_z_cos;

	/* calculate the angle between R_rp_x and R_sp_x (yaw angle error), and apply the yaw weight */
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q_error;
		q_error.from_dcm(R.transposed() * R_sp);
		math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f : -q_error.imag() * 2.0f;

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R);  // angle error
	_e_ang = e_R;


	/* Feed forward the yaw setpoint rate. We need to transform the yaw from world into body frame.
	 * The following is a simplification of:
	 * R.transposed() * math::Vector<3>(0.f, 0.f, _v_att_sp.yaw_sp_move_rate * _params.yaw_ff)
	 */
	math::Vector<3> yaw_feedforward_rate(R(2, 0), R(2, 1), R(2, 2));
	yaw_feedforward_rate *= _v_att_sp.yaw_sp_move_rate * _params.yaw_ff;

	yaw_feedforward_rate(2) *= yaw_w;
	_rates_sp += yaw_feedforward_rate;


	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
		    !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));

		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}
	}

	/* VTOL weather-vane mode, dampen yaw rate */
	if (_vehicle_status.is_vtol && _v_att_sp.disable_mc_yaw_control) {
		if (_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) {

			const float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale;
			_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);

			// prevent integrator winding up in weathervane mode
			_rates_int(2) = 0.0f;
		}
	}
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
math::Vector<3>
MulticopterAttitudeControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	math::Vector<3> pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_v_control_mode.flag_armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	// get the raw gyro data and correct for thermal errors
	math::Vector<3> rates;

	if (_selected_gyro == 0) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		rates(0) = _sensor_gyro.x;
		rates(1) = _sensor_gyro.y;
		rates(2) = _sensor_gyro.z;
	}

	// rotate corrected measurements from sensor to body frame
	rates = _board_rotation * rates;

	// correct for in-run bias errors
	rates(0) -= _sensor_bias.gyro_x_bias;
	rates(1) -= _sensor_bias.gyro_y_bias;
	rates(2) -= _sensor_bias.gyro_z_bias;

	if (_mixer_switch.mixer_switch_enable && _mixer_switch.motor_stop_num > 0 && _mixer_switch.motor_stop_num < 6) {
		float v;
		float roll_tc, pitch_tc;

		param_get(_params_handles.roll_tc, &roll_tc);
		param_get(_params_handles.pitch_tc, &pitch_tc);

		if (_mixer_switch.motor_stop_num == 1) {
			param_get(_params_handles.ht_pitchrate_p, &v);
			_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
			param_get(_params_handles.ht_rollrate_p, &v);
			_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
		}

		if (_mixer_switch.motor_stop_num == 8) {
			param_get(_params_handles.hts_pitchrate_p, &v);
			_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
			param_get(_params_handles.hts_rollrate_p, &v);
			_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
		}
	}

	math::Vector<3> rates_p_scaled = _params.rate_p.emult(pid_attenuations(_params.tpa_breakpoint_p, _params.tpa_rate_p));
	//math::Vector<3> rates_i_scaled = _params.rate_i.emult(pid_attenuations(_params.tpa_breakpoint_i, _params.tpa_rate_i));
	math::Vector<3> rates_d_scaled = _params.rate_d.emult(pid_attenuations(_params.tpa_breakpoint_d, _params.tpa_rate_d));

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;  // rates error
	_e_rate = rates_err;

	_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int +
		       rates_d_scaled.emult(_rates_prev - rates) / dt +
		       _params.rate_ff.emult(_rates_sp);

	_rates_sp_prev = _rates_sp;
	_rates_prev = rates;

	/* update integral only if motors are providing enough thrust to be effective */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);

			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);

			}

			// Perform the integration using a first order method and do not propaate the result if out of range or invalid
			float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_params.rate_int_lim(i) && rate_i < _params.rate_int_lim(i)) {
				_rates_int(i) = rate_i;

			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_params.rate_int_lim(i), _params.rate_int_lim(i));

	}
}

void
MulticopterAttitudeControl::wind_level()
{
	math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	math::Vector<3> euler_angles;
	euler_angles = q_att.to_euler();

	float roll_pitch_angle;
	roll_pitch_angle = sqrtf(euler_angles(0) * euler_angles(0) + euler_angles(1) * euler_angles(1));

	if (roll_pitch_angle * 180.0f / 3.14f > _params.wind_start_num
	    && roll_pitch_angle * 180.0f / 3.14f < (_params.wind_start_num + 2.0f)) {
		_wind_estimate.windlevel_horiz = 3.0f;

	} else if (roll_pitch_angle * 180.0f / 3.14f > (_params.wind_start_num + 2.0f)
		   && roll_pitch_angle * 180.0f / 3.14f < (_params.wind_start_num + 4.0f)) {
		_wind_estimate.windlevel_horiz = 4.0f;

	} else if (roll_pitch_angle * 180.0f / 3.14f > (_params.wind_start_num + 4.0f)
		   && roll_pitch_angle * 180.0f / 3.14f < (_params.wind_start_num + 6.0f)) {
		_time_start_wind++;

		/*		if (_time_start_wind % 250 == 1 || !_time_warn) {
					_time_warn = 1;*/
		if (_time_start_wind % 250 == 1) {
			mavlink_log_critical(&mavlink_log_pub, "Big wind, please fly with caution ");
		}

		_wind_estimate.windlevel_horiz = 5.0f;

	} else if (roll_pitch_angle * 180.0f / 3.14f > (_params.wind_start_num + 6.0f)) {
		_time_start_wind++;

		if (_time_start_wind % 250 == 1) {
//		if (_time_start_wind % 250 == 1 || !_time_warn) {
//			_time_warn = 1;
			mavlink_log_critical(&mavlink_log_pub, "Critical wind, land advise");
		}

		_wind_estimate.windlevel_horiz = 6.0f;

	} else if (roll_pitch_angle * 180.0f / 3.14f < _params.wind_start_num) {
		_time_start_wind = 0;
		_time_warn = 0;
		_wind_estimate.windlevel_horiz = 2.0f;
	}

	// publish wind estimate.
	if (_wind_estimate_pub != nullptr) {
		orb_publish(ORB_ID(wind_estimate), _wind_estimate_pub, &_wind_estimate);

	} else {
		_wind_estimate_pub = orb_advertise(ORB_ID(wind_estimate), &_wind_estimate);
	}

}

void
MulticopterAttitudeControl::wind_detect()
{
	bool updated;
	orb_check(_user_intention_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(user_intention), _user_intention_sub, &_user_intention_msg);
	}

	math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	math::Vector<3> euler_angles;
	euler_angles = q_att.to_euler();

//    PX4_INFO("user_intention_xy = %d, intention = %d, brake = %d", _user_intention_msg.user_intention_xy, _user_intention_msg.intention, _user_intention_msg.brake);
	bool rotating = ((fabsf(_v_att.rollspeed)  > _params.max_rotation) ||
			 (fabsf(_v_att.pitchspeed) > _params.max_rotation) ||
			 (fabsf(_v_att.yawspeed) > _params.max_rotation));/* || ((fabsf(euler_angles(0))  > 0.1f) ||
					 (fabsf(euler_angles(1)) > 0.1f));*/

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
	    || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
	    || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
	    || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET
	    || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) {
		math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
		math::Vector<3> euler_mission_sp;
		euler_mission_sp = q_sp.to_euler();
		float roll_pitch_angle_sp;
		roll_pitch_angle_sp = sqrtf(euler_mission_sp(0) * euler_mission_sp(0) + euler_mission_sp(1) * euler_mission_sp(1));
		_debug_vect_msg.x = (float)rotating;

		if (_debug_vect_pub == nullptr) {
			_debug_vect_pub = orb_advertise(ORB_ID(debug_vect), &_debug_vect_msg);

		} else {
			orb_publish(ORB_ID(debug_vect), _debug_vect_pub, &_debug_vect_msg);
		}

		// if roll pitch setpoint is small, check the real roll pitch angle.
		if ((roll_pitch_angle_sp * 180.0f / 3.14f) < 2.0f && !rotating) {
			wind_level();
		}
	}

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL) {
		// TODO check if we detect wind when rotating or not rotating.
		// !_user_intention_msg.user_intention_xy means we detect wind when the drone is braked when we don't give any commander.
		// !_user_intention_msg.brake means we don't detect wind when the drone is braking when the manual sticks goes to the middle.
		// !rotating means we don't detect wind when the drone is rotating when we check flight mode from manual to position.
		// SO the condition we detect wind when we don't give any commander thought GCS/RC or the drone isn't braking or rotating.
		// The level of wind is determined by the angle of the multirotor at that condition;
		if (!_user_intention_msg.user_intention_xy && !_user_intention_msg.brake && !rotating) {
			wind_level();
		}
	}

	// check if thrust or PWM of motors is so high that the uav cannot finish the mission.
	if (_vehicleLocalPosition.z_valid && fabsf(_vehicleLocalPosition.az) < 0.2f &&
	    fabsf(_vehicleLocalPosition.vz) < 0.2f) {
		if (_v_att_sp.thrust > _params.wind_thrust_max) {
			_time_high_thrust++;

			if (_time_high_thrust % 500 == 1) {
				mavlink_log_critical(&mavlink_log_pub, "high thrust hover, land advise");
			}
		}

		for (int i = 0; i < _params.sys_autostart / 1000 + 1; i++) {
			_time_high_pwm++;

			if (_actuator_outputs.output[i] > 1850.0f && _time_high_pwm % 500 == 1) {
				mavlink_log_critical(&mavlink_log_pub, "high motor pwm");
			}
		}
	}
}

void
MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control::g_control->task_main();
}

void
MulticopterAttitudeControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	_mixer_switch_sub   = orb_subscribe(ORB_ID(mixer_switch));
	_vehicleLocalPosition_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_actuator_outputs_sub  = orb_subscribe(ORB_ID(actuator_outputs));
	_user_intention_sub  = orb_subscribe(ORB_ID(user_intention));

	_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

	if (_gyro_count == 0) {
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));
	_sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));

	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	while (!_task_should_exit) {

		if (_params.wind_detect) {
			wind_detect();
		}

		poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("mc att ctrl: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy gyro data */
			orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();
			battery_status_poll();
			vehicle_attitude_poll();
			sensor_correction_poll();
			sensor_bias_poll();
			mixer_switch_pool();
			vehicleLocalPosition_pool();
			actuator_outputs_pool();

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control).  If both are true don't
			 * even bother running the attitude controllers */
			if (_v_control_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				    fabsf(_manual_control_sp.x) > _params.rattitude_thres) {
					_v_control_mode.flag_control_attitude_enabled = false;
				}
			}

			if (_v_control_mode.flag_control_attitude_enabled) {

				if (_ts_opt_recovery == nullptr) {
					// the  tailsitter recovery instance has not been created, thus, the vehicle
					// is not a tailsitter, do normal attitude control
					control_attitude(dt);

				} else {
					vehicle_attitude_setpoint_poll();
					_thrust_sp = _v_att_sp.thrust;
					math::Quaternion q(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
					math::Quaternion q_sp(&_v_att_sp.q_d[0]);
					_ts_opt_recovery->setAttGains(_params.att_p, _params.yaw_ff);
					_ts_opt_recovery->calcOptimalRates(q, q_sp, _v_att_sp.yaw_sp_move_rate, _rates_sp);

					/* limit rates */
					for (int i = 0; i < 3; i++) {
						_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
					}
				}

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					matrix::Vector3f man_rate_sp;
					man_rate_sp(0) = math::superexpo(_manual_control_sp.y, _params.acro_expo, _params.acro_superexpo);
					man_rate_sp(1) = math::superexpo(-_manual_control_sp.x, _params.acro_expo, _params.acro_superexpo);
					man_rate_sp(2) = math::superexpo(_manual_control_sp.r, _params.acro_expo, _params.acro_superexpo);
					man_rate_sp = man_rate_sp.emult(_params.acro_rate_max);
					_rates_sp = math::Vector<3>(man_rate_sp.data());
					_thrust_sp = _manual_control_sp.z;

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				/* publish actuator controls */
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.control[7] = _v_att_sp.landing_gear;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _sensor_gyro.timestamp;

//				 PX4_INFO("_actuators.control[0] = %.2f", (double)_actuators.control[0]);
				/* scale effort by battery status */
				if (_params.bat_scale_en && _battery_status.scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						_actuators.control[i] *= _battery_status.scale;
					}
				}

				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {

						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}

				/* publish controller status */
				if (_controller_status_pub != nullptr) {
					orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
			}


			if (_v_control_mode.flag_control_termination_enabled) {
				if (!_vehicle_status.is_vtol) {

					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();

					/* publish actuator controls */
					_actuators.control[0] = 0.0f;
					_actuators.control[1] = 0.0f;
					_actuators.control[2] = 0.0f;
					_actuators.control[3] = 0.0f;
					_actuators.timestamp = hrt_absolute_time();
					_actuators.timestamp_sample = _sensor_gyro.timestamp;

					if (!_actuators_0_circuit_breaker_enabled) {
						if (_actuators_0_pub != nullptr) {

							orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
							perf_end(_controller_latency_perf);

						} else if (_actuators_id) {
							_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
						}
					}

					_controller_status.roll_rate_integ = _rates_int(0);
					_controller_status.pitch_rate_integ = _rates_int(1);
					_controller_status.yaw_rate_integ = _rates_int(2);
					_controller_status.timestamp = hrt_absolute_time();

					/* publish controller status */
					if (_controller_status_pub != nullptr) {
						orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

					} else {
						_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
					}

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}
				}
			}
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
}

int
MulticopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ATTITUDE_CONTROL,
					   2100,    // bigger stack size
					   (px4_main_t)&MulticopterAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (mc_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		mc_att_control::g_control = new MulticopterAttitudeControl;

		if (mc_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != mc_att_control::g_control->start()) {
			delete mc_att_control::g_control;
			mc_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete mc_att_control::g_control;
		mc_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
