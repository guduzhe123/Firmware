clear all;

%% def
def.MIN_TAKEOFF_THRUST = 0.2;
def.TPA_RATE_LOWER_LIMIT = 0.05;
def.ATTITUDE_TC_DEFAULT = 0.2;

def.AXIS_INDEX_ROLL = 1;
def.AXIS_INDEX_PITCH = 2;
def.AXIS_INDEX_YAW = 3;
def.AXIS_COUNT = 3;

def.MAX_GYRO_COUNT = 3;

def.M_PI_F = 3.14159265;
def.M_TWOPI_F = 6.28318531;
def.SIGMA_SINGLE_OP = 0.000001;
def.SIGMA_NORM = 0.001;
def.CONSTANTS_ONE_G = 9.80665;

%% para_handle_pos
parH_pos.thr_min = 0.12;
parH_pos.thr_max = 0.9;
parH_pos.thr_hover = 0.5;
parH_pos.z_p = 1.0;
parH_pos.z_vel_p = 0.6;
parH_pos.z_vel_i = 0.15;
parH_pos.z_vel_d = 0;
parH_pos.z_vel_max_up = 3.0;
parH_pos.z_vel_max_down = 1.0;
parH_pos.slow_land_alt1 = 10.0;
parH_pos.slow_land_alt2 = 5.0;
parH_pos.xy_p = 0.95;
parH_pos.xy_vel_p = 0.09;
parH_pos.xy_vel_i = 0.02;
parH_pos.xy_vel_d = 0.01;
parH_pos.xy_vel_max = 12.0;
parH_pos.xy_vel_cruise = 5.0;
parH_pos.tilt_max_air = 45.0;
parH_pos.land_speed = 0.7;
parH_pos.tko_speed = 1.5;
parH_pos.tilt_max_land = 12.0;
parH_pos.man_tilt_max = 35;
parH_pos.man_yaw_max = 200;
parH_pos.global_yaw_max = 200;
parH_pos.mc_att_yaw_p = 2.8;
parH_pos.hold_max_xy = 0.8;
parH_pos.hold_max_z = 0.6;
parH_pos.alt_mode = 0;
parH_pos.opt_recover = 0;
parH_pos.rc_flt_smp_rate = 10.0;
parH_pos.rc_flt_cutoff = 50;

parH_pos.acc_up_max = 10;
parH_pos.acc_down_max = 10;

parH_pos.nav_acc_rad = 2;
parH_pos.mis_yaw_error = 12;
parH_pos.acceleration_hor = 5;
parH_pos.takeoff_ramp_time = 0.4;
parH_pos.acceleration_hor_max = 10;
parH_pos.cruise_speed_90 = 3;
%% par
par_pos.thr_min = parH_pos.thr_min;
par_pos.thr_max = parH_pos.thr_max;
par_pos.thr_hover = constrain(parH_pos.thr_hover,parH_pos.thr_min,parH_pos.thr_max);
par_pos.tilt_max_air = parH_pos.tilt_max_air*pi/180;
par_pos.tilt_max_land = parH_pos.tilt_max_land*pi/180;

par_pos.man_tilt_max = parH_pos.man_tilt_max*pi/180;
par_pos.man_yaw_max = parH_pos.man_yaw_max*pi/180;
par_pos.global_yaw_max = parH_pos.global_yaw_max*pi/180;
par_pos.mc_att_yaw_p = parH_pos.mc_att_yaw_p;

par_pos.hold_max_xy = max(0,parH_pos.hold_max_xy);
par_pos.hold_max_z = max(0,parH_pos.hold_max_z);
par_pos.vel_max_xy = parH_pos.xy_vel_max;
par_pos.vel_cruise_xy = parH_pos.xy_vel_cruise;
par_pos.vel_max_up = parH_pos.z_vel_max_up;
par_pos.vel_max_down = parH_pos.z_vel_max_down;
par_pos.slow_land_alt1 = parH_pos.slow_land_alt1;
par_pos.slow_land_alt2 = parH_pos.slow_land_alt2;
par_pos.alt_mode = parH_pos.alt_mode;
par_pos.opt_recover = parH_pos.opt_recover;
par_pos.rc_flt_smp_rate = max(1.0, parH_pos.rc_flt_smp_rate);
par_pos.rc_flt_cutoff = constrain(parH_pos.rc_flt_cutoff, 0.1, (par_pos.rc_flt_smp_rate / 2.0) - 1);

par_pos.land_speed = min(parH_pos.land_speed,par_pos.vel_max_down);
par_pos.tko_speed = min(parH_pos.tko_speed,par_pos.vel_max_up);

par_pos.pos_p = [parH_pos.xy_p,parH_pos.xy_p,parH_pos.z_p]';
par_pos.vel_p = [parH_pos.xy_vel_p,parH_pos.xy_vel_p,parH_pos.z_vel_p]';
par_pos.vel_i = [parH_pos.xy_vel_i,parH_pos.xy_vel_i,parH_pos.z_vel_i]';
par_pos.vel_d = [parH_pos.xy_vel_d,parH_pos.xy_vel_d,parH_pos.z_vel_d]';

par_pos.acceleration_z_max_up = parH_pos.acc_up_max;
par_pos.acceleration_z_max_down = parH_pos.acc_down_max;

par_pos.nav_rad = parH_pos.nav_acc_rad;
par_pos.mis_yaw_error = parH_pos.mis_yaw_error;
par_pos.acceleration_hor = parH_pos.acceleration_hor;
par_pos.takeoff_ramp_time = parH_pos.takeoff_ramp_time;
par_pos.acceleration_state_dependent_xy = parH_pos.acceleration_hor_max;
par_pos.acceleration_state_dependent_z = par_pos.acceleration_z_max_up;

par_pos.cruise_speed_90 = parH_pos.cruise_speed_90;
%% par_input
par_input.home_pos = [0,0,0]';

%% bool_pos
bool_pos.task_should_exit = 0;			% <true if task should exit */
bool_pos.gear_state_initialized = 0;	% <true if the gear state has been initialized */
bool_pos.reset_pos_sp = 1;  			% <true if position setpoint needs a reset */
bool_pos.reset_alt_sp = 1; 				% <true if altitude setpoint needs a reset */
bool_pos.do_reset_alt_pos_flag = 1; 	% < TODO: check if we need this */
bool_pos.mode_auto = 0 ;  				% <true if in auot mode */
bool_pos.pos_hold_engaged = 0; 			% <true if hold positon in xy desired */
bool_pos.alt_hold_engaged = 0; 			% <true if hold in z desired */
bool_pos.run_pos_control = 1;  			% < true if position controller should be used */
bool_pos.run_alt_control = 1; 			% <true if altitude controller should be used */
bool_pos.reset_int_z = 0; 				% <true if reset integral in z */
bool_pos.reset_int_xy = 0; 				% <true if reset integral in xy */
bool_pos.reset_yaw_sp = 1; 				% <true if reset yaw setpoint */
bool_pos.hold_offboard_xy = 0; 			% <TODO : check if we need this extra hold_offboard flag */
bool_pos.hold_offboard_z = 0;
bool_pos.in_smooth_takeoff = 0; 		% <true if takeoff ramp is applied */
bool_pos.in_landing = 0;				% <true if landing descent (only used in auto) */
bool_pos.lnd_reached_ground = 0; 		% <true if controller assumes the vehicle has reached the ground after landing */
bool_pos.triplet_lat_lon_finite = 1;

%% control_mode
control_mode.flag_control_manual_enabled = 0;
control_mode.flag_control_auto_enabled = 1;
control_mode.flag_control_rates_enabled = 1;
control_mode.flag_control_attitude_enabled = 1;
control_mode.flag_control_rattitude_enabled = 0;
control_mode.flag_control_altitude_enabled = 1;
control_mode.flag_control_velocity_enabled = 1;

control_mode.flag_control_climb_rate_enabled = 1;
control_mode.flag_control_position_enabled = 1;
control_mode.flag_control_acceleration_enabled = 0;
control_mode.flag_control_termination_enabled = 0;
control_mode.flag_control_offboard_enabled = 0;


%% position_setpoint
position_setpoint.SETPOINT_TYPE_POSITION=0;	% position setpoint
position_setpoint.SETPOINT_TYPE_VELOCITY=1;	% velocity setpoint
position_setpoint.SETPOINT_TYPE_LOITER=2;	% loiter setpoint
position_setpoint.SETPOINT_TYPE_TAKEOFF=3;	% takeoff setpoint
position_setpoint.SETPOINT_TYPE_LAND=4;	% land setpoint, altitude must be ignored, descend until landing
position_setpoint.SETPOINT_TYPE_IDLE=5;	% do nothing, switch off motors or keep at idle speed (MC)
position_setpoint.SETPOINT_TYPE_OFFBOARD=6; 	% setpoint in NED frame (x, y, z, vx, vy, vz) set by offboard
position_setpoint.SETPOINT_TYPE_FOLLOW_TARGET=7;  % setpoint in NED frame (x, y, z, vx, vy, vz) set by follow target

position_setpoint.VELOCITY_FRAME_LOCAL_NED = 1; % MAV_FRAME_LOCAL_NED
position_setpoint.VELOCITY_FRAME_BODY_NED = 8; % MAV_FRAME_BODY_NED


%% par
parH_att.roll_tc = 0.2;
parH_att.pitch_tc = 0.2;

parH_att.roll_p = 6;
parH_att.roll_rate_p = 0.2;
parH_att.roll_rate_i = 0.05;
parH_att.roll_rate_integ_lim = 0.30;
parH_att.roll_rate_d = 0.003;
parH_att.roll_rate_ff = 0.0;
parH_att.pitch_p = 6;
parH_att.pitch_rate_p = 0.2;
parH_att.pitch_rate_i = 0.05;
parH_att.pitch_rate_integ_lim = 0.30;
parH_att.pitch_rate_d = 0.003;
parH_att.pitch_rate_ff = 0.0;
parH_att.tpa_breakpoint_p = 1;
parH_att.tpa_breakpoint_i = 1;
parH_att.tpa_breakpoint_d = 1;
parH_att.tpa_rate_p = 0;
parH_att.tpa_rate_i = 0;
parH_att.tpa_rate_d = 0;
parH_att.yaw_p = 2.8;
parH_att.yaw_rate_p = 0.2;
parH_att.yaw_rate_i = 0.1;
parH_att.yaw_rate_integ_lim = 0.30;
parH_att.yaw_rate_d = 0;
parH_att.yaw_rate_ff = 0;
parH_att.yaw_ff = 0.5;
parH_att.roll_rate_max = 220;
parH_att.pitch_rate_max = 220;
parH_att.yaw_rate_max = 200;
parH_att.yaw_auto_max = 45;

parH_att.acro_roll_max = 120;
parH_att.acro_pitch_max = 120;
parH_att.acro_yaw_max = 120;
parH_att.rattitude_thres = 0.8;

parH_att.vtol_type = 0;
parH_att.vtol_opt_recovery_enabled = 0;
parH_att.vtol_wv_yaw_rate_scale = 0.15;

parH_att.bat_scale_en = 0;

parH_att.board_rotation = 0;

parH_att.board_offset = [0,0,0];

par.att_p = [parH_att.roll_p*(def.ATTITUDE_TC_DEFAULT/parH_att.roll_tc),parH_att.pitch_p*(def.ATTITUDE_TC_DEFAULT/parH_att.pitch_tc),parH_att.yaw_p]';				%< P gain for angular error */
par.rate_p = [parH_att.roll_rate_p*(def.ATTITUDE_TC_DEFAULT/parH_att.roll_tc),parH_att.pitch_rate_p*(def.ATTITUDE_TC_DEFAULT/parH_att.pitch_tc),parH_att.yaw_rate_p]';				%< P gain for angular rate error */
par.rate_i = [parH_att.roll_rate_i,parH_att.pitch_rate_i,parH_att.yaw_rate_i]';				%< I gain for angular rate error */
par.rate_int_lim = [parH_att.roll_rate_integ_lim,parH_att.pitch_rate_integ_lim,parH_att.yaw_rate_integ_lim]';	     	%< integrator state limit for rate loop */
par.rate_d = [parH_att.roll_rate_d*(def.ATTITUDE_TC_DEFAULT/parH_att.roll_tc),parH_att.pitch_rate_d*(def.ATTITUDE_TC_DEFAULT/parH_att.pitch_tc),parH_att.yaw_rate_d]';				%< D gain for angular rate error */
par.rate_ff = [parH_att.roll_rate_ff,parH_att.pitch_rate_ff,parH_att.yaw_rate_ff]';			%< Feedforward gain for desired rates */
par.yaw_ff = parH_att.yaw_ff;						%< yaw control feed-forward */

par.tpa_breakpoint_p = parH_att.tpa_breakpoint_p;				%< Throttle PID Attenuation breakpoint */
par.tpa_breakpoint_i = parH_att.tpa_breakpoint_i;				%< Throttle PID Attenuation breakpoint */
par.tpa_breakpoint_d = parH_att.tpa_breakpoint_d;				%< Throttle PID Attenuation breakpoint */
par.tpa_rate_p = parH_att.tpa_rate_p;					%< Throttle PID Attenuation slope */
par.tpa_rate_i = parH_att.tpa_rate_i;					%< Throttle PID Attenuation slope */
par.tpa_rate_d = parH_att.tpa_rate_d;				    %< Throttle PID Attenuation slope */

par.roll_rate_max = parH_att.roll_rate_max;
par.pitch_rate_max = parH_att.pitch_rate_max;
par.yaw_rate_max = parH_att.yaw_rate_max;
par.yaw_auto_max = parH_att.yaw_auto_max;
par.mc_rate_max = [parH_att.roll_rate_max,parH_att.pitch_rate_max,parH_att.yaw_rate_max]'*pi/180;		%< attitude rate limits in stabilized modes */
par.auto_rate_max = [parH_att.roll_rate_max,parH_att.pitch_rate_max,parH_att.yaw_auto_max]'*pi/180;		%< attitude rate limits in auto modes */
par.acro_rate_max = [parH_att.acro_roll_max,parH_att.acro_pitch_max,parH_att.acro_yaw_max]'*pi/180;		%< max attitude rates in acro mode */
par.rattitude_thres = parH_att.rattitude_thres;
par.vtol_type = parH_att.vtol_type;						%< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
par.vtol_opt_recovery_enabled = parH_att.vtol_opt_recovery_enabled;
par.vtol_wv_yaw_rate_scale = parH_att.vtol_wv_yaw_rate_scale;		%< Scale value [0, 1] for yaw rate setpoint  */

par.bat_scale_en = parH_att.bat_scale_en;
par.board_offset = parH_att.board_offset;
%par.board_rotation = parH_att.board_rotation;
par.board_rotation = euler_to_dcm(parH_att.board_offset*pi/180)*euler_to_dcm([0,0,0]);

%% v_control_mode

v_control_mode.flag_control_manual_enabled = 0;		    % true if manual input is mixed in
v_control_mode.flag_control_auto_enabled = 0;	        % true if onboard autopilot should act
v_control_mode.flag_control_rates_enabled = 1;			% true if rates are stabilized
v_control_mode.flag_control_attitude_enabled = 1;	    % true if attitude stabilization is mixed in
v_control_mode.flag_control_rattitude_enabled = 1;		% true if rate/attitude stabilization is enabled
v_control_mode.flag_control_altitude_enabled = 0;		% true if altitude is controlled
v_control_mode.flag_control_velocity_enabled = 0;

%% parmaters of quadrotor
% define the model data
Ix = 0.676603;  % initial moment of x (kg * m^2)
Iy = 0.996394;  % initial moment of y (kg * m^2)
Iz = 0.640393;  % initial moment of z (kg * m^2)
m   = 10.3551;    % mass of quardrotor (kg)
g   = 9.8;    % gravitational acceleration (N / kg)
L   = 0.6;   % distence between the rotor and center of quadrotor
R   = 0.25;   % rotor radius
pho = 1.184;      % Air density

% calculate the relationship between Fouce and rotating speed
if 0
  [b ,d , X1_PWM_MotorVel , X2_PWM_MotorVel , X3_PWM_MotorVel] = readlog();
end
b   = 1.380898059999359e-06;
d   = 4.152245751114358e-08 ;
% X1_PWM_MotorVel = 11.8874;
% X2_PWM_MotorVel = -1.264208027861401e+04;
% X1_PWM_MotorVel = 9.535;
% X2_PWM_MotorVel = -9547;

X1_PWM_MotorVel = -0.0023;
X2_PWM_MotorVel = 13.6492;
X3_PWM_MotorVel = -1.2185e+4;
% the initial position and angle
x0      = 0;  % the initial position coordinates along x axis
y0      = 0;  % the initial position coordinates along y axis
z0      = 0;  % the initial position coordinates along z axis
phi0    = 0;  % the initial roll angle
theta0  = 0;  % the initial pitch angle
psi0    = 0;  % the initial yaw angle
Ang_Pos_initial = [x0, y0, z0, phi0, theta0, psi0];

dx0     = 0;  % the initial velocity along x axis of ground frame
dy0     = 0;  % the initial velocity along y axis of ground frame
dz0     = 0;  % the initial velocity along z axis of ground frame
dphi0   = 0;  % the initial roll angular velocity
dtheta0 = 0;  % the initial pitch angular velocity
dpsi0   = 0;  % the initial yaw angular velocity

% rotating speed of motor
omiga1 = 0000; % rotating speed of motor one
omiga2 = 0000; % rotating speed of motor two
omiga3 = 0000; % rotating speed of motor three
omiga4 = 0000; % rotating speed of motor four

%% target angle
%target_roll       = 0;
%target_pitch      = 0;
%target_yaw_rate   = 0;
%throttle_in       = 0.5;
thrust_rpyt_up  = [1,1,1,1,1,1];
thrust_rpyt_low = [0,0,0,0,0,0];

%%
compensation_gain = 1;                        % apply voltage and air pressure compensation -1~1
dt                = 0.0025;                   % step time
smoothing_gain    = 2;                        % controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp,2~12

% sim('ModelSystem.slx');
