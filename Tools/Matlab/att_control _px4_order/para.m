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

%% par
parH_att.roll_tc = 0.2;
parH_att.pitch_tc = 0.2;

parH_att.roll_p = 6.5;
parH_att.roll_rate_p = 0.15;
parH_att.roll_rate_i = 0.05;
parH_att.roll_rate_integ_lim = 0.30;
parH_att.roll_rate_d = 0.003;
parH_att.roll_rate_ff = 0.0;
parH_att.pitch_p = 6.5;
parH_att.pitch_rate_p = 0.15;
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
v_control_mode.flag_control_manual_enabled = 1;		    % true if manual input is mixed in
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
target_roll       = 0;
target_pitch      = 0;
target_yaw_rate   = 0;
throttle_in       = 0.5;
thrust_rpyt_up  = [1,1,1,1,1,1];
thrust_rpyt_low = [0,0,0,0,0,0];

%%
compensation_gain = 1;                        % apply voltage and air pressure compensation -1~1
dt                = 0.0025;                   % step time
smoothing_gain    = 2;                        % controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp,2~12

% sim('ModelSystem.slx');