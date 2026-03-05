% initializes simulink simulation variables
addpath('utils')
addpath('utils\quaternion')
addpath('utils\motors')
addpath('..\USLI Simulink Library')

motor_eng_filename = "AeroTech_L2200G.eng";  % include .eng

launch_time_s = 0;  % Seconds on launch rail

wind_on_bool = 1;  % 1 for wind on, 0 for wind off

define_vehicle
define_initial_conditions
define_atmosphere
define_gravity
define_sensors
define_motor
define_airbrake