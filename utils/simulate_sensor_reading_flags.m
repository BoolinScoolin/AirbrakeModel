function sensor_reading_flags = simulate_sensor_reading_flags(t)
%
% Not currently using. Written on 12/8/2025, delete if I forgot ab it :)
%
% this function outputs boolean flags when a new sensor reading is
% available. To simulate when a new sensor reading is available, this
% function compares the simulation time to the sensor sample rate.
%
% Currently, this is set to not fire a sensor reading at t=0.

% Initialize flags
ACCELEROMETER_FLAG = false;
GYROSCOPE_FLAG = false;
MAGNETOMETER_FLAG = false;
GPS_FLAG = false;
ALTIMETER_FLAG = false;

% Grab sensor sampling rates
define_sensors;

% Initialize time since last sensor reading
persistent t_last_accel
persistent t_last_gyro
persistent t_last_mag
persistent t_last_gps
persistent t_last_alti

if isempty(t_last_accel)
    t_last_accel = 0;
    t_last_gyro = 0;
    t_last_mag = 0;
    t_last_gps = 0;
    t_last_alti = 0;
end

if (t - t_last_accel > accelerometer.Ts)
    ACCELEROMETER_FLAG = true;
    t_last_accel = t;
end

if (t - t_last_gyro > gyroscope.Ts)
    GYROSCOPE_FLAG = true;
    t_last_gyro = t;
end

if (t - t_last_mag > magnetometer.Ts)
    MAGNETOMETER_FLAG = true;
    t_last_mag = t;
end

if (t - t_last_gps > GPS.Ts)
    GPS_FLAG = true;
    t_last_gps = t;
end

if (t - t_last_alti > altimeter.Ts)
    ALTIMETER_FLAG = true;
    t_last_alti = t;
end

% Collect flags
sensor_reading_flags.accelerometer = ACCELEROMETER_FLAG;
sensor_reading_flags.gyroscope = GYROSCOPE_FLAG;
sensor_reading_flags.magnetometer = MAGNETOMETER_FLAG;
sensor_reading_flags.gps = GPS_FLAG;
sensor_reading_flags.altimeter = ALTIMETER_FLAG;