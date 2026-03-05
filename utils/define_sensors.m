
% Dynamics (assume the same for all sensors
wn = 1000*2*pi;
zeta = sqrt(2)/2;

% Accelerometer
accelerometer.noise_power = 1e3*(90e-6*9.81)^2;
accelerometer.bias = [0.1 -0.1 0.1]';
accelerometer.sf_cc = [1.01  0.01  -0.02;    % Scale factor and cross coupling errors
                       0.03  0.99  0.03;
                       0.02  0.01  1.01];
accelerometer.Ts = 1/100;

% Gyroscope
gyroscope.noise_power = 0*(75e-3)^2;
gyroscope.bias = [0.1 -0.1 0.1]*pi/180;
gyroscope.gbias = 0.05;
gyroscope.sf_cc = [1.01  -0.03  0.02;    % Scale factor and cross coupling errors
                   0.01  0.99  0.01;
                   -0.02  0.03  1.01];
gyroscope.Ts = 1/100;  % Sampling time [sec]

% GPS
GPS.Ts = 1/5;  % Sampling time [sec]
GPS.noise_power = 0.01;

% Altimeter
altimeter.Ts = 1/10;  % Sampling time [sec]
altimeter.noise_power = 0.01;

% Magnetometer
magnetometer.Ts = 1/10;  % Sampling time [sec]
magnetometer.noise_power = 0.01;

% Encoder
encoder.Ts = 1/20;  % Sampling time [sec]
encoder.noise_power = 0.000005;