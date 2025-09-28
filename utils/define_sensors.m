
% Dynamics (assume the same for all sensors
wn = 1000*2*pi;
zeta = sqrt(2)/2;

% Accelerometer
accelerometer.noise_power = (90e-6*9.81)^2;
accelerometer.bias = 0;
accelerometer.sf_cc = [1 0 0;    % Scale factor and cross coupling errors
                       0 1 0;
                       0 0 1];
accelerometer.Ts = 1/100;

% Gyroscope
gyroscope.noise_power = (75e-3)^2;
gyroscope.bias = 0;
gyroscope.gbias = 0.0;
gyroscope.sf_cc = [1 0 0;    % Scale factor and cross coupling errors
                   0 1 0;
                   0 0 1];
gyroscope.Ts = 1/100;  % Sampling time