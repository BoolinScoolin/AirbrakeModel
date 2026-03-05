data = readmatrix("Test_Data.csv");

n = 1;  % read every n outputs
         % 1 matches simulink output
         % 2 reads every other output etc

idx = 1:n:numel( data(:,1) );

g2mps2 = 9.80665;
d2r = pi/180;

time_s         = data(idx,1);
accel_x_mps2   = g2mps2*data(idx,2);
accel_y_mps2   = g2mps2*data(idx,3);
accel_z_mps2   = g2mps2*data(idx,4);
gyro_x_rps     = d2r*data(idx,5);
gyro_y_rps     = d2r*data(idx,6);
gyro_z_rps     = d2r*data(idx,7);
alt_z_m        = data(idx,8);
euler_1_rad    = zeros(size(idx,2),1);
euler_2_rad    = -81.5*d2r + zeros(size(idx,2),1);
euler_3_rad    = zeros(size(idx,2),1);
true_z_m       = zeros(size(idx,2),1);

% Add extra initial readings to simulate time between initialization and
% launch

% Option 1: Duplicating first reading 
            % Best for simulating very long very long periods of time due
            % to the limited memory of the flight computer

    % time_s = [0; time_s + idle_time];
    % accel_x_mps2 = [accel_x_mps2(1); accel_x_mps2];
    % accel_y_mps2 = [accel_y_mps2(1); accel_y_mps2];
    % accel_z_mps2 = [accel_z_mps2(1); accel_z_mps2];
    % gyro_x_rps = [gyro_x_rps(1); gyro_x_rps];
    % gyro_y_rps = [gyro_y_rps(1); gyro_y_rps];
    % gyro_z_rps = [gyro_z_rps(1); gyro_z_rps];
    % alt_z_m = [alt_z_m(1); alt_z_m];
    % euler_1_rad = [euler_1_rad(1); euler_1_rad];
    % euler_2_rad = [euler_2_rad(1); euler_2_rad];
    % euler_3_rad = [euler_3_rad(1); euler_3_rad];
    % true_z_m = [true_z_m(1); true_z_m];

% Option 2: Duplicating first n readings
            % Higher fidelity but uses a lot more memory

    % ind = 50;  % Duplicate first ind indices
    % ndupes = 1200;  % How many duplicates
    % time_ind_s = time_s(1:ind);
    % time_per_dupe = range(time_ind_s);
    % addl_time_s = time_per_dupe*ndupes;
    % time_s = time_s + addl_time_s;
    % for ii = floor(ndupes):-1:1
    %     time_s = [(time_per_dupe*ii) + time_ind_s; time_s];
    %     accel_x_mps2 = [accel_x_mps2(1:ind); accel_x_mps2];
    %     accel_y_mps2 = [accel_y_mps2(1:ind); accel_y_mps2];
    %     accel_z_mps2 = [accel_z_mps2(1:ind); accel_z_mps2];
    %     gyro_x_rps = [gyro_x_rps(1:ind); gyro_x_rps];
    %     gyro_y_rps = [gyro_y_rps(1:ind); gyro_y_rps];
    %     gyro_z_rps = [gyro_z_rps(1:ind); gyro_z_rps];
    %     alt_z_m = [alt_z_m(1:ind); alt_z_m];
    %     euler_1_rad = [euler_1_rad(1:ind); euler_1_rad];
    %     euler_2_rad = [euler_2_rad(1:ind); euler_2_rad];
    %     euler_3_rad = [euler_3_rad(1:ind); euler_3_rad];
    %     true_z_m = [true_z_m(1:ind); true_z_m];
    % end
