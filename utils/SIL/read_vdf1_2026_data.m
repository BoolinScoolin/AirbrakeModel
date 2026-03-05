data = readtable("Flight_Data_26.csv");

fs = data.state;

% Flight indices
idx = find(fs == 2 | fs == 3 | fs == 4 | fs == 5);
idx = [((idx(1)-100):(idx(1)-1))'; idx];
idx_launch = find(fs == 2,1);

% Time
time_s = data.time_s(idx);

% Accelerations (body frame)
accel_x_mps2 = data.ax_b_mps2(idx);
accel_y_mps2 = data.ay_b_mps2(idx);
accel_z_mps2 = data.az_b_mps2(idx);

% Angular rates (body frame)
gyro_x_rps = data.gx_b_rps(idx);
gyro_y_rps = data.gy_b_rps(idx);
gyro_z_rps = data.gz_b_rps(idx);

% Altitude
alt_z_m = data.baro_alt_m(idx);

% Quaternion attitude
q0 = data.q0(idx);
q1 = data.q1(idx);
q2 = data.q2(idx);
q3 = data.q3(idx);
q = [q0 q1 q2 q3];
euler = quat2eul(q);
euler_1_rad = euler(:,1);
euler_2_rad = euler(:,2);
euler_3_rad = euler(:,3);

true_z_m = alt_z_m*0;

