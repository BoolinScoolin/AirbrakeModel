%% Run Sim and Extract Sim Data
%results = sim("AirbrakeFlightSim.slx");
x = results.x.data;
xdot = results.xdot.data;
real_pos = x(:,10:12);
real_vel = xdot(:,10:12);
real_eul = x(:,7:9);
x = [real_pos real_vel real_eul];
t = results.tout;
alti_pd = results.alti_pd.data;
gps_pos = results.gps_pos.data;
gps_vel = results.gps_vel.data;
imu_accel = results.imu_accel.data;
imu_pqr = results.imu_pqr.data;
mag_yaw = results.mag_yaw.data;

%% Set known IC
xhat = zeros(length(t),9);
xhat(1,8) = deg2rad(85);
gz_n_mps2 = 9.81;
P = diag([0 0 0 0 0 0 0 0 0]);
Q = diag([
    0.01^2, 0.01^2, 0.01^2,...      % Position — assume near zero if derived from velocity
    0.5^2, 0.5^2, 0.5^2,...         % Velocity — unmodeled acceleration noise
    deg2rad(1)^2, deg2rad(1)^2, deg2rad(3)^2  % Euler angle process noise
]);

R = diag([
    3^2, 3^2, 5^2,...               % GPS position (x, y, z)
    0.2^2, 0.2^2, 0.2^2,...         % GPS velocity
    deg2rad(10)^2               % Yaw from magnetometer
]);
gyro_var = (deg2rad(0.05))^2;  % ~2.4e-6 rad^2/s
accel_var = (0.001)^2;         % ~1e-6 m^2/s^4
U = diag([
    gyro_var, gyro_var, gyro_var,... 
    accel_var, accel_var, accel_var
]);


for ii = 2:length(t)
    
    %% Predict
    % Compute time change
    deltat = t(ii)-t(ii-1);

    % Extract current state estimate
    p1_n_m = xhat(ii-1,1);
    p2_n_m = xhat(ii-1,2);
    p3_n_m = xhat(ii-1,3);
    p1dot_n_mps = xhat(ii-1,4);
    p2dot_n_mps = xhat(ii-1,5);
    p3dot_n_mps = xhat(ii-1,6);
    phi_rad = xhat(ii-1,7);
    theta_rad = xhat(ii-1,8);
    psi_rad = xhat(ii-1,9);

    % trig ops
    s_phi = sin(phi_rad);
    c_phi = cos(phi_rad);
    t_phi = tan(phi_rad);
    s_theta = sin(theta_rad);
    c_theta = cos(theta_rad);
    t_theta = tan(theta_rad);
    s_psi = sin(psi_rad);
    c_psi = cos(psi_rad);
    t_psi = tan(psi_rad);

    % Extract currrent inputs rates
    p_b_rps = imu_pqr(ii,1);
    q_b_rps = imu_pqr(ii,2);
    r_b_rps = imu_pqr(ii,3);
    a_meas_x = imu_accel(ii,1);
    a_meas_y = imu_accel(ii,2);
    a_meas_z = imu_accel(ii,3);
    u = [p_b_rps q_b_rps r_b_rps a_meas_x a_meas_y a_meas_z];

    % Compute euler angle rates
    phidot_rps = p_b_rps + s_phi*t_theta*q_b_rps + c_phi*t_theta*r_b_rps;
    thetadot_rps = c_phi*q_b_rps - s_phi*r_b_rps;
    psidot_rps = s_phi/c_theta*q_b_rps + c_phi/c_theta*r_b_rps;

    % Update orientation
    phi_rad = phi_rad + phidot_rps*deltat;
    theta_rad = theta_rad + thetadot_rps*deltat;
    psi_rad = psi_rad + psidot_rps*deltat;

    % trig ops (again)
    s_phi = sin(phi_rad);
    c_phi = cos(phi_rad);
    t_phi = tan(phi_rad);
    s_theta = sin(theta_rad);
    c_theta = cos(theta_rad);
    t_theta = tan(theta_rad);
    s_psi = sin(psi_rad);
    c_psi = cos(psi_rad);
    t_psi = tan(psi_rad);

    % Compute Body-fixed to NED DCM
    C_b2n_11 =  c_theta*c_psi;
    C_b2n_12 = -c_phi*s_psi + s_phi*s_theta*c_psi;
    C_b2n_13 =  s_phi*s_psi + c_phi*s_theta*c_psi;
    C_b2n_21 =  c_theta*s_psi;
    C_b2n_22 =  c_phi*c_psi + s_phi*s_theta*s_psi;
    C_b2n_23 = -s_phi*c_psi + c_phi*s_theta*s_psi;
    C_b2n_31 = -s_theta;
    C_b2n_32 =  s_phi*c_theta;
    C_b2n_33 =  c_phi*c_theta;
    
    % Extract measured accel components

    
    % Compute nav acceleration and add gravity
    ax = C_b2n_11*a_meas_x + C_b2n_12*a_meas_y + C_b2n_13*a_meas_z;
    ay = C_b2n_21*a_meas_x + C_b2n_22*a_meas_y + C_b2n_23*a_meas_z;
    az = C_b2n_31*a_meas_x + C_b2n_32*a_meas_y + C_b2n_33*a_meas_z + gz_n_mps2;

    % Update velocity
    p1dot_n_mps = p1dot_n_mps + ax*deltat;
    p2dot_n_mps = p2dot_n_mps + ay*deltat;
    p3dot_n_mps = p3dot_n_mps + az*deltat;

    % Update position
    p1_n_m = p1_n_m + p1dot_n_mps*deltat;
    p2_n_m = p2_n_m + p2dot_n_mps*deltat;
    p3_n_m = p3_n_m + p3dot_n_mps*deltat;

    % Compile state
    xhat(ii,:) = [p1_n_m p2_n_m p3_n_m p1dot_n_mps p2dot_n_mps p3dot_n_mps phi_rad theta_rad psi_rad]';
    
    % Predict Covariance
    F = compute_lin_f(x,u);
    G = compute_lin_g(x);
    P = F*P*F' + G*U*G'*deltat^2 + Q;
    
    norm(diag(P))

    %% Update
end

%% Post Process
x_err = x - xhat;