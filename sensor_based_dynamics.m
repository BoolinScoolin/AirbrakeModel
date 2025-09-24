function xdot = sensor_based_dynamics(t,x,u,drymass_kg,J_tensor_b_kgm2,motor_mass_kg,motor_time_s)
%
% Usage:
% xdot = sensor_based_dynamics(t,x,u,drymass_kg,J_tensor_b_kgm2,motor_mass_kg,motor_time_s)
%
% sensor_based_dynamics propagates model-rocket trajectory using live
% sensor data from an accelerometer and a gyroscope.
%
% Inputs:
%   t - time since launch [seconds]
%      double
%   x - state vector
%      12 x 1 double vector
%        u_b_mps = x(1);  % Body-fixed velocities along x,y,z
%        v_b_mps = x(2);
%        w_b_mps = x(3);
%        p_b_rps = x(4);  % Body-fixed angular velocities along x,y,z
%        q_b_rps = x(5);
%        r_b_rps = x(6);
%        phi_rad = x(7);  % Euler-angles roll,pitch,yaw
%        theta_rad = x(8);
%        psi_rad = x(9);
%        p1_n_m = x(10);  % Inertial position north,east,down
%        p2_n_m = x(11);
%        p3_n_m = x(12);
%   u - input vector
%      6 x 1 double vector
%        Fx_b_N = u(1); % External forces
%        Fy_b_N = u(2);
%        Fz_b_N = u(3);
%        p_b_rps = u(4);  % Body-fixed angular velocities along x,y,z
%        q_b_rps = u(5);
%        r_b_rps = u(6);
%   drymass_kg - unloaded vehicle mass [kg]
%          double
%   J_tensor_b_kgm2 - inertia matrix [kg m2]
%          3 x 3 double matrix
%   motor_mass_kg - motor mass breakpoints
%          N x 1 double vector
%   motor_time_s - motor time breakpoints
%          N x 1 double vector
%
% Outputs:
%   xdot - state derivative vector
%      12 x 1 double vector
%


% Extract state variables
u_b_mps = x(1);  % Body-fixed velocities along x,y,z
v_b_mps = x(2);
w_b_mps = x(3);
% p_b_rps = x(4);  % Body-fixed angular velocities along x,y,z
% q_b_rps = x(5);
% r_b_rps = x(6);
phi_rad = x(7);  % Euler-angles roll,pitch,yaw
theta_rad = x(8);
psi_rad = x(9);
p1_n_m = x(10);  % Inertial position north,east,down
p2_n_m = x(11);
p3_n_m = x(12);

% Extract input variables
ax_b_mps2 = u(1); % measured specific force
ay_b_mps2 = u(2);
az_b_mps2 = u(3);
p_b_rps = u(4);  % Body-fixed angular velocities along x,y,z
q_b_rps = u(5);
r_b_rps = u(6);

% Store trigonometric operations
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
C_n2b_13 = C_b2n_31;
C_n2b_23 = C_b2n_32;
C_n2b_33 = C_b2n_33;

% Atmopheric model
h_m = -p3_n_m;  % Store current altitude
gamma = 1.4;  % Ratio of specific heats
R = 287.05;  % Air specific gas constant
[temp_K, ~, rho_kgpm3] = interpolate_atmosphere(h_m, params.atmosphere);  % read/interpolate atmospheric model
true_airspeed_mps = sqrt(u_b_mps^2 + v_b_mps^2 + w_b_mps^2);  % true airspeed relative to atmosphere
qbar_kgpms2 = 0.5*rho_kgpm3*true_airspeed_mps^2;  % dynamic pressure
c_mps2 = sqrt(gamma*R*temp_K);  % local speed of sound

% Define gravity
gz_n_mps2 = 9.81;  % resolved in NED
if p3_n_m > 0  % 2 is arbitrary small number to signify that its AGL
    gz_n_mps2 = 0;  % don't want the rocket to fall through the ground  
end  % I think theres a better way to do this

% Resolve gravity in body-fixed
gx_b_mps2 = C_n2b_13*gz_n_mps2;
gy_b_mps2 = C_n2b_23*gz_n_mps2;
gz_b_mps2 = C_n2b_33*gz_n_mps2;

%% State derivatives
xdot = zeros(12,1);
% position rates
xdot(1) = ax_b_mps2 + gx_b_mps2 - w_b_mps*q_b_rps + v_b_mps*r_b_rps;  % vx
xdot(2) = ay_b_mps2 + gy_b_mps2 - u_b_mps*r_b_rps + w_b_mps*p_b_rps;  % vy
xdot(3) = az_b_mps2 + gz_b_mps2 - v_b_mps*p_b_rps + u_b_mps*q_b_rps;  % vz

% % angular rates
% xdot(4) = ( Jxz_b_kgm2*(Jxx_b_kgm2 - Jyy_b_kgm2 + Jzz_b_kgm2)*p_b_rps*q_b_rps ...
%             - (Jzz_b_kgm2*(Jzz_b_kgm2-Jyy_b_kgm2)+Jxz_b_kgm2^2)*q_b_rps*r_b_rps ...
%             + Jzz_b_kgm2*l_b_Nm + Jxz_b_kgm2*n_b_Nm ) / Den;                % roll rate
% xdot(5) = ( (Jzz_b_kgm2 - Jxx_b_kgm2)*r_b_rps*p_b_rps - Jxz_b_kgm2*(p_b_rps^2-r_b_rps^2) + m_b_Nm ) / Jyy_b_kgm2;  % pitch rate
% xdot(6) = ( -Jxz_b_kgm2*(Jxx_b_kgm2 - Jyy_b_kgm2 + Jzz_b_kgm2)*q_b_rps*r_b_rps...
%             + (Jxx_b_kgm2*(Jxx_b_kgm2 - Jyy_b_kgm2) + Jxz_b_kgm2^2)*p_b_rps*q_b_rps ...
%             + Jxz_b_kgm2*l_b_Nm+Jxx_b_kgm2*n_b_Nm ) / Den;  % yaw rate
xdot(4) = 0;
xdot(5) = 0;
xdot(6) = 0;

% euler kinematics
xdot(7) = p_b_rps + s_phi*t_theta*q_b_rps + c_phi*t_theta*r_b_rps;
xdot(8) = c_phi*q_b_rps - s_phi*r_b_rps;
xdot(9) = s_phi/c_theta*q_b_rps + c_phi/c_theta*r_b_rps;

% navigation equations
xdot(10) = C_b2n_11*u_b_mps + C_b2n_12*v_b_mps + C_b2n_13*w_b_mps;  % x-vel resolved in NED
xdot(11) = C_b2n_21*u_b_mps + C_b2n_22*v_b_mps + C_b2n_23*w_b_mps;  % y-vel in NED
xdot(12) = C_b2n_31*u_b_mps + C_b2n_32*v_b_mps + C_b2n_33*w_b_mps;  % z-vel in NED