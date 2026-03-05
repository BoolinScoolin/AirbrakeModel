% Launch Rail Conditions
d2r = pi/180;
azimuth_rad = 0*d2r;
elevation_rad = 85*d2r;

% Initial State Conditions
u0_b_mps = 0;
v0_b_mps = 0;
w0_b_mps = 0;
p0_b_rps = 0;
q0_b_rps = 0;
r0_b_rps = 0;
phi0_b2n_rad = 0*d2r;
theta0_b2n_rad = elevation_rad;      % Azimuth
psi0_b2n_rad = azimuth_rad;         % Elevation
p10_n_m = 0;
p20_n_m = 0;
p30_n_m = 0;