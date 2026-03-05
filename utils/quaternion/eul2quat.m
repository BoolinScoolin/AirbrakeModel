function quat_out = eul2quat(euler_in)
% quat = eul2quat(euler)
% Converts classic roll-pitch-yaw 1-2-3 sequence to a unit quaternion.
%
% Input:
%   euler - euler angles [rad] (3x1 vec   or    Nx3 array)
%
% Output:
%   quat - unit rotation quaternion (4x1 vec)
%
% Colin Riba
% December 8th 2025

if (size(euler_in,2) == 1)
    euler = euler_in';
else
    euler = euler_in;
end

phi_rad = euler(:,1);
theta_rad = euler(:,2);
psi_rad = euler(:,3);

cphio2 = cos(phi_rad/2);
sphio2 = sin(phi_rad/2);
cthetao2 = cos(theta_rad/2);
sthetao2 = sin(theta_rad/2);
cpsio2 = cos(psi_rad/2);
spsio2 = sin(psi_rad/2);

q0 = cpsio2.*cthetao2.*cphio2 + spsio2.*sthetao2.*sphio2;
q1 = cpsio2.*cthetao2.*sphio2 - spsio2.*sthetao2.*cphio2;
q2 = cpsio2.*sthetao2.*cphio2 + spsio2.*cthetao2.*sphio2;
q3 = spsio2.*cthetao2.*cphio2 - cpsio2.*sthetao2.*sphio2;

quat = [q0 q1 q2 q3];

if (size(quat,1) == 1)
    quat_out = quat';
else
    quat_out = quat;
end