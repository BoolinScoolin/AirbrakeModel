function euler_out = quat2eul(quat_in)
% quat = quat2eul(quat)
% Converts unit quaternion to yaw-pitch-roll 3-2-1 sequence.
%
% Input:
%   quat - unit rotation quaternion (4x1 vec  or    Nx4 array)
%
% Output:
%   euler - euler angles [rad] (3x1 vec)
%
% Colin Riba
% December 8th 2025

if (size(quat_in,2) == 1)
    quat = quat_in';
else
    quat = quat_in;
end

qnorm = vecnorm(quat,2,2);
qnorm(qnorm == 0) = 1; 
quat = quat./qnorm;

q0 = quat(:,1);
q1 = quat(:,2);
q2 = quat(:,3);
q3 = quat(:,4);

phi_rad = atan2(2*(q2.*q3 + q0.*q1),(q0.^2 - q1.^2 - q2.^2 + q3.^2));

theta_rad = -2*(q1.*q3 - q0.*q2);
theta_rad = asin(theta_rad);

psi_rad = atan2(2*(q1.*q2 + q0.*q3),(q0.^2 + q1.^2 - q2.^2 - q3.^2));

euler = [phi_rad theta_rad psi_rad];

if (size(euler,1) == 1)
    euler_out = euler';
else
    euler_out = euler;
end

