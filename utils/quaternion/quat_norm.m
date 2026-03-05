function qnorm = quat_norm(q)
% qnorm = quat_norm(q)
%
% Computes normalized quaternion
%
% Input:
%   q - quaternion (4x1 vec)
%
% Output:
%   qnorm - normalized quaternion (4x1 vec)
%
% Colin Riba
% December 8th, 2025
%

% Normalize
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
qnorm = q ./ norm;
