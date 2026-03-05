function qstar = q_transpose(q)
% qstar = q_transpose(q)
%
% Computes the quaternion transpose.
%
% Input:
%   q - input quaternion (4x1 vec)
%
% Output:
%   qstar - transpose of input quaternion (4x1 vec)
%
% Colin Riba
% December 8th, 2025
%

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

qstar = [q0 -q1 -q2 -q3]';


