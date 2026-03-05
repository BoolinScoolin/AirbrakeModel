function product = quatmult(quat1, quat2)
% product = quatmult(quat1, quat2)
%
% Multiples 2 quaternions. First entry of quaternion should be real part.
%
% Inputs:
%   quat1 - first quaternion (4x1 vec)
%   quat2 - second quaternion (4x1 vec)
%
% Output:
%   product - resulting product of quat1*quat2 (4x1 vec)
%
% Colin Riba
% December 8th, 2025
%

a0 = quat1(1);
avec = [quat1(2) quat1(3) quat1(4)]';
b0 = quat2(1);
bvec = [quat2(2) quat2(3) quat2(4)]';

c0 = a0*b0 - avec'*bvec;
cvec = a0*bvec + b0*avec + cross(avec, bvec);

product = [c0 cvec(1) cvec(2) cvec(3)]';
