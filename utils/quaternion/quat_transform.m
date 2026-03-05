function v_out = quat_transform(v_in, quat)
% v_out = quat_transform(v_in, quat)
% transforms v_in to a new basis about the rotation quaternion quat
%
% Input:
%   v_in - vector in basis A (3x1 vec)
%   quat - rotation quaternion encoding orientation of basis B w.r.t A
%
% Output:
%   v_out - vector in basis B (3x1 vec)
%

v_quat = [0 v_in(1) v_in(2) v_in(3)]';
quat_star = q_transpose(quat);

% Compute (q*)(v)(q)
v_out = quatmult(quat_star, v_quat);
v_out = quatmult(v_out, quat);
v_out = [v_out(2) v_out(3) v_out(4)]';