function F = compute_lin_f(x,u)
% Compute Jacobian of nonlinear dynamics w.r.t. state vector x
% x = [pN, pE, pD, vN, vE, vD, phi, theta, psi]
% u = [p, q, r, ax, ay, az]

% Extract states and inputs
phi = x(7);
theta = x(8);
psi = x(9);
p = u(1);
q = u(2);
r = u(3);
ax = u(4);
ay = u(5);
az = u(6);

% Trig shorthands
Cphi   = cos(phi);   Sphi   = sin(phi);
Ctheta = cos(theta); Stheta = sin(theta);
Cpsi   = cos(psi);   Spsi   = sin(psi);
Ttheta = tan(theta); % For angular rate equations

% Precompute reuse terms
sec_theta_sq = 1 / (cos(theta)^2);

% Allocate 9x9 F matrix (state: [r v theta])
F = zeros(9);

% Position derivatives w.r.t velocity (identity)
F(1,4) = 1;
F(2,5) = 1;
F(3,6) = 1;

% ∂v̇n/∂ϕ
F(4,7) = (Cphi*Stheta*Cpsi + Sphi*Spsi)*ay + (Cphi*Spsi - Sphi*Stheta*Cpsi)*az;

% ∂v̇n/∂θ
F(4,8) = -Stheta*Cpsi*ax + Sphi*Ctheta*Cpsi*ay + Cphi*Ctheta*Cpsi*az;

% ∂v̇n/∂ψ
F(4,9) = -Ctheta*Spsi*ax + (-Sphi*Stheta*Spsi - Cphi*Cpsi)*ay + (-Cphi*Stheta*Spsi + Sphi*Cpsi)*az;

% ∂v̇e/∂ϕ
F(5,7) = (Cphi*Stheta*Spsi - Sphi*Cpsi)*ay + (-Sphi*Stheta*Spsi - Cphi*Cpsi)*az;

% ∂v̇e/∂θ
F(5,8) = -Stheta*Spsi*ax + Sphi*Ctheta*Spsi*ay + Cphi*Ctheta*Spsi*az;

% ∂v̇e/∂ψ
F(5,9) = Ctheta*Cpsi*ax + (Sphi*Stheta*Cpsi - Cphi*Spsi)*ay + (Cphi*Stheta*Cpsi + Sphi*Spsi)*az;

% ∂v̇d/∂ϕ
F(6,7) = Cphi*Ctheta*ay - Sphi*Ctheta*az;

% ∂v̇d/∂θ
F(6,8) = -Ctheta*ax - Sphi*Stheta*ay - Cphi*Stheta*az;

% ∂ϕ̇/∂ϕ
F(7,7) = cos(phi)*Ttheta*q - sin(phi)*Ttheta*r;

% ∂ϕ̇/∂θ
F(7,8) = sin(phi)*sec_theta_sq*q + cos(phi)*sec_theta_sq*r;

% ∂θ̇/∂ϕ
F(8,7) = -sin(phi)*q - cos(phi)*r;

% ∂ψ̇/∂ϕ
F(9,7) = (cos(phi)/Ctheta)*q - (sin(phi)/Ctheta)*r;

% ∂ψ̇/∂θ
F(9,8) = (Sphi*Ttheta/Ctheta)*q - (Cphi*Ttheta/Ctheta)*r;
