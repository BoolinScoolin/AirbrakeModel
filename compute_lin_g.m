function G = compute_lin_g(x)
% Compute Jacobian of nonlinear dynamics w.r.t. input vector u
% x = [pN, pE, pD, vN, vE, vD, phi, theta, psi]
% u = [p, q, r, ax, ay, az]

% Extract states
phi = x(7);
theta = x(8);
psi = x(9);

% Trig shorthands
Cphi   = cos(phi);   Sphi   = sin(phi);
Ctheta = cos(theta); Stheta = sin(theta);
Cpsi   = cos(psi);   Spsi   = sin(psi);
Ttheta = tan(theta);

% Allocate 9x6 G matrix (∂f/∂u)
G = zeros(9,6);

% ∂v̇n/∂ax, ∂v̇n/∂ay, ∂v̇n/∂az
G(4,4) =  Ctheta*Cpsi;
G(4,5) =  Sphi*Stheta*Cpsi - Cphi*Spsi;
G(4,6) =  Cphi*Stheta*Cpsi + Sphi*Spsi;

% ∂v̇e/∂ax, ∂v̇e/∂ay, ∂v̇e/∂az
G(5,4) =  Ctheta*Spsi;
G(5,5) =  Sphi*Stheta*Spsi + Cphi*Cpsi;
G(5,6) =  Cphi*Stheta*Spsi - Sphi*Cpsi;

% ∂v̇d/∂ax, ∂v̇d/∂ay, ∂v̇d/∂az
G(6,4) = -Stheta;
G(6,5) =  Sphi*Ctheta;
G(6,6) =  Cphi*Ctheta;

% ∂ϕ̇/∂p, ∂ϕ̇/∂q, ∂ϕ̇/∂r
G(7,1) = 1;
G(7,2) = Sphi*Ttheta;
G(7,3) = Cphi*Ttheta;

% ∂θ̇/∂q, ∂θ̇/∂r
G(8,2) = Cphi;
G(8,3) = -Sphi;

% ∂ψ̇/∂q, ∂ψ̇/∂r
G(9,2) = Sphi / Ctheta;
G(9,3) = Cphi / Ctheta;
end
