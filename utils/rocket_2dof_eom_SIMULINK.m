function dudt = rocket_2dof_eom_SIMULINK(~,u,dragConstant)
% odeOfMotionSIMULINK incorporates a 2-DoF system of equations describing the motion
% of powered rocket flight. Assumes zero-lift and zero angle-of-attack
% Input Arguments:
%   u = 4x1 column vector
%       u(1) = x position (lateral)
%       u(2) = x velocity
%       u(3) = y position (vertical)
%       u(4) = y velocity
%   t = time
% Output Arguments
%   dudt = 4x1 column vector
%       dudt(1) = x velocity
%       dudt(2) = x acceleration
%       dudt(3) = y velocity
%       dudt(4) = y acceleration

%% Define motor
[~, ~, railCant, railLength] = define_default_SIMULINK();
% m =  emptyMotorMass + rocket_mass_noMotor;  % total mass (not needed when using dragConstant model)

%% Calculate Drag magnitude
totalVelocity = sqrt( u(2)^2 + u(4)^2);  % magnitude of total vehicle velocity
D = dragConstant*totalVelocity^2;
if u(4) < 0.1
    D = 0;
end

%% Define gravity
g = 9.81;  % [m s^-2]
if u(3) < 2  % 2 is arbitrary small number to signify that its AGL
    g = 0;  % don't want the rocket to fall through the ground  
end


%% Pitch / Yaw
% (Derived Quantities)
launchRodHeight = railLength*cos(railCant);  % launch rod height

if u(3) > launchRodHeight || u(4) < -0.1  % check if 'above launch rail' or 'descending'
    pitch = atan2( u(4) , u(2) );  % calculate pitch for 0 AoA
else  % on launch rail
    pitch = pi/2 - railCant;  % rail buttons keep angular orientation constant
end

%% System of Equations

dudt = zeros(4,1); % Initialize output size

dudt(1) = u(2);

dudt(2) = -D*cos(pitch);

dudt(3) = u(4);

dudt(4) = -D*sin(pitch) - g;