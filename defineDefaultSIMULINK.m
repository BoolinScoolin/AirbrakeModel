function [emptyMotorMass, rocket_mass_noMotor,railCant, railLength] = defineDefaultSIMULINK()

% Define Rocket
rocket_mass_noMotor = 581;  % vehicle mass with no motors [lb]
rocket_mass_noMotor = rocket_mass_noMotor*0.0283495;  % convert to [kg]

% Define Launch Rail
railCant = 5;  % rail cant [degrees]
railCant = deg2rad(railCant);  % convert to [radians]
railLength = 144;  % rail length [in]
railLength = railLength*0.0254;  % convert to [m]

% Define Motor
emptyMotorMass = 2.03208320000000;