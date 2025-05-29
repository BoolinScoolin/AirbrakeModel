function apogee = mainSIMULINK(airbrakeDragCharacteristics,ic)
% Propagates vehicle flight and returns vehicle apogee

% max time span of flight simulation
tspan = linspace(0, 200, 1000)';

% Solver
options = odeset('RelTol', 1e-4, 'Events', @landingEventSIMULINK);
[~, u] = ode15s(@(t,u) odeOfMotionSIMULINK(t,u,airbrakeDragCharacteristics) , tspan, ic, options);

%% Organize Data
u = u*3.28084;  % convert to [ft]

% Extract varibles
apogee = max(u(:,3));  % y distance  (altitude)