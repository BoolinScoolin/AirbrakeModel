function apogee = predict_apogee_SIMULINK(airbrakeDragCharacteristics,ic)
% Propagates vehicle flight and returns vehicle apogee

% max time span of flight simulation
tspan = linspace(0, 200, 1000)';

% Solver
options = odeset('RelTol', 1e-4, 'Events', @landing_event_SIMULINK);
[~, u] = ode45(@(t,u) rocket_2dof_eom_SIMULINK(t,u,airbrakeDragCharacteristics) , tspan, ic, options);

% Extract varibles
apogee = max(u(:,3));  % y distance  (altitude)

