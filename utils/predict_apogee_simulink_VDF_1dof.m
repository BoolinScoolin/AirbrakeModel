function predictedApogee = predict_apogee_simulink_VDF_1dof(ic, drag_const)

altitude = ic(3);  % in up direction
velocity = ic(4);
time = 0;
simulationSteps = 0;

% rocket_cd = 0.5;
% flap_cd = 0.0;
% totalCd = rocket_cd + flap_cd;

SIM_TIME_STEP = 0.01;
GRAVITY = 9.81;
ROCKET_MASS = 21.0;
% REFERENCE_AREA = 3.1415926535897932384626433832795*0.1567*0.1567/4.0;

while (velocity > 0.0 && time < 60000) 
    % Calculate drag force: F_drag = 0.5 * rho * v^2 * Cd * A
    dragForce = drag_const * velocity * velocity;
    
    % Total acceleration: a = -g - (F_drag / m)
    acceleration = -GRAVITY - (dragForce / ROCKET_MASS);
    
    % % Euler integration (Symplectic)
    % velocity = velocity + acceleration * SIM_TIME_STEP;
    % altitude = altitude + velocity * SIM_TIME_STEP;

    % Euler integration (Explicit)
    altitude = altitude + velocity * SIM_TIME_STEP + 0.5*acceleration * SIM_TIME_STEP * SIM_TIME_STEP;
    velocity = velocity + acceleration * SIM_TIME_STEP;


    time = time + SIM_TIME_STEP;
    simulationSteps = simulationSteps + 1;
    
    % Safety check
    if (simulationSteps > 1000000)
        break;  % Prevent infinite loop
    end
end

predictedApogee = altitude;