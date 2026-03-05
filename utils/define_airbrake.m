% other properties are in get_flap_cd, make sure to update that too!!!

% linear model used to get servo deflection from a command in flap
% deflection
estimated_flap2servo = 2.6443;
true_flap2servo = 2.6443;
% This is the lookup used in the actual software
deflection_breakpoints = (0:5:25)*pi/180;  % air brake flap deflection [rad]
flap_lookup_averages = [
    0 0.015094 0.065908 0.143237 0.23531 0.333633  % Averaged over mach numbers
];

estimated_vehicle_base_cd = 0.5;
estimated_xarea_m2 = 0.0192897963510147;
estimated_burnout_mass_kg = 24.6;
actuator_rate_limit_rps = 2.56;
