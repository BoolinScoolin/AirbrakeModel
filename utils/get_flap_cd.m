function Cd = get_flap_cd(del_servo, mach, true_flap2servo)

persistent F

del_flap = del_servo/true_flap2servo;

% In practice, the full table is used in the true dynamics model
% The averages are used in the real software

% Flap deflection vs drag coeff
% Vertical Axis: Mach number
% Horizontal Axis: Deflection
flap_lookup_table = [
    0.0 0.013819  0.060143  0.129426  0.217307  0.321524;
    0.0 0.013647  0.064083  0.144115  0.235399  0.332580;
    0.0 0.013634  0.060452  0.143916  0.235057  0.332250;
    0.0 0.018159  0.072735  0.150154  0.245353  0.341790;
    0.0 0.016214  0.072129  0.148574  0.243432  0.340020
];

mach_breakpoints = 0.1:0.1:0.5;  % vehicle mach number
deflection_breakpoints = (0:5:25)*pi/180;  % air brake flap deflection [rad]

if isempty(F)
    F = griddedInterpolant({mach_breakpoints, deflection_breakpoints},...
    flap_lookup_table, 'linear', 'nearest');
end

Cd = F(mach, del_flap);

%Cd = interp2(deflection_breakpoints, mach_breakpoints, flap_lookup_table, del, mach, 'linear', 0);

end

