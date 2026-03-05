% OpenRocket stuff
vehicle_diam_in = 6.17;
vehicle_stability_cal = 2.66;
mass_no_motors_oz = 792;
inertia_b_kgm2 = [9.8e-2 0 0; 0 17.3574 0; 0 0 17.3574];
vehicle_cd = 0.476;
fins.root_chord_in = 9.565; 
fins.tip_chord_in = 3.478;
fins.height_in = 6;
fins.num_fins = 3;
fins.c_n_alpha = 2.11;

% Conversions
in2m = 0.0254;
oz2kg = 0.0283495;
unloaded_mass_kg = mass_no_motors_oz*oz2kg;
cg2cp_m = [-vehicle_stability_cal*vehicle_diam_in*in2m 0 0];
xarea_m2 = pi*(vehicle_diam_in*in2m/2)^2;
fins.planform_area_m2 = (fins.tip_chord_in+fins.root_chord_in)/2 * fins.height_in * in2m^2;
