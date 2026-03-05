[motor_time_s, motor_thrust_N, motor_mass_kg] ...
    = motorReader(motor_eng_filename);

motor_time_s = [0; motor_time_s];
motor_thrust_N = [0; motor_thrust_N];
motor_mass_kg = [motor_mass_kg(1); motor_mass_kg];

motor_time_s = motor_time_s + launch_time_s;
