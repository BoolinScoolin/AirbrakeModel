% Sanity check
N = numel(time_s);
assert(all([
    numel(accel_x_mps2)
    numel(accel_y_mps2)
    numel(accel_z_mps2)
    numel(gyro_x_rps)
    numel(gyro_y_rps)
    numel(gyro_z_rps)
    numel(alt_z_m)
    numel(euler_1_rad)
    numel(euler_2_rad)
    numel(euler_3_rad)
    numel(true_z_m)
] == N), 'Signal length mismatch');

export_sil_header('sil_data.cpp', ...
    time_s, ...
    accel_x_mps2, accel_y_mps2, accel_z_mps2, ...
    gyro_x_rps, gyro_y_rps, gyro_z_rps, ...
    alt_z_m, euler_1_rad, euler_2_rad, euler_3_rad, ...
    true_z_m);
