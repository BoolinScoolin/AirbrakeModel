n = 10;  % read every n outputs
         % 1 matches simulink output
         % 2 reads every other output etc

idx = 1:n:numel(out.simout.Time);

time_s         = out.simout.Time(idx);
accel_x_mps2   = out.simout.Data(idx,1);
accel_y_mps2   = out.simout.Data(idx,2);
accel_z_mps2   = out.simout.Data(idx,3);
gyro_x_rps     = out.simout1.Data(idx,1);
gyro_y_rps     = out.simout1.Data(idx,2);
gyro_z_rps     = out.simout1.Data(idx,3);
alt_z_m        = out.simout2.Data(idx);
euler_1_rad    = out.simout3.Data(idx,7);
euler_2_rad    = out.simout3.Data(idx,8);
euler_3_rad    = out.simout3.Data(idx,9);
true_z_m       = -out.simout3.Data(idx,12);