function export_sil_header(fname, t, ax, ay, az, gx, gy, gz, alt, e1, e2, e3, true_z)

fid = fopen(fname, 'w');
assert(fid > 0, 'Failed to open file');

%  fprintf(fid, '#pragma once\n');  % use this if youre making a header
fprintf(fid, '#include <cstdint>\n\n');
fprintf(fid, '#include "sil_data.h"\n\n');

fprintf(fid, '// Put this in the header file!\n');
fprintf(fid, 'constexpr uint32_t SIL_N = %u;\n\n', numel(t));

write_array(fid, 'sil_time_s', t);
write_array(fid, 'sil_accel_x_mps2', ax);
write_array(fid, 'sil_accel_y_mps2', ay);
write_array(fid, 'sil_accel_z_mps2', az);

write_array(fid, 'sil_gyro_x_rps', gx);
write_array(fid, 'sil_gyro_y_rps', gy);
write_array(fid, 'sil_gyro_z_rps', gz);

write_array(fid, 'sil_alt_z_m', alt);

write_array(fid, 'sil_euler_1_rad', e1);
write_array(fid, 'sil_euler_2_rad', e2);
write_array(fid, 'sil_euler_3_rad', e3);

write_array(fid, 'sil_true_z_m', true_z);

fclose(fid);
end

function write_array(fid, name, x)
fprintf(fid, 'const float %s[SIL_N] = {\n', name);
for i = 1:numel(x)
    if mod(i-1, 8) == 0
        fprintf(fid, '  ');
    end
    fprintf(fid, '%.9g', x(i));
    if i < numel(x)
        fprintf(fid, ', ');
    end
    if mod(i, 8) == 0 || i == numel(x)
        fprintf(fid, '\n');
    end
end
fprintf(fid, '};\n\n');
end
