function [ gyro_vector, accel_vector, distance_vector ] = boat_measurements(x, xdot, params)
% This function returns three vectors computed from the boat state x vector
%   gyro_vector [3x1] [gx gy gz] rotational velocities in a body frame
%   accel_vector [3x1] [ax ay az] linear accelerations in a body frame
%   distance_vector [4x1] [front_left, front_right, rear_left, rear_right
%
% x vector values are in SI units but my sensors are reporting different
% units so scalling is needed:
%   gyro_vector [dps] degrees per second
%   accel_vector [g]
%   distance_vector [mm]

    %% Gyro
    % Gyro has some max measurement value, and gyro output is in dps not rads

    % _B frame rotation rates
    gyro_rads = [x(6); x(7); x(8)];

    % Convert to radians per second
    gyro_dps = rad2deg(gyro_rads);

    % Clamp to allowed sensor range
    range_dps = params.gyro.range_dps;
    gyro_vector = min(max(gyro_dps, -range_dps), range_dps);
    
    %% Rotation matrix
    phi_BW        = x(3);
    theta_BW      = x(4);
    psi_BW        = x(5);

    cphi = cos(phi_BW);   sphi = sin(phi_BW);
    cth  = cos(theta_BW); sth  = sin(theta_BW);
    cpsi = cos(psi_BW);   spsi = sin(psi_BW);

    R_x = [1 0 0;
           0 cphi -sphi;
           0 sphi  cphi];

    R_y = [cth 0 sth;
           0   1 0;
          -sth 0 cth];

    R_z = [cpsi -spsi 0;
           spsi  cpsi 0;
           0     0    1];

    R_WB = (R_z * R_y * R_x).';  % world -> body rotation
    R_BW = R_WB'; % body -> world rotation

    %% Accel
    % Accel has some max value that it can measure

    % Model contains only z_W at the time
    accel_mps_W = [0; 0; xdot(2)+params.g]; 

    % Since accelerometer is mounted in the _B frame we need to transform
    accel_mps_B = R_WB * accel_mps_W;

    accel_g_B = accel_mps_B / params.g;

    % Clamp to allowed sensor range
    range_g = params.accel.range_g;
    accel_vector = min(max(accel_g_B, -range_g), range_g);

    %% ToF
    % If it's underwater then 0
    % If it's over the range then 0
    % ToF sensor has conical FoV, how to account for that???

    p_COM_W = [0; 0; x(1)];

    % Computed position of all tofs
    p_FL_W  = p_COM_W + R_BW * params.tof.pos_FL_B;
    p_FR_W  = p_COM_W + R_BW * params.tof.pos_FR_B;
    p_AL_W  = p_COM_W + R_BW * params.tof.pos_AL_B;
    p_AR_W  = p_COM_W + R_BW * params.tof.pos_AR_B;

    % Direction where all ToFs are pointed in the _B frame
    e_tof_B = [0; 0; 1]; % Z+ is down
    % And in the _W frame after pitching/rolling
    e_tof_W = R_BW * e_tof_B; 

    % Ray equation  r(t) =  p_W + t * e_W 
    d_FL = -p_FL_W(3) / e_tof_W(3);
    d_FR = -p_FR_W(3) / e_tof_W(3);
    d_AL = -p_AL_W(3) / e_tof_W(3);
    d_AR = -p_AR_W(3) / e_tof_W(3);

    distance_m = [d_FL; d_FR; d_AL; d_AR];

    distance_mm = distance_m .* 1000;

    % Clamp to allowed sensor range
    range_mm_min = params.tof.range_mm_min;
    range_mm_max = params.tof.range_mm_max;
    distance_vector = min(max(distance_mm, range_mm_min), range_mm_max);

end
