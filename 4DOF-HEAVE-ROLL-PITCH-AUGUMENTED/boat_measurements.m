function [ gyro_vector, accel_vector, distance_vector, gps_speed ] = boat_measurements(x, xdot, params) %#codegen
% This function returns four vectors computed from the boat state x vector
%   gyro_vector [3x1] [gx gy gz] rotational velocities in a body frame
%   accel_vector [3x1] [ax ay az] linear accelerations in a body frame
%   distance_vector [4x1] [front_left, front_right, rear_left, rear_right]
%   gps_speed [1x1] 

% x vector values are in SI units but my sensors are reporting different
% units so scalling is needed:
%   gyro_vector [dps] degrees per second
%   accel_vector [g]
%   distance_vector [mm]
%   gps_speed [m/s]

    %% Gyro
    % Gyro output is in dps not rads

    % _B frame rotation rates
    gyro_rads = [x(8); x(9); x(10)];

    % Convert to radians per second
    gyro_dps = rad2deg(gyro_rads);

    gyro_vector = gyro_dps;

    
    %% Rotation matrix
    phi_BW        = x(5);
    theta_BW      = x(6);
    psi_BW        = x(7);

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
    % Accel output is in g not m/s^2
    accel_mps_W = [xdot(2); xdot(3); xdot(4)+params.g]; 

    % Since accelerometer is mounted in the _B frame we need to transform
    accel_mps_B = R_WB * accel_mps_W;

    accel_g_B = accel_mps_B / params.g;
    accel_vector = accel_g_B;
    % accel_vector = R_WB * [0; 0; 1]; % gravity direction, NED

    %% ToF
    % If it's 'underwater' then 0 dont allow for negative meas XD

    p_COM_W = [0; 0; x(3)];

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

    % To zero the negative measurements
    distance_vector = max(distance_mm, 0);


    %% GPS
    u = x(2);   % longitudinal velocity
    v = 0;      % sway neglected in the model
    gps_speed = sqrt(u^2 + v^2);


end
