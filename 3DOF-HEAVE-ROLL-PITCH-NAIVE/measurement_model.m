function y = measurement_model(x, params)
% 4 distance sensors + simple IMU derived from model state
%
% Inputs:
%   x: state vector
%   [ z zdot phi phidot theta thetadot ]
%   params: boat params struct
%
% Output: 
%   y: measurement vector
%   [ d_FL; d_FR; d_AL; d_AR; ax; ay; az; gx; gy; gz ]
%
    %% Unpack input
    z       = x(1);
    zdot    = x(2);
    phi     = x(3);
    phidot  = x(4);
    theta   = x(5);
    thetadot = x(6);

    %% Rotation matrix, body -> world
    R_roll = [ 1        0           0;
               0   cos(phi)  -sin(phi);
               0   sin(phi)   cos(phi)];
    R_pitch = [ cos(theta)  0  sin(theta);
                0           1  0;
               -sin(theta)  0  cos(theta)];
    R = R_pitch * R_roll;

    % Sensors positions in the boat's body frame (COM relative)
    s_FL = [225, -182, -37]' / 1000; % m
    s_FR = [225, +182, -37]' / 1000; % m
    s_AL = [-616, -182, -37]' / 1000; % m
    s_AR = [-616, +182, -37]' / 1000; % m
    

    % COM position in world frame
    pCOM = [0; 0; z];

    % Distance sensors: take z-component in world frame ===
    p_FL = pCOM + R * s_FL;
    p_FR = pCOM + R * s_FR;
    p_AL = pCOM + R * s_AL;
    p_AR = pCOM + R * s_AR;

    d_FL = p_FL(3);     % distance to water plane z=0 (down-positive)
    d_FR = p_FR(3);
    d_AL = p_AL(3);
    d_AR = p_AR(3);


    % Simpleified IMU model
    % Gyroscope (body angular rates; small-angle approx is fine here)
    gx = phidot;
    gy = thetadot;
    gz = 0;

    % Accelerometer simplified
    % For now: just align body-z accel with zdot (you can upgrade later
    % to use zddot and gravity with R' * (a - g) ).
    ax = 0;
    ay = 0;
    az = zdot;
    

    y = [d_FL; d_FR; d_AL; d_AR; ax; ay; az; gx; gy; gz];
end

function y_noisy = add_noise_to_measurements(y)
    y_noisy = y;
end
