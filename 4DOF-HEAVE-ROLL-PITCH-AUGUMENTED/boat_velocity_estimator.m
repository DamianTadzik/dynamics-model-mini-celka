function velocity = boat_velocity_estimator(accel, phi, theta, gps_speed, gps_new, params)
% Estimate forward WORLD velocity using IMU acceleration + GPS speed.
%
% State:
%   xv = [velocity; accel_bias]

    persistent xv Pv

    Ts = params.Ts;
    g  = params.g;

    if isempty(xv)
        xv = [gps_speed; 0];
        Pv = eye(2);
    end

    % BODY -> WORLD acceleration

    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(theta); sth = sin(theta);

    R_x = [1 0 0;
           0 cphi -sphi;
           0 sphi  cphi];

    R_y = [cth 0 sth;
           0   1 0;
          -sth 0 cth];

    R_BW = R_y * R_x;

    aB = accel * g;
    aW = R_BW * aB;

    ax = aW(1);

    % Prediction
    % x = [v; bias]
    %
    % v(k+1) = v(k) + Ts*(ax - bias)

    A = [1 -Ts;
         0  1];

    B = [Ts;
         0];

    Q = params.observer.velocity_KF.Q;

    xv = A*xv + B*ax;
    Pv = A*Pv*A' + Q;

    % GPS correction
    if gps_new
        H = [1 0];
        R = params.observer.velocity_KF.R;
    
        S = H*Pv*H' + R;
        K = Pv*H' / S;
    
        xv = xv + K*(gps_speed - H*xv);
        Pv = (eye(2) - K*H)*Pv;
    end

    velocity = xv(1);
end