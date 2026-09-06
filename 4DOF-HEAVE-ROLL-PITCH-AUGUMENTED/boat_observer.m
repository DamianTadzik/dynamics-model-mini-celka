function [x_hat, info] = boat_observer(gyro, accel, tof, tof_status, params) %#codegen
% Estimates attitude (Mahony) and heave states (KF).
%
% Frames:
%   - World: NED (Z+ down)
%   - Body:  IMU / vehicle frame
%
% Inputs:
%   gyro      [3x1] deg/s   body angular rates                      [gx gy gz]
%   accel     [3x1] g       body acceleration (includes gravity)    [ax ay az]
%   tof       [4x1] mm      ToF distances to water                  [front_left, front_right, rear_left, rear_right]
%   tof_status [4x1] logical      new ToF available
%   params                controller/observer parameters
%
% Output:
%   x_hat = [
%       z;        % heave position [m]
%       z_dot;    % heave velocity [m/s]
%       phi;      % roll  [rad]
%       theta;    % pitch [rad]
%       psi;      % yaw   [rad] (unused)
%       p;        % roll rate  [rad/s]
%       q;        % pitch rate [rad/s]
%       r         % yaw rate   [rad/s] (unused)
%   ];
    %% Parameters

    Ts = params.Ts; % Block's sample time
    g = params.g; % gravity [m/s^2]

    %% Mahony attitude estimation
    persistent quat gyro_bias
    if isempty(quat) || isempty(gyro_bias)
        % quat = [1; 0; 0; 0];  % [qw qx qy qz], body->world
        gyro_bias = [0; 0; 0];     % gyro bias [rad/s]

        % accel is in g, body frame, NED (+Z down)
        a = accel / norm(accel);
    
        phi0   = atan2( a(2), a(3) );
        theta0 = atan2(-a(1), sqrt(a(2)^2 + a(3)^2));
        psi0   = 0; % yaw unobservable
    
        quat = eul2quat([psi0 theta0 phi0])'; % ZYX
    end

    [quat, gyro_bias, gyro_corr] = mahony_update(quat, gyro_bias, ...
                                                 deg2rad(gyro), accel, ...
                                                 params);

    [phi, theta, psi] = quat_to_euler_BW(quat);

    %% Heave Kalman Filter [z; z_dot; a_bias]
    persistent xh Pz
    if isempty(xh)
        % z0 = tof_to_z(tof, phi, theta, params);
        z0 = mean([ ...
            tof_to_z_i(1, tof(1), phi, theta, params) ...
            tof_to_z_i(2, tof(2), phi, theta, params) ...
            tof_to_z_i(3, tof(3), phi, theta, params) ...
            tof_to_z_i(4, tof(4), phi, theta, params) ...
            ]);
        xh = [z0; 0; 0];
        Pz = eye(3);
    end

    % compute vertical accel input a_z (m/s^2)
    aB = accel * g;
    aW = quat_rotate(quat, aB);    
    a_z = aW(3) - g; % NED +down
    
    % KF predict with accel input and bias
    A = [1 Ts -0.5*Ts^2;
         0  1 -Ts;
         0  0  1];
    
    B = [0.5*Ts^2;
         Ts;
         0];
    
    % [z, z_dot, a_bias]
    Q = params.observer.heave_KF.Q;
    
    xh = A*xh + B*a_z;
    Pz = A*Pz*A' + Q;

    for i = 1:4
        if tof_status(i) == 1   % 1 = good (simulation convention)
    
            z_meas = tof_to_z_i(i, tof(i), phi, theta, params);
    
            H = [1 0 0];
            % R = params.observer.heave_KF.R; % Sensor noise
            R = params.observer.heave_KF.R_i(i);  % allow per-sensor noise 
    
            S = H*Pz*H' + R;
            K = Pz*H' / S;
    
            xh = xh + K*(z_meas - H*xh);
            Pz = (eye(3) - K*H)*Pz;
        end
    end

    z      = xh(1);
    z_dot  = xh(2);
    % a_bias = xh(3);


    %% Output vector creation
    x_hat = [ ...
        z;
        z_dot;
        phi;
        theta;
        psi;          % psi yaw (unused)
        gyro_corr(1);
        gyro_corr(2);
        gyro_corr(3) % r yaw rate (unused)
    ];

    %% Info struct creation
    info = struct( ...
        'gyro_bias', gyro_bias, ...
        'b', [], ...
        'c', [] ...
        );
end

function z = tof_to_z_i(i, tof_i, phi, theta, params)
% i = 1..4 : FL, FR, AL, AR

    d = tof_i * 1e-3;   % mm -> m

    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(theta); sth = sin(theta);

    R_x = [1 0 0; 0 cphi -sphi; 0 sphi cphi];
    R_y = [cth 0 sth; 0 1 0; -sth 0 cth];
    R_BW = R_y * R_x;

    e_W = R_BW * [0;0;1];
    ez  = e_W(3);

    switch i
        case 1, rB = params.tof.pos_FL_B;
        case 2, rB = params.tof.pos_FR_B;
        case 3, rB = params.tof.pos_AL_B;
        case 4, rB = params.tof.pos_AR_B;
        otherwise, error('Invalid ToF index');
    end

    rW = R_BW * rB;

    % NED (+down)
    z = -d * ez - rW(3);
end
% % function z = tof_to_z(tof, phi, theta, params)
% % % Convert ToF distances to heave position z (world, NED, meters)
% % %
% % % Inputs:
% % %   tof   [4x1]  distances in mm
% % %   phi           roll  [rad]
% % %   theta         pitch [rad]
% % %   params.tof.pos_*_B   sensor positions in BODY frame [m]
% % %
% % % Output:
% % %   z     scalar heave position [m], +down (NED)
% % 
% %     % Convert mm -> m
% %     d = tof * 1e-3;
% % 
% %     % Rotation (yaw irrelevant for vertical projection)
% %     cphi = cos(phi);   sphi = sin(phi);
% %     cth  = cos(theta); sth  = sin(theta);
% % 
% %     R_x = [1 0 0;
% %            0 cphi -sphi;
% %            0 sphi  cphi];
% % 
% %     R_y = [cth 0 sth;
% %            0   1 0;
% %           -sth 0 cth];
% % 
% %     R_BW = R_y * R_x;
% % 
% %     % ToF direction in world frame (body Z+ points down)
% %     e_B = [0; 0; 1];
% %     e_W = R_BW * e_B;
% % 
% %     ez = e_W(3);   % vertical projection
% % 
% %     % Sensor positions in world frame (relative to COM)
% %     rB = [ params.tof.pos_FL_B, ...
% %            params.tof.pos_FR_B, ...
% %            params.tof.pos_AL_B, ...
% %            params.tof.pos_AR_B ];
% % 
% %     rW = R_BW * rB;
% % 
% %     % Compute z estimate from each ToF
% %     z_i = -d .* ez - rW(3, :).';
% % 
% %     % Conservative fusion: average
% %     z = mean(z_i);
% % end

function [q, b, w] = mahony_update(q, b, gyro_rad_s, accel_g, params)

    % Gains 
    Kp = params.observer.attitude.Kp;
    Ki = params.observer.attitude.Ki;
    Ts = params.Ts;

    % Gate accel (gravity only)
    acc_norm_error = abs(norm(accel_g) - 1.0);
    use_acc = acc_norm_error < params.observer.attitude.acc_norm_tolerance;

    if use_acc
        a = accel_g / norm(accel_g);
        % Estimated gravity direction in BODY frame (NED: +Z down)
        g_est = quat_rotate(quat_conj(q), [0;0;1]);
        e = cross(a, g_est);
        % e = cross(g_est, a);
    else 
        e = [0;0;0];
    end

    % Bias update
    b = b - Ki * e * Ts;

    % Corrected gyro
    omega = gyro_rad_s - b + Kp * e;

    % Quaternion integration
    q_dot = 0.5 * quat_mul(q, [0; omega]);
    q = q + q_dot * Ts;
    q = q / norm(q);

    % Return correctec omega
    w = gyro_rad_s - b;
end


%% Smaller utility helpers
function qc = quat_conj(q)
    qc = [q(1); -q(2); -q(3); -q(4)];
end

function v = quat_rotate(q, v)
    v_q = quat_mul(quat_mul(q, [0; v]), quat_conj(q));
    v = v_q(2:4);
end

function q = quat_mul(a, b)
    q = [ ...
        a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
        a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
        a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
        a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end

function [phi, theta, psi] = quat_to_euler_BW(q)
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    
    phi   = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));
    theta = asin( 2*(qw*qy - qz*qx));
    psi   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
end
