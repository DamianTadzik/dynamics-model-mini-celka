function [ xdot, info ] = boat_dynamics_4dof(x, u, w, params) %#codegen
%  Input
%   x = [
%       xW;              % 1 surge position [m] (world frame, NED)
%       xWdot;           % 2 surge velocity [m/s] (world frame)
%
%       zW;              % 3 heave position [m] (world frame, NED)
%       zWdot;           % 4 heave velocity [m/s] (world frame)
%
%       phi_BW;          % 5 roll angle   (Body relative to World)
%       theta_BW;        % 6 pitch angle  (Body relative to World)
%       psi_BW;          % 7 yaw angle    (Body relative to World)
%
%       omega_phi_B;     % 8 roll  rate  in BODY frame
%       omega_theta_B;   % 9 pitch rate  in BODY frame
%       omega_psi_B;     % 10 yaw  rate  in BODY frame
%
%       alpha_FL         % 11 [deg]
%       alpha_FR         % 12 [deg]
%       alpha_R          % 13 [deg]
%   ];
%   u = [
%       alpha_FrontLeft  % 1  [deg]
%       alpha_FrontRight % 2  [deg]
%       alpha_Rear       % 3  [deg]
%       F_thrust         % 4  [N]
%   ];
%   w = [
%       z_force_disturbance
%       roll_torque_disturbance
%       pitch_torque_disturbance
%   ];
%   parameters bus;
%
% Output:
%   xdot = time derivative of state
%
% World frame W: NED (x forward, y right, z down), water surface at z_W = 0
% Body frame B: NED attached to COM
%
    %%  Unpack state
    xW            = x(1);
    xWdot         = x(2);

    zW            = x(3);
    zWdot         = x(4);

    phi_BW        = x(5);
    theta_BW      = x(6);
    psi_BW        = 0;%x(7); % Yaw is constrained in the reduced 4DOF model.

    omega_phi_B   = x(8);
    omega_theta_B = x(9);
    omega_psi_B   = 0;%x(10); % Yaw is constrained in the reduced 4DOF model.

    alpha_FL_act  = x(11);
    alpha_FR_act  = x(12);
    alpha_R_act   = x(13);

    omega_B = [omega_phi_B; omega_theta_B; omega_psi_B];

    %%  Unpack inputs
    alpha_FL_cmd  = u(1);
    alpha_FR_cmd  = u(2);
    alpha_R_cmd   = u(3);
    F_thrust        = u(4);
    % V_W             = u(5); % For now it is separate input, later will add fourth equation for x_W movement
    
    %% Calculate actuator dynamics 1st order
    T = params.hydrofoils.dynamics.T;
    alpha_min = params.hydrofoils.dynamics.alpha_min; % [deg]
    alpha_max = params.hydrofoils.dynamics.alpha_max;

    alpha_FL_cmd = min(max(alpha_FL_cmd, alpha_min), alpha_max); % [deg]
    alpha_FR_cmd = min(max(alpha_FR_cmd, alpha_min), alpha_max);
    alpha_R_cmd  = min(max(alpha_R_cmd,  alpha_min), alpha_max);

    alpha_FL_dot  = (-alpha_FL_act + alpha_FL_cmd ) / T; % [deg/s]
    alpha_FR_dot  = (-alpha_FR_act + alpha_FR_cmd ) / T;
    alpha_R_dot   = (-alpha_R_act  + alpha_R_cmd  ) / T;

    %% Unpack disturbances
    % disturbances are just additional forces/torques acting on the system
    F_z_dist = w(1);
    tau_roll_dist = w(2);
    tau_pitch_dist = w(3);

    %%  Unpack parameters
    m     = params.m;
    g     = params.g;
    rho   = params.rho;

    Ix_B  = params.Ix_B;
    Iy_B  = params.Iy_B;
    Iz_B  = params.Iz_B;

    % Positions in body frame (NED)
    r_FL_B = params.hydrofoils.pos_front_left_B;   % [x;y;z]
    r_FR_B = params.hydrofoils.pos_front_right_B;
    r_R_B  = params.hydrofoils.pos_rear_B;
    r_T_B  = params.thruster.pos_thruster_B;

    % Wing areas
    S_front = params.hydrofoils.S_front;
    S_rear  = params.hydrofoils.S_rear;


    %% Rotation matrices
    cphi = cos(phi_BW);   sphi = sin(phi_BW);
    cth  = cos(theta_BW); sth  = sin(theta_BW);
    cpsi = cos(psi_BW);   spsi = sin(psi_BW);

    R_x = [1 0 0;
           0 cphi -sphi;
           0 sphi  cphi];

    R_y = [ cth 0 sth;
            0   1 0;
           -sth 0 cth];

    R_z = [ cpsi -spsi 0;
            spsi  cpsi 0;
            0      0   1];

    % From B to W frame     w_frame_vector = R_BW * b_frame_vector;
    R_BW = R_z * R_y * R_x;

    % From W to B frame     b_frame_vector = R_WB * w_frame_vector;
    R_WB = R_BW.';

    %% Local flow velocity, and resulting inflow angle at each foil
    v_COM_W = [xWdot; 0; zWdot];
    v_COM_B = R_WB * v_COM_W;
    
    v_FL_B = v_COM_B + cross(omega_B, r_FL_B);
    v_FR_B = v_COM_B + cross(omega_B, r_FR_B);
    v_R_B  = v_COM_B + cross(omega_B, r_R_B);

    gamma_FL = atan2(v_FL_B(3), v_FL_B(1));
    gamma_FR = atan2(v_FR_B(3), v_FR_B(1));
    gamma_R  = atan2(v_R_B(3),  v_R_B(1));
    
    alpha_deg_FL = alpha_FL_act + rad2deg(gamma_FL);
    alpha_deg_FR = alpha_FR_act + rad2deg(gamma_FR);
    alpha_deg_R  = alpha_R_act  + rad2deg(gamma_R);

    %% Lift and drag coefficients
    % Account for boat pitch in alpha in CL/CD(alpha) calculation.
    alpha_deg_FL = min(max(alpha_deg_FL, params.hydrofoils.LUT.front.alpha(1)), params.hydrofoils.LUT.front.alpha(end)); % [deg]
    alpha_deg_FR = min(max(alpha_deg_FR, params.hydrofoils.LUT.front.alpha(1)), params.hydrofoils.LUT.front.alpha(end));
    alpha_deg_R  = min(max(alpha_deg_R, params.hydrofoils.LUT.rear.alpha(1)), params.hydrofoils.LUT.rear.alpha(end));

    CL_FrontLeft  = interp1(params.hydrofoils.LUT.front.alpha, params.hydrofoils.LUT.front.CL, alpha_deg_FL, 'linear', 'extrap');
    CL_FrontRight = interp1(params.hydrofoils.LUT.front.alpha, params.hydrofoils.LUT.front.CL, alpha_deg_FR, 'linear', 'extrap');
    CL_Rear       = interp1(params.hydrofoils.LUT.rear.alpha,  params.hydrofoils.LUT.rear.CL, alpha_deg_R,  'linear', 'extrap');

    CD_FrontLeft  = interp1(params.hydrofoils.LUT.front.alpha, params.hydrofoils.LUT.front.CD, alpha_deg_FL, 'linear', 'extrap');
    CD_FrontRight = interp1(params.hydrofoils.LUT.front.alpha, params.hydrofoils.LUT.front.CD, alpha_deg_FR, 'linear', 'extrap');
    CD_Rear       = interp1(params.hydrofoils.LUT.rear.alpha,  params.hydrofoils.LUT.rear.CD, alpha_deg_R,  'linear', 'extrap');

    %% Local hydrofoil flow velocity in x_B-z_B plane
    v_FL_xz_B = [v_FL_B(1); 0; v_FL_B(3)];
    v_FR_xz_B = [v_FR_B(1); 0; v_FR_B(3)];
    v_R_xz_B  = [v_R_B(1);  0; v_R_B(3)];
    
    V_FL = norm(v_FL_xz_B);
    V_FR = norm(v_FR_xz_B);
    V_R  = norm(v_R_xz_B);

    %%  Lift / drag magnitudes 
    FL_FrontLeft  = 0.5 * rho * S_front * V_FL^2 * CL_FrontLeft;
    FL_FrontRight = 0.5 * rho * S_front * V_FR^2 * CL_FrontRight;
    FL_Rear       = 0.5 * rho * S_rear  * V_R^2 * CL_Rear;
    
    FD_FrontLeft  = 0.5 * rho * S_front * V_FL^2 * CD_FrontLeft;
    FD_FrontRight = 0.5 * rho * S_front * V_FR^2 * CD_FrontRight;
    FD_Rear       = 0.5 * rho * S_rear  * V_R^2 * CD_Rear;

    %% Lift / drag vectors in the _W/_B? frame
    % Hydrofoil span direction
    e_span_B = [0; 1; 0];
    
    % Drag acts opposite to the local hydrofoil velocity
    e_drag_FL_B = -v_FL_xz_B / V_FL;
    e_drag_FR_B = -v_FR_xz_B / V_FR;
    e_drag_R_B  = -v_R_xz_B  / V_R;
    
    % Lift is perpendicular to both the local velocity and the foil span
    e_lift_FL_B = cross(e_span_B, v_FL_xz_B);
    e_lift_FR_B = cross(e_span_B, v_FR_xz_B);
    e_lift_R_B  = cross(e_span_B, v_R_xz_B);

    e_lift_FL_B = e_lift_FL_B / norm(e_lift_FL_B);
    e_lift_FR_B = e_lift_FR_B / norm(e_lift_FR_B);
    e_lift_R_B  = e_lift_R_B  / norm(e_lift_R_B);
    
    % Hydrofoil forces
    F_FL_B = FL_FrontLeft  * e_lift_FL_B + FD_FrontLeft  * e_drag_FL_B;
    F_FR_B = FL_FrontRight * e_lift_FR_B + FD_FrontRight * e_drag_FR_B;
    F_R_B  = FL_Rear       * e_lift_R_B  + FD_Rear       * e_drag_R_B;

    % Propeller force (along +x_B)
    F_T_B  = [F_thrust; 0; 0];

    %% Model the loss of the lift/drag as foils exit the water

    % World positions of foils / prop
    p_FL_W = [0;0;zW] + R_BW * r_FL_B;
    p_FR_W = [0;0;zW] + R_BW * r_FR_B;
    p_R_W  = [0;0;zW] + R_BW * r_R_B;

    if p_FL_W(3) < 0
        F_FL_B = [0; 0; 0];
    end
    if p_FR_W(3) < 0
        F_FR_B = [0; 0; 0];
    end
    if p_R_W(3) < 0
        F_R_B = [0; 0; 0];
    end

    %% Model the buoyancy (simple for now)
    % FB_up = buoyancy_force(zW, phi_BW, theta_BW, params);
    LUT_z = params.buoyancy.LUT.z;
    LUT_Fb = params.buoyancy.LUT.Fb;
    LUT_V = params.buoyancy.LUT.V;
    FB_up = interp1(LUT_z, LUT_Fb, zW, "pchip");
    V_submerged = interp1(LUT_z, LUT_V, zW, "pchip");

    %%% maybe the added momentum from COB placement, that should handle that but...
    %%% wow i do not think i will ever do this
        % TODO: replace with proper 3D LUT including roll/pitch dependence

    %% Strut drag
    F_strut_FL_B = zeros(3,1);
    tau_strut_FL_B = zeros(3,1);

    F_strut_FR_B = zeros(3,1);
    tau_strut_FR_B = zeros(3,1);

    F_strut_R_B = zeros(3,1);
    tau_strut_R_B = zeros(3,1);

    [F_strut_FL_B, tau_strut_FL_B] = strut_drag(...
        r_FL_B, ...
        params.front_struts.distance_m, ...
        params.front_struts.chord_m, ...
        params.front_struts.CD, ...
        0.002, ...
        ...
        v_COM_B, omega_B, zW, R_BW, rho);

    [F_strut_FR_B, tau_strut_FR_B] = strut_drag(...
        r_FR_B, ...
        params.front_struts.distance_m, ...
        params.front_struts.chord_m, ...
        params.front_struts.CD, ...
        0.002, ...
        ...
        v_COM_B, omega_B, zW, R_BW, rho);

    [F_strut_R_B, tau_strut_R_B] = strut_drag(...
        r_R_B, ...
        params.front_struts.distance_m, ...
        params.front_struts.chord_m, ...
        params.front_struts.CD, ...
        0.002, ...
        ...
        v_COM_B, omega_B, zW, R_BW, rho);

    %%  Torques via cross products in body frame
    tau_FL_B = cross(r_FL_B, F_FL_B);
    tau_FR_B = cross(r_FR_B, F_FR_B);
    tau_R_B  = cross(r_R_B,  F_R_B);
    tau_T_B  = cross(r_T_B,  F_T_B);

    tau_total_B = tau_FL_B + tau_FR_B + tau_R_B + tau_T_B + ...
        tau_strut_FL_B + tau_strut_FR_B + tau_strut_R_B;

    tau_total_B = tau_total_B + [tau_roll_dist; tau_pitch_dist; 0]; % Disturbance

    tau_roll_B  = tau_total_B(1);  % about x_B
    tau_pitch_B = tau_total_B(2);  % about y_B
    tau_yaw_B   = tau_total_B(3); % about z_B

    %% Resultant forces (for heave)
    % Sum only hydrofoil forces for vertical support 
    % (thrust has no z-component in a _B frame but when boat pitches up it does have z-component in _W frame)
    F_total_B = F_FL_B + F_FR_B + F_R_B + F_T_B + ...
        F_strut_FL_B + F_strut_FR_B + F_strut_R_B;

    % Transform to world frame
    F_total_W = R_BW * F_total_B;

    % In NED: +z down, so upward support is minus the z-component
    Fz_up = -F_total_W(3);   % >0 means net upward force from foils+thrust
    
    Fz_up = Fz_up + F_z_dist; % Add heave disturbance

    %% MODEL THE DISSIPATION FORCE!!!! that's hard actually 
    F_damp_z = 0;
    % Damping b1 * zW_dot * V/V_ref
    V_ref = 0.0171;
    b1 = 128;
    F_damp_z = F_damp_z - b1*zWdot*V_submerged/V_ref;

    %% Added mass 
    % Maybe to be done
    ma = 0;
    % ma = V(zW) * rho;

    %% Rotational dynamics: Newton–Euler in body frame
    %   I_B * domega_B + omega_B x (I_B * omega_B) = tau_B
    Iomega_B = [Ix_B * omega_phi_B;
                Iy_B * omega_theta_B;
                Iz_B * omega_psi_B];

    omega_cross_Iomega = cross(omega_B, Iomega_B);

    domega_B = [ (tau_roll_B  - omega_cross_Iomega(1)) / Ix_B;
                 (tau_pitch_B - omega_cross_Iomega(2)) / Iy_B;
                 (tau_yaw_B   - omega_cross_Iomega(3)) / Iz_B ];

    %% Reduced attitude kinematics
    phi_dot_BW   = omega_phi_B;
    theta_dot_BW = omega_theta_B;
    % Diagnostic value from unconstrained ZYX kinematics
    psi_dot_BW = omega_theta_B * sin(phi_BW) / cos(theta_BW);
    % %% Attitude kinematics: Euler ZYX (phi_BW, theta_BW, psi_BW) from body rates
    % % Valid for |theta_BW| ~= 90 deg (standard singularity)
    % tan_th = tan(theta_BW);
    % sec_th = 1 / cos(theta_BW);
    % 
    % phi_dot_BW   = omega_phi_B ...
    %              + omega_theta_B * sin(phi_BW) * tan_th ...
    %              + omega_psi_B   * cos(phi_BW) * tan_th;
    % theta_dot_BW = omega_theta_B * cos(phi_BW) ...
    %              - omega_psi_B   * sin(phi_BW);
    % psi_dot_BW   = omega_theta_B * sin(phi_BW) * sec_th ...
    %              + omega_psi_B   * cos(phi_BW) * sec_th;

    %% Surge, heave dynamics in world frame (z_W is downwards)
    xWddot = (F_total_W(1) - 0) / (m + ma); % Hull drag to be done yet ;)

    zWddot = (m*g - Fz_up - FB_up + F_damp_z) / (m + ma);   % gravity down (+), Fz_up up (-z)

    %% Final ODE system (8 states)
    xdot = zeros(13,1);

    % Surge
    xdot(1) = xWdot;
    xdot(2) = xWddot;

    % Heave
    xdot(3) = zWdot;
    xdot(4) = zWddot;

    % Angles
    xdot(5) = phi_dot_BW;
    xdot(6) = theta_dot_BW;
    xdot(7) = psi_dot_BW;

    % Body rates
    xdot(8) = domega_B(1);
    xdot(9) = domega_B(2);
    xdot(10) = domega_B(3);
    
    % Actuator rates
    xdot(11)  = alpha_FL_dot;
    xdot(12) = alpha_FR_dot;
    xdot(13) = alpha_R_dot;

    %% Update the info vector used for RL
    info = [ ...
        p_FL_W(3);         % 1  [m]  front-left foil vertical position (W frame)
        p_FR_W(3);         % 2  [m]  front-right foil vertical position (W frame)
        p_R_W(3);          % 3  [m]  rear foil vertical position (W frame)
        V_submerged;       % 4  [m^3] submerged hull volume
        tau_total_B(1);    % 5  [Nm] roll torque  (body frame)
        tau_total_B(2);    % 6  [Nm] pitch torque (body frame)
        tau_total_B(3);    % 7  [Nm] yaw torque   (body frame)
        xWdot;             % 8  [m/s] forward velocity

        F_strut_FL_B(1); % 9 Newton drag force
        F_strut_FR_B(1);
        F_strut_R_B(1);

        F_FL_B(1); % 12
        F_FR_B(1);
        F_R_B(1);

        F_T_B(1); % 15
    ];
end

function [F_strut_B, tau_strut_B] = strut_drag( ...
    r_bottom_position_B, ...
    strip_distances_m, ...
    strip_chords_m, ...
    strip_CDs, ...
    strip_dz, ...
    ...
    v_COM_B, ...
    omega_B, ...
    zW, ...
    R_BW, ...
    rho)

    F_strut_B   = zeros(3,1);
    tau_strut_B = zeros(3,1);

    V_min = 1e-6;

    for i = 1:length(strip_distances_m)

        % Position of strip centre

        % Distances are measured upwards from the bottom of the strut.
        % In the NED body frame, upwards corresponds to -z_B.
        r_i_B = r_bottom_position_B ...
              - [0; 0; strip_distances_m(i)];

        % Only the z-coordinate is needed to determine immersion.
        p_i_W = [0; 0; zW] + R_BW * r_i_B;

        % Submerged strip

        % In the NED world frame, z_W > 0 is below the water surface.
        if p_i_W(3) <= 0
            continue;
        end

        % Local strip velocity

        % Velocity of the strip relative to stationary water.
        v_i_B = v_COM_B + cross(omega_B, r_i_B);
        V_i = norm(v_i_B);
        if V_i <= V_min
            continue;
        end

        % Section drag
        S_i = strip_chords_m(i) * strip_dz;

        D_i = 0.5 * rho * V_i^2 ...
            * strip_CDs(i) * S_i;

        % Drag acts opposite to the local velocity through the water.
        e_drag_i_B = -v_i_B / V_i;
        F_i_B = D_i * e_drag_i_B;

        % Accumulate force and moment about COM
        F_strut_B = F_strut_B + F_i_B;

        tau_strut_B = tau_strut_B ...
                    + cross(r_i_B, F_i_B);
    end
end
