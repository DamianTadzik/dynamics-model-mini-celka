function [ xdot, info ] = boat_dynamics_3dof(x, u, w, params)
%  Input
%   x = [
%       zW;              % 1 heave position (world frame, NED)
%       zWdot;           % 2 heave velocity (world frame)
%
%       phi_BW;          % 3 roll angle   (Body relative to World)
%       theta_BW;        % 4 pitch angle  (Body relative to World)
%       psi_BW;          % 5 yaw angle    (Body relative to World)
%
%       omega_phi_B;     % 6 roll  rate  in BODY frame
%       omega_theta_B;   % 7 pitch rate  in BODY frame
%       omega_psi_B;     % 8 yaw   rate  in BODY frame
%
%       alpha_FL         % 9  [deg]
%       alpha_FR         % 10 [deg]
%       alpha_R          % 11 [deg]
%   ];
%   u = [
%       alpha_FrontLeft  % 1  [deg]
%       alpha_FrontRight % 2  [deg]
%       alpha_Rear       % 3  [deg]
%       F_thrust
%       V_W      (approx surge speed in world x)
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
    zW            = x(1);
    zWdot         = x(2);

    phi_BW        = x(3);
    theta_BW      = x(4);
    psi_BW        = x(5);

    omega_phi_B   = x(6);
    omega_theta_B = x(7);
    omega_psi_B   = x(8);

    alpha_FL_act  = x(9);
    alpha_FR_act  = x(10);
    alpha_R_act   = x(11);

    omega_B = [omega_phi_B; omega_theta_B; omega_psi_B];

    %%  Unpack inputs
    alpha_FL_cmd  = u(1);
    alpha_FR_cmd  = u(2);
    alpha_R_cmd   = u(3);
    F_thrust        = u(4);
    V_W             = u(5); % For now it is separate input, later will add fourth equation for x_W movement
    
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

    % CL CD LUTs 
    LUT_alpha = params.hydrofoils.LUT.alpha;
    LUT_CL    = params.hydrofoils.LUT.CL;
    LUT_CD    = params.hydrofoils.LUT.CD;

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

    %% Lift and drag coefficients (We account for the boat pitch in CD/CL calculation)
    alpha_deg_FL = alpha_FL_act  + rad2deg(theta_BW); % [deg]
    alpha_deg_FR = alpha_FR_act  + rad2deg(theta_BW);
    alpha_deg_R  = alpha_R_act   + rad2deg(theta_BW);

    CL_FrontLeft  = interp1(LUT_alpha, LUT_CL, alpha_deg_FL, 'linear', 'extrap');
    CL_FrontRight = interp1(LUT_alpha, LUT_CL, alpha_deg_FR, 'linear', 'extrap');
    CL_Rear       = interp1(LUT_alpha, LUT_CL, alpha_deg_R,  'linear', 'extrap');

    CD_FrontLeft  = interp1(LUT_alpha, LUT_CD, alpha_deg_FL, 'linear', 'extrap');
    CD_FrontRight = interp1(LUT_alpha, LUT_CD, alpha_deg_FR, 'linear', 'extrap');
    CD_Rear       = interp1(LUT_alpha, LUT_CD, alpha_deg_R,  'linear', 'extrap');

    %%  Lift / drag magnitudes 
    % Assumptions:
    %   - xdot_W is the only velocity that is taken into account when
    %   calculating FL and FD magintudes, we omit the zdot_W (vertical
    %   speed as it has very little influence - To Be Checked)
    %   - we also omit the speed coming from the rotation...
    xdot_W = V_W; % Temporarly as a input

    FL_FrontLeft  = 0.5 * rho * S_front * xdot_W^2 * CL_FrontLeft;
    FL_FrontRight = 0.5 * rho * S_front * xdot_W^2 * CL_FrontRight;
    FL_Rear       = 0.5 * rho * S_rear  * xdot_W^2 * CL_Rear;
    
    FD_FrontLeft  = 0.5 * rho * S_front * xdot_W^2 * CD_FrontLeft;
    FD_FrontRight = 0.5 * rho * S_front * xdot_W^2 * CD_FrontRight;
    FD_Rear       = 0.5 * rho * S_rear  * xdot_W^2 * CD_Rear;

    %% Lift / drag vectors in the _W/_B? frame
    % Assumptions:
    %   - boat is moving along x_W and z_W but we exclude the vertical
    %   movement from lift/drag force calculations so xdot_W movement is the 
    %   only contributor to the lift force.
    %   - Drag is opposing the xdot_W speed always in that case drag is [-1, 0, 0]_W
    %   - Lift is almost vertical in _W, but it is tilted by a roll angle 
    %   (it is acting perpendicular to the wingspan) [0, sin(phi), -cos(phi)]_W.
    
    e_drag_W = [-1; 0; 0 ];                 % oppsite to the xdot_W
    e_lift_W = [ 0; sin(phi_BW); -cos(phi_BW) ];  % tilted vertical, perpendicular to xdot_W
    
    e_lift_B = R_WB * e_lift_W;
    e_drag_B = R_WB * e_drag_W;

    % Hydrofoil forces
    F_FL_B = FL_FrontLeft  * e_lift_B + FD_FrontLeft  * e_drag_B;
    F_FR_B = FL_FrontRight * e_lift_B + FD_FrontRight * e_drag_B;
    F_R_B  = FL_Rear  * e_lift_B + FD_Rear  * e_drag_B;

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

    
    %%  Torques via cross products in body frame
    tau_FL_B = cross(r_FL_B, F_FL_B);
    tau_FR_B = cross(r_FR_B, F_FR_B);
    tau_R_B  = cross(r_R_B,  F_R_B);
    tau_T_B  = cross(r_T_B,  F_T_B);

    tau_total_B = tau_FL_B + tau_FR_B + tau_R_B + tau_T_B;

    tau_total_B = tau_total_B + [tau_roll_dist; tau_pitch_dist; 0]; % Disturbance

    tau_roll_B  = tau_total_B(1);  % about x_B
    tau_pitch_B = tau_total_B(2);  % about y_B
    tau_yaw_B   = tau_total_B(3); % about z_B

    %% Resultant forces (for heave)
    % Sum only hydrofoil forces for vertical support 
    % (thrust has no z-component in a _B frame but when boat pitches up it does have z-component in _W frame)
    F_total_B = F_FL_B + F_FR_B + F_R_B + F_T_B;

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

    %% Attitude kinematics: Euler ZYX (phi_BW, theta_BW, psi_BW) from body rates
    % Valid for |theta_BW| ~= 90 deg (standard singularity)
    tan_th = tan(theta_BW);
    sec_th = 1 / cos(theta_BW);

    phi_dot_BW   = omega_phi_B ...
                 + omega_theta_B * sin(phi_BW) * tan_th ...
                 + omega_psi_B   * cos(phi_BW) * tan_th;

    theta_dot_BW = omega_theta_B * cos(phi_BW) ...
                 - omega_psi_B   * sin(phi_BW);

    psi_dot_BW   = omega_theta_B * sin(phi_BW) * sec_th ...
                 + omega_psi_B   * cos(phi_BW) * sec_th;

    %% Heave dynamics in world frame (z_W is downwards)
    zW_ddot = (m*g - Fz_up - FB_up + F_damp_z) / (m + ma);   % gravity down (+), Fz_up up (-z)

    %% Final ODE system (8 states)
    xdot = zeros(11,1);

    % Heave
    xdot(1) = zWdot;
    xdot(2) = zW_ddot;

    % Angles
    xdot(3) = phi_dot_BW;
    xdot(4) = theta_dot_BW;
    xdot(5) = psi_dot_BW;

    % Body rates
    xdot(6) = domega_B(1);
    xdot(7) = domega_B(2);
    xdot(8) = domega_B(3);
    
    % Actuator rates
    xdot(9)  = alpha_FL_dot;
    xdot(10) = alpha_FR_dot;
    xdot(11) = alpha_R_dot;

    %% Update the info vector
    % TBD some info regarding out of water for simulation stop or sth like
    % that
    info = [ F_FL_B(3); F_FR_B(3); F_R_B(3) ];
end
