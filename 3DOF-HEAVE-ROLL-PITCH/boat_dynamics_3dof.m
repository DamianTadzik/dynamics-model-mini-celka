function xdot = boat_dynamics_3dof(x, u, params)
% x = [ zW; zWdot; phi; phidot; theta; thetadot ]
% inputs = struct with V_W, alpha_L, alpha_R, alpha_Rear, F_thrust
% params = struct with m, g, Ix_B, Iy_B, geometry...
%
% Output:
%   xdot = time derivative of state
%
% World frame W: NED (x forward, y right, z down), water surface at z_W = 0
% Body frame B: NED attached to COM
%
    %%  Unpack state
    zW       = x(1);
    zWdot    = x(2);
    phi      = x(3);
    phidot   = x(4);
    theta    = x(5);
    thetadot = x(6);

    %%  Unpack inputs
    alpha_Left      = u(1);
    alpha_Rear      = u(2);
    alpha_Right     = u(3);
    F_thrust        = u(4);
    V_W             = u(5); % For now it is separate input, later will add fourth equation for x_W movement

    %%  Unpack parameters
    m     = params.m;
    g     = params.g;
    rho   = params.rho;
    Ix_B  = params.Ix_B;
    Iy_B  = params.Iy_B;

    % Positions in body frame (NED)
    r_FL_B = params.hydrofoils.pos_front_left_B;   % [x;y;z]
    r_FR_B = params.hydrofoils.pos_front_right_B;
    r_R_B  = params.hydrofoils.pos_rear_B;
    r_T_B  = params.thruster.pos_thruster_B;

    % Wing areas
    S_front = params.hydrofoils.S_front;
    S_rear  = params.hydrofoils.S_rear;

    %% Rotation matrices
    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    R_x = [1  0    0;
           0  cphi -sphi;
           0  sphi  cphi];
    R_y = [ cth  0  sth;
            0    1   0;
           -sth  0  cth];
    % From B to W frame     w_frame_vector = R_BW * b_frame_vector;
    R_BW = R_x * R_y;
    % From W to B frame     b_frame_vector = R_WB * w_frame_vector;
    R_WB = R_BW.';

    %% Lift and drag coefficients (We account for the boat pitch in CD/CL calculation)
    CL_FrontLeft = interp1(params.LUT.alpha, params.LUT.CL, rad2deg(alpha_Left + theta), 'linear');
    CL_FrontRear = interp1(params.LUT.alpha, params.LUT.CL, rad2deg(alpha_Rear + theta), 'linear');
    CL_Right     = interp1(params.LUT.alpha, params.LUT.CL, rad2deg(alpha_Right + theta), 'linear');

    CD_FrontLeft = interp1(params.LUT.alpha, params.LUT.CD, rad2deg(alpha_Left + theta), 'linear');
    CD_FrontRear = interp1(params.LUT.alpha, params.LUT.CD, rad2deg(alpha_Rear + theta), 'linear');
    CD_Right     = interp1(params.LUT.alpha, params.LUT.CD, rad2deg(alpha_Right + theta), 'linear');

    %%  Lift / drag magnitudes 
    % Assumptions:
    %   - xdot_W is the only velocity that is taken into account when
    %   calculating FL and FD magintudes, we omit the zdot_W (vertical
    %   speed as it has very little influence)
    xdot_W = V_W; % Temporarly as a input

    FL_FrontLeft  = 0.5 * rho * S_front * xdot_W^2 * CL_FrontLeft;
    FL_FrontRight = 0.5 * rho * S_front * xdot_W^2 * CL_Right;
    FL_Rear       = 0.5 * rho * S_rear  * xdot_W^2 * CL_FrontRear;
    
    FD_FrontLeft  = 0.5 * rho * S_front * xdot_W^2 * CD_FrontLeft;
    FD_FrontRight = 0.5 * rho * S_front * xdot_W^2 * CD_Right;
    FD_Rear       = 0.5 * rho * S_rear  * xdot_W^2 * CD_FrontRear;

    %% Lift / drag vectors in the _W/_B? frame
    % Assumptions:
    %   - boat is moving along x_W and z_W but we exclude the vertical
    %   movement from lift/drag force calculations so xdot_W movement is the 
    %   only contributor to the lift force.
    %   - Drag is opposing the xdot_W speed always in that case drag is [-1, 0, 0]_W
    %   - Lift is almost vertical in _W, but it is tilted by a roll angle 
    %   (it is acting perpendicular to the wingspan) [0, sin(phi), -cos(phi)]_W.
    
    e_drag_W = [-1; 0; 0 ];                 % oppsite to the xdot_W
    e_lift_W = [ 0; sin(phi); -cos(phi) ];  % tilted vertical, perpendicular to xdot_W
    
    e_lift_B = R_WB * e_lift_W;
    e_drag_B = R_WB * e_drag_W;

    % Hydrofoil forces
    F_FL_B = FL_FrontLeft  * e_lift_B + FD_FrontLeft  * e_drag_B;
    F_FR_B = FL_FrontRight * e_lift_B + FD_FrontRight * e_drag_B;
    F_R_B  = FL_Rear  * e_lift_B + FD_Rear  * e_drag_B;
    % Propeller force (along +x_B)
    F_T_B  = [F_thrust; 0; 0];

    %%  Torques via cross products in body frame
    tau_FL_B = cross(r_FL_B, F_FL_B);
    tau_FR_B = cross(r_FR_B, F_FR_B);
    tau_R_B  = cross(r_R_B,  F_R_B);
    tau_T_B  = cross(r_T_B,  F_T_B);

    tau_total_B = tau_FL_B + tau_FR_B + tau_R_B + tau_T_B;

    tau_roll_B  = tau_total_B(1);  % about x_B
    tau_pitch_B = tau_total_B(2);  % about y_B

    %% Resultant forces (for heave)
    % Sum only hydrofoil forces for vertical support (thrust has no z-component)
    F_foils_B = F_FL_B + F_FR_B + F_R_B;

    % Transform to world frame
    F_foils_W = R_BW * F_foils_B;

    % In NED: +z down, so upward support is minus the z-component
    Fz_up = -F_foils_W(3);   % >0 means net upward force from foils

    %% Final ODE system
    xdot = zeros(6,1);

    % Heave dynamics in world frame (z_W is downwards)
    xdot(1) = zWdot;
    xdot(2) = (m*g - Fz_up) / m;   % gravity down (+), Fz_up up (-z)

    % Roll dynamics
    xdot(3) = phidot;
    xdot(4) = tau_roll_B  / Ix_B;

    % Pitch dynamics
    xdot(5) = thetadot;
    xdot(6) = tau_pitch_B / Iy_B;
end