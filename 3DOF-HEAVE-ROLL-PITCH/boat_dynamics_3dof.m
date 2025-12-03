function xdot = boat_dynamics_3dof(x, u, params)
% x = [ zW; zWdot; phi; phidot; theta; thetadot ]
% inputs = struct with V_W, alpha_L, alpha_R, alpha_Rear, F_thrust
% params = struct with m, g, Ix_B, Iy_B, geometry...
%
% Output:
%   xdot = time derivative of state

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
    V_W             = u(5);

    %%  Unpack parameters
    m     = params.m;
    g     = params.g;
    Ix_B  = params.Ix_B;
    Iy_B  = params.Iy_B;

    d_front_B = params.d_front_B;
    d_rear_B  = params.d_rear_B;
    h_front_B = params.h_front_B;
    h_rear_B  = params.h_rear_B;

    S_front = params.S_front;
    S_rear  = params.S_rear;
    rho     = params.rho;

    %%  Foil forces
    CL_Left = interp1(params.LUT_alpha, params.LUT_CL, rad2deg(alpha_Left + theta), 'linear');
    CL_Rear = interp1(params.LUT_alpha, params.LUT_CL, rad2deg(alpha_Rear + theta), 'linear');
    CL_Right = interp1(params.LUT_alpha, params.LUT_CL, rad2deg(alpha_Right + theta), 'linear');

    CD_Left = interp1(params.LUT_alpha, params.LUT_CD, rad2deg(alpha_Left + theta), 'linear');
    CD_Rear = interp1(params.LUT_alpha, params.LUT_CD, rad2deg(alpha_Rear + theta), 'linear');
    CD_Right = interp1(params.LUT_alpha, params.LUT_CD, rad2deg(alpha_Right + theta), 'linear');

    FL_Left = 0.5 * rho * S_front * V_W^2 * CL_Left;
    FL_Rear = 0.5 * rho * S_rear * V_W^2 * CL_Rear;
    FL_Right = 0.5 * rho * S_front * V_W^2 * CL_Right;
    
    FD_Left = 0.5 * rho * S_front * V_W^2 * CD_Left;
    FD_Rear = 0.5 * rho * S_rear * V_W^2 * CD_Rear;
    FD_Right = 0.5 * rho * S_front * V_W^2 * CD_Right;
    
    FL_front = FL_Left + FL_Right;
    FD_front = FD_Left + FD_Right;

    %%  Heave force (vertical)
    Fz = (FL_Left + FL_Right + FL_Rear) * cos(phi);   % roll reduces vertical support

    %%  Roll torque
    tau_roll = params.r_front_B * (FL_Left - FL_Right);

    %%  Pitch torque
    %   Lift torques
    tau_front_L = ( d_front_B*cos(theta) + h_front_B*sin(theta) ) * FL_front;
    tau_rear_L  = ( d_rear_B*cos(theta)  + h_rear_B*sin(theta)  ) * FL_Rear;

    %   Drag torques
    tau_front_D = ( -h_front_B*cos(theta) + d_front_B*sin(theta) ) * FD_front;
    tau_rear_D  = ( -h_rear_B*cos(theta)  - d_rear_B*sin(theta) ) * FD_Rear;

    %   Thrust torque
    tau_thrust = params.h_thrust_B * F_thrust;

    tau_pitch = tau_front_L ...
              - tau_rear_L ...
              - tau_front_D ...
              - tau_rear_D ...
              + tau_thrust;

    %%  Final ODE system
    xdot = zeros(6,1);

    xdot(1) = zWdot;
    xdot(2) = (m*g - Fz) / m;

    xdot(3) = phidot;
    xdot(4) = tau_roll / Ix_B;

    xdot(5) = thetadot;
    xdot(6) = tau_pitch / Iy_B;

end