function [F_strut_B, tau_strut_B] = strut_dragg(...
    r_bottom_position_B, ...
    strip_distances_m, ...
    strip_chords_m, ...
    strip_CDs, ...
    strip_dz, ...
    xWdot, ...
    zW, ...
    R_BW, ...
    R_WB, ...
    rho)
    
    F_strut_B = zeros(3,1);
    tau_strut_B = zeros(3,1);

    e_drag_W = [-1; 0; 0];  % oppsite to the xdot_W? approximately yes
    e_drag_B = R_WB * e_drag_W;


    for i = 1:length(strip_distances_m)
        % Position of each strip centre in body frame
        r_i_B = r_bottom_position_B - [0; 0; strip_distances_m(i)];

        % Position in world frame
        p_i_W = [0; 0; zW] + R_BW * r_i_B;

        % NED: z > 0 means below water surface
        if p_i_W(3) > 0
            % Surface area
            S_i = strip_chords_m(i) * strip_dz;

            % Drag
            D_i = 0.5 * rho * xWdot^2 ...
                * strip_CDs(i) ...
                * S_i;

            F_i_B = D_i * e_drag_B;

            F_strut_B = F_strut_B + F_i_B;
            tau_strut_B = tau_strut_B ...
                        + cross(r_i_B, F_i_B);
        end
    end
end
