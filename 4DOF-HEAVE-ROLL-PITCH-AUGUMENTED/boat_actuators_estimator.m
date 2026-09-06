function [delta_hat, delay_states] = boat_actuators_estimator(u, u0, params)
%BOAT_ACTUATORS_ESTIMATOR
% Estimates actual hydrofoil incidence angles from commanded angles.
%
% u = [alpha_FL_cmd;
%      alpha_FR_cmd;
%      alpha_R_cmd]      [deg]
%
% delta_hat = estimated actual hydrofoil angles [deg]
%
% delay_states ordering:
% [FL_d1; FR_d1; R_d1;
%  FL_d2; FR_d2; R_d2;
%  ...
%  FL_dLd; FR_dLd; R_dLd]

    persistent delta
    persistent delay_buffer

    Td = params.actuator_model.Td;
    Ld = params.actuator_model.Ld;

    alpha_min = params.actuator_model.alpha_min;
    alpha_max = params.actuator_model.alpha_max;

    % Initialization of simulation at trim point
    if isempty(delta)
        delta = u0;
        delay_buffer = repmat(u0, 1, Ld);
    end

    % Saturate commanded angles
    u = min(max(u, alpha_min), alpha_max);

    % Pure transport delay
    u_delayed = delay_buffer(:,end);

    if Ld > 1
        delay_buffer(:,2:end) = delay_buffer(:,1:end-1);
    end
    delay_buffer(:,1) = u;

    % First-order actuator dynamics
    delta = Td .* delta + (1 - Td) .* u_delayed;

    % Output
    delta_hat = delta;
    delay_states = delay_buffer(:);
end
