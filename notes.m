% Simplified equations of motion, that contains only:
% Heave (z, vertical movement)
% Surge (x, forward movement)
% Roll  (phi, rotation to the right)
% Pitch (theta, rotation backward)

% Controlled variables: 
% delta_rear
% delta_left
% delta_right

% User input variables (not meant to becontrolled but model is aware of them) :
% F_thrust
% delta_steering


%% Flight dynamics under consideration

% Roll, from momentum balance:
% M_x * diff(phi, 2) = F_lift_left * r_left + F_lift_right * r_right  

% Pitch, from momentum balance:
% M_y * diff(theta, 2) = F_thrust * h_thrust ... hard Front lifts at the
% same r, rear lift at different r, rear and front drags... this is really
% most complicated but this axis is most stable from my experience

% Heave, from forces balance in the vertical direction:
% m * diff(z, 2) = -m*g + F_lift_left + F_lift_right + F_lift_rear

% Surge, from forces balance in the forward direction:
% m * diff(x, 2) = F_thrust - F_drag_left - F_drag_right - F_drag_rear
