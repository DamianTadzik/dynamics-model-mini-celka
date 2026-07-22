clc;
rear = 1620;
front_left = 1275;
front_right = 1295;

g2N = 9.80665e-3;
mmsq2msq = 1e-6;

A_i = [8080, 8080, 8708] * mmsq2msq;
R_i = [front_left, front_right, rear] * g2N; % NORMAL AS IS 
sum(R_i)
R_i = [1320, 1330, 1550] * g2N;
sum(R_i)
CL_3D_i = [0, 0, 0];
CL_2D_i = [0, 0, 0];

% Dynamic pressure q = 1/2 rho * velocity^2
q = (1/2) * 1000 * ( 2.5 )^2;

% Lift force = q * A * CL
% Lift_needed / (q * A) = CL

% CL_3d = CL_2D * AR / (AR + 2) przyblizenie wynikajace z???
AR_i = [5.15 5.15 5.56];


for V = [2, 2.5 3]
    q = (1/2) * 1000 * ( V )^2;

    % effective 3d CL
    CL_3D_i =  R_i ./ (q .* A_i);

    % approximate 2d CL
    CL_2D_i = CL_3D_i .* (AR_i + 2) ./ AR_i;

    fprintf("V = %.1f m/s\n", V);
    fprintf("Required effective CL_3D:\n");
    fprintf("  FL = %.3f, FR = %.3f, R = %.3f\n", CL_3D_i(1), CL_3D_i(2), CL_3D_i(3));
    fprintf("Equivalent CL_2D for AirfoilTools comparison:\n");
    fprintf("  FL = %.3f, FR = %.3f, R = %.3f\n\n", CL_2D_i(1), CL_2D_i(2), CL_2D_i(3));
end 

return
%% skad sie wzielo AR / AR +2
https://eaglepubs.erau.edu/introductiontoaerospaceflightvehicles/chapter/lifting-line-theory

https://eaglepubs.erau.edu/introductiontoaerospaceflightvehicles/chapter/finite-wing-characteristics/

Z DUPY

prawda jest inna:
http://brennen.caltech.edu/FLUIDBOOK/externalflows/lift/finitespanperformance.pdf

http://brennen.caltech.edu/FLUIDBOOK/externalflows/lift/finitespanperformance.pdf