clear; clc;
%% Load the buoyancy force interpolant
load("Fb_interpolant.mat");

%% Damping force, just hydrodynamical resistance of the hyrofoils
syms Fd(z) z(t) k
A = (8080 + 8080 + 8700) * 1e-6;   % m^2
rho = 1000;                        % water, kg/m^3
Cd = 0.8; % Close to 1 since they are just flat and perpendicular to the speed vector in that case
k = rho * Cd * A; % combined drag coefficient
% Fd(z) = k * 0.5 * diff(z,1) * abs(diff(z,1));

%% Write extra simple equation, omit everything, just simple damping
syms z(t) m g Fb(z)

eq_extra_simple = m*diff(z,2) == - m*g + Fb(z) + k * 0.5 * abs(diff(z,1)) * diff(z,1);
simplify(eq_extra_simple)

%% Simulate this extra simple equation

m = 4.161;          % kg
g = 9.81;
z0 = 0.08;        % initial position [m]
zdot0 = 0.0;      % initial velocity [m/s]

z_init = [z0; zdot0];
dt = 0.01;                    % time step [s]
tspan = 0:dt:10;              % equal output times

% Define ODE: state = [z; zdot]
odefun = @(t, state) [
    state(2);                             % = dz/dt
    ( -m*g + Fb_interpolant(state(1)) - k*0.5*state(2)*abs(state(2)) ) / m % = d^2z/dt^2
];

[t, Z] = ode45(odefun, tspan, z_init);

% Extract
z = Z(:,1);
zdot = Z(:,2);

% Plot
figure; 
subplot(2,1,1)
plot(t, z, 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('z [m]');
grid on; title('Heave displacement');

subplot(2,1,2)
plot(t, zdot, 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('ż [m/s]');
grid on; title('Heave velocity');


%% Write simple equation, omit the force from foils
syms z(t) m g Fb(z)

eq_extra_simple = m*diff(z,2) == - m*g + Fb(z);
simplify(eq_extra_simple)