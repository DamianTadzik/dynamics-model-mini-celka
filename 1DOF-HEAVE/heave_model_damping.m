clear; clc;
%% Load the buoyancy force interpolant
load("V_interpolant.mat");
load("Fb_interpolant.mat");

%% Damping force, linear and quadratic damping
syms Fd(z) z(t) k c

k = 50;
c = 20;
% Fd(z) = k * 0.5 * diff(z,1) * abs(diff(z,1)) + c * 0.5 * diff(z,1);

% Scale it by submerged volume somehow?
% Fd(z) = Fd(z) * V(z)/v
v = 0.001;

%% Added mass (displaced water mass)

% m_eff = m + m_a(z) = m + rho * V(z)
rho = 1000;

%% Write extra simple equation, omit everything, just simple damping
syms z(t) m g Fb(z) m_eff(z)

eq_extra_simple = ...
    (m + m_eff(z))*diff(z,2) == ...
    - m*g + Fb(z) + ...
    k * 0.5 * abs(diff(z,1)) * diff(z,1) + ...
    c * 0.5 * diff(z,1);
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
    ( -m*g + Fb_interpolant(state(1)) + ... 
    (- k*0.5*state(2)*abs(state(2)) ...
    - c*0.5*state(2)) * V_interpolant(state(1))/v ) ...
    / (m + V_interpolant(state(1)*rho)) % = d^2z/dt^2
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