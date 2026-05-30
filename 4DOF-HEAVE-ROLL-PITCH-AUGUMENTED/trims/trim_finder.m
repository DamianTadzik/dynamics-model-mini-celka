clc; clear;

[trim_k, ~] = find_equilibrium_4dof();

thetas = -5:0.05:3;
trims(numel(thetas)) = struct; % preallocate

for k = 1:numel(thetas)
    th = thetas(k);

    [trim_k,info_k] = find_equilibrium_4dof( ...
        'theta', th, ...
        'v0',    trim_k.v);

    trims(k).theta = th;
    trims(k).trim  = trim_k;
    trims(k).info  = info_k;
end

%% Plot

theta_deg = [trims.theta];
x0_mat = cell2mat(arrayfun(@(s) s.trim.x0, trims, ...
                           'UniformOutput', false));
xdot_mat = cell2mat(arrayfun(@(s) s.trim.xdot, trims, ...
                             'UniformOutput', false));
info_mat = cell2mat(arrayfun(@(s) s.info(:), trims, ...
                             'UniformOutput', false));

% Theta vs surge speed
plot(theta_deg, x0_mat(2,:), '-o');
xlabel('\theta [deg]');
ylabel('xWdot [m/s]');
grid on;

% Theta vs heave position (zW)
plot(theta_deg, x0_mat(3,:), '-o');
xlabel('\theta [deg]');
ylabel('zW [m]');
grid on;

% Theta vs actuator trims
figure; hold on; grid on;
plot(theta_deg, x0_mat(11,:), '-o');
plot(theta_deg, x0_mat(12,:), '-o');
plot(theta_deg, x0_mat(13,:), '-o');
xlabel('\theta [deg]');
ylabel('\alpha [deg]');
legend('FL','FR','R');

% Select the residuals
idx = [2 4 8 9]; % surge, heave, roll, pitch accels
% Theta vs residuals
figure; grid on;
plot(theta_deg, xdot_mat(idx,:).'); 
xlabel('\theta [deg]');
ylabel('Residual');
legend('ẍ','z̈','ṗ','q̇');

res_norm = vecnorm(xdot_mat(idx,:),2,1);
% Convergence plot
figure
plot(theta_deg, res_norm, '-o');
xlabel('\theta [deg]');
ylabel('||trim residual||');
grid on;

figure; hold on; grid on;
plot(theta_deg, info_mat(1,:), '-o');
plot(theta_deg, info_mat(2,:), '-o');
plot(theta_deg, info_mat(3,:),  '-o');
xlabel('\theta [deg]');
ylabel('Foil vertical position z_W [m]');
legend('Front Left','Front Right','Rear');
title('Foil immersion vs pitch trim');