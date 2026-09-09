%% Analyze A variation along theta for fixed forward velocity

clc; clear;

load("linearization_grid_theta_xWdot.mat");

%% Select velocity slice

velocity_ms = 2.5;

[~,jv] = min(abs(xWdotGrid_ms - velocity_ms));
velocity_ms = xWdotGrid_ms(jv);

Ntheta = numel(thetaGrid_deg);

%% State groups

% Reduced state ordering:
% 1 xW
% 2 xWdot
% 3 zW
% 4 zWdot
% 5 phi
% 6 theta
% 7 p
% 8 q
% 9 delta_FL
% 10 delta_FR
% 11 delta_R

rowsDyn = [2 4 7 8];
colsDyn = [2 3 4 5 6 7 8];
colsAct = [9 10 11];

%% Choose reference point within this velocity slice

nominalTheta_deg = 0;
[~,it0] = min(abs(thetaGrid_deg - nominalTheta_deg));

A0 = linearizations(it0,jv).A;

if isempty(A0)
    error('Reference linearization is empty.');
end

A0_dynamic = A0(rowsDyn,colsDyn);
A0_control = A0(rowsDyn,colsAct);

%% Analyze variation along theta

deltaA          = nan(Ntheta,1);
deltaDynamic    = nan(Ntheta,1);
deltaControl    = nan(Ntheta,1);

for i = 1:Ntheta

    A = linearizations(i,jv).A;

    if isempty(A)
        continue;
    end

    deltaA(i) = ...
        norm(A - A0,'fro') / norm(A0,'fro');

    deltaDynamic(i) = ...
        norm(A(rowsDyn,colsDyn) - A0_dynamic,'fro') / ...
        norm(A0_dynamic,'fro');

    deltaControl(i) = ...
        norm(A(rowsDyn,colsAct) - A0_control,'fro') / ...
        norm(A0_control,'fro');
end

%% Plot relative variation

figure;

plot(thetaGrid_deg,100*deltaA,'o-','LineWidth',1.5);
hold on;
plot(thetaGrid_deg,100*deltaDynamic,'o-','LineWidth',1.5);
plot(thetaGrid_deg,100*deltaControl,'o-','LineWidth',1.5);

grid on;

xlabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

ylabel('Relative variation [\%]', ...
    'Interpreter','latex');

legend( ...
    '$A$', ...
    'Rigid-body dynamics', ...
    'Hydrofoil control effectiveness', ...
    'Interpreter','latex', ...
    'Location','best');

title(sprintf( ...
    'Variation along $\\theta$ at $\\dot{x}_W = %.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');


%% Element-wise variation of A

n = size(A0,1);

A_values = nan(n,n,Ntheta);

for i = 1:Ntheta

    A = linearizations(i,jv).A;

    if isempty(A)
        continue;
    end

    A_values(:,:,i) = A;
end

% Absolute range of each matrix entry over theta
A_range = max(A_values,[],3,'omitnan') - ...
          min(A_values,[],3,'omitnan');

figure;

imagesc(A_range);
set(gca,'YDir','normal');
colorbar;

xlabel('Column $j$','Interpreter','latex');
ylabel('Row $i$','Interpreter','latex');

title(sprintf( ...
    'Range of $A_{ij}$ along $\\theta$ at $\\dot{x}_W=%.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');


%% Numerical derivative dA/dtheta

dAdtheta = nan(size(A_values));

for k = 1:n
    for l = 1:n

        vals = squeeze(A_values(k,l,:));

        dAdtheta(k,l,:) = gradient(vals,thetaGrid_deg);
    end
end

max_dAdtheta = max(abs(dAdtheta),[],3,'omitnan');

figure;

imagesc(max_dAdtheta);
set(gca,'YDir','normal');
colorbar;

xlabel('Column $j$','Interpreter','latex');
ylabel('Row $i$','Interpreter','latex');

title(sprintf( ...
    'Maximum $|\\partial A_{ij}/\\partial \\theta|$ at $\\dot{x}_W=%.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');


%% Plot all rigid-body dynamic coefficients along theta

figure;
hold on;
grid on;

labels = {};

for r = rowsDyn
    for c = colsDyn

        vals = squeeze(A_values(r,c,:));

        % Skip coefficients which are zero over the whole slice
        if max(abs(vals),[],'omitnan') < 1e-10
            continue;
        end

        plot(thetaGrid_deg, vals, 'o-', 'LineWidth', 1.2);

        labels{end+1} = sprintf('$A_{%d,%d}$',r,c);
    end
end

xlabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

ylabel('$A_{ij}$', ...
    'Interpreter','latex');

title(sprintf( ...
    'Rigid-body coefficients at $\\dot{x}_W = %.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');

legend(labels, ...
    'Interpreter','latex', ...
    'Location','eastoutside');


%% Plot all hydrofoil -> rigid-body coefficients along theta

figure;
hold on;
grid on;

labels = {};

for r = rowsDyn
    for c = colsAct

        vals = squeeze(A_values(r,c,:));

        if max(abs(vals),[],'omitnan') < 1e-10
            continue;
        end

        plot(thetaGrid_deg, vals, 'o-', 'LineWidth', 1.2);

        labels{end+1} = sprintf('$A_{%d,%d}$',r,c);
    end
end

xlabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

ylabel('$A_{ij}$', ...
    'Interpreter','latex');

title(sprintf( ...
    'Hydrofoil control-effectiveness coefficients at $\\dot{x}_W = %.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');

legend(labels, ...
    'Interpreter','latex', ...
    'Location','eastoutside');


%% Normalized variation grouped by dynamic equation

figure;
tiledlayout(2,2);

rowNames = {
    '$\ddot{x}_W$'
    '$\ddot{z}_W$'
    '$\dot{p}$'
    '$\dot{q}$'
};

for rr = 1:numel(rowsDyn)

    r = rowsDyn(rr);

    nexttile;
    hold on;
    grid on;

    labels = {};

    for c = colsDyn

        vals = squeeze(A_values(r,c,:));

        if max(abs(vals),[],'omitnan') < 1e-10
            continue;
        end

        scale = max(abs(vals),[],'omitnan');

        vals_norm = ...
            100*(vals - vals(it0))/scale;

        plot(thetaGrid_deg, vals_norm, ...
            'o-', 'LineWidth',1.2);

        labels{end+1} = sprintf('$A_{%d,%d}$',r,c);
    end

    xlabel('$\theta$ [deg]','Interpreter','latex');
    ylabel('Normalized change [\%]','Interpreter','latex');

    title(rowNames{rr},'Interpreter','latex');

    legend(labels, ...
        'Interpreter','latex', ...
        'Location','best');
end

%% Eigenvalues along theta for fixed forward velocity

nEig = size(A0,1);
eigReal = nan(Ntheta,nEig);
eigImag = nan(Ntheta,nEig);

for i = 1:Ntheta
    A = linearizations(i,jv).A;

    if isempty(A)
        continue;
    end

    lambda = eig(A);
    lambda = sortrows([real(lambda), imag(lambda)], [1 2]);
    lambda = lambda(:,1) + 1i*lambda(:,2);

    eigReal(i,:) = real(lambda).';
    eigImag(i,:) = imag(lambda).';
end

labels = arrayfun(@(k) sprintf('$\\lambda_{%d}$',k), ...
    1:nEig, 'UniformOutput', false);

figure;
hold on;
grid on;

for k = 1:nEig
    plot(thetaGrid_deg, eigReal(:,k), 'o-', 'LineWidth',1.2);
end

yline(0,'--');

xlabel('Pitch angle $\theta$ [deg]', 'Interpreter','latex');
ylabel('$\mathrm{Re}\{\lambda_i\}$ [1/s]', 'Interpreter','latex');

title(sprintf( ...
    'Open-loop eigenvalues along $\\theta$ at $\\dot{x}_W = %.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');

legend(labels, 'Interpreter','latex', 'Location','eastoutside');