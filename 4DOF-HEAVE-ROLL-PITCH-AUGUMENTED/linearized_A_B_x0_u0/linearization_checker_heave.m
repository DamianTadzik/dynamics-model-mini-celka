%% Analyze A variation along zW for fixed forward velocity

clc; clear;

load("linearization_grid_zW_xWdot.mat");

%% Select velocity slice

velocity_ms = 2.5;

[~,jv] = min(abs(xWdotGrid_ms - velocity_ms));
velocity_ms = xWdotGrid_ms(jv);

Nz = numel(zWGrid_m);

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

nominalZW_m = -0.10;
[~,iz0] = min(abs(zWGrid_m - nominalZW_m));

A0 = linearizations(iz0,jv).A;

if isempty(A0)
    error('Reference linearization is empty.');
end

A0_dynamic = A0(rowsDyn,colsDyn);
A0_control = A0(rowsDyn,colsAct);

%% Analyze variation along zW

deltaA          = nan(Nz,1);
deltaDynamic    = nan(Nz,1);
deltaControl    = nan(Nz,1);

for i = 1:Nz

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

plot(zWGrid_m,100*deltaA,'o-','LineWidth',1.5);
hold on;
plot(zWGrid_m,100*deltaDynamic,'o-','LineWidth',1.5);
plot(zWGrid_m,100*deltaControl,'o-','LineWidth',1.5);

grid on;

xlabel('Heave position $z_W$ [m]', ...
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
    'Variation along $z_W$ at $\\dot{x}_W = %.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');


%% Element-wise variation of A

n = size(A0,1);

A_values = nan(n,n,Nz);

for i = 1:Nz

    A = linearizations(i,jv).A;

    if isempty(A)
        continue;
    end

    A_values(:,:,i) = A;
end

% Absolute range of each matrix entry over zW
A_range = max(A_values,[],3,'omitnan') - ...
          min(A_values,[],3,'omitnan');

figure;

imagesc(A_range);
set(gca,'YDir','normal');
colorbar;

xlabel('Column $j$','Interpreter','latex');
ylabel('Row $i$','Interpreter','latex');

title(sprintf( ...
    'Range of $A_{ij}$ along $z_W$ at $\\dot{x}_W=%.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');


%% Numerical derivative dA/dzW

dAdz = nan(size(A_values));

for k = 1:n
    for l = 1:n

        vals = squeeze(A_values(k,l,:));

        dAdz(k,l,:) = gradient(vals,zWGrid_m);
    end
end

max_dAdz = max(abs(dAdz),[],3,'omitnan');

figure;

imagesc(max_dAdz);
set(gca,'YDir','normal');
colorbar;

xlabel('Column $j$','Interpreter','latex');
ylabel('Row $i$','Interpreter','latex');

title(sprintf( ...
    'Maximum $|\\partial A_{ij}/\\partial z_W|$ at $\\dot{x}_W=%.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');

%% Plot all rigid-body dynamic coefficients along zW

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

        plot(zWGrid_m, vals, 'o-', 'LineWidth', 1.2);

        labels{end+1} = sprintf('$A_{%d,%d}$',r,c);
    end
end

xlabel('Heave position $z_W$ [m]', ...
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


%% Plot all hydrofoil -> rigid-body coefficients along zW

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

        plot(zWGrid_m, vals, 'o-', 'LineWidth', 1.2);

        labels{end+1} = sprintf('$A_{%d,%d}$',r,c);
    end
end

xlabel('Heave position $z_W$ [m]', ...
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
            100*(vals - vals(iz0))/scale;

        plot(zWGrid_m, vals_norm, ...
            'o-', 'LineWidth',1.2);

        labels{end+1} = sprintf('$A_{%d,%d}$',r,c);
    end

    xlabel('$z_W$ [m]','Interpreter','latex');
    ylabel('Normalized change [\%]','Interpreter','latex');

    title(rowNames{rr},'Interpreter','latex');

    legend(labels, ...
        'Interpreter','latex', ...
        'Location','best');
end

%% Eigenvalues along zW for fixed forward velocity

nEig = size(A0,1);
eigReal = nan(Nz,nEig);
eigImag = nan(Nz,nEig);

for i = 1:Nz
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
    plot(zWGrid_m, eigReal(:,k), 'o-', 'LineWidth',1.2);
end

yline(0,'--');

xlabel('Heave position $z_W$ [m]', 'Interpreter','latex');
ylabel('$\mathrm{Re}\{\lambda_i\}$ [1/s]', 'Interpreter','latex');

title(sprintf( ...
    'Open-loop eigenvalues along $z_W$ at $\\dot{x}_W = %.1f$ m/s', ...
    velocity_ms), ...
    'Interpreter','latex');

legend(labels, 'Interpreter','latex', 'Location','eastoutside');