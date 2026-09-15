%% Analyze A variation along forward velocity for fixed heave

clc; clear;

load("linearization_grid_zW_xWdot.mat");

%% Select heave slice

zW_m = -0.10;

[~,iz] = min(abs(zWGrid_m - zW_m));
zW_m = zWGrid_m(iz);

Nv = numel(xWdotGrid_ms);

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

%% Choose reference point within this heave slice

nominalVelocity_ms = 2.5;
[~,iv0] = min(abs(xWdotGrid_ms - nominalVelocity_ms));

A0 = linearizations(iz,iv0).A;

if isempty(A0)
    error('Reference linearization is empty.');
end

A0_dynamic = A0(rowsDyn,colsDyn);
A0_control = A0(rowsDyn,colsAct);

%% Analyze variation along velocity

deltaA       = nan(Nv,1);
deltaDynamic = nan(Nv,1);
deltaControl = nan(Nv,1);

for j = 1:Nv

    A = linearizations(iz,j).A;

    if isempty(A)
        continue;
    end

    deltaA(j) = ...
        norm(A - A0,'fro') / norm(A0,'fro');

    deltaDynamic(j) = ...
        norm(A(rowsDyn,colsDyn) - A0_dynamic,'fro') / ...
        norm(A0_dynamic,'fro');

    deltaControl(j) = ...
        norm(A(rowsDyn,colsAct) - A0_control,'fro') / ...
        norm(A0_control,'fro');
end

%% Plot relative variation

figure;

plot(xWdotGrid_ms,100*deltaA,'o-','LineWidth',1.5);
hold on;
plot(xWdotGrid_ms,100*deltaDynamic,'o-','LineWidth',1.5);
plot(xWdotGrid_ms,100*deltaControl,'o-','LineWidth',1.5);

grid on;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
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
    'Variation along $\\dot{x}_W$ at $z_W = %.2f$ m', ...
    zW_m), ...
    'Interpreter','latex');


%% Element-wise variation of A

n = size(A0,1);

A_values = nan(n,n,Nv);

for j = 1:Nv

    A = linearizations(iz,j).A;

    if isempty(A)
        continue;
    end

    A_values(:,:,j) = A;
end

A_range = max(A_values,[],3,'omitnan') - ...
          min(A_values,[],3,'omitnan');

figure;

imagesc(A_range);
set(gca,'YDir','normal');
colorbar;

xlabel('Column $j$','Interpreter','latex');
ylabel('Row $i$','Interpreter','latex');

title(sprintf( ...
    'Range of $A_{ij}$ along $\\dot{x}_W$ at $z_W=%.2f$ m', ...
    zW_m), ...
    'Interpreter','latex');


%% Numerical derivative dA/dxWdot

dAdV = nan(size(A_values));

for k = 1:n
    for l = 1:n

        vals = squeeze(A_values(k,l,:));

        dAdV(k,l,:) = gradient(vals,xWdotGrid_ms);
    end
end

max_dAdV = max(abs(dAdV),[],3,'omitnan');

figure;

imagesc(max_dAdV);
set(gca,'YDir','normal');
colorbar;

xlabel('Column $j$','Interpreter','latex');
ylabel('Row $i$','Interpreter','latex');

title(sprintf( ...
    'Maximum $|\\partial A_{ij}/\\partial \\dot{x}_W|$ at $z_W=%.2f$ m', ...
    zW_m), ...
    'Interpreter','latex');


%% Plot all rigid-body dynamic coefficients along velocity

figure;
hold on;
grid on;

labels = {};

for r = rowsDyn
    for c = colsDyn

        vals = squeeze(A_values(r,c,:));

        if max(abs(vals),[],'omitnan') < 1e-10
            continue;
        end

        plot(xWdotGrid_ms, vals, 'o-', 'LineWidth',1.2);

        labels{end+1} = sprintf('$A_{%d,%d}$',r,c);
    end
end

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('$A_{ij}$', ...
    'Interpreter','latex');

title(sprintf( ...
    'Rigid-body coefficients at $z_W = %.2f$ m', ...
    zW_m), ...
    'Interpreter','latex');

legend(labels, ...
    'Interpreter','latex', ...
    'Location','eastoutside');


%% Plot all hydrofoil -> rigid-body coefficients along velocity

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

        plot(xWdotGrid_ms, vals, 'o-', 'LineWidth',1.2);

        labels{end+1} = sprintf('$A_{%d,%d}$',r,c);
    end
end

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('$A_{ij}$', ...
    'Interpreter','latex');

title(sprintf( ...
    'Hydrofoil control-effectiveness coefficients at $z_W = %.2f$ m', ...
    zW_m), ...
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
            100*(vals - vals(iv0))/scale;

        plot(xWdotGrid_ms, vals_norm, ...
            'o-', 'LineWidth',1.2);

        labels{end+1} = sprintf('$A_{%d,%d}$',r,c);
    end

    xlabel('$\dot{x}_W$ [m/s]','Interpreter','latex');
    ylabel('Normalized change [\%]','Interpreter','latex');

    title(rowNames{rr},'Interpreter','latex');

    legend(labels, ...
        'Interpreter','latex', ...
        'Location','best');
end


%% Eigenvalues along velocity for fixed heave

nEig = size(A0,1);
eigReal = nan(Nv,nEig);
eigImag = nan(Nv,nEig);

for j = 1:Nv

    A = linearizations(iz,j).A;

    if isempty(A)
        continue;
    end

    lambda = eig(A);
    lambda = sortrows([real(lambda), imag(lambda)], [1 2]);
    lambda = lambda(:,1) + 1i*lambda(:,2);

    eigReal(j,:) = real(lambda).';
    eigImag(j,:) = imag(lambda).';
end

labels = arrayfun(@(k) sprintf('\\lambda_{%d}', k), ...
    1:nEig, 'UniformOutput', false);

f = figure('Name','open_loop_eigenvalues_in_f_of_velocity','Units','centimeters','Position',[2 2 16 9]);
hold on;
grid on;

for k = 1:nEig
    plot(xWdotGrid_ms, eigReal(:,k), 'o--', 'LineWidth',1.2);
end

yline(0,'--');

xlabel('Forward velocity ẋ_{W} [m/s]', ...
    'Interpreter','tex', ...
    'FontName','Times New Roman','FontSize',9);

ylabel('Re(\lambda_i) [1/s]', ...
    'Interpreter','tex', ...
    'FontName','Times New Roman','FontSize',9);

legend(labels, ...
    'Interpreter','tex', ...
    'Location','eastoutside', ...
    'FontName','Times New Roman','FontSize',9);
grid on;
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);

exportgraphics(f, 'open_loop_eigenvalues_in_f_of_velocity.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 16, ...
    'Height', 9);
