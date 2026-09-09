%% Find system linearizations on the trim grid.

clc; clear;

load("trim_grid_theta_xWdot.mat");

Ntheta = numel(thetaGrid_deg);
Nv     = numel(xWdotGrid_ms);

linearizations(Ntheta,Nv) = struct;

for i = 1:Ntheta
    for j = 1:Nv

        trim_k = trims(i,j).trim;

        if isempty(trim_k)
            continue;
        end

        x0_full = trim_k.x0;
        u0_full = trim_k.u0;

        [A, B, x0, u0] = find_linearized_A_B_x0_u0_4dof( ...
            x0_full, u0_full);

        linearizations(i,j).theta_deg = trims(i,j).theta_deg;
        linearizations(i,j).xWdot     = trims(i,j).xWdot;

        linearizations(i,j).A         = A;
        linearizations(i,j).B         = B;
        linearizations(i,j).x0        = x0;
        linearizations(i,j).u0        = u0;
    end
end

%% Save complete linearization grid

save("linearization_grid_theta_xWdot.mat", ...
    "linearizations", ...
    "thetaGrid_deg", ...
    "xWdotGrid_ms");

%% Analyze linearizations

% This section analyzes how the local linear model changes across the
% (xWdot, theta) trim grid.
%
% Analyzed properties:
%   1. Open-loop eigenvalues
%   2. Controllability rank
%   3. Numerical controllability measure
%   4. Relative variation of A and B with respect to a nominal trim point


%% Preallocate analysis arrays

maxRealEig       = nan(Ntheta,Nv);
minRealEig       = nan(Ntheta,Nv);

ctrbRank         = nan(Ntheta,Nv);
sigmaMinCtrb     = nan(Ntheta,Nv);

normA            = nan(Ntheta,Nv);
normB            = nan(Ntheta,Nv);

eigGrid          = cell(Ntheta,Nv);


%% Analyze every point of the grid

for i = 1:Ntheta
    for j = 1:Nv

        lin = linearizations(i,j);

        if ~isfield(lin,'A') || isempty(lin.A)
            continue;
        end

        A = lin.A;
        B = lin.B;

        %% Open-loop eigenvalues

        lambda = eig(A);

        eigGrid{i,j} = lambda;

        maxRealEig(i,j) = max(real(lambda));
        minRealEig(i,j) = min(real(lambda));


        %% Controllability

        Ctrb = ctrb(A,B);

        ctrbRank(i,j) = rank(Ctrb);

        s = svd(Ctrb);
        sigmaMinCtrb(i,j) = min(s);


        %% Matrix norms

        normA(i,j) = norm(A,'fro');
        normB(i,j) = norm(B,'fro');

    end
end


%% 1. Maximum real part of open-loop eigenvalues

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, maxRealEig);
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title('Maximum real part of open-loop eigenvalues', ...
    'Interpreter','latex');

grid on;


%% 2. Minimum real part of open-loop eigenvalues

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, minRealEig);
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title('Minimum real part of open-loop eigenvalues', ...
    'Interpreter','latex');

grid on;


%% 3. Controllability rank

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, ctrbRank);
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title('Rank of the controllability matrix', ...
    'Interpreter','latex');

grid on;


%% 4. Smallest singular value of controllability matrix

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, log10(sigmaMinCtrb));
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title('$\log_{10}(\sigma_{\min}(\mathcal{C}))$ of controllability matrix', ...
    'Interpreter','latex');

grid on;


%% 5. Frobenius norm of A

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, normA);
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title('Frobenius norm of system matrix $A$', ...
    'Interpreter','latex');

grid on;


%% 6. Frobenius norm of B

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, normB);
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title('Frobenius norm of input matrix $B$', ...
    'Interpreter','latex');

grid on;


%% Compare models with a nominal operating point

nominalVelocity_ms = 2.5;
nominalTheta_deg   = 0;

[~,iv0] = min(abs(xWdotGrid_ms - nominalVelocity_ms));
[~,it0] = min(abs(thetaGrid_deg - nominalTheta_deg));

A0 = linearizations(it0,iv0).A;
B0 = linearizations(it0,iv0).B;

if isempty(A0) || isempty(B0)
    error('Nominal linearization is empty.');
end

deltaA_rel = nan(Ntheta,Nv);
deltaB_rel = nan(Ntheta,Nv);

for i = 1:Ntheta
    for j = 1:Nv

        lin = linearizations(i,j);

        if ~isfield(lin,'A') || isempty(lin.A)
            continue;
        end

        A = lin.A;
        B = lin.B;

        deltaA_rel(i,j) = ...
            norm(A - A0,'fro') / norm(A0,'fro');

        deltaB_rel(i,j) = ...
            norm(B - B0,'fro') / norm(B0,'fro');

    end
end


%% 7. Relative variation of A

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, 100*deltaA_rel);
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title(sprintf( ...
    'Relative variation of $A$ from nominal trim ($\\dot{x}_W=%.1f$ m/s, $\\theta=%.1f^\\circ$)', ...
    xWdotGrid_ms(iv0), thetaGrid_deg(it0)), ...
    'Interpreter','latex');

grid on;


%% 8. Relative variation of B

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, 100*deltaB_rel);
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title(sprintf( ...
    'Relative variation of $B$ from nominal trim ($\\dot{x}_W=%.1f$ m/s, $\\theta=%.1f^\\circ$)', ...
    xWdotGrid_ms(iv0), thetaGrid_deg(it0)), ...
    'Interpreter','latex');

grid on;


%% Print summary

fprintf('\n=== LINEARIZATION GRID ANALYSIS ===\n');

fprintf('Grid size: %d x %d\n', Ntheta, Nv);

fprintf('\nOpen-loop eigenvalues:\n');

fprintf('Maximum max(Re(lambda)) : %+10.4e\n', ...
    max(maxRealEig,[],'all','omitnan'));

fprintf('Minimum max(Re(lambda)) : %+10.4e\n', ...
    min(maxRealEig,[],'all','omitnan'));

fprintf('\nControllability:\n');

fprintf('Minimum controllability rank : %d\n', ...
    min(ctrbRank,[],'all','omitnan'));

fprintf('Maximum controllability rank : %d\n', ...
    max(ctrbRank,[],'all','omitnan'));

fprintf('\nNominal point:\n');

fprintf('xWdot = %.3f m/s\n', xWdotGrid_ms(iv0));
fprintf('theta = %.3f deg\n', thetaGrid_deg(it0));

fprintf('\nModel variation relative to nominal point:\n');

fprintf('Maximum relative A variation : %.2f %%\n', ...
    100*max(deltaA_rel,[],'all','omitnan'));

fprintf('Maximum relative B variation : %.2f %%\n', ...
    100*max(deltaB_rel,[],'all','omitnan'));

fprintf('===================================\n');


%% Final

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

rowsDyn = [2 4 7 8];   % surge/heave/roll/pitch accelerations
colsAct = [9 10 11];   % actual hydrofoil angles
colsDyn = [2 3 4 5 6 7 8];

controlEffectiveness = nan(Ntheta,Nv);
dynamicVariation     = nan(Ntheta,Nv);

% Hydrofoil -> rigid-body acceleration coupling
A0_control = A0(rowsDyn,colsAct);

% Rigid-body dynamics only
A0_dynamic = A0(rowsDyn,colsDyn);

for i = 1:Ntheta
    for j = 1:Nv

        A = linearizations(i,j).A;

        if isempty(A)
            continue;
        end

        % Hydrofoil control effectiveness
        controlEffectiveness(i,j) = ...
            norm(A(rowsDyn,colsAct) - A0_control,'fro') / ...
            norm(A0_control,'fro');

        % Rigid-body dynamics variation
        dynamicVariation(i,j) = ...
            norm(A(rowsDyn,colsDyn) - A0_dynamic,'fro') / ...
            norm(A0_dynamic,'fro');

    end
end


%% Hydrofoil control effectiveness variation

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, 100*controlEffectiveness);
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title('Relative variation of hydrofoil control effectiveness [\%]', ...
    'Interpreter','latex');

grid on;


%% Rigid-body dynamics variation

figure;

imagesc(xWdotGrid_ms, thetaGrid_deg, 100*dynamicVariation);
set(gca,'YDir','normal');
colorbar;

xlabel('Forward velocity $\dot{x}_W$ [m/s]', ...
    'Interpreter','latex');

ylabel('Pitch angle $\theta$ [deg]', ...
    'Interpreter','latex');

title('Relative variation of rigid-body dynamics [\%]', ...
    'Interpreter','latex');

grid on;
