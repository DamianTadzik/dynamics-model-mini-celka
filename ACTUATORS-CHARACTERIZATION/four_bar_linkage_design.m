clear; clc; close all;

%% Geometry - replace with your values
A = [43.8, 194];          % servo axis position [mm]
D = [0, 0];       % hydrofoil hinge axis position [mm]

r_crank  = 18.8;       % servo horn length |A-B| [mm]
l_pushrod = 195.5;      % pushrod length |B-C| [mm]
r_rocker = 25;       % rocker arm length |D-C| [mm]

delta_min = -13.2; % [deg]
delta_max = 11.5; % [deg] TODO oblicz theta min theta max na podstawie: phi_min phi_max 

% TODO na potem, troche wariancji trzeba dodac do pomiarow dlugosci
% popychacza bo nie ma pewnosci ze jest idealnie zalozony na swoje L moze
% sie myli o 2mm nawet xd 
deltas_deg = [-75:4:delta_min, delta_min:.5:delta_max, delta_max:4:75].';   % servo angle sweep [deg]
deltas = deg2rad(deltas_deg + 180);

%% Solve four-bar kinematics
A_points  = repmat(A, length(deltas), 1);
D_points  = repmat(D, length(deltas), 1);

B_points  = nan(length(deltas), 2);

C1_points = nan(length(deltas), 2); % possible C branch
C2_points = nan(length(deltas), 2); % possible C branch

C_points  = nan(length(deltas), 2); % selected branch
alphas    = nan(size(deltas));   % selected branch


function [B, C1, C2, C, alpha] = kinematics(A, D, r_crank, l_pushrod, r_rocker, delta)
    % Point B: end of servo horn / crank
    B = A + r_crank * [cos(delta), sin(delta)];

    % Circle-circle intersection:
    % circle 1: center B, radius l_pushrod
    % circle 2: center D, radius r_rocker
    % Return C1 C2: up to two possible intersections
    [C1, C2, valid] = circleIntersection(B, l_pushrod, D, r_rocker);

    if ~valid
        warning('circleIntersection:NoSolution', ...
            'No intersection between circles at theta = %.2f deg.', rad2deg(delta));
    end

    % Select correct branch
    persistent previous_C;
    if isempty(previous_C)
        % Choose manually if needed: C1 or C2
        C = C1;
    else
        % Continuity condition
        if norm(C1 - previous_C) < norm(C2 - previous_C)
            C = C1;
        else
            C = C2;
        end
    end

    alpha = atan2(C(2) - D(2), C(1) - D(1));
end

clear kinematics;
for i = 1:length(deltas)

    [B, C1, C2, C, alpha] = kinematics(A, D, r_crank, l_pushrod, r_rocker, deltas(i));

    B_points(i, :) = B;

    C1_points(i, :) = C1;
    C2_points(i, :) = C2;

    C_points(i, :) = C;
    alphas(i, :) = alpha;
end

alphas = alphas .* -1;
alphas_deg = rad2deg(alphas);


%% PLOT the faint point clouds of the linkage in all positions
% also mark few positions like max min neutral
% also print the letters A B C D
% and the dotted line at the boottom with an angle written for max and min
% and also on the top

figure('Name','fig_four_bar_mechanism')

subplot(1,2,2)
hold on;
axis equal;
% grid on;

% Chmura po łuku
plot(B_points(:,1),  B_points(:,2),  '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 1);
% plot(C1_points(:,1), C1_points(:,2), '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 1); %Docelowo nie plotujemy tego C1 i C2
% plot(C2_points(:,1), C2_points(:,2), '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 1); %Docelowo nie plotujemy tego C1 i C2
plot(C_points(:,1),  C_points(:,2),  '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 1);

% Kółko i trójkąt na A D
plot(A(1), A(2), 'o', 'Color', [0 0 0], 'MarkerSize', 4);
plot(D(1), D(2), 'o', 'Color', [0 0 0], 'MarkerSize', 4);
plot(A(1), A(2)-4, '^', 'Color', [0 0 0], 'MarkerSize', 10);
plot(D(1), D(2)-4, '^', 'Color', [0 0 0], 'MarkerSize', 10);

% horizontal lines of reference
yline(A(2), '--', 'Color', [.8 .8 .8])
yline(D(2), '--', 'Color', [.8 .8 .8])

idxes = [66, 34, 18];
% Punkty B C
plot(B_points(idxes([1,3]),1), B_points(idxes([1,3]),2), 'o', 'Color', [0 0 0], 'MarkerSize', 3);
plot(C_points(idxes([1,3]),1), C_points(idxes([1,3]),2), 'o', 'Color', [0 0 0], 'MarkerSize', 3);

linkages = cat(3, A_points(idxes,:), B_points(idxes,:), C_points(idxes,:), D_points(idxes,:));
linkage_max = squeeze(linkages(1,:,:));
linkage_min = squeeze(linkages(3,:,:));
linkage_mid = squeeze(linkages(2,:,:));
plot(linkage_max(1,:),linkage_max(2,:), "Color", [.66 .66 .66])
plot(linkage_min(1,:),linkage_min(2,:), "Color", [.66 .66 .66])
% plot(linkage_mid(1,:),linkage_mid(2,:))

% Text ile stopnia wychylenia na czym
idx_max = idxes(1);
idx_min = idxes(3);

text(B_points(idx_min,1), B_points(idx_min,2), ...
    sprintf("%s%.1f° ↘", '\delta_{min} = ', deltas_deg(idx_min)), ...
    'FontSize', 12, 'HorizontalAlignment','right', 'VerticalAlignment', 'bottom')
text(C_points(idx_min,1), C_points(idx_min,2), ...
    sprintf("↙ %s%.1f°", '\alpha_{min} = ', alphas_deg(idx_min)), ...
    'FontSize', 12, 'HorizontalAlignment','left', 'VerticalAlignment', 'bottom')

text(B_points(idx_max,1), B_points(idx_max,2), ...
    sprintf("%s%.1f° ↗", '\delta_{max} = ', deltas_deg(idx_max)), ...
    'FontSize', 12, 'HorizontalAlignment','right', 'VerticalAlignment', 'top')
text(C_points(idx_max,1), C_points(idx_max,2), ...
    sprintf("↖ %s%.1f°", '\alpha_{max} = ', alphas_deg(idx_max)), ...
    'FontSize', 12, 'HorizontalAlignment','left', 'VerticalAlignment', 'top')

% Strzalka kirunku napływu wody
text(-40,20, sprintf("water flow direction\n → → → → → → →"), ...
    'FontSize', 12, 'HorizontalAlignment','right', 'VerticalAlignment','middle')

axis off
subplot(1,2,1)
hold on;
axis equal;
axis off
% grid on;
% horizontal lines of reference
yline(A(2), '--', 'Color', [.8 .8 .8])
yline(D(2), '--', 'Color', [.8 .8 .8])

% Chmura po łuku
plot(B_points(:,1),  B_points(:,2),  '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 1);
% plot(C1_points(:,1), C1_points(:,2), '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 1); %Docelowo nie plotujemy tego C1 i C2
% plot(C2_points(:,1), C2_points(:,2), '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 1); %Docelowo nie plotujemy tego C1 i C2
plot(C_points(:,1),  C_points(:,2),  '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 1);

% Kółko i trójkąt na A D
plot(A(1), A(2), 'o', 'Color', [0 0 0], 'MarkerSize', 4);
plot(D(1), D(2), 'o', 'Color', [0 0 0], 'MarkerSize', 4);
plot(A(1), A(2)-4, '^', 'Color', [0 0 0], 'MarkerSize', 10);
plot(D(1), D(2)-4, '^', 'Color', [0 0 0], 'MarkerSize', 10);

% Linkage plot
plot(linkage_mid(1,:),linkage_mid(2,:), 'r')

% Punkty B C
plot(B_points(idxes(2),1), B_points(idxes(2),2), 'o', 'Color', [0 0 0], 'MarkerSize', 3);
plot(C_points(idxes(2),1), C_points(idxes(2),2), 'o', 'Color', [0 0 0], 'MarkerSize', 3);

% plot literki A B C D gdzies w poblizu mid linkage kropek
text(linkage_mid(1,1)+5, ...
     linkage_mid(2,1), ...
     '\leftarrow A', 'FontSize', 12);
text(linkage_mid(1,2)-5, ...
     linkage_mid(2,2), ...
     'B \rightarrow', 'FontSize', 12, 'HorizontalAlignment','right');
text(linkage_mid(1,3)+5, ...
     linkage_mid(2,3), ...
     '\leftarrow C', 'FontSize', 12);
text(linkage_mid(1,4)-5, ...
     linkage_mid(2,4), ...
     'D \rightarrow', 'FontSize', 12, 'HorizontalAlignment','right');

% TODO Cyferki na elementy mechanizmu na linki
i = idxes(2); % neutral position
P_AB = 0.5 * (A_points(i,:) + B_points(i,:));
P_BC = 0.5 * (B_points(i,:) + C_points(i,:));
P_CD = 0.5 * (C_points(i,:) + D_points(i,:));
text(P_AB(1), P_AB(2)-2, ...
    ['\uparrow' newline 'Ⅰ'], 'FontSize', 12, 'HorizontalAlignment','center', 'VerticalAlignment','top');
text(P_BC(1), P_BC(2), ...
    ' \leftarrow Ⅱ', 'FontSize', 12, 'HorizontalAlignment','left', 'VerticalAlignment','middle');
text(P_CD(1), P_CD(2)+2, ...
    ['Ⅲ' newline '\downarrow'], 'FontSize', 12, 'HorizontalAlignment','center', 'VerticalAlignment','bottom');
% TODO plot 1 2 3 i strzalki na elementy plotu? 
% TODO plot angles on the screen numerical values of max and min both theta
% and psi

%% Analysis

figure('Name','fig_four_bar_linearity')
hold on;
grid on;
% axis equal;
plot(deltas_deg, alphas_deg, '.')

% Dopasuj prostą do zakresu pracy
idxes_op = delta_min-1 <= deltas_deg & deltas_deg <= delta_max+1;
deltas_op = deltas_deg(idxes_op);
alphas_op = alphas_deg(idxes_op);

% fit 
p = polyfit(deltas_op, alphas_op, 1);

% sprawdzenie liniowosci w zakresie
phis_lin = polyval(p, deltas_op);
% Plot kreski taki troche szerszy żeby pokazać 
plot([-60; deltas_op; 60], polyval(p, [-60; deltas_op; 60]))

error_deg = alphas_op - phis_lin;
max_error = max(abs(error_deg));
rms_error = sqrt(mean(error_deg.^2));

% Transmission ratio variation
gain = gradient(alphas_op, deltas_op);  % d alpha / d theta_s
mean_gain = mean(gain);
gain_variation_percent = max(abs(gain - mean_gain)) / abs(mean_gain) * 100;

% Print results
fprintf('Linear fit: alpha = %.5f * delta + %.5f\n', p(1), p(2));
fprintf('Max nonlinearity error: %.4f deg\n', max_error);
fprintf('RMS nonlinearity error: %.4f deg\n', rms_error);
fprintf('Mean transmission ratio: %.4f deg/deg\n', mean_gain);
fprintf('Max transmission ratio variation: %.2f %%\n', gain_variation_percent);

xlabel('Servo horn angle \delta [deg]');
ylabel('Hydrofoil incidence angle \alpha [deg]');
xlim([-50 50]); ylim([-30 30]);

% ticki 
alpha_min = alphas_deg(idx_min);
alpha_max = alphas_deg(idx_max);
delta_min = deltas_deg(idx_min);
delta_max = deltas_deg(idx_max);
ax = gca;
ax.TickLabelInterpreter = 'latex';

x_ticks = sort([-50:10:50, delta_min, delta_max]);
y_ticks = sort([-30:10:30, alpha_min, alpha_max]);

x_labels = compose('%.0f', x_ticks);
y_labels = compose('%.0f', y_ticks);

[~, i_delta_min] = min(abs(x_ticks - delta_min));
[~, i_delta_max] = min(abs(x_ticks - delta_max));
[~, i_alpha_min] = min(abs(y_ticks - alpha_min));
[~, i_alpha_max] = min(abs(y_ticks - alpha_max));

% x_labels{i_delta_min} = "$\delta_{\min}$";
% x_labels{i_delta_max} = "$\delta_{\max}$";
% y_labels{i_alpha_min} = "$\alpha_{\min}$";
% y_labels{i_alpha_max} = "$\alpha_{\max}$";
x_labels{i_delta_min} = '';
x_labels{i_delta_max} = '';
y_labels{i_alpha_min} = '';
y_labels{i_alpha_max} = '';
% xline(delta_min, ':', 'HandleVisibility', 'off');
% xline(delta_max, ':', 'HandleVisibility', 'off');
% yline(alpha_min, ':', 'HandleVisibility', 'off');
% yline(alpha_max, ':', 'HandleVisibility', 'off');

xticks(x_ticks);
yticks(y_ticks);

xticklabels(x_labels);
yticklabels(y_labels);
xl = xlim;
yl = ylim;

% Put special labels INSIDE the plot area
x_label_y = yl(1) + 0.035 * range(yl);   % slightly above bottom axis
y_label_x = xl(1) + 0.025 * range(xl);   % slightly right of left axis

text(delta_min, x_label_y, '$\delta_{\min}$', ...
    'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', ...
    'BackgroundColor', 'w', ...
    'Margin', 1, ...
    'Clipping', 'on');

text(delta_max, x_label_y, '$\delta_{\max}$', ...
    'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', ...
    'BackgroundColor', 'w', ...
    'Margin', 1, ...
    'Clipping', 'on');

text(y_label_x, alpha_min, '$\alpha_{\min}$', ...
    'Interpreter', 'latex', ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'middle', ...
    'BackgroundColor', 'w', ...
    'Margin', 1, ...
    'Clipping', 'on');

text(y_label_x, alpha_max, '$\alpha_{\max}$', ...
    'Interpreter', 'latex', ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'middle', ...
    'BackgroundColor', 'w', ...
    'Margin', 1, ...
    'Clipping', 'on');

legend( ...
    'Four-bar kinematics', ...
    sprintf('Linear fit: \\alpha=%.3f\\delta%+.3f', p(1), p(2)), ...
    'Location', 'northwest');

%% Save figures
% PNG
fig1 = findobj('Type', 'figure', 'Name', 'fig_four_bar_mechanism');
fig2 = findobj('Type', 'figure', 'Name', 'fig_four_bar_linearity');

exportgraphics(fig1, 'fig_four_bar_mechanism.png', ...
    'Resolution', 600, ...
    'BackgroundColor', 'white');

exportgraphics(fig2, 'fig_four_bar_linearity.png', ...
    'Resolution', 600, ...
    'BackgroundColor', 'white');
return
%PDF
fig1 = findobj('Type', 'figure', 'Name', 'fig_four_bar_mechanism');
fig2 = findobj('Type', 'figure', 'Name', 'fig_four_bar_linearity');

set(fig1, 'Renderer', 'painters');
set(fig2, 'Renderer', 'painters');

exportgraphics(fig1, 'fig_four_bar_mechanism.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');

exportgraphics(fig2, 'fig_four_bar_linearity.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');

%% Plots takie bardziej dla ciekawosci
figure;
plot(theta_op, alpha_op, 'LineWidth', 1.5);
hold on;
plot(theta_op, alpha_lin, '--', 'LineWidth', 1.5);
grid on;
xlabel('Servo angle \theta_s [deg]');
ylabel('Hydrofoil incidence angle \alpha [deg]');
legend('Four-bar mechanism', 'Linear fit', 'Location', 'best');
title('Servo angle to hydrofoil incidence angle');

figure;
plot(theta_op, error_deg, 'LineWidth', 1.5);
grid on;
xlabel('Servo angle \theta_s [deg]');
ylabel('Nonlinearity error [deg]');
title('Deviation from linear approximation');

figure;
plot(alpha_op, gain, 'LineWidth', 1.5);
grid on;
xlabel('Hydrofoil incidence angle \alpha [deg]');
ylabel('Transmission ratio d\alpha/d\theta_s [deg/deg]');
title('Transmission ratio variation');

%% Helper function
function [P1, P2, valid] = circleIntersection(C0, r0, C1, r1)
    d = norm(C1 - C0);

    valid = true;
    P1 = [NaN, NaN];
    P2 = [NaN, NaN];

    if d > r0 + r1 || d < abs(r0 - r1) || d == 0
        valid = false;
        return;
    end

    a = (r0^2 - r1^2 + d^2) / (2*d);
    h_sq = r0^2 - a^2;

    if h_sq < 0
        valid = false;
        return;
    end

    h = sqrt(h_sq);

    ex = (C1 - C0) / d;
    ey = [-ex(2), ex(1)];

    P = C0 + a * ex;

    P1 = P + h * ey;
    P2 = P - h * ey;
end
