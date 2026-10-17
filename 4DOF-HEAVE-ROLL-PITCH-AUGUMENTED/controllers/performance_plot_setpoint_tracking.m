%% Save simout out ot .mat
% save("controllers/performance_setpoint_tracking.mat", "out")
load("controllers\performance_setpoint_tracking.mat")

%% Load data from simout

e = out.estimated;
r = struct();
% Time
r.time = e.time(:);
% x_hat:
% [z z_dot phi theta psi p q r]
r.xhat = squeeze(e.signals.values(2:9, :, :)).';

s = out.state;
% x:
% [z z_dot phi theta psi p q r]
r.x = squeeze(s.signals.values(3:10, :, :)).';


%% Plot the roll

f = figure('Name','roll_tracking_performance','Units','centimeters','Position',[2 2 8 6]);
hold on; grid on;
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
plot(r.time, rad2deg(r.xhat(:,3)), 'LineWidth', 1.5);
plot(r.time, rad2deg(r.x(:,3)), 'LineWidth', 1.5);
plot([0 40 40 45 45 50 50 60], [0 0 5 5 -5 -5 0 0], '--', 'LineWidth', 1.5);
xlabel('Time [s]','FontName','Times New Roman','FontSize',9);
ylabel('Roll angle [{\circ}]','FontName','Times New Roman','FontSize',9);
xlim([37 53])
legend('Estimated', 'True', 'Reference', ...
    'Location', 'northeast', ...
    'FontSize', 8, ...
    'Box', 'off');

exportgraphics(f, 'roll_tracking_performance.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 6);


%% Plot the heave

ff = figure('Name','heave_tracking_performance','Units','centimeters','Position',[2 2 8 6]);
hold on; grid on;
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
plot(r.time, r.xhat(:,1), 'LineWidth', 1.5);
plot(r.time, r.x(:,1), 'LineWidth', 1.5);
plot([0 20 20 25 25 30 30 40], [-.1 -.1 -.12 -.12 -.08 -.08 -.1 -.1], '--', 'LineWidth', 1.5);
xlabel('Time [s]','FontName','Times New Roman','FontSize',9);
ylabel('Heave position [m]','FontName','Times New Roman','FontSize',9);
xlim([17 33])
legend('Estimated', 'True', 'Reference', ...
    'Location', 'southeast', ...
    'FontSize', 8, ...
    'Box', 'off');
exportgraphics(ff, 'heave_tracking_performance.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 6);
