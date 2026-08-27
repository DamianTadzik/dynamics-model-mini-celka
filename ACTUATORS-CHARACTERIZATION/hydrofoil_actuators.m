clc; clear;
%% Generate trajectory to be played by the python controller
generate_trajectory = false;
if generate_trajectory
    % Frequency 100hz so 100 samples per second
    % Max 12deg min -6deg so this has to be generated within those bounds
    
    % Firstly define few points that you want to cover, ZOH will be used to
    % extrapolate in beetwen those defined points
    REPS = 2; % or 20 for 20min
    vals = [0  repmat([-6  12], 1, REPS) 0 0];
    % vals = [vals repmat([-4 10],1, REPS) 0 0];
    vals = [vals repmat([-2 8],1, REPS) 0 0];
    % vals = [vals repmat([0 6],1, REPS) 0 0];
    vals = [vals repmat([2 4],1, REPS) 0 0];
    
    time = 0:numel(vals)-1;
    
    controller_time = time(1):0.01:time(end);
    controller_vals_sqr = interp1(time, vals, controller_time, 'previous');
    controller_vals_saw = interp1(time, vals, controller_time, 'linear');
    controller_vals_sin = interp1(time, vals, controller_time, 'pchip');

    % % controller_time = [controller_time  controller_time+controller_time(end)  controller_time+2*controller_time(end)];
    controller_time = [controller_time controller_time controller_time];
    controller_vals = [controller_vals_sqr controller_vals_saw controller_vals_sin];
    
    f = figure('Name','actutator_test_trajectory','Units','inches','Position',[2 2 9 3]);
    plot(time, vals, '*'); hold on; grid on;
    plot(controller_time, controller_vals, '.');
    xlabel('Time [s]', 'Interpreter','latex','FontName','Times New Roman');
    ylabel('Requested angle [$^\circ$]', 'Interpreter','latex','FontName','Times New Roman');
    set(gca,'FontName','Times New Roman');
    xlim([controller_time(1) controller_time(end)]);
    % exportgraphics(f, 'actutator_test_trajectory.pdf', ...
    %     'ContentType', 'vector', ...
    %     'BackgroundColor', 'none');
    return
    
    % Triplicate that trajectory for each of the three hydrofoils, one after
    % another.
    N = length(controller_vals);
    % controller_time_all = (0:(3*N-1))*0.01;
    % controller_vals_left  = [controller_vals zeros(1,N) zeros(1,N)];
    % controller_vals_right = [zeros(1,N) controller_vals zeros(1,N)];
    % controller_vals_rear  = [zeros(1,N) zeros(1,N) controller_vals];

    %Aleternative where they act together
    controller_time_all = (0:(N-1))*0.01;
    controller_vals_left  = [controller_vals];
    controller_vals_right = [controller_vals];
    controller_vals_rear  = [controller_vals];

    figure;
    ax = [subplot(3,1,1)];
    plot(controller_time_all, controller_vals_left, '.');
    ax = [ax subplot(3,1,2)];
    plot(controller_time_all, controller_vals_right, '.');
    ax = [ax subplot(3,1,3)];
    plot(controller_time_all, controller_vals_rear, '.');
    linkaxes(ax,'x');

    return
    % Save
    trajectory = [
        controller_vals_left;
        controller_vals_right;
        controller_vals_rear
    ];
    save("hydrofoil_actuators_trajectory_sqr_saq_sin_5REPS_ALL_TEST.mat", "trajectory");
end
%% Logs overwiev
clc
path = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_08_26_home\logs_parquet\";
% path = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_04_26_divonnes\logs_parquet\";
paths = path + ["log11" "log14" "log16"] + ".parquet";
for k = 1:numel(paths)
    p = paths(k);
    info = parquetinfo(p);
    T = parquetread(p);

    % for i = 1:length(T.Properties.VariableDescriptions)
    %     disp(T.Properties.VariableDescriptions{i});
    % end
    for i = 1:numel(T.Properties.VariableNames)
        disp(T.Properties.VariableNames{i});
    end
    % % CATCH THE ERRORNEOUS TYPE
    % for i = 1:numel(info.VariableNames)
    %     name = info.VariableNames(i);
    % 
    %     try
    %         parquetread(p, SelectedVariableNames=name, ...
    %                        VariableNamingRule="preserve");
    %         fprintf("OK   %s\n", name);
    %     catch ME
    %         fprintf("FAIL %s\n", name);
    %         fprintf("     %s\n", ME.message);
    %     end
    % end

    [~, filename, ~] = fileparts(paths(k));
    figure(Name=filename, WindowStyle="normal")
    tl = tiledlayout(3,1, "TileSpacing","compact", "Padding","compact");
    ax(1) = nexttile;
    plot_T(T, "can_signals_AUTO_CONTROL_FRONT_LEFT_SETPOINT")
    plot_T(T, "can_signals_AUTO_CONTROL_FRONT_RIGHT_SETPOINT")
    plot_T(T, "can_signals_AUTO_CONTROL_REAR_SETPOINT")
    ax(2) = nexttile;
    plot_T(T, "can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US")
    plot_T(T, "can_signals_ACTUATOR_RIGHT_FOIL_FEEDBACK_SETPOINT_US")
    plot_T(T, "can_signals_ACTUATOR_REAR_FOIL_FEEDBACK_SETPOINT_US")
    ax(3) = nexttile;
    plot_T(T, "can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW")
    plot_T(T, "can_signals_ACTUATOR_RIGHT_FOIL_FEEDBACK_POSITION_RAW")
    plot_T(T, "can_signals_ACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW")
    linkaxes(ax,'x');
    ax(1).XTickLabel = [];
    ax(2).XTickLabel = [];
    % legend('Location','best');
end

%% Time to come up with analyzer based on first short log
disp("GO TO %% Use above analyzer enclosed in a function over real data")
return
k = 1;
p = paths(k);
info = parquetinfo(p);
T = parquetread(p);

% Potnijmy przebieg dla FRONT LEFT
% c - control input from controller
% u - input for actuator
% y - actuator response

t = T.timestamp_s - T.timestamp_s(1);

c = T.can_signals_AUTO_CONTROL_FRONT_LEFT_SETPOINT;
c_idx = ~isnan(c) & ~isnan(t);
t_c = t(c_idx);
c = c(c_idx);

u = T.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US;
y = T.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW;
uy_idx = ~isnan(u) & ~isnan(y) & ~isnan(t);
t_uy = t(uy_idx);
u = u(uy_idx);
y = y(uy_idx);

% Cut identification data into N fragments, each 1 s long  starting at tnstart
N = 5;
tnstart = 5.0;
clear segments
segments(N) = struct();
for k = 1:N
    t0 = tnstart + (k-1) * 1.0;
    t1 = t0 + 1.0;

    % c - osobny timestamp
    idx_c = t_c >= t0 & t_c < t1;
    segments(k).command.t = t_c(idx_c);
    segments(k).command.c = c(idx_c);
     
    % u/y - wspólny timestamp
    idx_uy = t_uy >= t0 & t_uy < t1;
    segments(k).plant.t = t_uy(idx_uy);
    segments(k).plant.u = u(idx_uy);
    segments(k).plant.y = y(idx_uy);
end
% Cutted fragments should be suitable for identification

% Zmierzyc delay od AUTO_CONTROL frame do FEEDBACK_SETPOINT_US
for k = 1:numel(segments)
    % Find largest step
    [~, ic] = max(abs(diff(segments(k).command.c)));
    [~, iu] = max(abs(diff(segments(k).plant.u)));

    % Timestamp of the sample after the step
    t_step_c = segments(k).command.t(ic + 1);
    t_step_u = segments(k).plant.t(iu + 1);

    % Delay c -> u
    segments(k).delay = t_step_u - t_step_c;
end

% Dynamike zmierzyc od FEEDBACK_SETPOINT_US do FEEDBACK_POSITION_RAW
for k = 1:numel(segments)

    t = segments(k).plant.t;
    u = segments(k).plant.u;
    y = segments(k).plant.y;

    % Sampling time
    Ts = median(diff(t))
    % Identification data
    data = iddata(y-y(1), u-u(1), Ts);

    % Estimate P1 and P1D
    opts = procestOptions;
    opts.EstimateCovariance = true;
    opts.InputInterSample = 'zoh';
    opts.Focus = 'simulation';

    mdl_P1  = procest(data, 'P1', opts);
    mdl_P1D = procest(data, 'P1D', opts);

    % Store models
    segments(k).dynamics.P1.model  = mdl_P1;
    segments(k).dynamics.P1D.model = mdl_P1D;

    % ---------------------------------------------------------
    % P1
    % G(s) = Kp / (1 + Tp1*s)
    % ---------------------------------------------------------
    p = getpvec(mdl_P1, 'free');
    C = getcov(mdl_P1, 'value', 'free');
    sd = sqrt(diag(C));

    segments(k).dynamics.P1.Kp  = mdl_P1.Kp;
    segments(k).dynamics.P1.Tp1 = mdl_P1.Tp1;

    % 95% confidence intervals
    ci95 = [p - 1.96*sd, p + 1.96*sd];

    segments(k).dynamics.P1.parameters = p;
    segments(k).dynamics.P1.std        = sd;
    segments(k).dynamics.P1.ci95       = ci95;

    % Fit statistics
    segments(k).dynamics.P1.fit  = mdl_P1.Report.Fit.FitPercent;
    segments(k).dynamics.P1.AICc = mdl_P1.Report.Fit.AICc;
    segments(k).dynamics.P1.BIC  = mdl_P1.Report.Fit.BIC;

    % ---------------------------------------------------------
    % P1D
    % G(s) = Kp / (1 + Tp1*s) * exp(-Td*s)
    % ---------------------------------------------------------
    p = getpvec(mdl_P1D, 'free');
    C = getcov(mdl_P1D, 'value', 'free');
    sd = sqrt(diag(C));

    segments(k).dynamics.P1D.Kp  = mdl_P1D.Kp;
    segments(k).dynamics.P1D.Tp1 = mdl_P1D.Tp1;
    segments(k).dynamics.P1D.Td  = mdl_P1D.Td;

    % 95% confidence intervals
    ci95 = [p - 1.96*sd, p + 1.96*sd];

    segments(k).dynamics.P1D.parameters = p;
    segments(k).dynamics.P1D.std        = sd;
    segments(k).dynamics.P1D.ci95       = ci95;

    % Fit statistics
    segments(k).dynamics.P1D.fit  = mdl_P1D.Report.Fit.FitPercent;
    segments(k).dynamics.P1D.AICc = mdl_P1D.Report.Fit.AICc;
    segments(k).dynamics.P1D.BIC  = mdl_P1D.Report.Fit.BIC;

    % PLOT
    mdl_P1.Name  = 'P1';
    mdl_P1D.Name = 'P1D';
    figure;
    compare(data, mdl_P1, mdl_P1D);
    title(sprintf('Segment %d', k));
end


return
%% Use above analyzer enclosed in a function over real data
k = 3; N = 39; %third dataset 
% k=1; N=5; % first dataset k=1 has 3 periods, that gives N=5 edges to look at

p = paths(k);
info = parquetinfo(p);
T = parquetread(p);

% % biggest amplitude
% TSTART = 2.1;%third dataset 
% % TSTART=4.8; % first dataset k=1
% analyzed_biggest_segments_LEFT = analyze_square(T, 'LEFT', N, TSTART);
% % mid amplitude
% analyzed_mid_segments_LEFT = analyze_square(T, 'LEFT', N, TSTART+1*(N*1.0+3.0));
% % lowest amplitdue
% analyzed_lowest_segments_LEFT = analyze_square(T, 'LEFT', N, TSTART+2*(N*1.0+3.0));

TSTART = 380.1;%third dataset 
% TSTART=76.8; % first dataset k=1
analyzed_biggest_segments_RIGHT = analyze_square(T, 'RIGHT', N, TSTART);
% mid amplitude
analyzed_mid_segments_RIGHT = analyze_square(T, 'RIGHT', N, TSTART+1*(N*1.0+3.0));
% lowest amplitdue
analyzed_lowest_segments_RIGHT = analyze_square(T, 'RIGHT', N, TSTART+2*(N*1.0+3.0));

TSTART = 758.1;%third dataset 
% TSTART=148.8; % first dataset k=1
% TSTART=4.8;
analyzed_biggest_segments_REAR = analyze_square(T, 'REAR', N, TSTART);
% mid amplitude
analyzed_mid_segments_REAR = analyze_square(T, 'REAR', N, TSTART+1*(N*1.0+3.0));
% lowest amplitdue
analyzed_lowest_segments_REAR = analyze_square(T, 'REAR', N, TSTART+2*(N*1.0+3.0));

% datasets = {
%     analyzed_biggest_segments, 'Big amplitude';
%     analyzed_mid_segments,     'Mid amplitude';
%     analyzed_lowest_segments,  'Low amplitude'
% };

datasets = {
    analyzed_biggest_segments_LEFT,  analyzed_biggest_segments_RIGHT,  analyzed_biggest_segments_REAR,  'Big';
    analyzed_mid_segments_LEFT,      analyzed_mid_segments_RIGHT,      analyzed_mid_segments_REAR,      'Mid';
    analyzed_lowest_segments_LEFT,   analyzed_lowest_segments_RIGHT,   analyzed_lowest_segments_REAR,   'Low'
};

%% Analyze identified P1 and P1D parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

servo_names = {'LEFT','RIGHT','REAR'};

for a = 1:size(datasets,1)

    amplitude_name = datasets{a,4};

    for s = 1:3

        S = datasets{a,s};
        servo_name = servo_names{s};

        is_up = arrayfun(@(x) x.plant.u(end) > x.plant.u(1), S);

        directions = {
            S(is_up),  'UP';
            S(~is_up), 'DOWN'
        };

        for d = 1:size(directions,1)

            Sdir = directions{d,1};
            direction_name = directions{d,2};

            Kp_P1  = arrayfun(@(x) x.dynamics.P1.Kp, Sdir);
            Tp_P1  = arrayfun(@(x) x.dynamics.P1.Tp1, Sdir);
            fit_P1 = arrayfun(@(x) x.dynamics.P1.fit, Sdir);

            Kp_P1D  = arrayfun(@(x) x.dynamics.P1D.Kp, Sdir);
            Tp_P1D  = arrayfun(@(x) x.dynamics.P1D.Tp1, Sdir);
            Td_P1D  = arrayfun(@(x) x.dynamics.P1D.Td, Sdir);
            fit_P1D = arrayfun(@(x) x.dynamics.P1D.fit, Sdir);

            plot_name = sprintf('%s - %s - %s', servo_name, amplitude_name, direction_name);

            figure(Name=plot_name);
            tl = tiledlayout(4,1);
            title(tl, plot_name);

            cP1  = [0 0.4470 0.7410];
            cP1D = [0.8500 0.3250 0.0980];

            nexttile;
            plot(Kp_P1,'o-','Color',cP1); hold on;
            plot(Kp_P1D,'o-','Color',cP1D);
            grid on; ylabel('K_p'); legend('P1','P1D');

            nexttile;
            plot(Tp_P1,'o-','Color',cP1); hold on;
            plot(Tp_P1D,'o-','Color',cP1D);
            grid on; ylabel('T_p [s]');

            nexttile;
            plot(Td_P1D,'o-','Color',cP1D);
            grid on; ylabel('T_d [s]');

            nexttile;
            plot(fit_P1,'o-','Color',cP1); hold on;
            plot(fit_P1D,'o-','Color',cP1D);
            grid on; ylabel('Fit [%]'); xlabel('Step');

            fprintf('\n%s - %s - %s\n', servo_name, amplitude_name, direction_name);

            fprintf('P1:  Kp = %.4f +- %.4f, Tp = %.4f +- %.4f s, fit = %.2f +- %.2f %%\n', ...
                mean(Kp_P1), std(Kp_P1), mean(Tp_P1), std(Tp_P1), mean(fit_P1), std(fit_P1));

            fprintf('P1D: Kp = %.4f +- %.4f, Tp = %.4f +- %.4f s, Td = %.4f +- %.4f s, fit = %.2f +- %.2f %%\n', ...
                mean(Kp_P1D), std(Kp_P1D), mean(Tp_P1D), std(Tp_P1D), ...
                mean(Td_P1D), std(Td_P1D), mean(fit_P1D), std(fit_P1D));
        end
    end
end


%%% Actually useful analysis.. 
%% Compare servos - selected parameter

servo_names = {'LEFT','RIGHT','REAR'};
direction_names = {'UP','DOWN'};

plot_param(datasets, servo_names, direction_names, ...
    @(x) x.dynamics.P1D.Kp, 'P1D gain comparison', 'K_p');

plot_param(datasets, servo_names, direction_names, ...
    @(x) x.dynamics.P1D.Tp1, 'P1D time constant comparison', 'T_{p1} [s]');

plot_param(datasets, servo_names, direction_names, ...
    @(x) x.dynamics.P1D.Td, 'P1D delay comparison', 'T_d [s]');

plot_param(datasets, servo_names, direction_names, ...
    @(x) x.dynamics.P1D.fit, 'P1D fit comparison', 'Fit [%]');

plot_param(datasets, servo_names, direction_names, ...
    @(x) x.dynamics.P1.Kp, 'P1 gain comparison', 'K_p');

plot_param(datasets, servo_names, direction_names, ...
    @(x) x.dynamics.P1.Tp1, 'P1 time constant comparison', 'T_{p1} [s]');

plot_param(datasets, servo_names, direction_names, ...
    @(x) x.dynamics.P1.fit, 'P1 fit comparison', 'Fit [%]');

%% Helper functions
function plot_T(T, keyname)
    x = T.timestamp_s - T.timestamp_s(1);
    y = T.(keyname);

    mask = ~isnan(x) & ~isnan(y);

    plot(x(mask), y(mask), '.', "DisplayName", keyname);
    hold on
end

function segments = analyze_square(T, ACTUATOR, N, tnstart)

    % Potnijmy przebieg
    % c - control input from controller
    % u - input for actuator
    % y - actuator response
    t = T.timestamp_s - T.timestamp_s(1);
    
    switch(ACTUATOR)
        case 'LEFT'
            c = T.can_signals_AUTO_CONTROL_FRONT_LEFT_SETPOINT;
            u = T.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US;
            y = T.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW;

        case 'RIGHT'
            c = T.can_signals_AUTO_CONTROL_FRONT_RIGHT_SETPOINT;
            u = T.can_signals_ACTUATOR_RIGHT_FOIL_FEEDBACK_SETPOINT_US;
            y = T.can_signals_ACTUATOR_RIGHT_FOIL_FEEDBACK_POSITION_RAW;

        case 'REAR'
            c = T.can_signals_AUTO_CONTROL_REAR_SETPOINT;
            u = T.can_signals_ACTUATOR_REAR_FOIL_FEEDBACK_SETPOINT_US;
            y = T.can_signals_ACTUATOR_REAR_FOIL_FEEDBACK_POSITION_RAW;
    end

    c_idx = ~isnan(c) & ~isnan(t);
    t_c = t(c_idx);
    c = c(c_idx);

    uy_idx = ~isnan(u) & ~isnan(y) & ~isnan(t);
    t_uy = t(uy_idx);
    u = u(uy_idx);
    y = y(uy_idx);
    
    % Cut identification data into N fragments, each 1 s long  starting at tnstart
    % N = 5;
    % tnstart = 5.0;
    % clear segments
    segments(N) = struct();
    for k = 1:N
        t0 = tnstart + (k-1) * 1.0;
        t1 = t0 + 1.0;
    
        % c - osobny timestamp
        idx_c = t_c >= t0 & t_c < t1;
        segments(k).command.t = t_c(idx_c);
        segments(k).command.c = c(idx_c);
         
        % u/y - wspólny timestamp
        idx_uy = t_uy >= t0 & t_uy < t1;
        segments(k).plant.t = t_uy(idx_uy);
        segments(k).plant.u = u(idx_uy);
        segments(k).plant.y = y(idx_uy);
    end
    % Cutted fragments should be suitable for identification
    
    % Zmierzyc delay od AUTO_CONTROL frame do FEEDBACK_SETPOINT_US
    for k = 1:numel(segments)
        % Find largest step
        [~, ic] = max(abs(diff(segments(k).command.c)));
        [~, iu] = max(abs(diff(segments(k).plant.u)));
    
        % Timestamp of the sample after the step
        t_step_c = segments(k).command.t(ic + 1);
        t_step_u = segments(k).plant.t(iu + 1);
    
        % Delay c -> u
        segments(k).delay = t_step_u - t_step_c;
    end
    
    % Dynamike zmierzyc od FEEDBACK_SETPOINT_US do FEEDBACK_POSITION_RAW
    for k = 1:numel(segments)
    
        t = segments(k).plant.t;
        u = segments(k).plant.u;
        y = segments(k).plant.y;
    
        % Sampling time
        Ts = median(diff(t));
        % Identification data
        idx0 = (t - t(1)) <= 0.4;
        y0 = mean(y(idx0));
        u0 = mean(u(idx0));
        data = iddata(y - y0, u - u0, Ts);
        % data = iddata(y-y(1), u-u(1), Ts);
    
        % Estimate P1 and P1D
        opts = procestOptions;
        opts.EstimateCovariance = true;
        opts.InputInterSample = 'zoh';
        opts.InitialCondition = 'zero';
        opts.Focus = 'simulation';
    
        mdl_P1  = procest(data, 'P1', opts);
        mdl_P1D = procest(data, 'P1D', opts);
    
        % Store models
        segments(k).dynamics.P1.model  = mdl_P1;
        segments(k).dynamics.P1D.model = mdl_P1D;
    
        % ---------------------------------------------------------
        % P1
        % G(s) = Kp / (1 + Tp1*s)
        % ---------------------------------------------------------
        p = getpvec(mdl_P1, 'free');
        C = getcov(mdl_P1, 'value', 'free');
        sd = sqrt(diag(C));
    
        segments(k).dynamics.P1.Kp  = mdl_P1.Kp;
        segments(k).dynamics.P1.Tp1 = mdl_P1.Tp1;
    
        % 95% confidence intervals
        ci95 = [p - 1.96*sd, p + 1.96*sd];
    
        segments(k).dynamics.P1.parameters = p;
        segments(k).dynamics.P1.std        = sd;
        segments(k).dynamics.P1.ci95       = ci95;
    
        % Fit statistics
        segments(k).dynamics.P1.fit  = mdl_P1.Report.Fit.FitPercent;
        segments(k).dynamics.P1.AICc = mdl_P1.Report.Fit.AICc;
        segments(k).dynamics.P1.BIC  = mdl_P1.Report.Fit.BIC;
    
        % ---------------------------------------------------------
        % P1D
        % G(s) = Kp / (1 + Tp1*s) * exp(-Td*s)
        % ---------------------------------------------------------
        p = getpvec(mdl_P1D, 'free');
        C = getcov(mdl_P1D, 'value', 'free');
        sd = sqrt(diag(C));
    
        segments(k).dynamics.P1D.Kp  = mdl_P1D.Kp;
        segments(k).dynamics.P1D.Tp1 = mdl_P1D.Tp1;
        segments(k).dynamics.P1D.Td  = mdl_P1D.Td;
    
        % 95% confidence intervals
        ci95 = [p - 1.96*sd, p + 1.96*sd];
    
        segments(k).dynamics.P1D.parameters = p;
        segments(k).dynamics.P1D.std        = sd;
        segments(k).dynamics.P1D.ci95       = ci95;
    
        % Fit statistics
        segments(k).dynamics.P1D.fit  = mdl_P1D.Report.Fit.FitPercent;
        segments(k).dynamics.P1D.AICc = mdl_P1D.Report.Fit.AICc;
        segments(k).dynamics.P1D.BIC  = mdl_P1D.Report.Fit.BIC;
    
        % % % PLOT
        % % mdl_P1.Name  = 'P1';
        % % mdl_P1D.Name = 'P1D';
        % % figure;
        % % compare(data, mdl_P1, mdl_P1D, compareOptions('InitialCondition','z'));
        % % title(sprintf('Segment %d', k));
    end
end

















function plot_param(datasets, servo_names, direction_names, extractor, figure_title, ylabel_txt)

    figure;
    tl = tiledlayout(3,2);

    for a = 1:3
        amplitude_name = datasets{a,4};

        for d = 1:2
            nexttile;
            hold on; grid on;

            all_vals = [];
            all_groups = [];

            for s = 1:3
                S = datasets{a,s};
                is_up = arrayfun(@(x) x.plant.u(end) > x.plant.u(1), S);

                if d == 1
                    Sd = S(is_up);
                else
                    Sd = S(~is_up);
                end

                vals = arrayfun(extractor, Sd);

                all_vals = [all_vals vals];
                all_groups = [all_groups s*ones(size(vals))];

                scatter(s*ones(size(vals)), vals, 40, 'filled');
            end

            boxchart(all_groups', all_vals');

            xticks(1:3);
            xticklabels(servo_names);
            ylabel(ylabel_txt);
            title(sprintf('%s - %s', amplitude_name, direction_names{d}));
        end
    end

    title(tl, figure_title);
end



return
%% DELAYS ODPAL TO NA KONCU DOPIERO.. 
all_delays = [];

for a = 1:3
    for s = 1:3
        S = datasets{a,s};
        all_delays = [all_delays, [S.delay]];
    end
end

all_delays_ms = all_delays * 1000;

figure;
histogram(all_delays_ms);
grid on;
xlabel('Delay c \rightarrow u [ms]');
ylabel('Count');
title('Controller \rightarrow actuator delay');

fprintf('N = %d\n', numel(all_delays_ms));
fprintf('Mean   = %.2f ms\n', mean(all_delays_ms));
fprintf('Std    = %.2f ms\n', std(all_delays_ms));
fprintf('Median = %.2f ms\n', median(all_delays_ms));
fprintf('Min    = %.2f ms\n', min(all_delays_ms));
fprintf('Max    = %.2f ms\n', max(all_delays_ms));

%
all_Td = [];

for a = 1:3
    for s = 1:3
        S = datasets{a,s};
        all_Td = [all_Td, arrayfun(@(x) x.dynamics.P1D.Td, S)];
    end
end

all_Td_ms = all_Td * 1000;

figure;
histogram(all_Td_ms);
grid on;
xlabel('P1D transport delay T_d [ms]');
ylabel('Count');
title('P1D transport delay');

fprintf('P1D Td:\n');
fprintf('N      = %d\n', numel(all_Td_ms));
fprintf('Mean   = %.2f ms\n', mean(all_Td_ms));
fprintf('Std    = %.2f ms\n', std(all_Td_ms));
fprintf('Median = %.2f ms\n', median(all_Td_ms));
fprintf('Min    = %.2f ms\n', min(all_Td_ms));
fprintf('Max    = %.2f ms\n', max(all_Td_ms));

%
all_total_delay = [];

for a = 1:3
    for s = 1:3
        S = datasets{a,s};

        delay_cu = [S.delay];
        delay_p1d = arrayfun(@(x) x.dynamics.P1D.Td, S);

        total_delay = delay_cu + delay_p1d;

        all_total_delay = [all_total_delay, total_delay];
    end
end

all_total_delay_ms = all_total_delay * 1000;

figure;
histogram(all_total_delay_ms);
grid on;
xlabel('Total delay: c \rightarrow u + P1D T_d [ms]');
ylabel('Count');
title('Total controller \rightarrow plant delay');

fprintf('Total delay:\n');
fprintf('N      = %d\n', numel(all_total_delay_ms));
fprintf('Mean   = %.2f ms\n', mean(all_total_delay_ms));
fprintf('Std    = %.2f ms\n', std(all_total_delay_ms));
fprintf('Median = %.2f ms\n', median(all_total_delay_ms));
fprintf('Min    = %.2f ms\n', min(all_total_delay_ms));
fprintf('Max    = %.2f ms\n', max(all_total_delay_ms));


all_cu = [];
all_Td = [];

for a = 1:3
    for s = 1:3
        S = datasets{a,s};

        all_cu = [all_cu, [S.delay]];
        all_Td = [all_Td, arrayfun(@(x) x.dynamics.P1D.Td, S)];
    end
end

all_cu = all_cu * 1000;
all_Td = all_Td * 1000;
all_total = all_cu + all_Td;

figure;
tiledlayout(3,1, 'TileSpacing','compact');

nexttile;
histogram(all_cu);
grid on;
xlabel('c \rightarrow u [ms]');
ylabel('Count');
title('Communication / controller delay');

nexttile;
histogram(all_Td);
grid on;
xlabel('P1D T_d [ms]');
ylabel('Count');
title('Identified plant transport delay');

nexttile;
histogram(all_total);
grid on;
xlabel('Total delay [ms]');
ylabel('Count');
title('c \rightarrow u + P1D T_d');