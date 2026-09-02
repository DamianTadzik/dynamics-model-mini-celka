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

% %% biggest amplitude
TSTART = 2.1;%third dataset 
% TSTART=4.8; % first dataset k=1
analyzed_biggest_segments_LEFT = analyze_square(T, 'LEFT', N, TSTART);
% %% mid amplitude
analyzed_mid_segments_LEFT = analyze_square(T, 'LEFT', N, TSTART+1*(N*1.0+3.0));
% %% lowest amplitdue
analyzed_lowest_segments_LEFT = analyze_square(T, 'LEFT', N, TSTART+2*(N*1.0+3.0));
% %%

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
    
        % ---------------------------------------------------------
        % Initial operating point
        % First 0.4 s; step occurs around 0.5 s
        % ---------------------------------------------------------
        t_rel = t - t(1);
    
        idx0 = t_rel <= 0.4;
    
        y0 = mean(y(idx0));
        u0 = mean(u(idx0));
    
        % Identification data - useful later
        data = iddata(y - y0, u - u0, Ts);
    
    
        % ---------------------------------------------------------
        % Detect input step u
        % ---------------------------------------------------------
        [~, iu] = max(abs(diff(u)));
    
        % First sample after step
        iu = iu + 1;
        t_step_u = t(iu);
    
    
        % ---------------------------------------------------------
        % Estimate final output value
        % Last 20% of segment
        % ---------------------------------------------------------
        idx_end = t_rel >= 0.8 * t_rel(end);
    
        y_final = mean(y(idx_end));
    
        delta_y = y_final - y0;
    
    
        % ---------------------------------------------------------
        % Normalize response
        %
        % Works for BOTH directions:
        %
        % UP:   y increases -> r: 0 -> 1
        % DOWN: y decreases -> r: 0 -> 1
        % ---------------------------------------------------------
        r = (y - y0) / delta_y;
    
    
        % Only search after input step
        idx_after = find(t >= t_step_u);
    
        t_after = t(idx_after);
        r_after = r(idx_after);
    
    
        % ---------------------------------------------------------
        % Exact P1 response levels
        % ---------------------------------------------------------
        p63 = 1 - exp(-1);     % 0.632120...
        p86 = 1 - exp(-2);     % 0.864665...
        p95 = 1 - exp(-3);     % 0.950213...
    
    
        % =========================================================
        % Find t63
        % =========================================================
        i63 = find(r_after >= p63, 1, 'first');
    
        if ~isempty(i63)
    
            if i63 > 1
                % Linear interpolation between samples
                t1 = t_after(i63-1);
                t2 = t_after(i63);
    
                r1 = r_after(i63-1);
                r2 = r_after(i63);
    
                t63 = t1 + ...
                    (p63-r1)/(r2-r1) * (t2-t1);
            else
                t63 = t_after(i63);
            end
    
        else
            t63 = NaN;
        end
    
    
        % =========================================================
        % Find t86
        % =========================================================
        i86 = find(r_after >= p86, 1, 'first');
    
        if ~isempty(i86)
    
            if i86 > 1
                t1 = t_after(i86-1);
                t2 = t_after(i86);
    
                r1 = r_after(i86-1);
                r2 = r_after(i86);
    
                t86 = t1 + ...
                    (p86-r1)/(r2-r1) * (t2-t1);
            else
                t86 = t_after(i86);
            end
    
        else
            t86 = NaN;
        end
    
    
        % =========================================================
        % Find t95
        % =========================================================
        i95 = find(r_after >= p95, 1, 'first');
    
        if ~isempty(i95)
    
            if i95 > 1
                t1 = t_after(i95-1);
                t2 = t_after(i95);
    
                r1 = r_after(i95-1);
                r2 = r_after(i95);
    
                t95 = t1 + ...
                    (p95-r1)/(r2-r1) * (t2-t1);
            else
                t95 = t_after(i95);
            end
    
        else
            t95 = NaN;
        end
    
    
        % =========================================================
        % Estimate Tp = tau
        % =========================================================
        if ~isnan(t63) && ~isnan(t86) && ~isnan(t95)
    
            % 63 -> 86 corresponds to one tau
            tau_63_86 = t86 - t63;
    
            % 86 -> 95 corresponds to one tau
            tau_86_95 = t95 - t86;
    
            % 63 -> 95 corresponds to two tau
            tau_63_95 = (t95 - t63) / 2;
    
            % Final tau estimate
            tau_values = [
                tau_63_86
                tau_86_95
                tau_63_95
            ];
    
            Tp_manual = mean(tau_values);
    
    
            % =====================================================
            % Extrapolate all three points back to beginning
            % of P1 dynamics
            % =====================================================
    
            t_start_from_63 = t63 - 1*Tp_manual;
            t_start_from_86 = t86 - 2*Tp_manual;
            t_start_from_95 = t95 - 3*Tp_manual;
    
            t_start_values = [
                t_start_from_63
                t_start_from_86
                t_start_from_95
            ];
    
            % Estimated beginning of P1 response
            t_dynamic_start = mean(t_start_values);
    
            % Transport delay u -> y
            Td_manual = t_dynamic_start - t_step_u;
    
        else
    
            tau_63_86 = NaN;
            tau_86_95 = NaN;
            tau_63_95 = NaN;
    
            Tp_manual = NaN;
    
            t_start_from_63 = NaN;
            t_start_from_86 = NaN;
            t_start_from_95 = NaN;
    
            t_dynamic_start = NaN;
            Td_manual = NaN;
    
        end
    
    
        % =========================================================
        % Store results
        % =========================================================
    
        segments(k).manual_P1D.Tp = Tp_manual;
        segments(k).manual_P1D.Td = Td_manual;
    
        % Individual tau estimates
        segments(k).manual_P1D.tau_63_86 = tau_63_86;
        segments(k).manual_P1D.tau_86_95 = tau_86_95;
        segments(k).manual_P1D.tau_63_95 = tau_63_95;
    
        % Crossing times
        segments(k).manual_P1D.t63 = t63;
        segments(k).manual_P1D.t86 = t86;
        segments(k).manual_P1D.t95 = t95;
    
        % Extrapolated dynamic start
        segments(k).manual_P1D.t_start_from_63 = ...
            t_start_from_63;
    
        segments(k).manual_P1D.t_start_from_86 = ...
            t_start_from_86;
    
        segments(k).manual_P1D.t_start_from_95 = ...
            t_start_from_95;
    
        segments(k).manual_P1D.t_dynamic_start = ...
            t_dynamic_start;
    
        segments(k).manual_P1D.t_step_u = t_step_u;
    
        % Diagnostic
        segments(k).manual_P1D.y0 = y0;
        segments(k).manual_P1D.y_final = y_final;
    
    
        % % =========================================================
        % % Diagnostic plot
        % % =========================================================
        % figure('Name', sprintf('%s segment %d', ACTUATOR, k));
        % 
        % tiledlayout(2,1, ...
        %     'TileSpacing','compact', ...
        %     'Padding','compact');
        % 
        % 
        % % ---------------------------------------------------------
        % % Input u
        % % ---------------------------------------------------------
        % nexttile;
        % 
        % plot(t, u, 'LineWidth',1.2);
        % hold on;
        % grid on;
        % 
        % xline(t_step_u, '--', 'u step');
        % 
        % if ~isnan(t_dynamic_start)
        %     xline(t_dynamic_start, '--', ...
        %         sprintf('P1 start, Td = %.2f ms', ...
        %         Td_manual*1000));
        % end
        % 
        % ylabel('u');
        % 
        % title(sprintf( ...
        %     '%s - segment %d | Tp = %.2f ms | Td = %.2f ms', ...
        %     ACTUATOR, k, ...
        %     Tp_manual*1000, ...
        %     Td_manual*1000));
        % 
        % 
        % % ---------------------------------------------------------
        % % Normalized output
        % % ---------------------------------------------------------
        % nexttile;
        % 
        % plot(t, r, 'LineWidth',1.2);
        % hold on;
        % grid on;
        % 
        % yline(p63, ':', '63.2% = 1\tau');
        % yline(p86, ':', '86.5% = 2\tau');
        % yline(p95, ':', '95.0% = 3\tau');
        % 
        % xline(t_step_u, '--', 'u step');
        % 
        % 
        % if ~isnan(t63)
        %     xline(t63, ':', 't_{63}');
        % end
        % 
        % if ~isnan(t86)
        %     xline(t86, ':', 't_{86}');
        % end
        % 
        % if ~isnan(t95)
        %     xline(t95, ':', 't_{95}');
        % end
        % 
        % 
        % % The three extrapolated P1 start estimates
        % if ~isnan(t_start_from_63)
        % 
        %     xline(t_start_from_63, '--', ...
        %         'start from 63%');
        % 
        %     xline(t_start_from_86, '--', ...
        %         'start from 86%');
        % 
        %     xline(t_start_from_95, '--', ...
        %         'start from 95%');
        % 
        %     % Their mean
        %     xline(t_dynamic_start, '-', ...
        %         sprintf('mean start = %.3f s', ...
        %         t_dynamic_start));
        % end
        % 
        % 
        % xlabel('Time [s]');
        % ylabel('Normalized response');
        % 
        % ylim([-0.1 1.15]);
    
    end
end








%% DAJ MI TUTAJ FUNKCJE KTORA MI ZWIZUALIZUJE 







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
%% DELAYS - ODPAL NA KONCU

all_cu = [];
all_Td_manual = [];
all_total = [];

for a = 1:3
    for s = 1:3

        S = datasets{a,s};

        % Controller -> actuator command delay
        delay_cu = [S.delay];

        % Manually estimated transport delay u -> y
        delay_Td = arrayfun(@(x) x.manual_P1D.Td, S);

        % Total delay c -> beginning of P1 dynamics
        delay_total = delay_cu + delay_Td;

        all_cu        = [all_cu, delay_cu];
        all_Td_manual = [all_Td_manual, delay_Td];
        all_total     = [all_total, delay_total];

    end
end


% %% Convert to ms
all_cu_ms        = all_cu * 1000;
all_Td_manual_ms = all_Td_manual * 1000;
all_total_ms     = all_total * 1000;


% %% Remove NaNs
valid_cu = ~isnan(all_cu_ms);
valid_Td = ~isnan(all_Td_manual_ms);
valid_total = ~isnan(all_total_ms);

cu = all_cu_ms(valid_cu);
Td = all_Td_manual_ms(valid_Td);
total = all_total_ms(valid_total);


% %% =========================================================
% Statistics
% =========================================================

fprintf('\n=================================================\n');
fprintf('DELAYS\n');
fprintf('=================================================\n');

fprintf('\nController -> actuator (c -> u):\n');
fprintf('N      = %d\n', numel(cu));
fprintf('Mean   = %.2f ms\n', mean(cu));
fprintf('Std    = %.2f ms\n', std(cu));
fprintf('Median = %.2f ms\n', median(cu));
fprintf('Min    = %.2f ms\n', min(cu));
fprintf('Max    = %.2f ms\n', max(cu));

fprintf('\nManual transport delay (u -> y):\n');
fprintf('N      = %d\n', numel(Td));
fprintf('Mean   = %.2f ms\n', mean(Td));
fprintf('Std    = %.2f ms\n', std(Td));
fprintf('Median = %.2f ms\n', median(Td));
fprintf('Min    = %.2f ms\n', min(Td));
fprintf('Max    = %.2f ms\n', max(Td));

fprintf('\nTOTAL delay (c -> u + manual Td):\n');
fprintf('N      = %d\n', numel(total));
fprintf('Mean   = %.2f ms\n', mean(total));
fprintf('Std    = %.2f ms\n', std(total));
fprintf('Median = %.2f ms\n', median(total));
fprintf('Min    = %.2f ms\n', min(total));
fprintf('Max    = %.2f ms\n', max(total));


% %% =========================================================
% Main comparison figure
% =========================================================

figure('Name','Delay comparison');

tl = tiledlayout(3,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

title(tl, 'Delay decomposition');


% c -> u
nexttile;

histogram(cu, 'BinWidth',2);
grid on;

xlabel('Delay c \rightarrow u [ms]');
ylabel('Count');
title(sprintf( ...
    'Controller \\rightarrow actuator: mean %.2f ms, std %.2f ms', ...
    mean(cu), std(cu)));


% manual u -> y
nexttile;

histogram(Td, 'BinWidth',2);
grid on;

xlabel('Manual transport delay T_d [ms]');
ylabel('Count');
title(sprintf( ...
    'Manual u \\rightarrow y delay: mean %.2f ms, std %.2f ms', ...
    mean(Td), std(Td)));


% total
nexttile;

histogram(total, 'BinWidth',2);
grid on;

xlabel('Total delay [ms]');
ylabel('Count');
title(sprintf( ...
    'Total c \\rightarrow y delay: mean %.2f ms, std %.2f ms', ...
    mean(total), std(total)));

% %% Delay per measurement

figure('Name','Delays per segment');

plot(all_cu_ms, 'o-');
hold on;

plot(all_Td_manual_ms, 'o-');
plot(all_total_ms, 'o-');

grid on;

xlabel('Measurement');
ylabel('Delay [ms]');

legend( ...
    'c \rightarrow u', ...
    'manual T_d', ...
    'total', ...
    'Location','best');

title('Delay for each segment');


%%
plot_manual_P1D(datasets)
%%
function plot_manual_P1D(datasets)

    Tp_all = [];
    Td_all = [];

    tau_63_86_all = [];
    tau_86_95_all = [];
    tau_63_95_all = [];

    start63_all = [];
    start86_all = [];
    start95_all = [];

    % =========================================================
    % Collect data
    % =========================================================
    for a = 1:3
        for s = 1:3

            S = datasets{a,s};

            Tp = arrayfun(@(x) x.manual_P1D.Tp, S);
            Td = arrayfun(@(x) x.manual_P1D.Td, S);

            tau1 = arrayfun(@(x) x.manual_P1D.tau_63_86, S);
            tau2 = arrayfun(@(x) x.manual_P1D.tau_86_95, S);
            tau3 = arrayfun(@(x) x.manual_P1D.tau_63_95, S);

            st1 = arrayfun(@(x) x.manual_P1D.t_start_from_63, S);
            st2 = arrayfun(@(x) x.manual_P1D.t_start_from_86, S);
            st3 = arrayfun(@(x) x.manual_P1D.t_start_from_95, S);

            Tp_all = [Tp_all Tp];
            Td_all = [Td_all Td];

            tau_63_86_all = [tau_63_86_all tau1];
            tau_86_95_all = [tau_86_95_all tau2];
            tau_63_95_all = [tau_63_95_all tau3];

            start63_all = [start63_all st1];
            start86_all = [start86_all st2];
            start95_all = [start95_all st3];

        end
    end


    % =========================================================
    % Convert to ms where appropriate
    % =========================================================
    Tp_ms = Tp_all * 1000;
    Td_ms = Td_all * 1000;

    tau1_ms = tau_63_86_all * 1000;
    tau2_ms = tau_86_95_all * 1000;
    tau3_ms = tau_63_95_all * 1000;


    % =========================================================
    % Remove NaNs for statistics
    % =========================================================
    Tp_valid = Tp_ms(~isnan(Tp_ms));
    Td_valid = Td_ms(~isnan(Td_ms));

    tau1_valid = tau1_ms(~isnan(tau1_ms));
    tau2_valid = tau2_ms(~isnan(tau2_ms));
    tau3_valid = tau3_ms(~isnan(tau3_ms));


    % =========================================================
    % Statistics
    % =========================================================
    fprintf('\n=================================================\n');
    fprintf('MANUAL P1D ESTIMATION\n');
    fprintf('=================================================\n');

    fprintf('\nTp:\n');
    fprintf('N      = %d\n', numel(Tp_valid));
    fprintf('Mean   = %.2f ms\n', mean(Tp_valid));
    fprintf('Std    = %.2f ms\n', std(Tp_valid));
    fprintf('Median = %.2f ms\n', median(Tp_valid));
    fprintf('Min    = %.2f ms\n', min(Tp_valid));
    fprintf('Max    = %.2f ms\n', max(Tp_valid));

    fprintf('\nTd:\n');
    fprintf('N      = %d\n', numel(Td_valid));
    fprintf('Mean   = %.2f ms\n', mean(Td_valid));
    fprintf('Std    = %.2f ms\n', std(Td_valid));
    fprintf('Median = %.2f ms\n', median(Td_valid));
    fprintf('Min    = %.2f ms\n', min(Td_valid));
    fprintf('Max    = %.2f ms\n', max(Td_valid));

    fprintf('\nTau 63->86:\n');
    fprintf('Mean = %.2f ms, Std = %.2f ms\n', ...
        mean(tau1_valid), std(tau1_valid));

    fprintf('Tau 86->95:\n');
    fprintf('Mean = %.2f ms, Std = %.2f ms\n', ...
        mean(tau2_valid), std(tau2_valid));

    fprintf('Tau 63->95 / 2:\n');
    fprintf('Mean = %.2f ms, Std = %.2f ms\n', ...
        mean(tau3_valid), std(tau3_valid));


    % =========================================================
    % FIGURE 1 - Tp
    % =========================================================
    figure('Name','Manual P1D Tp');

    tiledlayout(2,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    nexttile;

    histogram(Tp_valid);
    grid on;

    xlabel('T_p [ms]');
    ylabel('Count');
    title('Manual P1 time constant');

    nexttile;

    plot(Tp_ms, 'o-');
    grid on;

    xlabel('Measurement');
    ylabel('T_p [ms]');
    title('T_p for each segment');


    % =========================================================
    % FIGURE 2 - Td
    % =========================================================
    figure('Name','Manual P1D Td');

    tiledlayout(2,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    nexttile;

    histogram(Td_valid);
    grid on;

    xlabel('T_d [ms]');
    ylabel('Count');
    title('Manual transport delay');

    nexttile;

    plot(Td_ms, 'o-');
    grid on;

    xlabel('Measurement');
    ylabel('T_d [ms]');
    title('T_d for each segment');


    % =========================================================
    % FIGURE 3 - Compare tau estimates
    % =========================================================
    figure('Name','Tau consistency');

    plot(tau1_ms, 'o-');
    hold on;

    plot(tau2_ms, 'o-');
    plot(tau3_ms, 'o-');

    grid on;

    xlabel('Measurement');
    ylabel('\tau [ms]');

    legend( ...
        '\tau from 63%-86%', ...
        '\tau from 86%-95%', ...
        '\tau from 63%-95%', ...
        'Location','best');

    title('Consistency of time constant estimates');


    % =========================================================
    % FIGURE 4 - Compare extrapolated start times
    % =========================================================
    %
    % Absolute timestamps are not very intuitive, so compare
    % their deviations from their mean for each segment.
    % =========================================================

    start_matrix = [
        start63_all
        start86_all
        start95_all
    ];

    start_mean = mean(start_matrix, 1, 'omitnan');

    err63 = (start63_all - start_mean) * 1000;
    err86 = (start86_all - start_mean) * 1000;
    err95 = (start95_all - start_mean) * 1000;


    figure('Name','P1 start consistency');

    plot(err63, 'o-');
    hold on;

    plot(err86, 'o-');
    plot(err95, 'o-');

    yline(0, '--');

    grid on;

    xlabel('Measurement');
    ylabel('Start estimate error [ms]');

    legend( ...
        'from 63%', ...
        'from 86%', ...
        'from 95%', ...
        'Location','best');

    title('Consistency of extrapolated P1 start');


    % =========================================================
    % Extra useful scalar:
    % spread of the three start estimates
    % =========================================================

    start_spread_ms = ...
        (max(start_matrix, [], 1) - min(start_matrix, [], 1)) * 1000;

    figure('Name','P1 start spread');

    histogram(start_spread_ms(~isnan(start_spread_ms)));
    grid on;

    xlabel('max(start) - min(start) [ms]');
    ylabel('Count');

    title('Agreement of 63%, 86% and 95% extrapolation');


    fprintf('\nP1 start consistency:\n');
    fprintf('Mean spread = %.2f ms\n', ...
        mean(start_spread_ms, 'omitnan'));

    fprintf('Median spread = %.2f ms\n', ...
        median(start_spread_ms, 'omitnan'));

end



%% MAGISTERKA 
return
% dwa niezbyt ładne wykresy, przebiegi skoku w gore i w dol ZROBIONE
k=1;
TSTART=4.8
T = parquetread(paths(k));


t = T.timestamp_s - T.timestamp_s(1);
c = T.can_signals_AUTO_CONTROL_FRONT_LEFT_SETPOINT;
u = T.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US;
y = T.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW;
from = TSTART;
to = from + 1.0;
c_idx = ~isnan(c) & ~isnan(t) & (t >= from) & (t <= to);
t_c = t(c_idx);
c = c(c_idx);
uy_idx = ~isnan(u) & ~isnan(y) & ~isnan(t) & (t >= from) & (t <= to);
t_uy = t(uy_idx);
u = u(uy_idx);
y = y(uy_idx);

ff = figure('Name','actuator_step_up','Units','inches','Position',[2 2 5 3]);
yyaxis left
plot(t_c, c, '.');
ylabel('Requested angle [$^\circ$]', ...
    'Interpreter','latex','FontName','Times New Roman');
yyaxis right
plot(t_uy, y, '.');
ylabel('Raw ADC measurement [-]', ...
    'Interpreter','latex','FontName','Times New Roman');
xlabel('Time [s]', 'Interpreter','latex','FontName','Times New Roman');
set(gca,'FontName','Times New Roman');


t = T.timestamp_s - T.timestamp_s(1);
c = T.can_signals_AUTO_CONTROL_FRONT_LEFT_SETPOINT;
u = T.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_SETPOINT_US;
y = T.can_signals_ACTUATOR_LEFT_FOIL_FEEDBACK_POSITION_RAW;
from = TSTART+1.0+2;
to = from + 1.0;
c_idx = ~isnan(c) & ~isnan(t) & (t >= from) & (t <= to);
t_c = t(c_idx);
c = c(c_idx);
uy_idx = ~isnan(u) & ~isnan(y) & ~isnan(t) & (t >= from) & (t <= to);
t_uy = t(uy_idx);
u = u(uy_idx);
y = y(uy_idx);

fff = figure('Name','actuator_step_down','Units','inches','Position',[2 2 5 3]);
yyaxis left
plot(t_c, c, '.');
ylabel('Requested angle [$^\circ$]', ...
    'Interpreter','latex','FontName','Times New Roman');
yyaxis right
plot(t_uy, y, '.');
ylabel('Raw ADC measurement [-]', ...
    'Interpreter','latex','FontName','Times New Roman');
xlabel('Time [s]', 'Interpreter','latex','FontName','Times New Roman');
set(gca,'FontName','Times New Roman');

exportgraphics(ff, 'actuator_step_up.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');
exportgraphics(fff, 'actuator_step_down.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');

%% MAGISTERKA
% Histogram z manualnego wyliczania dynamiki obiektu. za pomoca 

Tp_all = [];
Td_all = [];
for a = 1:3
    for s = 1:3
        S = datasets{a,s};
        Tp = arrayfun(@(x) x.manual_P1D.Tp, S);
        Td = arrayfun(@(x) x.manual_P1D.Td, S);
        Tp_all = [Tp_all Tp];
        Td_all = [Td_all Td];
    end
end

Tp_ms = Tp_all * 1000;
Td_ms = Td_all * 1000;

Tp_valid = Tp_ms(~isnan(Tp_ms));
Td_valid = Td_ms(~isnan(Td_ms));

ffff = figure('Name','time_constant','Units','inches','Position',[2 2 5 3]);
histogram(Tp_valid);
xlabel('T [ms]');
ylabel('Count');
xlim([0 27]);
% title('Manual P1 time constant');

fffff = figure('Name','transport_delay','Units','inches','Position',[2 2 5 3]);,
histogram(Td_valid);
xlabel('L [ms]');
ylabel('Count');
xlim([10 60]);
% title('Manual transport delay');

Tp_valid = Tp_valid(Tp_valid > 0 & Tp_valid <= 27);
Td_valid = Td_valid(Td_valid >= 10 & Td_valid <= 60);
clc
fprintf("T mean=%f\tmedian=%f\n", mean(Tp_valid), median(Tp_valid));
fprintf("L mean=%f\tmedian=%f\n", mean(Td_valid), median(Td_valid));

exportgraphics(ffff, 'actuator_hist_time_constant.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');
exportgraphics(fffff, 'actuator_hist_transport_delay.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');


%% Save the actuator identification data as .mat file 

T = median(Tp_valid);
L = median(Td_valid);
return
save("hydrofoil_actuator.mat", "L", "T");
return
