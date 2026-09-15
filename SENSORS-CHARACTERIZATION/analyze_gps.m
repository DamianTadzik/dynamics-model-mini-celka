%% Load file
path = "D:\Dane\workspace\logs-mini-celka\logs_storage\2026_04_26_divonnes\logs_parquet\";
parquet_files = ["log0" "log1" "log2" "log3"] + ".parquet";


% for each make subplot
throttle_data = [];
speed_data = [];
accuracy_data = [];

% for k = 2
for k = 1:numel(parquet_files)
    parquet_file = path + parquet_files(k);
    
    % Actually load file
    T = parquetread(parquet_file);
    % info = parquetinfo(parquet_file);
    % for i = 1:length(T.Properties.VariableDescriptions)
    %     disp(T.Properties.VariableDescriptions{i});
    % end
 
    time = T.timestamp - T.timestamp(1);
    dtime = diff(time);

    figure('Name', parquet_files(k));
    plot(dtime)

    figure('Name', parquet_files(k));
    ax1 = subplot(3,1,1);
    plot(time, T.inputs_RADIO_THROTTLE);
    
    ax2 = subplot(3,1,2);
    plot(time, T.inputs_GPS_GROUND_SPEED);

    ax3 = subplot(3,1,3);
    plot(time, T.inputs_GPS_GROUND_SPEED_ACCURACY);
    linkaxes([ax1 ax2 ax3], 'x');

    if k == 1
        throttle_data = [throttle_data, sns_cut_fragment(time, T.inputs_RADIO_THROTTLE,            122, 127)];
        speed_data    = [speed_data;    sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED,          122, 127)];
        accuracy_data = [accuracy_data; sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED_ACCURACY, 122, 127)];

        throttle_data = [throttle_data, sns_cut_fragment(time, T.inputs_RADIO_THROTTLE,            148, 155)];
        speed_data    = [speed_data;    sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED,          148, 155)];
        accuracy_data = [accuracy_data; sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED_ACCURACY, 148, 155)];
    end

    if k == 2
        throttle_data = [throttle_data, sns_cut_fragment(time, T.inputs_RADIO_THROTTLE,            333, 338)];
        speed_data    = [speed_data;    sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED,          333, 338)];
        accuracy_data = [accuracy_data; sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED_ACCURACY, 333, 338)];

        throttle_data = [throttle_data, sns_cut_fragment(time, T.inputs_RADIO_THROTTLE,            751, 755)];
        speed_data    = [speed_data;    sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED,          751, 755)];
        accuracy_data = [accuracy_data; sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED_ACCURACY, 751, 755)];
    end 

    if k == 3
        throttle_data = [throttle_data, sns_cut_fragment(time, T.inputs_RADIO_THROTTLE,            260, 330)];
        speed_data    = [speed_data;    sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED,          260, 330)];
        accuracy_data = [accuracy_data; sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED_ACCURACY, 260, 330)];

        throttle_data = [throttle_data, sns_cut_fragment(time, T.inputs_RADIO_THROTTLE,            532, 541)];
        speed_data    = [speed_data;    sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED,          532, 541)];
        accuracy_data = [accuracy_data; sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED_ACCURACY, 532, 541)];
    end 

    if k == 4
        throttle_data = [throttle_data, sns_cut_fragment(time, T.inputs_RADIO_THROTTLE,            615, 627)];
        speed_data    = [speed_data;    sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED,          615, 627)];
        accuracy_data = [accuracy_data; sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED_ACCURACY, 615, 627)];

        % throttle_data = [throttle_data, sns_cut_fragment(time, T.inputs_RADIO_THROTTLE,            532, 541)];
        % speed_data    = [speed_data;    sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED,          532, 541)];
        % accuracy_data = [accuracy_data; sns_cut_fragment(time, T.inputs_GPS_GROUND_SPEED_ACCURACY, 532, 541)];
    end 


    % %% quantization check (should be 1 mm/s? aka 0.001 m/s) M10Q woow
    % u = unique(T.inputs_GPS_GROUND_SPEED);
    % du = diff(u);
end

%% analysis

speed_sigmas = [];
speed_means = [];
speed_sigmas_diff = [];

sacc_means = [];
sacc_medians = [];

fprintf("speed mean\t speed sigma\t speed sigdiff\t sacc mean\tsacc median\n");

for k = 1:numel(speed_data)

    speed_sigmas = [speed_sigmas, std(speed_data(k).x)];
    speed_means = [speed_means, mean(speed_data(k).x)];

    speed_sigmas_diff = [speed_sigmas_diff, std(diff(speed_data(k).x))/sqrt(2)];

    sacc_means  = [sacc_means, mean(accuracy_data(k).x)];
    sacc_medians  = [sacc_medians, median(accuracy_data(k).x)];

    fprintf("%f\t%f\t%f\t%f\t%f\n", ...
        speed_means(k), ...
        speed_sigmas(k), ...
        speed_sigmas_diff(k), ...
        sacc_means(k), ...
        sacc_medians(k))
end

% speed_sigmas
% speed_means
% speed_sigmas_diff

%% analysis cd..

for k = 1:numel(speed_data)
    % extract data
    v = speed_data(k).x;

    % detrend linear
    p = polyfit((1:length(v))', v, 1);
    v_trend = polyval(p, (1:length(v))');
    % residual
    e = v - v_trend;
    % e = v - mean(v);
    
    % resamplig trzeba XD
    ind = 1:10:numel(e);
    e = e(ind);
    v = v(ind);


    figure;
    histogram(e, 30, 'Normalization','pdf', 'BinWidth',0.001);
    grid on
    hold on
    x = linspace(min(e), max(e), 200);
    mu = mean(e);
    sigma = std(e);
    plot(x, normpdf(x, mu, sigma), 'LineWidth', 2);

    % fprintf("%f\t%f\n", ...
    %     mu, sigma);


    [e_corr,lags] = xcorr(e - mean(e), 30, 'coeff');
    figure;
    stem(lags*0.1, e_corr);
    xlabel('lag [s]');
    ylabel('autocorrelation');
    grid on


    rho1 = corr(e(1:end-1), e(2:end));
    sigma_e = std(e);
    sigma_diff = std(diff(v))/sqrt(2);
    
    % fprintf("rho1 = %.3f\n", rho1);
    fprintf("sigma = %.4f m/s\n", sigma_e);
    fprintf("sigma_diff = %.4f m/s\n", sigma_diff);
end

%% PLOT MAGISTERA
% return
% reprezentatywny przebieg
i=2;
time = speed_data(i).t - speed_data(i).t(1);
value = speed_data(i).x;

% resamplig trzeba XD
ind = 1:10:numel(time);
time = time(ind);
value = value(ind);

    % detrend linear
    p = polyfit((1:length(value))', value, 1);
    v_trend = polyval(p, (1:length(value))');

    value = value - v_trend + mean(value);

% Plot przebiegu z sensora
f = figure('Name','sensor_GPS_output','Units','centimeters','Position',[2 2 8 5]);
plot(time, value, '.')
xlabel('Time [s]','FontName','Times New Roman','FontSize',9)
ylabel('Measured speed [m/s]','FontName','Times New Roman','FontSize',9)
grid on;
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
xlim([time(1) time(end)]);
ylim([min(value), max(value)]);

% Plot histogramu
ff = figure('Name','sensor_GPS_histogram','Units','centimeters','Position',[2 2 8 5]);
    q_step = 0.001;
    noise = (value - mean(value));
    edges = (max(value) - min(value))/q_step;
    edges = (-edges:edges+1)*q_step - q_step/2; 
    % histogram(noise, edges); grid on;
    histogram(noise, "BinWidth", 0.005); grid on;

xlabel('Deviation from mean speed [m/s]','FontName','Times New Roman','FontSize',9)
ylabel('Count','FontName','Times New Roman','FontSize',9)
grid on;
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',9);
xlim([-0.1 0.1]);

exportgraphics(f, 'sensor_GPS_output.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 5);
exportgraphics(ff, 'sensor_GPS_histogram.pdf', ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none', ...
    'Units', 'centimeters', ...
    'Width', 8, ...
    'Height', 5);


%% Save the parameters 


% vGPS​[k]=Q(vtrue​[k]+n[k]) model z gaussowskim szumem jako approximation
% good enough i would tell. 
gps_parameters.quantization_step = 0.001; % 1 mm/s
gps_parameters.sample_time = 0.1; % 10 Hz 
gps_parameters.std_noise = 0.05; % approx 

save("gps_parameters.mat", "gps_parameters")

% Ewentualnie mozna by sie pokusic o jakis AR(1) z gaussem ale to bez
% sensu imo. chyba ze chce napchac prace rzeczami.
