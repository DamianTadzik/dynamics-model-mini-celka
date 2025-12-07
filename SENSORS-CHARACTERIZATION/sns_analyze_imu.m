function params = sns_analyze_imu(t, x, q_step)
% SNS_ANALYZE  Analyze noise and timing properties of a quantized sensor.
%
% INPUTS:
%   x      - signal vector (already quantized by sensor)
%   t      - time vector
%   q_step - known quantization step of the sensor (e.g. 2 mm)
%
% OUTPUT:
%   params.noise_sigma  - STD of noise
%   params.dt_mean      - mean sampling period
%   params.dt_std       - sampling jitter
%   params.range        - [min, max]
%   params.n_samples    - number of usable samples
%   params.q_step       - copied from input
%
    arguments
        t (:,1) double
        x (:,1) double
        q_step   (1,1) double
    end

    %% Basic stats
    params.n_samples = numel(x);
    params.range = [min(x), max(x)];
    params.q_step = q_step;

    %% Noise estimation
    % Since sensor output is quantized and ideal signal during measurement
    % is constant (stationary test), noise ≈ variation around the mode.
    params.x_mode = mean(x);           % expected stationary signal value
    noise = (x - params.x_mode);       % variation around dominant quantized bin

    % Mean and sigma of noise
    params.noise_sigma = std(noise);
    params.noise_mean = mean(noise);

    % Skewness
    params.noise_skew = skewness(noise);

    % Probabilities of <0, ==0, >0
    params.p_neg  = mean(noise < 0);
    params.p_zero = mean(noise == 0);
    params.p_pos  = mean(noise > 0);

    % Counts per discrete level
    span  = max(abs(noise));
    K     = ceil(span / q_step);          % max level index
    levels = (-K:K) * q_step;             % -4 -2 0 2 4 ...
    edges  = levels - q_step/2;
    edges(end+1) = edges(end) + q_step;

    [counts, ~] = histcounts(noise, edges);
    params.levels = levels;
    params.counts = counts;

    %% Timing jitter
    dt = diff(t);
    params.dt_mean = mean(dt);
    params.dt_std  = std(dt);

    %% Plots
    figure;

    subplot(2,1,1);
    plot(t, x, '-ob'); grid on;
    title("Signal");

    subplot(2,1,2);
    edges = (max(x) - min(x))/q_step;
    edges = (-edges:edges+1)*q_step - q_step/2; 
    histogram(noise, edges); grid on;
    title("Noise histogram");

end
