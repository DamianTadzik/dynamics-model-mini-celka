function y_noisy = apply_sensor_noise(y, params)
% SNS_APPLY_NOISE   Apply noise + quantization to an ideal signal.
%
% INPUTS:
%   y       - ideal signal (vector)
%   params  - structure returned by sns_analyze(), containing:
%       .noise_sigma   (STD of noise, in same units as y)
%       .q_step        (quantization step, same units as y)
%
% OUTPUT:
%   y_noisy - distorted signal

    arguments
        y (:,:) double
        params struct
    end

    %% Add noise
    %%%% Add white noise
    if isfield(params, "noise_sigma")
        y_noisy = y + params.noise_sigma .* randn(size(y));
    %%%% Add linearly dependant white noise
    else if isfield(params, "noise_sigma_a") && isfield(params, "noise_sigma_b")
            sigma = params.noise_sigma_a .* y + params.noise_sigma_b;
            y_noisy = y + sigma .* randn(size(y));
    %%%% Do not add any noise
    else
        y_noisy = y;
    end

    %% Add bias 
    if isfield(params, "bias")
        y_noisy = y_noisy + params.bias;
    end

    %% Apply quantization
    if isfield(params, "quantization_step")
        y_noisy = params.quantization_step .* round(y_noisy ./ params.quantization_step);
    end


end
