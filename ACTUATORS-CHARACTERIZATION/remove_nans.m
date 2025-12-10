function [clean_time, clean_signal] = remove_nans(time, signal)
    % Remove NaNs from the input vectors
    valid_indices = ~isnan(time) & ~isnan(signal);
    clean_time = time(valid_indices);
    clean_signal = signal(valid_indices);
end