function ret = sns_cut_fragment(t, x, t_start, t_end)
% SNS_CUT_FRAGMENT  Visualizes signal and extracts a time-window fragment.
% Removes NaNs automatically.

    arguments
        t (:,1) double
        x (:,1) double
        t_start (1,1) double
        t_end   (1,1) double
    end

    %% Remove NaNs
    valid = ~isnan(x) & ~isnan(t);
    x = x(valid);
    t = t(valid);

    %% Compute index range
    idx = (t >= t_start) & (t <= t_end);

    x_trim = x(idx);
    t_trim = t(idx);

    %% Plot
    figure; hold on;
    plot(t, x, 'b-', 'LineWidth', 1);
    ylims = ylim;

    % Mark trimming boundaries
    plot([t_start t_start], ylims, 'r--', 'LineWidth', 1.5);
    plot([t_end   t_end],   ylims, 'r--', 'LineWidth', 1.5);

    title("Signal with selected fragment");
    xlabel("Time [s]");
    ylabel("Signal");
    grid on;
    legend("Signal", "t\_start", "t\_end");

    %% Return
    ret.x = x_trim;
    ret.t = t_trim;
end
