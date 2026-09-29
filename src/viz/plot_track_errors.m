function plot_track_errors(truth, res_base, res_full)
%PLOT_TRACK_ERRORS  Position error vs time per target, baseline vs improved.
st = plot_style();
f = figure('Position', [100 100 1150 360]);
for k = 1:numel(truth)
    subplot(1, numel(truth), k); hold on; grid on; box on;
    [t1, e1] = with_gaps(res_base.err_t{k}, res_base.err_pos{k});
    [t2, e2] = with_gaps(res_full.err_t{k}, res_full.err_pos{k});
    plot(t1, e1, '-', 'Color', st.cfg(1,:), 'LineWidth', 1.4);
    plot(t2, e2, '-', 'Color', st.cfg(3,:), 'LineWidth', 1.6);
    xlabel('Time (s)'); ylabel('Position error (m)');
    title(truth(k).name);
    xlim([0 8]);
    if k == 1, legend({'Baseline', 'Adaptive R + notch coast'}, 'Location', 'northeast'); end
end
save_fig(f, 'track_error');
end

function [t, e] = with_gaps(t, e)
% Insert NaN where no confirmed track existed, so gaps aren't drawn as lines
gap = find(diff(t) > 0.15);
for g = fliplr(gap)
    t = [t(1:g), nan, t(g+1:end)];
    e = [e(1:g), nan, e(g+1:end)];
end
end
