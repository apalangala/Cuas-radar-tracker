function plot_mc_summary(mc, truth)
%PLOT_MC_SUMMARY  Monte Carlo figures: RMS error / coverage bars, and ANEES.
st = plot_style();
nT = numel(truth); nV = numel(mc.variants);
lbl = {'Baseline', 'Adaptive R', 'Adaptive R + notch coast'};
tnames = {'T1 quadcopter', 'T2 crossing', 'T3 turning'};

% ---------- RMS position error and coverage ----------
f = figure('Position', [100 100 1100 420]);
M = zeros(nT, nV); E = zeros(nT, nV); C = zeros(nT, nV);
for k = 1:nT
    for v = 1:nV
        x = mc.rms_pos(:, k, v); x = x(~isnan(x));
        M(k, v) = mean(x); E(k, v) = std(x) / sqrt(numel(x));   % standard error
        C(k, v) = 100 * mean(mc.coverage(:, k, v));
    end
end
subplot(1, 2, 1); hold on; box on; grid on;
draw_grouped(M, E, st.cfg);
set(gca, 'XTick', 1:nT, 'XTickLabel', tnames);
ylabel('Steady-state RMS position error (m)');
title(sprintf('Position accuracy (%d runs, mean \\pm s.e.)', mc.Nmc));
legend(lbl, 'Location', 'northeast');

subplot(1, 2, 2); hold on; box on; grid on;
draw_grouped(C, zeros(size(C)), st.cfg);
set(gca, 'XTick', 1:nT, 'XTickLabel', tnames);
ylabel('Frames with a confirmed track (%)'); ylim([0 105]);
title('Track coverage (T2 crosses the MTI blind zone)');
save_fig(f, 'mc_rms');

% ---------- ANEES vs time, with 95% consistency band ----------
% For N independent runs the sum of N NEES values (4 dof each) is chi-square
% with 4N dof, so the average must lie in [chi2inv(.025,4N), chi2inv(.975,4N)]/N.
f = figure('Position', [100 100 1100 360]);
t = (0:mc.Nframes-1) * mc.dt;
show = [1 nV];   % baseline vs full
for k = 1:nT
    subplot(1, nT, k); hold on; box on; grid on;
    for v = show
        X = mc.nees{v, k};
        n = sum(~isnan(X), 2);
        X(isnan(X)) = 0;
        anees = sum(X, 2) ./ max(n, 1); anees(n < 0.5*mc.Nmc) = nan;
        N = max(n);
        lo = 2*gammaincinv(0.025, 2*N) / N; hi = 2*gammaincinv(0.975, 2*N) / N;
        if v == show(1)
            fill([t(1) t(end) t(end) t(1)], [lo lo hi hi], [0.85 0.93 0.85], 'EdgeColor', 'none');
            plot([t(1) t(end)], [4 4], 'k:');
        end
        plot(t, anees, '-', 'Color', st.cfg(v, :), 'LineWidth', 1.6);
    end
    set(gca, 'YScale', 'log'); ylim([0.05 100]);
    xlabel('Time (s)'); ylabel('Average NEES');
    title(tnames{k});
    if k == 1
        h = get(gca, 'Children');
        legend(h([2 1]), {lbl{show(1)}, lbl{show(2)}}, 'Location', 'northeast');
    end
end
save_fig(f, 'mc_nees');
end

function draw_grouped(M, E, cols)
[nG, nV] = size(M);
w = 0.8 / nV;
for v = 1:nV
    x = (1:nG) - 0.4 + w*(v - 0.5);
    bar(x, M(:, v), w, 'FaceColor', cols(v, :), 'EdgeColor', 'none');
end
for v = 1:nV
    x = (1:nG) - 0.4 + w*(v - 0.5);
    if any(E(:, v) > 0)
        he = errorbar(x, M(:, v), E(:, v), 'k.');
        set(he, 'MarkerSize', 1);
    end
end
xlim([0.5 nG + 0.5]);
end
