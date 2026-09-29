%% RUN_CALIBRATION  Measure how detection accuracy depends on SNR.
%  A single target is simulated many times at random positions.  For each
%  detection we record the estimated SNR and the errors in range, sin(azimuth)
%  and radial velocity.  Estimation theory (the Cramer-Rao bound) says each
%  error should scale like  sigma = k * resolution / sqrt(SNR).  At high SNR
%  the curve flattens at a small floor set by the bias of the sub-bin
%  interpolator, so the full model fitted is
%        sigma^2 = (k * resolution / sqrt(SNR))^2 + floor^2
%
%  The fitted constants are what tracker_config.m uses for the SNR-adaptive
%  measurement-noise model.  Re-run this if you change the radar design.
%  Runtime: ~1 min.

clear; close all;
setup_paths;
p = radar_params();
p.clutter = false; p.mti = false; p.swerling = 0;
set_seed(11);

Ntrials = 600;
rec = zeros(Ntrials, 4);   % [snr_lin, err_range, err_sin_az, err_vr]
n = 0;
for i = 1:Ntrials
    tg.range = 60 + 420*rand;
    tg.az    = (rand - 0.5) * 80*pi/180;
    tg.vr    = sign(rand - 0.5) * (8 + 40*rand);
    tg.rcs   = 10^((-25 + 20*rand)/10);           % -25 .. -5 dBsm
    dets = process_frame(p, simulate_frame(p, tg));
    if isempty(dets), continue; end
    [~, j] = min(abs([dets.range] - tg.range));
    if abs(dets(j).range - tg.range) > 3*p.range_res, continue; end
    n = n + 1;
    rec(n, :) = [10^(dets(j).snr_dB/10), dets(j).range - tg.range, ...
                 sin(dets(j).az) - sin(tg.az), dets(j).vr - tg.vr];
end
rec = rec(1:n, :);

% Bin by SNR, then fit  sigma^2 = k^2 * x^2 + floor^2,  x = res/sqrt(snr).
% Linear least squares on relative residuals: [x^2/s^2, 1/s^2] * [k^2; fl^2] = 1
res_units = [p.range_res, 2/p.Nrx, p.vel_res];   % range bin, beamwidth in sin-space, Doppler bin
edges = 10:3:40;  centres = edges(1:end-1) + 1.5;
snr_dB = 10*log10(rec(:,1));
k_fit = zeros(1, 3); fl_fit = zeros(1, 3);
stds = nan(numel(centres), 3);
for c = 1:3
    for b = 1:numel(centres)
        in = snr_dB >= edges(b) & snr_dB < edges(b+1);
        if sum(in) >= 8, stds(b, c) = std(rec(in, c+1)); end
    end
    ok = ~isnan(stds(:, c));
    x = res_units(c) ./ sqrt(10.^(centres(ok)/10));  x = x(:);
    s2 = stds(ok, c).^2;
    A = [x.^2 ./ s2, 1 ./ s2];
    sol = A \ ones(size(s2));
    sol = max(sol, 0);
    k_fit(c) = sqrt(sol(1)); fl_fit(c) = sqrt(sol(2));
end

fprintf('%d detections from %d trials\n', n, Ntrials);
fprintf('Fitted model  sigma^2 = (k*res/sqrt(SNR))^2 + floor^2:\n');
fprintf('  range : k = %.3f, floor = %.4f m\n',   k_fit(1), fl_fit(1));
fprintf('  sin az: k = %.3f, floor = %.5f\n',     k_fit(2), fl_fit(2));
fprintf('  vr    : k = %.3f, floor = %.4f m/s\n', k_fit(3), fl_fit(3));
fprintf('Copy these into src/tracking/tracker_config.m\n');

% Plot
st = plot_style();
f = figure('Position', [100 100 1100 360]);
names = {'Range error (m)', 'sin(azimuth) error', 'Radial velocity error (m/s)'};
for c = 1:3
    subplot(1, 3, c); hold on; grid on; box on;
    ss = linspace(10, 40, 100);
    plot(centres, stds(:, c), 'o', 'Color', st.c(1,:), 'MarkerFaceColor', st.c(1,:));
    plot(ss, sqrt((k_fit(c) * res_units(c)).^2 ./ 10.^(ss/10) + fl_fit(c)^2), '-', 'Color', st.c(2,:), 'LineWidth', 1.8);
    set(gca, 'YScale', 'log');
    xlabel('Detection SNR (dB)'); ylabel(['\sigma  ' names{c}]);
    title(sprintf('k = %.2f, floor = %.2g', k_fit(c), fl_fit(c)));
    if c == 1, legend({'Measured', 'Fitted model'}, 'Location', 'southwest'); end
end
save_fig(f, 'calibration');
