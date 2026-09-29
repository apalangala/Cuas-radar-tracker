%% RUN_PD_CURVE  Detection performance envelope: Pd vs range for several RCS.
%  Four targets of different size are placed at the same range (separated in
%  Doppler and angle so they don't interact), and the radar is run many times
%  at each range.  Pd = fraction of trials in which the target is detected.
%  Targets fluctuate (Swerling I), noise-limited (no clutter).
%
%  Outputs: docs/img/pd_vs_range.png and a table of the range at which each
%  target is detected with Pd >= 0.9.   Runtime: ~3 min in MATLAB.

clear; close all;
setup_paths;
p = radar_params();
p.clutter = false; p.mti = false;
set_seed(21);

ranges  = 75:25:475;
rcs     = [0.01 0.02 0.05 0.10];
rcs_lbl = {'0.01 m^2 (micro drone)', '0.02 m^2 (small quadcopter)', ...
           '0.05 m^2 (large quadcopter)', '0.10 m^2 (small fixed-wing)'};
vr = [-20 -8 8 20];
az = [-30 -10 10 30] * pi/180;
Ntrials = 100;

nR = numel(ranges); nS = numel(rcs);
hits = zeros(nR, nS);
t0 = tic;
for ir = 1:nR
    for n = 1:Ntrials
        tg = struct('range', {}, 'vr', {}, 'az', {}, 'rcs', {});
        for s = 1:nS
            tg(s).range = ranges(ir) + (rand - 0.5)*p.range_res;   % random sub-bin offset
            tg(s).vr = vr(s); tg(s).az = az(s); tg(s).rcs = rcs(s);
        end
        dets = process_frame(p, simulate_frame(p, tg));
        for s = 1:nS
            for j = 1:numel(dets)
                if abs(dets(j).range - tg(s).range) < 2*p.range_res && ...
                   abs(dets(j).vr - tg(s).vr) < 2*p.vel_res
                    hits(ir, s) = hits(ir, s) + 1; break;
                end
            end
        end
    end
    fprintf('range %3d m done (%.0f s)\n', ranges(ir), toc(t0));
end
Pd = hits / Ntrials;

% Range at which Pd first drops below 0.9 (linear interpolation)
R90 = nan(1, nS);
for s = 1:nS
    i = find(Pd(:, s) < 0.9, 1, 'first');
    if isempty(i), R90(s) = ranges(end);
    elseif i == 1, R90(s) = ranges(1);
    else
        R90(s) = interp1(Pd([i-1 i], s), ranges([i-1 i]), 0.9);
    end
end
fprintf('\nDetection range at Pd >= 0.9 (Pfa = %.0e per cell, Swerling I):\n', p.Pfa);
for s = 1:nS
    if all(Pd(:, s) >= 0.9)
        fprintf('  %-30s >= %.0f m (edge of coverage)\n', rcs_lbl{s}, ranges(end));
    else
        fprintf('  %-30s %4.0f m\n', rcs_lbl{s}, R90(s));
    end
end

if ~exist('results', 'dir'), mkdir('results'); end
save(fullfile('results', 'pd_results.mat'), 'ranges', 'rcs', 'Pd', 'R90', 'Ntrials');

% ---- Plot ----
st = plot_style();
cols = [0.60 0.75 0.90; st.c(1,:); st.c(3,:); st.c(2,:)];
f = figure('Position', [100 100 900 430]); hold on; box on; grid on;
h = zeros(1, nS);
for s = 1:nS
    h(s) = plot(ranges, Pd(:, s), 'o-', 'Color', cols(s,:), 'MarkerFaceColor', cols(s,:), ...
                'MarkerSize', 4, 'LineWidth', 1.8);
end
plot([ranges(1) ranges(end)], [0.9 0.9], 'k--');
text(ranges(1) + 5, 0.93, 'P_d = 0.9', 'FontSize', 10);
xlabel('Range (m)'); ylabel('Probability of detection per CPI');
title(sprintf('Detection performance, Swerling I, P_{fa} = %.0e, %d trials per point', p.Pfa, Ntrials));
legend(h, rcs_lbl, 'Location', 'southwest');
ylim([0 1.05]); xlim([ranges(1) ranges(end)]);
save_fig(f, 'pd_vs_range');
