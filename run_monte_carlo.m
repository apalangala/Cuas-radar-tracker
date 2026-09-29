%% RUN_MONTE_CARLO  Statistical comparison of the three tracker designs.
%  One simulated run is an anecdote.  This script repeats the full scenario
%  (clutter + MTI on, Swerling I targets) with Nmc different random seeds,
%  and runs all three tracker variants on EXACTLY the same detections each
%  time (a paired comparison), so differences come from the tracker alone.
%
%  Outputs: printed summary table, results/mc_results.mat, and figures
%  docs/img/mc_rms.png and docs/img/mc_nees.png.
%  Runtime: ~5 min in MATLAB for Nmc = 30 (longer in Octave).

clear; close all;
setup_paths;
p = radar_params();
Nframes = 80;
Nmc = 30;
variants = {'baseline', 'adaptive', 'full'};
nV = numel(variants);

truth = make_scenario(p, Nframes);
nT = numel(truth);

mc.variants = variants;
mc.Nmc = Nmc; mc.Nframes = Nframes; mc.dt = p.dt;
mc.rms_pos = nan(Nmc, nT, nV);  mc.rms_vel = nan(Nmc, nT, nV);
mc.coverage = nan(Nmc, nT, nV); mc.breaks = nan(Nmc, nT, nV);
mc.false_tracks = nan(Nmc, nV);
mc.n_true = nan(Nmc, 1); mc.n_false = nan(Nmc, 1);
for v = 1:nV, for k = 1:nT, mc.nees{v, k} = nan(Nframes, Nmc); end, end

t0 = tic;
for s = 1:Nmc
    sim = simulate_detections(p, truth, Nframes, 1000 + s);
    mc.n_true(s) = mean(sim.n_true); mc.n_false(s) = mean(sim.n_false);
    for v = 1:nV
        cfg = tracker_config(p, variants{v});
        res = evaluate_tracks(truth, run_tracker(sim.dets, cfg), p, Nframes);
        mc.rms_pos(s, :, v)  = res.rms_pos_ss;
        mc.rms_vel(s, :, v)  = res.rms_vel_ss;
        mc.coverage(s, :, v) = res.coverage;
        mc.breaks(s, :, v)   = res.breaks;
        mc.false_tracks(s, v) = res.n_false_tracks;
        for k = 1:nT
            fi = round(res.err_t{k} / p.dt) + 1;
            mc.nees{v, k}(fi, s) = res.nees{k};
        end
    end
    fprintf('run %2d/%d done (%.0f s elapsed)\n', s, Nmc, toc(t0));
end

if ~exist('results', 'dir'), mkdir('results'); end
save(fullfile('results', 'mc_results.mat'), 'mc');

% ---------------- Summary ----------------
fprintf('\n=== Monte Carlo summary: %d runs x %.0f s, clutter + MTI, Swerling I ===\n', Nmc, Nframes*p.dt);
fprintf('Detection: %.2f of 3 targets detected per frame, %.2f false alarms per frame\n\n', ...
        mean(mc.n_true), mean(mc.n_false));
fprintf('%-26s %-9s %16s %16s %10s %12s\n', 'Target', 'Tracker', 'Pos RMS (m)', 'Vel RMS (m/s)', 'Coverage', 'Breaks/run');
for k = 1:nT
    for v = 1:nV
        a = mc.rms_pos(:, k, v); b = mc.rms_vel(:, k, v);
        fprintf('%-26s %-9s %7.2f +/- %-5.2f %7.2f +/- %-5.2f %9.0f%% %12.2f\n', ...
            truth(k).name, variants{v}, mean_omitnan(a), std_omitnan(a), mean_omitnan(b), std_omitnan(b), ...
            100*mean(mc.coverage(:, k, v)), mean(mc.breaks(:, k, v)));
    end
end
fprintf('\nFalse confirmed tracks per run: %s\n', sprintf('%s %.2f  ', ...
    variants{1}, mean(mc.false_tracks(:,1)), variants{2}, mean(mc.false_tracks(:,2)), ...
    variants{3}, mean(mc.false_tracks(:,3))));
fprintf('Average NEES (ideal = 4):\n');
for v = 1:nV
    anees = zeros(1, nT);
    for k = 1:nT, x = mc.nees{v, k}; anees(k) = mean_omitnan(x(:)); end
    fprintf('  %-9s  %s\n', variants{v}, sprintf('%-6.2f', anees));
end

plot_mc_summary(mc, truth);
