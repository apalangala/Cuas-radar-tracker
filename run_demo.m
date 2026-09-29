%% RUN_DEMO  Counter-UAS radar: one scenario, end to end.
%  X-band FMCW radar, 8-element receive array, three small UAVs in ground
%  clutter.  Every 0.1 s:
%     raw beat signal -> MTI -> range-Doppler map -> 2-D CA-CFAR -> angle FFT
%     -> multi-target EKF tracker
%  Compares the baseline tracker with the improved one on the same data and
%  saves every figure used in the README to docs/img.
%
%  Plain MATLAB (R2016b+), no toolboxes.  Also runs in GNU Octave.
%  Runtime: ~30 s in MATLAB.

clear; close all; clc;
setup_paths;
p = radar_params();
Nframes = 80;                   % 8 s
seed = 7;
snap_frame = 30;

truth = make_scenario(p, Nframes);

fprintf('=== Radar design ===\n');
fprintf('Range resolution     %5.2f m     (unambiguous to %.0f m)\n', p.range_res, p.range_max);
fprintf('Velocity resolution  %5.2f m/s   (unambiguous to +/-%.1f m/s)\n', p.vel_res, p.vel_max);
fprintf('Angle resolution    ~%5.1f deg   (%d-element array)\n', (2/p.Nrx)*180/pi, p.Nrx);
fprintf('CPI %.1f ms, processing gain %.1f dB, MTI blind zone +/-%.2f m/s\n\n', ...
        p.Nc*p.Tpri*1e3, p.proc_gain_dB, p.notch_vr);

%  Front end: simulate + detect
fprintf('Simulating %d frames ...\n', Nframes);
sim = simulate_detections(p, truth, Nframes, seed);
fprintf('Detections on targets: %.2f of 3 per frame | false alarms: %.2f per frame\n\n', ...
        mean(sim.n_true), mean(sim.n_false));

% Same frame processed with and without the clutter canceller
set_seed(seed + 100);
cube = simulate_frame(p, sim.tg{snap_frame}, sim.clutter);
p_off = p; p_off.mti = false;
[d_off, rd_off] = process_frame(p_off, cube);
[d_on,  rd_on]  = process_frame(p, cube);
fprintf('Frame %d: %d detections without MTI, %d with MTI (3 real targets)\n\n', ...
        snap_frame, numel(d_off), numel(d_on));

%  Trackers 
variants = {'baseline', 'full'};
for v = 1:numel(variants)
    cfg = tracker_config(p, variants{v});
    hs{v}  = run_tracker(sim.dets, cfg);                 %#ok<SAGROW>
    res{v} = evaluate_tracks(truth, hs{v}, p, Nframes);  %#ok<SAGROW>
end

fprintf('=== Tracking (confirmed tracks, steady state) ===\n');
fprintf('%-26s %-9s %10s %12s %9s %7s %6s\n', 'Target', 'Tracker', 'Pos RMS', 'Vel RMS', 'Coverage', 'Breaks', 'NEES');
for k = 1:numel(truth)
    for v = 1:numel(variants)
        r = res{v};
        fprintf('%-26s %-9s %8.2f m %8.2f m/s %8.0f%% %7d %6.1f\n', truth(k).name, variants{v}, ...
            r.rms_pos_ss(k), r.rms_vel_ss(k), 100*r.coverage(k), r.breaks(k), r.mean_nees(k));
    end
end
fprintf('False confirmed tracks: baseline %d, full %d\n', res{1}.n_false_tracks, res{2}.n_false_tracks);
fprintf('(NEES: 4 = statistically consistent filter; >> 4 over-confident; << 4 under-confident)\n\n');

%  Figures 
plot_rd_compare(p, rd_off, rd_on, sim.tg{snap_frame}, snap_frame);
plot_tactical(p, truth, sim, hs{2});
plot_track_errors(truth, res{1}, res{2});
make_tracking_gif(p, truth, sim, hs{2});
fprintf('Figures saved to docs/img\n');
