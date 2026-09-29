function test_tracker_consistency()
%TEST_TRACKER_CONSISTENCY  With measurements whose noise exactly matches the
%   filter's assumption, the tracker converges and its NEES averages ~4.
set_seed(4);
p = radar_params();
cfg = tracker_config(p, 'baseline');
sig = cfg.fixed_sig;
N = 100; Nruns = 10;
nees_all = [];
for run = 1:Nruns
    x0 = [-80; 8; 300; -10];
    truth.pos = zeros(2, N); truth.vel = zeros(2, N);
    dets_all = cell(1, N);
    for f = 1:N
        x = x0 + (f-1)*p.dt*[x0(2); 0; x0(4); 0];
        truth.pos(:, f) = x([1 3]); truth.vel(:, f) = x([2 4]);
        z = meas_model(x) + sig(:) .* randn(3, 1);
        dets_all{f} = struct('range', z(1), 'az', z(2), 'vr', z(3), 'snr_dB', 20);
    end
    hs = run_tracker(dets_all, cfg);
    res = evaluate_tracks(truth, hs, p, N);
    assert(res.breaks(1) == 0, 'track broke on a clean straight-line target');
    nees_all = [nees_all, res.nees{1}(end-49:end)]; %#ok<AGROW>
    assert(res.rms_pos_ss(1) < 3, sprintf('RMS position error %.2f m', res.rms_pos_ss(1)));
end
m = mean(nees_all);
assert(m > 2.5 && m < 6, sprintf('average NEES %.2f, expected ~4', m));
end
