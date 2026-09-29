function test_notch_aware_coasting()
%TEST_NOTCH_AWARE_COASTING  A target that flies tangentially is invisible to
%   the MTI for ~2 s.  The 'full' tracker keeps one track through the gap;
%   the baseline deletes it and has to start again.
set_seed(5);
p = radar_params();
N = 80;
x0 = [-60; 20; 300; 0];                % flies across the beam at 20 m/s
truth.pos = zeros(2, N); truth.vel = zeros(2, N);
dets_all = cell(1, N);
% Measurement noise drawn from the tracker's own calibrated model at 25 dB
% SNR, so the test checks the coasting logic rather than a noise mismatch.
cfg_full = tracker_config(p, 'full');
for f = 1:N
    x = x0 + (f-1)*p.dt*[x0(2); 0; x0(4); 0];
    truth.pos(:, f) = x([1 3]); truth.vel(:, f) = x([2 4]);
    z = meas_model(x);
    if abs(z(3)) < p.notch_vr
        dets_all{f} = struct('range', {}, 'az', {}, 'vr', {}, 'snr_dB', {});   % blind
    else
        d = struct('range', z(1), 'az', z(2), 'vr', z(3), 'snr_dB', 25);
        z = z + sqrt(diag(meas_noise(d, cfg_full))) .* randn(3, 1);
        dets_all{f} = struct('range', z(1), 'az', z(2), 'vr', z(3), 'snr_dB', 25);
    end
end
blind = sum(cellfun(@isempty, dets_all));
assert(blind >= 15, sprintf('scenario only blind for %d frames', blind));

r_full = evaluate_tracks(truth, run_tracker(dets_all, cfg_full), p, N);
r_base = evaluate_tracks(truth, run_tracker(dets_all, tracker_config(p, 'baseline')), p, N);
assert(r_full.breaks(1) == 0, 'notch-aware tracker lost the target');
assert(r_full.coverage(1) > r_base.coverage(1), 'notch coasting did not improve coverage');
end
