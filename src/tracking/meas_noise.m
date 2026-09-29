function R = meas_noise(det, cfg)
%MEAS_NOISE  3x3 measurement covariance for one detection.
%   Baseline: the same fixed covariance for every detection.
%   Adaptive: each standard deviation follows the calibrated model
%       sigma^2 = (k * res / sqrt(SNR))^2 + floor^2
%   so a weak, far-away detection is trusted less than a strong one.
if ~cfg.adaptive_R
    R = diag(cfg.fixed_sig.^2);
    return;
end
snr = max(10^(det.snr_dB/10) - 1, 1);           % remove the noise contribution
res = [cfg.range_res, 2/cfg.Nrx, cfg.vel_res];
sig = sqrt((cfg.k_meas .* res).^2 / snr + cfg.floor_meas.^2);
% Error was fitted in sin(az); convert to az:  d(az) = d(sin az) / cos(az)
sig(2) = sig(2) / max(cos(det.az), 0.3);
R = diag(sig.^2);
end
