function cfg = tracker_config(p, variant)
%TRACKER_CONFIG  Tracker settings.  Three variants are compared in the study:
%
%   'baseline'  fixed measurement noise, standard track deletion
%   'adaptive'  + measurement noise scaled by each detection's SNR
%   'full'      + Doppler-notch-aware coasting (don't delete a track just
%                 because it has flown into the MTI blind zone)
%
%   Measurement-noise constants come from run_calibration.m.

if nargin < 2, variant = 'full'; end

cfg.variant = variant;
cfg.dt = p.dt;
cfg.range_res = p.range_res; cfg.vel_res = p.vel_res; cfg.Nrx = p.Nrx;

% Motion model: constant velocity + white-noise acceleration
cfg.q_accel = 3;                 % expected manoeuvre acceleration (m/s^2)

% ---- Measurement noise ----
cfg.adaptive_R = ~strcmp(variant, 'baseline');
% Fixed values (baseline): a typical "reasonable guess" at one SNR
cfg.fixed_sig = [0.5, 1.0*pi/180, 0.5];      % [range (m), az (rad), vr (m/s)]
% SNR-adaptive model: sigma^2 = (k*res/sqrt(SNR))^2 + floor^2
% (k, floor fitted by run_calibration.m; res = range bin / beamwidth in
%  sin-space (2/Nrx) / Doppler bin)
cfg.k_meas     = [0.202, 0.242, 0.206];
cfg.floor_meas = [0.021, 0.0005, 0.15];      % [m, sin(az), m/s]
% NOTE on the radial-velocity floor: calibration on a point target gives
% ~0.01 m/s, but a real drone is not a point.  Rotor and body micro-Doppler
% spread its return by tenths of a m/s, and an EKF fed an over-precise
% Doppler measurement becomes over-confident (NEES >> 4) through the
% nonlinear range-rate/position coupling.  0.15 m/s is a deliberate margin.

% ---- Data association ----
cfg.gate = 16.27;                % chi-square gate, 3 dof, 99.9 %

% ---- Track management ----
cfg.confirm_M = 3; cfg.confirm_N = 5;        % confirm after 3 hits in 5 scans
cfg.delete_tentative = 2;        % consecutive misses before a tentative track dies
cfg.delete_confirmed = 5;        % ... and a confirmed track
cfg.init_vt_sigma = 30;          % initial tangential-velocity uncertainty (m/s)

% ---- Doppler-notch-aware coasting ----
% With MTI on, a target whose radial speed is inside the notch CANNOT be
% detected.  A miss there is expected, so it is not counted towards deletion
% (up to max_notch_coast scans).
cfg.notch_aware     = strcmp(variant, 'full') && p.mti;
cfg.notch_vr        = p.notch_vr;
cfg.max_notch_coast = 40;        % 4 s
end
