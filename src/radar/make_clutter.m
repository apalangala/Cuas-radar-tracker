function clut = make_clutter(p)
%MAKE_CLUTTER  Fixed field of discrete ground-clutter scatterers.
%   Trees, poles and buildings: stationary, much larger RCS than a drone,
%   spread uniformly over the surveillance area (range 20-480 m, +/-60 deg).
%   Generated once per scenario; each CPI adds a small random internal
%   motion (wind) in simulate_frame.
%
%   clut.range, clut.az, clut.rcs are 1 x p.n_clutter row vectors.

if ~p.clutter || p.n_clutter == 0
    clut = struct('range', [], 'az', [], 'rcs', []);
    return;
end
N = p.n_clutter;
% Uniform over area: range density proportional to R
rmin = 20; rmax = 480;
clut.range = sqrt(rmin^2 + (rmax^2 - rmin^2) * rand(1, N));
clut.az    = (rand(1, N) - 0.5) * (2*pi/3);                 % +/- 60 deg
rcs_dB     = p.clutter_rcs_median_dBsm + p.clutter_rcs_sigma_dB * randn(1, N);
clut.rcs   = 10.^(rcs_dB/10);
end
