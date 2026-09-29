function p = radar_params()
%RADAR_PARAMS  X-band FMCW radar parameters for a short-range counter-UAS sensor.
%   Every derived quantity (resolution, unambiguous range/velocity) is computed
%   here, so changing one design parameter propagates through the whole chain.
%
%   See docs/DESIGN.md for the reasoning behind each choice.

p.c      = 3e8;          % speed of light (m/s)
p.fc     = 10e9;         % carrier frequency, X-band (Hz)
p.lambda = p.c / p.fc;   % wavelength (m)

%  Chirp (fast time) 
p.B_sweep = 150e6;       % RF sweep bandwidth over the full ramp (Hz)
p.Tchirp  = 50e-6;       % ramp duration (s)
p.Tidle   = 70e-6;       % idle time between chirps (s) -> PRI 120 us
p.Tpri    = p.Tchirp + p.Tidle;       % chirp repetition interval (s)
p.S       = p.B_sweep / p.Tchirp;     % chirp slope (Hz/s)
p.fs      = 10e6;        % complex (IQ) ADC sample rate (Hz)
p.Ns      = 256;         % samples per chirp  -> range bins

%  Slow time / array 
p.Nc      = 128;         % chirps per CPI -> Doppler bins (CPI = 15.4 ms)
p.Nrx     = 8;           % receive elements, uniform linear array
p.d       = p.lambda/2;  % element spacing (m)

%  Link budget (radar equation) 
p.Pt      = 0.5;         % transmit power (W)
p.Gt_dB   = 20;          % transmit antenna gain (dBi)
p.Gr_dB   = 12;          % per-element receive gain (dBi)
p.NF_dB   = 6;           % receiver noise figure (dB)
p.L_dB    = 4;           % system losses (dB)
p.T0      = 290;         % reference temperature (K)
p.kB      = 1.380649e-23;% Boltzmann constant (J/K)
p.swerling = 1;          % 0 = constant RCS, 1 = RCS fluctuates CPI-to-CPI (Swerling I)

%  Signal processing 
p.cfar_guard = [2 2];    % guard cells  [range doppler] each side
p.cfar_train = [8 4];    % training cells [range doppler] each side
p.Pfa        = 1e-5;     % design false-alarm probability per cell
p.min_range  = 15;       % ignore detections closer than this (m)
p.mti        = true;     % static-clutter canceller (remove slow-time mean)
p.notch_bins = 1;        % Doppler bins either side of zero blanked when MTI is on

%  Environment 
p.clutter    = true;     % include static ground clutter (trees, poles, buildings)
p.n_clutter  = 400;      % number of discrete clutter scatterers
p.clutter_rcs_median_dBsm = 0;   % median clutter RCS (dBsm), log-normal
p.clutter_rcs_sigma_dB    = 6;   % log-normal spread (dB)
p.clutter_vel_std = 0.1; % internal motion (wind-blown foliage), m/s

%  Frame timing 
p.dt       = 0.1;        % time between CPIs = tracker update interval (s)

%  Derived 
p.B_eff     = p.S * p.Ns / p.fs;           % bandwidth actually sampled (Hz)
p.range_res = p.c / (2*p.B_eff);           % range resolution (m)
p.range_max = p.fs * p.c / (2*p.S);        % max unambiguous range, IQ sampling (m)
p.vel_res   = p.lambda / (2*p.Nc*p.Tpri);  % velocity resolution (m/s)
p.vel_max   = p.lambda / (4*p.Tpri);       % max unambiguous radial speed (m/s)
p.range_axis = (0:p.Ns-1) * p.range_res;           % m
p.vel_axis   = (-p.Nc/2:p.Nc/2-1) * p.vel_res;     % m/s, after fftshift
p.notch_vr   = (p.notch_bins + 0.5) * p.vel_res;   % half-width of MTI blind zone (m/s)

% Radar-equation constant: per-sample SNR = K * rcs / R^4
Gt = 10^(p.Gt_dB/10); Gr = 10^(p.Gr_dB/10);
NF = 10^(p.NF_dB/10); L  = 10^(p.L_dB/10);
p.K_snr = p.Pt*Gt*Gr*p.lambda^2 / ((4*pi)^3 * p.kB*p.T0*p.fs*NF*L);
p.proc_gain_dB = 10*log10(p.Ns*p.Nc*p.Nrx);        % ideal coherent integration gain
end
