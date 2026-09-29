function [dets, rd] = process_frame(p, cube)
%PROCESS_FRAME  Signal processing chain for one CPI.
%   dets: struct array, one per detection, with fields
%         range (m), vr (m/s), az (rad), snr_dB (peak power / local noise)
%   rd:   struct with the range-Doppler map (dB above median noise) and the
%         CFAR mask, used for plotting.
%
%   Chain:
%     [MTI: subtract slow-time mean]  -> Hann windows -> range FFT -> Doppler FFT
%     -> non-coherent sum over array  -> 2-D CA-CFAR  -> [blank MTI notch]
%     -> local-maximum peak grouping  -> parabolic sub-bin interpolation
%     -> zero-padded angle FFT across the array at each detection

% ---- Static clutter canceller ----
% Anything that does not move has the same phase on every chirp, i.e. it is
% the slow-time mean.  Subtracting it removes stationary clutter.  The cost:
% targets with ~zero radial velocity (flying tangentially) vanish too.
if p.mti
    cube = bsxfun(@minus, cube, mean(cube, 2));
end

wr = hann_win(p.Ns);
wd = hann_win(p.Nc).';
X = bsxfun(@times, cube, wr * wd);
X = fft(X, [], 1);                          % range FFT
X = fftshift(fft(X, [], 2), 2);             % Doppler FFT, zero-centred

P = sum(abs(X).^2, 3);                      % non-coherent integration over array
noise_floor = median(P(:));

[mask, ~, noise_est] = cfar2d(P, p.cfar_guard, p.cfar_train, p.Pfa, p.Nrx);
mask(p.range_axis < p.min_range, :) = false;
if p.mti
    zero_bin = p.Nc/2 + 1;
    mask(:, zero_bin-p.notch_bins : zero_bin+p.notch_bins) = false;
end

% ---- Peak grouping: CFAR hits that are local maxima in a 3x3 neighbourhood ----
Pw = [P(:, end), P, P(:, 1)];
Pw = [zeros(1, size(Pw,2)); Pw; zeros(1, size(Pw,2))];
is_peak = true(size(P));
for dr = -1:1
    for dd = -1:1
        if dr == 0 && dd == 0, continue; end
        is_peak = is_peak & (P >= Pw((2:end-1)+dr, (2:end-1)+dd));
    end
end
[ri, di] = find(mask & is_peak);

dets = struct('range', {}, 'vr', {}, 'az', {}, 'snr_dB', {});
Nang = 256;
wa   = hann_win(p.Nrx);

for q = 1:numel(ri)
    r = ri(q); d = di(q);
    dr_off = 0;
    if r > 1 && r < p.Ns
        dr_off = parabolic_offset(P(r-1,d), P(r,d), P(r+1,d));
    end
    dm = mod(d-2, p.Nc) + 1; dp = mod(d, p.Nc) + 1;
    dd_off = parabolic_offset(P(r,dm), P(r,d), P(r,dp));

    snap = squeeze(X(r, d, :));
    A = abs(fftshift(fft(snap(:) .* wa, Nang))).^2;
    [~, ia] = max(A);
    da_off = 0;
    if ia > 1 && ia < Nang
        da_off = parabolic_offset(A(ia-1), A(ia), A(ia+1));
    end
    u = (ia - 1 - Nang/2 + da_off) / Nang;      % cycles per element
    sin_az = u * p.lambda / p.d;
    if abs(sin_az) >= 1, continue; end

    dets(end+1).range = (r - 1 + dr_off) * p.range_res; %#ok<AGROW>
    dets(end).vr      = (d - 1 - p.Nc/2 + dd_off) * p.vel_res;
    dets(end).az      = asin(sin_az);
    dets(end).snr_dB  = 10*log10(P(r,d) / noise_est(r,d));
end

if nargout > 1
    rd.map_dB = 10*log10(P / noise_floor);
    rd.mask   = mask;
end
end

function w = hann_win(N)
w = 0.5 - 0.5*cos(2*pi*(0:N-1).'/(N-1));
end

function off = parabolic_offset(ym, y0, yp)
% Peak of a parabola through three log-power samples (sub-bin estimate)
ym = log(ym + eps); y0 = log(y0 + eps); yp = log(yp + eps);
den = ym - 2*y0 + yp;
if abs(den) < eps, off = 0; else, off = 0.5*(ym - yp)/den; end
off = max(min(off, 0.5), -0.5);
end
