function cube = simulate_frame(p, targets, clut)
%SIMULATE_FRAME  De-chirped (beat) signal data cube for one CPI.
%   targets: struct array with fields range (m), vr (m/s, + = receding),
%            az (rad, + toward +x), rcs (m^2)
%   clut:    output of make_clutter (optional)
%   cube:    Ns x Nc x Nrx complex baseband samples, unit noise power per sample
%
%   Signal model for scatterer i, sample n, chirp m, element k:
%     s = a_i * exp(j*2*pi*( fb_i*n/fs + 2*vr_i*m*Tpri/lambda + d*k*sin(az_i)/lambda ))
%   fb_i = 2*S*R_i/c is the beat frequency and |a_i|^2 is the per-sample SNR
%   from the radar equation.  With p.swerling = 1 the RCS is redrawn from an
%   exponential distribution every CPI (Swerling I fluctuation).

if nargin < 3, clut = []; end

n = (0:p.Ns-1).';                 % fast-time index   (column)
m = 0:p.Nc-1;                     % slow-time index   (row)
k = reshape(0:p.Nrx-1, 1, 1, []); % element index     (3rd dim)

cube = (randn(p.Ns, p.Nc, p.Nrx) + 1j*randn(p.Ns, p.Nc, p.Nrx)) / sqrt(2);

%  Targets 
for i = 1:numel(targets)
    R = targets(i).range;
    if R <= 0 || R >= p.range_max, continue; end
    rcs = targets(i).rcs;
    if p.swerling == 1, rcs = rcs * (-log(rand)); end      % exponential, mean = rcs
    a   = sqrt(p.K_snr * rcs / R^4) * exp(1j*2*pi*rand);
    fb  = 2 * p.S * R / p.c;
    ph_fast = exp(1j*2*pi*fb*n/p.fs);                              % Ns x 1
    ph_slow = exp(1j*4*pi*targets(i).vr*m*p.Tpri/p.lambda);        % 1 x Nc
    ph_elem = exp(1j*2*pi*p.d*k*sin(targets(i).az)/p.lambda);      % 1x1xNrx
    cube = cube + a * bsxfun(@times, ph_fast*ph_slow, ph_elem);
end

%  Clutter: hundreds of scatterers, done as one matrix product 
if ~isempty(clut) && ~isempty(clut.range)
    Q  = numel(clut.range);
    vr = p.clutter_vel_std * randn(1, Q);                          % wind this CPI
    a  = sqrt(p.K_snr * clut.rcs ./ clut.range.^4) .* exp(1j*2*pi*rand(1, Q));
    A  = exp(1j*2*pi*(2*p.S*n/p.c)*clut.range/p.fs);               % Ns x Q
    Bs = exp(1j*4*pi*(vr.')*m*p.Tpri/p.lambda);                    % Q x Nc
    Be = exp(1j*2*pi*p.d*(sin(clut.az).')*(0:p.Nrx-1)/p.lambda);   % Q x Nrx
    % B(q, m, k) = a_q * Bs(q,m) * Be(q,k), flattened to Q x (Nc*Nrx)
    B  = bsxfun(@times, reshape(bsxfun(@times, a.', Bs), Q, p.Nc, 1), ...
                reshape(Be, Q, 1, p.Nrx));
    cube = cube + reshape(A * reshape(B, Q, p.Nc*p.Nrx), p.Ns, p.Nc, p.Nrx);
end
end
