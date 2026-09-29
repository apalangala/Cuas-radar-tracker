function test_cfar_false_alarm_rate()
%TEST_CFAR_FALSE_ALARM_RATE  Measured Pfa on noise-only maps matches the design.
%   Noise-only cells that sum 8 channels are Gamma(8)-distributed.  With the
%   corrected threshold the measured Pfa should be close to the design value
%   (a loose factor-of-2 band, since CA estimation adds a small loss).
set_seed(1);
Pfa = 1e-3; n_int = 8;
Nr = 256; Nd = 128; Nmaps = 20;
fa = 0; cells = 0;
for m = 1:Nmaps
    P = zeros(Nr, Nd);
    for c = 1:n_int
        P = P + (-log(rand(Nr, Nd)));         % exponential(1) per channel
    end
    mask = cfar2d(P, [2 2], [8 4], Pfa, n_int);
    inner = mask(11:end-10, :);                % ignore range edges
    fa = fa + nnz(inner); cells = cells + numel(inner);
end
measured = fa / cells;
assert(measured > Pfa/2 && measured < Pfa*2, ...
       sprintf('measured Pfa %.2e vs design %.1e', measured, Pfa));
end
