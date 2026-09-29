function [det_mask, threshold, noise_est] = cfar2d(P, guard, train, Pfa, n_int)
%CFAR2D  Two-dimensional cell-averaging CFAR on a range-Doppler power map.
%   P      Nr x Nd power map (linear); Doppler along dim 2 (circular)
%   guard  [gr gd] guard cells each side;  train [tr td] training cells beyond guard
%   Pfa    design probability of false alarm per cell
%   n_int  number of channels summed non-coherently into P (1 if none)
%
%   The window is applied to the whole map at once with conv2.  The Doppler
%   axis wraps (it is periodic); at the range edges the noise estimate uses
%   only the training cells that exist.
%
%   Threshold: under noise only, a cell that sums n_int channels is
%   Gamma(n_int)-distributed, NOT exponential.  Using the textbook
%   single-channel formula here would put the real Pfa near 1e-30 and throw
%   away ~10 dB of sensitivity.  So we solve
%       P( Gamma(n_int, 1) > alpha * n_int ) = Pfa
%   for alpha.  (With ~250 training cells the CA estimation loss is < 0.2 dB
%   and is ignored.)

if nargin < 5, n_int = 1; end
gr = guard(1); gd = guard(2);
tr = train(1); td = train(2);

wr = gr + tr; wd = gd + td;
kern = ones(2*wr+1, 2*wd+1);
kern(wr+1-gr:wr+1+gr, wd+1-gd:wd+1+gd) = 0;   % remove guard band + cell under test

Pp   = [P(:, end-wd+1:end), P, P(:, 1:wd)];    % circular pad in Doppler
sumN = conv2(Pp, kern, 'same');
cntN = conv2(ones(size(Pp)), kern, 'same');
noise_est = sumN(:, wd+1:end-wd) ./ cntN(:, wd+1:end-wd);

alpha = cfar_alpha(Pfa, n_int);
threshold = alpha .* noise_est;
det_mask  = P > threshold;
end

function alpha = cfar_alpha(Pfa, n)
f = @(a) log(gammainc(a*n, n, 'upper')) - log(Pfa);
alpha = fzero(f, [1+1e-6, 60]);
end
