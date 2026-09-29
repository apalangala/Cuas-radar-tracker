function m = mean_omitnan(x)
%MEAN_OMITNAN  Mean ignoring NaNs (works in any MATLAB version and Octave).
x = x(~isnan(x));
m = mean(x);
end
