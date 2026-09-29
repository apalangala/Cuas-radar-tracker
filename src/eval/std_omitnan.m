function s = std_omitnan(x)
%STD_OMITNAN  Standard deviation ignoring NaNs.
x = x(~isnan(x));
s = std(x);
end
