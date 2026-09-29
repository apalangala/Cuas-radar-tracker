function set_seed(s)
%SET_SEED  Seed the random number generators (MATLAB and GNU Octave).
if exist('OCTAVE_VERSION', 'builtin')
    rand('seed', s); randn('seed', s);   %#ok<RAND>
    rand('state', s); randn('state', s); %#ok<RAND>
else
    rng(s, 'twister');
end
end
