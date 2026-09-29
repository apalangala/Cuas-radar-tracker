function test_mti_removes_clutter()
%TEST_MTI_REMOVES_CLUTTER  Static clutter floods the detector without MTI,
%   and is almost entirely removed with it.
set_seed(3);
p = radar_params();
clut = make_clutter(p);
cube = simulate_frame(p, struct('range', {}, 'vr', {}, 'az', {}, 'rcs', {}), clut);
p.mti = false; n_off = numel(process_frame(p, cube));
p.mti = true;  n_on  = numel(process_frame(p, cube));
assert(n_off > 20, sprintf('expected clutter false alarms without MTI, got %d', n_off));
assert(n_on <= 3, sprintf('MTI left %d false alarms', n_on));
end
