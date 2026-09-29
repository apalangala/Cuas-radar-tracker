function test_radar_params()
%TEST_RADAR_PARAMS  Derived radar quantities match the textbook formulas.
p = radar_params();
assert(abs(p.range_res - p.c/(2*p.S*p.Ns/p.fs)) < 1e-9, 'range resolution != c/2B');
assert(abs(p.vel_max - p.lambda/(4*p.Tpri)) < 1e-9, 'max velocity != lambda/4T');
assert(abs(p.vel_res - 2*p.vel_max/p.Nc) < 1e-9, 'velocity resolution inconsistent');
assert(p.vel_max > 40, 'cannot measure a 40 m/s drone unambiguously');
assert(p.range_max >= 480, 'unambiguous range shorter than the surveillance area');
end
