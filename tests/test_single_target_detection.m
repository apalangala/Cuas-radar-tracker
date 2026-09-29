function test_single_target_detection()
%TEST_SINGLE_TARGET_DETECTION  One strong target is detected at the right place.
set_seed(2);
p = radar_params();
p.clutter = false; p.swerling = 0;
tg.range = 203.7; tg.vr = 12.3; tg.az = 20*pi/180; tg.rcs = 0.1;
dets = process_frame(p, simulate_frame(p, tg));
assert(~isempty(dets), 'target not detected');
[~, j] = min(abs([dets.range] - tg.range));
d = dets(j);
assert(abs(d.range - tg.range) < 0.3, sprintf('range error %.2f m', d.range - tg.range));
assert(abs(d.vr - tg.vr) < 0.2, sprintf('velocity error %.2f m/s', d.vr - tg.vr));
assert(abs(d.az - tg.az)*180/pi < 0.5, sprintf('azimuth error %.2f deg', (d.az - tg.az)*180/pi));
end
