function tg = truth_to_radar(truth, f)
%TRUTH_TO_RADAR  Convert Cartesian truth at frame f into radar coordinates.
%   tg(k): range (m), vr (m/s, + = receding), az (rad from boresight), rcs.
tg = struct('range', {}, 'vr', {}, 'az', {}, 'rcs', {});
for k = 1:numel(truth)
    pos = truth(k).pos(:, f);
    vel = truth(k).vel(:, f);
    r = norm(pos);
    tg(k).range = r;
    tg(k).vr    = dot(pos, vel) / r;
    tg(k).az    = atan2(pos(1), pos(2));
    tg(k).rcs   = truth(k).rcs;
end
end
