function res = evaluate_tracks(truth, hist_store, p, Nframes)
%EVALUATE_TRACKS  Score confirmed tracks against ground truth.
%   Each frame, every truth target is matched to the nearest confirmed track
%   within match_dist.  Reported per target k:
%     rms_pos(k), rms_vel(k)        RMS error over all matched frames
%     rms_pos_ss(k), rms_vel_ss(k)  "steady state": >= 1 s after first match
%     coverage(k)                   fraction of frames with a matched track
%     breaks(k)                     number of extra track IDs (track breaks)
%     nees{k}                       normalised estimation error squared per frame
%   and overall:
%     n_false_tracks                confirmed tracks that never matched truth
%
%   NEES = e' * P^-1 * e, with e = estimate - truth (4 states).  For a
%   consistent filter its average is 4.  Much larger = over-confident filter.

match_dist = 25;
nT = numel(truth);
dt = p.dt;

% Put every track's history on a frame grid: S(:, f, id), confirmed(f, id)
nid = numel(hist_store);
X = nan(4, Nframes, nid); Pc = nan(16, Nframes, nid);
conf = false(Nframes, nid);
for id = 1:nid
    h = hist_store{id};
    if isempty(h), continue; end
    fidx = round(h(1, :) / dt) + 1;
    X(:, fidx, id)  = h(2:5, :);
    conf(fidx, id)  = h(6, :) == 1;
    Pc(:, fidx, id) = h(7:22, :);
end

res.err_pos = cell(1, nT); res.err_vel = cell(1, nT);
res.err_t = cell(1, nT);   res.nees = cell(1, nT);
res.ids = cell(1, nT);
covered = zeros(1, nT);
matched_any = false(1, nid);

for f = 1:Nframes
    ids = find(conf(f, :));
    if isempty(ids), continue; end
    S = reshape(X(:, f, ids), 4, []);
    for k = 1:nT
        tp = truth(k).pos(:, f); tv = truth(k).vel(:, f);
        d = sqrt((S(1,:) - tp(1)).^2 + (S(3,:) - tp(2)).^2);
        [dmin, im] = min(d);
        if dmin < match_dist
            id = ids(im);
            e = S(:, im) - [tp(1); tv(1); tp(2); tv(2)];
            P = reshape(Pc(:, f, id), 4, 4);
            covered(k) = covered(k) + 1;
            res.err_pos{k}(end+1) = dmin;
            res.err_vel{k}(end+1) = norm(e([2 4]));
            res.err_t{k}(end+1) = (f-1)*dt;
            res.nees{k}(end+1) = e.' * (P \ e);
            if ~any(res.ids{k} == id), res.ids{k}(end+1) = id; end
            matched_any(id) = true;
        end
    end
end

ever_confirmed = any(conf, 1);
res.n_false_tracks = sum(ever_confirmed & ~matched_any);

for k = 1:nT
    tt = res.err_t{k};
    if isempty(tt)
        [res.rms_pos(k), res.rms_vel(k), res.rms_pos_ss(k), res.rms_vel_ss(k)] = deal(nan);
        res.breaks(k) = 0; res.mean_nees(k) = nan;
    else
        ss = tt >= tt(1) + 1.0;
        res.rms_pos(k)    = sqrt(mean(res.err_pos{k}.^2));
        res.rms_vel(k)    = sqrt(mean(res.err_vel{k}.^2));
        res.rms_pos_ss(k) = sqrt(mean(res.err_pos{k}(ss).^2));
        res.rms_vel_ss(k) = sqrt(mean(res.err_vel{k}(ss).^2));
        res.breaks(k)     = numel(res.ids{k}) - 1;
        res.mean_nees(k)  = mean(res.nees{k}(ss));
    end
    res.coverage(k) = covered(k) / Nframes;
end
end
