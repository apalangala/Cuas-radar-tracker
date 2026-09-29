function sim = simulate_detections(p, truth, Nframes, seed, snap_frames)
%SIMULATE_DETECTIONS  Run the radar front end for a whole scenario.
%   Generates the raw data cube for every frame and runs the signal
%   processing chain.  The tracker is run separately (run_tracker), so that
%   several tracker designs can be compared on exactly the same detections.
%
%   sim.dets{f}      detections at frame f
%   sim.tg{f}        truth in radar coordinates at frame f
%   sim.n_true(f)    detections that belong to a real target
%   sim.n_false(f)   false alarms (noise or clutter)
%   sim.snap{i}      range-Doppler maps for the frames listed in snap_frames

if nargin < 5, snap_frames = []; end
set_seed(seed);
clut = make_clutter(p);

sim.dets = cell(1, Nframes);
sim.tg   = cell(1, Nframes);
sim.n_true  = zeros(1, Nframes);
sim.n_false = zeros(1, Nframes);
sim.snap = {};
sim.snap_frames = snap_frames;
sim.clutter = clut;

for f = 1:Nframes
    tg = truth_to_radar(truth, f);
    cube = simulate_frame(p, tg, clut);
    if any(snap_frames == f)
        [dets, rd] = process_frame(p, cube);
        rd.frame = f; rd.tg = tg; rd.dets = dets;
        sim.snap{end+1} = rd;
    else
        dets = process_frame(p, cube);
    end
    sim.dets{f} = dets;
    sim.tg{f} = tg;
    for j = 1:numel(dets)
        if is_target_detection(p, dets(j), tg)
            sim.n_true(f) = sim.n_true(f) + 1;
        else
            sim.n_false(f) = sim.n_false(f) + 1;
        end
    end
end
end

function hit = is_target_detection(p, det, tg)
hit = false;
for k = 1:numel(tg)
    if abs(det.range - tg(k).range) < 3*p.range_res && abs(det.vr - tg(k).vr) < 3*p.vel_res
        hit = true; return;
    end
end
end
