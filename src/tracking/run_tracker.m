function hist_store = run_tracker(dets_all, cfg)
%RUN_TRACKER  Run the tracker over a whole scenario's detections.
%   hist_store{id} is the 22 x n history of track 'id' (see tracker_step),
%   kept even after the track is deleted.
tracks = [];
next_id = 1;
hist_store = {};
for f = 1:numel(dets_all)
    t = (f-1) * cfg.dt;
    [tracks, next_id] = tracker_step(tracks, dets_all{f}, cfg, next_id, t);
    for i = 1:numel(tracks)
        hist_store{tracks(i).id} = tracks(i).hist; %#ok<AGROW>
    end
end
end
