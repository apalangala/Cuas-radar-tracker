function [tracks, next_id] = tracker_step(tracks, dets, cfg, next_id, t)
%TRACKER_STEP  One scan of the multi-target EKF tracker.
%   State  x = [px; vx; py; vy]  (Cartesian, constant-velocity model)
%   Meas   z = [range; azimuth; radial velocity]  (polar -> nonlinear -> EKF)
%
%   1. Predict every track to time t
%   2. Gate: Mahalanobis distance between each track and each detection
%   3. Assign: greedy global nearest neighbour inside the gate
%   4. Update assigned tracks (Joseph-form covariance update)
%   5. Manage: M-of-N confirmation, deletion after consecutive misses,
%      with optional Doppler-notch-aware coasting
%   6. Start tentative tracks on unassigned detections
%
%   Each track stores its history (time, state, covariance, status) so it can
%   be scored against truth afterwards, including filter consistency (NEES).

dt = cfg.dt;
F  = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
q  = cfg.q_accel^2;
Q1 = q * [dt^4/4 dt^3/2; dt^3/2 dt^2];
Q  = blkdiag(Q1, Q1);

nT = numel(tracks); nD = numel(dets);
Rj = cell(1, nD);
Z  = zeros(3, nD);
for j = 1:nD
    Rj{j} = meas_noise(dets(j), cfg);
    Z(:, j) = [dets(j).range; dets(j).az; dets(j).vr];
end

%  1. Predict 
for i = 1:nT
    tracks(i).x = F * tracks(i).x;
    tracks(i).P = F * tracks(i).P * F.' + Q;
end

%  2. Gate 
D2 = inf(nT, nD);
for i = 1:nT
    [zhat, H] = meas_model(tracks(i).x);
    HPH = H * tracks(i).P * H.';
    for j = 1:nD
        nu = innovation(Z(:, j), zhat);
        d2 = nu.' * ((HPH + Rj{j}) \ nu);
        if d2 < cfg.gate, D2(i, j) = d2; end
    end
end

%  3. Assign (greedy GNN) 
assigned = zeros(1, nT);
used = false(1, nD);
while true
    [mn, idx] = min(D2(:));
    if isempty(mn) || ~isfinite(mn), break; end
    [i, j] = ind2sub(size(D2), idx);
    assigned(i) = j; used(j) = true;
    D2(i, :) = inf; D2(:, j) = inf;
end

%  4/5. Update and manage 
I4 = eye(4);
for i = 1:nT
    j = assigned(i);
    if j > 0
        [zhat, H] = meas_model(tracks(i).x);
        S = H * tracks(i).P * H.' + Rj{j};
        K = tracks(i).P * H.' / S;
        tracks(i).x = tracks(i).x + K * innovation(Z(:, j), zhat);
        IKH = I4 - K*H;
        tracks(i).P = IKH * tracks(i).P * IKH.' + K * Rj{j} * K.';
        tracks(i).hits = [tracks(i).hits(2:end) 1];
        tracks(i).misses = 0;
        tracks(i).notch_coast = 0;
    else
        tracks(i).hits = [tracks(i).hits(2:end) 0];
        in_notch = false;
        if cfg.notch_aware && tracks(i).confirmed
            % Treat as "in the notch" if the predicted radial velocity is
            % within the notch plus 2 sigma of the track's own uncertainty.
            [zhat, H] = meas_model(tracks(i).x);
            sig_vr = sqrt(H(3,:) * tracks(i).P * H(3,:).');
            in_notch = abs(zhat(3)) < cfg.notch_vr + 2*sig_vr && ...
                       tracks(i).notch_coast < cfg.max_notch_coast;
        end
        if in_notch
            tracks(i).notch_coast = tracks(i).notch_coast + 1;   % expected miss
        else
            tracks(i).misses = tracks(i).misses + 1;
        end
    end
    if ~tracks(i).confirmed && sum(tracks(i).hits) >= cfg.confirm_M
        tracks(i).confirmed = true;
    end
    tracks(i).hist(:, end+1) = pack_hist(t, tracks(i));
end

keep = true(1, nT);
for i = 1:nT
    if tracks(i).confirmed
        keep(i) = tracks(i).misses < cfg.delete_confirmed;
    else
        keep(i) = tracks(i).misses < cfg.delete_tentative;
    end
end
tracks = tracks(keep);

%  6. Initiate 
for j = find(~used)
    r = Z(1, j); az = Z(2, j); vr = Z(3, j);
    sig = sqrt(diag(Rj{j}));
    u  = [sin(az); cos(az)];       % line of sight (x, y)
    ut = [cos(az); -sin(az)];      % tangential direction
    Cp = sig(1)^2 * (u*u.') + (r*sig(2))^2 * (ut*ut.');
    Cv = sig(3)^2 * (u*u.') + cfg.init_vt_sigma^2 * (ut*ut.');
    trk.id = next_id; next_id = next_id + 1;
    trk.x = [r*u(1); vr*u(1); r*u(2); vr*u(2)];   % only radial velocity observed
    trk.P = zeros(4);
    trk.P([1 3], [1 3]) = Cp;
    trk.P([2 4], [2 4]) = Cv;
    trk.hits = [zeros(1, cfg.confirm_N - 1) 1];
    trk.misses = 0;
    trk.notch_coast = 0;
    trk.confirmed = false;
    trk.hist = pack_hist(t, trk);
    if isempty(tracks), tracks = trk; else, tracks(end+1) = trk; end %#ok<AGROW>
end
end

function h = pack_hist(t, trk)
% Column layout: [t; x(4); confirmed; P(:) (16)]  -> 22 rows
h = [t; trk.x; double(trk.confirmed); trk.P(:)];
end

function nu = innovation(z, zhat)
nu = z - zhat;
nu(2) = atan2(sin(nu(2)), cos(nu(2)));   % wrap azimuth error
end
