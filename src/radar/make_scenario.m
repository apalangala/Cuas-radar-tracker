function truth = make_scenario(p, Nframes)
%MAKE_SCENARIO  Ground-truth trajectories for three small UAVs.
%   Radar at the origin; y = down-range (boresight), x = cross-range.
%   truth(k).pos / .vel are 2 x Nframes, in m and m/s.
%
%   T1  quadcopter: small (0.02 m^2, DJI-Mini class), slow, inbound, far away
%   T2  fixed-wing: flies across the beam, so its radial velocity passes
%       through zero and it drops into the MTI blind zone for ~1.5 s
%   T3  fixed-wing: straight leg, then a 3 m/s^2 coordinated turn

dt = p.dt;
t  = (0:Nframes-1) * dt;

spec(1).name = 'T1 quadcopter (inbound)';
spec(1).x0 = [-60; 420];  spec(1).v0 = [5; -15];  spec(1).rcs = 0.02;
spec(1).turn = [0 0 0];                       % [omega (rad/s), t_start, t_end]

spec(2).name = 'T2 fixed-wing (crossing)';
spec(2).x0 = [-150; 250]; spec(2).v0 = [25; 2];   spec(2).rcs = 0.10;
spec(2).turn = [0 0 0];

spec(3).name = 'T3 fixed-wing (turning)';
spec(3).x0 = [120; 150];  spec(3).v0 = [0; 20];   spec(3).rcs = 0.05;
spec(3).turn = [0.15 2.0 5.0];                % left turn between 2 s and 5 s

for k = 1:numel(spec)
    pos = zeros(2, Nframes); vel = zeros(2, Nframes);
    pos(:,1) = spec(k).x0; vel(:,1) = spec(k).v0;
    for n = 2:Nframes
        w = 0;
        if t(n) > spec(k).turn(2) && t(n) <= spec(k).turn(3)
            w = spec(k).turn(1);
        end
        R = [cos(w*dt) -sin(w*dt); sin(w*dt) cos(w*dt)];   % rotate velocity (CCW = left)
        vel(:,n) = R * vel(:,n-1);
        pos(:,n) = pos(:,n-1) + 0.5*(vel(:,n-1) + vel(:,n))*dt;
    end
    truth(k).name = spec(k).name;
    truth(k).short = spec(k).name(1:2);
    truth(k).rcs  = spec(k).rcs;
    truth(k).pos  = pos;
    truth(k).vel  = vel;
end
end
