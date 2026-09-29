function [z, H] = meas_model(x)
%MEAS_MODEL  Radar measurement of a Cartesian state, and its Jacobian.
%   x = [px; vx; py; vy]            (m, m/s)
%   z = [range; azimuth; radial velocity]
%   Azimuth is measured from +y (boresight) toward +x.
%   H = dz/dx (3 x 4) is what makes this an *Extended* Kalman filter.
px = x(1); vx = x(2); py = x(3); vy = x(4);
r2 = px^2 + py^2;
r  = sqrt(r2);
rr = px*vx + py*vy;              % r * rdot
z  = [r; atan2(px, py); rr / r];
H = [ px/r,                    0,     py/r,                    0;
      py/r2,                   0,    -px/r2,                   0;
      vx/r - px*rr/r^3,     px/r,     vy/r - py*rr/r^3,     py/r ];
end
