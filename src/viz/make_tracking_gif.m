function make_tracking_gif(p, truth, sim, hist_store)
%MAKE_TRACKING_GIF  Animated plan view of the tracker (docs/img/tracking.gif).
st = plot_style();
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
fname = fullfile(root, 'docs', 'img', 'tracking.gif');
N = numel(sim.dets);
f = figure('Position', [100 100 640 560], 'Color', 'w');
first = true;
for fr = 1:2:N
    clf; hold on; box on; grid on;
    t = (fr-1) * p.dt;
    % clutter + FOV
    a = linspace(-pi/3, pi/3, 60);
    for R = 100:100:500, plot(R*sin(a), R*cos(a), '-', 'Color', [0.88 0.88 0.88]); end
    if ~isempty(sim.clutter.range)
        plot(sim.clutter.range.*sin(sim.clutter.az), sim.clutter.range.*cos(sim.clutter.az), ...
             '^', 'Color', [0.86 0.80 0.70], 'MarkerSize', 2, 'MarkerFaceColor', [0.86 0.80 0.70]);
    end
    plot(0, 0, 'k^', 'MarkerFaceColor', 'k', 'MarkerSize', 9);
    % truth trails
    for k = 1:numel(truth)
        plot(truth(k).pos(1,1:fr), truth(k).pos(2,1:fr), '-', 'Color', st.c(k,:), 'LineWidth', 2.5);
    end
    % current detections
    d = sim.dets{fr};
    if ~isempty(d)
        plot([d.range].*sin([d.az]), [d.range].*cos([d.az]), 'x', 'Color', [0.8 0 0], ...
             'MarkerSize', 8, 'LineWidth', 1.5);
    end
    % confirmed tracks up to now, with 3-sigma ellipse
    for id = 1:numel(hist_store)
        h = hist_store{id};
        if isempty(h), continue; end
        c = find(h(6,:) == 1 & h(1,:) <= t + 1e-9);
        if isempty(c) || abs(h(1, c(end)) - t) > 1e-9, continue; end
        plot(h(2,c), h(4,c), 'k--', 'LineWidth', 1.1);
        P = reshape(h(7:22, c(end)), 4, 4); C = P([1 3], [1 3]);
        [V, E] = eig((C + C.')/2);
        q = V * sqrt(max(E, 0)) * [cos(a*3); sin(a*3)] * 3;
        plot(h(2,c(end)) + q(1,:), h(4,c(end)) + q(2,:), 'k-');
        text(h(2,c(end)) + 8, h(4,c(end)), sprintf('#%d', id), 'FontSize', 9);
    end
    axis equal; xlim([-260 260]); ylim([-10 470]);
    xlabel('x (m)'); ylabel('y (m)');
    title(sprintf('Counter-UAS radar tracker   t = %.1f s', t));
    drawnow;
    fr_img = getframe(f);
    [im, map] = to_indexed(frame2im(fr_img));
    if first
        imwrite(im, map, fname, 'gif', 'LoopCount', Inf, 'DelayTime', 0.12);
        first = false;
    else
        imwrite(im, map, fname, 'gif', 'WriteMode', 'append', 'DelayTime', 0.12);
    end
end
close(f);
end

function [im, map] = to_indexed(rgb)
% GIF needs an indexed image.  MATLAB: minimum-variance quantisation.
% Octave's rgb2ind has no colour-count option, so use a uniform 6x6x6 cube.
if ~exist('OCTAVE_VERSION', 'builtin')
    [im, map] = rgb2ind(rgb, 128, 'nodither');
    return;
end
q = round(double(rgb) / 255 * 5);                  % 0..5 per channel
im = uint8(q(:,:,1)*36 + q(:,:,2)*6 + q(:,:,3));
[r, g, b] = ndgrid(0:5, 0:5, 0:5);
map = [reshape(permute(r, [3 2 1]), [], 1), reshape(permute(g, [3 2 1]), [], 1), ...
       reshape(permute(b, [3 2 1]), [], 1)] / 5;
end
