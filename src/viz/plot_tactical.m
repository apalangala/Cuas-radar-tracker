function plot_tactical(p, truth, sim, hist_store)
%PLOT_TACTICAL  Plan view: clutter field, detections, truth and confirmed tracks.
st = plot_style();
f = figure('Position', [100 100 900 760]); hold on; box on; grid on;

draw_background(p, sim.clutter);

% All detections over the scenario
D = zeros(2, 0);
for fr = 1:numel(sim.dets)
    d = sim.dets{fr};
    if isempty(d), continue; end
    D = [D, [[d.range].*sin([d.az]); [d.range].*cos([d.az])]]; %#ok<AGROW>
end
hD = plot(D(1,:), D(2,:), '.', 'Color', st.det, 'MarkerSize', 5);

hT = zeros(1, numel(truth));
for k = 1:numel(truth)
    hT(k) = plot(truth(k).pos(1,:), truth(k).pos(2,:), '-', 'Color', st.c(k,:), 'LineWidth', 3);
    plot(truth(k).pos(1,1), truth(k).pos(2,1), 'o', 'Color', st.c(k,:), ...
         'MarkerFaceColor', st.c(k,:), 'MarkerSize', 7);
end

hTrk = [];
for id = 1:numel(hist_store)
    h = hist_store{id};
    if isempty(h) || ~any(h(6,:) == 1), continue; end
    c = find(h(6,:) == 1);
    hTrk = plot(h(2,c), h(4,c), '--', 'Color', st.trk, 'LineWidth', 1.3);
    last = c(end);
    P = reshape(h(7:22, last), 4, 4);
    draw_ellipse(h([2 4], last), P([1 3], [1 3]), 3, st.trk);
    text(h(2,last) + 8, h(4,last) + 4, sprintf('Track %d', id), 'FontSize', 9);
end

axis equal; xlim([-260 260]); ylim([-10 470]);
xlabel('Cross-range x (m)'); ylabel('Down-range y (m)');
title('Tracks vs ground truth (8 s, clutter + MTI, Swerling I targets)');
hC = plot(nan, nan, '^', 'Color', [0.84 0.78 0.66], 'MarkerFaceColor', [0.84 0.78 0.66], 'MarkerSize', 4);
hh = [hT, hD, hC]; lg = [{truth.name}, {'Detections', 'Clutter (trees, poles, buildings)'}];
if ~isempty(hTrk), hh = [hh hTrk]; lg{end+1} = 'Confirmed tracks (3\sigma at end)'; end
legend(hh, lg, 'Location', 'southwest');
save_fig(f, 'tactical_picture');
end

function draw_background(p, clut)
% Field of view, range rings, radar and clutter scatterers
for R = 100:100:500
    a = linspace(-pi/3, pi/3, 100);
    plot(R*sin(a), R*cos(a), '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 0.6);
    text(R*sin(pi/3) + 4, R*cos(pi/3), sprintf('%d m', R), 'Color', [0.6 0.6 0.6], 'FontSize', 8);
end
plot([0 480*sin(-pi/3)], [0 480*cos(-pi/3)], ':', 'Color', [0.6 0.6 0.6]);
plot([0 480*sin(pi/3)],  [0 480*cos(pi/3)],  ':', 'Color', [0.6 0.6 0.6]);
if ~isempty(clut.range)
    plot(clut.range.*sin(clut.az), clut.range.*cos(clut.az), '^', 'Color', [0.84 0.78 0.66], ...
         'MarkerSize', 2.5, 'MarkerFaceColor', [0.84 0.78 0.66]);
end
plot(0, 0, 'k^', 'MarkerFaceColor', 'k', 'MarkerSize', 11);
text(10, 10, 'Radar', 'FontWeight', 'bold');
end

function draw_ellipse(mu, C, nsig, col)
[V, E] = eig((C + C.')/2);
a = linspace(0, 2*pi, 60);
xy = V * sqrt(max(E, 0)) * [cos(a); sin(a)] * nsig;
plot(mu(1) + xy(1,:), mu(2) + xy(2,:), '-', 'Color', col, 'LineWidth', 1);
end
