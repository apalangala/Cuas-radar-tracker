function plot_rd_compare(p, rd_off, rd_on, tg, frame)
%PLOT_RD_COMPARE  Range-Doppler map of one frame, without and with MTI.
plot_style();
f = figure('Position', [100 100 1150 500]);
pos = {[0.06 0.12 0.37 0.70], [0.52 0.12 0.37 0.70]};
cmax = max(rd_off.map_dB(:));
panels = {rd_off, rd_on};
ttl = {'Without clutter canceller', 'With MTI (static clutter removed)'};
for i = 1:2
    axes('Position', pos{i});
    rd = panels{i};
    imagesc(p.vel_axis, p.range_axis, rd.map_dB); axis xy; hold on;
    caxis([0 min(cmax, 60)]);
    [ri, di] = find(rd.mask);
    plot(p.vel_axis(di), p.range_axis(ri), 'r.', 'MarkerSize', 5);
    for k = 1:numel(tg)
        plot(tg(k).vr, tg(k).range, 'wo', 'MarkerSize', 13, 'LineWidth', 1.6);
    end
    xlim([-45 45]); ylim([0 500]);
    xlabel('Radial velocity (m/s)'); ylabel('Range (m)');
    title({ttl{i}, sprintf('frame %d: %d CFAR cells (red), true targets circled', frame, nnz(rd.mask))});
end
if exist('OCTAVE_VERSION', 'builtin'), colormap(viridis); else, colormap(parula); end
cb = colorbar; ylabel(cb, 'dB above noise floor');
save_fig(f, 'range_doppler_mti');
end
