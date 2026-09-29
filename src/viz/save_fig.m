function save_fig(f, name)
%SAVE_FIG  Save a figure as PNG into docs/img (used by the README).
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
out = fullfile(root, 'docs', 'img');
if ~exist(out, 'dir'), mkdir(out); end
set(f, 'PaperPositionMode', 'auto');
print(f, fullfile(out, [name '.png']), '-dpng', '-r150');
end
