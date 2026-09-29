function st = plot_style()
%PLOT_STYLE  Shared colours and default figure styling for every plot.
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultTextFontSize', 11);
set(0, 'DefaultLineLineWidth', 1.4);
set(0, 'DefaultAxesLineWidth', 0.8);
set(0, 'DefaultFigureColor', 'w');
% Target colours (T1, T2, T3) and tracker-configuration colours
st.c   = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.20 0.60 0.20];
st.cfg = [0.55 0.55 0.55; 0.30 0.55 0.85; 0.05 0.30 0.60];
st.det = [0.62 0.62 0.62];
st.trk = [0.10 0.10 0.10];
end
