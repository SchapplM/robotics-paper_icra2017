% Beide Bilder unabhängig voneinander speichern
% Vorher ausführen: colltest_single_filter_eval.m

exp_pfad = fullfile(collhdl_pfad, 'paper','figures','colltest_single_filter');

% Format sollte so sein wie im CAD-Bild
lformat = {'k',  'x', '-', 5; ... % Sim: Schwarz
          'b', 's', '-', 5; ... % M1: blau
          'r', 'o', '-', 5};    % M2: rot

%% Geschätzte Kräfte
close all
% Öffne xyz-Bild
uiopen(fullfile(exp_pfad, 'colltest_single_filter_FM.fig'),1);
figHandle = gcf;
set(figHandle, 'WindowStyle', 'Normal');
c = get(figHandle, 'Children');
% Plots der Momente löschen
hdl_del = false(length(c),1);
for i = 1:length(c)
  if strcmp(get(c(i), 'Type'), 'legend')
    hdl_del(i) = true;
  elseif ~contains(get(get(c(i), 'ylabel'),'String'), 'F x')
    hdl_del(i) = true;
  end
end
delete(c(hdl_del));
axhdl = c(~hdl_del);

for i = 1:1
  axes(axhdl(i)); %#ok<LAXES>
  plothdl = get(axhdl(i), 'Children');
  legplothdl = line_format_publication(plothdl([3 2 1]), lformat, {'$f_{\mathrm{ext},i}$', '$\hat{f}_{\mathrm{ext},S,i}$', '$\hat{f}_{\mathrm{ext},\hat{\tau}_\epsilon,i}$', 'interpreter', 'latex'});
  ylabel(sprintf('$f_{\\mathrm{ext},%s}$ [N]', char(119+i)), 'interpreter', 'latex');
end

h = legend(legplothdl, {'$f_{\mathrm{ext},x}$', ...
  '$\hat{f}_{\mathrm{ext},x}(\hat{\mathcal{F}}_{\mathrm{ext,S}})$', ...
  '$\hat{f}_{\mathrm{ext},x}(\hat{\tau}_\varepsilon)$'}, 'interpreter', 'latex');
set(h, 'Position', [0.43, 0.88, .25, .07], ... % x y b h
  'Orientation', 'Horizontal') 

figure_format_publication(axhdl)
remove_inner_labels(axhdl,1)
set_size_plot_subplot(figHandle, ...
  8.6, 3, ... % halbe Spalte
  axhdl, ...
  0.15, 0.04, 0.19, 0.3, ... % l r u d
  0.12, 0.03) % x y

set_y_autoscale(figHandle, 0.1, false)
linkxaxes
xlim( ResultStruct.t([1,end]) );
xlabel('$t$ [s]', 'interpreter', 'latex');

% x-Position der y-Achsenbeschriftungen ändern
YL = NaN(size(axhdl));
for i = 1:length(axhdl(:))
  [X_off, X_slope] = get_relative_position_in_axes(axhdl(i), 'x');
  YL(i) = get(axhdl(i), 'YLABEL');
  YLP = get(YL(i), 'POSITION');
  yl = get(axhdl(i), 'YLIM');
  set(YL(i), 'POSITION', [X_off+X_slope*(-1.26), mean(yl), 1]);
end

Filebasename_res = 'colltest_single_filter_f';
export_fig(fullfile(exp_pfad, [Filebasename_res, '.pdf']));
