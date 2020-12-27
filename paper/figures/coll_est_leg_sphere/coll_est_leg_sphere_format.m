% Beide Bilder unabhängig voneinander speichern
% Vorher ausführen: coll_est_leg_sphere_eval.m

exp_pfad = fullfile(collhdl_pfad, 'paper','figures','coll_est_leg_sphere');
%% Ausschnitt Bein
close all
uiopen(fullfile(exp_pfad, 'coll_est_leg_sphere_legcad_raw.fig'),1);
figHandle = gcf;
figure(figHandle);
set(figHandle, 'WindowStyle', 'Normal');

% get(gca)

get(gca, 'View')
get(gca, 'CameraViewAngle')
get(gca, 'CameraPosition')
get(gca, 'CameraTarget')

set(gca, 'View', [  154.7441    7.2197])
set(gca, 'CameraViewAngle', 0.8558)
set(gca, 'CameraPosition', [ 6.0608    8.1378    1.5465])
set(gca, 'CameraTarget', [-0.0261    0.1115    0.4223])
% set(gca, 'XLim', [-0.9112 0.9858])
% set(gca, 'YLim', [-0.4219 0.6454])
% set(gca, 'ZLim', [-0.0078 1.0595])
figure_format_publication(gca);
set_size_plot_subplot(figHandle, ...
  4.3, 5.5, ... % halbe Spalte
  gca, ...
  0.07, 0.08, 0.05, 0.17, ... % l r u d
  0.01, 0.01); % x y

hdl1=plot(NaN,NaN,'ko', 'MarkerSize', 5);% Sim: Schwarz
hdl2=plot(NaN,NaN,'bd', 'MarkerSize', 5);% M1: blau
hdl3=plot(NaN,NaN,'rx', 'MarkerSize', 5);% M2: rot
% c=get(gca, 'Children');
linhdl = [hdl1;hdl2;hdl3];
% Keine Legende. Nehme in Artikel die vom anderen Bild als Verweis
% h = legend(linhdl, {'Sim','M1', 'M2'});
% set(h, 'Position', [0.1, 0.1, .01, .05], ... % x y b h
%   'Orientation', 'Vertical') 

xlim( minmax(r_W_W_C1_ges(IpMG,1)') + [0.02 0]);
ylim( minmax(r_W_W_C1_ges(IpMG,2)') + [0.05 0]);
zlim( minmax(r_W_W_C1_ges(IpMG,3)') +[-0.05, -0.32] );
% Achsenbeschriftung so ändern, dass [mm] statt [m]
plot_scale_ticklabel(gca, 1e3, {'X','Y','Z'}, {'%1.0f'});
YTL=get(gca,'YTICKLABEL');
YTL_neu = YTL;
YTL_neu{3} = '';
set(gca,'YTICKLABEL', YTL_neu);
hdlxl=xlabel('x [mm]');hdlyl=ylabel('y [mm]');hdlzl=zlabel(sprintf('z\n[mm]'));
% Positionend er Achsenbeschriftungen
set(hdlxl,'POSITION', [0, 0.15, 0.33]);
set(hdlyl,'POSITION', [0.01 0.1101 0.32]);
set(hdlzl,'POSITION', [0.0425 0.1 0.475]);
% change_plot_data_scale(gca, [1e3 1e3 1e3]);
Filebasename_res = 'coll_est_leg_sphere_legcad_detail';
export_fig(fullfile(exp_pfad, [Filebasename_res, '.pdf']));

%% XYZ-Kontaktpunktschätzung
close all
% Öffne xyz-Bild
uiopen(fullfile(exp_pfad, 'coll_est_leg_sphere_xyzcoll_raw.fig'),1);
figHandle1 = gcf;
pause(1)
c1 = get(figHandle1, 'Children');
% Öffne CD-Bild
uiopen(fullfile(exp_pfad, 'coll_est_leg_sphere_cd_ic_raw.fig'),1);
pause(1) % Manchmal ist das handle noch nicht verfügbar und gcf bezieht sich auf das vorherige figur
figHandle2 = gcf;
c2 = get(figHandle2, 'Children');

% Füge Passende Subplots zu einem großen Bild zusammen
figHandle = figure();
axhdl = NaN(2,1);
for i = 1:2
  axhdl(i) = subplot(2,1,i);
  % Kopiere Inhalt
  if i==1
    quellhdl = c1(2); % Letzter Subplot = erstes Axes-Handle
  else
    quellhdl = c2(4);
  end
  copyobj(get(quellhdl,'children'), axhdl(i));
  set(axhdl(i), 'YLABEL', get(quellhdl, 'YLABEL'));
end

% Gesamtbild formatieren
set(figHandle, 'WindowStyle', 'Normal');

% Format sollte so sein wie im CAD-Bild
lformat = {'k',  'o', '--', 50; ... % Sim: Schwarz
          'b', 's', '-', 90; ... % M1: blau
          'r', 'x', '-', 100};    % M2: rot
        
for i = 1:2
  axes(axhdl(i)) %#ok<LAXES>
  % Formatieren
  plothdl = get(axhdl(i), 'Children');
  if i == 1
    legplothdl = line_format_publication(plothdl([2 1]), lformat(2:3,:), {'M1', 'M2'});
  else
    legplothdl = line_format_publication(plothdl([1 3 2]), lformat, {'M1', 'M2', 'Sim'});
  end

  grid on;
end
axes(axhdl(1))
change_plot_data_scale(axhdl(1),[1 1e3 1]); 
ylabel(sprintf('$||\\Delta{r}_{C}||$ [mm]'), 'interpreter', 'latex');
axes(axhdl(2))
xlabel('$t$ [s]', 'interpreter', 'latex');

figure_format_publication(axhdl)
remove_InnerLabels_byHandle(axhdl,1)
set_size_plot_subplot(figHandle, ...
  8, 5, ... % halbe Spalte
  axhdl, ...
  0.16, 0.02, 0.12, 0.18, ... % l r u d
  0.14, 0.05) % x y

linkxaxes
xlim([0.565, 0.63])
h = legend(legplothdl, {'Sim', 'M1', 'M2'}, 'interpreter', 'latex');
set(h, 'Position', [0.4, 0.93, .25, .05], ... % x y b h
  'Orientation', 'Horizontal') 


set_YAutoscale(figHandle, 0.1, false)
axes(axhdl(1))
ylim([10, 70])

% x-Position der y-Achsenbeschriftungen ändern
YL = NaN(size(axhdl));
for i = 1:length(axhdl(:));
  [X_off, X_slope] = get_relative_position_in_axes(axhdl(i), 'x');
  [Y_off, Y_slope] = get_relative_position_in_axes(axhdl(i), 'y');
  YL(i) = get(axhdl(i), 'YLABEL');
  YLP = get(YL(i), 'POSITION');
  yl = get(axhdl(i), 'YLIM');
  set(YL(i), 'POSITION', [X_off+X_slope*(-1.26), mean(yl), 1]);
end

Filebasename_res = 'coll_est_leg_sphere_excoll_cd';
export_fig(fullfile(exp_pfad, [Filebasename_res, '.pdf']));

%% Geschätzte Kräfte
close all
% Öffne xyz-Bild
uiopen(fullfile(exp_pfad, 'coll_est_leg_sphere_forces_raw.fig'),1);
figHandle = gcf;
set(figHandle, 'WindowStyle', 'Normal');
c = get(figHandle, 'Children');
% Plots der Momente löschen
delete(c([1, 2 4 6]))
axhdl = c([7;5;3]);

for i = 1:3
  axes(axhdl(i)); %#ok<LAXES>
  plothdl = get(axhdl(i), 'Children');
  legplothdl = line_format_publication(plothdl([3 2 1]), lformat, {'M1', 'M2', 'Sim'});
  ylabel(sprintf('$f_{\\mathrm{ext},%s}$ [N]', char(119+i)), 'interpreter', 'latex');
end

h = legend(legplothdl, {'Sim', 'M1', 'M2'}, 'interpreter', 'latex');
set(h, 'Position', [0.4, 0.93, .25, .05], ... % x y b h
  'Orientation', 'Horizontal') 

figure_format_publication(axhdl)
remove_InnerLabels_byHandle(axhdl,1)
set_size_plot_subplot(figHandle, ...
  8.6, 8, ... % halbe Spalte
  axhdl, ...
  0.16, 0.04, 0.09, 0.13, ... % l r u d
  0.14, 0.03) % x y

set_YAutoscale(figHandle, 0.1, false)
linkxaxes
xlim( sl.t(IpM([1,end])) );
xlabel('$t$ [s]', 'interpreter', 'latex');

% x-Position der y-Achsenbeschriftungen ändern
YL = NaN(size(axhdl));
for i = 1:length(axhdl(:));
  [X_off, X_slope] = get_relative_position_in_axes(axhdl(i), 'x');
  YL(i) = get(axhdl(i), 'YLABEL');
  YLP = get(YL(i), 'POSITION');
  yl = get(axhdl(i), 'YLIM');
  set(YL(i), 'POSITION', [X_off+X_slope*(-1.26), mean(yl), 1]);
end

Filebasename_res = 'coll_est_leg_sphere_forces';
export_fig(fullfile(exp_pfad, [Filebasename_res, '.pdf']));

%% Geschätztes Störmoment
close all
figHandle=10002;
figure(figHandle);clf;set(figHandle, 'Name', 'tauJ_obs_cmp', 'NumberTitle', 'off');
IJ = AS.JI_lLeg([1 3 4 5]);
spi=0;
axhdl = NaN(2,length(IJ));
hdl_up = NaN(2,1);
hdl_down = NaN(2,1);
format2 = {'b',  '', '-', 0; ...
          'r', 'v', '--', 70}; % andere Farben als oben!
format3 = {'k',  '', '-', 0; ...
          'g', '^', '--', 70}; % andere Farben, damit keine Verwirrung entsteht
for i = 1:length(IJ)
%   spi=spi+1;
  % Bild 1: Gesamtansicht externes Moment
  axhdl(1,i)=subplot(2,length(IJ),sprc2no(2,length(IJ),1,i));
  hold on;
  hdl_up(1)=plot(sl.t, sl.tau_ext_feet(:,IJ(i)));
  hdl_up(2)=plot(sl.t, sl.tauJ_ext_sum(:,IJ(i)));
  grid on;
  ylim([-90, 25])
  ylabel('$\tau_{\mathrm{ext}}$ [Nm]', 'interpreter', 'latex');
  title(sprintf('\\tau_{%d}', IJ(i)));
  
  legplothdl_up = line_format_publication(hdl_up, format2, {'feet', 'sum'});
  
  
  % Bild 2: Detailansicht externes Moment
  axhdl(2,i)=subplot(2,length(IJ),sprc2no(2,length(IJ),2,i));hold on;
  hdl_down(1)=stairs(sl.t, sl.tau_obs(:,IJ(i)));
  hdl_down(2)=plot(sl.t, sl.tau_ext_coll(:,IJ(i)));
  % Grenzen
  plot(sl.t([1, end]), -ones(2,1)*tauJ_obs_thresh(IJ(i)), 'r--');
  plot(sl.t([1, end]),  ones(2,1)*tauJ_obs_thresh(IJ(i)), 'r--');
  grid on;
%   ylabel(sprintf('\\tau_{%d}', IJ(i))); %  (%s) , AS.JN{i}
%   ylim([-3;3]);
  ylabel('$\hat{\tau}_{\mathrm{ext,coll}}$ [Nm]', 'interpreter', 'latex');
  
  legplothdl_down = line_format_publication(hdl_down, format3, {'obs', 'coll'});
end
linkxaxes
xlim( sl.t(IpMG([1,end])) );

% X-Ticks
for i = 1:length(IJ)
  XTL = get(axhdl(2,i), 'XTICKLABEL');
  if i == 1
    set(axhdl(2,i), 'XTICKLABEL', {XTL{1}, '', ''});
  elseif  i < length(IJ)
    set(axhdl(2,i), 'XTICKLABEL', {'', XTL{2}, ''});
  else
    set(axhdl(2,i), 'XTICKLABEL', {'', '', XTL{3}});
  end
end
figure_format_publication(axhdl)
% axhdl=axhdl';
remove_InnerLabels_byHandle(axhdl,1)
remove_InnerLabels_byHandle(axhdl,2)
set(figHandle, 'WindowStyle', 'Normal');
set_size_plot_subplot(figHandle, ...
  8.6, 5, ... % halbe Spalte
  axhdl, ...
  0.13, 0.04, 0.08, 0.1, ... % l r u d
  0.01, 0.05) % x y

linkxaxes
linkaxes( axhdl(1,:), 'y' );
linkaxes( axhdl(2,:), 'y' );

axes(axhdl(1,end));
h_up=legend(legplothdl_up, {'$\tau_{\mathrm{ext,loc}}$', '$\tau_\mathrm{{ext}}$'}, 'interpreter', 'latex');
set(h_up, 'Position', [0.7, 0.83, .10, .05], 'Orientation', 'Horizontal')  % x y b h
ylim([-35, 90])

axes(axhdl(2,end));
h_down=legend(legplothdl_down, {'$\hat{\tau}_{\mathrm{ext,coll}}$', '$\tau_{\mathrm{ext,coll}}$'}, 'interpreter', 'latex');
set(h_down, 'Position', [0.7, 0.15, .10, .05], 'Orientation', 'Horizontal')  % x y b h
ylim([-85, 1])

% set_YAutoscale(figHandle, 0.1, false)

Filebasename_res = 'coll_est_leg_sphere_tauext';
export_fig(fullfile(exp_pfad, [Filebasename_res, '.pdf']));
