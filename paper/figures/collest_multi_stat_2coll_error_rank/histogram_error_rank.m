% Erzeuge Bild mit Histogramm des Fehlers über den Rang der Jacobimatrix
% 
% Vorher ausführen: atlas5wbody_collisions_statistics_mc.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

clc
clear

%% Init
NP = 2;
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
figpath = fullfile(collhdl_pfad, 'paper', 'figures', 'collest_multi_stat_2coll_error_rank');
res_path = fullfile(collhdl_pfad, 'results', 'atlas5wbody_collisions_statistics_mc');
res_dat = fullfile(res_path, sprintf('result_%dgleichzeitig.mat', NP));

d = load(res_dat);
AS = atlas_const(5);

n_dr = 6; % Anzahl der Datenreihen (eine pro Rang, bis Rang 12)
%% Histogramm berechnen
nbars = 9;
Histdata_X = linspace(.1, 80.1, nbars);
Hist_grenze_selbst = true;
Histdata_Y = NaN(nbars,n_dr); % Gesamtzahl
Histdata_Yp = NaN(nbars,n_dr); % prozentualer Anteil
for ii = 1:n_dr
  rr = 12-n_dr+ii;
  IRang = (d.nInf2 == rr);
  if ii == 1 && ~Hist_grenze_selbst
    [no,xo] = hist(1e2*d.maxfehler_JTp(IRang));
    Histdata_X = xo;
  else
    [no,xo] = hist(1e2*d.maxfehler_JTp(IRang), Histdata_X);
  end
  % Einzelne Reihen normieren
  Histdata_Y(:,ii) = no;
  Histdata_Yp(:,ii) = no/sum(no);
end

%% Zeichnen
% Siehe Ende von atlasn_drwbody_collisions_statistics_mc.m
figHandle = 1;
figure(figHandle);clf;

legplothdl = bar3(1e2*Histdata_Yp);

XTL = {};
for i = 1:n_dr
  XTL{i} = sprintf('%d', 12-n_dr+i); %#ok<SAGROW>
end

YTL = {};
for i = 1:nbars
  YTL{i} = sprintf('%1.0f', Histdata_X(i)); %#ok<SAGROW>
end

hdl=xlabel('rank(J)');
set(hdl, 'Position', [5.2926 9.0297 -17.3933])
set(gca, 'XTICKLABEL', XTL);

set(gca, 'YTICKLABEL', YTL);
ylabel('error [cm]');

zlabel('percentage [%]');

ylim([0,8.5])

% Zahlen nicht auf die Säulen schreiben. Stattdessen in Latex mit Overpic
% Pfeile und Beschriftungen einfügen (hier Probleme mit
% Bildebenenreihenfolge).
% Erster Wert: Rang (je höher die Zahl deste weiter hinten)
% Zweiter Wert: Fehler (je kleiner die Zahl deste geringer der Fehler)
ZahlenInd = [6,1; 5,2; 5,3; 3,3; 3,7]; % ; 4,6
for ii = 1:size(ZahlenInd,1)
  i=ZahlenInd(ii,1); % x: Rang-Achse
  j=ZahlenInd(ii,2); % y: Fehler-Achse
%   hdl=text(i,j,1e2*Histdata_Yp(j,i)+9,sprintf('%1.0f%%', 100*Histdata_Yp(j,i)), ...
%     'FontName', 'Times', 'FontSize', 8);
  % Funktioniert nicht richtig (Bildebenen-Reihenfolge):
%   set(hdl, 'BackgroundColor', [1 1 1], 'EdgeColor', [0 0 0]); %

  fprintf('Säule %d von links (Fehler %1.1f), %d von vorne (Rang %d): Anteil %1.1f%%\n', ...
    j, Histdata_X(j), i, n_dr+i, 1e2*Histdata_Yp(j,i));
end

%% Text
% Setze Fehlergrenze so, dass sie in der Mitte zwischen zwei
% Histogramm-Mittelwerten liegt (sonst ist die Aussage über eine Anzahl
% schwierig.
err_lim = Histdata_X(3) + (Histdata_X(4)-Histdata_X(3))/2;
I = Histdata_X < err_lim;

P = sum( Histdata_Yp(I, :) );
for i = 1:n_dr
  fprintf('Rang %d. %1.0f%% der Fehler kleiner %1.1fcm\n', ...
    12-n_dr+i, 1e2*P(i), err_lim);
end

N = sum( Histdata_Y(I, :) );
Nrd = N(1:end-1); % Gesamtzahlen für jeden Rang mit Rangdefizit und kleinerem Fehler als vorgegeben
Nges_rd = Histdata_Y(:,1:end-1); % Gesamt für jeden Rang mit Rangdefizit

% Angabe eines prozentualen Anteils für Fehler kleiner als vorgegeben
% (berücksichtigt, dass die Fälle unterschiedlich oft vorkommen).
p_err = sum(Nrd) / sum(Nges_rd(:));
fprintf('%1.0f%% der Werte mit Rang %d-%d mit Fehler kleiner %1.1fcm\n', ...
  100*p_err, 12-n_dr+1, 12-1, err_lim);
%% Formatierung
set(figHandle, 'windowstyle', 'normal');
figure_format_publication(gca);
set_size_plot_subplot(figHandle, ...
  10, 7, ... % eine Spalte (halbe Seite)
  gca, ...
  0, 0, 0.005, 0.05, ... % l r u d
  0.01, 0.01); % x y

legtxt = cell(1,n_dr);
for i = 1:n_dr
  legtxt{i} = sprintf('$\\mathrm{rank}(J) = %d$', 12-n_dr+i);
end
h = legend(legplothdl, legtxt, 'interpreter', 'latex');
set(h, 'Position', [0.6061 0.4717 0.2938 0.2940], ... % x y b h (nach händischem Verschieben)
  'Orientation', 'Vertical') 

Filebasename_res = 'hist_rank_err_2coll';
export_fig(fullfile(figpath, [Filebasename_res, '.pdf']));
% saveaspdf(figHandle, fullfile(figpath, Filebasename_res));