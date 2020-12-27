% Generierungsskript für ein Bild mit Auswertung aller Kollisionen mit
% einem Kontakt gleichzeitig
% 
% Vorher:
% atlas5_wbody_distobs_test2_start6.m
% 
% Damit das Bild immer generierbar ist, werden die Zusammenfassungsdaten in
% eine eigene Backupdatei gespeichert und eingecheckt. Diese wird auf
% Abfrage direkt geladen.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc
close all

AS = atlas_const(5);

collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
res_path = fullfile(collhdl_pfad, 'results', 'distobs2_test6_20160728');
vdname_format = 'distobs2test6_Ziel%02dRichtung%d_res.mat';

bild_pfad = fullfile(collhdl_pfad, 'paper','figures','collest_single_allresults');
zusammenfassung_dateipfad = fullfile(bild_pfad, 'collest_single_allresults_backupdata.mat');

% Zusatzinfos zur Fehlersuche
usr_debug = false;
%% Gesamtbild erstellen
% siehe atlas5_wbody_collision_pipeline_simulink_debug_eval

% Erstes Ergebnis laden 
if exist(zusammenfassung_dateipfad, 'file') && input(sprintf('Lade %s [1/0]?', zusammenfassung_dateipfad))
  zsfgdat = load(zusammenfassung_dateipfad);
  qJ_t0 = zsfgdat.qJ_t0;
  T_W_Bt0 = zsfgdat.T_W_Bt0;
else
  erg1 = load( fullfile(res_path, sprintf(vdname_format, 1, 1)) );
  i1 = 1; % Zeichne alles in das Roboterbild für t=0 ein.
  qJ_t0 = erg1.sl.qJ(i1,:)';
  T_W_Bt0 = [erg1.sl.R_base(:,:,i1), erg1.sl.xq_base(i1,1:3)'; [0 0 0 1]];
end
T_c_urdf_t0 = atlas5_wbody_fkine_urdf_num(qJ_t0');


T_c_Wurdf_t0 = NaN(size(T_c_urdf_t0));
for i = 1:size(T_c_Wurdf_t0,3)
  T_c_Wurdf_t0(:,:,i) = T_W_Bt0 * T_c_urdf_t0(:,:,i);
end

figure(555);clf;
set(555, 'Name', sprintf('Rob_Kollision_CAD_Ges'));
hold on; grid on;axis equal;view(3);set(555, 'Renderer','OpenGL')
for li = 1:(AS.NJ+1)
  hdl=atlas_plot_wbody_link_stl(li, uint8(5), T_c_Wurdf_t0);
  % Unterschiedliche Grauschattierungen
  try %#ok<TRYNC> % es gibt teilweise kein CAD-Modell. Dann NaN und Absturz
    grau1 = 0.2+mod(li-1,3)*0.25;
    set(hdl, 'FaceAlpha', 0.2, 'EdgeColor', (grau1+0.1)*[1 1 1], 'FaceColor', grau1*[1 1 1]);
  end
end

% Statistik speichern
% Positionsfehler für alle Teilversuche
stat_dx1 = NaN(31*8,1);
stat_dx2 = NaN(31*8,1);
stat_dx2s = NaN(31*8,1);
stat_ic_ges = NaN(31*8,1);

% Gesamtdaten zur Dokumentation speichern
r_W_W_C_simp_mean_ges = NaN(31*8,3);
r_W_W_C1_mean_ges = NaN(31*8,3);
r_W_W_C2_mean_ges = NaN(31*8,3);
r_W_W_C2s_mean_ges = NaN(31*8,3);
%% Ergebnisse einzeichnen
iW = 0;
for I_Ziel = 1:31 % Körpernummer des Ziels
  for richt = 1:8 % verschiedene Richtungen
    iW = iW+1;
    %% Ergebnisse laden und verarbeiten
    % Die Berechnung der Kontaktpunkte in
    % atlas5_wbody_collision_pipeline_simulink_debug wurde schon
    % durchgeführt
    if ~exist('zsfgdat', 'var')
      datafile = fullfile(res_path, sprintf(vdname_format, I_Ziel, richt));
      if ~exist(datafile, 'file')
        warning('Abgespeicherte Messdaten in %s nicht vorhanden', datafile);
        continue
      end
      fprintf('Lade Datei %d/%d: %s\n', (richt+8*(I_Ziel-1)), 31*8, datafile);
      erg = load( datafile );
    end
    
    
    % Mittlere Kraft im Zeitverlauf
    if ~exist('zsfgdat', 'var')
      F_eff_sim = sqrt( sum( erg.F_ext_W_sim(:,1:3)  .^2, 2) );
      F_eff_1 =   sqrt( sum( erg.F_ext_W_1_ges(:,1:3).^2, 2) );
      F_eff_2 =   sqrt( sum( erg.F_ext_W_2_ges(:,1:3).^2, 2) );
      F_eff_2s =   sqrt( sum( erg.F_ext_W_2s_ges(:,1:3).^2, 2) );
    end
    
    if exist('zsfgdat', 'var')
      r_W_W_C_simp_mean = zsfgdat.r_W_W_C_simp_mean_ges(iW,:);
      r_W_W_C1_mean = zsfgdat.r_W_W_C1_mean_ges(iW,:);
      r_W_W_C2_mean = zsfgdat.r_W_W_C2_mean_ges(iW,:);
      r_W_W_C2s_mean = zsfgdat.r_W_W_C2s_mean_ges(iW,:);
    else
      % Indizes, die zu simulierten bzw erkannten Simulationen gehören
      % Binär-Indizes
      I_sim = ~isnan(F_eff_sim) & ~isnan(erg.r_W_W_Csimp_ges_t0(:,1));
      I_1 = ~isnan(F_eff_1)     & ~isnan(erg.r_W_W_C1_ges_t0(:,1));
      I_2 = ~isnan(F_eff_2)     & ~isnan(erg.r_W_W_C2_ges_t0(:,1));
      I_2s = ~isnan(F_eff_2s)     & ~isnan(erg.r_W_W_C2s_ges_t0(:,1));
      
      % Falls mehrere Körper betroffen: Nicht betrachten
      if length( unique(erg.icsim_ges(I_sim,1)) ) > 1 % size(erg.icsim_ges, 2) > 1 ||
        continue
      end
      
      % Zahlen-Indizes
      II_sim = find(I_sim);
      II_1 = find(I_1);
      II_2 = find(I_2);
      II_2s = find(I_2s);

      % Werte nur die erste Kollision. Bei aufeinanderfolgenden ist der
      % Kraftmittelpunkt irgendwo in der Luft.
      % Kriterium: Sim. oder geschätzte Kollisionsnummer wird unterbrochen
      if any (diff(II_sim) > 1)
        II_sim = II_sim(1:find(diff(II_sim)~=1));
      end
      if any (diff(II_1) > 1)
        II_1 = II_1(1:find(diff(II_1)~=1));
      end
      if any (diff(II_2) > 1)
        II_2 = II_2(1:find(diff(II_2)~=1));
      end
      if any (diff(II_2s) > 1)
        II_2s = II_2s(1:find(diff(II_2s)~=1));
      end
      
      % "wahrer" Kontakt
      % Kraftgemittelter Kontaktmittelpunkt
      r_W_W_C_simp_mean = sum(erg.r_W_W_Csimp_ges_t0(II_sim,:).*repmat(F_eff_sim(II_sim,:),1,3),1) ./ sum(F_eff_sim(II_sim,:));
      r_W_W_C1_mean = sum(erg.r_W_W_C1_ges_t0(II_1,:).*repmat(F_eff_1(II_1,:),1,3),1) ./ sum(F_eff_1(II_1,:));
      r_W_W_C2_mean = sum(erg.r_W_W_C2_ges_t0(II_2,:).*repmat(F_eff_2(II_2,:),1,3),1) ./ sum(F_eff_2(II_2,:));
      r_W_W_C2s_mean = sum(erg.r_W_W_C2s_ges_t0(II_2s,:).*repmat(F_eff_2s(II_2s,:),1,3),1) ./ sum(F_eff_2s(II_2s,:));
    end
    
    % Gesamtdaten speichern
    r_W_W_C_simp_mean_ges(iW,:) = r_W_W_C_simp_mean;
    r_W_W_C1_mean_ges(iW,:) = r_W_W_C1_mean;
    r_W_W_C2_mean_ges(iW,:) = r_W_W_C2_mean;
    r_W_W_C2s_mean_ges(iW,:) = r_W_W_C2s_mean;
    
    % Debug
    if usr_debug
      figure(1);clf;
      subplot(4,1,1); hold on;
      plot(erg.sl.t,F_eff_sim, 'k-')
      plot(erg.sl.t,F_eff_1, 'b-')
      plot(erg.sl.t,F_eff_2, 'r--')
      plot(erg.sl.t,F_eff_2s, 'c-')
      plot(erg.sl.t(II_sim([1 end])), mean(F_eff_sim(II_sim))*[1;1], 'k--o')
      plot(erg.sl.t(II_2s([1 end])), mean(F_eff_2s(II_2s))*[1;1], 'c--x')
      grid on;
      ylabel('F');

      for k = 1:3
        subplot(4,1,1+k); hold on;
        hdl1=plot(erg.sl.t, erg.r_W_W_Csimp_ges_t0(:,k), 'k-');
        hdl2=plot(erg.sl.t, erg.r_W_W_C1_ges_t0(:,k), 'b-');
        hdl3=plot(erg.sl.t, erg.r_W_W_C2_ges_t0(:,k), 'r--');
        hdl4=plot(erg.sl.t, erg.r_W_W_C2s_ges_t0(:,k), 'c-');
        set (gca, 'ColorOrderIndex', 1);
        plot(erg.sl.t(II_sim([1 end])), [1;1]*r_W_W_C_simp_mean(k), 'k--o')
        plot(erg.sl.t(II_1([1 end])), [1;1]*r_W_W_C1_mean(k), 'b--d')
        plot(erg.sl.t(II_2([1 end])), [1;1]*r_W_W_C2_mean(k), 'r--x')
        plot(erg.sl.t(II_2s([1 end])), [1;1]*r_W_W_C2s_mean(k), 'c--x')
        grid on;
        legend([hdl1,hdl2,hdl3,hdl4], {'Sim', 'M1', 'M2', 'M2/Sim'});
        ylabel(sprintf('r %s', char(119+k)));
      end
      linkxaxes
    end
    %% Einzeichnen
%     if richt == 2 || richt == 6 % Zeichne niht alle Punkte ein (wird unübersichtlich). 2 und 6 sind die Würfe mit 30° Winkel von der Mittelachse
    % Mittelwert für jede Methode
    figure(555)
    plot3(r_W_W_C_simp_mean(1),r_W_W_C_simp_mean(2),r_W_W_C_simp_mean(3),'ko');
    
%     plot3(r_W_W_C1_mean(1),r_W_W_C1_mean(2),r_W_W_C1_mean(3),'bs', 'MarkerSize', 10);
%     plot3([r_W_W_C1_mean(1);r_W_W_C_simp_mean(1)], ...
%           [r_W_W_C1_mean(2);r_W_W_C_simp_mean(2)], ...
%           [r_W_W_C1_mean(3);r_W_W_C_simp_mean(3)], 'b--', 'LineWidth', 2);
    plot3(r_W_W_C2_mean(1),r_W_W_C2_mean(2),r_W_W_C2_mean(3),'rx', 'MarkerSize', 10);
    plot3([r_W_W_C2_mean(1);r_W_W_C_simp_mean(1)], ...
          [r_W_W_C2_mean(2);r_W_W_C_simp_mean(2)], ...
          [r_W_W_C2_mean(3);r_W_W_C_simp_mean(3)], 'r-', 'LineWidth', 2);

    if usr_debug
      text(r_W_W_C2_mean(1), r_W_W_C2_mean(2), r_W_W_C2_mean(3), sprintf('Z%02d/R%d', I_Ziel, richt));
    end
        
%     end
        
    %% Auswertung speichern
    stat_dx1(iW) = norm(r_W_W_C1_mean(:)-r_W_W_C_simp_mean(:));
    stat_dx2(iW) = norm(r_W_W_C2_mean(:)-r_W_W_C_simp_mean(:));
    stat_dx2s(iW) = norm(r_W_W_C2s_mean(:)-r_W_W_C_simp_mean(:));
    % Werte dies als Kollision mit dem Körper, der zuerst getroffen wurde.
    if exist('zsfgdat', 'var')
      stat_ic_ges(iW) = zsfgdat.stat_ic_ges(iW);
    else
      stat_ic_ges(iW) = erg.icsim_ges(II_sim(1));
    end
  end
end
saveas(555, fullfile(bild_pfad, 'coll_est_sc_all_raw.fig'));
if ~exist('zsfgdat', 'var')
  save(zusammenfassung_dateipfad, ...
    'r_W_W_C_simp_mean_ges', 'r_W_W_C1_mean_ges', 'r_W_W_C2_mean_ges', 'r_W_W_C2s_mean_ges', ...
    'stat_ic_ges', 'qJ_t0', 'T_W_Bt0');
end
%% Tabelle mit Werten
% Gebe Ergebnistabelle für jede Methode aus
Ilegs = false(length(stat_ic_ges),1);
Iarms = false(length(stat_ic_ges),1);
Itorso = false(length(stat_ic_ges),1);
for Methode = 1:3
  % Prüfe, zu welcher Kette jeder der Kollisionen gehört
  if Methode == 1
    stat_dx = stat_dx1;
    MName = '1';
  elseif Methode == 2
    stat_dx = stat_dx2;
    MName = '2';
  else
    stat_dx = stat_dx2s;
    MName = '2s';
  end
  for iW = 1:length(stat_ic_ges)
    if any([AS.LI_lLeg, AS.LI_rLeg] == stat_ic_ges(iW))
      Ilegs(iW) = true;
    end
    if any([AS.LI_lArm, AS.LI_rArm] == stat_ic_ges(iW))
      Iarms(iW) = true;
    end
    if any([AS.LI_Torso] == stat_ic_ges(iW))
      Itorso(iW) = true;
    end
  end
  fprintf('\n\nMethode %s:\n', MName);
  fprintf('Torso: Number, Mean, Min, Max\n');
  fprintf('\t%d, %1.0f %1.0f %1.0f\n', sum(Itorso), 1e3*mean(stat_dx(Itorso)), 1e3*min(stat_dx(Itorso)), 1e3*max(stat_dx(Itorso)));
  fprintf('Legs: Number, Mean [mm], Min [mm], Max [mm]\n');
  fprintf('\t%d, %1.0f %1.0f %1.0f\n', sum(Ilegs), 1e3*mean(stat_dx(Ilegs)), 1e3*min(stat_dx(Ilegs)), 1e3*max(stat_dx(Ilegs)));
  fprintf('Arms: Number, Mean, Min, Max\n');
  fprintf('\t%d, %1.0f %1.0f %1.0f\n', sum(Iarms), 1e3*mean(stat_dx(Iarms)), 1e3*min(stat_dx(Iarms)), 1e3*max(stat_dx(Iarms)));
  fprintf('Multiple Collision. Disregarded\n\t%d.\n', sum(isnan(stat_ic_ges)));
end
return
%% Bild Formatieren

close all
uiopen(fullfile(bild_pfad, 'coll_est_sc_all_raw.fig'),1);
figHandle = gcf;

set(figHandle, 'WindowStyle', 'Normal');

view(75,15)

xlim( [-0.35 0.35] );
ylim( [-1.1 1.1] );
zlim( [0 2] );

% Achsen in mm angeben
% plot_scale_ticklabel(gca, 1e3, {'X','Y','Z'}, {'%1.0f'});
hdlxl=xlabel('x [m]');hdlyl=ylabel('y [m]');hdlzl=zlabel(sprintf('z\n[m]'));
set(hdlxl,'Position', [0 -1.1 -0.2229]);
set(hdlzl,'Position', [0 -1.2880 1.9000]);
figure_format_publication(gca);
set_size_plot_subplot(figHandle, ...
  8.6, 8, ... % halbe Spalte
  gca, ...
  0.05, 0.02, 0.05, 0.10, ... % l r u d
  0.01, 0.01); % x y

% Legende
hdl1=plot(NaN,NaN,'ko', 'MarkerSize', 5);% Sim: Schwarz
% hdl2=plot(NaN,NaN,'-bd', 'MarkerSize', 5);% M1: blau
hdl3=plot(NaN,NaN,'-rx', 'MarkerSize', 5);% M2: rot
linhdl = [hdl1;hdl3];
h = legend(linhdl, {'Sim','M2'}, ...
  'Position', [0.6, 0.9, .01, .05], ... % x y b h
  'Orientation', 'Horizontal');

Filebasename_res = 'coll_est_sc_all';
export_fig(fullfile(bild_pfad, [Filebasename_res, '.pdf']));

return
