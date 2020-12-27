% Generierungsskript für ein Bild mit Auswertung aller Kollisionen mit
% mehreren Kontakt gleichzeitig
% 
% Vorher:
% atlas5_wbody_sphere4_test_start4.m
% 
% Damit das Bild immer generierbar ist, werden die Zusammenfassungsdaten in
% eine eigene Backupdatei gespeichert und eingecheckt. Diese wird auf
% Abfrage direkt geladen.
% 
% Siehe: collest_sc_allres_picturegen.m, atlas5_wbody_sphere4_test_start4.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc
close all

AS = atlas_const(5);

collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
res_path = fullfile(collhdl_pfad, 'results', 'atlas5_wbody_sphere4_test_start4_20160728');
vdname_format = 'atlas5_wbody_sphere4_test_start4_Konf%02dRichtung%d_res.mat';

bild_pfad = fullfile(collhdl_pfad, 'paper','figures','collest_multi_allresults');
zusammenfassung_dateipfad = fullfile(bild_pfad, 'collest_multi_allresults_backupdata.mat');

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
  erg1 = load( fullfile(res_path, sprintf(vdname_format, 1, 2)) );
  i1 = 1; % Zeichne alles in das Roboterbild für t=0 ein.
  qJ_t0 = erg1.sl.qJ(i1,:)';
  T_W_Bt0 = [erg1.sl.R_base(:,:,i1), erg1.sl.xq_base(i1,1:3)'; [0 0 0 1]];
end
T_c_urdf_t0 = atlas5_wbody_fkine_urdf_num(qJ_t0');


T_c_Wurdf_t0 = NaN(size(T_c_urdf_t0));
for i = 1:size(T_c_Wurdf_t0,3)
  T_c_Wurdf_t0(:,:,i) = T_W_Bt0 * T_c_urdf_t0(:,:,i);
end

% Bilder öffnen und initialisieren
for im = 1:2
  figure(555+im);clf;
  set(555+im, 'Name', sprintf('Rob_Kollision_CAD_Ges'));
  hold on; grid on;axis equal;view(3);set(555+im, 'Renderer','OpenGL')
  for li = 1:(AS.NJ+1)
    hdl=atlas_plot_wbody_link_stl(li, uint8(5), T_c_Wurdf_t0);
    % Unterschiedliche Grauschattierungen
    try %#ok<TRYNC> % es gibt teilweise kein CAD-Modell. Dann NaN und Absturz
      grau1 = 0.2+mod(li-1,3)*0.25;
      set(hdl, 'FaceAlpha', 0.2, 'EdgeColor', (grau1+0.1)*[1 1 1], 'FaceColor', grau1*[1 1 1]);
    end
  end
end

% Statistik speichern
% Positionsfehler für alle Teilversuche
stat_dx4 = NaN(31*8,5);
stat_ic_ges = NaN(31*8,5);

% Gesamtdaten zur Dokumentation speichern
r_W_W_C_simp_mean_ges = NaN(3,5,31*8);
r_W_W_C4_mean_ges = NaN(3,5,31*8);
%% Ergebnisse einzeichnen
iW = 0;
for iZK = 1:300
  for richt = 1:4 % verschiedene Richtungen
    iW = iW+1;
    %% Ergebnisse laden und verarbeiten
    % Die Berechnung der Kontaktpunkte in
    % atlas5_wbody_collision_pipeline_simulink_debug wurde schon
    % durchgeführt
    if ~exist('zsfgdat', 'var')
      datafile = fullfile(res_path, sprintf(vdname_format, iZK, richt));
      if ~exist(datafile, 'file')
        warning('Abgespeicherte Messdaten in %s nicht vorhanden', datafile);
        continue
      end
      fprintf('Lade Datei %d/%d: %s\n', (richt+4*(iZK-1)), 31*4, datafile);
      erg = load( datafile );
    end
    
    for jjc = 1:5
      % Mittlere Kraft im Zeitverlauf
      if ~exist('zsfgdat', 'var')
        F_eff_sim = sqrt( sum( (squeeze(erg.F_ext_W_sim_gesm(1:3,jjc,:))')  .^2, 2) );
        F_eff_4 =   sqrt( sum( (squeeze(erg.F_ext_W_4_ges(1:3,jjc,:))')  .^2, 2) );
      end

      if exist('zsfgdat', 'var')
        r_W_W_C_simp_mean = zsfgdat.r_W_W_C_simp_mean_ges(:,jjc,iW);
        r_W_W_C4_mean = zsfgdat.r_W_W_C4_mean_ges(:,jjc,iW);
      else
        % Indizes, die zu simulierten bzw erkannten Simulationen gehören
        % Binär-Indizes
        I_sim = ~isnan(F_eff_sim) & any(~isnan( squeeze(erg.r_W_W_Csimp_gesm_t0(1,:,:))' ),2);
        I_4   = ~isnan(F_eff_4)   & any(~isnan( squeeze(erg.r_W_W_C4_ges_t0(1,:,:))' ),2);

        % Falls mehrere Körper betroffen: Nicht betrachten
        if length( unique(erg.icsim_ges(I_sim,1)) ) > 1 % size(erg.icsim_ges, 2) > 1 ||
          continue
        end

        % Zahlen-Indizes
        II_sim = find(I_sim);
        II_4 = find(I_4);

        if isempty(II_sim)
          % Keine Kollision mit Kette jjc
          continue
        end
        if isempty(II_4)
          % Keine Kollision mit Methode erkannt
          continue
        end
        
        % Werte nur die erste Kollision. Bei aufeinanderfolgenden ist der
        % Kraftmittelpunkt irgendwo in der Luft.
        % Kriterium: Sim. oder geschätzte Kollisionsnummer wird unterbrochen
        if any (diff(II_sim) > 1)
          II_sim = II_sim(1:find(diff(II_sim)~=1));
        end
        if any (diff(II_4) > 1)
          II_4 = II_4(1:find(diff(II_4)~=1));
        end
        
        % "wahrer" Kontakt
        % Kraftgemittelter Kontaktmittelpunkt
        r_W_W_C_simp_mean = sum( squeeze(erg.r_W_W_Csimp_gesm_t0(1:3,jjc,II_sim))' ...
          .*repmat(F_eff_sim(II_sim,:),1,3), 1) ./ sum(F_eff_sim(II_sim,:));
        r_W_W_C4_mean = sum( squeeze(erg.r_W_W_C4_ges_t0(1:3,jjc,II_4))' ...
          .*repmat(F_eff_4(II_4,:),1,3),1) ./ sum(F_eff_4(II_4,:));
      end

      % Gesamtdaten speichern
      r_W_W_C_simp_mean_ges(:,jjc,iW) = r_W_W_C_simp_mean;
      r_W_W_C4_mean_ges(:,jjc,iW) = r_W_W_C4_mean;

      % Debug
      if usr_debug
        figure(1);clf;
        subplot(4,1,1); hold on;
        plot(erg.sl.t,F_eff_sim, 'k-')
        stairs(erg.sl.t,F_eff_4, 'r-')
        plot(erg.sl.t([1 end]), mean(F_eff_sim(II_sim))*[1;1], 'k--')

        grid on;

        for k = 1:3
          subplot(4,1,1+k); hold on;
          hdl1=plot(erg.sl.t, squeeze(erg.r_W_W_Csimp_gesm_t0(k,jjc,:)), 'k-');
          hdl2=plot(erg.sl.t, squeeze(erg.r_W_W_C4_ges_t0(k,jjc,:)), 'b-');
          set (gca, 'ColorOrderIndex', 1);
          plot(erg.sl.t(II_sim([1 end])), [1;1]*r_W_W_C_simp_mean(k), 'k--o')
          plot(erg.sl.t(II_4([1 end])), [1;1]*r_W_W_C4_mean(k), 'b--d')
          grid on;
          legend([hdl1,hdl2], {'Sim', 'M4'});
        end
        linkxaxes
      end

      %% Einzeichnen
  %     if richt == 2 || richt == 6 % Zeichne niht alle Punkte ein (wird unübersichtlich). 2 und 6 sind die Würfe mit 30° Winkel von der Mittelachse
      % Mittelwert für jede Methode
      figure(555+1)
      plot3(r_W_W_C_simp_mean(1),r_W_W_C_simp_mean(2),r_W_W_C_simp_mean(3),'ko');

      plot3(r_W_W_C4_mean(1),r_W_W_C4_mean(2),r_W_W_C4_mean(3),'rx', 'MarkerSize', 10);
      plot3([r_W_W_C4_mean(1);r_W_W_C_simp_mean(1)], ...
            [r_W_W_C4_mean(2);r_W_W_C_simp_mean(2)], ...
            [r_W_W_C4_mean(3);r_W_W_C_simp_mean(3)], 'r-', 'LineWidth', 2);

      if usr_debug
        text(r_W_W_C4_mean(1), r_W_W_C4_mean(2), r_W_W_C4_mean(3), sprintf('ZK%02d/R%d', iZK, richt));
      end

  %     end

      %% Auswertung speichern
      stat_dx4(iW,jjc) = norm(r_W_W_C4_mean(:)-r_W_W_C_simp_mean(:));
      if isnan(stat_dx4(iW,jjc))
        error('nan');
      end
      % Werte dies als Kollision mit dem Körper, der zuerst getroffen wurde.
      if exist('zsfgdat', 'var')
        stat_ic_ges(iW,:) = zsfgdat.stat_ic_ges(iW,:);
      else
        stat_ic_ges(iW,:) = erg.icsim_ges(II_sim(1),:);
      end
    end
  end
end
for im = 1:2
  saveas(555+im, fullfile(bild_pfad, sprintf('coll_est_mc_all_raw_%d.fig', im)));
end
if ~exist('zsfgdat', 'var')
  save(zusammenfassung_dateipfad, ...
    'r_W_W_C_simp_mean_ges', 'r_W_W_C4_mean_ges', ...
    'stat_ic_ges', 'qJ_t0', 'T_W_Bt0');
end

%% Tabelle mit Werten
% Gebe Ergebnistabelle für jede Methode aus
Ilegs = zeros(size(stat_ic_ges)); % Anzahl der Kollisionen
Iarms = zeros(size(stat_ic_ges));
Itorso = zeros(size(stat_ic_ges));
for Methode = 1:3
  % Prüfe, zu welcher Kette jeder der Kollisionen gehört
  if Methode == 1
    stat_dx = stat_dx4;
    MName = '4';
  end
  chain_names = {'Torso', 'lLeg', 'rLeg', 'lArm', 'rArm'};
  fprintf('\n\nMethode %s:\n', MName);
  for jjc = 1:5 % Jede Kette einmal betrachten
    % Indizes mit Kollisionen zu der betrachteten Kette
    Ichain = stat_ic_ges(:,jjc)>0;
    % Indizes mit erkannten Kollisionen mit der aktuellen Methode
    Ichainrec = Ichain & (~isnan(stat_dx(:,jjc)));
    % Ergebnisse berechnen und anzeigen
    fprintf('%s: Number, Recognized, Mean, Min, Max\n', chain_names{jjc});
    fprintf('\t%d, %d, %1.0f %1.0f %1.0f\n', sum(Ichain), sum(Ichainrec), ...
      1e3*mean(stat_dx(Ichainrec,jjc)), 1e3*min(stat_dx(Ichainrec,jjc)), 1e3*max(stat_dx(Ichainrec,jjc)));
  end
end
return
%% Bild Formatieren
MNamen = {'M4'};
for im = 1%:2
  close all
  uiopen(fullfile(bild_pfad, sprintf('coll_est_mc_all_raw_%d.fig', im)),1);
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
  h = legend(linhdl, {'Sim',MNamen{im}}, ...
    'Position', [0.6, 0.9, .01, .05], ... % x y b h
    'Orientation', 'Horizontal');

  Filebasename_res = sprintf('coll_est_sc_all_%d', im);
  export_fig(fullfile(bild_pfad, [Filebasename_res, '.pdf']));

end
