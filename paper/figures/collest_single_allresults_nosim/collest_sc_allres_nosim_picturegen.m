% Generierungsskript für ein Bild mit Auswertung aller Kollisionen mit
% einem Kontakt gleichzeitig (Ohne Simulation des Zeitverlaufs)
% 
% Vorher:
% atlas5wbody_collisions_statistics_sc.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc
close all

AS = atlas_const(5);

collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
atlas5_collbodies = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'I', 'T', 'P');
res_path = fullfile(collhdl_pfad, 'results', 'atlas5wbody_collisions_statistics_sc');

bild_pfad = fullfile(collhdl_pfad,'paper','figures','collest_single_allresults_nosim');

% Zusatzinfos zur Fehlersuche
usr_debug = false;

% Anzahl der eingezeichneten Punkte pro Segment
NPP = 40; % maximale Anzahl an Punkten pro Segment
p_mindist = 0.20; % minimaler Abstand der Punkte
%% Gesamtbild erstellen
% siehe atlas5_wbody_collision_pipeline_simulink_debug_eval


ResultStruct = load(fullfile(res_path, 'result.mat'));
qJ = ResultStruct.qJ_wbody;
T_W_0 = ResultStruct.T_W_0;
ForceSensorStruct = ResultStruct.ForceSensorStruct;


T_c_urdf = atlas5_wbody_fkine_urdf_num(qJ');
T_c_mdh = atlas5_wbody_fkine_mdh_num(qJ');

T_c_Wurdf = NaN(size(T_c_urdf));
T_c_Wmdh = NaN(size(T_c_mdh));
for i = 1:size(T_c_Wurdf,3)
  T_c_Wmdh(:,:,i) = T_W_0*T_c_mdh(:,:,i);
  T_c_Wurdf(:,:,i) = T_W_0*T_c_urdf(:,:,i);
end

figure(555);clf;
set(555, 'Name', sprintf('Rob_Kollision_CAD_Ges'));
hold on; grid on;axis equal;view(3);set(555, 'Renderer','OpenGL')
for li = 1:(AS.NJ+1)
  grau1 = 0.7;%+mod(li-1,2)*0.25;
  atlas_plot_wbody_link_collision_body(li, T_c_Wmdh, atlas5_collbodies, ...
    0.2, grau1*[1 1 1], (grau1)*[1 1 1], 0.2);
end


%% Ergebnisse einzeichnen

for IL = 1:31 % Körpernummer des Ziels
  
  % Indizes der Kollisionen zu diesem Segment
  IC_IL = find(ResultStruct.iCB == IL);
  
  if isempty(IC_IL)
    continue % Keine Ersatzkörper
  end
  
  % Plotte auf jedem Ersatzkörper eine Anzahl NPP Punkte
  I_offen = IC_IL;
  IC_Wahl = [];
  for ii = 1:100 % Obergrenze für Anzahl der Versuche
    
    I1 = randperm(length(I_offen),1);
    IC1 = IC_IL(I1);
    % Prüfe ob weit genug von allen anderen entfernt
    Diff_Alle = ResultStruct.r_0_0_C1(IC_Wahl,:) - repmat(ResultStruct.r_0_0_C1(IC1,:), length(IC_Wahl), 1);
    % Abstand berechnen
    if any( sqrt( sum(Diff_Alle.^2, 2) ) < p_mindist )
      % Der neue Punkt ist zu nah an bereits gewählten Punkten. Nächster
      % Versuch
      continue
    end
    
    IC_Wahl = [IC_Wahl; IC1]; %#ok<AGROW>
    I_offen( I_offen == IC1 ) = NaN; % Aus Liste entfernen
    if length(IC_Wahl) == NPP
      break; % Genug Punkte für diesen Körper
    end
  end
  
  for ii = 1:length(IC_Wahl)
    jj = IC_Wahl(ii);
    r_W_W_C1 = T_W_0*[ResultStruct.r_0_0_C1(jj,:)';1];
    r_W_W_C2 = T_W_0*[ResultStruct.r_0_0_C2(jj,:)';1];
    r_W_W_C3 = T_W_0*[ResultStruct.r_0_0_C3(jj,:)';1];
    
    hdl1=plot3(r_W_W_C1(1), r_W_W_C1(2), r_W_W_C1(3), 'kx', 'MarkerSize', 6);
    hdl2=plot3(r_W_W_C2(1), r_W_W_C2(2), r_W_W_C2(3), 'b^', 'MarkerSize', 6);
    hdl3=plot3(r_W_W_C3(1), r_W_W_C3(2), r_W_W_C3(3), 'rv', 'MarkerSize', 6);
    hdl4=plot3([r_W_W_C1(1);r_W_W_C2(1)], [r_W_W_C1(2);r_W_W_C2(2)], [r_W_W_C1(3);r_W_W_C2(3)], 'b-', 'LineWidth', 2);
    hdl5=plot3([r_W_W_C1(1);r_W_W_C3(1)], [r_W_W_C1(2);r_W_W_C3(2)], [r_W_W_C1(3);r_W_W_C3(3)], 'r-', 'LineWidth', 2);
  end
  

  
end

% Anfangspunkt der Sensoren speichern (falls abweichend von Standard)
r_W_P1m = NaN(ForceSensorStruct.nS,3);
u_W_P2m = NaN(ForceSensorStruct.nS,3);

d_sens = 0.05;

% Hände
u_W_P2m(1,:) = T_c_Wmdh(1:3,3,24)*d_sens;
u_W_P2m(2,:) = -T_c_Wmdh(1:3,3,31)*d_sens;

% Füße
u_W_P2m(3,:) = -T_c_Wurdf(1:3,3,11)*d_sens;
u_W_P2m(4,:) = -T_c_Wurdf(1:3,3,17)*d_sens;

% Torso
u_W_P2m(5,:) = -T_c_Wurdf(1:3,3,4)*d_sens;

% Kraftsensoren einzeichnen
for iS = 1:ForceSensorStruct.nS
  % Position des Sensors
  LI = ForceSensorStruct.LI(iS);
  T_W_i = T_c_Wurdf(:,:,LI);
  
  % direkt einzeichnen als roten Zylinder
  if any(isnan(r_W_P1m(iS,:)))
    r_W_P1 = T_W_i(1:3,4);
  else
    r_W_P1 = r_W_P1m(iS,:)';
  end
  if any(isnan(u_W_P2m(iS,:)))
    u_W_P1 = T_W_i(1:3,3)*d_sens;
  else
    u_W_P1 = u_W_P2m(iS,:)';
  end
  
  r_W_P2 = r_W_P1 + u_W_P1;
  r_zyl = 0.05;
  drawCylinder([r_W_P1', r_W_P2', r_zyl], 'EdgeColor', [0 0 0], ...
    'FaceAlpha', 1, 'FaceColor', [0 0 0], 'EdgeAlpha', 1);
  
end
%% Statistik

% return

%% Speichern
saveas(555, fullfile(bild_pfad, 'coll_est_sc_all_nosim_raw.fig') );

%% Bild Formatieren

close all
uiopen(fullfile(bild_pfad, 'coll_est_sc_all_nosim_raw.fig'),1);
figHandle = gcf;

set(figHandle, 'WindowStyle', 'Normal');

view(75,15)
set(gca,'XTICK',[]);set(gca,'YTICK',[]);set(gca,'ZTICK',[]);
xlim( [-0.35 0.35] );
ylim( [-1.1 1.1] );
zlim( [0 2] );

% Kameraeinstellungen durch händisches zoomen
get(gca, 'CameraPosition')
get(gca, 'CameraTarget')
get(gca, 'CameraViewAngle')

set(gca, 'CameraPosition', [14.1848 -3.6902 4.9447]);
set(gca, 'CameraTarget', [-0.0408    0.2173    0.9919]);
set(gca, 'CameraViewAngle', 5.9480);

set(gcf,'color','w');
set_size_plot_subplot(figHandle, ...
  8.6, 8, ... % halbe Spalte
  gca, ...
  0.05, 0.02, 0.05, 0.10, ... % l r u d
  0.01, 0.01); % x y

% Legende
hdl1=plot(NaN,NaN,'kx', 'MarkerSize', 5);% Sim: Schwarz
hdl2=plot(NaN,NaN,'b^', 'MarkerSize', 5);% M1: Blau
hdl3=plot(NaN,NaN,'rv', 'MarkerSize', 5);% M2: Rot
linhdl = [hdl1;hdl2;hdl3];
legtext = {'$r_C$','$r_\mathrm{C}(\hat{\tau}_\varepsilon)$', ...
  '$r_C(\hat{\mathcal{F}}_{\mathrm{ext,S}})$'};
h = legend(linhdl, legtext, ...
  'Position', [0.8, 0.85, .01, .05], ... % x y b h
  'Orientation', 'Vertical', ...
  'interpreter', 'latex');

Filebasename_res = 'coll_est_sc_all_nosim';
export_fig(fullfile(bild_pfad, [Filebasename_res, '.pdf']));