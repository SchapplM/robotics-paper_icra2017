% Auswertung des Versuchs, bei dem das Bein von einer Kugel getroffen wird.
% 
% TODO: Allgemeinen Teil in eigenes Skript auslagern

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc

% Mex_Erstellen({ 'find_intersection_line_box', ...
%                 'find_intersection_line_cylinder', ...
%                 'atlas5_wbody_collision_detection', ...
%                 'atlas5_wbody_collision_isolation', ...
%                 'atlas5_wbody_collision_isolation_fullchain', ...
%                 'atlas5_link_contact_force_Hunt_Crossley'});

%% Init
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));

exp_pfad = fullfile(collhdl_pfad, 'paper','figures','coll_est_leg_sphere');

% data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160713', 'distobs2test5_Koll18res.mat');
% data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160714_2045', 'distobs2test5_Koll03res.mat');
% data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160714_2056', 'distobs2test5_Koll11res.mat');
data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160721_1359', 'distobs2test5_Koll01res.mat');
load(data_file);
sl.qJ = sl.q;
sl.qJD = sl.qD;
sl.Fb_ext_obs_comp = sl.F_obs;
sl.tauJ_obs_comp = sl.tau_obs;
sl.tauJ_ext_coll = sl.tau_ext_coll;

AS = atlas_const(5);
nt = length(sl.t);
I_plot = 1:1e-3/mean(diff(sl.t)):nt;

%% Kollisionen für jeden Zeitpunkt bestimmen
% Höhere Schwellwerte für Kollisionserkennung als Standard
usr_Fb_obs_thresh = [5*ones(3,1); 2*ones(3,1)];
usr_tauJ_obs_thresh = [2*ones(3,1); 5; 1*ones(26,1)]; % höherer Schwellwert für neck_ry (dort sollten keine Kollisionen erkannt werden, da keine Ersatzkörper vorliegen)

usr_tauJ_obs_thresh([AS.JI_lLeg(4),AS.JI_rLeg(4)]) = 2;
% Hier ist die Kollision am längsten
usr_tauJ_obs_thresh([AS.JI_lLeg(1:3),AS.JI_rLeg(1:3)]) = 5;

atlas5_wbody_collision_pipeline_simulink_debug

%% Indizes
IKoll = (cd_ges==true) | ~isnan(r_W_W_Csim_ges_t0(:,1)); % Binär-Indizes für Kollisionszeitpunkte
IpMG = find(IKoll); % Indizes für Kollisionszeitpunkte
% IpM = IpM(1:100:end); % Ausdünnung
% Anfang und Ende des simulierten Kontaktes sollen drin sein
I_sim = find(~isnan(r_W_W_Csim_ges_t0(:,1)));
% I_KD = (cd_ges==true) & (~I_sim);
IpM = unique([ I_sim([1;round(length(I_sim)/2);end]); IpMG([length(I_sim):100:end]) ]);
% sl.t(IpM)
%% Auswertung der Kollisionen

figure(1);clf;
subplot(2,1,1);hold on
stairs(sl.t, cd_ges); % Zeigen, dass für Methode 1 und Methode 2 die gleiche Erkennung gilt
stairs(sl.t, cd_ges);
stairs(sl.t, ~isnan(r_W_W_Csim_ges_t0(:,1))); % Simuliert
grid on;
ylabel('cd');
legend({'cd', 'cd', 'Sim'});

subplot(2,1,2);hold on;
stairs(sl.t, ic1_ges);
stairs(sl.t, ic2_ges);
hdl=stairs(sl.t, icsim_ges, 'k-');

set(gca, 'YTick', 1:31);
YTL = cell(31,1);
for i = 1:31
  YTL{i} = sprintf('%s (%d)', strrep(AS.LN{i}, '_', '\_'), i);
end
set(gca, 'YTickLabel', YTL);

grid on;
ylabel('ic');
legend({'M1', 'M2', 'Sim'});
linkxaxes

%% Zeitverlauf der Kontaktpunktschätzungen
figure(10);clf;
for i = 1:4
  subplot(4,1,i);hold on
  if i < 4
  stairs(sl.t(I_plot), r_W_W_C1_ges(I_plot,i),'b-');
  stairs(sl.t(I_plot), r_W_W_C2_ges(I_plot,i),'r-');
  % stairs(sl.t(I_plot), r_W_W_Csim_ges(I_plot,i),'k-');
  stairs(sl.t(I_plot), r_W_W_Csimp_ges(I_plot,i),'k-');
  
%   plot(sl.t, r_W_W_C1_ges_t0(:,i),'b--');
%   plot(sl.t, r_W_W_C2_ges_t0(:,i),'r--');
%   plot(sl.t, r_W_W_Csim_ges_t0(:,i),'k--');
  

  ylabel(sprintf('r %s', char(119+i)));
  else
    % letzten Wert festhalten: Wenn der nächste Wert NaN ist, nehme den
    % aktuellen
    r_W_W_Csimp_ges_halt = r_W_W_Csimp_ges;
    for kt = 2:nt-1
      if isnan(r_W_W_Csimp_ges_halt(kt+1,1))
        r_W_W_Csimp_ges_halt(kt+1,:) = r_W_W_Csimp_ges_halt(kt,:);
      end
    end
    
    % Fehler berechnen
    ex1 = sum(sqrt((r_W_W_C1_ges-r_W_W_Csimp_ges_halt).^2),2);
    ex2 = sum(sqrt((r_W_W_C2_ges-r_W_W_Csimp_ges_halt).^2),2);
    
    stairs(sl.t(I_plot), ex1(I_plot));
    stairs(sl.t(I_plot), ex2(I_plot));
    
    ylabel('ex [m]');
  end
  grid on
  
end
legend({'M1', 'M2', 'sim'});
linkxaxes
xlim(sl.t(IpM(([1,end]))))
% Test
% any(r_W_W_C1_ges(:,:) == 0)
%% Zeitverlauf der Kraftschätzung
figure(8);clf;
FM_string = {'F', 'M'};
for i = 1:4
  for j = 1:2
    subplot(4,2,sprc2no(4,2,i,j));hold on
    
    if i < 4
    stairs(sl.t(I_plot), F_ext_W_sim(I_plot,3*(j-1)+i),'k-');
    stairs(sl.t(I_plot), F_ext_W_1_ges(I_plot,3*(j-1)+i),'b-');
    stairs(sl.t(I_plot), F_ext_W_2_ges(I_plot,3*(j-1)+i),'r--');
    
    ylabel(sprintf('%s %s [N]', FM_string{j}, char(119+i)));
    
    elseif j == 1
      % Winkelfehler der Kraft berechnen
      ephi1 = NaN(nt,1);
      ephi2 = NaN(nt,1);
      for kt = 1:nt
        ephi1(kt) = acos( F_ext_W_sim(kt,1:3)*F_ext_W_1_ges(kt,1:3)'/ ...
                         (norm(F_ext_W_sim(kt,1:3))*norm(F_ext_W_1_ges(kt,1:3))) ) *180/pi;
        ephi2(kt) = acos( F_ext_W_sim(kt,1:3)*F_ext_W_2_ges(kt,1:3)'/ ...
                         (norm(F_ext_W_sim(kt,1:3))*norm(F_ext_W_2_ges(kt,1:3))) ) *180/pi;
      end
      stairs(sl.t(I_plot), ephi1(I_plot), 'b-');
      stairs(sl.t(I_plot), ephi2(I_plot), 'r--');
      ylabel('e phi [deg]');
    end
    
    grid on
  end
end
legend({'M1', 'M2', 'sim'});
linkxaxes
xlim(sl.t(IpM(([1,end]))))
%% Ortskurve des Kollisionspunktes auf dem Bein
% Körperkoordinatensysteme ins Welt-KS transformieren

T_c_Wurdf_t0 = NaN(size(T_c_urdf_t0));
for i = 1:size(T_c_Wurdf_t0,3)
  T_c_Wurdf_t0(:,:,i) = T_W_Bt0 * T_c_urdf_t0(:,:,i);
end
for ip = 1:2
  % Bein zeichnen
  figure(3+ip);clf;
  set(3+ip,'NumberTitle', 'off');
  hold on; grid on;axis equal;view(3);set(3+ip, 'Renderer','OpenGL')
  
  % Zeichne entweder das CAD-Modell oder die Ersatzkörper
  if ip == 1
    set(3+ip, 'Name', sprintf('Bein_Kollision_CAD'));
    for ji = AS.LI_lLeg
      hdl=atlas_plot_wbody_link_stl(ji, uint8(5), T_c_Wurdf_t0);
      % Unterschiedliche Grauschattierungen
      grau1 = 0.2+mod(ji-1,3)*0.25;
      set(hdl, 'FaceAlpha', 0.4, 'EdgeColor', (grau1+0.2)*[1 1 1], 'FaceColor', grau1*[1 1 1]);
    end
  else
    set(3+ip, 'Name', sprintf('Bein_Kollision_ErsatzK'));
    % Zeichne Ersatzkörper des Beins
    
    % Alle Ersatzkörper durchgehen
    for ji = AS.LI_lLeg
      T_Wmdh_i = T_W_Bt0*T_c_mdh_t0(:,:,ji);
      I_i = find(atlas5_collbodies.I == ji);
      for j = I_i' % Index in Ersatzkörper-Liste
        Typ_j = atlas5_collbodies.T(j);
        Par_j = atlas5_collbodies.P(j,:);

        if Typ_j == 1 % Quader
        % Eckpunkte des Quaders
        r_0_Q1 = eye(3,4) * T_Wmdh_i*[Par_j(1:3)';1];
        r_0_Q1Q2 = T_Wmdh_i(1:3,1:3)*Par_j(4:6)';
        r_0_Q1Q3 = T_Wmdh_i(1:3,1:3)*Par_j(7:9)';
        r_0_Q1Q4 = T_Wmdh_i(1:3,1:3)*Par_j(10:12)';
        plot_cube2(r_0_Q1, r_0_Q1Q2, r_0_Q1Q3, r_0_Q1Q4, 'b');
        elseif Typ_j == 2 % Zylinder
          r_0_P1 = eye(3,4) * T_Wmdh_i*[Par_j(1:3)';1];
          r_0_P2 = eye(3,4) * T_Wmdh_i*[Par_j(4:6)';1];
          r_zyl = Par_j(7)';
          drawCylinder([r_0_P1', r_0_P2', r_zyl], 'EdgeColor', 'k', ...
            'FaceAlpha', 0.3, 'FaceColor', 'b')
        end
      end
    end
  end
  % Drehachsen einzeichnen
  for ji = AS.LI_lLeg
    p_ks = T_W_Bt0*T_c_mdh_t0(1:4,4,ji);
    u_ks = T_W_Bt0*T_c_mdh_t0(1:4,3,ji);
    p1 = p_ks+0.1*u_ks;
    p2 = p_ks-0.1*u_ks;
    plot3([p1(1);p2(1)], [p1(2);p2(2)], [p1(3);p2(3)], 'k-', 'LineWidth', 2);
  end
  % Verlauf der Kontaktortschätzung einzeichnen
  plot3(r_W_W_C1_ges_t0(IpM,1),r_W_W_C1_ges_t0(IpM,2),r_W_W_C1_ges_t0(IpM,3), ...
    'bs', 'LineWidth', 5, 'MarkerSize', 8);
  plot3(r_W_W_C2_ges_t0(IpM,1),r_W_W_C2_ges_t0(IpM,2),r_W_W_C2_ges_t0(IpM,3), ...
    'rx', 'LineWidth', 5, 'MarkerSize', 8);
  plot3(r_W_W_Csimp_ges_t0(IpM,1),r_W_W_Csimp_ges_t0(IpM,2),r_W_W_Csimp_ges_t0(IpM,3), ...
    'ko', 'LineWidth', 5, 'MarkerSize', 8);
  plot3(r_W_W_Csimp_ges_t0(:,1),r_W_W_Csimp_ges_t0(:,2),r_W_W_Csimp_ges_t0(:,3), ...
    'k-', 'LineWidth', 1);
  % jeweils Textmarkierungen
  for iIpM = IpM'
    text(r_W_W_C1_ges_t0(iIpM,1),r_W_W_C1_ges_t0(iIpM,2),r_W_W_C1_ges_t0(iIpM,3), ...
      sprintf('-- --$t=%1.3f$', sl.t(iIpM)), 'interpreter', 'latex');
    text(r_W_W_C2_ges_t0(iIpM,1),r_W_W_C2_ges_t0(iIpM,2),r_W_W_C2_ges_t0(iIpM,3), ...
      sprintf('-- --$t=%1.3f$', sl.t(iIpM)), 'interpreter', 'latex');
    text(r_W_W_Csimp_ges_t0(iIpM,1),r_W_W_Csimp_ges_t0(iIpM,2),r_W_W_Csimp_ges_t0(iIpM,3), ...
      sprintf('-- --$t=%1.3f$', sl.t(iIpM)), 'interpreter', 'latex');
  end
  % plot3(r_W_W_Csim_ges_t0(:,1),r_W_W_Csim_ges_t0(:,2),r_W_W_Csim_ges_t0(:,3),'ko', 'LineWidth', 5, 'MarkerSize', 8);
  
  % Kraftwirklinien einzeichnen
  
%   l=0.01;
%   xmatrix = [WL1_ges_t0(:,1), WL1_ges_t0(:,1)-l*WL1_ges_t0(:,4), NaN(nt,2)]'; 
%   ymatrix = [WL1_ges_t0(:,2), WL1_ges_t0(:,2)-l*WL1_ges_t0(:,5), NaN(nt,2)]';
%   zmatrix = [WL1_ges_t0(:,3), WL1_ges_t0(:,3)-l*WL1_ges_t0(:,6), NaN(nt,2)]';
%   plot3(xmatrix(:),ymatrix(:),zmatrix(:), 'k-');
  

  
%   plot3(WL1_ges_t0(:,1),WL1_ges_t0(:,2),WL1_ges_t0(:,3), 'c^');
%   
%   plot3(WL1_ges_t0(:,1),WL1_ges_t0(:,2),WL1_ges_t0(:,3), 'c^');
%   
%   
%   plot3(WL2_ges_t0(:,1),WL2_ges_t0(:,2),WL2_ges_t0(:,3), 'cv');
%   
%   
%   
%   plot3(WL1_ges(:,1),WL1_ges(:,2),WL1_ges(:,3), 'g^');
%   plot3(WL2_ges(:,1),WL2_ges(:,2),WL2_ges(:,3), 'gv');
%   l=2000;
% 
%   hdl=quiver3(r_W_W_C1_ges_t0(:,1),r_W_W_C1_ges_t0(:,2),r_W_W_C1_ges_t0(:,3), l*WL1_ges_t0(:,4), l*WL1_ges_t0(:,5), l*WL1_ges_t0(:,6));
%   set(hdl,'MaxHeadSize', 10, 'MarkerSize', 10)
%   quiver3(WL1_ges_t0(:,1),WL1_ges_t0(:,2),WL1_ges_t0(:,3),-l*WL1_ges_t0(:,4),-l*WL1_ges_t0(:,5),-l*WL1_ges_t0(:,6));
%   quiver3(WL2_ges_t0(:,1),WL2_ges_t0(:,2),WL2_ges_t0(:,3),WL2_ges_t0(:,4),WL2_ges_t0(:,5),WL2_ges_t0(:,6));
%   
%   % Kraftwirklinien direkt am Kollisionsort
%   quiver3(WL1_ges_t0(:,1),WL1_ges_t0(:,2),WL1_ges_t0(:,3), l*WL1_ges_t0(:,4), l*WL1_ges_t0(:,5), l*WL1_ges_t0(:,6));
end


%% Störbeobachter: Vergleiche externe Kraft auf Gelenkmoment
figure(1002);clf;set(1002, 'Name', 'tauJ_obs_cmp', 'NumberTitle', 'off');
for i = 1:30
  subplot(5,6,i);hold on;
  stairs(sl.t, sl.tau_obs(:,i));
  plot(sl.t, sl.tauJ_ext_sum(:,i));
  % Grenzen
  plot(sl.t([1, end]), -ones(2,1)*tauJ_obs_thresh(i), 'r--');
  plot(sl.t([1, end]),  ones(2,1)*tauJ_obs_thresh(i), 'r--');
  grid on;
  ylabel(sprintf('\\tau_{%d}', i));
  title(AS.JN{i}, 'interpreter', 'none')
  ylim(tauJ_obs_thresh(i)*[-2;2]);
end
legend({'obs', 'ext'});
linkxaxes

%% Bilder Speichern
saveas(1, fullfile(exp_pfad, 'coll_est_leg_sphere_cd_ic_raw.fig'));
saveas(4, fullfile(exp_pfad, 'coll_est_leg_sphere_legcad_raw.fig'));
saveas(8, fullfile(exp_pfad, 'coll_est_leg_sphere_forces_raw.fig'));
saveas(10, fullfile(exp_pfad, 'coll_est_leg_sphere_xyzcoll_raw.fig'));

return
%% Versuch graphisch wiedergeben (in CoSim)
qJ_t0_gzb = atlas5_TransferQFromSimulinkToGazebo(sl.q(1,:));
base_pose_quat_t0 = sl.xq_base(1,:)';
obj1_pose_quat_t0 = sl.obj_pose(1,:)';
obj2_pose_quat_t0 = [0 0 100 1 0 0 0]';

simin_qJ_atlas = struct('time', sl.t, ...
    'signals', struct('values', sl.q, 'dimensions', 30), ...
    'name', 'qJ');
  
simin_xq_atlas = struct('time', sl.t, ...
    'signals', struct('values', sl.xq_base, 'dimensions', 7), ...
    'name', 'xq_atlas');
  
simin_xq_sphere = struct('time', sl.t, ...
    'signals', struct('values', sl.obj_pose, 'dimensions', 7), ...
    'name', 'xq_atlas');

% Kiste ist versteckt
simin_xq_box = struct('time', 0, ...
    'signals', struct('values', [0 0 100 1 0 0 0], 'dimensions', 7), ...
    'name', 'xq_atlas');
  
sim('atlas5_wbody_sphere_box_replay', 'StopTime', sprintf('%1.5e', sl.t(end)), ...
  'SimulationMode', 'normal');
