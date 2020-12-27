% Vorher ausführen: atlas5wbody_collisions_statistics_sc_filter.m
% Auswertung des Versuchs, bei dem das Bein von einer Kugel getroffen wird.
% 
% TODO: Allgemeinen Teil in eigenes Skript auslagern

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc

%% Init
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));

exp_pfad = fullfile(collhdl_pfad,'paper','figures','colltest_single_filter');

data_file = fullfile(collhdl_pfad, 'results', 'atlas5wbody_collisions_statistics_sc_filter', 'result.mat');
ResultStruct = load(data_file);

AS = atlas_const(5);
nt = length(ResultStruct.t);
I_plot = 1:1e-3/mean(diff(ResultStruct.t)):nt;
iRS = 291; % Resultatnummer fuer Auswertung

% Praepariere Plotdaten
for i=1:length(ResultStruct.t)
  ResultStruct.F_c_W_t(i,:,iRS) = ResultStruct.F_c_W(iRS,:)*ResultStruct.tau_t(i);
  ResultStruct.tau_wbody_t(i,:,iRS) = ResultStruct.tau_wbody(iRS,:)*ResultStruct.tau_t(i);
end

%% Zeitverlauf der Kontaktpunktschätzungen
figure(10);clf;
for i = 1:4
  subplot(4,1,i);hold on
  if i < 4
  stairs(ResultStruct.t, repmat(ResultStruct.r_0_0_C1(iRS,i), length(ResultStruct.t),1),'b-');
  stairs(ResultStruct.t, ResultStruct.r_0_0_C2(:,i,iRS),'r-');
  % stairs(ResultStruct.t(I_plot), r_W_W_Csim_ges(I_plot,i),'k-');
  stairs(ResultStruct.t, ResultStruct.r_0_0_C3(:,i,iRS),'k-');
  
%   plot(ResultStruct.t, r_W_W_C1_ges_t0(:,i),'b--');
%   plot(ResultStruct.t, r_W_W_C2_ges_t0(:,i),'r--');
%   plot(ResultStruct.t, r_W_W_Csim_ges_t0(:,i),'k--');
  

  ylabel(sprintf('r %s', char(119+i)));
  else    
    % Fehler berechnen
    ex1 = sum(sqrt((ResultStruct.r_0_0_C2(:,:,iRS)-repmat(ResultStruct.r_0_0_C1(iRS,:),length(ResultStruct.t),1)).^2),2);
    ex2 = sum(sqrt((ResultStruct.r_0_0_C3(:,:,iRS)-repmat(ResultStruct.r_0_0_C1(iRS,:),length(ResultStruct.t),1)).^2),2);
    
    stairs(ResultStruct.t(I_plot), ex1(I_plot));
    stairs(ResultStruct.t(I_plot), ex2(I_plot));
    
    ylabel('ex [m]');
  end
  grid on
  
end
legend({'r', 'r_{\tau J}', 'r_\mathrm{fs}'});
linkxaxes
xlim(ResultStruct.t([1,end]))
% Test
% any(r_W_W_C1_ges(:,:) == 0)
%% Zeitverlauf der Kraftschätzung
figure(8);clf;
FM_string = {'F', 'M'};
for i = 1:4
  for j = 1:2
    subplot(4,2,sprc2no(4,2,i,j));hold on
    
    if i < 4
    stairs(ResultStruct.t, ResultStruct.F_c_W_t(:,i,iRS),'k-');
    stairs(ResultStruct.t, ResultStruct.F_W_S_ges(:,i,iRS),'b-');
    stairs(ResultStruct.t, ResultStruct.F_c_W_erg(:,3*(j-1)+i,iRS),'r--');
    
    ylabel(sprintf('%s %s [N]', FM_string{j}, char(119+i)));
    
    elseif j == 1
      % Winkelfehler der Kraft berechnen
      ephi1 = NaN(nt,1);
      ephi2 = NaN(nt,1);
      for kt = 1:nt
        ephi1(kt) = acos( ResultStruct.F_c_W(iRS,:)*ResultStruct.F_c_W_erg(kt,1:3,iRS)'/ ...
                         (norm(ResultStruct.F_c_W(iRS,:))*norm(ResultStruct.F_c_W_erg(kt,1:3,iRS))) ) *180/pi;
        ephi2(kt) = acos( ResultStruct.F_c_W(iRS,:)*ResultStruct.F_c_W(iRS,:)'/ ...
                         (norm(ResultStruct.F_c_W(iRS,:))*norm(ResultStruct.F_c_W(iRS,:))) ) *180/pi;
      end
      stairs(ResultStruct.t, ephi1, 'b-');
      stairs(ResultStruct.t, ephi2, 'r--');
      ylabel('e phi [deg]');
    end
    
    grid on
  end
end
legend({'M1', 'M2', 'sim'});
linkxaxes
xlim(ResultStruct.t([1,end]))

%% Störbeobachter: Vergleiche externe Kraft auf Gelenkmoment
figure(1002);clf;set(1002, 'Name', 'tauJ_obs_cmp', 'NumberTitle', 'off');
for i = 1:30
  subplot(5,6,i);hold on;
  stairs(ResultStruct.t, ResultStruct.tau_wbody_t(:,:,iRS));
%   plot(ResultStruct.t, ResultStruct.tauJ_ext_sum(:,i));
  % Grenzen
  plot(ResultStruct.t([1, end]), -ones(2,1)*1e-3, 'r--');
  plot(ResultStruct.t([1, end]),  ones(2,1)*1e-3, 'r--');
  grid on;
  ylabel(sprintf('\\tau_{%d}', i));
  title(AS.JN{i}, 'interpreter', 'none')
  ylim(1e-3*[-2;2]);
end
legend({'obs', 'ext'});
linkxaxes

%% Bilder Speichern
% saveas(1, fullfile(exp_pfad, 'coll_est_leg_sphere_cd_ic_raw.fig'));
% saveas(4, fullfile(exp_pfad, 'coll_est_leg_sphere_legcad_raw.fig'));
saveas(8, fullfile(exp_pfad, 'colltest_single_filter_FM.fig'));
saveas(10, fullfile(exp_pfad, 'colltest_single_filter_r.fig'));

return
%% Versuch graphisch wiedergeben (in CoSim)
qJ_t0_gzb = atlas5_TransferQFromSimulinkToGazebo(sl.q(1,:));
base_pose_quat_t0 = sl.xq_base(1,:)';
obj1_pose_quat_t0 = sl.obj_pose(1,:)';
obj2_pose_quat_t0 = [0 0 100 1 0 0 0]';

simin_qJ_atlas = struct('time', ResultStruct.t, ...
    'signals', struct('values', sl.q, 'dimensions', 30), ...
    'name', 'qJ');
  
simin_xq_atlas = struct('time', ResultStruct.t, ...
    'signals', struct('values', sl.xq_base, 'dimensions', 7), ...
    'name', 'xq_atlas');
  
simin_xq_sphere = struct('time', ResultStruct.t, ...
    'signals', struct('values', sl.obj_pose, 'dimensions', 7), ...
    'name', 'xq_atlas');

% Kiste ist versteckt
simin_xq_box = struct('time', 0, ...
    'signals', struct('values', [0 0 100 1 0 0 0], 'dimensions', 7), ...
    'name', 'xq_atlas');
  
sim('atlas5_wbody_sphere_box_replay', 'StopTime', sprintf('%1.5e', ResultStruct.t(end)), ...
  'SimulationMode', 'normal');
