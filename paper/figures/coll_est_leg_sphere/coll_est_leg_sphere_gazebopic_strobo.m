% Starten der Gazebo-CoSim zu bestimmten Zeiten des Versuchs zum Erstellen
% von Bildschirmfotos
% Die CoSim wird mehrfach mit einem eingefrorenen Bild gestartet. Daraufhin
% müssen händisch Bildschirmfotos erstellt werden.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc

%% Init
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));

exp_pfad = fullfile(collhdl_pfad, 'paper','figures','coll_est_leg_sphere');

% data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160713', 'distobs2test5_Koll18res.mat');
% data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160714_2045', 'distobs2test5_Koll03res.mat');
% data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160714_2056', 'distobs2test5_Koll11res.mat');
data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160721_1359', 'distobs2test5_Koll01res.mat');
load(data_file);

%% Versuch graphisch wiedergeben (in CoSim)

t_scrshot = [0.54, 0.56, 0.58, 0.60, 0.62, 0.64, 0.66, 0.68];
nt = length(sl.t);
for i = 1:length(t_scrshot)

  [~, it] = min(abs(sl.t-t_scrshot(i)));
  % Simulation starten
  qJ_t0_gzb = atlas5_TransferQFromSimulinkToGazebo(sl.q(1,:));
  base_pose_quat_t0 = sl.xq_base(1,:)';
  obj1_pose_quat_t0 = sl.obj_pose(1,:)';
  obj2_pose_quat_t0 = [0 0 100 1 0 0 0]';

  simin_qJ_atlas = struct('time', sl.t(1:10), ...
      'signals', struct('values', repmat(sl.q(it,:),10,1), 'dimensions', 30), ...
      'name', 'qJ');

  simin_xq_atlas = struct('time', sl.t(1:10), ...
      'signals', struct('values', repmat(sl.xq_base(it,:),10,1), 'dimensions', 7), ...
      'name', 'xq_atlas');

  simin_xq_sphere = struct('time', sl.t(1:10), ...
      'signals', struct('values', repmat(sl.obj_pose(it,:),10,1), 'dimensions', 7), ...
      'name', 'xq_atlas');

  % Kiste ist versteckt
  simin_xq_box = struct('time', 0, ...
      'signals', struct('values', [0 0 100 1 0 0 0], 'dimensions', 7), ...
      'name', 'xq_atlas');

  sim('atlas5_wbody_sphere_box_replay', 'StopTime', sprintf('%1.5e', sl.t(10)), ...
    'SimulationMode', 'normal');

  % Warte auf Benutzereingabe
  fprintf('Bild %d/%d. ', i, length(t_scrshot));
  input('Nächstes Bild anzeigen. Bestätigen...','s')
  fprintf('\n');
end
