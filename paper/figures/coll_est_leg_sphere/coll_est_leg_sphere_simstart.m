% Starte die Simulation neu mit den gespeicherten Parametern um Daten für
% das Bild zu erzeugen.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover
clear
clc

%% Init
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));

%% Automatische Einstellung (laden der Datei)

% data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160713', 'distobs2test5_Koll18res.mat');
data_file = fullfile(collhdl_pfad, 'results', 'distobs2test5_20160714_2056', 'distobs2test5_Koll11res.mat');
load(data_file);

%% Manuelle Einstellung (falls Messwerte nicht vorhanden)
% Messwerte sind zu groß, um sie ins Repo einzuchecken
usr_simsetting = struct('obj_pose_quat_t0', [-0.4535    2.8792    2.0685 1 0 0 0]', ...
  'rD_W_O_t0', [0.7849   -4.6129         0]', ...
  'r_W_O_t0', [-0.4535    2.8792    2.0685]', ...
  'HC_params_sphere_atlaslinks', [0.1 5.1504e-06 5]');
%% Starten der Simulation
atlas5_wbody_distobs_test2_start5