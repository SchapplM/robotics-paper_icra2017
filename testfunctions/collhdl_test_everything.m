% Prüfe, ob alle Funktionen und Skripte der Toolbox funktionieren

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

humanoid_collisionhandling_path

atlas_collhdl_mex_all

%% Ersatzkörper erstellen
atlas_contactbody_parse
atlas_contactbody_properties

%% Untersuchungen mit realen Daten
atlas5wbody_collisions_statistics_sc
atlas5wbody_collisions_statistics_sc_filter

atlas5wbody_collisions_statistics_mc
atlas5wbody_collisions_statistics_mc_manip

%% Darauf aufbauende Bilder aus Veröffentlichung generieren

% Basierend auf atlas5wbody_collisions_statistics_mc.m
% Fig. 8 im ICRA-Paper:
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
run(fullfile('', collhdl_pfad, 'paper','figures','collest_multi_stat_2coll_error_rank',...
  'histogram_error_rank.m'));
% Fig. 7 im ICRA-Paper:
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
run(fullfile('', collhdl_pfad, 'paper','figures','collest_multi_stat_2coll_rankmat',...
  'rankmat_fig_gen.m'));

% Basierend auf atlas5wbody_collisions_statistics_mc_manip.m
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
run(fullfile('', collhdl_pfad, 'paper','figures','collest_multi_stat_manipcoll_error',...
  'error_rank_manipcoll_fig_gen.m'));
% Fig. 9 im ICRA-Paper:
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
run(fullfile('', collhdl_pfad, 'paper','figures','collest_multi_stat_manipcoll_error',...
  'error_rank_manipcoll_hist_fig_gen.m'));

% Basierend auf atlas5wbody_collisions_statistics_sc.m
% Fig. 6 im ICRA-Paper:
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
run(fullfile('', collhdl_pfad, 'paper','figures','collest_single_allresults_nosim',...
  'collest_sc_allres_nosim_picturegen.m'));

% Basierend auf atlas5wbody_collisions_statistics_sc_filter.m
% Fig. 10 im ICRA-Paper:
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
run(fullfile('', collhdl_pfad, 'paper','figures','colltest_single_filter',...
  'colltest_single_filter_eval.m'));
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
run(fullfile('', collhdl_pfad, 'paper','figures','colltest_single_filter',...
  'colltest_single_filter_format.m'));
