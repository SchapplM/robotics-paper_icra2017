% Pfad-Initialisierung

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
% (c) Institut für Regelungstechnik, Universität Hannover

humcollhdl_path = fileparts( mfilename('fullpath') );
addpath(humcollhdl_path);
addpath(fullfile(humcollhdl_path, 'robot_model'));
addpath(fullfile(humcollhdl_path, 'contact'));
addpath(fullfile(humcollhdl_path, 'forcesensor'));
addpath(fullfile(humcollhdl_path, 'misc'));
addpath(fullfile(humcollhdl_path, 'collision_pipeline'));
addpath(fullfile(humcollhdl_path, 'testfunctions'));
