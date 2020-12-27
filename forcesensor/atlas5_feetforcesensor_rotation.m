% Virtueller Fußkraftsensor für Atlas v5
% Einfachste Modellierung: Rotiere die externe Kraft ins Fuß-KS
% 
% Eingabe:
% base_pose [7x1]
%   Position und Orientierung der Basis (als Quaternion)
%   r_W_0_W, quat(R_W_0)
% qJ [30x1]
%   Gelenkwinkel
% % F_left_W, F_right_W [6x1]
%   Kraft auf linken und rechten Fuß (im Welt-KS)
% 
% Ausgabe:
% % F_left_F, F_right_F [6x1]
%   Kraft auf linken und rechten Fuß (im Fuß-KS = Sensor-KS)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function [F_left_F, F_right_F] = atlas5_feetforcesensor_rotation(base_pose, qJ, F_left_W, F_right_W)
%% Init
%#codegen
assert(isa(base_pose,'double') && isreal(base_pose) && all(size(base_pose) == [7 1]), ...
  'base_pose has to be [7x1] double');
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [30 1]), ...
  'Joint angles q have to be [30x1] double');
assert(isa(F_left_W,'double') && isreal(F_left_W) && all(size(F_left_W) == [6 1]), ...
  'Foot wrench F_left_W has to be [6x1] double');
assert(isa(F_right_W,'double') && isreal(F_right_W) && all(size(F_right_W) == [6 1]), ...
  'Foot wrench F_right_W has to be [6x1] double');
%% Kinematik
R_W_0 = quat2r(base_pose(4:7)');

T_0_Fl = atlas5_wbody_leg_fkine_mdh_sym(qJ', true);
T_0_Fr = atlas5_wbody_leg_fkine_mdh_sym(qJ', false);
R_W_Fl = R_W_0 * T_0_Fl(1:3, 1:3);
R_W_Fr = R_W_0 * T_0_Fr(1:3, 1:3);

%% Rotation der Kraft ins Fuß-KS
F_left_F = rotate_wrench(F_left_W, R_W_Fl');
F_right_F = rotate_wrench(F_right_W, R_W_Fr');