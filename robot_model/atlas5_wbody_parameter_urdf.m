% Return URDF Parameter of the Atlas Robot (v5) whole body (hard coded)
% Order: torso, left leg, right leg, left arm, right arm
% 
% Output:
% rSges_num [31x3]
%   Center of Gravity of all indepedent bodies in urdf frames
% m_num [31x1]
%   masses of all indepedent bodies
% Iges_num  [31x6]
%   inertia of all indepedent bodies
%     rows: bodies
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz
% 
% Sources:
% [1] atlas_v5_simple_shapes_with_head.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Universität Hannover

function [rSges_num_wbody, m_num_wbody, Iges_num_wbody] ...
  = atlas5_wbody_parameter_urdf()

%% URDF Parameter
[~, rSges_num_torso, m_num_torso, Iges_num_torso] = atlas5_torso_parameter_urdf();
[~, rSges_num_lleg, m_num_lleg, Iges_num_lleg] = atlas4_leg_parameter_urdf(true);
[~, rSges_num_rleg, m_num_rleg, Iges_num_rleg] = atlas4_leg_parameter_urdf(false);
[~, rSges_num_larm, m_num_larm, Iges_num_larm] = atlas5_arm_parameter_urdf(true);
[~, rSges_num_rarm, m_num_rarm, Iges_num_rarm] = atlas5_arm_parameter_urdf(false);

m_num_wbody = [m_num_torso; m_num_lleg; m_num_rleg; m_num_larm; m_num_rarm];
rSges_num_wbody = [rSges_num_torso; rSges_num_lleg; rSges_num_rleg; rSges_num_larm; rSges_num_rarm];
Iges_num_wbody = [Iges_num_torso; Iges_num_lleg; Iges_num_rleg; Iges_num_larm; Iges_num_rarm];