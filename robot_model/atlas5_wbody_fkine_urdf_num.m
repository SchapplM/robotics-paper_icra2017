% Calculate Forward Kinematics for Atlas Robot (v5) whole body in urdf frames
% 
% Input:
% q [1x30]
%   Joint Angles [rad] (back [1x3], neck [1x1], l_leg [1x6], r_leg [1x6],
%   l_arm [1x7], r_arm [1x7]) 
% 
% Output:
% T_c_urdf [4x4x31]
%   homogenous transformation matrices for each body frame
%    1: pelvis -> pelvis
%    2: pelvis -> ltorso
%    3: pelvis -> mtorso
%    4: pelvis -> utorso
%    5: pelvis -> head
% 
%    6: pelvis -> l_glut
%    7: pelvis -> l_lglut
%    8: pelvis -> l_uleg
%    9: pelvis -> l_lleg
%   10: pelvis -> l_talus
%   11: pelvis -> l_foot
% 
%   12: pelvis -> r_glut
%   13: pelvis -> r_lglut
%   14: pelvis -> r_uleg
%   15: pelvis -> r_lleg
%   16: pelvis -> r_talus
%   17: pelvis -> r_foot
% 
%   18: pelvis -> l_clav
%   19: pelvis -> l_scap
%   20: pelvis -> l_uarm
%   21: pelvis -> l_larm
%   22: pelvis -> l_ufarm
%   23: pelvis -> l_lfarm
%   24: pelvis -> l_hand
% 
%   25: pelvis -> r_clav
%   26: pelvis -> r_scap
%   27: pelvis -> r_uarm
%   28: pelvis -> r_larm
%   29: pelvis -> r_ufarm
%   30: pelvis -> r_lfarm
%   31: pelvis -> r_hand
% 
% Sources:
% [1] atlas_v5.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_urdf = atlas5_wbody_fkine_urdf_num(q)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 30]), ...
  'Joint angles q have to be [1x30] double');
atlas_version = uint8(5);
%% Function call
% Einzelne Ketten. Anfang ist jeweils Ursprung d. Welt-KS
T_torso_c_urdf = atlas_torso_fkine(q(1:4), atlas_version);
T_legL_c_urdf = atlas_leg_fkine(q(5:10), true, atlas_version);
T_legR_c_urdf = atlas_leg_fkine(q(11:16), false, atlas_version);
T_armL_c_urdf = atlas_arm_fkine(q(17:23), true, atlas_version);
T_armR_c_urdf = atlas_arm_fkine(q(24:30), false, atlas_version);

% Arm Transformations start at utorso
% pelvis-> utorso
for i = 1:size(T_armL_c_urdf,3)
  T_armL_c_urdf(:,:,i) = T_torso_c_urdf(:,:,4) * T_armL_c_urdf(:,:,i);
  T_armR_c_urdf(:,:,i) = T_torso_c_urdf(:,:,4) * T_armR_c_urdf(:,:,i);
end

T_c_urdf = NaN(4,4,31);
T_c_urdf(:,:,1:5) = T_torso_c_urdf;
T_c_urdf(:,:,6:11) = T_legL_c_urdf(:,:,2:end);
T_c_urdf(:,:,12:17) = T_legR_c_urdf(:,:,2:end);
T_c_urdf(:,:,18:24) = T_armL_c_urdf(:,:,2:end-1);
T_c_urdf(:,:,25:31) = T_armR_c_urdf(:,:,2:end-1);