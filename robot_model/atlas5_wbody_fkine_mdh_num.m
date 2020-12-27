% Calculate Forward Kinematics for Atlas Robot (v5) whole body in MDH frames
% 
% Input:
% q [1x30]
%   Joint Angles [rad] (back [1x3], neck [1x1], l_leg [1x6], r_leg [1x6],
%   l_arm [1x7], r_arm [1x7]) 
% 
% Output:
% T_c_mdh [4x4x35]
%   homogenious transformation matrices for each body frame (MDH)
%    1: pelvis -> pelvis
%    2: pelvis -> ltorso
%    3: pelvis -> mtorso
%    4: pelvis -> utorso   %anderer als vorher, aufgrund von Verzweigung!
%    5: pelvis -> head
% 
%    6: pelvis -> l_uglut
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
%   32: pelvis -> left_leg_endpoint
%   33: pelvis -> right_leg_endpoint
%   34: pelvis -> left_arm_endpoint
%   35: pelvis -> right_arm_endpoint
% 
% Sources:
% [1] atlas_v5.urdf
% [2] [KhalilKle1986]

% Philipp Karbowski, 2015-05
% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function T_c_mdh = atlas5_wbody_fkine_mdh_num(q)

%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 30]), ...
  'Joint angles q have to be [1x30] double');

%% Calculation

% Initialize Variables
[a, d, alpha, q_offset, b, beta, v] = atlas5_wbody_parameter_mdh();

% assign cumulated transformation matrices
T_c_mdh = NaN(4,4,35);

% set base Transformation (pelvis)
T_c_mdh(:,:,1) = eye(4);

% [2] Equ. (9), (7)
% Alle Gelenkwinkel durchgehen
for i = 1:30
  j = v(i)+1;
  T_c_mdh(:,:,i+1) = T_c_mdh(:,:,j) * trotz(beta(i)) * ... 
                     transl([0;0;b(i)]) * trotx(alpha(i)) * ...
                     transl([a(i);0;0]) * trotz(q(i)+q_offset(i)) ...
                     * transl([0;0;d(i)]);
end

% left leg endpoint: foot transformation as in urdf
T_c_mdh(:,:,32) = T_c_mdh(:,:,11)*troty(pi/2)*trotz(pi);
% right leg endpoint: 
T_c_mdh(:,:,33) = T_c_mdh(:,:,17)*troty(pi/2)*trotz(pi);
% left arm endpoint: hand transformation as in urdf
T_c_mdh(:,:,34) = T_c_mdh(:,:,24) * troty(pi/2) * trotz(pi/2) * trotz(pi);
% right arm endpoint:
T_c_mdh(:,:,35) = T_c_mdh(:,:,31) * troty(pi/2) * trotz(pi/2);
