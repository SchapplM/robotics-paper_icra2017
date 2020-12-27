% Return MDH Parameter of the Atlas Robot (v5) whole body (hard coded)
% 
% 
% Output:
% a_mdh, d_mdh, alpha_mdh, q_offset_mdh, b_mdh, beta_mdh [30x1]
%   Numeric Values of Kinematic Parameters, MDH-Notation
%
%
% rSges_num [31x3]
%   Center of Gravity of all indepedent bodies in urdf frames
%     rows: bodies
%     columns: Gravity-Vector. Order: x, y, z
% m_num [31x1]
%   masses of all indepedent bodies
% Iges_num  [31x6]
%   inertia of all indepedent bodies 
%     rows: bodies
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz
% 
% Sources:
% [1] atlas_v5.urdf
% [2] [KhalilKle1986]

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Universität Hannover

function [a_mdh, d_mdh, alpha_mdh, q_offset_mdh, b_mdh, beta_mdh, v, ...
  rSges_mdh_wbody, m_wbody, Iges_mdh_wbody] = atlas5_wbody_parameter_mdh
%% URDF Parameter
p_num_torso = atlas5_torso_parameter_urdf();
p_num_l_leg = atlas4_leg_parameter_urdf(true);
p_num_r_leg = atlas4_leg_parameter_urdf(false);
p_num_l_arm = atlas5_arm_parameter_urdf(true);
p_num_r_arm = atlas5_arm_parameter_urdf(false);
[rSges_urdf_wbody, m_wbody, Iges_urdf_wbody] ...
  = atlas5_wbody_parameter_urdf();

%% Initialize MDH Parameter
alpha_mdh = zeros(30,1);
a_mdh = zeros(30,1);
q_offset_mdh = zeros(30,1);
d_mdh = zeros(30,1);
b_mdh = zeros(30,1);
beta_mdh = zeros(30,1);

%% Determine MDH kinematic Parameter

% Vektor mit Nummern der Vorgänger-Körper
v = uint8([0:2, ... % torso
  3, ... % head
  0, 5:9, ... % l_leg
  0, 11:15, ... % r_leg
  3, 17:22, ... % l_arm
  3, 24:29])'; % r_arm

% Torso
alpha_mdh(1) = 0;
a_mdh(1) = p_num_torso(1);
q_offset_mdh(1) = 0;
d_mdh(1) = p_num_torso(2);

alpha_mdh(2) = -pi/2;
a_mdh(2) = 0;
q_offset_mdh(2) = -pi/2;
d_mdh(2) = 0;

alpha_mdh(3) = -pi/2;
a_mdh(3) = p_num_torso(3);
q_offset_mdh(3) = -pi/2;
d_mdh(3) = p_num_l_arm(1);

% neck:
beta_mdh(4) = pi/2; % [2], p. 1178
b_mdh(4) = -p_num_l_arm(1)+p_num_torso(4); % [2], p. 1178
alpha_mdh(4) = pi/2;
a_mdh(4) = p_num_torso(5);

% linkes Bein
[a_l_leg, d_l_leg, alpha_l_leg, q_offset_l_leg] = atlas4_leg_parameter_mdh(true);
alpha_mdh(5) = 0;
a_mdh(5) = p_num_l_leg(1);  
q_offset_mdh(5) = 0;
d_mdh(5) = 0;    
beta_mdh(5) = pi/2;

alpha_mdh(6:10) = alpha_l_leg(2:6);
a_mdh(6:10) = a_l_leg(2:6);
q_offset_mdh(6:10) = q_offset_l_leg(2:6);
d_mdh(6:10) = d_l_leg(2:6);    

% rechtes Bein
[a_r_leg, d_r_leg, alpha_r_leg, q_offset_r_leg] = atlas4_leg_parameter_mdh(false);
alpha_mdh(11) = 0;
a_mdh(11) = p_num_r_leg(1);  
q_offset_mdh(5) = 0;
d_mdh(11) = 0;    
beta_mdh(11) = pi/2;

alpha_mdh(12:16) = alpha_r_leg(2:6);
a_mdh(12:16) = a_r_leg(2:6);
q_offset_mdh(12:16) = q_offset_r_leg(2:6);
d_mdh(12:16) = d_r_leg(2:6);  

% linker Arm
[a_l_arm, d_l_arm, alpha_l_arm, q_offset_l_arm] = atlas5_arm_parameter_mdh(true);
alpha_mdh(17) = -pi/2;
a_mdh(17) = p_num_l_arm(2);  
q_offset_mdh(17) = pi;
d_mdh(17) = p_num_l_arm(3) + p_num_l_arm(7);    

alpha_mdh(18:23) = alpha_l_arm(2:7);
a_mdh(18:23) = a_l_arm(2:7);
q_offset_mdh(18:23) = q_offset_l_arm(2:7);
d_mdh(18:23) = d_l_arm(2:7);             

% rechter Arm
[a_r_arm, d_r_arm, alpha_r_arm, q_offset_r_arm] = atlas5_arm_parameter_mdh(false);
alpha_mdh(24) = -pi/2;
a_mdh(24) = p_num_r_arm(2);
q_offset_mdh(24) = pi;
d_mdh(24) = p_num_r_arm(3) + p_num_l_arm(7);   
alpha_mdh(25:30) = alpha_r_arm(2:7);
a_mdh(25:30) = a_r_arm(2:7);
q_offset_mdh(25:30) = q_offset_r_arm(2:7);
d_mdh(25:30) = d_r_arm(2:7);

%% Calculate MDH dynamic parameters

% Different frame position and orientation
% load mdh parameters from different chains
[rSges_mdh_ubody, Iges_mdh_ubody] = atlas5_ubody_convert_urdf_mdh ...
  (rSges_urdf_wbody([1:5, 18:31],:), Iges_urdf_wbody([1:5, 18:31],:));
[rSges_mdh_lleg, Iges_mdh_lleg] = atlas4_leg_convert_urdf_mdh(true, rSges_urdf_wbody(6:11,:), Iges_urdf_wbody(6:11,:));
[rSges_mdh_rleg, Iges_mdh_rleg] = atlas4_leg_convert_urdf_mdh(true, rSges_urdf_wbody(12:17,:), Iges_urdf_wbody(12:17,:));
% stack output variables
rSges_mdh_wbody = [rSges_mdh_ubody(1:5,:); rSges_mdh_lleg; rSges_mdh_rleg; rSges_mdh_ubody(6:end,:)];
Iges_mdh_wbody = [Iges_mdh_ubody(1:5,:); Iges_mdh_lleg; Iges_mdh_rleg; Iges_mdh_ubody(6:end,:)];