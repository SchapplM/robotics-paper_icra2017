% function for the compensation of dynamic and gravitational forces in a
% force/torque sensor in the atlas robot v5. This is a wrapper for
% forcesensor_dyn_comp using the kinematics and inverse dynamics. Inputs
% are the robot state and the sensor data (position, attached
% mass, com, inertia, containing links, raw measurements). The number of
% sensors NS cannot be bigger than 30.
% 
% Input:
% F_S_raw_ges [6xNS]
%   measured and uncompensated force in sensor frame
% q [36x1]
%   generalised coordinates of the atlas robot (position, orientation
%   (rpy), joint angles
% qD [36x1]
%   velocities
% qDD [36x1]
%   accelerations
% g_world [3x1]
%   gravity vector in world coordinates
% LI_S_ges [1xNS]
%   indices (1 - NL, 1 = base) of the links containing the sensors
% r_LI_S_ges [3xNS]
%   vectors from respective link frames to sensors
% m_S_ges [1xNS]
%   masses of partial links directly attached to the sensors
% r_LI_S_c_ges [3xNS]
%   vectors from sensors to coms of partial links directly attached to the
%   sensors
% I_LI_S_c_ges [NSx6]
%   inertias of partial links directly attached to the sensors, the inertia
%   is formulated around the coms of partial links directly attached to the
%   sensors
% R_LI_S_stack [NSx9]
%   rotation matrices from link frames to sensor frames for each sensor,
%   each row represents one rotation matrix

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-11
% (c) Institut für Regelungstechnik, Universität Hannover

function [F_S_cmp_ges, F_W_cmp_ges] = atlas5_forcesensor_dyn_comp(F_S_raw_ges, q, qD, qDD,...
  g_world, LI_S_ges, r_LI_S_ges, m_S_ges, r_LI_S_c_ges, I_LI_S_c_ges, R_LI_S_stack)
%#codegen
assert(isa(F_S_raw_ges,'double') && isreal(F_S_raw_ges) && all(size(F_S_raw_ges) <= [6 30]), ...
  'atlas5_forcesensor_dyn_comp: raw measured forces F_S_raw_ges have to be <= [6x30] double');
assert(isa(q,'double') && isreal(q) && all(size(q) == [36 1]), ...
  'atlas5_forcesensor_dyn_comp: q has to be [36x1] double');
assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [36 1]), ...
  'atlas5_forcesensor_dyn_comp: qD has to be [36x1] double');
assert(isa(qDD,'double') && isreal(qDD) && all(size(qDD) == [36 1]), ...
  'atlas5_forcesensor_dyn_comp: qDD has to be [36x1] double');
assert(isa(g_world,'double') && isreal(g_world) && all(size(g_world) == [3 1]), ...
  'atlas5_forcesensor_dyn_comp: g_world has to be [3x1] double');
assert(isa(LI_S_ges,'double') && isreal(LI_S_ges) && all(size(LI_S_ges) <= [1 30]), ...
  'atlas5_forcesensor_dyn_comp: LI_S_ges has to be <= [1x30] double');
assert(isa(r_LI_S_ges,'double') && isreal(r_LI_S_ges) && all(size(r_LI_S_ges) <= [3 30]), ...
  'atlas5_forcesensor_dyn_comp: r_LI_S_ges has to be <= [3x30] double');
assert(isa(m_S_ges,'double') && isreal(m_S_ges) && all(size(m_S_ges) <= [1 30]), ...
  'atlas5_forcesensor_dyn_comp: m_S_ges has to be <= [1 30] double');
assert(isa(r_LI_S_c_ges,'double') && isreal(r_LI_S_c_ges) && all(size(r_LI_S_c_ges) <= [3 30]), ...
  'atlas5_forcesensor_dyn_comp: r_LI_S_c_ges has to be <= [3x30] double');
assert(isa(I_LI_S_c_ges,'double') && isreal(I_LI_S_c_ges) && all(size(I_LI_S_c_ges) <= [30 6]), ...
  'atlas5_forcesensor_dyn_comp: I_LI_S_ges has to be <= [30x6] double');
assert(isa(R_LI_S_stack,'double') && isreal(R_LI_S_stack) && all(size(R_LI_S_stack) <= [30 9]), ...
  'atlas5_forcesensor_dyn_comp: R_LI_S_stack has to be <= [30x9] double');
NS = length(LI_S_ges);
assert(NS == length(F_S_raw_ges(1,:)) && NS == length(r_LI_S_ges(1,:)) && NS == length(m_S_ges) &&...
       NS == length(r_LI_S_c_ges(1,:)) && NS == length(I_LI_S_c_ges(:,1)) && NS == length(R_LI_S_stack(:,1)),...
       ['atlas5_forcesensor_dyn_comp: inconssistent dimensions, vectors F_S_raw_ges, '...
       'LI_S_ges, r_LI_S_ges, m_S_ges, r_LI_S_c_ges, I_LI_S_ges and R_LI_S_stack have to have the same length']);

%% Kinematik- und Dynamikparameter
[a_mdh, d_mdh, alpha_mdh, q_offset_mdh, b_mdh, beta_mdh, v_mdh, ...
  rSges_num_mdh, m_num_mdh, Iges_mdh_wbody] = atlas5_wbody_parameter_mdh();
%% Inverse Dynamik
[~, ~, w_i_i_ges, vD_i_i_ges, wD_i_i_ges, f_i_i_ges, n_i_i_ges] =...
  atlas5_wbody_invdyn_floatb_eulangrpy_nnew_vp1(q(7:36), qD(7:36), qDD(7:36),...
  q(4:6), qD(1:6), qDD(1:6)+[-g_world;zeros(3,1)], alpha_mdh, a_mdh, d_mdh,...
  q_offset_mdh, b_mdh, beta_mdh, v_mdh, m_num_mdh, rSges_num_mdh,...
  Iges_mdh_wbody); % add negative g-vector to take gravitational forces into account
%% Kinematik und Transformation in Welt-KS
T_mdh = atlas5_wbody_fkine_mdh_num(q(7:36)');
R_W_0 = eulxyz2r([q(4); q(5); q(6)]);
R_LI_S = NaN(3,3,NS);
for i = 1:NS
  R_LI_S(:,:,i) = reshape(R_LI_S_stack(i, :),3,3);
end
% initialzation
F_W_S_raw_ges = zeros(6, NS);
r_W_LI_S_ges  = zeros(3, NS);
r_W_S_c_ges   = zeros(3, NS);
I_W_S_ges     = zeros(NS, 6);
w_W_S_ges     = zeros(3, NS);
vD_W_S_ges    = zeros(3, NS);
wD_W_S_ges    = zeros(3, NS);
r_W_LI_ges    = zeros(3, 31);
F_W_LI_ges    = zeros(6, 31);
% transform sensor data to world frame
for i = 1:NS
  R_W_LI             = R_W_0 * T_mdh(1:3,1:3,LI_S_ges(i));
  R_W_S              = R_W_LI * R_LI_S(:,:,i);
  F_W_S_raw_ges(:,i) = rotate_wrench(F_S_raw_ges(:,i), R_W_S);
  r_W_LI_S_ges(:,i)  = R_W_LI*r_LI_S_ges(:,i);
  r_W_S_c_ges(:,i)   = R_W_LI*r_LI_S_c_ges(:,i);
  I_W_S_ges(i,:)     = inertiamatrix2vector(R_W_LI * ...
                               inertiavector2matrix(I_LI_S_c_ges(i,:))*R_W_LI');
  w_W_S_ges(:,i)     = R_W_LI*w_i_i_ges(:, LI_S_ges(i));
  wD_W_S_ges(:,i)    = R_W_LI*wD_i_i_ges(:, LI_S_ges(i));
  vD_W_S_ges(:,i)    = R_W_LI*vD_i_i_ges(:, LI_S_ges(i)) +...
                         cross(wD_W_S_ges(:,i), r_W_LI_S_ges(:,i)) +...
                         cross(w_W_S_ges(:,i), cross(w_W_S_ges(:,i),...
                           r_W_LI_S_ges(:,i))) + g_world;
                      % correct for gravity, because -g_world was added
                      % earlier in inverse dynamics to take into account
                      % the gravitational forces
end
% transform joint forces to world frame
F_W_LI_ges(:,1) = rotate_wrench([f_i_i_ges(:,1); n_i_i_ges(:,1)], R_W_0);
for i = 2:31
  r_W_LI_ges(:,i) = R_W_0 * T_mdh(1:3,4,i);
  R_W_i           = R_W_0 * T_mdh(1:3,1:3,i);
  F_W_LI_ges(:,i) = rotate_wrench([f_i_i_ges(:,i); n_i_i_ges(:,i)], R_W_i);
end

%% Kompensation und Ruecktransformation
F_W_cmp_ges = forcesensor_dyn_comp(F_W_S_raw_ges, LI_S_ges, r_W_LI_S_ges,...
  m_S_ges, r_W_S_c_ges, I_W_S_ges, g_world, w_W_S_ges, vD_W_S_ges,...
  wD_W_S_ges, r_W_LI_ges, F_W_LI_ges, v_mdh');
F_S_cmp_ges = zeros(size(F_W_cmp_ges));
for i = 1:NS
  R_S_W = R_LI_S(:,:,i)' * T_mdh(1:3,1:3,LI_S_ges(i))' * R_W_0';
  F_S_cmp_ges(:,i) = rotate_wrench(F_W_cmp_ges(:,i), R_S_W);
end


