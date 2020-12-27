% function for calculating the measurement value of a perfect force/torque
% sensor within the structure of an atlas robot. Inputs are the robot
% state, external forces with link and point of action and sensor
% information. The number of sensors NS cannot be greater than 30. The
% number of external forces NF cannot be greater than 50. Assumption:
% forcessensor is placed in the origin of an mdh coordinate system. Todo:
% get rid of assumption.
%
% Input:
% q [36x1]
%   generalised coordinates of the atlas robot (position, orientation
%   (rpy), joint angles
% qD [36x1]
%   velocities
% qDD [36x1]
%   accelerations
% F_W_ext_ges [6xNF]
%   external forces in world frame
% r_i_F_ext_ges [3xNF]
%   point of action for external forces
% LI_F_ext_ges [1xNF]
%   link index of the links, rthe external forces are acting on
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
% 
% Output:
% F_S_raw_ges [6xNS]
%   sensor forces in sensor frame

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-11
% (c) Institut für Regelungstechnik, Universität Hannover

function F_S_raw_ges = atlas5_forcesensor(q, qD, qDD, F_W_ext_ges,...
  r_i_F_ext_ges, LI_F_ext_ges, g_world, LI_S_ges, r_LI_S_ges, m_S_ges,...
  r_LI_S_c_ges, I_LI_S_c_ges, R_LI_S_stack)
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [36 1]), ...
  'atlas5_forcesensor: q has to be [36x1] double');
assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [36 1]), ...
  'atlas5_forcesensor: qD has to be [36x1] double');
assert(isa(qDD,'double') && isreal(qDD) && all(size(qDD) == [36 1]), ...
  'atlas5_forcesensor: qDD has to be [36x1] double');
assert(isa(F_W_ext_ges,'double') && isreal(F_W_ext_ges) && all(size(F_W_ext_ges) <= [6 50]), ...
  'atlas5_forcesensor: F_W_ext_ges has to be <= [6x50] double');
assert(isa(r_i_F_ext_ges,'double') && isreal(r_i_F_ext_ges) && all(size(r_i_F_ext_ges) <= [3 50]), ...
  'atlas5_forcesensor: r_i_F_ext_ges has to be <= [3x50] double');
assert(isa(LI_F_ext_ges,'double') && isreal(LI_F_ext_ges) && all(size(LI_F_ext_ges) <= [1 50]), ...
  'atlas5_forcesensor: LI_F_ext_ges has to be <= [1 50] double');
assert(isa(g_world,'double') && isreal(g_world) && all(size(g_world) == [3 1]), ...
  'atlas5_forcesensor: g_world has to be [3x1] double');
assert(isa(LI_S_ges,'double') && isreal(LI_S_ges) && all(size(LI_S_ges) <= [1 30]), ...
  'atlas5_forcesensor: LI_S_ges has to be <= [1x30] double');
assert(isa(r_LI_S_ges,'double') && isreal(r_LI_S_ges) && all(size(r_LI_S_ges) <= [3 30]), ...
  'atlas5_forcesensor: r_LI_S_ges has to be <= [3x30] double');
assert(isa(m_S_ges,'double') && isreal(m_S_ges) && all(size(m_S_ges) <= [1 30]), ...
  'atlas5_forcesensor: m_S_ges has to be <= [1 30] double');
assert(isa(r_LI_S_c_ges,'double') && isreal(r_LI_S_c_ges) && all(size(r_LI_S_c_ges) <= [3 30]), ...
  'atlas5_forcesensor: r_LI_S_c_ges has to be <= [3x30] double');
assert(isa(I_LI_S_c_ges,'double') && isreal(I_LI_S_c_ges) && all(size(I_LI_S_c_ges) <= [30 6]), ...
  'atlas5_forcesensor: I_LI_S_ges has to be <= [30x6] double');
assert(isa(R_LI_S_stack,'double') && isreal(R_LI_S_stack) && all(size(R_LI_S_stack) <= [30 9]), ...
  'atlas5_forcesensor: R_LI_S_stack has to be <= [30x9] double');
NS = length(LI_S_ges);
assert(NS == length(r_LI_S_ges(1,:)) && NS == length(m_S_ges) &&...
       NS == length(r_LI_S_c_ges(1,:)) && NS == length(I_LI_S_c_ges(:,1)) &&...
       NS == length(R_LI_S_stack(:,1)),...
       ['atlas5_forcesensor: inconssistent dimensions, vectors F_S_raw_ges, '...
       'LI_S_ges, r_LI_S_ges, m_S_ges, r_LI_S_c_ges, I_LI_S_ges and R_LI_S_stack have to have the same length']);
NF = length(LI_F_ext_ges);
assert(NF == length(F_W_ext_ges(1,:)) && NF == length(r_i_F_ext_ges(1,:)),...
  ['atlas5_forcesensor: inconsistent dimensions, vectors LI_F_ext_ges, '...
   'F_W_ext_ges and r_i_F_ext_ges have to have the same length']);

%% Kinematik- und Dynamikparameter
[a_mdh, d_mdh, alpha_mdh, q_offset_mdh, b_mdh, beta_mdh, v_mdh, ...
  rSges_num_mdh, m_num_mdh, Iges_mdh_wbody] = atlas5_wbody_parameter_mdh();
%% Inverse Dynamik
[~, ~, ~, ~, ~, f_i_i_ges, n_i_i_ges] =...
  atlas5_wbody_invdyn_floatb_eulangrpy_nnew_vp1(q(7:36), qD(7:36), qDD(7:36),...
  q(4:6), qD(1:6), qDD(1:6)+[-g_world;zeros(3,1)], alpha_mdh, a_mdh, d_mdh,...
  q_offset_mdh, b_mdh, beta_mdh, v_mdh, m_num_mdh,...
  rSges_num_mdh, Iges_mdh_wbody);
%% Kinematik und Transformation in Welt-KS
T_mdh = atlas5_wbody_fkine_mdh_num(q(7:36)');
R_W_0 = eulxyz2r([q(4); q(5); q(6)]);
R_LI_S = NaN(3,3,NS);
for i = 1:NS
  R_LI_S(:,:,i) = reshape(R_LI_S_stack(i, :),3,3);
end
% initialzation
F_S_raw_ges = zeros(6, NS);
%% calculate sensor measurements in world frame and transform into sensor frame
for i = 1:NS
  R_LI_W           = T_mdh(1:3,1:3,LI_S_ges(i))' * R_W_0';
  R_S_W            = R_LI_S(:,:,i)' *  R_LI_W;
  % cutting force at origin of link coordinate system, only valid because
  % of assumption (without assumption, one has to take the cutting forces
  % at the following links. Maybe, we have to add an argument for the
  % following links later, in the case of multiple following links.
  F_S_raw_ges(:,i) = -rotate_wrench([f_i_i_ges(:,LI_S_ges(i));...
                                     n_i_i_ges(:,LI_S_ges(i))], R_LI_S(:,:,i)');
  followers = get_children_from_parent_list(v_mdh, LI_S_ges(i)-1)+1;
  for j=1:NF
    % add external forces seen by the sensor. Todo1: check wether forces
    % are really seen by the sensor. Todo2: (after Todo1) add interial
    % forces of partial links attached to the sensors. One can compare the
    % projection of the sensor position in link frame and the point of
    % force action onto the connection of the link frame and the following
    % link(s) in order to determine, wether forces are seen by the sensor.
    % For a fully correct solution, one would have to take into account the
    % full geometry of the robot.
    if any(F_W_ext_ges(:,j)) && (any(followers == LI_F_ext_ges(j)) ||...
                                 LI_F_ext_ges(j) == LI_S_ges(i))
      F_S_ext = rotate_wrench(F_W_ext_ges(:,j), R_S_W);
      r_0_F_ext = T_mdh(1:3,4,LI_F_ext_ges(j)) +...
                  T_mdh(1:3,1:3,LI_F_ext_ges(j)) * r_i_F_ext_ges(:,j);
      r_0_S_F_ext = -T_mdh(1:3,4,LI_S_ges(i)) + r_0_F_ext;
      r_S_F_ext   = R_LI_S(:,:,i)' * T_mdh(1:3,1:3,LI_S_ges(i))' *r_0_S_F_ext;
      F_S_raw_ges(:,i) = F_S_raw_ges(:,i) + F_S_ext + [zeros(3,1);...
                           cross(r_S_F_ext, F_S_ext(1:3))];
    end
  end
end
end