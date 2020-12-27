% function for the compensation of a force/torque sensor in a kinematic
% chain. All entities are expressed in world frame, so that the function is
% completely independent of the robot structure and therefore reusable for
% any robot. You will need to write a wrapper though and the newton-euler
% inverse dynamics for your robot ( for an example, see
% atlas5_forcesensor_dyn_comp). The number of sensors is NS (max 30) and
% the number of links is NL (max 50). Variables containing _LI_(..)_ges
% contain the elements for all links stacked together and are numbered from
% base (1) to end-effector NL. 
% 
% Input:
% F_W_S_raw_ges [6xNS]
%   measured and uncompensated forces of all NS sensors (max 30) in world
%   frame
% LI_S_ges [1xNS]
%   indices (1 - NL, 1 = base) of the links containing the sensors
% r_W_LI_S_ges [3xNS]
%   vectors from respective link frames to sensors
% m_S_ges [1xNS]
%   masses of partial links directly attached to the sensors
% r_W_S_c_ges [3xNS]
%   vectors from sensors to coms of partial links directly attached to the sensors
% I_W_S_ges [NSx6]
%   inertias of partial links directly attached to the sensors
% g_world [3x1]
%   gravity vector in world coordinates
% w_W_S_ges [3xNS]
%   rotational velocities of sensor frames
% vD_W_S_ges [3xNS]
%   translational acceleration of sensor frames
% wD_W_S_ges [3xNS]
%   rotational acceleration of sensor frames
% r_W_LI_ges [3xNL]
%   positions of link frames
% F_W_LI_ges [6xNL]
%   forces at the joints (2:end) and base (1)
% v_mdh [1xNL]
%   predecessor list of all links but the base (mdh notation, 0 - NL-1,
%   0 = base (!))
% 
% Output:
% F_cmp_W_S_ges [6xNS]
%   compensated sensor forces in world frame

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-11
% (c) Institut für Regelungstechnik, Universität Hannover

function F_cmp_W_S_ges = forcesensor_dyn_comp(F_W_S_raw_ges, LI_S_ges,...
  r_W_LI_S_ges, m_S_ges, r_W_S_c_ges, I_W_S_ges, g_world, w_W_S_ges,...
  vD_W_S_ges, wD_W_S_ges, r_W_LI_ges, F_W_LI_ges, v_mdh)
%#codegen
assert(isa(F_W_S_raw_ges,'double') && isreal(F_W_S_raw_ges) && all(size(F_W_S_raw_ges) <= [6 30]), ...
  'forcesensor_dyn_comp: raw measured forces F_W_S_raw_ges have to be <= [6x30] double');
assert(isa(LI_S_ges,'double') && isreal(LI_S_ges) && all(size(LI_S_ges) <= [1 30]), ...
  'forcesensor_dyn_comp: Link indices LI_S_ges have to be <= [1x30] double');
assert(isa(r_W_LI_S_ges,'double') && isreal(r_W_LI_S_ges) && all(size(r_W_LI_S_ges) <= [3 30]), ...
  'forcesensor_dyn_comp: position vectors r_W_LI_S_ges have to be <= [3x30] double');
assert(isa(m_S_ges,'double') && isreal(m_S_ges) && all(size(m_S_ges) <= [1 30]), ...
  'forcesensor_dyn_comp: masses m_S_ges have to be <= [1x30] double');
assert(isa(r_W_S_c_ges,'double') && isreal(r_W_S_c_ges) && all(size(r_W_S_c_ges) <= [3 30]), ...
  'forcesensor_dyn_comp: position vectors r_W_S_c_ges have to be <= [3x30] double');
assert(isa(I_W_S_ges,'double') && isreal(I_W_S_ges) && all(size(I_W_S_ges) <= [30 6]), ...
  'forcesensor_dyn_comp: inertia vectors I_W_S_ges have to be <= [30x6] double');
assert(isa(g_world,'double') && isreal(g_world) && all(size(g_world) == [3 1]), ...
  'forcesensor_dyn_comp: gravity vector g_world has to be [3x1] double');
assert(isa(w_W_S_ges,'double') && isreal(w_W_S_ges) && all(size(w_W_S_ges) <= [3 30]), ...
  'forcesensor_dyn_comp: angular velocities w_W_S_ges have to be <= [3x30] double');
assert(isa(vD_W_S_ges,'double') && isreal(vD_W_S_ges) && all(size(vD_W_S_ges) <= [3 30]), ...
  'forcesensor_dyn_comp: translational accelerations vD_W_S_ges have to be <= [3x30] double');
assert(isa(wD_W_S_ges,'double') && isreal(wD_W_S_ges) && all(size(wD_W_S_ges) <= [3 30]), ...
  'forcesensor_dyn_comp: angular accelerations wD_W_S_ges have to be <= [3x30] double');
assert(isa(r_W_LI_ges,'double') && isreal(r_W_LI_ges) && all(size(r_W_LI_ges) <= [3 50]), ...
  'forcesensor_dyn_comp: positions r_W_LI_ges have to be <= [3x50] double');
assert(isa(F_W_LI_ges,'double') && isreal(F_W_LI_ges) && all(size(F_W_LI_ges) <= [6 50]), ...
  'forcesensor_dyn_comp: raw measured forces F_W_LI_ges have to be <= [6x50] double');
assert(isa(v_mdh,'uint8') && isreal(v_mdh) && all(size(v_mdh) <= [1 50]), ...
  'forcesensor_dyn_comp: predecessor list v_mdh has to be <= [1x50] uint8');
NS = length(LI_S_ges);
assert(NS == length(F_W_S_raw_ges(1,:)) && NS == length(r_W_LI_S_ges(1,:)) && NS == length(m_S_ges) &&...
       NS == length(r_W_S_c_ges(1,:)) && NS == length(I_W_S_ges(:,1)),...
       ['atlas5_forcesensor_dyn_comp: inconssistent dimensions, vectors F_S_raw_ges, '...
       'LI_S_ges, r_W_LI_S_ges, m_S_ges, r_W_S_c_ges and I_W_S_ges have to have the same length']);

F_cmp_W_S_ges = zeros(6,NS);
% Add up inertial forces of all links following the sensor and the partial
% links attached to the sensors
for i = 1:NS
  % sum up all inertial forces of following bodies
  direct_followers_tmp = find(v_mdh == LI_S_ges(i)-1)+1;
  F = F_W_S_raw_ges(:,i);
  r_W_S = r_W_LI_ges(:,LI_S_ges(i)) + r_W_LI_S_ges(:,i);
  for j=1:length(direct_followers_tmp)
    r_W_S_i_next = r_W_LI_ges(:,direct_followers_tmp(j)) - r_W_S;
    F = F + F_W_LI_ges(:,direct_followers_tmp(j)) + [zeros(3,1);...
          cross(r_W_S_i_next, F_W_LI_ges(1:3,direct_followers_tmp(j)))];
  end
  % add intertial force of partial link mounted to the sensor
  m   = m_S_ges(i);
  I   = inertiavector2matrix(I_W_S_ges(i,:));
  r   = r_W_S_c_ges(:,i);
  w   = w_W_S_ges(:,i);
  wD  = wD_W_S_ges(:,i);
  xDD = [vD_W_S_ges(:,i) + cross(wD, r) + cross(w, cross(w, r)); wD];
  F   = F + [m*eye(3) zeros(3,3); m*skew(r) I]*(xDD-[g_world; zeros(3,1)])...
          + [zeros(3,1); cross(w, I*w)];
  F_cmp_W_S_ges(:,i) = F;
end
end