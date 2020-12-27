% Calculate Forward Kinematics for Atlas Robot (v4) in modified DH frames
% 
% Input:
% q [1x7]
%   Joint Angles [rad]
% lr [1x1 logical]
%   true for left, false for right
% 
% Output:
% T_c_mdh [4x4x9]
%   homogenious transformation matrices for each body frame (MDH)
%   1: utorso -> utorso
%   2: utorso -> clav (depending on q1)
%   3: utorso -> scap (depending on q1, q2)
%   4: utorso -> uarm
%   5: utorso -> larm
%   6: utorso -> ufarm
%   7: utorso -> lfarm
%   8: utorso -> hand
%   9: utorso -> endpoint
% 
% Sources:
% [1] atlas_v3.urdf
% [2] [KhalilKle1986]

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-03
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function T_c_mdh = atlas5_arm_fkine_mdh_num(q, lr)

%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'Joint angles q have to be [1x7] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');      

%% Calculation

% Get urdf parameters
p_num = atlas5_arm_parameter_urdf(lr);

% Initialize Variables
[a, d, alpha, q_offset] = atlas5_arm_parameter_mdh(lr);

% assign cumulated transformation matrices
T_c_mdh = NaN(4,4,9);

% Determine base transformation
% Rotate utorso-frame so that z-Axis is q1-Axis
R_base = NaN(3,3);
R_base(:,3) = [0, p_num(4:5)]';
R_base(:,1) = [1; 0; 0];
R_base(:,2) = cross(R_base(:,3), R_base(:,1));
R_base = R_base * rotz(-pi/2);%* rotz(pi);
T_base = transl(p_num(1:3)) * r2t(R_base);

T_c_mdh(:,:,1) = T_base;

% Alle Gelenkwinkel durchgehen
for i = 1:7
    T_c_mdh(:,:,i+1) = T_c_mdh(:,:,i) * trotx(alpha(i)) * ...
        transl([a(i);0;0]) * trotz(q(i)+q_offset(i)) * transl([0;0;d(i)]);
end

% hand transformation as in urdf
T_c_mdh(:,:,9) = T_c_mdh(:,:,8) * troty(pi/2) * trotz(pi/2);


return;