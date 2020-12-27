% Jacobimatrix zu beliebigen Punkten eines Körpers für atlas5_wbody
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% qJ [1x30]
%   Joint Angles [rad]
% r_i_i_C [1x3]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt
%   Nummerierung von 1 (pelvis) bis 31 (r_hand). Wie in atlas5_wbody_fkine_mdh_num
% 
% Output:
% Jg_C [6x36]
%   geometric body jacobian for the defined point with respect to
%   generalized coordinates:
% 
% Quellen:
% [1] Eigene Berechnungen (vom 10.06.2016)
% [2] BouyarmaneKhe2012: On the dynamics modeling of free-floating-base
%     articulated mechanisms and applications to humanoid whole-body dynamics
%     and control

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function Jg_C = atlas5_wbody_body_jacobig_mdh_eulangrpy_num(phi_base, qJ, link_index, r_i_i_C)
%% Init
%#codegen
assert(isa(phi_base,'double') && isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'atlas5_wbody_body_jacobig_mdh_eulangrpy_num: Base RPY angles have to be [3x1] double');
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [30 1]), ...
  'atlas5_wbody_body_jacobig_mdh_eulangrpy_num: Joint angles qJ have to be [30x1] double');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
  'atlas5_wbody_body_jacobig_mdh_eulangrpy_num: link_index has to be [1x1] uint8'); 
assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ...
  'atlas5_wbody_body_jacobig_mdh_eulangrpy_num: Position vector r_i_i_C has to be [3x1] double');

%% Kinematik berechnen
T_c_mdh = atlas5_wbody_fkine_mdh_num(qJ');
T_B_i = T_c_mdh(:,:,link_index);
% Vektor von der Basis (B) zum Punkt C (auf Körper i)
r_B_B_C = T_B_i(1:3,4) + T_B_i(1:3,1:3) * r_i_i_C; % r_B_B_i + R_B_i_C

% Rotationsmatrix vom Welt- ins Basis-KS
R_W_B = eulxyz2r([phi_base(1); phi_base(2); phi_base(3)]);

% Transformation zwischen RPY-Winkel-Zeitableitung und
% Winkelgeschwindigkeit der Basis im Welt-KS
T_angvel = eulxyzjac([phi_base(1); phi_base(2); phi_base(3)]);
%% Basis-Jacobi-Matrix
% Siehe [1]
% [2], equ. (14)
Jg_BTC = [eye(3),     -skew(R_W_B*r_B_B_C)*T_angvel];
% [2], equ. (17)
Jg_BRC = [zeros(3,3), T_angvel];

%% Gelenk-Jacobi-Matrix
Jg_JC_B = atlas5_wbody_body_jacobig_mdh_num(qJ', link_index, r_i_i_C');
Jg_JTC = R_W_B * Jg_JC_B(1:3,:);
Jg_JRC = R_W_B * Jg_JC_B(4:6,:);

%% Gesamt-Jacobi-Matrix zusammensetzen
Jg_C = [Jg_BTC, Jg_JTC; ... % [2], equ. (14)
        Jg_BRC, Jg_JRC]; % [2], equ. (17)