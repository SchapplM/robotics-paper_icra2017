% Gelenk-Jacobimatrix zu beliebigen Punkten eines Körpers für atlas5_wbody
% 
% Input:
% q [1x30]
%   Joint Angles [rad]
% r_i_i_C [1x3]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt
%   Nummerierung von 1 (pelvis) bis 31 (r_hand). Wie in atlas5_wbody_fkine_mdh_num
% 
% Output:
% Jg_C [6x30]
%   geometric body jacobian for the defined point
% 
% Quellen:
% [1] Ortmaier: Robotik I Skript

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function Jg_C = atlas5_wbody_body_jacobig_mdh_num(q, link_index, r_i_i_C)
%% Init
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 30]), ...
  'atlas5_wbody_body_jacobig_mdh_num: Joint angles q have to be [1x30] double');
assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [1 3]), ...
  'atlas5_wbody_body_jacobig_mdh_num: Position vector r_i_i_C has to be [1x3] double');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
  'atlas5_wbody_body_jacobig_mdh_num: link_index has to be [1x1] uint8'); 

% Initialisierung. Alle Spalten die nicht gesetzt werden haben keinen
% Einfluss.
Jg_C = zeros(6,30);

%% Daten
[a_mdh, d_mdh, alpha_mdh, q_offset_mdh, b_mdh, beta_mdh, v, ...
  rSges_mdh_wbody, m_wbody, Iges_mdh_wbody] = atlas5_wbody_parameter_mdh(); %#ok<ASGLU>

if link_index == 1
  % Die Gelenkwinkel haben keinen Einfluss auf die Basis
  return;
end

%% Kinematik berechnen
T_c_mdh = atlas5_wbody_fkine_mdh_num(q);
T_0_i = T_c_mdh(:,:,link_index);
R_0_i = t2r(T_0_i);
r_0_i_C = (R_0_i) * (r_i_i_C');

j = link_index;
for tmp = 1:30
  % Vorgänger-Index
  k = v(j-1); % Diese Indizes haben die Basis als 0.
  
  %  Drehachse des Gelenks, das diesen Körper bewegt ist die z-Achse dieses
  %  Körpers (bei DH-Notation ist es der vorherige, hier MDH-Notation).
  ax = T_c_mdh(1:3,3,j);
  jr = ax;
  
  % Vektor vom Gelenk zum Punkt
  r_0_j_i = -T_c_mdh(1:3,4,j) + T_0_i(1:3,4);
  r_0_j_C = r_0_j_i + r_0_i_C;
  
  % Hebelarm vom Gelenk zum Punkt
  jt = cross(ax, r_0_j_C);
  
  % Spalte der Jacobi-Matrix eintragen
  Jg_C(:,j-1) = [jt; jr];
  
  % Indizes tauschen: Kinematische Kette weiter Richtung Basis entlanggehen
  j = k+1; % Index mit Basis als 1
  if j == 1
    % An Basis angekommen
    return;
  end
end
