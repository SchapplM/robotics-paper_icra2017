% Zeichne Kollisionskörper des Atlas-Roboters (v5)
% 
% Eingabe:
% LI
%   Segment-Indizes
% T_c_mdh
%   Treansformationsmatrizen zu allen Körper-Koordinatensystemen
% atlas5_collbodies
%   Struktur mit allen Kollisionskörpern des Roboters. Enthält Felder:
%   .T: Typ des Ersatzkörpers. 1=Quader, 2=Zylinder
%   .P: Parameter des Ersatzkörpers (Punkte, Vektoren)
%   Siehe atlas_contactbody_parse.m
% FaceAlpha, FaceColor, EdgeColor, EdgeAlpha
%   Optionale Argumente für Aussehen der Körper

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

function atlas_plot_wbody_link_collision_body(LI, T_c_mdh, atlas5_collbodies, ...
  FaceAlpha, FaceColor, EdgeColor, EdgeAlpha)

if nargin < 4
  FaceAlpha = 0.3;
end
if nargin < 5
  FaceColor =  'b';
end
if nargin < 6
  EdgeColor = 'k';
end
if nargin < 7
  EdgeAlpha = 1;
end

% Alle übergebenen Körper durchgehen
for li = LI(:)'
  T_mdh_i = T_c_mdh(:,:,li);
  I_i = find(atlas5_collbodies.I == li);
  % Alle Ersatzkörper durchgehen
  for j = I_i' % Index in Ersatzkörper-Liste
    Typ_j = atlas5_collbodies.T(j);
    Par_j = atlas5_collbodies.P(j,:);
    if Typ_j == 1 % Quader
      % Eckpunkte des Quaders
      r_0_Q1 = eye(3,4) * T_mdh_i*[Par_j(1:3)';1];
      r_0_Q1Q2 = T_mdh_i(1:3,1:3)*Par_j(4:6)';
      r_0_Q1Q3 = T_mdh_i(1:3,1:3)*Par_j(7:9)';
      r_0_Q1Q4 = T_mdh_i(1:3,1:3)*Par_j(10:12)';
      cubpar_c = r_0_Q1+(r_0_Q1Q2+r_0_Q1Q3+r_0_Q1Q4)/2; % Mittelpunkt des Quaders
      cubpar_l = [norm(r_0_Q1Q2); norm(r_0_Q1Q3); norm(r_0_Q1Q4)]; % Dimension des Quaders
      cubpar_a = rotation3dToEulerAngles(t2r(T_mdh_i))'; % Orientierung des Quaders
      drawCuboid([cubpar_c', cubpar_l', cubpar_a'], 'FaceColor', FaceColor, ...
        'FaceAlpha', FaceAlpha, 'EdgeColor', EdgeColor, 'EdgeAlpha', EdgeAlpha);
    elseif Typ_j == 2 % Zylinder
      r_0_P1 = eye(3,4) * T_mdh_i*[Par_j(1:3)';1];
      r_0_P2 = eye(3,4) * T_mdh_i*[Par_j(4:6)';1];
      r_zyl = Par_j(7)';
      drawCylinder([r_0_P1', r_0_P2', r_zyl], 'EdgeColor', EdgeColor, ...
        'FaceAlpha', FaceAlpha, 'FaceColor', FaceColor, 'EdgeAlpha', EdgeAlpha);
    end
  end
end