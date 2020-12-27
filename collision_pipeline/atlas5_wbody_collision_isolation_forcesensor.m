% Isolierung des Kollisionsortes (Segment und Koordinaten)
% basierend auf Kraftsensoren
% 
% Methode:
% Hier nur Erkennung einer einzigen Kollision an einer gegebenen
% kinematischen Kette, die hinter einem Kraft-Momentensensor mit gegebenem Messwert liegt
% 
% Eingabe:
% rpy_base [3x1]
%   Basis-Orientierung im Welt-KS in RPY-Euler-Winkeln
% qJ [30x1]
%   Gelenkwinkel
% F_S [6x1]
%   Gemessene Kraft und Drehmoment des Sensors
% J [1x1]
%   Segment-Nummer, vor dem der Sensor befestigt ist. Der Sensor wird
%   zunächst im Körper-KS dieses Segments angenommen.
% Ichain [Nx1]
%   Indizes der Nachfolger-Segmente des Sensors. Mit den
%   Kollisions-Ersatzkörpern dieser Segmente werden Kollisionen geprüft
% atlas5_collbodies
%   Struktur mit allen Kollisionskörpern des Roboters. Enthält Felder:
%   .T: Typ des Ersatzkörpers. 1=Quader, 2=Zylinder
%   .P: Parameter des Ersatzkörpers (Punkte, Vektoren)
%   Siehe atlas_contactbody_parse.m
% 
%
% Ausgabe:
% ic [1x1]
%   Nummer des Segmentes, an dem die Kollision vermutet wird
% r_i_i_C [3x1]
%   Ortsvektor zum Kollisionspunkt im Körper-KS des Kollisionssegmentes
% r_i_i_d [3x1]
%   Aufpunkt der Kraftwirklinie
% u_i [3x1]
%   Richtungsvektor der Kraftwirklinie
% 
% Quelle:
% [0_HaddadinDeAlb2016] Haddadin, De Luca, Albu-Schäffer: Robot Collisions:
% Detection, Isolation, and Identification, Submitted to IEEE Transactions
% on Robotics, 2016 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

function [ic, r_i_i_C, r_i_i_d, u_i] = atlas5_wbody_collision_isolation_forcesensor( ...
  rpy_base, qJ, F_S, J, Ichain, atlas5_collbodies)

%% Init
assert(isa(rpy_base,'double') && isreal(rpy_base) && all(size(rpy_base) == [3 1]), ...
  'atlas5_wbody_collision_isolation_forcesensor: phi_base has to be [3x1] double');
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [30 1]), ...
  'atlas5_wbody_collision_isolation_forcesensor: Joint angles q have to be [30x1] double');
assert(isa(F_S,'double') && isreal(F_S) && all(size(F_S) == [6 1]), ...
  'atlas5_wbody_collision_isolation_forcesensor: F_S has to be [6x1] double');
assert(isa(J,'uint8') &&  all(size(J) == [1 1]), ...
  'atlas5_wbody_collision_isolation_forcesensor: J has to be [1x1] uint8');
assert(isa(Ichain,'uint8') &&  all(size(Ichain) <= [31 1]), ...
  'atlas5_wbody_collision_isolation_forcesensor: Ichain has to be [Nx1] uint8');

assert(isa(atlas5_collbodies,'struct') ...
    && isa(atlas5_collbodies.I, 'uint8') ...
    && size(atlas5_collbodies.I, 1) <= 200 ...
    && size(atlas5_collbodies.I, 2) == 1 ...
    && isa(atlas5_collbodies.T, 'uint8') ...
    && size(atlas5_collbodies.T, 1) <= 200 ...
    && size(atlas5_collbodies.T, 2) == 1 ...
    && isa(atlas5_collbodies.P, 'double') && isreal(atlas5_collbodies.P) ...
    && size(atlas5_collbodies.P, 1) <= 200 ...
    && size(atlas5_collbodies.P, 2) == 12);
  
r_i_i_C = NaN(3,1);
r_i_i_d = NaN(3,1);
u_i = NaN(3,1);
ic = uint8(0);  
if all(F_S(1:3) == 0)
  return
end
  
%% Vorverarbeitung
T_c_mdh = atlas5_wbody_fkine_mdh_num(qJ');

%% Effektive Kraft auf Kraftsensor-Segment-KS
% Transformationsmatrizen
T_0_J = NaN(4,4);
T_0_J(1:4,1:4) = T_c_mdh(:,:,J);
T_W_0 = r2t(eulxyz2r([rpy_base(1); rpy_base(2); rpy_base(3)])); % Translation W-0 ist egal
T_W_J = T_W_0*T_0_J;
% Jacobi-Matrix zum Segment, an dem die Kollision durch
% Schwellwertüberschreitung festgestellt wurde
F_J_W = F_S; % Externe Kraft auf Koordinatenursprung i ausgedrückt in KS W
F_J_J = rotate_wrench( F_J_W, T_W_J(1:3,1:3)' ); % Externe Kraft auf Koordinatenursprung i ausgedrückt in KS i

%% Kraftwirklinie bestimmen
% [0_HaddadinDeAlb2016] equ. (60)
% Aufpunkt der Geraden
r_J_J_d = solve_pseudo_inverse_Skew_transpose(F_J_J(1:3),F_J_J(4:6)); % auf Matrixstruktur optimierte Berechnung
% r_J_J_d = pinv(skew(F_J_J(1:3))') * F_J_J(4:6); % Berechnung mit allgemeiner Formel, gleiches Ergebnis
u_J = F_J_J(1:3)/norm(F_J_J(1:3)); % Richtungsvektor der Geraden (Verschiebung der Kraft entlang der Wirklinie)


%% Kontaktort mit Geometrie bestimmen

% Alle Ersatzkörper für dieses Segment und alle folgenden Segmente auswählen
% Das ist notwendig, da manchmal die Schwellwertüberschreitung bei einem
% vorherigen Segment erkannt wurde 
I_cb = false(length(atlas5_collbodies.I), 1);
for jj_tmp = 1:31 % Obergrenze der Schleife für Kompilierbarkeit
  if jj_tmp>length(Ichain)
    break;
  end
  i = Ichain(jj_tmp);
  I_cb = I_cb | (atlas5_collbodies.I == i);
end

if sum(I_cb) == 0
  % Für diese Kette sind keine Ersatzkörper definiert.
  r_i_i_C = zeros(3,1); % Nehme einen Kontakt im Ursprung an.
  return
end

% Transformationsmatrix in 2D-Format umwandeln
T_c_mdh_stack = NaN(31,16);
for i = 1:31
  T_c_mdh_stack(i,:) = reshape(T_c_mdh(:,:,i),1,16);
end
S = atlas5_wbody_intersect_collbodies(r_J_J_d, u_J, J, T_c_mdh_stack, I_cb, atlas5_collbodies);

%% Wahrscheinlichsten Kontaktpunkt auswählen, falls mehrere Ersatzkörper geschnitten werden
% Nehme den äußersten Schnittpunkt, der einer Druckkraft entspricht
% Untersuche alle erkannten Schnittpunkte auf der Geraden. Es kommen keine
% inneren Ersatzkörper in Frage

% Indizes der Ersatzkörper, die von der Geraden geschnitten werden.
I_Schnitt = ~isnan(S(:,4));
% Indizes, die nicht geschnitten werden und nur nahe dran liegen
I_keinSchnitt = ~I_Schnitt;

% Durchsuche Ersatzkörper, die die Gerade schneiden
lambda_min = Inf;
ii_min = -1;
I_ks_tmp = find(I_Schnitt)';
for jj_tmp = 1:200 % Schleife mit begrenztem Ende für Kompilierbarkeit
  if jj_tmp > length(I_ks_tmp)
    break;
  end
  ii = I_ks_tmp(jj_tmp);
  % suche den kleinsten Parameter lambda der Geraden. Dieser entspricht der
  % äußersten Druckkraft
  if S(ii,4) < lambda_min
    lambda_min = S(ii,4);
    ii_min = ii;
  end
end

% Durchsuche Ersatzkörper, die die Gerade nicht schneiden.
% Nehme den Punkt, der am nächsten an der Gerade liegt (nicht nur am
% Aufpunkt). Der jeweils kürzeste Abstand zur Geraden ist in S(ii,6) enthalten
if ii_min == -1
  d_min = Inf;
  I_ks_tmp = find(I_keinSchnitt)';
  for jj_tmp = 1:200 % Schleife mit begrenztem Ende für Kompilierbarkeit
    if jj_tmp > length(I_ks_tmp)
      break;
    end
    ii = I_ks_tmp(jj_tmp);
    if S(ii,6) < d_min % Prüfe, ob dieser Abstand kleiner ist als der bisher kleinste
      d_min = S(ii,6);
      ii_min = ii;
    end
  end
end
r_J_J_C = S(ii_min,1:3)';
ic = atlas5_collbodies.I( S(ii_min,5) );

% Rechner den im Kraftsensor-KS angegebenen Kollisionsort in das Körper-KS
% des Kollisionssegmentes um (und alle anderen Größen auch)
if ic ~= J
  T_0_i = T_c_mdh(:,:,ic);
  rh_0_0_C = T_0_J * [r_J_J_C;1];
  rh_0_0_D = T_0_J * [r_J_J_d;1];
  r_i_i_C = T_0_i(1:3,1:3)' * (- T_0_i(1:3,4) + rh_0_0_C(1:3));
  r_i_i_d = T_0_i(1:3,1:3)' * (- T_0_i(1:3,4) + rh_0_0_D(1:3));
  u_i = T_0_i(1:3,1:3)' * T_0_J(1:3,1:3)*u_J;
else
  r_i_i_C = r_J_J_C;
  r_i_i_d = r_J_J_d;
  u_i = u_J;
end



% Im Gegensatz zu atlas5_wbody_collision_isolation_fullchain muss keine
% weitere Iteration berechnet werden, da keine neue Jacobi-Matrix gebildet
% werden muss, wenn ein neues Kollisionssegment geschätzt wurde.

return  %#ok<*UNRCH>
%% Debug: Zeichnen
% Siehe atlas5_wbody_collision_isolation_fullchain


T_c_urdf = atlas5_wbody_fkine_urdf_num(qJ');
figure(801);clf;
hold on; grid on;axis equal;view(3);set(801, 'Renderer','OpenGL');
for j = Ichain(:)'
  atlas_plot_wbody_link_stl(j, uint8(5), T_c_urdf);
end
AS = atlas_const(uint8(5));
title(sprintf('Sensor: Link %d (%s)', j, AS.LN{j}), 'interpreter', 'none');
xlabel('x_0 [m]'); ylabel('y_0 [m]'); zlabel('z_0 [m]')

% Körper-KS einzeichnen
trplot(T_0_J, 'frame', AS.LN{j}, 'arrow', 'rgb', 'length', 0.3)

% Gerade zeichnen
p = T_0_J*[r_J_J_d;1];
u = T_0_J(1:3,1:3)*u_J;
d = 0.2;
plot3([p(1)-d*u(1) p(1)+d*u(1)]', [p(2)-d*u(2) p(2)+d*u(2)]', [p(3)-d*u(3) p(3)+d*u(3)]',...
  'y' , 'LineWidth', 5);

% Aufpunkt der Geraden zeichnen
plot3(p(1), p(2), p(3), 'k+', 'MarkerSize', 15, 'LineWidth', 5);

% Bestimmten Kollisionspunkt einzeichnen
r_0_0_C = T_0_J*[r_J_J_C;1];
plot3(r_0_0_C(1), r_0_0_C(2), r_0_0_C(3),'mo', 'MarkerSize', 15);


% Punkte zu allen Kollisionskörpern einzeichnen
for j = find(I_cb)'
  Typ_J = atlas5_collbodies.T(j);
  Par_J = atlas5_collbodies.P(j,:);
  Tmdh_J = T_c_mdh(:,:,atlas5_collbodies.I(j));

  if Typ_J == 1 % Quader
    % Eckpunkte des Quaders
    r_0_Q1 = eye(3,4) * Tmdh_J*[Par_J(1:3)';1];
    r_0_Q1Q2 = Tmdh_J(1:3,1:3)*Par_J(4:6)';
    r_0_Q1Q3 = Tmdh_J(1:3,1:3)*Par_J(7:9)';
    r_0_Q1Q4 = Tmdh_J(1:3,1:3)*Par_J(10:12)';
    plot_cube2(r_0_Q1, r_0_Q1Q2, r_0_Q1Q3, r_0_Q1Q4, 'b')
  elseif Typ_J == 2 % Zylinder
    r_0_P1 = eye(3,4) * Tmdh_J*[Par_J(1:3)';1];
    r_0_P2 = eye(3,4) * Tmdh_J*[Par_J(4:6)';1];
    r_zyl = Par_J(7)';
    drawCylinder([r_0_P1', r_0_P2', r_zyl], 'EdgeColor', 'k', ...
      'FaceAlpha', 0.3, 'FaceColor', 'b')
  end
    
  % Punkte einzeichnen
  for ii = 1:size(S,1)
    r_J_J_Sii = S(ii,1:3)';
    r_0_0_Sii = T_0_J*[r_J_J_Sii;1];
    plot3(r_0_0_Sii(1), r_0_0_Sii(2), r_0_0_Sii(3),'rx', 'MarkerSize', 15);
  end
end

axis auto