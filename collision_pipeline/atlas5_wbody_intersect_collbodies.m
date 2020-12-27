% Berechne Schnittpunkt einer Geraden mit den Kollisionskörpern des
% Atlas-Roboters
% 
% Eingabe:
% r_i_i_d [3x1] double
%   Aufpunkt zur Geraden, die mit den Kollisionskörpern geschnitten werden
%   soll (in Körperkoordianten)
% u_i [3x1] double
%   Richtungsvektor der Geraden (im Körper-KS)
% ic [1x1] uint8
%   Nummer des Körpers, in dem die Gerade (r_i_i_d,u_i) ausgedrückt ist
% T_c_mdh_stack [31x16] double
%   Transformationsmatrizen zu allen Körpern. Mit reshape(1,16) zu
%   Zeilenvektoren umgeformtes [4x4x31]-Array
% I_cb [Nx1] logical
%   Binär-Indizes für atlas5_collbodies.I. Dient zur Auswahl der
%   Kollisionskörper
% atlas5_collbodies [struct]
% 
% Ausgabe:
% S [nx6] double
%   Enthält Zeilenweise die Schnittpunkte der Geraden mit allen
%   ausgewählten Kollisionskörpern
%   Spalten: 
%   1-3: Schnittpunkt oder Punkt des kürzesten Abstandes, Koordinaten
%   4: Laufvariable der Geradengleichung zum Schnittpunkt
%   5: Index in Kollisionskörper-Struktur
%   6: Kürzester Abstand zum Körper (nur gesetzt, wenn kein Schnittpunkt).
%      Sonst NaN

% TODO: Nur Übergabe der Kollisionskörper, die tatsächlich geprüft werden.
% TODO: Übergabe der Ergebnisse in mehreren Variablen unterschiedlichen
%       Typs

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

function S = atlas5_wbody_intersect_collbodies(r_i_i_d, u_i, ic, T_c_mdh_stack, I_cb, atlas5_collbodies)
%% Init

assert(isa(r_i_i_d,'double') && isreal(r_i_i_d) && all(size(r_i_i_d) == [3 1]) && all(~isnan(r_i_i_d)), ...
  'atlas5_wbody_intersect_collbodies: r_i_i_d has to be [3x1] double');
assert(isa(u_i,'double') && isreal(u_i) && all(size(u_i) == [3 1]) && all(~isnan(u_i)), ...
  'atlas5_wbody_intersect_collbodies: u_i has to be [3x1] double');
assert(isa(ic,'uint8') && all(size(ic) == [1 1]), ...
  'atlas5_wbody_intersect_collbodies: ic has to be [1x1] uint8');
assert(isa(T_c_mdh_stack,'double') && isreal(T_c_mdh_stack) && all(size(T_c_mdh_stack) == [31,16]), ...
  'atlas5_wbody_intersect_collbodies: T_c_mdh has to be [31x16] double');
assert(isa(I_cb, 'logical') ...
    && size(I_cb, 1) <= 200 ...
    && size(I_cb, 2) == 1, ...
    'atlas5_wbody_intersect_collbodies: I_cb has to be 200x1 logical');
assert(isa(atlas5_collbodies,'struct') ...
    && isa(atlas5_collbodies.I, 'uint8') ...
    && size(atlas5_collbodies.I, 1) <= 200 ...
    && size(atlas5_collbodies.I, 2) == 1 ...
    && isa(atlas5_collbodies.T, 'uint8') ...
    && size(atlas5_collbodies.T, 1) <= 200 ...
    && size(atlas5_collbodies.T, 2) == 1 ...
    && isa(atlas5_collbodies.P, 'double') && isreal(atlas5_collbodies.P) ...
    && size(atlas5_collbodies.P, 1) <= 200 ...
    && size(atlas5_collbodies.P, 2) == 12, ...
    'atlas5_wbody_intersect_collbodies: atlas5_collbodies does not have the correct structure properties');

T_c_mdh = NaN(4,4,31);
for i = 1:31
  T_c_mdh(:,:,i) = reshape(T_c_mdh_stack(i, :),4,4);
end
T_0_i = T_c_mdh(:,:,ic);
%% Berechnung  
% Schnittpunkt mit allen Körpern finden
S = NaN(sum(I_cb), 6);
i = 0;
I_cb_candidates = find(I_cb)';
for jj_tmp = 1:200 % Schleife mit begrenztem Ende für Kompilierbarkeit
  if jj_tmp > length(I_cb_candidates)
    break;
  end
  j = I_cb_candidates(jj_tmp);
  i = i+1;
  Typ_j = atlas5_collbodies.T(j);
  Par_j = atlas5_collbodies.P(j,:);
  if atlas5_collbodies.I(j) == ic
    T_ij = eye(4); % Keine Transformation notwendig.
  else
    % Transformationsmatrix vom Basis-KS zum aktuellen Körper-KS j
    Tmdh_j = T_c_mdh(:,:,atlas5_collbodies.I(j));
    % Drücke alle Punkte und Vektoren im KS i des aktuell vermuteten
    % Kollisionssegmentes aus. Nicht in ihrem jeweiligen Körper-KS j
    T_ij = T_0_i \ Tmdh_j;
  end
  if Typ_j == 1 % Quader
    S_tmp = find_intersection_line_box(r_i_i_d, ... % Aufpunkt der Geraden
      u_i, ... % Richtungsvektor der Geraden
      T_ij(1:3,4)+T_ij(1:3,1:3)*Par_j(1:3)', ... % Vektor zum ersten Eckpunkt (ausgedrückt in KS i)
      T_ij(1:3,1:3)*Par_j(4:6)', T_ij(1:3,1:3)*Par_j(7:9)', T_ij(1:3,1:3)*Par_j(10:12)'); % Vektoren vom ersten Eckpunkt zu allen anderen Eckpunkten
  elseif Typ_j == 2 % Zylinder
    S_tmp = find_intersection_line_cylinder(r_i_i_d, u_i, ... % Aufpunkt und Richtungsvektor der Geraden
      T_ij(1:3,4)+T_ij(1:3,1:3)*Par_j(1:3)', T_ij(1:3,4)+T_ij(1:3,1:3)*Par_j(4:6)', ... % Start- und Endpunkt (ausgedrückt in KS i)
      Par_j(7)); % Radius
  else
    S_tmp = NaN(1,6); % Für Kompilierbarkeit.
  end
  % Die Schnittpunkte in S_tmp sind in KS i angegeben (genau wie die Eingaben)
  if isnan(S_tmp(6)) % kein Schnittpunkt, nur kürzeste Distanz
    S(i,1:3) = S_tmp(:,1); % erster Schnittpunkt
    S(i,4) = NaN; % zweiter Schnittpunkt. NaN wenn nur nächster Punkt
    S(i,6) = S_tmp(1,2); % kürzester Abstand der Geraden zum Körper
  else % Zwei Schnittpunkte gefunden
    % Wähle den Punkt aus den beiden möglichen Schnittpunkten so, dass die
    % Kraft eine Druckkraft ist
    % Finde den Punkt mit Druckkraft aus der Parametrierung (lambda) der
    % Geradengleichung
    % Parametrierung für ersten Schnittpunkt:
    lambda_A = (S_tmp(1,1) - r_i_i_d(1)) ./ u_i(1);
    % Parametrierung für zweiten Schnittpunkt:
    lambda_B = (S_tmp(1,2) - r_i_i_d(1)) ./ u_i(1);
    if lambda_B > lambda_A %#ok<BDSCI> % Siehe Aufzeichnungen vom 14.06.2016 (Schappler)
      S(i,1:3) = S_tmp(:,1); % erster Schnittpunkt (A).
      S(i,4) = lambda_A;
    else
      S(i,1:3) = S_tmp(:,2); % zweiter Schnittpunkt (B).
      S(i,4) = lambda_B;
    end
  end
  S(i,5) = j;
end