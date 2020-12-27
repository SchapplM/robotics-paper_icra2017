% Isolierung des Kollisionsortes (Segment und Koordinaten)
% 
% Methode:
% Hier nur Erkennung einer einzigen Kollision an einem Segment, das direkt
% hinter dem letzten Gelenk liegt, an dem das Störmoment gemessen wurde.
% 
% Eingabe:
% rpy_base [3x1]
%   Basis-Orientierung im Welt-KS in RPY-Euler-Winkeln
% qJ [30x1]
%   Gelenkwinkel
% tau_obs [30x1]
%   Geschätzte externe Kräfte in verallgemeinerten Koordinaten
% tauJ_obs_thresh [30x1]
%   Schwellwert für Kollisionserkennung der Gelenkmomente
% atlas5_collbodies
%   Struktur mit allen Kollisionskörpern des Roboters. Enthält Felder:
%   .T: Typ des Ersatzkörpers. 1=Quader, 2=Zylinder
%   .P: Parameter des Ersatzkörpers (Punkte, Vektoren)
%   Siehe atlas_contactbody_parse.m
% IQ [36x1 logical]
%   Indizes der verallgemeinerten Koordinaten, die in tau_obs verwendet
%   werden sollen.
%   Für Floating Base Ganzkörper: IQ=true(36,1);
%   Für reine Betrachtung des linken Arms: IQ=[false(22;1); true(7;1);false(7;1)];
% ic_fix [1x1]
%   vorgegebenes Kollisionssegment. Falls ungleich Null gesetzt, wird nur noch der
%   Kontaktort geschätzt, nicht mehr das Kontaktsegment.
% 
% Ausgabe:
% ic [1x1]
%   Nummer des Segmentes, an dem die Kollision vermutet wird
% r_i_i_C [3x1]
%   Ortsvektor zum Kollisionspunkt im Körper-KS des Kollisionssegmentes
% r_i_i_d [3x1]
%   Aufpunkt zur Geraden für die Kraftwirklinie (im Körper-KS)
% u_i [3x1]
%   Richtungsvektor zur Geraden für die Kraftwirklinie (im Körper-KS)

% Quelle:
% [0_HaddadinDeAlb2016] Haddadin, De Luca, Albu-Schäffer: Robot Collisions:
% Detection, Isolation, and Identification, Submitted to IEEE Transactions
% on Robotics, 2016 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function [ic, r_i_i_C, r_i_i_d, u_i] = atlas5_wbody_collision_isolation(rpy_base, qJ, tau_obs, ...
  tauJ_obs_thresh, atlas5_collbodies, IQ, ic_fix)

%% Init
assert(isa(rpy_base,'double') && isreal(rpy_base) && all(size(rpy_base) == [3 1]), ...
  'atlas5_wbody_collision_isolation: phi_base has to be [3x1] double');
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [30 1]), ...
  'atlas5_wbody_collision_isolation: Joint angles q have to be [30x1] double');
assert(isa(tau_obs,'double') && isreal(tau_obs) && all(size(tau_obs) == [36 1]), ...
  'atlas5_wbody_collision_isolation: Observer output tau_obs has to be [36x1] double');
assert(isa(tauJ_obs_thresh,'double') && isreal(tauJ_obs_thresh) && all(size(tauJ_obs_thresh) == [30 1]), ...
  'atlas5_wbody_collision_isolation: Threshold tauJ_obs_thresh has to be [30x1] double');

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
    'atlas5_wbody_collision_isolation: atlas5_collbodies does not have the correct structure properties');
  
assert(isa(IQ,'logical')  && all(size(IQ) == [36 1]), ...
  'atlas5_wbody_collision_identification: IQ has to be [36x1] logical');
  
assert(isa(ic_fix,'uint8')  && all(size(ic_fix) == [1 1]), ...
  'atlas5_wbody_collision_identification: ic_fix has to be [1x1] uint8');

r_i_i_C = NaN(3,1);

%% Vorverarbeitung
tauJ_obs = tau_obs(7:end);
T_c_mdh = atlas5_wbody_fkine_mdh_num(qJ');

%% Kontakt-Segment erkennen
% Zur Definition der Körper, siehe atlas5_wbody_fkine_mdh_num.m
if ic_fix > 0
  % Segment-Nummer ist vorgegeben.
  ic = ic_fix;
else
  % Ib_coll = abs(FB_obs_W) > FB_obs_thresh;
  Ij_coll = abs(tauJ_obs) > tauJ_obs_thresh;
  if ~any(Ij_coll)
    ic = uint8(1); % Nur Kollision an Basis
  else
    % [0_HaddadinDeAlb2016] equ. (55)
    % Der Kollisionskörper ist hinter dem letzten Gelenk der Kette, bei dem
    % die Kollision festgestellt wurde.
    ic = uint8(find(Ij_coll, 1, 'last' )+1);
  end
  ic = uint8(ic(1)); % Typumwandlung für andere Funktionen mit assert-Befehlen. Muss auch explizit skalar gesetzt sein für spätere Funktionen
end
%% Effektive Kraft auf Segment-KS
% Transformationsmatrizen
T_0_i = NaN(4,4);
T_0_i(1:4,1:4) = T_c_mdh(:,:,ic);
T_W_0 = r2t(eulxyz2r([rpy_base(1); rpy_base(2); rpy_base(3)])); % Translation W-0 ist egal
T_W_i = T_W_0*T_0_i;
% Jacobi-Matrix zum Segment
Jg_ic_wbodyfloatb = atlas5_wbody_body_jacobig_mdh_eulangrpy_num(rpy_base, qJ, ic, zeros(3,1));
Jg_ic = Jg_ic_wbodyfloatb(:,IQ);
% [0_HaddadinDeAlb2016] equ. (58)
JT_PI = pinv(Jg_ic'); % besser als (Jg_ic*Jg_ic')\(Jg_ic), da auch bei Rangverlust berechenbar
F_i_W = JT_PI * tau_obs(IQ); % Externe Kraft auf Koordinatenursprung i ausgedrückt in KS W
F_i_i = rotate_wrench( F_i_W, T_W_i(1:3,1:3)' ); % Externe Kraft auf Koordinatenursprung i ausgedrückt in KS i

%% Kraftwirklinie bestimmen
% [0_HaddadinDeAlb2016] equ. (60)
% Bestimme Aufpunkt der Geraden
r_i_i_d =solve_pseudo_inverse_Skew_transpose(F_i_i(1:3),F_i_i(4:6)); % spezialisierte Funktion
% r_i_i_d = pinv(skew(F_i_i(1:3))') * F_i_i(4:6); % allgemein Funktion (langsamer)
u_i = F_i_i(1:3)/norm(F_i_i(1:3)); % Richtungsvektor der Geraden (Verschiebung der Kraft entlang der Wirklinie)

if any(isnan(u_i))
  % Bei Weitermachen mit NaN-Werten kommt es zu kryptischen Fehlern bei der
  % Kontaktberechnung
  return
end
%% Kontaktort mit Geometrie bestimmen

% Alle Ersatzkörper für dieses Segment auswählen
I_cb = (atlas5_collbodies.I == ic);

if sum(I_cb) == 0
  % Für dieses Segment sind keine Ersatzkörper definiert.
  r_i_i_C = zeros(3,1); % Nehme einen Kontakt im Ursprung an.
  return
end

% Transformationsmatrix in 2D-Format umwandeln
T_c_mdh_stack = NaN(31,16);
for i = 1:31
  T_c_mdh_stack(i,:) = reshape(T_c_mdh(:,:,i),1,16);
end
S = atlas5_wbody_intersect_collbodies(r_i_i_d, u_i, ic, T_c_mdh_stack, I_cb, atlas5_collbodies);

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
r_i_i_C = S(ii_min,1:3)';

%%
return  %#ok<*UNRCH>
%% Debug: Zeichnen
% Die Zeichnung ist im Basis-KS

T_c_urdf = atlas5_wbody_fkine_urdf_num(qJ');
figure(800);clf;
hold on; grid on;axis equal;view(3);set(800, 'Renderer','OpenGL');
atlas_plot_wbody_link_stl(ic, uint8(5), T_c_urdf);
AS = atlas_const(uint8(5));
title(sprintf('Link %d: %s', ic, AS.LN{ic}), 'interpreter', 'none');
xlabel('x_0 [m]'); ylabel('y_0 [m]'); zlabel('z_0 [m]')

% Körper-KS einzeichnen
trplot(T_0_i, 'frame', AS.LN{ic}, 'arrow', 'rgb', 'length', 0.3)

% Gerade zeichnen
p = T_0_i*[r_i_i_d;1];
u = T_0_i(1:3,1:3)*u_i;
d = 0.2;
plot3([p(1)-d*u(1) p(1)+d*u(1)]', [p(2)-d*u(2) p(2)+d*u(2)]', [p(3)-d*u(3) p(3)+d*u(3)]',...
  'y' , 'LineWidth', 5);

% Bestimmten Kollisionspunkt einzeichnen
r_0_0_C = T_0_i*[r_i_i_C;1];
plot3(r_0_0_C(1), r_0_0_C(2), r_0_0_C(3),'mo', 'MarkerSize', 15);


% Punkte zu allen Kollisionskörpern einzeichnen
for j = find(I_cb)'
  Typ_j = atlas5_collbodies.T(j);
  Par_j = atlas5_collbodies.P(j,:);

  if Typ_j == 1 % Quader
    % Eckpunkte des Quaders
    r_0_Q1 = eye(3,4) * T_0_i*[Par_j(1:3)';1];
    r_0_Q1Q2 = T_0_i(1:3,1:3)*Par_j(4:6)';
    r_0_Q1Q3 = T_0_i(1:3,1:3)*Par_j(7:9)';
    r_0_Q1Q4 = T_0_i(1:3,1:3)*Par_j(10:12)';
    plot_cube2(r_0_Q1, r_0_Q1Q2, r_0_Q1Q3, r_0_Q1Q4, 'b')
  elseif Typ_j == 2 % Zylinder
    r_0_P1 = eye(3,4) * T_0_i*[Par_j(1:3)';1];
    r_0_P2 = eye(3,4) * T_0_i*[Par_j(4:6)';1];
    r_zyl = Par_j(7)';
    drawCylinder([r_0_P1', r_0_P2', r_zyl], 'EdgeColor', 'k', ...
      'FaceAlpha', 0.3, 'FaceColor', 'b')
  end
    
  % Punkte einzeichnen
  for ii = 1:size(S,1)
    r_i_i_Sii = S(ii,1:3)';
    r_0_0_Sii = T_0_i*[r_i_i_Sii;1];
    plot3(r_0_0_Sii(1), r_0_0_Sii(2), r_0_0_Sii(3),'rx', 'MarkerSize', 15);
  end
end

axis auto
