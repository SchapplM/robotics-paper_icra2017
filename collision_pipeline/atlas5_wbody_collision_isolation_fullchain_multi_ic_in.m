% Isolierung des Kollisionsortes (Segment und Koordinaten)
% 
% Methode:
% Hier nur Erkennung einer einzigen Kollision pro kinematischer Kette
% (Arme, Beine, Torso), die hinter dem letzten Gelenk liegt, an dem das
% Störmoment in dieser Kette gemessen wurde. 
% 
% Eingabe:
% ic_in [5x1]
%   Im Vorfeld ermitteltes Kontaktsegment der Ketten, 0, falls keine
%   Kollision fuer die Kette gefunden wurde. Reihenfolge: Torso, linkes
%   Bein, rechtes Bein, linker Arm, rechter Arm
% rpy_base [3x1]
%   Basis-Orientierung im Welt-KS in RPY-Euler-Winkeln
% qJ [30x1]
%   Gelenkwinkel
% tau_obs [36x1]
%   Geschätzte externe Kräfte in verallgemeinerten Koordinaten
% atlas5_collbodies
%   Struktur mit allen Kollisionskörpern des Roboters. Enthält Felder:
%   .T: Typ des Ersatzkörpers. 1=Quader, 2=Zylinder
%   .P: Parameter des Ersatzkörpers (Punkte, Vektoren)
%   Siehe atlas_contactbody_parse.m
% 
%
% Ausgabe:
% ic [5x1]
%   Nummer der Segmente, an denen die Kollisionen vermutet werden. Die
%   Einträge sind den kinematischen Ketten in folgender Reihenfolge
%   zugeordnet: Torso (einschließlich Nacken), linkes Bein, rechtes Bein,
%   linker Arm rechter Arm. Wird an einer Kette keine Kollision erkannt,
%   ist der Eintrag 0.
% r_i_i_C [3x5]
%   Ortsvektoren zu den Kollisionspunkten in den Körper-KS der
%   Kollisionssegmente. Die Reihenfolge ist wie oben, falls keine
%   Kollision mit einer bestimmten Kette vorliegt, werden NaNs im
%   entsprechenden Eintrag zurück gegeben.
% F_i_W [6x5]
%   Kontakt Wrenches zu den Kollisionspunkten im Welt-KS. Die Reihenfolge
%   ist wie oben, falls keine Kollision mit einer bestimmten Kette
%   vorliegt, werden NaNs im entsprechenden Eintrag zurück gegeben.
% 
% Quelle:
% [0_HaddadinDeAlb2016] Haddadin, De Luca, Albu-Schäffer: Robot Collisions:
% Detection, Isolation, and Identification, Submitted to IEEE Transactions
% on Robotics, 2016 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function [ic, r_i_i_C, F_i_W] = atlas5_wbody_collision_isolation_fullchain_multi_ic_in( ...
  ic_in, rpy_base, qJ, tau_obs, atlas5_collbodies)

%% Init
assert(isa(ic_in,'uint8')  && all(size(ic_in) == [5 1]), ...
  'atlas5_wbody_collision_isolation_fullchain_multi_ic_in: ic has to be [5x1] uint8');
assert(isa(rpy_base,'double') && isreal(rpy_base) && all(size(rpy_base) == [3 1]), ...
  'atlas5_wbody_collision_isolation_fullchain_multi_ic_in: phi_base has to be [3x1] double');
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [30 1]), ...
  'atlas5_wbody_collision_isolation_fullchain_multi_ic_in: Joint angles q have to be [30x1] double');
assert(isa(tau_obs,'double') && isreal(tau_obs) && all(size(tau_obs) == [36 1]), ...
  'atlas5_wbody_collision_isolation_fullchain_multi_ic_in: Observer output tau_obs has to be [36x1] double');
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
    'atlas5_wbody_collision_isolation_fullchain_multi_ic_in: atlas5_collbodies does not have the correct structure properties');
%% Vorverarbeitung
T_c_mdh = atlas5_wbody_fkine_mdh_num(qJ');
[~, ~, ~, ~, ~, ~, v] = atlas5_wbody_parameter_mdh();

%% Kontakt-Segmente erkennen
% Zur Definition der Körper, siehe atlas5_wbody_fkine_mdh_num.m

ic_neu = ic_in;
ic     = ic_in;
r_i_i_C = NaN(3,5);
r_i_i_C(:,ic_in~=0) = 0;
F_i_W = NaN(6,5);

coll_chains = find(ic~=0); % Ketten, die sich in Kollision befinden
% Indizes der Ketten, die sich geändert haben
changed_chains = coll_chains;
% Indizes der Ketten, fuer die ein neues Kontaktlink gefunden wurde
coll_chain_nr = NaN(length(coll_chains),1); 
% Gestapelte Jacobi-Matrizen der einzelnen Kontaktpunkte
JT = NaN(6*length(coll_chains),36);
T_0_i = NaN(4,4,5);
T_W_0 = r2t(eulxyz2r([rpy_base(1); rpy_base(2); rpy_base(3)])); % Translation W-0 ist egal
T_W_i = T_0_i;

% Zum Plotten am Ende der Funktion
u_i_debug = NaN(3,5);
r_i_i_d_debug = NaN(3,5);

for iterations = 1:10
  
  %% Für alle Ketten mit Kollisionen Transformationen berechnen und Jacobis stacken
  % Bei späteren Iterationen Jacobi-Matrix nur für geänderte Segmente neu
  % berechnen (Rechenzeit sparen).
  % Wird durch Indexvariable changed_chains erreicht
  for i_chain = 1:length(changed_chains)
    %% Effektive Kraft auf Segment-KS
    % Transformationsmatrizen
    % Index in den Gleichungssystem-Variablen (JT, F_i_Ws)
    coll_chain_nr(i_chain) = find(coll_chains==changed_chains(i_chain));
    T_0_i(:,:,changed_chains(i_chain)) = T_c_mdh(:,:,ic(changed_chains(i_chain)));
    T_W_i(:,:,changed_chains(i_chain)) = T_W_0*T_0_i(:,:,changed_chains(i_chain));
    % Jacobi-Matrix zum Segment, an dem die Kollision durch
    % Schwellwertüberschreitung festgestellt wurde
    Jg_ic = atlas5_wbody_body_jacobig_mdh_eulangrpy_num(rpy_base, qJ, ...
      uint8(ic(changed_chains(i_chain))), zeros(3,1));
    % Jacobimatrizen werden gestackt zur gemeinsamen berechnung aller
    % Kontakt-Wrenches
    JT(6*(coll_chain_nr(i_chain)-1)+1:6*coll_chain_nr(i_chain),:) = Jg_ic;
  end

  % [0_HaddadinDeAlb2016] equ. (58)
  F_i_Ws = pinv(JT')*tau_obs; % Externe Kräfte auf Koordinatenursprünge i ausgedrückt in KS W (identisch mit pinv(JT)*tau_obs
  %% Für alle Ketten mit Kollision Kontaktpunkt bestimmen
  % Extrahiere Kräfte neu aus geändertem Gleichungssystem für alle Ketten,
  % nicht nur die geänderten. Die Lösung des GLS hat sich auch für alle
  % Ketten verändert. Benutze Indexvariable coll_chains
  for i_chain = 1:length(coll_chains) 
    F_i_W(:,coll_chains(i_chain)) = F_i_Ws(1+(i_chain-1)*6:i_chain*6);
    F_i_i = rotate_wrench( F_i_W(:,coll_chains(i_chain)), T_W_i(1:3,1:3,coll_chains(i_chain))' ); % Externe Kraft auf Koordinatenursprung i ausgedrückt in KS i

    %% Kraftwirklinie bestimmen
    % [0_HaddadinDeAlb2016] equ. (60)
    % Aufpunkt der Geraden
    r_i_i_d = solve_pseudo_inverse_Skew_transpose(F_i_i(1:3),F_i_i(4:6)); % auf Matrixstruktur optimierte Berechnung
    r_i_i_d_debug(:,coll_chains(i_chain)) = r_i_i_d;
    % r_i_i_d = pinv(skew(F_i_i(1:3))') * F_i_i(4:6); % Berechnung mit allgemeiner Formel, gleiches Ergebnis
    u_i = F_i_i(1:3)/norm(F_i_i(1:3)); % Richtungsvektor der Geraden (Verschiebung der Kraft entlang der Wirklinie)
    u_i_debug(:,coll_chains(i_chain)) = u_i;
    %% Liste mit allen Segmenten in den kinematischen Ketten nach den ersten Segmenten der jeweiligen Kette
    % Nachfolger-Elemente mit Funktionsaufruf: Die Indizes in der
    % Vorgängerliste v sind in MDH-Notation (0=Basis).
    % ic ist in Matlab-Notation (1=Basis). Das Ergebnis soll auch in
    % Matlab-Notation sein.
    % Ichain enthält alle Körper in der kinematischen Kette, für die
    % Kollisionen in Frage kommen.
    Ichain = [ic_in(coll_chains(i_chain));get_children_from_parent_list(v,ic_in(coll_chains(i_chain))-1)+1];
    % korrigiere Torso chain (keine Arme und Beine beruecksichtigen)
    if coll_chains(i_chain)==1 && ic_in(1)<=5
      Ichain(Ichain>5)=[];
    end
    
    %% Kontaktort mit Geometrie bestimmen

    % Alle Ersatzkörper für dieses Segment und alle folgenden Segmente auswählen
    % Das ist notwendig, da manchmal die Schwellwertüberschreitung bei einem
    % vorherigen Segment erkannt wurde 
    I_cb = false(length(atlas5_collbodies.I), 1);
    for jj_tmp = 1:200 % Schleife mit begrenztem Ende für Kompilierbarkeit
      if jj_tmp > length(Ichain)
        break;
      end
      i = Ichain(jj_tmp);
      I_cb = I_cb | (atlas5_collbodies.I == i);
    end

    if sum(I_cb) == 0
      % Für diese Kette sind keine Ersatzkörper definiert.
      r_i_i_C(:,coll_chains(i_chain)) = zeros(3,1); % Nehme einen Kontakt im Ursprung an.
      continue;
    end
    
    % Transformationsmatrix in 2D-Format umwandeln
    T_c_mdh_stack = NaN(31,16);
    for i = 1:31
      T_c_mdh_stack(i,:) = reshape(T_c_mdh(:,:,i),1,16);
    end
    % Die Nummer des aktuell betrachteten Körpers ist "ic(changed_chains(i_chain))"
    S = atlas5_wbody_intersect_collbodies(r_i_i_d, u_i, uint8(ic(coll_chains(i_chain))), T_c_mdh_stack, I_cb, atlas5_collbodies);

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
    for jj_tmp = 1:200 % Schleife mit begrenztem Ende für Kompilierbarkeit. Anstelle `for ii = find(I_Schnitt)'`
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
      for jj_tmp = 1:200 % Schleife mit begrenztem Ende für Kompilierbarkeit. Anstatt `for ii = find(I_keinSchnitt)'`
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
    r_i_i_C(:,coll_chains(i_chain)) = S(ii_min,1:3)';

    %% Neuen Kollisionsort angeben
    % Durch die Prüfung der restlichen kinematischen Kette ist evtl ein neues
    % Segment als Kollisionssegment erkennbar.
    ic_neu(coll_chains(i_chain)) = atlas5_collbodies.I(S(ii_min,5));
    
    continue
    %% Debug: Zeichnen
  end
  % Falls abweichende Kollisionssegmente bestimmt wurden, wiederhole die
  % Prozedur mit den neuen Segmenten. Durch die geänderte Jacobi-Matrix
  % können sich auch die Koordinaten ändern.
  if all(ic_neu == ic)
    break;
  end
  changed_chains = find(ic~=ic_neu);
  ic = uint8(ic_neu);
  continue
  %% Debug: Zeichnen
  
  % Benutze ic_alt, da hier am Ende der iterations-Schleife die Datn nicht
  % konsistent mit ic sind.
  % Die Zeichnung ist im Basis-KS
  T_c_urdf = atlas5_wbody_fkine_urdf_num(qJ');
  figure(802+iterations);clf;
  hold on; grid on;axis equal;view(3);set(802+iterations, 'Renderer','OpenGL');
  for j = 1:31
    atlas_plot_wbody_link_stl(j, uint8(5), T_c_urdf);
  end
  AS = atlas_const(uint8(5));
  title(sprintf('Iteration %d', iterations));
  % title(sprintf('Link %d: %s', ic, AS.LN{ic}), 'interpreter', 'none');
  xlabel('x_0 [m]'); ylabel('y_0 [m]'); zlabel('z_0 [m]')

  for iii = 1:5
    if ic_alt(iii) == 0
      continue
    end
    T_0_i = T_c_mdh(:,:,ic_alt(iii));
      % Körper-KS einzeichnen
      trplot(T_0_i, 'frame', AS.LN{ic_alt(iii)}, 'arrow', 'rgb', 'length', 0.3)

      % Gerade zeichnen

      p = T_0_i*[r_i_i_d_debug(:,iii);1];
  %     p = T_0_i*[r_i_i_C(:,iii);1];
      u = T_0_i(1:3,1:3)*u_i_debug(:,iii);
  %     u = T_0_i(1:3,1:3)*t2r(T_W_0)'*F_i_W(1:3,iii)/norm(F_i_W(1:3,iii));

      d = 0.2;
      plot3([p(1)-d*u(1) p(1)+d*u(1)]', [p(2)-d*u(2) p(2)+d*u(2)]', [p(3)-d*u(3) p(3)+d*u(3)]',...
        'y' , 'LineWidth', 5);

      % Aufpunkt der Geraden zeichnen
      plot3(p(1), p(2), p(3), 'k+', 'MarkerSize', 15, 'LineWidth', 5);

      % Bestimmten Kollisionspunkt einzeichnen
      r_0_0_C = T_0_i*[r_i_i_C(:,iii);1];
      plot3(r_0_0_C(1), r_0_0_C(2), r_0_0_C(3),'mo', 'MarkerSize', 15);
  end
  axis auto
end

% Abschlusskorrektur: Neuberechnung der Krafte mit Jacobimatrix am
% endgueltigen Kontaktpunkt
for i_chain = 1:length(coll_chains)
  %% Effektive Kraft auf Segment-KS
  % Transformationsmatrizen
  % Index in den Gleichungssystem-Variablen (JT, F_i_Ws)
  T_0_i(:,:,coll_chains(i_chain)) = T_c_mdh(:,:,ic(coll_chains(i_chain)));
  T_W_i(:,:,coll_chains(i_chain)) = T_W_0*T_0_i(:,:,coll_chains(i_chain));
  % Jacobi-Matrix zum Segment, an dem die Kollision durch
  % Schwellwertüberschreitung festgestellt wurde
  Jg_ic = atlas5_wbody_body_jacobig_mdh_eulangrpy_num(rpy_base, qJ, ...
    uint8(ic(coll_chains(i_chain))), r_i_i_C(:,coll_chains(i_chain)));
  % Jacobimatrizen werden gestackt zur gemeinsamen berechnung aller
  % Kontakt-Wrenches
  JT(6*(i_chain-1)+1:6*i_chain,:) = Jg_ic;
end

% [0_HaddadinDeAlb2016] equ. (58)
F_i_Ws = pinv(JT')*tau_obs; % Externe Kräfte auf Koordinatenursprünge i ausgedrückt in KS W (identisch mit pinv(JT)*tau_obs
for i_chain = 1:length(coll_chains) 
  F_i_W(:,coll_chains(i_chain)) = F_i_Ws(1+(i_chain-1)*6:i_chain*6);
end
%%
return  %#ok<*UNRCH>

