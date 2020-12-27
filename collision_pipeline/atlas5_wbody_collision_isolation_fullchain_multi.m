% Isolierung des Kollisionsortes (Segment und Koordinaten)
% 
% Methode:
% Hier nur Erkennung einer einzigen Kollision pro kinematischer Kette
% (Arme, Beine, Torso), die hinter dem letzten Gelenk liegt, an dem das
% Störmoment in dieser Kette gemessen wurde. 
% 
% Eingabe:
% rpy_base [3x1]
%   Basis-Orientierung im Welt-KS in RPY-Euler-Winkeln
% qJ [30x1]
%   Gelenkwinkel
% tau_obs [36x1]
%   Geschätzte externe Kräfte in verallgemeinerten Koordinaten
% tau_obs_thresh [36x1]
%   Schwellwert für Kollisionserkennung der Gelenkmomente
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
%   Kollisionssegmente. Die Reighenfolge ist wie oben, falls keine
%   Kollision mit einer bestimmten Kette vorliegt, werden NaNs im
%   entsprechenden Eintrag zurück gegeben.
% F_i_W [6x5]
%   Kontakt Wrenches zu den Kollisionspunkten im Welt-KS. Die Reighenfolge
%   ist wie oben, falls keine Kollision mit einer bestimmten Kette
%   vorliegt, werden NaNs im entsprechenden Eintrag zurück gegeben.

% Quelle:
% [0_HaddadinDeAlb2016] Haddadin, De Luca, Albu-Schäffer: Robot Collisions:
% Detection, Isolation, and Identification, Submitted to IEEE Transactions
% on Robotics, 2016 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function [ic, r_i_i_C, F_i_W] = atlas5_wbody_collision_isolation_fullchain_multi(rpy_base, qJ, tau_obs, ...
  tau_obs_thresh, atlas5_collbodies)

%% Init
assert(isa(rpy_base,'double') && isreal(rpy_base) && all(size(rpy_base) == [3 1]), ...
  'atlas5_wbody_collision_isolation_fullchain_multi: phi_base has to be [3x1] double');
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [30 1]), ...
  'atlas5_wbody_collision_isolation_fullchain_multi: Joint angles q have to be [30x1] double');
assert(isa(tau_obs,'double') && isreal(tau_obs) && all(size(tau_obs) == [36 1]), ...
  'atlas5_wbody_collision_isolation_fullchain_multi: Observer output tau_obs has to be [36x1] double');
assert(isa(tau_obs_thresh,'double') && isreal(tau_obs_thresh) && all(size(tau_obs_thresh) == [36 1]), ...
  'atlas5_wbody_collision_isolation_fullchain_multi: Threshold tauJ_obs_thresh has to be [36x1] double');
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
    'atlas5_wbody_collision_isolation_fullchain_multi: atlas5_collbodies does not have the correct structure properties');


%% Kontakt-Segmente erkennen
% Zur Definition der Körper, siehe atlas5_wbody_fkine_mdh_num.m

% Ketten bestimmen, in denen Kollisionen vorliegen
I_coll = false(5,7); % Zeilen: Ketten. Spalten: Gelenke der Ketten (max. 7 Elemente)
I_coll(1,1:5) = [any(abs(tau_obs(1:6)) > tau_obs_thresh(1:6)); abs(tau_obs(7:10)) > tau_obs_thresh(7:10)]; % torso chain
I_coll(2,1:6) = abs(tau_obs(11:16)) > tau_obs_thresh(11:16); % left leg chain
I_coll(3,1:6) = abs(tau_obs(17:22)) > tau_obs_thresh(17:22); % right leg chain
I_coll(4,:) = abs(tau_obs(23:29)) > tau_obs_thresh(23:29); % left arm chain
I_coll(5,:) = abs(tau_obs(30:36)) > tau_obs_thresh(30:36); % right arm chain
chain_starts = [0 5 11 17 24];

ic0     = uint8(zeros(5,1));
for chain = 1:length(I_coll(:,1))
  if ~any(I_coll(chain,:))
    ic0(chain) = uint8(0); % Keine Kollision in dieser Kette
  else
    % [0_HaddadinDeAlb2016] equ. (55)
    % Der Kollisionskörper ist hinter dem letzten Gelenk der Kette, bei dem
    % die Kollision festgestellt wurde.
    ic0(chain) = uint8(find(I_coll(chain,:), 1, 'last' ) + chain_starts(chain));
  end
end

% Zunaechst werden torsokontakte ignoriert, falls andere Kontakte
% vorliegen, falls mit dieser Annahme ein nicht plausibles Ergebnis erzielt
% wird, werden die Identifizierbaren Kontakte Systematisch geprueft
ic = ic0;
if ic0(1) && any(ic0(2:5))
  ic(1) = 0;
end

% Funktionsaufruf der eigentlichen Berechnung
[ic, r_i_i_C, F_i_W] = atlas5_wbody_collision_isolation_fullchain_multi_ic_in( ...
  ic, rpy_base, qJ, tau_obs, atlas5_collbodies);

% Korrekturschritt, falls widerspruechliche Ergebnisse rauskommen (Momente
% ungleich null) in diesem Fall werden die Kollisionssegmente, fuer die
% eine Isolation moeglich ist. Denkbar waere, hier ueber den Rang der
% Jacobimatrix die Gelenke herauszusuchen
if ic0(2)>=9 && ic0(3)>=15 && ic0(4)>=22 && ic0(5)>=29
  ic_test = [1 18 25 3 4];
elseif sum(ic0~=0)<=2 && (ic0(2)==0 || ic0(3)==0)
  ic_test = [6 12 1 18 25 3 4];
elseif sum(ic0~=0)<=2 && ic0(4)==0 && ic0(5)==0
  ic_test = [1 18 25 3 4];
else
  ic_test = [1 18 25 3 4];
end
i = 1;
min_max_F_i_W=max(max(abs(F_i_W(4:6,:))/max(max(abs(F_i_W)))));
while any(any(abs(F_i_W(4:6,:))/max(max(abs(F_i_W)))>1e-10)) && i<=length(ic_test) % TODO: threshold muss evtl. noch getuned werden
    ic_ = ic0;
    ic_(1) = ic_test(i);
    [ic_, r_i_i_C_, F_i_W_] = atlas5_wbody_collision_isolation_fullchain_multi_ic_in( ...
        ic_, rpy_base, qJ, tau_obs, atlas5_collbodies);
    if max(max(abs(F_i_W_(4:6,:))/max(max(abs(F_i_W_)))))<min_max_F_i_W
      min_max_F_i_W = max(max(abs(F_i_W_(4:6,:))/max(max(abs(F_i_W_)))));
      ic = ic_;
      r_i_i_C = r_i_i_C_;
      F_i_W = F_i_W_;
    end
    i = i+1;
end
ic1=ic0;
ic1((ic1<11 & ic1>5) | (ic1<17 & ic1>11) | (ic1<24 & ic1>17) | (ic1<30 & ic1>24))=ic1((ic1<11 & ic1>5) | (ic1<17 & ic1>11) | (ic1<24 & ic1>17) | (ic1<30 & ic1>24))+1;
i=1;
while any(any(abs(F_i_W(4:6,:))/max(max(abs(F_i_W)))>1e-10)) && i<=length(ic_test) % TODO: threshold muss evtl. noch getuned werden
    ic_ = ic1;
    ic_(1) = ic_test(i);
    [ic_, r_i_i_C_, F_i_W_] = atlas5_wbody_collision_isolation_fullchain_multi_ic_in( ...
        ic_, rpy_base, qJ, tau_obs, atlas5_collbodies);
    if max(max(abs(F_i_W_(4:6,:))/max(max(abs(F_i_W_)))))<min_max_F_i_W
      min_max_F_i_W = max(max(abs(F_i_W_(4:6,:))/max(max(abs(F_i_W_)))));
      ic = ic_;
      r_i_i_C = r_i_i_C_;
      F_i_W = F_i_W_;
    end
    i = i+1;
end
