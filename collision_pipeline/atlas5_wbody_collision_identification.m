% Identifikation der Kontaktkraft. Berechne 
% 
% Eingabe:
% rpy_base [3x1]
%   Basis-Orientierung im Welt-KS in RPY-Euler-Winkeln
% qJ [30x1]
%   Gelenkwinkel
% tau_obs [30x1]
%   Geschätzte externe Kräfte in verallgemeinerten Koordinaten
% ic [1x1]
%   Nummer des Segmentes, an dem die Kollision vermutet wird
% r_i_i_C [3x1]
%   Ortsvektor zum Kollisionspunkt im Körper-KS des Kollisionssegmentes
% 
% Ausgabe:
% F_ext_W [6x1]
%   Geschätzte externe Kraft (angreifend am Kontaktort) im Welt-KS

% Quelle:
% [0_HaddadinDeAlb2016] Haddadin, De Luca, Albu-Schäffer: Robot Collisions:
% Detection, Isolation, and Identification, Submitted to IEEE Transactions
% on Robotics, 2016 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function F_ext_W = atlas5_wbody_collision_identification(rpy_base, qJ, tau_obs, ic, r_i_i_C)

%% Init
assert(isa(rpy_base,'double') && isreal(rpy_base) && all(size(rpy_base) == [3 1]), ...
  'atlas5_wbody_collision_identification: phi_base has to be [3x1] double');
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [30 1]), ...
  'atlas5_wbody_collision_identification: Joint angles q have to be [30x1] double');
assert(isa(tau_obs,'double') && isreal(tau_obs) && all(size(tau_obs) == [36 1]), ...
  'atlas5_wbody_collision_identification: Observer output tau_obs has to be [36x1] double');
assert(isa(ic,'uint8')  && all(size(ic) == [1 1]), ...
  'atlas5_wbody_collision_identification: ic has to be [1x1] uint8');
assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ...
  'atlas5_wbody_collision_identification: r_i_i_C has to be [3x1] double');
%% Berechnung
% [0_HaddadinDeAlb2016] equ. (62)
Jg_C = atlas5_wbody_body_jacobig_mdh_eulangrpy_num(rpy_base, qJ, ic, r_i_i_C);
F_ext_W = pinv(Jg_C') * tau_obs;
