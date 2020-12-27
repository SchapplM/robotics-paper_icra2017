% Identifikation der Kontaktkraft bei mehreren Kontakten.
% 
% Eingabe:
% rpy_base [3x1]
%   Basis-Orientierung im Welt-KS in RPY-Euler-Winkeln
% qJ [30x1]
%   Gelenkwinkel
% tau_obs [30x1]
%   Geschätzte externe Kräfte in verallgemeinerten Koordinaten
% ic [5x1]
%   Nummer des Segmentes, an dem die Kollision vermutet wird. Für jede
%   Kette ein Segment möglich. Keine Kollision: 0
% r_i_i_C [3x5]
%   Ortsvektor zum Kollisionspunkt im Körper-KS des Kollisionssegmentes
%   Für jede der 5 Ketten ein Kontaktpunkt
% 
% Ausgabe:
% F_ext_W [6x5]
%   Geschätzte externe Kraft (angreifend am Kontaktort) im Welt-KS. Für
%   jede der 5 Ketten eine Kontaktkraft

% Quelle:
% [ICRA2017] Diese Veröffentlichung

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function F_ext_W = atlas5_wbody_collision_identification_multi(rpy_base, qJ, tau_obs, ic, r_i_i_C)

%% Init
assert(isa(rpy_base,'double') && isreal(rpy_base) && all(size(rpy_base) == [3 1]), ...
  'atlas5_wbody_collision_identification: phi_base has to be [3x1] double');
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [30 1]), ...
  'atlas5_wbody_collision_identification: Joint angles q have to be [30x1] double');
assert(isa(tau_obs,'double') && isreal(tau_obs) && all(size(tau_obs) == [36 1]), ...
  'atlas5_wbody_collision_identification: Observer output tau_obs has to be [36x1] double');
assert(isa(ic,'uint8')  && all(size(ic) == [5 1]), ...
  'atlas5_wbody_collision_identification: ic has to be [5x1] uint8');
assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 5]), ...
  'atlas5_wbody_collision_identification: r_i_i_C has to be [3x5] double');

%% Init
F_ext_W = NaN(6,5);

if all (ic == 0)
  return
end

nColl = sum(ic~=0);

%% Berechnung
% [0_HaddadinDeAlb2016] equ. (62)
% [ICRA2017], equ. (30)

% Jacobi-Matrix zusammenstellen
JT = NaN(nColl*6, 36);
ii = 0;
for i_chain = 1:5 % Schleife mit Obergrenze. Anstelle von `for i_chain = find(ic~=0)'`
  if ic(i_chain) == 0
    continue
  end
  ii = ii+1;
  Jg_ic = atlas5_wbody_body_jacobig_mdh_eulangrpy_num(rpy_base, qJ, ...
    ic(i_chain), r_i_i_C(:,i_chain));
  % Jacobimatrizen werden gestackt zur gemeinsamen berechnung aller
  % Kontakt-Wrenches
  JT((6*(ii-1)+1):(6*ii),:) = Jg_ic;
end

% Externe Kräfte berechnen
F_i_Ws = pinv(JT')*tau_obs;

% Umformen
F_ext_W(:, ic~=0) = reshape(F_i_Ws, 6, nColl);

