% Kollisionserkennung durch Schwellwertüberschreitung des geschätzten
% Störmoments
% 
% Eingabe:
% rpy_base [3x1]
%   Basis-Orientierung im Welt-KS in RPY-Euler-Winkeln
% tau_obs [30x1]
%   Geschätzte externe Kräfte in verallgemeinerten Koordinaten
% FB_obs_thresh [6x1]
%   Schwellwert für Kollisionserkennung der Basiskräfte und -momente im
%   Welt-KS
% tauJ_obs_thresh [30x1]
%   Schwellwert für Kollisionserkennung der Gelenkmomente
%
% Ausgabe:
% cd [1x1]
%   1: Kollision erkannt

% Quelle:
% [0_HaddadinDeAlb2016] Haddadin, De Luca, Albu-Schäffer: Robot Collisions:
% Detection, Isolation, and Identification, Submitted to IEEE Transactions
% on Robotics, 2016 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function [cd, Ib, Ij] = atlas5_wbody_collision_detection(rpy_base, tau_obs, FB_obs_thresh, tauJ_obs_thresh)

%% Init
assert(isa(rpy_base,'double') && isreal(rpy_base) && all(size(rpy_base) == [3 1]), ...
  'atlas5_wbody_collision_detection: rpy_base has to be [3x1] double');
assert(isa(tau_obs,'double') && isreal(tau_obs) && all(size(tau_obs) == [36 1]), ...
  'atlas5_wbody_collision_detection: Observer output tau_obs has to be [36x1] double');
assert(isa(FB_obs_thresh,'double') && isreal(FB_obs_thresh) && all(size(FB_obs_thresh) == [6 1]), ...
  'atlas5_wbody_collision_detection: Threshold FB_obs_thresh has to be [6x1] double');
assert(isa(tauJ_obs_thresh,'double') && isreal(tauJ_obs_thresh) && all(size(tauJ_obs_thresh) == [30 1]), ...
  'atlas5_wbody_collision_detection: Threshold tauJ_obs_thresh has to be [30x1] double');
%% Berechnung
% Berechne Geschätzte Störkräfte und -momente auf die Basis im Welt-KS aus
% Kräften entlang der verallgemeinerten Koordinaten, RPY-Konvention)
T_basevel = eulxyzjac([rpy_base(1); rpy_base(2); rpy_base(3)]);
FB_obs_W = [tau_obs(1:3); (T_basevel') \ tau_obs(4:6)];

tauJ_obs = tau_obs(7:end);

% Prüfe Schwellwertüberschreitung für Basis-Kräfte und Gelenkmomente
% [0_HaddadinDeAlb2016] equ. (51)
cd = false;
Ib = abs(FB_obs_W) > FB_obs_thresh;
Ij = abs(tauJ_obs) > tauJ_obs_thresh;
if any( Ib );
  cd = true;
end
if any( Ij )
  cd = true;
end