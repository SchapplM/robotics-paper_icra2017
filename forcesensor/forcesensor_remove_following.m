% Entferne die Messung nachfolgender Kraftsensoren bei mehreren
% Kraftsensoren in einer kinematischen Kette
% 
% Eingabe:
% R_W_0 [3x3]
%   Rotation vom Welt- ins Basis-KS
% T_c_mdh_stack [31x16] double
%   Transformationsmatrizen zu allen Körpern. Mit reshape(1,16) zu
%   Zeilenvektoren umgeformtes [4x4x31]-Array
% F_W_S_ges [6xnS]
%   nS Sensorkräfte aus den externen Kräfte
% LI_FS [nSx1]
%   Segment-Nummer der direkt an den Sensoren nachfolgenden Segmente.
% NFS [nSxnS]
%   Matrix, die angibt, ob der betrachtete Kraftsensor der Zeile den
%   betrachteten Kraftsensor der Spalte als Nachfolger hat
% IIo [nSx1]
%   Permutationsvektor für eine Reihenfolge, bei der die Sensoren von außen
%   nach innen aufgelistet werden.
% 
% F_W_S_red [6xnS]
%   nS Sensorkräfte reduziert um Signale der nachfolgenden Sensoren, sodass
%   nur die externen Kräfte zwischen dem jeweiligen Sensor und dem direkt
%   nachfolgenden Sensor enthalten sind.
% 
% Siehe auch:
% atlas5_forcesensor_settings_default.m, 
% atlas5_simulink_forcesensor_colldet_postprocess.m,
% atlas5wbody_collisions_statistics_mc.m

% Quelle
% [0_VorSch2016] Geplante ICRA2017-Veröffentlichung

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover


function F_W_S_red = forcesensor_remove_following(R_W_0, T_c_mdh_stack, ...
                                                  F_W_S_ges, LI_FS, NFS, IIo)
                                                
%% Eingangsvariablenprüfung
assert(isa(R_W_0, 'double') && all(size(R_W_0) == [3 3]), ...
  'forcesensor_remove_following: R_W_0 has to be [3x3]');
assert(isa(T_c_mdh_stack, 'double') && size(T_c_mdh_stack,1) <= 50 ...
                                    && size(T_c_mdh_stack,2) == 16, ...
  'forcesensor_remove_following: R_W_0 has to be [3x3]');                                     
                                                
%% Init
nT = size(T_c_mdh_stack, 1);
T_c_mdh = NaN(4,4,nT);
for i = 1:nT
  T_c_mdh(:,:,i) = reshape(T_c_mdh_stack(i, :),4,4);
end

%% Nachfolgende Kraftsensoren herausrechnen
% Nach diesem Schritt enthält jeder Kraftsensor nur noch Messungen externer
% Kräfte, die an der Kette zwischen diesem und dem nächsten Sensor
% angreifen.
% [0_VorSch2016] Gl. (27), Alg.1?
F_W_S_red = F_W_S_ges;

for iS = IIo(:)' % Gehe alle Sensoren von außen nach innen durch
  % Indizes der nachfolgenden Kraftsensoren
  LI_nfS = find(NFS(iS,:));

  for iT = LI_nfS % Schleife über Nachfolger-Sensoren (ungeordnet)
    % Korrekturwert: Gemessenes Signal des äußeren Sensors mit
    % Berücksichtigung des Hebelarms
    r_B_B_S = T_c_mdh(1:3,4,LI_FS(iS));
    r_B_B_T = T_c_mdh(1:3,4,LI_FS(iT));
    r_W_S_T = R_W_0 * (-r_B_B_S+r_B_B_T);

    FT = F_W_S_red(1:6, iT);
    F_korr_it = FT ... % Signal des nachfolgenden Sensors iT (zu iS)
       + [zeros(3,1);cross(r_W_S_T, FT(1:3))];
    F_W_S_red(1:6, iS) = F_W_S_red(1:6, iS) - F_korr_it;
  end
end