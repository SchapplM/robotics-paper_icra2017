% Berechne Sensorkräfte aus gegebenen externen Kräften am Roboter.
% 
% Eingabe:
% base_pose
% qJ
% F_ext_feet [nE*6x1]
%   nE verschiedene externe Kräfte (wrench)
% link_ind_in [nEX1]
%   Angriffsort der externen Kräfte. Die Kräfte greifen im Ursprung des
%   spezifizierten Körpers an
% LI_FS [nSx1]
%   Segment-Nummer des direkt an den Sensor nachfolgenden Segmentes.
% 
% Ausgabe:
% F_W_S_ges [nS*6x1]
%   gestapelte Sensorkräfte der externen Kräfte

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

function F_W_S_ges = atlas5_extforce2sensorforce(base_pose, qJ, F_ext_ges, link_ind_in, LI_FS)

%% Init
%#codegen

nS = length(LI_FS);
F_W_S_ges = zeros(6*nS,1);

R_W_B1 = quat2r(base_pose(4:7)');
T_c_mdh = atlas5_wbody_fkine_mdh_num(qJ');

% Nachfolger-Indizes
[~, ~, ~, ~, ~, ~, v] = atlas5_wbody_parameter_mdh();


% TODO: Aufruf von extforce2sensorforce testen
% T_c_mdh_stack = NaN((AS.NJ+1),16); % Zur Übergabe an Funktionen
% for i = 1:(AS.NJ+1)
%   T_c_mdh_stack(i,:) = reshape(T_c_mdh(:,:,i),1,16);
% end
% nE = length(link_ind_in);
% F_ext_ges_mat = reshape(F_ext_ges, 6, nE);
% 
% F_W_S_ges_matrix = extforce2sensorforce(T_c_mdh_stack, R_W_B1, v, F_ext_ges_mat, link_ind_in, zeros(3,nE), LI_FS);
% F_W_S_ges = reshape(F_W_S_ges_matrix, 6*nS,1);
%% Berechnung
% Gehe über alle externen Kräfte
for i = 1:length(link_ind_in)
  link_ind = link_ind_in(i);
  % externe Kraft auf den Roboter
  F_ext_rob_W_i = F_ext_ges((6*(i-1)+1):6*i);
  
  r_B1_B1_C1  = T_c_mdh(1:3,4,link_ind);
  r_W_B1_C1 = R_W_B1 * r_B1_B1_C1;
  
  % Ordne die Kraft den richtigen Sensoren zu
  for iS = 1:length(LI_FS)
    % Suche nachfolgende Elemente für den jeweiligen Kraftsensor
    LI_nf = [LI_FS(iS); get_children_from_parent_list(v,LI_FS(iS)-1)+1]; % Nachfolger-Segmentindizes
    if ~any(link_ind == LI_nf)
      continue % Das aktuelle Kollisionssegment ist nicht nachfolger des Sensors
    end

    % Die Sensorkraft wird im Sensor-KS gemessen (=Körper-KS). -> Hebelarm
    % Zur einfacheren Berechnung wird die Kraft im Welt-KS angegeben
    T_B1_S = T_c_mdh(:,:,LI_FS(iS));
    % Anteil der aktuellen externen Kraft dieses Kontaktes (j) auf den
    % aktuell betrachteten Kraftsensor (iS)
    r_B1_B1_S = T_B1_S(1:3,4); % Vektor von der Basis zum Sensor-KS
    r_W_B1_S = R_W_B1*r_B1_B1_S; % Rotation ins Welt-KS
    r_W_S_C1 = -r_W_B1_S + r_W_B1_C1;
    F_W_S_i = F_ext_rob_W_i(1:6)+[zeros(3,1); cross(r_W_S_C1, F_ext_rob_W_i(1:3))];
    
    F_W_S_ges((6*(iS-1)+1):6*iS) = F_W_S_ges((6*(iS-1)+1):6*iS) + F_W_S_i;
  end
end