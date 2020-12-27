% Rechne externe Kräfte in gemessene Sensorkräfte von Kraft-Momenten-Sensoren um
% 
% Eingabe:
% T_c_mdh_stack [31x16] double
%   Transformationsmatrizen zu allen Körpern. Mit reshape(1,16) zu
%   Zeilenvektoren umgeformtes [4x4x31]-Array
% R_W_B [3x3]
%   Rotation vom Welt- ins Basis-KS
% v [30x1]
%   Vorgänger-Indizes (MDH)
% F_ext_ges [6xnE]
%   Matrix mit am Roboter angreifenden externen Kräften
% link_ind_in [nEX1]
%   Angriffs-Segment der externen Kräfte.
% r_i_i_C [3xnE]
%   Angriffskoordinaten der externen Kräfte (im KS von link_ind_in)
% LI_FS [nSx1]
%   Segment-Nummer der direkt an den Sensoren nachfolgenden Segmente.
% 
% Ausgabe:
% F_W_S_ges [6xnS]
%   nS Sensorkräfte aus den externen Kräfte

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

function F_W_S_ges = extforce2sensorforce(T_c_mdh_stack, R_W_B, v, F_ext_ges, link_ind_in, r_i_i_C, LI_FS)

%% Init
nS = length(LI_FS);
F_W_S_ges = zeros(6,nS);

nT = size(T_c_mdh_stack, 1);
T_c_mdh = NaN(4,4,nT);
for i = 1:nT
  T_c_mdh(:,:,i) = reshape(T_c_mdh_stack(i, :),4,4);
end

%% Berechnung

% Gehe über alle externen Kräfte
for i = 1:length(link_ind_in)
  link_ind = link_ind_in(i);
  % externe Kraft auf den Roboter
  F_ext_rob_W_i = F_ext_ges(:,i);
  
  
  r_B_B_C  = T_c_mdh(1:4,1:4,link_ind)*[r_i_i_C(:,i);1];
  r_W_B_C = R_W_B * r_B_B_C(1:3);
  

  % Ordne die Kraft den richtigen Sensoren zu
  for iS = 1:length(LI_FS)
    % Suche nachfolgende Elemente für den jeweiligen Kraftsensor
    LI_nf = [LI_FS(iS); get_children_from_parent_list(v,LI_FS(iS)-1)+1]; % Nachfolger-Segmentindizes
    if ~any(link_ind == LI_nf)
      continue % Das aktuelle Kollisionssegment ist nicht nachfolger des Sensors
    end

    % Die Sensorkraft wird im Sensor-KS gemessen (=Körper-KS). -> Hebelarm
    % Zur einfacheren Berechnung wird die Kraft im Welt-KS angegeben
    T_B_S = T_c_mdh(:,:,LI_FS(iS));
    % Anteil der aktuellen externen Kraft dieses Kontaktes (j) auf den
    % aktuell betrachteten Kraftsensor (iS)
    r_B_B_S = T_B_S(1:3,4); % Vektor von der Basis zum Sensor-KS
    r_W_B_S = R_W_B*r_B_B_S; % Rotation ins Welt-KS
    r_W_S_C = -r_W_B_S + r_W_B_C;
    F_W_S_i = F_ext_rob_W_i(1:6)+[zeros(3,1); cross(r_W_S_C, F_ext_rob_W_i(1:3))];

    F_W_S_ges(:,iS) = F_W_S_ges(:,iS) + F_W_S_i;
  end

end
return