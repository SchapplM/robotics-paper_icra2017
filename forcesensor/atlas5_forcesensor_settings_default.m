% Standardeinstellungen für Kraftsensoren
% 
% Erstelle Struktur ForceSensorStruct mit Feldern
% LI [nSx1]
%   Liste der Nachfolger-Segmente des Roboters
% IIo [nSx1]
%   Permutationsvektor für eine Reihenfolge, bei der die Sensoren von außen
%   nach innen aufgelistet werden.
% nS [1x1]
%   Anzahl der Kraftsensoren.
% SI_CD [1xnSCD]
%   Liste von Sensoren zur Kollisionserkennung
% NFM [nSxnL]
%   Für jeden Sensor eine Zeile mit Bool-Variablen, ob das Segment der
%   betrachteten Spalte ein Nachfolger des Sensors ist
% DNFM [nSxnL]
%   gleiches Format für direkte Nachfolger, die durch den nächsten
%   Kraftsensor begrenzt werden.
% NFS [nSxnS]
%   Matrix, die angibt, ob der betrachtete Kraftsensor der Zeile den
%   betrachteten Kraftsensor der Spalte als Nachfolger hat
% nNFS [nSx1]
%   Anzahl der zu jedem Sensor insgesamt nachfolgenden Sensoren
% NFSo [nSxnS]
%   Indizes der nachfolgenden Sensoren in der Reihenfolge von außen nach
%   innen. Dient zum Abziehen der Sensorsignale von folgenden Sensoren
%   Nicht belegte Einträge sind mit NaN aufgefüllt.
% CN [nSx1]
%   Nummer der Kette, die der Sensor misst
%   Ketten: 1=Torso, 2=lLeg, 3=rLeg, 4=lArm, 5=rArm

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

function ForceSensorStruct = atlas5_forcesensor_settings_default(usr_debug)

%% Init
AS = atlas_const(5);

if nargin == 0
  usr_debug = false;
end

%% Kraftsensoren

% Kraftsensoren am Anfang und am Ende jeder kinematischen Kette, zusätzlich
% ein Sensor hinter dem Torso-Dreifachgelenk
ForceSensorStruct = struct('LI', uint8([AS.LI_lArm(7); AS.LI_rArm(7); AS.LI_lLeg(6); AS.LI_rLeg(6); ...
  AS.LI_Torso(4); AS.LI_lArm(1); AS.LI_rArm(1); AS.LI_lLeg(1); AS.LI_rLeg(1)]));

ForceSensorStruct.nS = length(ForceSensorStruct.LI);

% Nutze die Sensoren 5-9 zur Kollisionserkennung. Die Sensoren 1-4 dienen
% zum Herausrechnen erwünschter Kräfte an den Endeffektoren
ForceSensorStruct.SI_CD = uint8([5 6 7 8 9]);

% Vorgänger-Indizes der Segmente
[~, ~, ~, ~, ~, ~, v] = atlas5_wbody_parameter_mdh();

nS = length(ForceSensorStruct.LI);
LI_FS = ForceSensorStruct.LI;
% Bestimme Nachfolgende Segmente jedes Kraftsensors bis zum nächsten
% Kraftsensor: Matrix der direkten Nachfolger
DNFM = false(nS, AS.NL); 
% Matrix der indirekten Nachfolger
NFM = false(nS, AS.NL); 
% Matrix der Nachfolger-Sensoren
NFS = false(nS, nS);
for iS = 1:nS
  % Indizes der nachfolgenden Kraftsensoren
  LI_nf = [LI_FS(iS); get_children_from_parent_list(v,LI_FS(iS)-1)+1]; % Nachfolger-Segmentindizes
  NFM(iS,LI_nf) = true;
end
ForceSensorStruct.NFM = NFM;

% Bestimme NFS (nachfolgende Sensoren
for iS = 1:nS
  LI_nf = find(NFM(iS,:));
  SI_nfS = []; % Sensor-Indizes (keine Segment-Indizes)
  for jj = 1:nS
    if any(LI_FS(jj) == LI_nf) && jj ~= iS
      % Kraftsensor jj ist Nachfolger zu Kraftsensor iS
      SI_nfS = [SI_nfS; jj]; %#ok<AGROW>
    end
  end
  NFS(iS,SI_nfS) = true;
end
ForceSensorStruct.NFS = NFS;

% Ordne die nachfolgenden Sensoren von außen nach innen
NFSo = NaN(nS,nS);
n_nfS = NaN(nS,1);
for iS = 1:nS
  % Indizes der nachfolgenden Kraftsensoren (ungeordnet)
  SI_nfS = find(NFS(iS,:));
  n_nfS(iS) = length(SI_nfS); % Anzahl der insgesamt nachfolgenden Sensoren
  % Links, zu den Sensoren
  LI_nfS = LI_FS(SI_nfS);
  LI_nf = find(NFM(iS,:));
  % [Link-Indizes der nachfolgenden Sensoren, Reihenfolge]
  LI_nfS_Rf = NaN(length(LI_nfS),1); % Position der Segments des nachfolgenden Sensors in der kinematischen Kette
  % Ordne die nachfolgenden Sensoren
  for ii = 1:length(LI_nfS)
    LI_nfS_Rf(ii) = (find(LI_nf == LI_nfS(ii)));
  end
  [~,II] = sort(LI_nfS_Rf); % Permutationsvektor
  NFSo(iS,1:length(SI_nfS)) = SI_nfS(II);
end
ForceSensorStruct.NFSo = NFSo;
ForceSensorStruct.nNFS = n_nfS;

% Ordne alle Sensoren von außen nach innen
% Konvention: Segmente mit höheren Nummern sind in der kinematischen Kette
% immer weiter außen als das vorherigen Segment.
[~,IIio] = sort(LI_FS);
ForceSensorStruct.IIo = flipud(IIio);

% Bestimme DNFM (direkte-Nachfolger-Matrix)
for iS = fliplr(IIio') % von außen nach innen durchgehen
  % Indizes der nachfolgenden Kraftsensoren (ungeordnet)
  SI_nfS = find(NFS(iS,:));
  % Entferne alle Segment-Indizes hinter dem nächsten Kraftsensor. Übrig
  % bleiben nur Segment-Indizes zwischen diesem und dem nächsten Sensor
  DNFM(iS,:) = NFM(iS,:); % Starte mit allen nachfolgenden Segmenten
  for iT = SI_nfS % Schleife über Nachfolger-Sensoren
    DNFM(iS,:) = DNFM(iS,:) & (~DNFM(iT,:));
  end
end
ForceSensorStruct.DNFM = DNFM;

% Nummer der Kette, die der Sensor misst
% Ketten: 1=Torso, 2=lLeg, 3=rLeg, 4=lArm, 5=rArm
CN = uint8(zeros(nS,1));
for i = 1:nS
  if any(LI_FS(i) == AS.LI_Torso)
    CN(i) = 1;
  elseif any(LI_FS(i) == AS.LI_lLeg)
    CN(i) = 2;
  elseif any(LI_FS(i) == AS.LI_rLeg)
    CN(i) = 3;
  elseif any(LI_FS(i) == AS.LI_lArm)
    CN(i) = 4;
  elseif any(LI_FS(i) == AS.LI_rArm)
    CN(i) = 5;    
  else
   error('Sensor sitzt an nicht definierter Kette');
  end
end
ForceSensorStruct.CN = CN;


% Probe
if usr_debug
  for iS = 1:nS
    fprintf('Kraftsensor %d. Segment %d\n', iS, LI_FS(iS));
    fprintf('Kraftsensor %d.         Nachfolger-Sensoren %s.\n', iS, disp_array(find(NFS(iS,:)), '%d'));
    fprintf('Kraftsensor %d. geordn. Nachfolger-Sensoren %s.\n', iS, disp_array(ForceSensorStruct.NFSo(iS,1:ForceSensorStruct.nNFS(iS)), '%d'));
    fprintf('Kraftsensor %d.         Nachfolger-Segmente %s.\n', iS, disp_array(find(NFM(iS,:)), '%d'));
    fprintf('Kraftsensor %d. Dir.    Nachfolger-Segmente %s.\n', iS, disp_array(find(DNFM(iS,:)), '%d'));
  end
end