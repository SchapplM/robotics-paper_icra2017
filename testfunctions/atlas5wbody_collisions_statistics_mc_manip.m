% Statistische Untersuchung von Kontakten an Vier Endeffektor-Segmenten
% und einem zusätzlichen Segment für den Atlas-Roboter
% 
% Nehme zwei Kontakte an unterschiedlichen Ketten des Roboters an

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

clc
clear

mex_script_dependencies(mfilename('fullpath'))
rng(0); % Zurücksetzen des Zufallsgenerators, um reproduzierbare Ergebnisse zu erhalten

%% Init

debug = false;

atlas_version = uint8(5);
AS = atlas_const(atlas_version);

% Anzahl der betrachteten Kontaktsituationen
NC = 5000;

% Roboter-Ersatzkörper laden
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
atlas5_collbodies = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'I', 'T', 'P');
% Detailinformationen. Erzeugt durch atlas_contactbody_properties.m
atlas5_collbodies_prop = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'A');

res_path = fullfile(collhdl_pfad, 'results', 'atlas5wbody_collisions_statistics_mc');
mkdirs(res_path);

qJ_wbody = zeros(30,1);
qJ0_lArm = [0 -pi/4 pi/4 pi/3 pi/4 pi/3 0]'; % TODO: Bessere Pose
qJ0_lLeg = [0 pi/12 -pi/6 pi/3 -pi/6 -pi/12]';

qJ_wbody(AS.JI_lLeg) = qJ0_lLeg;
qJ_wbody(AS.JI_rLeg) = AS.q_Leg_mirror .* qJ0_lLeg';

qJ_wbody(AS.JI_lArm) = qJ0_lArm;
qJ_wbody(AS.JI_rArm) = AS.q_Arm_mirror .* qJ0_lArm';

rpy_base = zeros(3,1);
R_W_0 = eulxyz2r([rpy_base(1); rpy_base(2); rpy_base(3)]);
r_W_0 = [0;0;0.9];
T_W_0 = [R_W_0, r_W_0; [0 0 0 1]];
% Kinematik
T_c_mdh = atlas5_wbody_fkine_mdh_num(qJ_wbody');
T_c_urdf = atlas5_wbody_fkine_urdf_num(qJ_wbody');
T_c_Wurdf = NaN(size(T_c_urdf));
T_c_Wmdh = NaN(size(T_c_mdh));
for i = 1:size(T_c_Wurdf,3)
  T_c_Wurdf(:,:,i) = T_W_0 * T_c_urdf(:,:,i);
  T_c_Wmdh(:,:,i) = T_W_0 * T_c_mdh(:,:,i);
end
T_c_mdh_stack = NaN((AS.NJ+1),16); % Zur Übergabe an Funktionen
for i = 1:(AS.NJ+1)
  T_c_mdh_stack(i,:) = reshape(T_c_mdh(:,:,i),1,16);
end

% Nehme jeden Ersatzkörper zu jedem Robotersegment
IKollK = cell((AS.NJ+1),1); % Index, welcher Kollisionskörper für jedes Segment genommen werden soll
for i = 1:(AS.NJ+1) % length(atlas5_collbodies.I)
  IKollK{i} = find(atlas5_collbodies.I == i);
end

% Eigenschaften der Kraftsensoren laden
ForceSensorStruct = atlas5_forcesensor_settings_default();
nFS = ForceSensorStruct.nS;
% Nutze alle Sensoren zur Kollisionserkennung (Abweichung von
% Standardeinstellungen)
ForceSensorStruct.SI_CD = 1:9;

[~, ~, ~, ~, ~, ~, v] = atlas5_wbody_parameter_mdh();

iRS = 0; % Laufindex

% Ergebnis-Variablen
cd_FT = NaN(NC,nFS);
ic_FT_ges = NaN(NC,nFS);
r_i_i_C_FT_ges = NaN(3,nFS,NC);
r_W_W_C_FT_ges = NaN(3,nFS,NC);
er_W_W_C_FT_ges = NaN(NC,5); % Positionsfehler zur Schätzung jedes simulierten Punktes.
F_ext_W_FT_gesm = NaN(6,nFS,NC);

cd_JT = NaN(NC,5);
ic_JT_ges = NaN(NC,5);
r_i_i_C_JT_ges = NaN(3,5,NC);
r_W_W_C_JT_ges = NaN(3,5,NC);
er_W_W_C_JT_ges = NaN(NC,5); % Fehler mit dieser Methode
F_ext_W_JT_gesm = NaN(6,5,NC);

cd_JTp = NaN(NC,5);
ic_JTp_ges = NaN(NC,5);
r_i_i_C_JTp_ges = NaN(3,5,NC);
r_W_W_C_JTp_ges = NaN(3,5,NC);
er_W_W_C_JTp_ges = NaN(NC,5);
F_ext_W_JTp_gesm = NaN(6,5,NC);

icsim_ges = uint8(zeros(NC,5)); % Kollisionssegment aus Simulation. Für jede Kette einen Platz vorgesehen
r_W_W_Csim_ges = NaN(3,5,NC); % Kollisionsort (Welt-KS) aus Simulation. Für jede Kette ein fester Platz
r_i_i_Csim_ges = NaN(3,5,NC); % Kollisionsort (Körper-KS) aus Simulation. Für jede Kette ein fester Platz
F_ext_W_sim_gesm = NaN(6,5,NC); % Externe Kraft/Moment (Moment-Einträge bleiben Null). Für jede Kette ein fester Platz

% Anzahl der korrekten Erkennungen (der Segmente) mit den jeweiligen
% Methoden. Einträge für 0 (keine fünfte Kollision) und alle Segmente (1-31)
n_ickorr_FT = zeros(AS.NL+1,1);
n_ickorr_JT = zeros(AS.NL+1,1);
n_ickorr_JTp = zeros(AS.NL+1,1);
n_ic_komb = zeros(AS.NL+1,1); % Anzahl der Kombinationen für Kollisionen mit diesen beteiligten Segmenten

% Nummern der Ketten
CN = uint8(zeros(AS.NL,1));
for i = 1:AS.NL
  if any(i == AS.LI_Torso)
    CN(i) = 1;
  elseif any(i == AS.LI_lLeg)
    CN(i) = 2;
  elseif any(i == AS.LI_rLeg)
    CN(i) = 3;
  elseif any(i == AS.LI_lArm)
    CN(i) = 4;
  elseif any(i == AS.LI_rArm)
    CN(i) = 5;    
  end
end

CNames = {'Torso', 'lLeg', 'rLeg', 'lArm', 'rArm'};
%% Punkte berechnen
pktdat = fullfile(res_path, 'PunkteListeManip.mat');
if exist(pktdat, 'file')
  ResultStruct = load(pktdat);
  fprintf('Generierte Punkte aus Datei %s geladen.\n', pktdat);
else
  SettingsStruct = struct('IL_Liste', 1:(AS.NJ+1), 'NCL', 1000, 'qJ_wbody', qJ_wbody, ...
    'rpy_base', rpy_base, 'r_W_0', r_W_0);
  ResultStruct = atlas5wbody_collisions_statistics_gen_points(SettingsStruct);
  save(pktdat, '-struct', 'ResultStruct');
end
% Dummy-Zeile mit NaN anhängen, auf die bei vier Kontakten verwiesen werden
% kann
ResultStruct.r_W_W_C = [ResultStruct.r_W_W_C; NaN(1,3)];
ResultStruct.r_i_i_C = [ResultStruct.r_i_i_C; NaN(1,3)];
ResultStruct.F_c_W = [ResultStruct.F_c_W; NaN(1,3)];
ResultStruct.IL = [ResultStruct.IL; 0];
ResultStruct.iCB = [ResultStruct.iCB; 0];

%% Kontakt-Kombinationen
pktkombdat = fullfile(res_path, 'PunktKombListeManip.mat');
if exist(pktkombdat, 'file')
  tmp = load(pktkombdat);
  n_ic_komb = tmp.n_ic_komb;
  KontaktKomb = tmp.KontaktKomb;
  fprintf('Generierte Zufallspunktkombinationen aus Datei %s geladen.\n', pktkombdat);
else
  % Matrix mit Kontaktkombinationen erstellen
  % Kontakt 1-4: Arme und Beine

  KontaktKomb = struct('LI1', NaN(NC,1), 'LI2', NaN(NC,1), ...
    'LI3', NaN(NC,1), 'LI4', NaN(NC,1), 'LI5', NaN(NC,1), ...
    'II1', NaN(NC,1), 'II2', NaN(NC,1), 'II3', NaN(NC,1), 'II4', NaN(NC,1), ...
    'II5', NaN(NC,1));

  I1_erlaubt = false(length(ResultStruct.IL), 1);
  % Der erste Kontakt ist an linker Hand
  for LI_test = AS.LI_lArm(end)
    I1_erlaubt = I1_erlaubt | (ResultStruct.IL == LI_test);
  end
  J1_erlaubt = find(I1_erlaubt);
  
  I2_erlaubt = false(length(ResultStruct.IL), 1);
  % Der zweite Kontakt ist an rechter Hand
  for LI_test = AS.LI_rArm(end)
    I2_erlaubt = I2_erlaubt | (ResultStruct.IL == LI_test);
  end
  J2_erlaubt = find(I2_erlaubt);
  
  I3_erlaubt = false(length(ResultStruct.IL), 1);
  % Der dritte Kontakt ist an linkem Fuss
  for LI_test = AS.LI_lLeg(end)
    I3_erlaubt = I3_erlaubt | (ResultStruct.IL == LI_test);
  end
  J3_erlaubt = find(I3_erlaubt);
  
  I4_erlaubt = false(length(ResultStruct.IL), 1);
  % Der vierte Kontakt ist an rechtem Fuss
  for LI_test = AS.LI_rLeg(end)
    I4_erlaubt = I4_erlaubt | (ResultStruct.IL == LI_test);
  end
  J4_erlaubt = find(I4_erlaubt);
  
  I5_erlaubt = false(length(ResultStruct.IL), 1);
  % Der fuenfte Kontakt ist irgendwo, aber nicht an Haenden oder Fuessen
  for LI_test = [AS.LI_Torso AS.LI_lLeg(1:end-1) AS.LI_rLeg(1:end-1) AS.LI_lArm(1:end-1) AS.LI_rArm(1:end-1)] % TODO: erweitern auf andere chains
    I5_erlaubt = I5_erlaubt | (ResultStruct.IL == LI_test);
  end
  J5_erlaubt = find(I5_erlaubt);
  
  for i = 1:NC
    % Wähle ersten Kontakt
    iitmp1 = randperm(sum(I1_erlaubt),1); % zufälliger Index in J1_erlaubt
    II1 = J1_erlaubt(iitmp1); % Index des ersten Kontakts in ResultStruct
    LI1 = ResultStruct.IL(II1);

    % Wähle zweiten Kontakt
    iitmp2 = randperm(sum(I2_erlaubt),1); % zufälliger Index in J2_erlaubt
    II2 = J2_erlaubt(iitmp2); % Index des zweiten Kontakts in ResultStruct
    LI2 = ResultStruct.IL(II2);
    
    % Wähle dritten Kontakt
    iitmp3 = randperm(sum(I3_erlaubt),1); % zufälliger Index in J2_erlaubt
    II3 = J3_erlaubt(iitmp3); % Index des zweiten Kontakts in ResultStruct
    LI3 = ResultStruct.IL(II3);
    
    % Wähle vierten Kontakt
    iitmp4 = randperm(sum(I4_erlaubt),1); % zufälliger Index in J2_erlaubt
    II4 = J4_erlaubt(iitmp4); % Index des zweiten Kontakts in ResultStruct
    LI4 = ResultStruct.IL(II4);
    
    % Wähle fuenften Kontakt. In 10% der Fälle keinen Kontakt durchführen
    if rand(1) > 0.9
      LI5 = 0;
      II5 = length(ResultStruct.IL); % Dummy-Index der auf NaN verweist
    else
      iitmp5 = randperm(sum(I5_erlaubt),1); % zufälliger Index in J2_erlaubt
      II5 = J5_erlaubt(iitmp5); % Index des zweiten Kontakts in ResultStruct
      LI5 = ResultStruct.IL(II5);
    end
    % Speichere die Kontakte so ab, dass der erste Kontakt am niedrigeren
    % Segment ist
    KontaktKomb.II1(i) = II1;
    KontaktKomb.II2(i) = II2;
    KontaktKomb.II3(i) = II3;
    KontaktKomb.II4(i) = II4;
    KontaktKomb.II5(i) = II5;
    KontaktKomb.LI1(i) = LI1;
    KontaktKomb.LI2(i) = LI2;
    KontaktKomb.LI3(i) = LI3;
    KontaktKomb.LI4(i) = LI4;
    KontaktKomb.LI5(i) = LI5;

    n_ic_komb(KontaktKomb.LI5(i)+1) = n_ic_komb(KontaktKomb.LI5(i)+1) + 1;
  end
  save(pktkombdat, 'n_ic_komb', 'KontaktKomb');
end

fprintf('Kombinationen nur mit vier Kollisionen: %d/%d=%1.1f%%\n', ...
  sum(KontaktKomb.LI5==0), NC,  1e2*sum(KontaktKomb.LI5==0)/NC);

t_iso_beo = NaN(1,sum(sum(n_ic_komb)));
t_iso_fs  = NaN(1,sum(sum(n_ic_komb)));
ind_pkt = 1;
%% Kontakt-Isolation für alle Zufallspunkte berechnen
for iC = 1:NC
  %% Kontaktpunkte holen
  II1 = KontaktKomb.II1(iC);
  II2 = KontaktKomb.II2(iC);
  II3 = KontaktKomb.II3(iC);
  II4 = KontaktKomb.II4(iC);
  II5 = KontaktKomb.II5(iC);
  ictmp = ResultStruct.IL([II5, II3, II4, II1, II2]);
  icsim_ges(iC,1:5) = ictmp;
  LI_Sim = sort( icsim_ges(iC, icsim_ges(iC,:)~=0) ); % aktive Segmente im Kontakt

  % Permutationsvektor, um aufsteigende Reihenfolge der Segmente zu erhalten
  [~,Rf_Sim] = sort(icsim_ges(iC,:));
  if icsim_ges(iC,1) == 0 % Kein Kontakt: Überspringen in Indizierungsvektor
    Rf_Sim = Rf_Sim(1:4) + 1;
  end
  r_W_W_Csim_ges(:,1:5,iC) = ResultStruct.r_W_W_C([II5, II3, II4, II1, II2],:)';
  r_i_i_Csim_ges(:,1:5,iC) = ResultStruct.r_i_i_C([II5, II3, II4, II1, II2],:)';
  F_ext_W_sim_gesm(:,1:5,iC) = [ResultStruct.F_c_W([II5, II3, II4, II1, II2],:)';zeros(3,5)];
  
  if debug
    for ii = 1:5
      if ictmp(ii) == 0
        continue
      end
      fprintf('Kombination %d. Kontaktpunkt %d: [%s]mm an Segment %d (%s). Kette %s.\n', ...
        iC, ii, disp_array(1e3*r_i_i_Csim_ges(:,ii,iC)', '%1.1f'), icsim_ges(iC,ii), ...
        AS.LN{icsim_ges(iC,ii)}, CNames{CN(icsim_ges(iC,ii))});
    end
  end
  %% Gelenkmomente berechnen
  tauwbody_floatb_ext_sum = 0;
  Jges = [];
  for ii = 1:5
    if icsim_ges(iC,ii) == 0
      continue
    end
    J_wbody_ii = atlas5_wbody_body_jacobig_mdh_eulangrpy_num_mex( ...
      rpy_base, qJ_wbody, uint8(icsim_ges(iC,ii)), r_i_i_Csim_ges(:,ii,iC));
    tauwbody_floatb_ext_ii = J_wbody_ii' * F_ext_W_sim_gesm(:,ii,iC);
    tauwbody_floatb_ext_sum = tauwbody_floatb_ext_sum + tauwbody_floatb_ext_ii;
    Jges = [Jges; J_wbody_ii]; %#ok<AGROW>
  end
  RangJ = rank(Jges); % Rang zur Beurteilung, ob Isolation überhaupt gelingen kann
  %% Kraftsensor-Signale berechnen
  I_Sim = (icsim_ges(iC,:) ~= 0);
  F_W_S_ges = extforce2sensorforce(T_c_mdh_stack, R_W_0, v, ...
    F_ext_W_sim_gesm(:,I_Sim,iC), ...
    icsim_ges(iC,I_Sim), r_i_i_Csim_ges(:,I_Sim,iC), ForceSensorStruct.LI);
  
  %% Nachfolgende Kraftsensoren herausrechnen
  F_W_S_red = forcesensor_remove_following(R_W_0, T_c_mdh_stack, F_W_S_ges, ForceSensorStruct.LI, ...
    ForceSensorStruct.NFS, ForceSensorStruct.IIo);

  %% Berechnung Isolation mit Gelenkmomenten
  % Isolation nur mit perfekten externen Gelenkmomenten 
  tic
  [ic_JT_ges(iC,:), r_i_i_C_JT_ges(:,:,iC), F_ext_W_JT_gesm(:,:,iC)] = ...
    atlas5_wbody_collision_isolation_fullchain_multi_mex(rpy_base, qJ_wbody, tauwbody_floatb_ext_sum, ...
                                                     ones(36,1)*1e-3, atlas5_collbodies);
  t_iso_beo(ind_pkt) = toc;
  
  % zusätzlich Eingabe der bekannten Kollisionssegmente
  ic_in=icsim_ges(iC,:)';
  [ic_JTp_ges(iC,:), r_i_i_C_JTp_ges(:,:,iC), F_ext_W_JTp_gesm(:,:,iC)] = ...
    atlas5_wbody_collision_isolation_fullchain_multi_ic_in_mex( ...
    ic_in, rpy_base, qJ_wbody, tauwbody_floatb_ext_sum, atlas5_collbodies);
  for ii = 1:5
    if ic_JT_ges(iC,ii) > 0
      r_W_W_C_JT_ges (1:3,ii,iC) = eye(3,4)*T_W_0*T_c_mdh(:,:,ic_JT_ges(iC,ii))* [r_i_i_C_JT_ges(1:3,ii,iC);1];
    end
    if ic_JTp_ges(iC,ii) > 0
      r_W_W_C_JTp_ges(1:3,ii,iC) = eye(3,4)*T_W_0*T_c_mdh(:,:,ic_JTp_ges(iC,ii))*[r_i_i_C_JTp_ges(1:3,ii,iC);1];
    end
  end
  %% Gelenkmoment: Fehler betrachten
  % Prüfe, ob alle Segmente richtig erkannt wurden. 
  I_JT_gut = (ic_JT_ges(iC,:) ~= 0 & ~isnan(ic_JT_ges(iC,:)));
  II_JT = sort( ic_JT_ges(iC, I_JT_gut) );
  I_JTp_gut = (ic_JTp_ges(iC,:) ~= 0 & ~isnan(ic_JTp_ges(iC,:)));
  II_JTp = sort( ic_JTp_ges(iC, I_JTp_gut) );
  
  if length(II_JT) == length(LI_Sim) && all ( II_JT == LI_Sim )
    n_ickorr_JT( icsim_ges(iC,1)+1 ) = n_ickorr_JT( icsim_ges(iC,1)+1 ) + 1;
  end
  if length(II_JTp) == length(LI_Sim) && all ( II_JTp == LI_Sim )
    n_ickorr_JTp( icsim_ges(iC,1)+1 ) = n_ickorr_JTp( icsim_ges(iC,1)+1 ) + 1;
  end
  
%   
%   ic_JT_ges(iC, :)
%   ic_JTp_ges(iC, :)
%   icsim_ges(iC, :)
%   
%   r_W_W_Csim_ges(1:3,:,iC)
%   r_W_W_C_JT_ges(1:3,:,iC)  
%   r_W_W_C_JTp_ges(1:3,:,iC)    
%   
%   F_ext_W_sim_gesm(:,:,iC)
%   F_ext_W_JT_gesm(:,:,iC)
  
  % Prüfe Positionsfehler
  for ii = 1:5 % alle Ketten durchgehen
    er_W_W_C_JT_ges(iC, ii) =  norm( r_W_W_C_JT_ges(1:3,ii,iC)  - r_W_W_Csim_ges(1:3,ii,iC) );
    er_W_W_C_JTp_ges(iC, ii) = norm( r_W_W_C_JTp_ges(1:3,ii,iC) - r_W_W_Csim_ges(1:3,ii,iC) );
  end
  %% Berechnung Isolation mit Kraftsensoren
  Fs_thresh = 1e-3;

  tic
  for iS = ForceSensorStruct.SI_CD
    % Nachfolger-Segmente dieses Sensors
    INF = uint8(find(ForceSensorStruct.DNFM(iS,:)));
    
    %% Detektion
    if any( abs(F_W_S_red(:,iS)) > Fs_thresh )
      cd_FT(iS) = true;
    else
      continue
    end

    %% Isolation
    [ic_FT_ges(iC, iS), r_i_i_C_FT_ges(1:3,iS,iC)] = ...
      atlas5_wbody_collision_isolation_forcesensor_mex( rpy_base, qJ_wbody, ...
      F_W_S_red(1:6, iS), ForceSensorStruct.LI(iS), INF(:), atlas5_collbodies);
    if ic_FT_ges(iC,iS) == 0
      continue
    end
    
    r_W_W_C_FT_ges(1:3,iS,iC) = eye(3,4)*T_W_0*T_c_mdh(:,:,ic_FT_ges(iC,iS))*[r_i_i_C_FT_ges(1:3,iS,iC);1];
    
    %% Identifikation
    F_ext_W_FT_gesm(1:6,iS,iC) = F_W_S_red(1:6, iS);
  end
  t_iso_fs(ind_pkt) = toc;
  ind_pkt = ind_pkt + 1;
  %% Kraftsensoren: Fehler betrachten
  % Prüfe, ob alle Segmente richtig erkannt wurden.
  ic_FT_ges(iC, :);
  icsim_ges(iC, :);
  
  I_FT_gut = (ic_FT_ges(iC,:) ~= 0 & ~isnan(ic_FT_ges(iC,:)));
  II_FT = sort( ic_FT_ges(iC, I_FT_gut) );
  
  % Permutationsvektor, damit die erkannten Segmente aufsteigend sind.
  [~,Rf_FT] = sort(ic_FT_ges(iC,:));
  
  if length(II_FT) == length(LI_Sim) && all ( II_FT == LI_Sim )
    n_ickorr_FT( icsim_ges(iC, 1)+1 ) = n_ickorr_FT( icsim_ges(iC, 1)+1 ) + 1;
  else
    % An Segmenten ohne Kraftsensor kann nichts erkannt werden.
  end
  r_i_i_C_FT_ges(1:3,:,iC);
  r_i_i_Csim_ges(1:3,:,iC);
  
  r_W_W_C_FT_ges(1:3,:,iC);
  r_W_W_Csim_ges(1:3,:,iC);
  
  F_ext_W_sim_gesm(:,:,iC);
  F_W_S_ges;
  F_W_S_red;
  F_W_S_ges-F_W_S_red; %#ok<MNEFF>
  
  % Suche zusammengehörende Positionsfehler.
  DistMat = NaN(5,iS); % Abstand von jedem Kraftsensor-Punkt zu Sim.-Punkt
  for ii = 1:5
    for iS = 1:ForceSensorStruct.nS
      DistMat(ii,iS) = norm( r_W_W_C_FT_ges(1:3,iS,iC) - r_W_W_Csim_ges(1:3,ii,iC) );
    end
  end
  for tmp = 1:5 % Berechne für jeden simulierten Kontakt einen Fehler
    [~,IDM] = sort(DistMat(:)); % Sortiere die Abstandsmatrix mit kleinstem Abstand zuerst
    [iii,jjj] = ind2sub([5,iS], IDM(1)); % generiere den 2D-Index: Kombination Simulations-Index (iii) und Sensor-Index (jjj)
    
    er_W_W_C_FT_ges(iC, iii) = DistMat(iii,jjj); % wähle den vorher berechneten Fehler aus.
    DistMat(iii,:) = NaN; % Zu diesem simulierten Kontakt wird damit nicht noch ein Kraftsensor gefunden
    DistMat(:,jjj) = NaN; % Zu diesem Kraftsensor wurde schon ein Kontakt gefunden. Nicht noch einen weiteren.
  end
  
  % Falls keine Zuordnung erfolgen kann, steht NaN
  er_W_W_C_FT_ges(iC,:);
  %% Ergebnis zeichnen

  if debug
    % Siehe: atlas5_wbody_collision_pipeline_mc_simulink_debug_eval.m
    figure(726);clf; %#ok<*UNRCH>
    set(726,'NumberTitle', 'off');
    set(726, 'Renderer','OpenGL')
    set(726, 'Name', sprintf('MC_Rob_Koll_ErsatzK'));
    for ip = 1%1:3
%       % Roboter zeichnen
      if ip == 1
%         subplot(1,2,1);
        view(3); % vorne
%       elseif ip == 2
%         subplot(2,2,3);
%         view([60 30]);
%       else
%         subplot(2,2,4);
%         view([60 120]);
      end
      hold on; grid on;axis equal;
      % Zeichne Ersatzkörper
      for li = 1:(AS.NJ+1)
        grau1 = 0.6+mod(li-1,2)*0.15;
        atlas_plot_wbody_link_collision_body(1:(AS.NJ+1), T_c_Wmdh, atlas5_collbodies, ...
          0.4, grau1*[1 1 1], (grau1+0.2)*[1 1 1], 0.0)
      end

      % Drehachsen einzeichnen
%       for li = 1:(AS.NJ+1)
%         p_ks = T_W_0*T_c_mdh(1:4,4,li);
%         u_ks = T_W_0*T_c_mdh(1:4,3,li);
%         p1 = p_ks+0.1*u_ks;
%         p2 = p_ks-0.1*u_ks;
%         plot3([p1(1);p2(1)], [p1(2);p2(2)], [p1(3);p2(3)], 'k-', 'LineWidth', 1);
%       end

      % Kontaktortschätzung einzeichnen
      LegTxt = {};
      LegHdl = [];

      % Isolation mit Kraftsensoren
      ph=plot3(r_W_W_C_FT_ges(1,:,iC), r_W_W_C_FT_ges(2,:,iC), r_W_W_C_FT_ges(3,:,iC), ...
        'bd', 'LineWidth', 5, 'MarkerSize', 8);
      LegHdl=[LegHdl;ph]; LegTxt={LegTxt{:},'FT'}; %#ok<AGROW,CCAT>
      % Kraftwirklinie
      for ii = 1:nFS
        plot3([r_W_W_C_FT_ges(1,ii,iC);r_W_W_C_FT_ges(1,ii,iC)-0.2*F_ext_W_FT_gesm(1,ii,iC)], ...
              [r_W_W_C_FT_ges(2,ii,iC);r_W_W_C_FT_ges(2,ii,iC)-0.2*F_ext_W_FT_gesm(2,ii,iC)], ...
              [r_W_W_C_FT_ges(3,ii,iC);r_W_W_C_FT_ges(3,ii,iC)-0.2*F_ext_W_FT_gesm(3,ii,iC)], ...
              'b-', 'LineWidth', 5);
      end
        
      % Isolation mit Gelenkmoment
      ph=plot3(r_W_W_C_JT_ges(1,:,iC), r_W_W_C_JT_ges(2,:,iC), r_W_W_C_JT_ges(3,:,iC), ...
        'cx', 'LineWidth', 5, 'MarkerSize', 8);
      LegHdl=[LegHdl;ph]; LegTxt={LegTxt{:},'JT'}; %#ok<AGROW,CCAT>
      % Kraftwirklinie
      for ii = 1:5
        plot3([r_W_W_C_JT_ges(1,ii,iC);r_W_W_C_JT_ges(1,ii,iC)-0.2*F_ext_W_JT_gesm(1,ii,iC)], ...
              [r_W_W_C_JT_ges(2,ii,iC);r_W_W_C_JT_ges(2,ii,iC)-0.2*F_ext_W_JT_gesm(2,ii,iC)], ...
              [r_W_W_C_JT_ges(3,ii,iC);r_W_W_C_JT_ges(3,ii,iC)-0.2*F_ext_W_JT_gesm(3,ii,iC)], ...
              'c-', 'LineWidth', 5);
      end
      
      % Isolation mit Gelenkmoment und vorgegebenen Segmenten
      ph=plot3(r_W_W_C_JTp_ges(1,:,iC), r_W_W_C_JTp_ges(2,:,iC), r_W_W_C_JTp_ges(3,:,iC), ...
        'gv', 'LineWidth', 5, 'MarkerSize', 8);
      LegHdl=[LegHdl;ph]; LegTxt={LegTxt{:},'JTp'}; %#ok<AGROW,CCAT>
      % Kraftwirklinie
      for ii = 1:5
        plot3([r_W_W_C_JTp_ges(1,ii,iC);r_W_W_C_JTp_ges(1,ii,iC)-0.2*F_ext_W_JTp_gesm(1,ii,iC)], ...
              [r_W_W_C_JTp_ges(2,ii,iC);r_W_W_C_JTp_ges(2,ii,iC)-0.2*F_ext_W_JTp_gesm(2,ii,iC)], ...
              [r_W_W_C_JTp_ges(3,ii,iC);r_W_W_C_JTp_ges(3,ii,iC)-0.2*F_ext_W_JTp_gesm(3,ii,iC)], ...
              'g--', 'LineWidth', 3);
      end
        
      % Simulierter Kontaktpunkt
      ph=plot3(r_W_W_Csim_ges(1,:,iC), r_W_W_Csim_ges(2,:,iC), r_W_W_Csim_ges(3,:,iC), ...
        'rs', 'LineWidth', 5, 'MarkerSize', 8);
      % Kraftwirklinie
      for ii = 1:5
        plot3([r_W_W_Csim_ges(1,ii,iC);r_W_W_Csim_ges(1,ii,iC)-0.2*F_ext_W_sim_gesm(1,ii,iC)], ...
              [r_W_W_Csim_ges(2,ii,iC);r_W_W_Csim_ges(2,ii,iC)-0.2*F_ext_W_sim_gesm(2,ii,iC)], ...
              [r_W_W_Csim_ges(3,ii,iC);r_W_W_Csim_ges(3,ii,iC)-0.2*F_ext_W_sim_gesm(3,ii,iC)], ...
              'r--', 'LineWidth', 4);
      end
      LegHdl=[LegHdl;ph]; LegTxt={LegTxt{:},'Sim'}; %#ok<AGROW,CCAT>

      legend(LegHdl, LegTxt);
    end
  end
end

%% Plot-Vorbereitung
% XLabel
LNTL = cell(31,1);
for i = 1:31
  LNTL{i} = sprintf('%s (%d)', strrep(AS.LN{i}, '_', '\_'), i);
end
XTL = {'no contact', LNTL{:}}; %#ok<CCAT>


%% Fehlerdarstellung: Richtig erkannte Segmente
% 3D-Plot: xy-Kombination der Segmente. z: Größe des Fehlers
figure(100);clf;
set(100, 'Name', 'Anteil_SegKorrekt', 'NumberTitle', 'off');
for ii = 1:4
  subplot(2,2,ii);hold on
  if ii == 1
    bar(0:31,n_ic_komb);
    title('Anzahl der untersuchten Kombinationen');
  elseif ii > 1
    switch ii
      case 2
        anteil = n_ickorr_FT./n_ic_komb;
        mn='FT';
      case 3
        anteil = n_ickorr_JT./n_ic_komb;
        mn='JT';
      case 4
        anteil = n_ickorr_JTp./n_ic_komb;
        mn='JTp';
    end
    KLI_ges = (0:31);
    I_gut = (anteil >= 0.99);
    I_mittel = (anteil<0.99 & anteil > 0.5 );
    I_schlecht = ~I_gut & ~I_mittel;
    plot(KLI_ges(I_gut),anteil(I_gut),'g^');
    plot(KLI_ges(I_mittel),anteil(I_mittel),'kx');
    plot(KLI_ges(I_schlecht),anteil(I_schlecht),'rv');
    title(sprintf('Anteil der komplett richtigen Segmente mit Methode "%s"', mn));
  end
  xlabel('Segment-Nr. Koll.'); ylabel('Anzahl');
  set(gca, 'XTICK', 0:31);
  set(gca, 'XTICKLABEL', XTL);
  set(gca, 'XTICKLABELROTATION', 90);
  grid on;
end

%% Fehlerdarstellung Positionsfehler
for meth = 1:3
  figure(200+meth);clf;hold on
  % [x,y]=meshgrid(1:31,1:31);
  % Positionsfehler als Gitter erstellen
  % Maximaler Fehler für jede Kombination
  if meth == 1
    maxfehler = max(er_W_W_C_FT_ges')'; %#ok<UDIM>
    maxfehler_FT = maxfehler;
    mn = 'FT';
  elseif meth == 2
    maxfehler = max(er_W_W_C_JT_ges')'; %#ok<UDIM>
    maxfehler_JT = maxfehler;
    mn = 'JT';
  else
    maxfehler = max(er_W_W_C_JTp_ges')'; %#ok<UDIM>
    maxfehler_JTp = maxfehler;
    mn = 'JTp';
  end
  I_gut = maxfehler<1e-8;
  plot(KontaktKomb.LI5(I_gut), maxfehler(I_gut), 'g^');
  plot(KontaktKomb.LI5(~I_gut), maxfehler(~I_gut), 'rv');
  grid on;
  set(gca, 'XTick', 0:31);
  set(gca, 'XTickLabel', XTL);
  set(gca, 'XTickLabelRotation', 45);
  ylabel(sprintf('Fehler %s [m]', mn));
  xlabel('Segment-Nr. Koll.');
  title(sprintf('Positionsfehler mit Methode "%s"', mn));
  set(200+meth, 'Name', sprintf('er_Seg_%s',mn), 'NumberTitle', 'off')
end


%% Fehler verglichen mit Anzahl der Informationen

% Anzahl der Informationen zählen
nInf = zeros(NC,1);
nInf2 = zeros(NC,1); % Informationen aus Jacobi-Matrix
% LI_nf = get_children_from_parent_list(v,ForceSensorStruct.LI(iS)-1)+1]; % Nachfolger-Segmentindizes

for iC = 1:NC
  nInf(ii) = 6; % Für Basis
  QI_aktiv = false(36,1);
  QI_aktiv(1:6) = true; % Basis
  for ii = 1:5
    if icsim_ges(iC,ii) == 0
      continue
    end
    % Bestimme, das wievielte Segment in der Kette vorliegt
    switch CN(icsim_ges(iC,ii))
      case 1
        JI_chain = AS.JI_Torso;
      case 2
        JI_chain = AS.JI_lLeg;
      case 3
        JI_chain = AS.JI_rLeg;
      case 4
        JI_chain = [AS.JI_Torso,AS.JI_lArm];
      case 5
        JI_chain = [AS.JI_Torso,AS.JI_rArm];
    end
    % Liste der vorhergehenden Gelenke bis zur Basis
     % TODO: Keine Sonderbehandlung für Basis
    if icsim_ges(iC,ii) == 1
      cl = [];
    else
      % TODO: Funktion get_parents_from_parent_list sauber neu und hier
      % anpassen
      cl = [ icsim_ges(iC,ii); get_parents_from_parent_list(v, icsim_ges(iC,ii) ) ];
    end
    
    QJ_ii = (6+ (cl(1:end-1)-1) ); % Diese Freiheitsgrade werden bei diesem Kontakt angesprochen.
    QI_aktiv(QJ_ii) = true;
    if debug && icsim_ges(iC,ii) ~= 1 % TODO: Keine Sonderbehandlung für Basis
      fprintf('iC=%d,ii=%d. Kontakt an Segment %d (%s). Bewegt von Gelenk %d (%s)\n', ...
        iC, ii, icsim_ges(iC,ii), AS.LN{icsim_ges(iC,ii)}, icsim_ges(iC,ii)-1, AS.JN{icsim_ges(iC,ii)-1});
      % Gelenk i bewegt Körper i+1
      fprintf('Vorgänger-Gelenke:\n');
      disp( AS.JN( cl(1:end-1)-1 )' );
    end
  end
  nInf(iC) = sum(QI_aktiv);
  
  % Informationsgehalt mit Jacobi-Matrizen prüfen
  J_wbody_gesamt = [];
  for ii = 1:5
    if icsim_ges(iC,ii) == 0
      continue
    end
    J_wbody_ii = atlas5_wbody_body_jacobig_mdh_eulangrpy_num_mex( ...
      rpy_base, qJ_wbody, uint8(icsim_ges(iC,ii)), r_i_i_Csim_ges(:,ii,iC));
    J_wbody_gesamt = [J_wbody_gesamt; J_wbody_ii]; %#ok<AGROW>
  end
  nInf2(iC) = rank(J_wbody_gesamt);
end

%% Zeichnen: Anzahl Informationen
for meth = 2:3
  figure(300+meth);clf;
  if meth == 2
    maxfehler = max(er_W_W_C_JT_ges')'; %#ok<UDIM>
    mn = 'JT';
  else
    maxfehler = max(er_W_W_C_JTp_ges')'; %#ok<UDIM>
    mn = 'JTp';
  end
  plot(nInf2, maxfehler, 'x');
  title(sprintf('Fehler über Anzahl Informationen Methode "%s"', mn));
  ylabel('Fehler [m]');
  xlabel('Anzahl Informationen (Rang)');
end

% Zusammenhang gezählte Koordinaten und Jacobi-Matrix
figure(400);clf;
plot(nInf, nInf2, 'x');
xlabel('Anzahl der verallg. Koord.');
ylabel('Rang der Jacobi-Matrix');
grid on;
title('Zusammenhang zwischen den Informationsangaben');

% Zusammenhang Segmente und Rang Jacobi-Matrix
figure(401);clf;
hold on;
nCont = 4 + (icsim_ges(:,1) > 0);
I_gut = (nInf2==nCont*6);
plot(KontaktKomb.LI5(I_gut), nInf2(I_gut), 'g^');
plot(KontaktKomb.LI5(~I_gut), nInf2(~I_gut), 'rv');
grid on;
set(gca, 'XTick', 1:31);
set(gca, 'XTickLabel', LNTL);
set(gca, 'XTickLabelRotation', 45);
ylabel('Rang(Jges)');
xlabel('Segment-Nr. Koll.');
title('Verlauf des Ranges der Jacobi-Matrix');

%% Speichern der Ergebnisse
res_dat = fullfile(res_path, sprintf('result_manip.mat'));
save(res_dat, 'KontaktKomb', 'nInf2', 'er_W_W_C_JT_ges', 'er_W_W_C_JTp_ges', ...
  'er_W_W_C_JT_ges', 'nCont', 'maxfehler_JTp', 'maxfehler_JT');
save([res_dat,'alles.mat']); % load([res_dat,'alles.mat']);

%% Debug: Fehler für Kraftsensor-Isolation
I_falsch = isnan(maxfehler_FT) | (maxfehler_FT>1e-10);
fprintf('Fehler mit Methode FT: %d/%d\n', sum(I_falsch), NC);
if any(I_falsch)
  [~,iC] = max( maxfehler_FT );
  
end

%% Debug: Fehler für Gelenkmoment-Isolation
I_falsch = isnan(maxfehler_JT) | (maxfehler_JT>1e-10);
fprintf('Fehler mit Methode JT: %d/%d\n', sum(I_falsch), NC);
if any(I_falsch)
  [~,iC] = max( maxfehler_JT );
  
end