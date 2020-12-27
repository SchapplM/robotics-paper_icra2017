% Statistische Untersuchung von Kontakten an unterschiedlichen Segmenten
% des gesamten Atlas-Roboters
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
NP = 2; % Anzahl der gleichzeitigen Kontakte. TODO: Anpassbar

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
er_W_W_C_FT_ges = NaN(NC,5);
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
% Methoden
n_ickorr_FT = zeros(AS.NL,AS.NL);
n_ickorr_JT = zeros(AS.NL,AS.NL);
n_ickorr_JTp = zeros(AS.NL,AS.NL);
n_ic_komb = zeros(AS.NL,AS.NL); % Anzahl der Kombinationen für Kollisionen mit diesen beteiligten Segmenten

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
pktdat = fullfile(res_path, 'PunkteListe.mat');
if exist(pktdat, 'file')
  ResultStruct = load(pktdat);
  fprintf('Generierte Punkte aus Datei %s geladen.\n', pktdat);
else
  SettingsStruct = struct('IL_Liste', 1:(AS.NJ+1), 'NCL', 1000, 'qJ_wbody', qJ_wbody, ...
    'rpy_base', rpy_base, 'r_W_0', r_W_0);
  ResultStruct = atlas5wbody_collisions_statistics_gen_points(SettingsStruct);
  save(pktdat, '-struct', 'ResultStruct');
end
%% Kontakt-Kombinationen
pktkombdat = fullfile(res_path, 'PunktKombListe.mat');
if exist(pktkombdat, 'file')
  tmp = load(pktkombdat);
  n_ic_komb = tmp.n_ic_komb;
  KontaktKomb = tmp.KontaktKomb;
  fprintf('Generierte Zufallspunktkombinationen aus Datei %s geladen.\n', pktkombdat);
else
  % Matrix mit Kontaktkombinationen erstellen
  % Kontakt 1: Alle vier Ketten

  KontaktKomb = struct('LI1', NaN(NC,1), 'LI2', NaN(NC,1), ...
    'II1', NaN(NC,1), 'II2', NaN(NC,1));

  I1_erlaubt = true(length(ResultStruct.IL), 1);
  % Der erste Kontakt ist irgendwo
  J1_erlaubt = find(I1_erlaubt);
  for i = 1:NC
    % Wähle ersten Kontakt
    iitmp1 = randperm(sum(I1_erlaubt),1); % zufälliger Index in J1_erlaubt
    II1 = J1_erlaubt(iitmp1); % Index des ersten Kontakts in ResultStruct
    LI1 = ResultStruct.IL(II1);

    % Wähle zweiten Kontakt
    I2_erlaubt = true(length(ResultStruct.IL), 1);
    I2_erlaubt = I2_erlaubt & ResultStruct.IL~=LI1; % alle ausser demselben link sind erlaubt

    J2_erlaubt = find(I2_erlaubt); % erlaubte Indizes in ResultStruct

    iitmp2 = randperm(sum(I2_erlaubt),1); % zufälliger Index in J2_erlaubt
    II2 = J2_erlaubt(iitmp2); % Index des zweiten Kontakts in ResultStruct
    LI2 = ResultStruct.IL(II2);

    % Speichere die Kontakte so ab, dass der erste Kontakt am niedrigeren
    % Segment ist
    if LI1 > LI2
      KontaktKomb.II1(i) = II2;
      KontaktKomb.II2(i) = II1;
      KontaktKomb.LI1(i) = LI2;
      KontaktKomb.LI2(i) = LI1;
    else
      KontaktKomb.II1(i) = II1;
      KontaktKomb.II2(i) = II2;
      KontaktKomb.LI1(i) = LI1;
      KontaktKomb.LI2(i) = LI2;
    end

    n_ic_komb(KontaktKomb.LI1(i),KontaktKomb.LI2(i)) = n_ic_komb(KontaktKomb.LI1(i),KontaktKomb.LI2(i)) + 1;
  end
  save(pktkombdat, 'n_ic_komb', 'KontaktKomb');
end
if debug
  figure(1);clf;hold on;grid on
  xlabel('Index Körper 1');
  ylabel('Index Körper 2');
  plot(KontaktKomb.LI1, KontaktKomb.LI2, 'x');
end

t_iso_beo = NaN(1,sum(sum(n_ic_komb)));
t_iso_fs  = NaN(1,sum(sum(n_ic_komb)));
ind_pkt = 1;

%% Kontakt-Isolation für alle Zufallspunkte berechnen
for iC = 1:NC
  % Kontaktpunkte holen
  II1 = KontaktKomb.II1(iC);
  II2 = KontaktKomb.II2(iC);
  ictmp = ResultStruct.IL([II1, II2]);
%   if ictmp(1)<5
    icsim_ges(iC,1:2) = ictmp; % eventuell besser 2,3?
    r_W_W_Csim_ges(:,1:2,iC) = ResultStruct.r_W_W_C([II1, II2],:)';
    r_i_i_Csim_ges(:,1:2,iC) = ResultStruct.r_i_i_C([II1, II2],:)';
    F_ext_W_sim_gesm(:,1:2,iC) = [ResultStruct.F_c_W([II1, II2],:)';zeros(3,2)];
%   else
%     icsim_ges(iC,2:3) = ictmp;
%     r_W_W_Csim_ges(:,2:3,iC) = ResultStruct.r_W_W_C([II1, II2],:)';
%     r_i_i_Csim_ges(:,2:3,iC) = ResultStruct.r_i_i_C([II1, II2],:)';
%     F_ext_W_sim_gesm(:,2:3,iC) = [ResultStruct.F_c_W([II1, II2],:)';zeros(3,2)];
%   end
  LI_Sim = sort( icsim_ges(iC, icsim_ges(iC,:)~=0) ); % aktive Segmente im Kontakt
  
  if debug
    for ii = CN(ictmp)'
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
  F_W_S_red = forcesensor_remove_following(R_W_0, T_c_mdh_stack, F_W_S_ges, ...
    ForceSensorStruct.LI, ForceSensorStruct.NFS, ForceSensorStruct.IIo);

  %% Berechnung Isolation mit Gelenkmomenten
  % Isolation nur mit perfekten externen Gelenkmomenten 
  tic
  [ic_JT_ges(iC,:), r_i_i_C_JT_ges(:,:,iC), F_ext_W_JT_gesm(:,:,iC)] = ...
    atlas5_wbody_collision_isolation_fullchain_multi_mex(rpy_base, qJ_wbody, tauwbody_floatb_ext_sum, ...
                                                     ones(36,1)*1e-3, atlas5_collbodies);
  t_iso_beo(ind_pkt) = toc;
  % ergebnisreihenfolge korrigieren
  ic_JT_ges(iC,ic_JT_ges(iC,:)==0)=NaN;
  [ic_JT_ges(iC,:),Isort] = sort(ic_JT_ges(iC,:));
  r_i_i_C_JT_ges(:,:,iC) = r_i_i_C_JT_ges(:,Isort,iC);
  F_ext_W_JT_gesm(:,:,iC) = F_ext_W_JT_gesm(:,Isort,iC);
  ic_JT_ges(iC,isnan(ic_JT_ges(iC,:)))=0;
  
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
  
  if length(II_JT) == length(LI_Sim) && all ( II_JT == LI_Sim )
    n_ickorr_JT( LI_Sim(1), LI_Sim(2) ) = n_ickorr_JT( LI_Sim(1), LI_Sim(2) ) + 1;
  end
  n_ickorr_JTp( LI_Sim(1),LI_Sim(2)) = n_ickorr_JTp( LI_Sim(1), LI_Sim(2) ) + 1; % ist immer richtig
  
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
    [ic_FT_ges(iC, iS), r_i_i_C_FT_ges(1:3,iS,iC)] = atlas5_wbody_collision_isolation_forcesensor_mex( ...
      rpy_base, qJ_wbody, F_W_S_red(1:6, iS), ForceSensorStruct.LI(iS), INF(:), atlas5_collbodies);
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
  
  if length(II_FT) == length(LI_Sim) && all ( II_FT == LI_Sim )
    n_ickorr_FT( LI_Sim(1), LI_Sim(2) ) = n_ickorr_FT( LI_Sim(1), LI_Sim(2) ) + 1;
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
  
  % Prüfe den Positionsfehler
  for iS = 1:nFS
    if any(isnan(r_W_W_C_FT_ges(1:3,iS,iC)))
      continue
    end
    cn = CN(ic_FT_ges(iC, iS)); % Kette zu diesem Kraftsensor
    % Berechne kartesischen Fehler der zusammengehörenden Punkte
    er_W_W_C_FT_ges(iC, cn) = norm( r_W_W_C_FT_ges(1:3,iS,iC) - r_W_W_Csim_ges(1:3,cn,iC) );
  end
  
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

%% Fehlerdarstellung: Richtig erkannte Segmente
% 3D-Plot: xy-Kombination der Segmente. z: Größe des Fehlers
figure(100);clf;
[x,y]=meshgrid(1:31,1:31);

subplot(2,2,1);
mesh(x,y,n_ic_komb);
title('Anzahl der untersuchten Kombinationen');
xlabel('Segment-Nr. Koll. 1'); ylabel('Segment-Nr. Koll. 2'); zlabel('Anzahl');

subplot(2,2,2);
mesh(x,y,n_ickorr_FT./n_ic_komb);
title('Anteil der richtigen Segmente mit Methode "FT"');
xlabel('Segment-Nr. Koll. 1'); ylabel('Segment-Nr. Koll. 2'); zlabel('Anzahl');

subplot(2,2,3);
mesh(x,y,n_ickorr_JT./n_ic_komb);
title('Anteil der richtigen Segmente mit Methode "JT"');
xlabel('Segment-Nr. Koll. 1'); ylabel('Segment-Nr. Koll. 2'); zlabel('Anzahl');

subplot(2,2,4);
mesh(x,y,n_ickorr_JTp./n_ic_komb);
title('Anteil der richtigen Segmente mit Methode "JTp"');
xlabel('Segment-Nr. Koll. 1'); ylabel('Segment-Nr. Koll. 2'); zlabel('Anzahl');

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
  plot3(KontaktKomb.LI1(I_gut), KontaktKomb.LI2(I_gut), maxfehler(I_gut), 'g^');
  plot3(KontaktKomb.LI1(~I_gut), KontaktKomb.LI2(~I_gut), maxfehler(~I_gut), 'rv');
  grid on;
  set(gca, 'XTick', 1:31);
  set(gca, 'YTick', 1:31);
  YTL = cell(31,1);
  for i = 1:31
    YTL{i} = sprintf('%s (%d)', strrep(AS.LN{i}, '_', '\_'), i);
  end
  set(gca, 'XTickLabel', YTL);
  set(gca, 'XTickLabelRotation', 45);
  set(gca, 'YTickLabel', YTL);
  zlabel(sprintf('Fehler %s [m]', mn));
  xlabel('Segment-Nr. Koll. 1'); ylabel('Segment-Nr. Koll. 2');
  title(sprintf('Positionsfehler mit Methode "%s"', mn));
  set(200+meth, 'Name', sprintf('er_Seg_%s',mn))
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
  set(300+meth, 'Name', sprintf('er_rankJ_%s',mn))
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
I_gut = (nInf2==NP*6);
plot3(KontaktKomb.LI1(I_gut), KontaktKomb.LI2(I_gut), nInf2(I_gut), 'g^');
plot3(KontaktKomb.LI1(~I_gut), KontaktKomb.LI2(~I_gut), nInf2(~I_gut), 'rv');
grid on;
set(gca, 'XTick', 1:31);
set(gca, 'YTick', 1:31);
set(gca, 'XTickLabel', YTL);
set(gca, 'XTickLabelRotation', 45);
set(gca, 'YTickLabel', YTL);
zlabel('Rang(Jges)');
xlabel('Segment-Nr. Koll. 1'); ylabel('Segment-Nr. Koll. 2');
title('Verlauf des Ranges der Jacobi-Matrix');

%% Histogramm Positionsfehler über Rang

figure(600);clf;
for ir = 1:4
  rr = 12-ir;
  IRangk12 = (nInf2 == rr);
  subplot(2,2,ir);
  histogram(1e3*maxfehler_JTp(IRangk12))
  xlabel('Fehler [mm]');ylabel('Anzahl');
  title(sprintf('Rang: %d', rr));
end

%% Speichern der Ergebnisse
res_dat = fullfile(res_path, sprintf('result_%dgleichzeitig.mat', NP));
save(res_dat, 'KontaktKomb', 'nInf2', 'er_W_W_C_JT_ges', 'er_W_W_C_JTp_ges', ...
  'maxfehler_JTp', 'maxfehler_JT');
save([res_dat,'alles.mat']);

%% Debug: Fehler für Kraftsensor-Isolation
if any(isnan(maxfehler_FT)) || any(maxfehler_FT>1e-10)
  % Achtung: Es werden Fehler angezeigt, falls ein Kontakt nicht gefunden
  % wurde, für den es gar keinen Sensor gibt.
  % TODO: Ändern.
  % Hier eintragen der Segment-Kombinationen zum Finden der laufenden
  % Nummer des Fehlers (siehe Bild)
  I_such = (KontaktKomb.LI1 == 24) & (KontaktKomb.LI2 == 31);
  I_such = I_such(1:NC); % falls mehr Kombinationen gespeichert sind als berechnet wurde.
  II_such=find(I_such) %#ok<NOPTS>
  er_W_W_C_FT_ges( II_such(1), : ) %#ok<NOPTS>
  [~,JJ] = max( maxfehler_FT(II_such) ) %#ok<NOPTS>
  maxfehler_FT(II_such(JJ)) %#ok<NOPTS>
  iC = II_such(JJ) %#ok<NOPTS>
end

%% Debug: Fehler für Gelenkmoment-Isolation mit perfekten Segmentvorgaben
I_falsch= isnan(maxfehler_JTp) | (maxfehler_JTp>1e-10);
if any(I_falsch)
  fprintf('%d Berechnungen mit Gelenkmoment JTp fehlgeschlagen.\n', sum(I_falsch));
  [~,iC] = max( maxfehler_JTp );
  er_W_W_C_JTp_ges(iC,:) %#ok<NOPTS>
end