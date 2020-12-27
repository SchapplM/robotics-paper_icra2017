% Generiere zulässige Punkte für die Auswertung der Kontakterkennung

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

function ResultStruct = atlas5wbody_collisions_statistics_gen_points(SettingsStruct)

debug = false;
IL_Liste = SettingsStruct.IL_Liste;
NCL = SettingsStruct.NCL;
SettingsStruct.qJ_wbody;
rpy_base = SettingsStruct.rpy_base;
r_W_0 = SettingsStruct.r_W_0;
qJ_wbody = SettingsStruct.qJ_wbody;
%% Init

atlas_version = uint8(5);
AS = atlas_const(atlas_version);

% Roboter-Ersatzkörper laden
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
atlas5_collbodies = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'I', 'T', 'P');
% Detailinformationen. Erzeugt durch atlas_contactbody_properties.m
atlas5_collbodies_prop = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'A');

R_W_0 = eulxyz2r([rpy_base(1); rpy_base(2); rpy_base(3)]);
T_W_0 = [R_W_0, r_W_0; [0 0 0 1]];

% Kinematik
T_c_mdh_wbody = atlas5_wbody_fkine_mdh_num(qJ_wbody');
T_c_mdh_stack = NaN((AS.NJ+1),16); % Zur Übergabe an Funktionen
for i = 1:(AS.NJ+1)
  T_c_mdh_stack(i,:) = reshape(T_c_mdh_wbody(:,:,i),1,16);
end

% Nehme jeden Ersatzkörper zu jedem Robotersegment
IKollK = cell((AS.NJ+1),1); % Index, welcher Kollisionskörper für jedes Segment genommen werden soll
for i = 1:(AS.NJ+1) % length(atlas5_collbodies.I)
  IKollK{i} = find(atlas5_collbodies.I == i);
end

iRS = 0; % Laufindex

ResultStruct = struct('r_W_W_C', [], 'r_i_i_C', [], ...
  'nColl', zeros((AS.NJ+1),1), 'iCB', [], 'IL', [], 'F_c_W', []);

%% Statistik
np_L_res = zeros((AS.NJ+1),1); % Anzahl der Punkt für jedes Segment
for IL = IL_Liste(:)' % Schleife über Armsegmente
  fprintf('Berechne Kontaktpunkte für Körper %d (%s)\n', IL, AS.LN{IL});
  % Transformation zu aktuellem Kontaktsegment
  T_0_i = T_c_mdh_wbody(:,:,IL);
  KollK_IL = IKollK{IL};
  EMi = 0; % Spalten-Index in der Fehler-Matrix
  NCB = length(KollK_IL); % Anzahl der Kollisionskörper zu diesem Segment
  if NCB == 0
    continue % Keine Ersatzkörper
  end
  % Verteile die Kollisionen zu diesem Robotersegment auf die Ersatzkörper
  % gewichtet mit ihrer Oberfläche
  ACB_IL = sum( atlas5_collbodies_prop.A(KollK_IL) ); % Oberfläche aller Kollisionskörper zu diesem Körper
  NCL_CB = round( atlas5_collbodies_prop.A(KollK_IL)/ACB_IL * NCL );
  if sum(NCL_CB) < NCL 
    % Verteile zuerst Punkte an Ersatzkörper die noch gar keinen
    % Kollisionspunkt haben
    for ii = 1:length(NCL_CB)
      NCL_rest = NCL-sum(NCL_CB);
      if NCL_rest == 0
        break;
      end
      if NCL_CB(ii) == 0
        NCL_CB(ii) = 1;
      end
    end
    % Durch Rundung fehlenden Punkt dem größten zuschlagen 
    [~,iiimax] = max( atlas5_collbodies_prop.A(KollK_IL) );
    NCL_CB(iiimax) = NCL_CB(iiimax) + NCL-sum(NCL_CB);
  end
  for iCB = 1:NCB % Schleife über Kollisionskörper
    IHK = KollK_IL(iCB);
    iCL = 1; % Schleifenindex über Kollisionen
    iCL_try = 1; % Anzahl der Versuche für diesen Kollisionskörper (falls Schnittpunkt falsch ist wird wiederholt)
    while iCL < NCL_CB(iCB) && iCL_try <= 1000*NCL_CB(iCB) % Schleife Kollisionen. Abbruch falls zu viele Fehlversuch.
      iCL_try = iCL_try + 1;
      % Generiere zufällige Kontaktorte auf der Außenhülle
      Par_j = atlas5_collbodies.P(IHK,:);
      %% Zufälligen Kontaktpunkt generieren
      if atlas5_collbodies.T(IHK) == 1 % Quader
        r_i_i_C0 = box_random_surface_point_equal_mex(Par_j(1:12));
      elseif atlas5_collbodies.T(IHK) == 2 % Zylinder
        r_i_i_C0 = cylinder_random_surface_point_equal_mex(Par_j(1:7));
      else
        error('Nicht behandelter Körper');
      end
      r_W_W_C0 = eye(3,4) * T_W_0 * T_0_i*[r_i_i_C0;1];
      skip = false;
      for it = 1:2 % Zwei Iterationen für die Kraft
        %% Kontaktkraft
        if it == 1
          F_c_W = rand(3,1)-0.5; % zufällige Kraft mit Mittelwert Null
          % normieren: Betrag 1
          F_c_W = F_c_W / norm(F_c_W);
        elseif it == 2
          F_c_W = -F_c_W; % Versuche anderes Vorzeichen
        end
        F_c_i = T_0_i(1:3,1:3)' * T_W_0(1:3,1:3)' * F_c_W;
        %% Prüfe, ob Punkt auf gewünschtem Körper liegt
        % TODO: Abschattungen auch mit einschließen und nicht verwerfen.
        % Berechne Schnittpunkte aller Kollisionskörper mit Punkt als
        % Aufpunkt und Kraftvektor als Richtungsvektor einer Geraden
        S = atlas5_wbody_intersect_collbodies_mex(r_i_i_C0, F_c_i, uint8(IL), T_c_mdh_stack, ...
          true(length(atlas5_collbodies.I),1), atlas5_collbodies); % nur Ersatzkörper dieses Segments: (atlas5_collbodies.I==IL)
        % Indizes der Ersatzkörper, die von der Geraden geschnitten werden.
        I_Schnitt = ~isnan(S(:,4));
        if isempty(I_Schnitt)
          if debug
            warning('%d.[%d/%d].%d Die Kraftwirklinie schneidet keinen der Ersatzkörper von Segment %d', ...
              IL, (iCB-1)*NCB+iCL, NCL*NCB, it, IL); %#ok<UNRCH>
          end
          continue
        end
        % suche den kleinsten Parameter lambda der Geraden. Dieser entspricht der
        % äußersten Druckkraft
        [~,iitmp] = min(S(:,4));
        IHK_sim = S(iitmp, 5);
        r_i_i_C1 = S(iitmp, 1:3)';
        r_W_W_C1 = eye(3,4)*T_W_0*T_c_mdh_wbody(:,:,IL)*[r_i_i_C1;1];
        if IHK_sim ~= IHK
          if debug
            warning('%d.[%d/%d].%d Statt Ersatzkörper %d (zu Segment %d) wurde Ersatzkörper %d (zu Segment %d) geschnitten', ...
              IL, (iCB-1)*NCB+iCL, NCL*NCB, it, IHK, atlas5_collbodies.I(IHK), IHK_sim, atlas5_collbodies.I(IHK_sim)); %#ok<UNRCH>
          end
          % Der falsche Ersatzkörper wird geschnitten
          if it == 1
            continue % Versuche Kraft anders herum
          else
            skip = true;
            break;
          end
        end
        %% Prüfe, ob Punkt auf der richtigen Seiten des Körpers liegt
        ep = norm(r_W_W_C1-r_W_W_C0);
        if ep > 1e-6
          if debug
            warning('%d.[%d/%d].%d Punkt [%s] generiert, bestimmter Punkt [%s] weicht zu stark ab (e=%1.7f).', ...
              IL, (iCB-1)*NCB+iCL, NCL*NCB, it, disp_array(r_W_W_C0', '%1.3f'), disp_array(r_W_W_C1', '%1.3f'), ep); %#ok<UNRCH>
          end
          continue
        end
        
      end
      
      if skip
        continue % Kombination Kraftlinie/Punkt verwerfen
      end
      % Punkt zählt
      np_L_res(IL) = np_L_res(IL) + 1;
      EMi = EMi + 1;

      %% Ergebnisse speichern
      iRS = iRS + 1;
      ResultStruct.r_W_W_C(iRS,1:3) = r_W_W_C1;
      ResultStruct.r_i_i_C(iRS,1:3) = r_i_i_C1;
      ResultStruct.iCB(iRS,1) = iCB;
      ResultStruct.IL(iRS,1) = IL;
      ResultStruct.F_c_W(iRS,1:3) = F_c_W;
      
      iCL = iCL + 1; % Iteration der while-Schleife
    end % while iCL
    if debug
      fprintf('Segment %d. Ersatzkörper %d. iCL=%d, iCL_try=%d\n', IL, iCB, iCL, iCL_try); %#ok<UNRCH>
    end
    if iCL_try == 1000*NCL_CB(iCB)
      warning('Grenze für zufällige Punkte überschritten (%d Versuche für Segment Nr. %d, Ersatzkörper Nr. %d)\n', ...
        iCL_try, IL, iCB);
    end
  end % iCB
end % IL
ResultStruct.nColl = np_L_res;
