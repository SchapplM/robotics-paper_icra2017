% Statistische Untersuchung von Kontakten an unterschiedlichen Segmenten
% des gesamten Atlas-Roboters

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

clc
clear

mex_script_dependencies(mfilename('fullpath'))
rng(0); % Zurücksetzen des Zufallsgenerators, um reproduzierbare Ergebnisse zu erhalten

%% Init
NCL = 100; % Anzahl der untersuchten Kontakte pro Segment

debug = false;
debug_linkplot = false;
debug_robotplot = false;
debug_mex = false; % Zur Überprüfung, ob bei mex-Funktionen das gleiche Ergebnis herauskommt.

atlas_version = uint8(5);
AS = atlas_const(atlas_version);
E_Matrix2 = NaN((AS.NJ+1), NCL);
E_Matrix3 = NaN((AS.NJ+1), NCL);
% Roboter-Ersatzkörper laden
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
atlas5_collbodies = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'I', 'T', 'P');
% Detailinformationen. Erzeugt durch atlas_contactbody_properties.m
atlas5_collbodies_prop = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'A');

res_path = fullfile(collhdl_pfad, 'results', 'atlas5wbody_collisions_statistics_sc');
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

% Eigenschaften der Kraftsensoren laden
ForceSensorStruct = atlas5_forcesensor_settings_default();

[~, ~, ~, ~, ~, ~, v] = atlas5_wbody_parameter_mdh();

ResultStruct = struct('r_0_0_C1', [], 'r_0_0_C2', [], 'r_0_0_C3', [], ...
  'nColl', zeros((AS.NJ+1),1), 'iCB', [], ...
  'e_C2', [], 'e_C3', [], ...
  'qJ_wbody', qJ_wbody, 'T_W_0', T_W_0, ...
  'ForceSensorStruct', ForceSensorStruct);
iRS = 0; % Laufindex
%% Test: Roboter plotten
if debug
  figure(3000);clf;set(3000, 'Name', 'Rob', 'NumberTitle', 'off');
  hold on; grid on;axis equal;view(3);set(3000, 'Renderer','OpenGL')
  atlas_plot_robot(qJ_wbody', atlas_version, 3000, eye(4));
end
t_iso_beo = NaN(1,3000);
t_iso_fs  = NaN(1,3000);
ind_pkt = 1;

%% Statistik
np_L_res = zeros((AS.NJ+1),1); % Anzahl der Punkt für jedes Segment
for IL = 1:(AS.NJ+1) % Schleife über Armsegmente
  fprintf('Berechne Kontaktpunkte für Körper %d (%s)\n', IL, AS.LN{IL});
  % Transformation zu aktuellem Kontaktsegment
  T_i = T_c_mdh_wbody(:,:,IL);
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
    while iCL <= NCL_CB(iCB) && iCL_try <= 1000*NCL_CB(iCB) % Schleife Kollisionen. Abbruch falls zu viele Fehlversuch.
      iCL_try = iCL_try + 1;
      % Generiere zufällige Kontaktorte auf der Außenhülle
      Par_j = atlas5_collbodies.P(IHK,:);
      %% Zufälligen Kontaktpunkt generieren
      if atlas5_collbodies.T(IHK) == 1 % Quader
        r_i_i_C0 = box_random_surface_point_equal(Par_j(1:12));
      elseif atlas5_collbodies.T(IHK) == 2 % Zylinder
        r_i_i_C0 = cylinder_random_surface_point_equal(Par_j(1:7));
      else
        error('Nicht behandelter Körper');
      end
      r_0_0_C0 = eye(3,4) * T_i*[r_i_i_C0;1];
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
        F_c_i = T_i(1:3,1:3)' * T_W_0(1:3,1:3)' * F_c_W;
        %% Prüfe, ob Punkt auf gewünschtem Körper liegt
        % TODO: Abschattungen auch mit einschließen und nicht verwerfen.
        % Berechne Schnittpunkte aller Kollisionskörper mit Punkt als
        % Aufpunkt und Kraftvektor als Richtungsvektor einer Geraden
        S = atlas5_wbody_intersect_collbodies_mex(r_i_i_C0, F_c_i, uint8(IL), T_c_mdh_stack, ...
          true(length(atlas5_collbodies.I),1), atlas5_collbodies); % nur Ersatzkörper dieses Segments: (atlas5_collbodies.I==IL)
        if debug_mex
          S_nomex = atlas5_wbody_intersect_collbodies(r_i_i_C0, F_c_i, uint8(IL), T_c_mdh_stack, ...
            true(length(atlas5_collbodies.I),1), atlas5_collbodies);
          if any( abs( S(:) - S_nomex(:) ) > 1e-6)
            error('mex stimmt nicht');
          end
        end
        % Indizes der Ersatzkörper, die von der Geraden geschnitten werden.
        I_Schnitt = ~isnan(S(:,4));
        if isempty(I_Schnitt)
          if debug
            warning('%d.[%d/%d].%d Die Kraftwirklinie schneidet keinen der Ersatzkörper von Segment %d', ...
              IL, (iCB-1)*NCB+iCL, NCL*NCB, it, IL); %#ok<UNRCH>
          end
          IHK_sim = NaN;
          continue
        end
        % suche den kleinsten Parameter lambda der Geraden. Dieser entspricht der
        % äußersten Druckkraft
        [~,iitmp] = min(S(:,4));
        IHK_sim = S(iitmp, 5);
        r_i_i_C1 = S(iitmp, 1:3)';
        r_0_0_C1 = eye(3,4)*T_c_mdh_wbody(:,:,IL)*[r_i_i_C1;1];
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
        ep = norm(r_0_0_C1-r_0_0_C0);
        if ep > 1e-6
          if debug
            warning('%d.[%d/%d].%d Punkt [%s] generiert, bestimmter Punkt [%s] weicht zu stark ab (e=%1.7f).', ...
              IL, (iCB-1)*NCB+iCL, NCL*NCB, it, disp_array(r_0_0_C0', '%1.3f'), disp_array(r_0_0_C1', '%1.3f'), ep); %#ok<UNRCH>
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

      %% Berechnung des Gelenkmoments

      J_wbody1 = atlas5_wbody_body_jacobig_mdh_eulangrpy_num_mex(rpy_base, qJ_wbody, uint8(IL), r_i_i_C1);
      tauwbody_floatb_ext1 = J_wbody1' * [F_c_W; zeros(3,1)];
      
      %% Kontaktortschätzung mit externem Moment
      tic
      [ic2,r_i_i_C2,~,~] = atlas5_wbody_collision_isolation_fullchain_mex( ...
        rpy_base, qJ_wbody, tauwbody_floatb_ext1, ...
        ones(30,1)*eps, atlas5_collbodies, true(36,1));
      t_iso_beo(ind_pkt) = toc;
      if debug_mex
        [ic_nomex,r_i_i_C2_nomex,~,~] = atlas5_wbody_collision_isolation_fullchain( ...
          rpy_base, qJ_wbody, tauwbody_floatb_ext1, ...
          ones(30,1)*eps, atlas5_collbodies, true(36,1));
        if any(abs(r_i_i_C2-r_i_i_C2_nomex) > 1e-6)
          error('mex-Funktion stimmt nicht');
        end
      end
      if ic2 == 0
        % Keine Schätzung möglich.
        continue;
      end
      r_0_0_C2 = eye(3,4)*T_c_mdh_wbody(:,:,ic2)*[r_i_i_C2;1];
      
      
      %% Kräfte in den Sensoren berechnen
      % Siehe atlas5_link_contact_force_Hunt_Crossley
      NFS = length(ForceSensorStruct.LI);
      F_W_S_ges = NaN(6, NFS);
      for iS = 1:NFS
        % Suche nachfolgende Elemente für den jeweiligen Kraftsensor
        LI_nf = [ForceSensorStruct.LI(iS); get_children_from_parent_list(v,ForceSensorStruct.LI(iS)-1)+1]; % Nachfolger-Segmentindizes
        if ~any(IL == LI_nf)
          continue % Das aktuelle Kollisionssegment ist nicht nachfolger des Sensors
        end
        
        % Transformationsmatrix zum Sensor-Körper-KS
        T_0_S = T_c_mdh_wbody(:,:,ForceSensorStruct.LI(iS));
        r_0_0_S = T_0_S(1:3,4); % Sensor liegt im Ursprung des Körper-KS
        r_0_S_C = -r_0_0_S + r_0_0_C1;
        r_W_S_C = R_W_0 * r_0_S_C;
        F_W_S_ges(1:3,iS) = F_c_W;
        F_W_S_ges(4:6,iS) = cross(r_W_S_C, F_c_W);
      end
      
      %% Kontaktortschätzung mit Kraftsensor
      ic3 = NaN;
      r_0_0_C3 = NaN(3,1);
      % Führe die Kontaktschätzung für alle Kraftsensoren aus. Das Ergebnis
      % des letzten Sensors überschreibt die Ergebnisse der vorherigen,
      % falls mehrere Sensoren die Kraft messen.
      % TODO: Bewusste Auswahl des Sensors, falls mehrere messen.
      tic
      for iS = 1:NFS
        if any(isnan(F_W_S_ges(:,iS)))
          continue
        end
        % Nachfolger dieses Sensors
        INF = uint8(find(ForceSensorStruct.NFM(iS,:)));
        [ic3, r_i_i_C3] = atlas5_wbody_collision_isolation_forcesensor_mex( ...
          rpy_base, qJ_wbody, F_W_S_ges(:,iS), ForceSensorStruct.LI(iS), INF(:), atlas5_collbodies);
      end
      t_iso_fs(ind_pkt) = toc;
      if ~isnan(ic3)
        r_0_0_C3 = eye(3,4)*T_c_mdh_wbody(:,:,ic3)*[r_i_i_C3;1];
      end
      ind_pkt = ind_pkt + 1;
      
      %% Test: Hüllkörper
      if debug_linkplot
        figure(IL);
        if EMi == 1,
          % Hüllkörper neu zeichnen
          clf; 
          hold on; grid on;axis equal;view(3);set(IL, 'Renderer','OpenGL');
          set(IL, 'Name', sprintf('L%02d', IL), 'NumberTitle', 'off');
          % Hüllkörper dieses Segmentes zeichnen
          atlas_plot_wbody_link_collision_body(IL, T_c_mdh_wbody, atlas5_collbodies, ...
            0.4, 0.6*[0 0 1], (0.6+0.2)*[1 1 1]);
          % Hüllkörper anderer Segmente zeichnen (zur Einordnung)
          for iil = find(1:(AS.NJ+1) ~= IL)
            grau1 = 0.6+mod(iil-1,2)*0.15;
            atlas_plot_wbody_link_collision_body(iil, T_c_mdh_wbody, atlas5_collbodies, ...
              0.6, grau1*[1 1 1], (grau1+0.2)*[1 1 1]);
          end        

          title(sprintf('Hüllkörper Segment %d', IL));
        end
        hdl = NaN(6,1);
        hdl(1) = plot3(r_0_0_C0(1), r_0_0_C0(2), r_0_0_C0(3), 'rx');
        % Kreuz für Punkt
        % plot3(p_0(1)+[-0.1 +0.1], p_0(2)*[1;1], p_0(3)*[1;1], 'k-');
        % plot3(p_0(1)*[1;1], p_0(2)+[-0.1 +0.1], p_0(3)*[1;1], 'k-');
        % plot3(p_0(1)*[1;1], p_0(2)*[1;1], p_0(3)+[-0.1 +0.1], 'k-');  

        % Erkannter Punkt
        hdl(2) = plot3(r_0_0_C1(1), r_0_0_C1(2), r_0_0_C1(3), 'ro');
        % Verbindung dazwischen
        hdl(3) = plot3([r_0_0_C0(1);r_0_0_C1(1)], [r_0_0_C0(2);r_0_0_C1(2)], [r_0_0_C0(3);r_0_0_C1(3)], 'r-');
        
        % Schätzungen der Punkte
        hdl(4)=plot3(r_0_0_C2(1), r_0_0_C2(2), r_0_0_C2(3), 'cd');
        hdl(5)=plot3(r_0_0_C3(1), r_0_0_C3(2), r_0_0_C3(3), 'gs');
        hdl(6)=plot3([r_0_0_C1(1);r_0_0_C2(1)], [r_0_0_C1(2);r_0_0_C2(2)], [r_0_0_C1(3);r_0_0_C2(3)], 'c-');
        hdl(7)=plot3([r_0_0_C1(1);r_0_0_C3(1)], [r_0_0_C1(2);r_0_0_C3(2)], [r_0_0_C1(3);r_0_0_C3(3)], 'g-');
        
        if EMi == 1
          legend(hdl, {'Kontaktpunkt It. 1', 'Kontaktpunkt It. 2', 'It1.-It2', ...
            'IsolationJT', 'IsolationFS', 'Diff-JT', 'Diff-FS'});
        end
        % Kraftwirklinie (als Strich einzeichnen)
        plot3([r_0_0_C1(1);r_0_0_C1(1)-0.2*F_c_W(1)], ...
              [r_0_0_C1(2);r_0_0_C1(2)-0.2*F_c_W(2)], ...
              [r_0_0_C1(3);r_0_0_C1(3)-0.2*F_c_W(3)], 'k-');
      end
      
      %% Bild für Segment (alle Ersatzkörper)
      if debug_robotplot
        figure(200);hold on
        set(200, 'Name', 'Gesamt', 'NumberTitle', 'off');
        if EMi == 1 && IL == 1
          clf; 
          hold on; grid on;axis equal;view(3);set(200, 'Renderer','OpenGL');
          for iil = 1:(AS.NJ+1)
            grau1 = 0.4+mod(iil-1,2)*0.25;
            atlas_plot_wbody_link_collision_body(iil, T_c_mdh_wbody, atlas5_collbodies, ...
              0.4, grau1*[0 0 1], (grau1+0.2)*[1 1 1], 0.0);
          end
        end
        hdl1=plot3(r_0_0_C1(1), r_0_0_C1(2), r_0_0_C1(3), 'rx');
        hdl2=plot3(r_0_0_C2(1), r_0_0_C2(2), r_0_0_C2(3), 'cd');
        hdl3=plot3(r_0_0_C3(1), r_0_0_C3(2), r_0_0_C3(3), 'gs');
        hdl4=plot3([r_0_0_C1(1);r_0_0_C2(1)], [r_0_0_C1(2);r_0_0_C2(2)], [r_0_0_C1(3);r_0_0_C2(3)], 'c-');
        hdl5=plot3([r_0_0_C1(1);r_0_0_C3(1)], [r_0_0_C1(2);r_0_0_C3(2)], [r_0_0_C1(3);r_0_0_C3(3)], 'g-');
        if EMi == 1 && IL == 1
          legend([hdl1, hdl2, hdl3, hdl4, hdl5], {'Sim', 'IsolationJT', 'IsolationFS', 'Diff-JT', 'Diff-FS'});
        end
      end
      %% Vergleich mit eingegebenen Daten
      e2 = r_0_0_C1 - r_0_0_C2;
      E_Matrix2(IL, EMi) = norm(e2);
      
      e3 = r_0_0_C1 - r_0_0_C3;
      E_Matrix3(IL, EMi) = norm(e3);

      %% Ergebnisse speichern
      iRS = iRS + 1;
      ResultStruct.r_0_0_C1(iRS,:) = r_0_0_C1;
      ResultStruct.r_0_0_C2(iRS,:) = r_0_0_C2;
      ResultStruct.r_0_0_C3(iRS,:) = r_0_0_C3;
      
      ResultStruct.e_C2(iRS,:) = e2;
      ResultStruct.e_C3(iRS,:) = e3;
      
      ResultStruct.iCB(iRS,1) = IL;
      
      iCL = iCL + 1; % Iteration der while-Schleife
    end % while iCL
    if debug
      fprintf('Segment %d. Ersatzkörper %d. iCL=%d, iCL_try=%d\n', IL, iCB, iCL, iCL_try);
    end
    if iCL_try == 1000*NCL_CB(iCB)
      warning('Grenze für zufällige Punkte überschritten (%d Versuche für Segment Nr. %d, Ersatzkörper Nr. %d)\n', ...
        iCL_try, IL, iCB);
    end
  end % iCB
end % IL
ResultStruct.nColl = np_L_res;

%% Plotten
figure(100);clf;hold on
set(100, 'Name', 'Statistics', 'Numbertitle', 'off');
for i = 1:(AS.NJ+1)
  hdl1=plot(i, E_Matrix2(i,:),  'cd');
  hdl2=plot(i, E_Matrix3(i,:),  'gs');
end
legend([hdl1(1), hdl2(1)], {'\Delta r_0_0_C2', '\Delta r_0_0_C3'});
% Mittelwerte
plot(1:(AS.NJ+1), mean(E_Matrix2, 2), 'c--');
plot(1:(AS.NJ+1), mean(E_Matrix3, 2), 'g--' );

% Min-Max
mm2 = minmax2(E_Matrix2);
plot(1:(AS.NJ+1), mm2(:,1), 'c.-');
plot(1:(AS.NJ+1), mm2(:,2), 'c.-');
mm3 = minmax2(E_Matrix3);
plot(1:(AS.NJ+1), mm3(:,1), 'g.-');
plot(1:(AS.NJ+1), mm3(:,2), 'g.-');
%% Ergebnis speichern
save(fullfile(res_path, 'result.mat'), '-struct', 'ResultStruct');

%% Bilder Speichern
if debug
  for i = find( ResultStruct.nColl ~= 0)'
    saveas(i, fullfile(res_path, sprintf('Link_%02d.fig', i)));
    saveas(i, fullfile(res_path, sprintf('Link_%02d.png', i)));
  end

  saveas(100, fullfile(res_path, sprintf('Statistik.fig')));
  saveas(100, fullfile(res_path, sprintf('Statistik.png')));
  
  saveas(200, fullfile(res_path, sprintf('Gesamt.fig')));
  saveas(200, fullfile(res_path, sprintf('Gesamt.png')));

end