% Statistische Untersuchung von Kontakten an unterschiedlichen Segmenten
% des gesamten Atlas-Roboters

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-08
% (c) Institut für Regelungstechnik, Universität Hannover

clc
clear

mex_script_dependencies(mfilename('fullpath'))
rng(0); % Zurücksetzen des Zufallsgenerators, um reproduzierbare Ergebnisse zu erhalten

%% Init
NCL = 10; % Anzahl der untersuchten Kontakte pro Segment

debug = false;
lr = true;
atlas_version = uint8(5);
AS = atlas_const(atlas_version, lr);
E_Matrix2 = NaN((AS.NJ+1), NCL);
E_Matrix3 = NaN((AS.NJ+1), NCL);
% Roboter-Ersatzkörper laden
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
atlas5_collbodies = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'I', 'T', 'P');
% Detailinformationen. Erzeugt durch atlas_contactbody_properties.m
atlas5_collbodies_prop = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'A');

res_path = fullfile(collhdl_pfad, 'results', 'atlas5wbody_collisions_statistics_sc_filter');
mkdirs(res_path);

qJ_wbody = zeros(AS.NJ,1);
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

% Kraftverlauf: hochfrequenter Sinus (T = 40ms, halbe Periode)
K_O = 500;
T = 0.001;
T_kontakt = 0.02; %
T_filter = 0.03;
n_zs_k = length(0:T:T_kontakt); % Anzahl Zeitschritte, des Kontakts
n_zs_f = length(0:T:T_filter); % Anzahl Zeitschritte, die der Filter laeuft
% TODO: Anderes Zeichen. Es handelt sich nicht um ein Gelenkmoment sondern
% um die Amplitude des Filtes
tau_t = zeros(n_zs_f+1,1);
tau_t(1:n_zs_k) = sin((0:T:T_kontakt)*pi/T_kontakt);
tau_filt_t = NaN(n_zs_f,1);
tau_filt_t(1) = 0;
tau_err_t = tau_filt_t;
tau_err_t(1) = -1/K_O*tau_t(1);
for i=1:n_zs_f
  tau_filt_t(i+1) = exp(-T*K_O)*tau_filt_t(i)+(1-exp(-T*K_O))*tau_t(i); % Moment folgt echtem Moment mit PT1 Charakteristik
  tau_err_t(i+1) = exp(-T*K_O)*tau_err_t(i)+1/K_O*tau_t(i+1); % Beschleunigungsfehler folgt echtem Moment mit DT1 Charakteristik
end

% Nehme jeden Ersatzkörper zu jedem Robotersegment
IKollK = cell((AS.NJ+1),1); % Index, welcher Kollisionskörper für jedes Segment genommen werden soll
for i = 1:(AS.NJ+1) % length(atlas5_collbodies.I)
  IKollK{i} = find(atlas5_collbodies.I == i);
end

% Eigenschaften der Kraftsensoren laden
ForceSensorStruct = atlas5_forcesensor_settings_default();

[a_mdh, d_mdh, alpha_mdh, q_offset_mdh, b_mdh, beta_mdh, v, rSges_mdh_wbody, m_wbody, Iges_mdh_wbody] = atlas5_wbody_parameter_mdh();

ResultStruct = struct('r_0_0_C1', [], 'r_i_i_C1', [], 'r_0_0_C2', [],...
  'r_i_i_C2', [], 'r_0_0_C3', [], 'nColl', zeros((AS.NJ+1),1), 'iCB', [], ...
  'e_C2', [], 'e_C3', [], 'F_c', [], 'F_c_erg', [], 'tau_wbody', [], ...
  'qJ_wbody', qJ_wbody, 'T_W_0', T_W_0, 't', 0:T:T_filter,...
  'tau_t', tau_t, 'tau_filt_t', tau_filt_t);
iRS = 0; % Laufindex
%% Test: Roboter plotten
if debug
  figure(3000);clf;set(3000, 'Name', 'Rob', 'NumberTitle', 'off'); %#ok<UNRCH>
  hold on; grid on;axis equal;view(3);set(3000, 'Renderer','OpenGL')
  atlas_plot_robot(qJ_wbody', atlas_version, 3000, eye(4));
end

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
        r_i_i_C0 = box_random_surface_point_equal(Par_j);
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
        % Indizes der Ersatzkörper, die von der Geraden geschnitten werden.
        I_Schnitt = ~isnan(S(:,4));
        if isempty(I_Schnitt)
          if debug
            warning('%d.[%d/%d].%d Die Kraftwirklinie schneidet keinen der Ersatzkörper von Segment %d', ...
              IL, (iCB-1)*NCB+iCL, NCL*NCB, it, ic); %#ok<UNRCH>
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
      
      %% Kräfte in den Sensoren berechnen
      % Siehe atlas5_link_contact_force_Hunt_Crossley
      NFS = length(ForceSensorStruct.LI);
      F_W_S_ges = NaN(n_zs_f, 6, NFS);
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
        F_W_S_ges(1:n_zs_f,1:3,iS) = repmat(F_c_W',n_zs_f,1);
        F_W_S_ges(1:n_zs_f,4:6,iS) = repmat(cross(r_W_S_C, F_c_W)',n_zs_f,1);
      end
      
      %% Dynamikanteil der Kräfte (nur für rechte Hand)
      if iRS==290
        t
      end
      if IL==AS.LI_rArm(end)
        for iS = 1:NFS
          if ForceSensorStruct.LI(iS)==IL
            for t=1:n_zs_f
              m_hand = m_wbody(IL);
              R_W_i=T_c_mdh_wbody(1:3,1:3,IL);
              rSD_hand = rSges_mdh_wbody(IL,:)';
              M = atlas5_wbody_inertia_floatb_eulangrpy_slag_vp1(qJ_wbody, rpy_base, alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_wbody, rSges_mdh_wbody, Iges_mdh_wbody);
              J_D = atlas5_wbody_body_jacobig_mdh_eulangrpy_num_mex(rpy_base, qJ_wbody, uint8(IL), rSD_hand);
              I_hand = inertiavector2matrix( Iges_mdh_wbody(IL,:) );
              Mat_cmp=[m_hand*eye(3) zeros(3); m_hand*skew(R_W_i*rSD_hand) R_W_i*I_hand*R_W_i'];
              ResultStruct.F_W_S_ges(t,:,iRS+1)=F_W_S_ges(t,:,iS)*tau_t(t)-(Mat_cmp*J_D*inv(M)*tau_err_t(t)*tauwbody_floatb_ext1)';
            end
          end
        end
      end
      
      iRS = iRS + 1;
      ResultStruct.r_0_0_C1(iRS,:) = r_0_0_C1;
      ResultStruct.r_i_i_C1(iRS,:) = r_i_i_C1;
      ResultStruct.F_c_W(iRS,:) = F_c_W;
      
      for t=1:n_zs_f
        
        %% Kontaktortschätzung mit Kraftsensor
        ic3 = NaN;
        r_0_0_C3 = NaN(3,1);
        for iS = 1:NFS
          if any(isnan(F_W_S_ges(t,:,iS)))
            continue
          end
          % Nachfolger dieses Sensors
          INF = uint8(find(ForceSensorStruct.NFM(iS,:)));
          [ic3, r_i_i_C3] = atlas5_wbody_collision_isolation_forcesensor_mex( ...
            rpy_base, qJ_wbody, F_W_S_ges(t,:,iS)', ForceSensorStruct.LI(iS), INF(:), atlas5_collbodies);
        end
        if ~isnan(ic3)
          r_0_0_C3 = eye(3,4)*T_c_mdh_wbody(:,:,ic3)*[r_i_i_C3;1];
        end



        %% Vergleich mit eingegebenen Daten
        e3 = r_0_0_C1 - r_0_0_C3;
        if norm(e3)>1e-10
          e3 %#ok<NOPTS>
        end
        E_Matrix3(IL, EMi) = norm(e3);

        %% Ergebnisse speichern (1)
        ResultStruct.r_0_0_C3(t,:,iRS) = r_0_0_C3;
        ResultStruct.e_C3(t,:,iRS) = e3;
        ResultStruct.iCB(iRS,t) = IL;

  %       ResultStruct.e_C2 = ;
        %% Kontaktortschätzung mit externem Moment
        [ic, r_i_i_C2, r_i_i_d, u_i] = atlas5_wbody_collision_isolation_fullchain_mex(rpy_base, qJ_wbody, tauwbody_floatb_ext1*tau_filt_t(t), ...
          ones(AS.NJ,1)*1e-3, atlas5_collbodies, true(36,1));
        if ic == 0
          % Keine Schätzung möglich.
          ResultStruct.r_0_0_C2(t,1:3,iRS) = NaN;
          ResultStruct.r_i_i_C2(t,1:3,iRS) = NaN;
          ResultStruct.e_C2(t,1:3,iRS) = NaN;
          continue;
        end
        r_0_0_C2 = eye(3,4)*T_c_mdh_wbody(:,:,ic)*[r_i_i_C2;1];

        %% Vergleich mit eingegebenen Daten
        e2 = r_0_0_C1 - r_0_0_C2;
        if norm(e2)>1e-10
          e2 %#ok<NOPTS>
        end
        

        %% Ergebnisse speichern (2)
        ResultStruct.r_0_0_C2(t,:,iRS) = r_0_0_C2;
        ResultStruct.r_i_i_C2(t,:,iRS) = r_i_i_C2;
        ResultStruct.e_C2(t,:,iRS) = e2;
        
      end
      % TODO: Vergleichbare Fehlerberechnung
      E_Matrix2(IL, EMi) = max(max( ResultStruct.e_C2(:,1:3,iRS) ));
      iCL = iCL + 1; % Iteration der while-Schleife
    end % while iCL
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
plot(1:(AS.NJ+1), mean(E_Matrix2, 2), 'c--' );
plot(1:(AS.NJ+1), mean(E_Matrix3, 2), 'g--' );

% Min-Max
mm = minmax2(E_Matrix2);
plot(1:(AS.NJ+1), mm(:,1), 'c.-');
plot(1:(AS.NJ+1), mm(:,2), 'c.-' );
mm3 = minmax2(E_Matrix3);
plot(1:(AS.NJ+1), mm3(:,1), 'g.-');
plot(1:(AS.NJ+1), mm3(:,2), 'g.-');

% Zeitverlauf Kontaktpunkt
ResultStruct.F_c_erg = NaN(size(ResultStruct.F_c_W));
for i=1:iRS
  for t=1:n_zs_f
    if any(isnan( ResultStruct.r_i_i_C2(t,:,i) ))
      % Keine Kollision gefunden
      continue
    end
    if t>3 && any(abs(ResultStruct.r_0_0_C2(t,:,i)-ResultStruct.r_0_0_C2(3,:,i))>1e-10)
      warning('Abweichung in Isolation: %e, iRS: %i, t: %i',norm(ResultStruct.r_0_0_C2(t,:,i)-ResultStruct.r_0_0_C2(3,:,i)), i, t)
    end
    J_wbody = atlas5_wbody_body_jacobig_mdh_eulangrpy_num_mex(rpy_base, qJ_wbody, uint8(ResultStruct.iCB(i,1)), ResultStruct.r_i_i_C1(i,:)');
    tauwbody = J_wbody' * [ResultStruct.F_c_W(i,:)'*tau_filt_t(t); zeros(3,1)];
ResultStruct.tau_wbody(iRS,:) = tauwbody;
    J = atlas5_wbody_body_jacobig_mdh_eulangrpy_num_mex(rpy_base, qJ_wbody, uint8(ResultStruct.iCB(i,1)), ResultStruct.r_i_i_C2(t,:,i)');
    ResultStruct.F_c_W_erg(t,:,i) = pinv(J.')*tauwbody;
    if t>3 && t<=n_zs_k && any(abs([ResultStruct.F_c_W(i,:) 0 0 0]-ResultStruct.F_c_W_erg(t,:,i)/tau_filt_t(t))>1e-10)
      warning('Abweichung in Kraft: %e, iRS: %i, t: %i',norm([ResultStruct.F_c_W(i,:) 0 0 0]-ResultStruct.F_c_W_erg(t,:,i)/tau_filt_t(t)), i, t)
    end
  end
end

figure(1);clf;hold on;
plot3(ResultStruct.r_0_0_C2(:,1,3),ResultStruct.r_0_0_C2(:,2,3),ResultStruct.r_0_0_C2(:,3,3));
xlabel('x');
ylabel('y');
zlabel('z');
title('Verlauf r_0_0_C2', 'interpreter', 'none');
%% Ergebnis speichern
save(fullfile(res_path, 'result.mat'), '-struct', 'ResultStruct');

return
%% Bilder Speichern
if debug %#ok<UNRCH>
  for i = 1:(AS.NJ+1)
    saveas(i, fullfile(res_path, sprintf('Link_%02d.fig')));
  end
end
saveas(100, fullfile(res_path, sprintf('Statistik.fig')));
saveas(200, fullfile(res_path, sprintf('Gesamt.fig')));
