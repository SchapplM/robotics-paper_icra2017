% Statistische Untersuchung von Kontakten an unterschiedlichen Segmenten
% des Atlas-Roboter-Arms

% TODO: Wähle passenden Hüllkörper aus

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-08
% (c) Institut für Regelungstechnik, Universität Hannover

mex_script_dependencies(mfilename('fullpath'), false)
rng(0); % Zurücksetzen des Zufallsgenerators, um reproduzierbare Ergebnisse zu erhalten

%% Init
NCL = 100; % Anzahl der untersuchten Kontakte pro Segment

debug = true;
lr = true;
atlas_version = uint8(5);
AS = atlas_const(atlas_version, lr);
E_Matrix = NaN(7, NCL);
% Roboter-Ersatzkörper laden
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
atlas5_collbodies = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'), 'I', 'T', 'P');

qJ_wbody = zeros(30,1);
qJ0_Arm = [0 -pi/4 pi/4 pi/3 pi/4 pi/3 0]'; % TODO: Bessere Pose

qJ_wbody(AS.JI_Arm) = qJ0_Arm;
% Kinematik
T_c_mdh_Wbody = atlas5_wbody_fkine_mdh_num(qJ_wbody');
T_c_mdh_Arm = atlas5_arm_fkine_mdh_num(qJ0_Arm', lr);

% Binärindizes der Floating-Base-Modellierung des untersuchten Arms
IQ = false(36,1);
IQ(6+AS.JI_Arm) = true;

IKollK = [1 1 1 1 1 1 1]; % Index, welcher Kollisionskörper für jedes Segment genommen werden soll
%% Test: Arm plotten
if debug
  figure(1);clf;set(1, 'Name', '1:CAD', 'NumberTitle', 'off');
  hold on; grid on;axis equal;view(3);set(1, 'Renderer','OpenGL')
  atlas_plot_robot(qJ_wbody', atlas_version, 1, eye(4));
end

%% Statistik
for IAL = 2:7 % Schleife über Armsegmente
  for iCL = 1:NCL
    IL = AS.LI_Arm(IAL);
    % Generiere zufällige Kontaktorte auf der Außenhülle

    IH = find(atlas5_collbodies.I == IL);
    IHK = IH(IKollK(IAL));

    % Transformation zu aktuellem Kontaktsegment
    T_i = T_c_mdh_Wbody(:,:,IL);
    Par_j = atlas5_collbodies.P(IHK,:);
    %% Zufälligen Kontaktpunkt generieren
    if atlas5_collbodies.T(IHK) == 1 % Quader
      p_i = box_random_surface_point_equal(Par_j);
    elseif atlas5_collbodies.T(IHK) == 2 % Zylinder
      p_i = cylinder_random_surface_point_equal(Par_j(1:7));
    else
      error('Nicht behandelter Körper');
    end
    p_0 = eye(3,4) * T_i*[p_i;1];

    %% Kontaktkraft
    F_c = rand(3,1)-0.5; % zufällige Kraft mit Mittelwert Null


    J_wbody1 = atlas5_wbody_body_jacobig_mdh_num_mex(qJ_wbody', uint8(AS.LI_Arm(IAL)), p_i');
    J_Arm1 = J_wbody1(:, AS.JI_Arm);

    tauArm_ext1 = J_Arm1' * [F_c; zeros(3,1)];
    tauwbody_floatb_ext1 = zeros(36,1);
    tauwbody_floatb_ext1(6+AS.JI_Arm) = tauArm_ext1;
    %% Kontaktortschätzung Iteration 1

    % Neuen Kontaktpunkt an einem weiter außen gelegenen Ersatzkörper
    [ic, r_i_i_C1, r_i_i_d, u_i] = atlas5_wbody_collision_isolation_forcesensor_mex(zeros(3,1), qJ_wbody, ...
      rotate_wrench([F_c;cross(p_i,F_c)],T_i(1:3,1:3)), uint8(IL), uint8(IL:AS.LI_Arm(end))', atlas5_collbodies);
    r_0_0_C1 = eye(3,4)*T_c_mdh_Wbody(:,:,IL)*[r_i_i_C1;1];
    if ic == 0 || any(isnan(r_i_i_C1))
      error('Keine Vorauswahl möglich');
    end
    
    % Momente anpassen
    J_wbody2 = atlas5_wbody_body_jacobig_mdh_num_mex(qJ_wbody', uint8(AS.LI_Arm(IAL)), r_i_i_C1');
    J_Arm2 = J_wbody2(:, AS.JI_Arm);

    tauArm_ext2 = J_Arm2' * [F_c; zeros(3,1)];
    tauwbody_floatb_ext2 = zeros(36,1);
    tauwbody_floatb_ext2(6+AS.JI_Arm) = tauArm_ext2;
%     return
    %% Test: Hüllkörper
    if debug
      change_current_figure(IAL);
      if iCL == 1,
        % Hüllkörper neu zeichnen
        clf; 
        hold on; grid on;axis equal;view(3);set(1, 'Renderer','OpenGL');
        
        % Hüllkörper dieses Segmentes zeichnen
        atlas_plot_wbody_link_collision_body(IL, T_c_mdh_Wbody, atlas5_collbodies, ...
          0.4, 0.6*[0 0 1], (0.6+0.2)*[1 1 1]);
        % Hüllkörper anderer Segmente zeichnen (zur Einordnung)
        for iil = AS.LI_Arm(1:7 ~= IAL)
          grau1 = 0.6+mod(iil-1,2)*0.15;
          atlas_plot_wbody_link_collision_body(iil, T_c_mdh_Wbody, atlas5_collbodies, ...
            0.4, grau1*[1 1 1], (grau1+0.2)*[0 0 1]);
        end        
        

        title(sprintf('Hüllkörper Segment %d', IAL));
      end
      hdl1 = plot3(p_0(1), p_0(2), p_0(3), 'rx');
      % Kreuz für Punkt
      % plot3(p_0(1)+[-0.1 +0.1], p_0(2)*[1;1], p_0(3)*[1;1], 'k-');
      % plot3(p_0(1)*[1;1], p_0(2)+[-0.1 +0.1], p_0(3)*[1;1], 'k-');
      % plot3(p_0(1)*[1;1], p_0(2)*[1;1], p_0(3)+[-0.1 +0.1], 'k-');  

      % Erkannter Punkt
      hdl2 = plot3(r_0_0_C1(1), r_0_0_C1(2), r_0_0_C1(3), 'cd');
      % Verbindung dazwischen
      hdl3 = plot3([p_0(1);r_0_0_C1(1)], [p_0(2);r_0_0_C1(2)], [p_0(3);r_0_0_C1(3)], 'm-');
      if iCL == 1
        legend([hdl1, hdl2, hdl3], {'Kontaktpunkt It. 1', 'Kontaktpunkt It. 2', 'Verbindung'});
      end
    end

    
    %% Kontaktortschätzung
    [ic, r_i_i_C2, r_i_i_d, u_i] = atlas5_wbody_collision_isolation_fullchain_mex(zeros(3,1), qJ_wbody, tauwbody_floatb_ext2, ...
      ones(30,1)*eps, atlas5_collbodies, IQ);
    if ic == 0
      % Keine Schätzung möglich.
      continue;
    end
    r_0_0_C2 = eye(3,4)*T_c_mdh_Wbody(:,:,ic)*[r_i_i_C2;1];
    %% Bild für Segment (alle Ersatzkörper)
    change_current_figure(20);hold on
    if iCL == 1 && IAL == 2
      clf; 
      hold on; grid on;axis equal;view(3);set(20, 'Renderer','OpenGL');
      for iil = AS.LI_Arm
        grau1 = 0.4+mod(iil-1,2)*0.25;
        atlas_plot_wbody_link_collision_body(iil, T_c_mdh_Wbody, atlas5_collbodies, ...
          0.4, grau1*[0 0 1], (grau1+0.2)*[1 1 1]);
      end
    end
    hdl1=plot3(r_0_0_C1(1), r_0_0_C1(2), r_0_0_C1(3), 'rx');
    hdl2=plot3(r_0_0_C2(1), r_0_0_C2(2), r_0_0_C2(3), 'cd');
    hdl3=plot3([r_0_0_C1(1);r_0_0_C2(1)], [r_0_0_C1(2);r_0_0_C2(2)], [r_0_0_C1(3);r_0_0_C2(3)], 'm-');
    if iCL == 1 && IAL == 2
      legend([hdl1, hdl2, hdl3], {'Sim', 'Isolation', 'Diff'});
    end
    %% Vergleich mit eingegebenen Daten

    
    e0 = r_0_0_C1 - r_0_0_C2;
  %   r_i_i_C1 - p_i
  %   tauJ_ext = 
    E_Matrix(IAL, iCL) = norm(e0);

    
  end
end

%% Plotten
figure(10);clf;hold on
for i = 1:7
  plot(i, E_Matrix(i,:),  'x');
end
% Mittelwerte
plot(1:7, mean(E_Matrix, 2) );

% Min-Max
mm = minmax2(E_Matrix);
plot(1:7, mm(:,1) );
plot(1:7, mm(:,2) );