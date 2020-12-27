% Plotte die abgespeicherten Ersatzkörper zur Kollisionserkennung zum
% Testen

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc

%% Init
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
atlas5_collbodies = load(fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat'));
atlas_version = uint8(5);
AS = atlas_const(atlas_version);

qJ = zeros(1,AS.NJ);
T_pelvis = eye(4);

%% Roboter plotten
figure(1);clf;set(1, 'Name', '1:CAD', 'NumberTitle', 'off');
hold on; grid on;axis equal;view(3);set(1, 'Renderer','OpenGL')
atlas_plot_robot(qJ, atlas_version, 1, T_pelvis);

%% Alle Körper plotten (Gesamtbild und Einzelkörperbild)
figure(2);clf;set(2, 'Name', '2:RepBody', 'NumberTitle', 'off');
hold on; grid on;axis equal;view(3);set(2, 'Renderer','OpenGL')
T_c_urdf = atlas5_wbody_fkine_urdf_num(qJ);
T_c_mdh = atlas5_wbody_fkine_mdh_num(qJ);
% Alle Segmente durchgehen
for i = 1:AS.NJ+1 % Index in Segment-Liste
  for Bild = 1:2 % Zwei verschiedene Bilder für jeden Körper zeichnen: Gesamt und Einzel
    if Bild == 1
      % Gesamt-Bild
      figure(2);
    else
      % Einzelkörper-Bild
      figure(10+i);clf;set(10+i, 'Name', sprintf('B%02d', i), 'NumberTitle', 'off');
      hold on; grid on;axis equal;view(3);set(10+i, 'Renderer','OpenGL');
      if i ~= AS.LI_rArm(7) && i ~= AS.LI_lArm(7)
        atlas_plot_wbody_link_stl(i, atlas_version, T_c_urdf);
      else
        atlas_plot_wbody_link_stl(i-1, atlas_version, T_c_urdf);
      end
      title(sprintf('Link %d: %s', i, AS.LN{i}), 'interpreter', 'none');
    end
    
    T_mdh_i = T_c_mdh(:,:,i);
    % Alle Ersatzkörper durchgehen
    I_i = find(atlas5_collbodies.I == i);
    for j = I_i' % Index in Ersatzkörper-Liste
      Typ_j = atlas5_collbodies.T(j);
      Par_j = atlas5_collbodies.P(j,:);

      if Typ_j == 1 % Quader
      % Eckpunkte des Quaders
      r_0_Q1 = eye(3,4) * T_mdh_i*[Par_j(1:3)';1];
      r_0_Q1Q2 = T_mdh_i(1:3,1:3)*Par_j(4:6)';
      r_0_Q1Q3 = T_mdh_i(1:3,1:3)*Par_j(7:9)';
      r_0_Q1Q4 = T_mdh_i(1:3,1:3)*Par_j(10:12)';
      plot_cube2(r_0_Q1, r_0_Q1Q2, r_0_Q1Q3, r_0_Q1Q4, 'b');
      elseif Typ_j == 2 % Zylinder
        r_0_P1 = eye(3,4) * T_mdh_i*[Par_j(1:3)';1];
        r_0_P2 = eye(3,4) * T_mdh_i*[Par_j(4:6)';1];
        r_zyl = Par_j(7)';
        drawCylinder([r_0_P1', r_0_P2', r_zyl], 'EdgeColor', 'k', ...
          'FaceAlpha', 0.3, 'FaceColor', 'b')
      end
    
      if Bild == 2
        % Koordinatensysteme
        trplot(T_pelvis*T_c_urdf(:,:,i), 'frame', 'urdf', 'arrow', 'rgb', 'length', 0.2);
        trplot(T_pelvis*T_c_mdh(:,:,i), 'frame', 'mdh', 'arrow', 'rgb', 'length', 0.3);
        axis auto
      end
    end
  end
end

%% Beenden
dockall
