% Generiere die Ersatzkörper für Atlas und speichere sie in kompakter Form
% ab.
% Die Ersatzkörper entsprechen denen aus dem URDF.
% Zusatzinformationen werden mit folgendem Skript generiert:
% contact/atlas_contactbody_properties.m
% 
% Erzeugt Variablen in contact/atlas5_contactbody_list.mat
% I
%   Robotersegment-Indizes (0=Basis, ...)
% T
%   Typ des Ersatzkörpers
%     1. Quader
%     2. Zylinder
%     3. Kapsel
% P
%   Parameter der Ersatzkörpers
%   Quader
%     Punkt1, 3 Vektoren (zu jeder Ecke eine)
%   Zylinder/Kapsel
%     Punkt1, Punkt2, Radius
% 
% Siehe auch:
% atlas_contactbody_test.m, atlas_contactbody_properties.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc

%% Init
AS = atlas_const(5);
% Initialisiere Ausgabevariablen
I = uint8( zeros(98,1) );
T = I;
P = NaN(98,12);

% Atlas-URDF einlesen. Quelle: drcsim
filename = fullfile(fileparts(which('humanoid_collisionhandling_path.m')), ...
  'robot_model', 'atlas_v5_simple_shapes_with_head.urdf');

[atlasurdf, atlasxmltree] = parseXML(filename);
% Liste aller Link-Elemente
link_List = atlasxmltree.getElementsByTagName('link');

% Umwandlung von URDF in MDH-Koordinatensysteme
T_c_urdf = atlas5_wbody_fkine_urdf_num(zeros(1,30));
T_c_mdh = atlas5_wbody_fkine_mdh_num(zeros(1,30));

%% Alle Links durchgehen
II = 0;
for k = 0:link_List.getLength-1
  % Aktuelles link-Element
  Link_k_Item = link_List.item(k);
  linkname = char(Link_k_Item.getAttribute('name'));
  i_link = atlas5wbody_get_link_index_from_string(linkname);
  
  if i_link == -1
    continue % Kein Atlas-Segment
  end
  % Von MDH nach URDF
  T_km_ku = T_c_mdh(:,:,i_link) \ T_c_urdf(:,:,i_link);
  
  % Kind-Elemente nach Kollisionen durchsuchen
  link_k_collList = Link_k_Item.getElementsByTagName('collision');
  for ii = 0:link_k_collList.getLength-1
    collision_ii_Item = link_k_collList.item(ii);
    % Origin-Element
    origin_ii_List = collision_ii_Item.getElementsByTagName('origin');
    origin_ii_Item = origin_ii_List.item(0);
    % Ursprung des Ersatzkörpers (im URDF-KS)
    ryp_tmp = char(origin_ii_Item.getAttribute('rpy'));
    xyz_tmp = char(origin_ii_Item.getAttribute('xyz')); % Mittelpunkt des Ersatzkörpers
    rpy_num = str2double( strsplit(ryp_tmp,' ') );
    xyz_num = str2double( strsplit(xyz_tmp,' ') );
    R = eulxyz2r([rpy_num(1); rpy_num(2); rpy_num(3)]);
    % Geometry-Element
    geometry_ii_List = collision_ii_Item.getElementsByTagName('geometry');
    geometry_ii_Item = geometry_ii_List.item(0);
    
    % Prüfe auf unterschiedliche Objekte
    box_ii_List = geometry_ii_Item.getElementsByTagName('box');
    box_ii_Item = box_ii_List.item(0);
    if ~isempty(box_ii_Item)
      boxsize_tmp = char( box_ii_Item.getAttribute('size') );
      boxsize_num = str2double( strsplit(boxsize_tmp,' ') );
      II = II + 1;
      I(II) = i_link;
      T(II) = 1;

      % Vektor vom Körper-KS-Ursprung (URDF) zu einem Eckpunkt Q1 als Aufpunkt und
      % dann Vektoren von diesem ersten Eckpunkt zu allen anderen
      % Eckpunkten Q2,Q3,Q4
      r_ku_Q1 = xyz_num' - R(1:3,1)*boxsize_num(1)/2 + ...
                         - R(1:3,2)*boxsize_num(2)/2 + ...
                         - R(1:3,3)*boxsize_num(3)/2;
      r_ku_Q1_Q2 = boxsize_num(1)*R(1:3,1);
      r_ku_Q1_Q3 = boxsize_num(2)*R(1:3,2);
      r_ku_Q1_Q4 = boxsize_num(3)*R(1:3,3);
      
      % Umwandlung der Punkte ins MDH-Körper-KS, da dieses später auch nur
      % noch verwendet wird.
      r_km_Q1 = eye(3,4) * T_km_ku * [r_ku_Q1;1]; % Ortsvektor verschieben und drehen
      r_km_Q1_Q2 = T_km_ku(1:3,1:3) * r_ku_Q1_Q2; % Richtungsvektoren nur drehen
      r_km_Q1_Q3 = T_km_ku(1:3,1:3) * r_ku_Q1_Q3;
      r_km_Q1_Q4 = T_km_ku(1:3,1:3) * r_ku_Q1_Q4;
      
      % Übergabeformat für Quader: Alle Punkte im MDH-KS
      P(II,:) = [r_km_Q1', r_km_Q1_Q2', r_km_Q1_Q3', r_km_Q1_Q4'];
      
      fprintf('Körper %d (%s). Quader. 0Q1=[%s], Q1Q2=[%s], Q1Q3=[%s],Q1Q4=[%s].\n', ...
        i_link, linkname, disp_array(P(II,1:3), '%1.3f'), disp_array(P(II,4:6), '%1.3f'), ...
        disp_array(P(II,7:9), '%1.3f'), disp_array(P(II,10:12), '%1.3f'));
      continue  % Nur eine Geometrie enthalten
    end
    cylinder_ii_List = geometry_ii_Item.getElementsByTagName('cylinder');
    cylinder_ii_Item = cylinder_ii_List.item(0);
    if ~isempty(cylinder_ii_Item)
      cylrad_tmp = char( cylinder_ii_Item.getAttribute('radius') );
      cyllen_tmp = char( cylinder_ii_Item.getAttribute('length') );
      cylrad_num = str2double( strsplit(cylrad_tmp,' ') );
      cyllen_num = str2double( strsplit(cyllen_tmp,' ') );
      
      
      II = II + 1;
      I(II) = i_link;
      T(II) = 2;

      % Anfangs- und Endpunkt der Mittellinie im URDF-Körper-KS
      r_ku_P1 = xyz_num' - R(1:3,3)*cyllen_num/2;
      r_ku_P2 = r_ku_P1 + cyllen_num*R(1:3,3);
      
      % Umwandlung der Punkte ins MDH-Körper-KS, da dieses später auch nur
      % noch verwendet wird.
      r_km_P1 = eye(3,4) * T_km_ku * [r_ku_P1;1];
      r_km_P2 = eye(3,4) * T_km_ku * [r_ku_P2;1];
      
      % Übergabeformat für Zylinder
      % 1..3: Aufpunkt im MDH-Körper-KS
      % 4..6: Zweiter Punkt im MDH-Körper-KS
      % 7: Radius
      P(II,:) = [r_km_P1', r_km_P2', cylrad_num, NaN(1,5)];
      fprintf('Körper %d (%s). Zylinder. 0P1=[%s], 0P2=[%s], r=%1.3f\n', ...
        i_link, linkname, disp_array(P(II,1:3), '%1.3f'), disp_array(P(II,4:6), '%1.3f'), P(II,7));
    end
  end
end

%% Kontaktdaten speichern
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
export_pfad = fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat');
save(export_pfad, 'T', 'I', 'P');
fprintf('Kollisionskörper nach %s gespeichert\n', export_pfad);
