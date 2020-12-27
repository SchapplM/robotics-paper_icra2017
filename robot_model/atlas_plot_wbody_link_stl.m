% Plot one link of the Atlas wbody model with STL files from drcsim
% 
% Input:
% j
%   Number of Robot Segment
% version [1x1 uint8]
%   Atlas Model Version number
% T_c_urdf
%   All frames in urdf coordinates
% 
% Output:
% hdl
%   handle to the created patch object
% 
% Sources:
% [1] Atlas Simulator STL files

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
% (c) Institut für Regelungstechnik, Universität Hannover


function hdl = atlas_plot_wbody_link_stl(j, version, T_c_urdf)

%% Init

AS = atlas_const(version);

find_Torso = (AS.LI_Torso == j);
if any(find_Torso);
  hdl = atlas_plot_torso_link_stl(find(find_Torso), version, T_c_urdf(:,:,j));
end

find_lLeg = (AS.LI_lLeg == j);
if any(find_lLeg);
  hdl = atlas_plot_leg_link_stl( find(find_lLeg), true, version, T_c_urdf(:,:,j) );
end

find_rLeg = (AS.LI_rLeg == j);
if any(find_rLeg);
  hdl = atlas_plot_leg_link_stl( find(find_rLeg), false, version, T_c_urdf(:,:,j) );
end

find_lArm = (AS.LI_lArm == j);
if any(find_lArm);
  hdl = atlas_plot_arm_link_stl( find(find_lArm), true, version, T_c_urdf(:,:,j) );
end

find_rArm = (AS.LI_rArm == j);
if any(find_rArm);
  hdl = atlas_plot_arm_link_stl( find(find_rArm), false, version, T_c_urdf(:,:,j) );
end