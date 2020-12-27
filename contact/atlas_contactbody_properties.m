% Berechne Kennzahlen zu den abgespeicherten Kollisionskörpern
% Fläche und Volumen jedes Ersatzkörpers
% 
% Modifiziert den Inhalt von contact/atlas5_contactbody_list.mat

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc

%% Init
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
matdat_pfad = fullfile(collhdl_pfad, 'contact', 'atlas5_contactbody_list.mat');
atlas5_collbodies = load(matdat_pfad);
atlas_version = uint8(5);

%% Berechne Oberflächen und Volumina
NEK = length(atlas5_collbodies.I);
A = NaN(NEK, 1);
V = NaN(NEK, 1);
for i = 1:NEK
  if atlas5_collbodies.T(i) == 1
    r_0_Q1Q2 = atlas5_collbodies.P(i,4:6);
    r_0_Q1Q3 = atlas5_collbodies.P(i,7:9);
    r_0_Q1Q4 = atlas5_collbodies.P(i,10:12);
    
    A3 = norm( cross(r_0_Q1Q2, r_0_Q1Q3) );
    A2 = norm( cross(r_0_Q1Q2, r_0_Q1Q4) );
    A1 = norm( cross(r_0_Q1Q3, r_0_Q1Q4) );
    A_Gesamt = A1*2 + A2*2 + A3*2;
    
    V_Gesamt = dot( r_0_Q1Q2, cross(r_0_Q1Q3, r_0_Q1Q4) );
  elseif atlas5_collbodies.T(i) == 2
    R = atlas5_collbodies.P(i,7);
    r_0_P1 = atlas5_collbodies.P(i,1:3)';
    r_0_P2 = atlas5_collbodies.P(i,4:6)';
    zeta = (r_0_P1-r_0_P2);
    A_Deckel = pi*R^2;
    A_Mantel = norm(zeta) * pi * 2*R;
    A_Gesamt = 2*A_Deckel + A_Mantel;
    
    V_Gesamt = A_Deckel*norm(zeta);
  else
    error('Typ nicht definiert');
  end
  
  A(i) = A_Gesamt;
  V(i) = V_Gesamt;
end
%% Abspeichern der modifizierten Ersatzkörper
T = atlas5_collbodies.T;
I = atlas5_collbodies.I;
P = atlas5_collbodies.P;
save(matdat_pfad, 'T', 'I', 'P', 'A', 'V');