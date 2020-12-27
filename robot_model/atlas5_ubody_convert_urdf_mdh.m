% Convert upper body dynamic parameters from urdf to mdh (v5)
% 
% Input:
% lr [1x1 logical]
%   true for left, false for right
% rSges_num_urdf [19x3 double]
%   center of mass of all atlas arm bodies in urdf frames
% Iges_num_urdf [19x6 double]
%   inertia of all atlas arm bodies in urdf frames
%   order: xx, yy, zz, xy, xz, yz
% 
% Output:
% rSges_num_mdh [19x3 double]
%   center of mass of all atlas arm bodies in MDH frames
% Iges_num_mdh [19x6 double]
%   inertia of all atlas arm bodies in MDH frames
%   order: xx, yy, zz, xy, xz, yz

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Universität Hannover


function [rSges_num_mdh, Iges_num_mdh] = ...
             atlas5_ubody_convert_urdf_mdh(rSges_num_urdf, Iges_num_urdf)

%% Init
% Coder Information
%#codegen
 
assert(isa(rSges_num_urdf,'double') && all(size(rSges_num_urdf) == [19 3]));  
assert(isa(Iges_num_urdf,'double') && all(size(Iges_num_urdf) == [19 6]));  


%% get frames
% hard coded transformations calculated in testfunctions/atlas_transformation_urdf_mdh_test
% T_c_mdh_urdf_hc(:,:,j) = T_c_mdh(:,:,j) \ T_c_urdf(:,:,j)

% T_c_mdh(:,:,j) ist die Transformation zu Segment Nr. j
% rSges_num_urdf(j,:) ist der Schwerpunkt davon

T_mdh_urdf_hc = NaN(4,4,19); % Transformation from urdf to mdh frame (hard coded)
for i = 1:19
  T_mdh_urdf_hc(:,:,i) = NaN(4);
end

%% Hard code these Transformations
p_num_arm = atlas5_arm_parameter_urdf(true);
% Torso
p_num_torso = atlas5_torso_parameter_urdf();
T_mdh_urdf_hc(:,:,1) = eye(4); % pelvis
T_mdh_urdf_hc(:,:,2) = transl([0;0;-p_num_torso(2)]);
T_mdh_urdf_hc(:,:,3) = [[[0,0,1;1,0,0;0,1,0], [0;0;0]];[0,0,0,1]];
T_mdh_urdf_hc(:,:,4) = trotx(-pi/2)*trotz(-pi/2)*transl([-p_num_arm(1);0;0]);
T_mdh_urdf_hc(:,:,5) = troty(pi/2)*trotz(pi/2);

% Ubody
for lr = [true, false]
  if lr == true
    lr_Off = 0;
  else
    lr_Off = 7;
  end
  p_num_arm = atlas5_arm_parameter_urdf(lr);

  T_mdh_urdf_hc(:,:,1+5+lr_Off) = [0.0000    -1.0000         0                0
                                   1.0000     0.0000         0                0
                                        0          0    1.0000    -p_num_arm(7)
                                        0          0         0           1.0000];

  T_mdh_urdf_hc(:,:,2+5+lr_Off) = [0.0000    0         1         0
                                   0        -1         0         0
                                   1         0         0         0
                                   0         0         0    1.0000];

  T_mdh_urdf_hc(:,:,3+5+lr_Off) = [-0.0000    0.0000    1.0000       -0.0000
                                    1.0000         0    0.0000             0
                                    0.0000    1.0000   -0.0000 p_num_arm(10)
                                         0         0         0        1.0000];
  if lr == false
    T_mdh_urdf_hc(3,4,3+5+lr_Off) = -T_mdh_urdf_hc(3,4,3+5+lr_Off);
  end

  T_mdh_urdf_hc(:,:,4+5+lr_Off) = [-0.0000    0.0000    1.0000   -0.0000
                                    0.0000   -1.0000    0.0000    0.0000
                                    1.0000    0.0000    0.0000         0
                                         0         0         0    1.0000];

  T_mdh_urdf_hc(:,:,5+5+lr_Off) = [-0.0000    0.0000    1.0000   -0.0000
                                    1.0000   -0.0000    0.0000    0.0000
                                    0.0000    1.0000    0.0000         0
                                         0         0         0    1.0000];
  % bis hierhin identisch mit v4

  T_mdh_urdf_hc(:,:,6+5+lr_Off) = [[[0,0,1;0,-1,0;1,0,0], [0;0;0]];[0,0,0,1]];
  T_mdh_urdf_hc(:,:,7+5+lr_Off) = [[[0,0,1;1,0,0;0,1,0], [0;0;0]];[0,0,0,1]];

  if lr % add additional transformations in new kinematic of simple shapes urdf
    T_mdh_urdf_hc(:,:,1+5+lr_Off) = T_mdh_urdf_hc(:,:,1+5+lr_Off) * trotz(3.14159265359); % shz->shx rotz pi
    T_mdh_urdf_hc(:,:,2+5+lr_Off) = T_mdh_urdf_hc(:,:,2+5+lr_Off) * trotz(3.14159265359); % shx->ely rotz pi (from before)
    T_mdh_urdf_hc(:,:,3+5+lr_Off) = T_mdh_urdf_hc(:,:,3+5+lr_Off) * trotz(3.14159265359); % ely->elx rotz pi (from before)
    T_mdh_urdf_hc(:,:,4+5+lr_Off) = T_mdh_urdf_hc(:,:,4+5+lr_Off) * trotz(3.14159265359); % elx->wry rotz pi (from before)
    T_mdh_urdf_hc(:,:,5+5+lr_Off) = T_mdh_urdf_hc(:,:,5+5+lr_Off) * trotz(3.14159265359) * troty(3.14159265359); % wry->wrx rotz pi (from before) and roty pi
    T_mdh_urdf_hc(:,:,6+5+lr_Off) = T_mdh_urdf_hc(:,:,6+5+lr_Off) * trotz(3.14159265359) * troty(3.14159265359); % wrx->wry2 rotz pi (from before) and roty pi (from before)
    T_mdh_urdf_hc(:,:,7+5+lr_Off) = T_mdh_urdf_hc(:,:,7+5+lr_Off) * trotz(3.14159265359); % wry2->ee rotz pi (from before) roty pi from before compensated by additional roty pi
  end
end

%% Convert center of mass
rSges_num_mdh = NaN(19,3);
for i = 1:19
  rSges_num_mdh(i,:) = (T_mdh_urdf_hc(1:3,1:4,i) * [rSges_num_urdf(i,:)';1])';
end
%% Convert Inertia
Iges_num_mdh = NaN(19,6);
for i = 1:19
  I_i_urdf = inertiavector2matrix(Iges_num_urdf(i,:));
  I_i_mdh = T_mdh_urdf_hc(1:3,1:3,i) * I_i_urdf * T_mdh_urdf_hc(1:3,1:3,i)';
  Iges_num_mdh(i,:) = inertiamatrix2vector(I_i_mdh);
end     