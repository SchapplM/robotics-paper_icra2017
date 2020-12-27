% Testen der Kollisionspunktschätzung für den Atlas-Arm mit idealen
% Eingabedaten für einen Zeitschritt
% 
% Ziel:
% Simulierter Kontaktpunkt soll geschätztem exakt entsprechen
% 
% Ergebnis:
% Externes Moment ist richtig, Kontaktpunkt falsch.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover

load('atlas5_contactbody_list.mat');
AS = atlas_const(5);
atlas_bodies.I=I;
atlas_bodies.T=T;
atlas_bodies.P=P;

noise_level = 1;%7.3e-2;

I_contact = 7; % Linknummer am Arm
Ichain = uint8(AS.LI_lArm(I_contact:7))';

% Atlas Arm
AS = atlas_const(5);
q_Arm = [pi/6; pi/6; pi/6; pi/6; 0; 1e-5; 0]; % zeros(1,7)';
q_wbody = zeros(30,1);
q_wbody(AS.JI_lArm) = q_Arm;
J = atlas5_wbody_body_jacobig_mdh_num(q_wbody', uint8(AS.LI_lArm(I_contact)), zeros(3,1)');

% (q_Arm, true);

% J_teil = J;
% J_teil(:,2:end) = 0;

F_i_test = [3;2;1;0;0;0]; % Im Körper-KS
T_c_mdh = atlas5_wbody_fkine_mdh_num(q_wbody');
T_W_i = T_c_mdh(1:4,1:4,AS.LI_lArm(I_contact));
T_i_W = inv(T_W_i);
r_i_test_raw = [0.1;0.2;0.3];

% Transformationsmatrix in 2D-Format umwandeln
T_c_mdh_stack = NaN(31,16);
for i = 1:31
  T_c_mdh_stack(i,:) = reshape(T_c_mdh(:,:,i),1,16);
end

% determine point on surface in line of force action
S1 = atlas5_wbody_intersect_collbodies(r_i_test_raw, F_i_test(1:3),...
     uint8(AS.LI_lArm(I_contact)), T_c_mdh_stack,...
     atlas_bodies.I==AS.LI_lArm(I_contact), atlas_bodies);
% plot_zyl_pic
ind_r = find(S1(:,4)==min(S1(:,4)), 1, 'first');
if isempty(ind_r) || isnan(ind_r)
  ind_r = find(S1(:,6)==min(S1(:,6)), 1, 'first');
  S2 = atlas5_wbody_intersect_collbodies(S1(ind_r,1:3)', F_i_test(1:3),...
     uint8(AS.LI_lArm(I_contact)), T_c_mdh_stack,...
     atlas_bodies.I==AS.LI_lArm(I_contact), atlas_bodies);
%   tmpS1=S1;
%   tmpr_i_test_raw=r_i_test_raw;
%   r_i_test_raw = S1(ind_r,1:3)';
%   S1=S2;
%   plot_zyl_pic
%   S1=tmpS1;
%   r_i_test_raw = tmpr_i_test_raw;
  ind_r = find(S2(:,4)==min(S2(:,4)), 1, 'first');
  if isempty(ind_r) || isnan(ind_r)
    ind_r = find(S2(:,6)==min(S2(:,6)), 1, 'first');
  end
  r_i_test = S2(ind_r,1:3)';
else
  r_i_test = S1(ind_r,1:3)';
end

J_test = atlas5_wbody_body_jacobig_mdh_num(q_wbody', uint8(AS.LI_lArm(I_contact)), r_i_test');
tau_ext_r = J_test' * rotate_wrench(F_i_test, T_W_i(1:3,1:3) );
% tau_noise = noise_level*(-1+2*rand(length(tau_ext_r),1));
start_link = 2;
tau_noise = zeros(length(tau_ext_r),1);
tau_noise(AS.JI_lArm(2:end)) = 0.1;
tau_ext = tau_ext_r + tau_noise;

J_Arm = J(:,AS.JI_lArm(start_link:end));

F_W_i = pinv(J_Arm')*tau_ext(AS.JI_lArm(start_link:end)); % Gl. (58). In Welt-KS!!!
F_i = rotate_wrench(F_W_i, T_i_W(1:3,1:3) );

if any( abs( F_i_test(1:3) - F_i(1:3)) > 1e-10 )
  warning('Kraft falsch geschätzt');
end

m_test = cross(r_i_test, F_i_test(1:3)); % im Körper-KS

if any( abs( m_test-F_i(4:6)) > 1e-10 )
  warning('Moment falsch geschätzt');
end

[ic, r_i_i_C, r_i_i_d, u_i] = atlas5_wbody_collision_isolation_forcesensor( ...
  zeros(3,1), q_wbody, rotate_wrench(F_i, T_W_i(1:3,1:3)), uint8(AS.LI_lArm(I_contact)), Ichain, atlas_bodies);

if any( abs( r_i_test-r_i_i_C) > 1e-10 )
  warning('Angriffspunkt r_i_i_C falsch geschätzt');
end

if any(abs((r_i_test-r_i_i_C)./F_i_test(1:3)-(r_i_test(1)-r_i_i_C(1))/F_i_test(1))>1e-10)
  warning('r_i_i_C nicht auf Kraftwirklinie');
end

if any(abs((r_i_test-r_i_i_d)./F_i_test(1:3)-(r_i_test(1)-r_i_i_d(1))/F_i_test(1))>1e-10)
  warning('r_i_i_d nicht auf Kraftwirklinie');
end

if abs( r_i_i_d'*F_i_test(1:3)) > 1e-10 
  warning('r_i_i_d und F_i_test nicht senkrecht');
end

J_d = atlas5_wbody_body_jacobig_mdh_num(q_wbody', uint8(AS.LI_lArm(I_contact)), r_i_i_C');

if any( abs( J_d(:)-J_test(:)) > 1e-10 )
  warning('Jacobi falsch geschätzt');
end


tau_ext_test = J_d(:,AS.JI_lArm)' * rotate_wrench(F_i_test, T_W_i(1:3,1:3));

if any( abs( tau_ext_test-tau_ext(AS.JI_lArm) ) > 1e-10 )
  warning('tau_ext falsch geschätzt');
end

r_W_test = T_W_i*[r_i_test; 1];
r_W_C = T_W_i*[r_i_i_C; 1];

figure(1);clf;
axis equal
hold on;
plot3(r_W_test(1), r_W_test(2), r_W_test(3), 'xg', 'linewidth', 2);
plot3(r_W_C(1), r_W_C(2), r_W_C(3), 'xr', 'linewidth', 2);
atlas_plot_robot(q_wbody', uint8(5), 1, eye(4));
legend('real','estimated')

print_tables({'tau_noise', 'F_i_r', 'F_i', 'r_i_r', 'r_i'}, [9 6 15 10 10],...
  {'%9.4f', '%6.1f', '%15e', '%10f', '%10f'}, tau_noise(AS.JI_lArm(2:end)), F_i_test, F_i, r_i_test, r_i_i_C);
