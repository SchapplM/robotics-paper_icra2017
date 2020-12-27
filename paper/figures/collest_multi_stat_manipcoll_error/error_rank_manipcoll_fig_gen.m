% Erzeuge Bild mit Vergleich des Fehlers der Positionsschätzung bei
% EE-Kontakten und einem weiteren (als Punktwolke der Fehler)
% 
% Vorher ausführen: atlas5wbody_collisions_statistics_mc_manip.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

clc
clear

%% Init
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
figpath = fullfile(collhdl_pfad, 'paper', 'figures', 'collest_multi_stat_manipcoll_error');
res_path = fullfile(collhdl_pfad, 'results', 'atlas5wbody_collisions_statistics_mc');
res_dat = fullfile(res_path, sprintf('result_manip.mat'));

d = load(res_dat);
AS = atlas_const(5);

%% Zeichnen
% Siehe Ende von atlas5wbody_collisions_statistics_mc_manip.m

% Indizes zu den Ergebnissen mit unterschiedlichen Rangdefizit
I = false( length(d.nInf2), 3 );
Ber_Start = [0;1;2;4];
Ber_Ende =  [0;1;3;6];
nBer = 4;
legText = {'$RD=0$', '$RD=1$', '$RD \leq 3$', '$RD \geq 4$'};
RD_Vec = d.nCont*6-d.nInf2; % Rangdefizit
for ii = 1:nBer
  I(:,ii) = RD_Vec>=Ber_Start(ii) & RD_Vec<=Ber_Ende(ii);
end

figHandle = 1;
figure(figHandle);clf;
set(figHandle, 'windowstyle', 'normal');
hold on;
legplothdl = NaN(3,1);
styles = {'g^', 'mo', 'kx', 'rv'};
for ii = 1:nBer
  legplothdl(ii)=plot(d.KontaktKomb.LI5(I(:,ii)), 1e2*d.maxfehler_JT(I(:,ii)), styles{ii});
end

set(gca, 'XTick', 0:31);


% Namen der Ketten eintragen
LNTL = cell(AS.NL,1);
LNTL{AS.LI_Torso(4)} = 'Torso';
LNTL{AS.LI_lLeg(4)} = 'l. Leg';
LNTL{AS.LI_rLeg(4)} = 'r. Leg';
LNTL{AS.LI_lArm(4)} = 'l. Arm';
LNTL{AS.LI_rArm(4)} = 'r. Arm';
XTL = {'none', LNTL{:}}; %#ok<CCAT>
  
set(gca, 'XTickLabel', XTL);
xlabel('Contact link 5'); ylabel('Position error [cm]');

xlim([0,AS.LI_rArm(end)+2]);
ylim([0, 1e2*max(d.maxfehler_JT)]);
% Eigenes Gitter zwischen den Ketten einzeichnen
grenzen = [0.5, AS.LI_Torso(end)+0.5, AS.LI_lLeg(end)+0.5, AS.LI_rLeg(end)+0.5, AS.LI_lArm(end)+0.5, AS.LI_rArm(end)+0.5];
for i = 1:length(grenzen)
  plot(grenzen(i)*[1;1], [0;1e5], 'k-');
end
grid on

%% Formatierung
figure_format_publication(gca);
set_size_plot_subplot(figHandle, ...
  8.8, 6, ... % eine Spalte (halbe Seite)
  gca, ...
  0.12, 0.01, 0.01, 0.16, ... % l r u d
  0.01, 0.01); % x y

h = legend(legplothdl, legText, 'interpreter', 'latex');
set(h, 'Position', [0.2, 0.8, 0.15, .08], ... % x y b h
  'Orientation', 'Vertical') 

Filebasename_res = 'error_rank_manipcoll';
export_fig(fullfile(figpath, [Filebasename_res, '.pdf']));