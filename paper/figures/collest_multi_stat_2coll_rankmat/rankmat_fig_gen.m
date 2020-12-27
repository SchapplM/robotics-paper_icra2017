% Modus 1:
% Erzeuge Bild mit Vergleich des Rangs der Jacobi-Matrix über alle
% Kombinationen von Kontakten
% Modus 2:
% Erzeuge Bild mit Vergleich des Fehlers bei Isolation basierend auf
% verallgemeinerten Kräften über alle  Kombinationen von Kontakten
% Ähnliches Bild wie aus rankmat_fig_gen.m
% 
% Vorher ausführen: atlas5wbody_collisions_statistics_mc.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

clc
clear

Modus = 2; % Benutzereingabe

%% Init
NP = 2;
collhdl_pfad = fileparts(which('humanoid_collisionhandling_path'));
figpath = fullfile(collhdl_pfad, 'paper', 'figures', 'collest_multi_stat_2coll_rankmat');
res_path = fullfile(collhdl_pfad, 'results', 'atlas5wbody_collisions_statistics_mc');
res_dat = fullfile(res_path, sprintf('result_%dgleichzeitig.mat', NP));

d = load(res_dat);
AS = atlas_const(5);

%% Zeichnen
% Siehe Ende von atlas5wbody_collisions_statistics_mc.m
figHandle = 1;
figure(figHandle);clf;
set(figHandle, 'windowstyle', 'normal');
hold on;
if Modus == 1
  I_gut = (d.nInf2==NP*6);
else
  I_gut = (d.maxfehler_JT < 1e-3);
end
I_schlecht = ~I_gut;
legplothdl = NaN(3,1);
% getestete Fälle
legplothdl(1)=plot3(d.KontaktKomb.LI1(I_gut), d.KontaktKomb.LI2(I_gut), d.nInf2(I_gut), 'g^');
legplothdl(2)=plot3(d.KontaktKomb.LI1(~I_gut), d.KontaktKomb.LI2(I_schlecht), d.nInf2(I_schlecht), 'rv');
% nicht getestete Fälle (nicht funktionierend mit Algorithmus)
[x,y] = meshgrid( 1:AS.NL );
nf = NaN(AS.NL, AS.NL);
for i = 1:AS.NL
  for j = 1:AS.NL
    % Prüfe, ob die Segmente zur selben Kette gehören
    if i == j
      nf(i,j) = true;
    end
  end
end

legplothdl(3)=plot3(x(:), y(:), nf(:), 'kx');

set(gca, 'XTick', 1:31);
set(gca, 'YTick', 1:31);

% Nummern der Segmente eintragen als Achsenbeschriftung
TL = cell(AS.NL,1);
LNTL = cell(31,1);
for i = 1:AS.NL
  iic = NaN;
  if any(i == AS.LI_Torso)
    iic = find( i == AS.LI_Torso );
  end
  if any(i == AS.LI_lLeg)
    iic = find( i == AS.LI_lLeg );
  end
  if any(i == AS.LI_rLeg)
    iic = find( i == AS.LI_rLeg );
  end
  if any(i == AS.LI_lArm)
    iic = find( i == AS.LI_lArm );
  end
  if any(i == AS.LI_rArm)
    iic = find( i == AS.LI_rArm );
  end
  LNTL{i} = sprintf('%d', iic);
end
XTL = LNTL;
YTL = LNTL;
for i = 1:AS.NL
  if any(i == AS.LI_Torso)
    YTL{i} = '';
  end
  if any(i == AS.LI_rArm)
    XTL{i} = '';
  end
end

set(gca, 'XTickLabel', XTL);
set(gca, 'YTickLabel', YTL);
set(gca, 'YTickLabelRotation', 90);
zlabel('Rang(Jges)');
xlabel('Location Contact 1'); ylabel('Location Contact 2');

xlim([0,AS.LI_lArm(end)+1]);
ylim([AS.LI_lLeg(1)-1,AS.LI_rArm(end)+1]);

% Eigenes Gitter zwischen den Ketten einzeichnen
grenzen = [0.5, AS.LI_Torso(end)+0.5, AS.LI_lLeg(end)+0.5, AS.LI_rLeg(end)+0.5, AS.LI_lArm(end)+0.5, AS.LI_rArm(end)+0.5];
for i = 1:length(grenzen)
  plot(grenzen(i)*[1;1], [0;32], 'k-');
  plot([0;32], grenzen(i)*[1;1], 'k-');
end
grid on

% Achsenbeschriftung nach unten setzen
yl = get(gca, 'YLABEL');
set(yl, 'Position' , [-2.9286 18.3767 0])
xl = get(gca, 'XLABEL');
set(xl, 'Position' , [12.5000 1.5 0])

% Beschriftungen für Ketten einfügen
xty = 2.0;
xtxs = 0;
ytx = -2.5;
CN = {'Torso', 'left Leg', 'right Leg', 'left Arm', 'right Arm'};
XPx = [AS.LI_Torso(1), AS.LI_lLeg(2), AS.LI_rLeg(2), AS.LI_lArm(3), AS.LI_rArm(3)];
YPy = XPx+[-1,-1,-1,-1,-1];
for i = 1:5
  if i<5
    hdl=text(XPx(i), xty, 0, CN{i}, 'FontName', 'Times', 'FontSize', 8);
  end
  if i>1
    hdl=text(ytx, YPy(i), 0, CN{i}, 'Rotation', 90, 'FontName', 'Times', 'FontSize', 8);
  end
end
  

%% Ausgabe: Anzahl der falschen
for i = 1:31
  for j = (i+1):31
    I_seg = (d.KontaktKomb.LI1 == i & d.KontaktKomb.LI2 == j);
    I_rd = d.nInf2<12;
    nrichtig = sum(I_gut&I_seg);
    nfalsch = sum(I_schlecht&I_seg);
    if nrichtig == 0 || nfalsch == 0
      continue
    end
    fprintf('Seg.1: %d (%s), Seg.2: %d (%s). %d richtig. %d falsch.\n', ...
      i, AS.LN{i}, j, AS.LN{j}, nrichtig, nfalsch);
  end
end

%% Formatierung
figure_format_publication(gca);
set_size_plot_subplot(figHandle, ...
  8.8, 7, ... % eine Spalte (halbe Seite)
  gca, ...
  0.15, 0.01, 0.01, 0.17, ... % l r u d
  0.01, 0.01); % x y

% h = legend(legplothdl, {'$\mathrm{rank}(J) = 12$', '$\mathrm{rank}(J) < 12$'}, 'interpreter', 'latex');
h = legend(legplothdl, {'success', 'failure', 'same link'});
set(h, 'Position', [0.6354 0.2020 0.3153 0.1723], ... % x y b h
  'Orientation', 'Vertical') 

Filebasename_res = sprintf('rankmat_fig_2coll_Modus%d', Modus);
export_fig(fullfile(figpath, [Filebasename_res, '.pdf']));