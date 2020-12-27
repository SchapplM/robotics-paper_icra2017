% Erzeuge Bild mit Vergleich des Fehlers der Positionsschätzung bei
% EE-Kontakten und einem weiteren
% 
% Vorher ausführen: atlas5wbody_collisions_statistics_mc_manip.m
% 
% Punkte der Zufallsversuche nicht als Punktwolke sondern als
% senkrechtes Histogramm für jedes Segment
% 
% Um dieses Bild und error_rank_manipcoll_fig_gen.m in ein Bild zu zeichnen:
% Anpassen der if-else-Bedingung im Abschnitt "Zeichnen"

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
NC = length( d.maxfehler_JT );

% Siehe Ende von atlas5wbody_collisions_statistics_mc_manip.m
%% Punktwolke berechnen

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


%% Histogramme berechnen
nHist = 100;
Histdata_X = linspace(0, max(d.maxfehler_JT), nHist);
Histdata_Ya = NaN(nHist,AS.NL+1);
Histdata_Yp = NaN(nHist,AS.NL+1);
RD_iLd = NaN(AS.NL+1,1); % Annahme: Gleicher Rang für alle Kollisionen in einem Segment
for iL = 0:AS.NL
  ILd = d.KontaktKomb.LI5(1:NC) == iL;
  Histdata_Ya(:,iL+1) = hist(d.maxfehler_JT(ILd), Histdata_X);
  Histdata_Yp(:,iL+1) = Histdata_Ya(:,iL+1)/sum(Histdata_Ya(:,iL+1));
  RD_iL = RD_Vec(ILd);
  if length(unique(RD_iL)) > 1
    error('Verschiedene Rangdefizite für Segment %d', iL);
  end
  if ~isempty( RD_iL )
    RD_iLd(iL+1) = RD_iL(1);
  end
end

%% Zeichnen
figHandle = 1;
figure(figHandle);clf;
set(figHandle, 'windowstyle', 'normal');
hold on;
legplothdl = NaN(3,1);
styles = {'g^', 'mo', 'kx', 'rv'};

for iL = 0:AS.NL
  if true % any(iL == [AS.LI_Torso AS.LI_lArm, AS.LI_lLeg])
    %% Histogramm-Bild für linke Seite
    % Markertyp für Datenreihe bestimmen (aus Rang)
    for ii = 1:nBer
      if RD_iLd(iL+1)>=Ber_Start(ii) && RD_iLd(iL+1)<=Ber_Ende(ii)
        ms = styles{ii};
        break;
      end
    end

    % Histogramm skalieren auf Bereich der x-Achse
    h1 = iL;
    h2 = iL+0.8;
    % Skalierung auf größten Wert des Histogramms
    hmax = max( Histdata_Yp(:,iL+1) );

    Histdata_Y_skal = h1 + Histdata_Yp(:,iL+1)/hmax * (h2-h1);
    plot(Histdata_Y_skal, 1e2*Histdata_X, [ms(1),'-']);

    % Mittelwert berechnen und eintragen
    hmean = sum(Histdata_X'.*Histdata_Yp(:,iL+1));
    [~,It] = min(abs( Histdata_X - hmean ));
    pmean = Histdata_Yp(It,iL+1);

    plot(h1+pmean/hmax*(h2-h1), 1e2*hmean, ms);
    legplothdl(ii) = plot(NaN,NaN,[ms(1:2),'-']); % Leerer Plot für Legende
  else
    %% Punktwolken-Bild für Rest
    % Indizes zu den Punkten für dieses Segment iL
    ILd = d.KontaktKomb.LI5(1:NC) == iL;
    for ii = 1:nBer
      IP = ILd&I(:,ii);
      if sum(IP) == 0
        continue % keine Punktwolke
      end
      legplothdl(ii)=plot(d.KontaktKomb.LI5(IP), 1e2*d.maxfehler_JT(IP), styles{ii});
    end
  
  end
end

% Nummern der Segmente eintragen als Achsenbeschriftung
set(gca, 'XTick', 0:31);
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

XTL = {'no', LNTL{:}}; %#ok<CCAT>
set(gca, 'XTickLabel', XTL);

% Einen Ticklabel weglassen damit mehr Platz für das yLabel ist
set(gca, 'YTICK', [0, 50, 100, 150], 'YTICKLABEL', {'0', '50', '', '150'});

xl=xlabel('Location contact 5'); yl=ylabel(sprintf('Position\nerror [cm]'));
set(xl, 'Position' , [12.5000 -40 0])
set(yl, 'Position' , [-0.3 100 0])

% Beschriftungen für Ketten einfügen
xty = -33;
xtxs = 0;
CN = {'Torso', 'left Leg', 'right Leg', 'left Arm', 'right Arm'};
XPx = [AS.LI_Torso(2), AS.LI_lLeg(3), AS.LI_rLeg(3), AS.LI_lArm(3), AS.LI_rArm(3)];
for i = 1:5
  hdl=text(XPx(i), xty, 0, CN{i}, 'FontName', 'Times', 'FontSize', 8);
end
xlim([0,AS.LI_rArm(end)+1]);
ylim([0, 155]);
% Eigenes Gitter zwischen den Ketten einzeichnen
grenzen = [0.5, AS.LI_Torso(end)+0.5, AS.LI_lLeg(end)+0.5, AS.LI_rLeg(end)+0.5, AS.LI_lArm(end)+0.5, AS.LI_rArm(end)+0.5];
for i = 1:length(grenzen)
  plot(grenzen(i)*[1;1], [-1e5;1e5], 'k--', 'LineWidth', 0.5);
end
grid on

%% Formatierung
figure_format_publication(gca);
set_size_plot_subplot(figHandle, ...
  8.8*2, 4, ... % eine Spalte (halbe Seite)
  gca, ...
  0.05, 0.01, 0.01, 0.27, ... % l r u d
  0.01, 0.01); % x y

h = legend(legplothdl, legText, 'interpreter', 'latex');
set(h, 'Position', [0.0535 0.5872 0.1498 0.3514], ... % x y b h
  'Orientation', 'Vertical') 

Filebasename_res = 'error_rank_manipcoll_hist_breit';
export_fig(fullfile(figpath, [Filebasename_res, '.pdf']));