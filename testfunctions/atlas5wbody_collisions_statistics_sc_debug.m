% Debug-Skript für Matlab-Skript atlas5wbody_collisions_statistics_sc.m
% 
% Suche die Indizes, die zu falsch berechneten Werten führen.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

% Nomenklatur:
% 
% I   Binärindizes
% J   Zahlindizes mehrerer Einträge
% i   Ein Zahlindex
% 
% Benennung:
% L   Links
% C   Kollisionen

% Anzahl die falsch sind (beide Methode)
fprintf('Fehler mit Methode 2 (Gelenkmomente): %d\n', sum(sum(E_Matrix2>1e-10)));
fprintf('Fehler mit Methode 3 (Kraftsensoren): %d\n', sum(sum(E_Matrix3>1e-10)));

% Kontaktkörper, die falsch sind
IL_falsch = any((E_Matrix3'>1e-3));
JL_falsch = find(IL_falsch);
fprintf('Fehler für folgende Segmente:\n');
disp(JL_falsch);

iL_fi = JL_falsch(1);
IC_fi = (E_Matrix3(iL_fi, :) > 1e-3);
JC_fi = find(IC_fi);
iC_fi = JC_fi(1);
fprintf('Fehler bei Segment %d und Kollision Nummer %d\n', iL_fi, iC_fi);

jC_fi = JC_fi(1);

% Finde Index der gespeicherten Punkte
Jpl = find(ResultStruct.iCB == iL_fi);

% Daten vorbereiten
r_0_0_C1 = ResultStruct.r_0_0_C1(Jpl(jC_fi),:);
r_0_0_C3 = ResultStruct.r_0_0_C3(Jpl(jC_fi),:);
e_test = norm( r_0_0_C1-r_0_0_C3 );
E_Matrix3(iL_fi, jC_fi);
fprintf('Betrachteter Fehler: %1.5e\n', e_test);


%% Eingabe in statistics_sc.m
IL = iL_fi;

iRS = Jpl(jC_fi);
% Suche folgende Werte:
% fprintf