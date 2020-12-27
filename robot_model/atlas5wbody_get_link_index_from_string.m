% Gebe Index eines Segments bei bekanntem Namen zurück
% In Simulink-Funktionen notwendig, da keine Cell-Arrays erlaubt sind.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
% (c) Institut für Regelungstechnik, Universität Hannover

function ind_link = atlas5wbody_get_link_index_from_string(string_link)

if strcmp(string_link, 'pelvis')
  ind_link = 1;
elseif strcmp(string_link, 'ltorso')
	ind_link = 2;
elseif strcmp(string_link, 'mtorso')
	ind_link = 3;
elseif strcmp(string_link, 'utorso')
	ind_link = 4;
elseif strcmp(string_link, 'head')
	ind_link = 5;
elseif strcmp(string_link, 'l_uglut')
	ind_link = 6;
elseif strcmp(string_link, 'l_lglut')
	ind_link = 7;
elseif strcmp(string_link, 'l_uleg')
	ind_link = 8;
elseif strcmp(string_link, 'l_lleg')
	ind_link = 9;
elseif strcmp(string_link, 'l_talus')
	ind_link = 10;
elseif strcmp(string_link, 'l_foot')
	ind_link = 11;
elseif strcmp(string_link, 'r_uglut')
	ind_link = 12;
elseif strcmp(string_link, 'r_lglut')
	ind_link = 13;
elseif strcmp(string_link, 'r_uleg')
	ind_link = 14;
elseif strcmp(string_link, 'r_lleg')
	ind_link = 15;
elseif strcmp(string_link, 'r_talus')
	ind_link = 16;
elseif strcmp(string_link, 'r_foot')
	ind_link = 17;
elseif strcmp(string_link, 'l_clav')
	ind_link = 18;
elseif strcmp(string_link, 'l_scap')
	ind_link = 19;
elseif strcmp(string_link, 'l_uarm')
	ind_link = 20;
elseif strcmp(string_link, 'l_larm')
	ind_link = 21;
elseif strcmp(string_link, 'l_ufarm')
	ind_link = 22;
elseif strcmp(string_link, 'l_lfarm')
	ind_link = 23;
elseif strcmp(string_link, 'l_hand')
	ind_link = 24;
elseif strcmp(string_link, 'r_clav')
	ind_link = 25;
elseif strcmp(string_link, 'r_scap')
	ind_link = 26;
elseif strcmp(string_link, 'r_uarm')
	ind_link = 27;
elseif strcmp(string_link, 'r_larm')
	ind_link = 28;
elseif strcmp(string_link, 'r_ufarm')
	ind_link = 29;
elseif strcmp(string_link, 'r_lfarm')
	ind_link = 30;
elseif strcmp(string_link, 'r_hand')
	ind_link = 31;
else
  ind_link = -1;
end
 
%  % Generierung:
%  AS = atlas_const(5);
%  for i = 1:length(AS.LN)
%    fprintf('elseif strcmp(string_link, ''%s'')\n', AS.LN{i});
%    fprintf('\tind_link = %d;\n', i);
%  end