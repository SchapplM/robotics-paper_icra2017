% Funktion zur Ausgabe von daten als Tabelle im Matlab terminal
%
% Eingabe:
% headings [1xn cell-array]
%   Spaltenueberschriften als cell-array
% width [1xn]
%   Spaltenbreiten in der Ausgabe (muessen ausrecihend fuer alle Inhalte sein,
%   damit die Tabelle sauber formatiert werden kann
% format [1xn cell-array]
%   cell-array mit printf-Formatstrings zur formatierung der Ausgabe, z.B. %5.3f
% uebrige Eingaben: Daten fuer die Ausgabe, die Daten duerfen unterschiedliche
% Laenge besitzen
%
% Ausgabe:
%   keine
function print_tables(headings, width, format, varargin)
for i = 1:length(headings)
  spaces_needed = width(i)-length(headings{i});
  for j = 1:spaces_needed
    fprintf(' ');
  end
  fprintf('%s', headings{i});
end
fprintf('\n');
l_max = 0;
for i = 1:length(varargin)
  if size(varargin{i},1) > l_max
    l_max = size(varargin{i},1);
  end
end
for i = 1:l_max
  for j = 1:length(headings)
    if length(varargin{j}) >= i
      s = sprintf(format{j}, varargin{j}(i));
      spaces_needed = width(j)-length(s);
      for k = 1:spaces_needed
        fprintf(' ');
      end
      fprintf('%s', s);
    end
  end
  fprintf('\n');
end
  