function T = leer_csv(nombre)
%LEER_CSV Lee datos/<nombre>.csv en una estructura con un campo por columna.
%   Las columnas numericas quedan como vectores columna; las de texto, como cell arrays.
%   (Funciona igual en MATLAB y en Octave, que no tiene readtable.)
archivo = fullfile(fileparts(mfilename('fullpath')), 'datos', nombre);
fid = fopen(archivo, 'r');
if fid < 0, error('No se encuentra %s. Ejecuta antes exportar_datos_matlab.py', archivo); end
cabecera = strtrim(fgetl(fid));
nombres = strsplit(cabecera, ',');
filas = {};
while true
    l = fgetl(fid);
    if ~ischar(l), break; end
    if isempty(strtrim(l)), continue; end
    filas{end+1, 1} = strsplit(l, ',', 'CollapseDelimiters', false); %#ok<AGROW>
end
fclose(fid);
T = struct();
for j = 1:numel(nombres)
    col = cellfun(@(f) f{j}, filas, 'UniformOutput', false);
    num = str2double(col);
    campo = matlab.lang.makeValidName(nombres{j});
    if all(isnan(num) & ~cellfun(@isempty, col) & ~strcmpi(col, 'nan'))
        T.(campo) = col;
    else
        T.(campo) = num;
    end
end
end
