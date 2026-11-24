function guardar_figura(fig, nombre)
%GUARDAR_FIGURA Exporta la figura a PDF vectorial en ../figures/<nombre>.pdf
%   En MATLAB R2020a o posterior usa exportgraphics; en versiones anteriores y en Octave usa print.
carpeta = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
if ~exist(carpeta, 'dir'), mkdir(carpeta); end
archivo = fullfile(carpeta, [nombre '.pdf']);
es_octave = exist('OCTAVE_VERSION', 'builtin') ~= 0;
if ~es_octave && exist('exportgraphics', 'file')
    exportgraphics(fig, archivo, 'ContentType', 'vector', 'BackgroundColor', 'white');
elseif es_octave
    print(fig, archivo, '-dpdf');
else
    print(fig, archivo, '-dpdf', '-painters');
end
close(fig);
fprintf('  %s\n', archivo);
end
