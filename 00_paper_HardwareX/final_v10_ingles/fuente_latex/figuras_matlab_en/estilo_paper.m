function E = estilo_paper()
%ESTILO_PAPER Parametros de estilo comunes a todas las figuras del paper de AeroTray.
%   E = estilo_paper() devuelve tamanos, colores y fuente, y fija los valores por defecto de
%   MATLAB (o Octave) para que ejes, textos y leyendas usen la misma fuente.
%   Las figuras se dibujan a su tamano final (ancho de texto del paper = 13,7 cm), asi que en LaTeX
%   se incluyen sin escalar y la letra queda en 8-9 pt, como el resto del documento.

E.ancho   = 13.7;      % ancho de texto del paper (elsarticle preprint, 390 pt) en cm
E.fs      = 8;         % numeros de los ejes, leyendas y anotaciones (pt)
E.fs_lab  = 9;         % nombres de los ejes (pt)
E.fs_pan  = 9;         % letra del panel, (a), (b)...
E.lw      = 1.1;       % lineas de datos (pt)
E.lw_ref  = 0.7;       % lineas de referencia (pt)
E.lw_eje  = 0.6;       % ejes

% Paleta categorica (validada para daltonismo y contraste sobre blanco), siempre en este orden
E.azul    = [0.000 0.447 0.698];   % #0072B2
E.rojo    = [0.835 0.369 0.000];   % #D55E00
E.verde   = [0.000 0.620 0.451];   % #009E73
E.ocre    = [0.690 0.478 0.000];   % #B07A00
E.morado  = [0.627 0.322 0.604];   % #A0529A
E.gris    = [0.43 0.43 0.43];      % referencias del modelo
E.negro   = [0 0 0];
E.claro   = @(c, a) 1 - a*(1 - c);  % color de relleno (sin transparencia, imprime igual en PDF)

% Fuente: la del paper (Latin Modern) si esta instalada; si no, Times New Roman.
% Para forzar otra: setenv('FUENTE_FIGURAS','CMU Serif') antes de llamar.
E.fuente = getenv('FUENTE_FIGURAS');
if isempty(E.fuente)
    preferidas = {'Latin Modern Roman', 'LM Roman 10', 'CMU Serif', 'Times New Roman'};
    E.fuente = 'Times New Roman';
    try
        disponibles = listfonts();
        for k = 1:numel(preferidas)
            if any(strcmpi(disponibles, preferidas{k})), E.fuente = preferidas{k}; break; end
        end
    catch
    end
end

props = {'defaultAxesFontName', E.fuente; 'defaultTextFontName', E.fuente; ...
         'defaultAxesFontSize', E.fs; 'defaultTextFontSize', E.fs; ...
         'defaultLegendFontName', E.fuente; 'defaultLegendFontSize', E.fs; ...
         'defaultAxesLineWidth', E.lw_eje; 'defaultLineLineWidth', E.lw; ...
         'defaultAxesTickDir', 'out'; 'defaultAxesBox', 'off'; ...
         'defaultAxesLabelFontSizeMultiplier', E.fs_lab/E.fs; ...
         'defaultAxesTitleFontSizeMultiplier', 1; 'defaultFigureColor', 'w'};
for k = 1:size(props, 1)
    try, set(0, props{k,1}, props{k,2}); catch, end   % algunas propiedades no existen en Octave
end
end
