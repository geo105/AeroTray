function rotular(ax, E, etiqueta_x, etiqueta_y, panel)
%ROTULAR Nombres de los ejes, letra del panel y coma decimal en los numeros de los ejes.
%   rotular(ax, E, 'Tiempo (s)', 'Inclinacion (°)', '(a)')
if ~isempty(etiqueta_x), xlabel(ax, etiqueta_x, 'FontName', E.fuente, 'FontSize', E.fs_lab); end
if ~isempty(etiqueta_y), ylabel(ax, etiqueta_y, 'FontName', E.fuente, 'FontSize', E.fs_lab); end
if nargin > 4 && ~isempty(panel)
    text(ax, 0, 1.05, panel, 'Units', 'normalized', 'FontName', E.fuente, 'FontSize', E.fs_pan, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
end
coma_decimal(ax);
end
