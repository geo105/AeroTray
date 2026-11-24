function fig = nueva_figura(ancho_cm, alto_cm)
%NUEVA_FIGURA Figura en blanco con el tamano final que tendra en el paper (cm).
fig = figure('Units', 'centimeters', 'Color', 'w', 'Visible', 'off');
pos = get(fig, 'Position');
set(fig, 'Position', [pos(1) pos(2) ancho_cm alto_cm], ...
    'PaperUnits', 'centimeters', 'PaperSize', [ancho_cm alto_cm], ...
    'PaperPosition', [0 0 ancho_cm alto_cm], 'PaperPositionMode', 'manual');
end
