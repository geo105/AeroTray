function lg = leyenda(ax, h, textos, ubicacion, E)
%LEYENDA Leyenda sin recuadro, con la fuente y el tamano del paper.
%   Si todos los elementos son lineas, la leyenda se dibuja a mano (tramo de linea + texto) dentro de
%   los ejes: asi cada muestra sale con su color y trazo exactos en MATLAB y en Octave (la leyenda de
%   Octave 10 a veces repite el color de la primera linea). Llamar despues de fijar xlim/ylim.
%   Si hay rellenos (patch, bar), se usa legend().
if all(strcmp(get(h, 'Type'), 'line'))
    lg = leyenda_manual(ax, h, textos, ubicacion, E);
    return
end
lg = legend(ax, h, textos, 'Location', ubicacion, 'Box', 'off', 'FontName', E.fuente, 'FontSize', E.fs);
try, set(lg, 'Color', 'none'); catch, end
try, set(lg, 'ItemTokenSize', [16 8]); catch, end
end

function g = leyenda_manual(ax, h, textos, ubicacion, E)
n = numel(h);
pos = get(ax, 'Position'); un = get(ax, 'Units');
set(ax, 'Units', 'centimeters'); pcm = get(ax, 'Position'); set(ax, 'Units', un);
dy = 0.36 / pcm(4);                  % alto de renglon: 0,36 cm
lx = 0.55 / pcm(3);                  % largo del tramo: 0,55 cm
mx = 0.12 / pcm(3); my = 0.10 / pcm(4);
ub = lower(ubicacion);
% ancho aproximado del texto mas largo (0,6 * tamano de letra por caracter)
largo = max(cellfun(@numel, textos)) * 0.48 * E.fs * 0.0353 / pcm(3);
if ~isempty(strfind(ub, 'east')), x0 = max(mx, 1 - mx - largo - lx - 0.12/pcm(3)); else, x0 = mx; end
if ~isempty(strfind(ub, 'south')), y0 = my + (n - 0.5) * dy; else, y0 = 1 - my - 0.5*dy; end
g = gobjects_compat(2*n);
for k = 1:n
    yk = y0 - (k - 1) * dy;
    [xa, ya] = norm2data(ax, [x0 x0 + lx], [yk yk]);
    g(2*k-1) = plot(ax, xa, ya, 'Color', get(h(k), 'Color'), 'LineStyle', get(h(k), 'LineStyle'), ...
        'LineWidth', max(get(h(k), 'LineWidth'), 0.9), 'Clipping', 'off', 'HandleVisibility', 'off');
    mk = get(h(k), 'Marker');
    if ~strcmp(mk, 'none')
        [xm, ym] = norm2data(ax, x0 + lx/2, yk);
        plot(ax, xm, ym, 'LineStyle', 'none', 'Marker', mk, 'Color', get(h(k), 'Color'), ...
            'MarkerFaceColor', get(h(k), 'MarkerFaceColor'), 'MarkerSize', get(h(k), 'MarkerSize'), 'HandleVisibility', 'off');
    end
    g(2*k) = text(ax, x0 + lx + 0.10/pcm(3), yk, textos{k}, 'Units', 'normalized', 'FontName', E.fuente, ...
        'FontSize', E.fs, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
end
end

function [x, y] = norm2data(ax, xn, yn)
xl = get(ax, 'XLim'); yl = get(ax, 'YLim');
if strcmp(get(ax, 'XScale'), 'log'), x = 10.^(log10(xl(1)) + xn*(log10(xl(2)) - log10(xl(1))));
else, x = xl(1) + xn*(xl(2) - xl(1)); end
if strcmp(get(ax, 'YScale'), 'log'), y = 10.^(log10(yl(1)) + yn*(log10(yl(2)) - log10(yl(1))));
else, y = yl(1) + yn*(yl(2) - yl(1)); end
end

function g = gobjects_compat(n)
try, g = gobjects(n, 1); catch, g = zeros(n, 1); end
end
