function h = caja(ax, x, p, ancho, color, E)
%CAJA Dibuja una caja de bigotes a partir de percentiles ya calculados (no necesita Statistics Toolbox).
%   p = [P5 Q1 mediana Q3 P95]; los bigotes van de P5 a P95. Devuelve el handle del relleno (para leyendas).
a = ancho/2;
plot(ax, [x x], [p(1) p(2)], '-', 'Color', E.negro, 'LineWidth', E.lw_ref);
plot(ax, [x x], [p(4) p(5)], '-', 'Color', E.negro, 'LineWidth', E.lw_ref);
plot(ax, x + [-a a]/2, [p(1) p(1)], '-', 'Color', E.negro, 'LineWidth', E.lw_ref);
plot(ax, x + [-a a]/2, [p(5) p(5)], '-', 'Color', E.negro, 'LineWidth', E.lw_ref);
h = patch(ax, x + [-a a a -a], [p(2) p(2) p(4) p(4)], E.claro(color, 0.45), 'EdgeColor', color, 'LineWidth', E.lw_ref);
plot(ax, x + [-a a], [p(3) p(3)], '-', 'Color', E.negro, 'LineWidth', E.lw);
end
