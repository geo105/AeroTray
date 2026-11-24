function fig_ensayos_23nov()
%FIG_ENSAYOS_23NOV Ensayos del 23/11 sobre el robot completo: error por ensayo y dispersion por eje.
E = estilo_paper();
C = leer_csv('ensayos23_cajas.csv');
n = numel(C.ensayo);
fig = nueva_figura(E.ancho, 5.6);

% (a) distribucion del error de la bola en cada ensayo
ax = crear_ejes(fig, [1.2 1.05 8.6 3.85], E);
plot(ax, [0.3 n + 0.7], [30 30], '--', 'Color', E.gris, 'LineWidth', E.lw_ref);
plot(ax, [0.3 n + 0.7], [96.5 96.5], ':', 'Color', E.rojo, 'LineWidth', E.lw_ref);
text(ax, n + 0.6, 98, 'R_u', 'Color', E.rojo, 'FontSize', E.fs - 1, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
hOff = []; hOn = [];
for k = 1:n
    p = [C.p5(k) C.q1(k) C.mediana(k) C.q3(k) C.p95(k)];
    if C.rotores(k), col = E.ocre; else, col = E.azul; end
    h = caja(ax, k, p, 0.62, col, E);
    if C.rotores(k), hOn = h; else, hOff = h; end
    if C.completo(k)                       % ensayos con la base cruzando la rampa
        text(ax, k, C.p95(k) + 2, '*', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', E.fs + 2);
    end
end
etiquetas = cellfun(@(s) s(end-1:end), C.ensayo, 'UniformOutput', false);
xlim(ax, [0.3 n + 0.7]); ylim(ax, [0 135]);
set(ax, 'XTick', 1:n, 'YTick', 0:20:120, 'XGrid', 'off');
lg = leyenda(ax, [hOff hOn], {'rotors off', 'rotors on'}, 'north', E);
try, set(lg, 'Orientation', 'horizontal'); catch, end
rotular(ax, E, 'Trial BP23-', '|e| (mm)', '(a)');
set(ax, 'XTickLabel', etiquetas);

% (b) dispersion de la posicion en cada eje (mediana de los ensayos, barras de minimo a maximo)
axb = crear_ejes(fig, [11.1 1.05 2.35 3.85], E);
sx = C.sx_mm; sy = C.sy_mm;
bar(axb, 1, median(sx), 0.6, 'FaceColor', E.claro(E.morado, 0.7), 'EdgeColor', E.morado);
bar(axb, 2, median(sy), 0.6, 'FaceColor', E.claro(E.verde, 0.7), 'EdgeColor', E.verde);
plot(axb, [1 1], [min(sx) max(sx)], '-', 'Color', E.negro, 'LineWidth', E.lw_ref);
plot(axb, [2 2], [min(sy) max(sy)], '-', 'Color', E.negro, 'LineWidth', E.lw_ref);
xlim(axb, [0.4 2.6]); ylim(axb, [0 55]); set(axb, 'XTick', [1 2], 'YTick', 0:10:50, 'XGrid', 'off');
rotular(axb, E, '', 'Std. dev. (mm)', '(b)');
set(axb, 'XTickLabel', {'x-axis', 'y-axis'});
guardar_figura(fig, 'fig_ensayos_23nov');
end
