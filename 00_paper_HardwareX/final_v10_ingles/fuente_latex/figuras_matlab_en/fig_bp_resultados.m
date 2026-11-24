function fig_bp_resultados()
%FIG_BP_RESULTADOS Error de la bola en las cuatro condiciones del 17-18/11 y un recorrido en la rampa.
E = estilo_paper();
C = leer_csv('bp_cajas.csv'); S = leer_csv('bp_rampa_serie.csv');
fig = nueva_figura(E.ancho, 5.4);

ax = crear_ejes(fig, [1.2 1.45 4.9 3.3], E);
cols = {E.azul, E.ocre, E.verde, E.morado};
for k = 1:numel(C.n)
    caja(ax, k, [C.p5(k) C.q1(k) C.mediana(k) C.q3(k) C.p95(k)], 0.55, cols{k}, E);
end
plot(ax, [0.4 4.6], [110 110], ':', 'Color', E.negro, 'LineWidth', E.lw_ref);
text(ax, 0.5, 111, 'plate half-width', 'FontSize', E.fs - 1.5, 'VerticalAlignment', 'bottom');
xlim(ax, [0.4 4.6]); ylim(ax, [0 125]); set(ax, 'XTick', 1:4, 'YTick', 0:25:125, 'XGrid', 'off');
rotular(ax, E, '', '|e| (mm)', '(a)');
set(ax, 'XTickLabel', {'(a)', '(b)', '(c)', '(d)'});
text(ax, 0.5, -0.2, 'Condition', 'Units', 'normalized', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'top', 'FontSize', E.fs_lab);

axb = crear_ejes(fig, [7.35 1.45 6.0 3.3], E);
h1 = plot(axb, S.t_s, S.x_mm, '-', 'Color', E.azul, 'LineWidth', 0.6);
h2 = plot(axb, S.t_s, S.y_mm, '-', 'Color', E.rojo, 'LineWidth', 0.6);
plot(axb, [0 70], [0 0], '-', 'Color', E.negro, 'LineWidth', 0.4);
xlim(axb, [0 70]); ylim(axb, [-110 110]); set(axb, 'XTick', 0:10:70, 'YTick', -100:50:100);
leyenda(axb, [h1 h2], {'x', 'y'}, 'southeast', E);
rotular(axb, E, 'Time (s)', 'Ball position (mm)', '(b)');
guardar_figura(fig, 'fig_bp_results');
end
