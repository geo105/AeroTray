function fig_base()
%FIG_BASE Fig. 3: velocidad estacionaria frente a la pendiente y margen de vuelco de la base.
E = estilo_paper();
B = leer_csv('base_limites.csv'); be = B.beta_deg;
fig = nueva_figura(E.ancho, 5.4);

ax = crear_ejes(fig, [1.25 1.05 5.4 3.85], E);
h1 = plot(ax, be, B.v_15, '-', 'Color', E.azul, 'LineWidth', E.lw);
h2 = plot(ax, be, B.v_30, '--', 'Color', E.verde, 'LineWidth', E.lw);
h3 = plot(ax, be, B.v_50, '-.', 'Color', E.ocre, 'LineWidth', E.lw);
h4 = errorbar(ax, 0, 0.07, 0.01, 'o');
set(h4, 'Color', E.rojo, 'MarkerFaceColor', E.rojo, 'MarkerSize', 4, 'LineWidth', 0.9);
xlim(ax, [0 25]); ylim(ax, [0 0.5]); set(ax, 'XTick', 0:5:25, 'YTick', 0:0.1:0.5);
leyenda(ax, [h1 h2 h3 h4], {'ciclo 15 %', 'ciclo 30 %', 'ciclo 50 %', 'medido, 15 %'}, 'northeast', E);
rotular(ax, E, 'Pendiente \beta (°)', 'Velocidad estacionaria (m/s)', '(a)');

ax = crear_ejes(fig, [8.15 1.05 5.15 3.85], E);
for b = [10 20], plot(ax, [b b], [-1.5 7], ':', 'Color', E.gris, 'LineWidth', E.lw_ref); end
plot(ax, [0 25], [0 0], '-', 'Color', E.rojo, 'LineWidth', E.lw_ref);
h1 = plot(ax, be, B.amax_fijo, '-', 'Color', E.negro, 'LineWidth', E.lw);
h2 = plot(ax, be, B.amax_nivelado, '--', 'Color', E.negro, 'LineWidth', E.lw);
text(ax, 0.5, -0.12, 'por debajo de 0: vuelca', 'Color', E.rojo, 'FontSize', E.fs - 1, 'VerticalAlignment', 'top');
xlim(ax, [0 25]); ylim(ax, [-1.5 7]); set(ax, 'XTick', 0:5:25, 'YTick', -1:1:7);
leyenda(ax, [h1 h2], {'módulo fijo a la base', 'módulo nivelado (rótula libre)'}, 'northeast', E);
rotular(ax, E, 'Pendiente \beta (°)', 'a_{máx} antes de volcar (m/s^2)', '(b)');
guardar_figura(fig, 'fig_base_limits');
end
