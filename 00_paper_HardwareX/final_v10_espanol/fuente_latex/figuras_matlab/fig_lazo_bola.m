function fig_lazo_bola()
%FIG_LAZO_BOLA Fig. 10: frontera de estabilidad del lazo de la bola y mecanismo de cuatro barras.
E = estilo_paper();
K = leer_csv('kd_critico.csv'); B = leer_csv('cuatro_barras.csv');
fig = nueva_figura(E.ancho, 5.6);

% (a) mayor k_d estable frente al retardo total del lazo
ax = crear_ejes(fig, [1.25 1.05 6.2 4.05], E);
fill(ax, [30 180 180 30], [4.5 4.5 6 6], E.claro(E.ocre, 0.22), 'EdgeColor', 'none');
plot(ax, [30 180], [4.5 4.5], '-', 'Color', E.negro, 'LineWidth', E.lw_ref);
plot(ax, [30 180], [6 6], '--', 'Color', E.negro, 'LineWidth', E.lw_ref);
cols = {'Kb_6_0', E.gris, '-', 'K_b = 6,0 (CAD)'; 'Kb_4_9', E.azul, '-', 'K_b = 4,9'; ...
        'Kb_3_5', E.verde, '--', 'K_b = 3,5'; 'Kb_2_5', E.rojo, '-.', 'K_b = 2,5'};
h = zeros(1, 4);
for k = 1:4
    h(k) = plot(ax, K.T_ms, K.(cols{k,1}), cols{k,3}, 'Color', cols{k,2}, 'LineWidth', E.lw);
end
xlim(ax, [30 180]); ylim(ax, [0 16]); set(ax, 'XTick', 30:30:180, 'YTick', 0:2:16);
leyenda(ax, h, cols(:,4)', 'northeast', E);
rotular(ax, E, 'Retardo total del lazo T (ms)', 'k_d crítico (unidades del firmware)', '(a)');

% (b) angulo del plato frente al angulo del servo
ax = crear_ejes(fig, [9.0 1.05 4.3 4.05], E);
g = B.gamma_deg; r = 220*45/400;
fill(ax, [-r r r -r], [-28 -28 28 28], E.claro(E.verde, 0.18), 'EdgeColor', 'none');
h1 = plot(ax, g, B.alpha_deg, '-', 'Color', E.azul, 'LineWidth', E.lw);
h2 = plot(ax, g, 0.436*g, '--', 'Color', E.gris, 'LineWidth', E.lw_ref);
text(ax, 0, -25, 'rango usado (±220 µs)', 'Color', E.verde, 'FontSize', E.fs - 1, 'HorizontalAlignment', 'center');
xlim(ax, [-60 60]); ylim(ax, [-28 28]); set(ax, 'XTick', -60:30:60, 'YTick', -20:10:20);
leyenda(ax, [h1 h2], {'cuatro barras (CAD)', 'lineal, pendiente 0,436'}, 'northwest', E);
rotular(ax, E, 'Ángulo del servo \gamma (°)', 'Ángulo del plato \alpha (°)', '(b)');
guardar_figura(fig, 'fig_ball_stability');
end
