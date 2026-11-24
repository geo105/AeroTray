function fig_bp23_despegue()
%FIG_BP23_DESPEGUE Ensayo BP23-05 (23/11): bola, servos y despegue del dron con la bola encima.
E = estilo_paper();
D = leer_csv('bp23_despegue.csv'); M = leer_csv('bp23_marcas.csv');
fig = nueva_figura(E.ancho, 7.0);
tf = M.t_flotar_s; td = M.t_despegue_s;

ax = crear_ejes(fig, [1.35 4.20 11.9 2.25], E);
patch(ax, [tf td td tf], [-200 -200 200 200], E.claro(E.ocre, 0.18), 'EdgeColor', 'none');
h1 = plot(ax, D.t_s, D.x_mm, '-', 'Color', E.azul, 'LineWidth', 0.6);
h2 = plot(ax, D.t_s, D.y_mm, '-', 'Color', E.rojo, 'LineWidth', 0.6);
plot(ax, [td td], [-200 200], '-', 'Color', E.rojo, 'LineWidth', E.lw_ref);
plot(ax, [-2 100], [0 0], '-', 'Color', E.negro, 'LineWidth', 0.4);
text(ax, tf + 1, 190, {'Hover: collective', 'rises to 1750 µs'}, 'FontSize', E.fs - 1, 'VerticalAlignment', 'top');
text(ax, td - 1, -190, 'takeoff', 'Color', E.rojo, 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
xlim(ax, [-2 100]); ylim(ax, [-200 200]); set(ax, 'XTick', 0:20:100, 'YTick', -200:100:200);
leyenda(ax, [h1 h2], {'x', 'y'}, 'northwest', E);
rotular(ax, E, '', 'Ball (mm)', '(a)'); set(ax, 'XTickLabel', {});

axb = crear_ejes(fig, [1.35 1.40 11.9 2.05], E);
patch(axb, [tf td td tf], [1000 1000 1650 1650], E.claro(E.ocre, 0.18), 'EdgeColor', 'none');
g1 = plot(axb, D.t_s, D.servo_x_us, '-', 'Color', E.azul, 'LineWidth', 0.4);
g2 = plot(axb, D.t_s, D.servo_y_us, '-', 'Color', E.rojo, 'LineWidth', 0.4);
plot(axb, [-2 100], M.neutro_x_us*[1 1], ':', 'Color', E.azul, 'LineWidth', E.lw_ref);
plot(axb, [-2 100], M.tope_x_us*[1 1], '--', 'Color', E.negro, 'LineWidth', E.lw_ref);
plot(axb, [td td], [1000 1650], '-', 'Color', E.rojo, 'LineWidth', E.lw_ref);
xlim(axb, [-2 100]); ylim(axb, [1000 1650]); set(axb, 'XTick', 0:20:100, 'YTick', 1000:200:1600);
leyenda(axb, [g1 g2], {'servo x', 'servo y'}, 'southwest', E);
rotular(axb, E, 'Time from trial start (s)', 'Pulse (µs)', '(b)');
guardar_figura(fig, 'fig_bp23_despegue');
end
