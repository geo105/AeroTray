function fig_rampa()
%FIG_RAMPA Fig. 6: efecto del peso sostenido por los rotores, f, en la subida y la bajada de la rampa.
%   (a) pendiente de vuelco estatico hacia atras (subida de frente) y hacia adelante (bajada de frente)
%   (b) aceleracion maxima sin vuelco en 10 y 20 grados (el empuje quita peso, no masa)
%   (c) coeficiente de friccion que necesitan las ruedas motrices al subir
E = estilo_paper();
R = leer_csv('rampa.csv'); Q = leer_csv('rampa_param.csv');
f = 100*R.f; fr = 100*Q.f_ralenti;
fig = nueva_figura(E.ancho, 5.6);
x0 = [1.15 5.7 10.25]; w = 3.05; y0 = 1.05; hh = 3.85;
banda = E.claro(E.ocre, 0.2);

ax = crear_ejes(fig, [x0(1) y0 w hh], E);
fill(ax, [35 55 55 35], [0 0 70 70], banda, 'EdgeColor', 'none');
plot(ax, [0 100], [20 20], ':', 'Color', E.gris, 'LineWidth', E.lw_ref);
plot(ax, [0 100], [10 10], ':', 'Color', E.gris, 'LineWidth', E.lw_ref);
text(ax, 98, 20.6, '20°', 'Color', E.gris, 'FontSize', E.fs - 1, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
text(ax, 98, 10.6, '10°', 'Color', E.gris, 'FontSize', E.fs - 1, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
h1 = plot(ax, f, R.vuelco_atras, '-', 'Color', E.azul, 'LineWidth', E.lw);
h2 = plot(ax, f, R.vuelco_adelante, '--', 'Color', E.rojo, 'LineWidth', E.lw);
plot(ax, [fr fr], [0 70], '-.', 'Color', E.gris, 'LineWidth', E.lw_ref);
% vuelco estatico medido el 23/11 con el robot completo (10,4 kg), sin empuje (f = 0)
plot(ax, 0, 21.5, 'o', 'Color', E.azul, 'MarkerFaceColor', E.azul, 'MarkerSize', 3.5, 'Clipping', 'off');
plot(ax, 0, 30.5, 's', 'Color', E.rojo, 'MarkerFaceColor', E.rojo, 'MarkerSize', 3.5, 'Clipping', 'off');
text(ax, 3, 21.5, 'measured', 'Color', E.azul, 'FontSize', E.fs - 1.5, 'VerticalAlignment', 'top');
text(ax, fr + 2, 2, 'idle', 'Color', E.gris, 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom');
xlim(ax, [0 100]); ylim(ax, [0 70]); set(ax, 'XTick', 0:25:100, 'YTick', 0:10:70);
leyenda(ax, [h1 h2], {'rear (uphill)', 'front (downhill)'}, 'northwest', E);
rotular(ax, E, 'f (%)', 'Tip-over angle (°)', '(a)');

ax = crear_ejes(fig, [x0(2) y0 w hh], E);
fill(ax, [35 55 55 35], [-0.5 -0.5 5 5], banda, 'EdgeColor', 'none');
plot(ax, [0 100], [0 0], '-', 'Color', E.gris, 'LineWidth', E.lw_ref);
h1 = plot(ax, f, R.amax_10, '-', 'Color', E.verde, 'LineWidth', E.lw);
h2 = plot(ax, f, R.amax_20, '--', 'Color', E.rojo, 'LineWidth', E.lw);
xlim(ax, [0 100]); ylim(ax, [-0.5 5]); set(ax, 'XTick', 0:25:100, 'YTick', 0:1:5);
leyenda(ax, [h1 h2], {'10° ramp', '20° ramp'}, 'northwest', E);
rotular(ax, E, 'f (%)', 'a_{max} before tip-over (m/s^2)', '(b)');

ax = crear_ejes(fig, [x0(3) y0 w hh], E);
fill(ax, [0 100 100 0], [0.5 0.5 0.8 0.8], E.claro(E.gris, 0.18), 'EdgeColor', 'none');
fill(ax, [35 55 55 35], [0 0 1 1], banda, 'EdgeColor', 'none');
text(ax, 2, 0.79, {'available', 'rubber–wood'}, 'Color', E.gris, 'FontSize', E.fs - 1, 'VerticalAlignment', 'top');
h1 = plot(ax, f, R.mu_10, '-', 'Color', E.verde, 'LineWidth', E.lw);
h2 = plot(ax, f, R.mu_20, '--', 'Color', E.rojo, 'LineWidth', E.lw);
xlim(ax, [0 100]); ylim(ax, [0 1]); set(ax, 'XTick', 0:25:100, 'YTick', 0:0.2:1);
leyenda(ax, [h1 h2], {'10° ramp', '20° ramp'}, 'southeast', E);
rotular(ax, E, 'f (%)', 'Required \mu uphill', '(c)');
guardar_figura(fig, 'fig_rampa');
end
