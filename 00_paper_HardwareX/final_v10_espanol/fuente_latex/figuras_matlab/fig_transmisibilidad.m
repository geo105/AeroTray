function fig_transmisibilidad()
%FIG_TRANSMISIBILIDAD Fig. 11: propagacion de perturbaciones predicha por el modelo acoplado.
E = estilo_paper();
T = leer_csv('transmisibilidad.csv'); f = T.f_Hz;
fig = nueva_figura(E.ancho, 5.2);
x0 = [1.20 5.80 10.45]; w = 2.85; y0 = 1.05; hh = 3.7;

ax = crear_ejes(fig, [x0(1) y0 w hh], E); set(ax, 'XScale', 'log', 'YScale', 'log');
h1 = plot(ax, f, T.X_theta_a, '-', 'Color', E.azul, 'LineWidth', E.lw);
h2 = plot(ax, f, T.X_theta_b, '--', 'Color', E.azul, 'LineWidth', E.lw);
ajustar(ax, [1e-3 1e3]);
lg = leyenda(ax, [h1 h2], {'K_b=4,2 (100 ms)', 'K_b=3,5 (140 ms)'}, 'southwest', E);
set(findobj(lg, 'Type', 'text'), 'FontSize', E.fs - 1.5);
rotular(ax, E, 'Frecuencia (Hz)', '|X_b/\Theta| (mm/°)', '(a)');

ax = crear_ejes(fig, [x0(2) y0 w hh], E); set(ax, 'XScale', 'log', 'YScale', 'log');
h1 = plot(ax, f, T.theta_A_a, '-', 'Color', E.rojo, 'LineWidth', E.lw);
h2 = plot(ax, f, T.theta_A_b, '--', 'Color', E.rojo, 'LineWidth', E.lw);
ajustar(ax, [1e-2 1e2]);
lg = leyenda(ax, [h1 h2], {'h_c = 0,14 m', 'h_c = 0,10 m'}, 'northwest', E);
set(findobj(lg, 'Type', 'text'), 'FontSize', E.fs - 1);
rotular(ax, E, 'Frecuencia (Hz)', '|\Theta/A_J| (° por m/s^2)', '(b)');

ax = crear_ejes(fig, [x0(3) y0 w hh], E); set(ax, 'XScale', 'log', 'YScale', 'log');
h1 = plot(ax, f, T.X_A_directo, '-', 'Color', E.gris, 'LineWidth', E.lw);
h2 = plot(ax, f, T.X_A_total, '-', 'Color', E.negro, 'LineWidth', E.lw);
ajustar(ax, [1e-2 1e4]);
lg = leyenda(ax, [h1 h2], {'camino directo', 'directo + dron'}, 'southwest', E);
set(findobj(lg, 'Type', 'text'), 'FontSize', E.fs - 1);
rotular(ax, E, 'Frecuencia (Hz)', '|X_b/A_J| (mm por m/s^2)', '(c)');
guardar_figura(fig, 'fig_transmissibility');
end

function ajustar(ax, ylims)
xlim(ax, [2e-3 15]); ylim(ax, ylims);
set(ax, 'XTick', [1e-2 1e-1 1e0 1e1], 'YTick', 10.^(round(log10(ylims(1))):round(log10(ylims(2)))));
set(ax, 'XMinorTick', 'off', 'YMinorTick', 'off', 'XMinorGrid', 'off', 'YMinorGrid', 'off');
end
