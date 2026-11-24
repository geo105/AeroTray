function fig_banco()
%FIG_BANCO Fig. 14: actitud del octocoptero en la rotula libre con colectivo de ralenti (18/11).
E = estilo_paper();
reg = {'banco_20261118_144610.csv', 'trial 1 (14:46:10)'; 'banco_20261118_144932.csv', 'trial 2 (14:49:32)'};
fig = nueva_figura(E.ancho, 4.75);
x0 = [1.25 7.95];
for k = 1:2
    D = leer_csv(reg{k,1});
    ax = crear_ejes(fig, [x0(k) 1.05 5.55 3.05], E);
    h1 = plot(ax, D.t_s, D.alabeo_deg, '-', 'Color', E.azul, 'LineWidth', 0.8);
    h2 = plot(ax, D.t_s, D.cabeceo_deg, '-', 'Color', E.rojo, 'LineWidth', 0.8);
    xlim(ax, [0 8]); ylim(ax, [-4 4]); set(ax, 'XTick', 0:2:8, 'YTick', -4:2:4);
    text(ax, 0.98, 0.96, reg{k,2}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'FontSize', E.fs - 1);
    if k == 1
        leyenda(ax, [h1 h2], {'roll', 'pitch'}, 'southwest', E);
        rotular(ax, E, 'Time (s)', 'Angle (°)', '(a)');
    else
        rotular(ax, E, 'Time (s)', '', '(b)');
    end
end
guardar_figura(fig, 'fig_drone_bench');
end
