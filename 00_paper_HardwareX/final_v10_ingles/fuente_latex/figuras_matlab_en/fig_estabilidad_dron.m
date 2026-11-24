function fig_estabilidad_dron()
%FIG_ESTABILIDAD_DRON Fig. 9: autoridad de par y mapa de estabilidad lineal del lazo de actitud.
E = estilo_paper();
R = leer_csv('autoridad_requerida.csv'); D = leer_csv('autoridad_disponible.csv');
fig = nueva_figura(E.ancho, 5.75);

% (a) autoridad de par: par necesario frente a par disponible con la velocidad angular saturada
ax = crear_ejes(fig, [1.25 1.2 5.4 4.0], E);
plot(ax, R.theta_deg, R.tau_hc_id, '-', 'Color', E.negro, 'LineWidth', E.lw);
plot(ax, R.theta_deg, R.tau_hc_cad, '--', 'Color', E.negro, 'LineWidth', E.lw);
lim = {'banco', E.rojo, 'available, v4–v7 limits (25 °/s)'; 'vuelo', E.verde, 'available, v8–v9 limits (60 °/s)'};
for k = 1:2
    sel = strcmp(D.limites, lim{k,1});
    tau = D.tau(sel); kT = D.kT(sel);
    plot(ax, [0 14], tau(kT == 1)*[1 1], '-', 'Color', lim{k,2}, 'LineWidth', 1.3);
    plot(ax, [0 14], tau(kT > 1)*[1 1], '-', 'Color', E.claro(lim{k,2}, 0.55), 'LineWidth', 0.8);
    if k == 1
        text(ax, 13.8, tau(kT == 1) + 0.03, lim{k,3}, 'Color', lim{k,2}, 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    else
        text(ax, 0.3, tau(kT == 1) + 0.03, lim{k,3}, 'Color', lim{k,2}, 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom');
    end
end
plot(ax, [12 12], [0 2.0], ':', 'Color', E.gris, 'LineWidth', E.lw_ref);
text(ax, 11.8, 1.98, '12° cutoff', 'Color', E.gris, 'FontSize', E.fs - 1, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top');
text(ax, 13.8, R.tau_hc_id(end) + 0.05, 'required, h_c = 0.14 m', 'FontSize', E.fs - 1, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
text(ax, 13.8, 0.95, {'required,', 'h_c = 0.10 m (CAD)'}, 'FontSize', E.fs - 1, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top');
xlim(ax, [0 14]); ylim(ax, [0 2.0]); set(ax, 'XTick', 0:2:14, 'YTick', 0:0.5:2);
rotular(ax, E, 'Tilt |\theta| (°)', 'Torque (N m)', '(a)');

% (b) region de estabilidad del modelo lineal completo en el plano (h_c, k_T)
ax = crear_ejes(fig, [8.15 1.2 5.15 4.0], E);
estilos = {'012', '-', 'J = 0.12 kg m^2'; '009', '--', 'J = 0.09 kg m^2'; '015', ':', 'J = 0.15 kg m^2'};
h = zeros(1, 3);
for k = 1:3
    M = dlmread(fullfile(fileparts(mfilename('fullpath')), 'datos', ['estabilidad_J' estilos{k,1} '.csv']), ',');
    hc = M(1, 2:end); kT = M(2:end, 1); Z = M(2:end, 2:end);
    kcrit = nan(size(hc));
    for j = 1:numel(hc)
        i = find(Z(:, j) < 0, 1, 'first');
        if ~isempty(i), kcrit(j) = kT(i); end
    end
    ok = ~isnan(kcrit);
    if k == 1
        xs = hc(ok); ys = kcrit(ok);
        fill(ax, [xs fliplr(xs)], [ys max(kT)*ones(size(ys))], E.claro(E.verde, 0.25), 'EdgeColor', 'none');
    end
    h(k) = plot(ax, hc(ok), kcrit(ok), estilos{k,2}, 'Color', E.negro, 'LineWidth', E.lw);
end
plot(ax, [0.08 0.20 0.20 0.08 0.08], [1.33 1.33 1.52 1.52 1.33], '-', 'Color', E.rojo, 'LineWidth', 1.1);
plot(ax, 0.14, 1.42, 'o', 'Color', E.rojo, 'MarkerFaceColor', E.rojo, 'MarkerSize', 4);
text(ax, 0.081, 1.56, 'identified', 'Color', E.rojo, 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom');
plot(ax, 0.10, 1.0, 's', 'Color', E.azul, 'MarkerFaceColor', E.azul, 'MarkerSize', 4);
text(ax, 0.094, 1.0, {'design: CAD h_c,', 'firmware map'}, 'Color', E.azul, 'FontSize', E.fs - 1, 'HorizontalAlignment', 'right');
text(ax, 0.015, 1.88, 'stable', 'Color', E.verde, 'FontSize', E.fs);
text(ax, 0.245, 0.75, 'unstable', 'Color', E.rojo, 'FontSize', E.fs, 'HorizontalAlignment', 'right');
xlim(ax, [0 0.25]); ylim(ax, [0.6 2.0]); set(ax, 'XTick', 0:0.05:0.25, 'YTick', 0.6:0.2:2.0, 'XGrid', 'off', 'YGrid', 'off');
rotular(ax, E, 'CoM height above joint h_c (m)', 'Thrust gain ratio k_T', '(b)');
guardar_figura(fig, 'fig_drone_stability');
end
