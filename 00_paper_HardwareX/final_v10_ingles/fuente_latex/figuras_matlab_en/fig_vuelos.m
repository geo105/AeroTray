function fig_vuelos()
%FIG_VUELOS Fig. 15: error de actitud de cada vuelo en la guia y series de los vuelos 7 (v8) y 8 (v9).
E = estilo_paper();
R = leer_csv('vuelos_rms.csv'); A = leer_csv('vuelo_serie_v8.csv'); B = leer_csv('vuelo_serie_v9.csv');
fig = nueva_figura(E.ancho, 6.4);

% (a) error RMS por vuelo
ax = crear_ejes(fig, [1.25 1.45 5.2 4.25], E);
n = numel(R.vuelo); x = (1:n)'; d = 0.19;
h1 = bar(ax, x - d, R.alabeo_rms, 0.36, 'FaceColor', E.azul, 'EdgeColor', 'none');
h2 = bar(ax, x + d, R.cabeceo_rms, 0.36, 'FaceColor', E.rojo, 'EdgeColor', 'none');
for k = 1:n
    if isnan(R.alabeo_rms(k))
        text(ax, x(k), 0.15, 'no takeoff', 'Rotation', 90, 'FontSize', E.fs - 1.5, 'Color', E.gris);
    end
    text(ax, x(k), -0.95, R.fw{k}, 'FontSize', E.fs - 1, 'HorizontalAlignment', 'center', 'Clipping', 'off');
end
xlim(ax, [0.4 n + 0.6]); ylim(ax, [0 6.5]); set(ax, 'XTick', x, 'YTick', 0:1:6, 'XGrid', 'off');
leyenda(ax, [h1 h2], {'roll', 'pitch'}, 'northeast', E);
rotular(ax, E, '', 'RMS error above 1650 µs (°)', '(a)');
text(ax, 0.5, -0.27, 'Flight and firmware', 'Units', 'normalized', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'top', 'FontSize', E.fs_lab);

% (b) inclinacion y (c) orden media de motor, vuelos 7 (v8) y 8 (v9)
axb = crear_ejes(fig, [8.15 3.85 5.15 1.95], E);
h1 = plot(axb, A.t_s, A.inclinacion_deg, '-', 'Color', E.azul, 'LineWidth', 0.9);
h2 = plot(axb, B.t_s, B.inclinacion_deg, '-', 'Color', E.verde, 'LineWidth', 0.9);
plot(axb, [0 130], [12 12], ':', 'Color', E.negro, 'LineWidth', E.lw_ref);
text(axb, 2, 12.3, '12° cutoff', 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom');
xlim(axb, [0 130]); ylim(axb, [0 14]); set(axb, 'XTick', 0:25:125, 'YTick', 0:4:12, 'XTickLabel', {});
ylabel(axb, 'Tilt (°)', 'FontName', E.fuente, 'FontSize', E.fs_lab);
coma_decimal(axb); set(axb, 'XTickLabel', {});
text(axb, 0, 1.05, '(b)', 'Units', 'normalized', 'FontSize', E.fs_pan, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');

axc = crear_ejes(fig, [8.15 1.45 5.15 1.8], E);
g1 = plot(axc, A.t_s, A.colectivo_us, '-', 'Color', E.azul, 'LineWidth', 0.9);
g2 = plot(axc, B.t_s, B.colectivo_us, '-', 'Color', E.verde, 'LineWidth', 0.9);
text(axc, 71, 1620, 'flight 7 (v8)', 'Color', E.azul, 'FontSize', E.fs - 1);
text(axc, 128, 1340, 'flight 8 (v9)', 'Color', E.verde, 'FontSize', E.fs - 1, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
xlim(axc, [0 130]); ylim(axc, [1000 2000]); set(axc, 'XTick', 0:25:125, 'YTick', 1000:250:2000);
rotular(axc, E, 'Time (s)', 'Mean command (µs)', '(c)');
guardar_figura(fig, 'fig_flight_campaign');
end
