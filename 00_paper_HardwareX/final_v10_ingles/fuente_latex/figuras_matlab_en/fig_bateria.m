function fig_bateria()
%FIG_BATERIA Fig. 7: colectivo de despegue y relacion empuje/peso frente a la tension de las LiPo.
E = estilo_paper();
B = leer_csv('bateria.csv'); S = leer_csv('soc_ticks.csv'); v = B.Voc;
fig = nueva_figura(E.ancho, 6.15);
pos = {[1.25 1.25 5.4 3.60], [8.15 1.25 5.15 3.60]};

% (a) colectivo necesario para despegar
ax = crear_ejes(fig, pos{1}, E); set(ax, 'XDir', 'reverse');
fill(ax, [v; flipud(v)], [min(B.u_min, 2050); flipud(min(B.u_max, 2050))], E.claro(E.azul, 0.3), 'EdgeColor', 'none');
plot(ax, v, min(B.u_c, 2050), '-', 'Color', E.azul, 'LineWidth', E.lw);
techos = [1750 1850 1950]; est = {':', '--', '-.'};
nom = {'hover command, 1750 µs', 'sustained limit, 1850 µs', '20 s max, 1950 µs'};
for k = 1:3
    plot(ax, [10.6 12.6], techos(k)*[1 1], est{k}, 'Color', E.negro, 'LineWidth', E.lw_ref);
    if k == 3
        text(ax, 12.57, techos(k) + 6, nom{k}, 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom');
    else
        text(ax, 10.63, techos(k) + 6, nom{k}, 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end
end
marcas(ax, E, [1650 2050]);
xlim(ax, [10.6 12.6]); ylim(ax, [1650 2050]); set(ax, 'XTick', 10.6:0.4:12.6, 'YTick', 1650:100:2050);
rotular(ax, E, 'LiPo voltage at rest (V)', 'Takeoff collective PWM (µs)', '');
eje_carga(fig, ax, S, E, '(a)');

% (b) empuje maximo / peso en el limite sostenido y en el maximo
ax = crear_ejes(fig, pos{2}, E); set(ax, 'XDir', 'reverse');
fill(ax, [v; flipud(v)], [max(B.tw1950_min, 0.7); flipud(max(B.tw1950_max, 0.7))], E.claro(E.morado, 0.25), 'EdgeColor', 'none');
fill(ax, [v; flipud(v)], [max(B.tw1850_min, 0.7); flipud(max(B.tw1850_max, 0.7))], E.claro(E.ocre, 0.3), 'EdgeColor', 'none');
h1 = plot(ax, v, max(B.tw1850_c, 0.7), '-', 'Color', E.ocre, 'LineWidth', E.lw);
h2 = plot(ax, v, max(B.tw1950_c, 0.7), '--', 'Color', E.morado, 'LineWidth', E.lw);
plot(ax, [10.6 12.6], [1 1], '-', 'Color', E.negro, 'LineWidth', E.lw_ref);
text(ax, 12.57, 0.98, 'thrust = weight', 'FontSize', E.fs - 1, 'VerticalAlignment', 'top');
marcas(ax, E, [0.7 1.9]);
xlim(ax, [10.6 12.6]); ylim(ax, [0.7 1.9]); set(ax, 'XTick', 10.6:0.4:12.6, 'YTick', 0.7:0.2:1.9);
leyenda(ax, [h1 h2], {'at 1850 µs', 'at 1950 µs'}, 'northeast', E);
rotular(ax, E, 'LiPo voltage at rest (V)', 'Max thrust / weight', '');
eje_carga(fig, ax, S, E, '(b)');
guardar_figura(fig, 'fig_bateria');
end

function marcas(ax, E, yl)
% tensiones observadas el 20/11: flota con 12,4 V y no flota con 11,1 V
plot(ax, [12.4 12.4], yl, '-', 'Color', E.verde, 'LineWidth', 1);
plot(ax, [11.1 11.1], yl, '-', 'Color', E.rojo, 'LineWidth', 1);
text(ax, 12.37, yl(1) + 0.03*diff(yl), 'hovered', 'Color', E.verde, 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom');
text(ax, 11.07, yl(1) + 0.03*diff(yl), 'no hover', 'Color', E.rojo, 'FontSize', E.fs - 1, 'VerticalAlignment', 'bottom');
end

function eje_carga(fig, ax, S, E, panel)
% eje superior con el estado de carga correspondiente a la tension en reposo
p = get(ax, 'Position');
a2 = axes('Parent', fig, 'Units', 'centimeters', 'Position', p, 'Color', 'none', 'XAxisLocation', 'top', ...
    'YTick', [], 'XDir', 'reverse', 'XLim', get(ax, 'XLim'), 'Box', 'off', 'TickDir', 'out', ...
    'FontName', E.fuente, 'FontSize', E.fs, 'LineWidth', E.lw_eje);
[vv, i] = sort(S.Voc); set(a2, 'XTick', vv, 'XTickLabel', arrayfun(@(s) sprintf('%d', round(s)), S.soc_pct(i), 'UniformOutput', false));
try, set(a2, 'YColor', 'none'); catch, set(a2, 'YColor', 'w'); end
xlabel(a2, 'State of charge (%)', 'FontName', E.fuente, 'FontSize', E.fs);
text(ax, 0, 1.24, panel, 'Units', 'normalized', 'FontName', E.fuente, 'FontSize', E.fs_pan, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
end
