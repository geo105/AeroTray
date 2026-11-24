function fig_identificacion()
%FIG_IDENTIFICACION Fig. 4 del paper: centro de empuje frente a inclinacion y mapa de empuje.
E = estilo_paper();
P = leer_csv('ident_puntos.csv'); A = leer_csv('ident_ajuste.csv');
K = leer_csv('ident_param.csv'); U = leer_csv('empuje.csv');
fig = nueva_figura(E.ancho, 5.6);

% (a) balance de momentos
ax = crear_ejes(fig, [1.25 1.05 5.4 4.05], E);
h1 = plot(ax, P.theta_deg, P.xcp_mm, 'o', 'Color', E.azul, 'MarkerFaceColor', E.azul, 'MarkerSize', 4);
h2 = plot(ax, P.phi_deg, P.ycp_mm, 's', 'Color', E.rojo, 'MarkerFaceColor', E.rojo, 'MarkerSize', 4);
h5 = plot(ax, A.ang_deg, A.x_mm, '-', 'Color', E.azul, 'LineWidth', E.lw);
h6 = plot(ax, A.ang_deg, A.y_mm, '--', 'Color', E.rojo, 'LineWidth', E.lw);
% Desplazamientos manuales por numero de vuelo (3..9) para evitar solapamientos (6/8 y 9/4)
dx_lbl = [ 0.12,  0.14,  0.12, -0.32, -0.32,  0.16,  0.12];
dy_lbl = [ 1.0,   1.2,   1.0,  -2.8,   1.0,   0.4,  -2.2];
for k = 1:numel(P.vuelo)
    idx = P.vuelo(k) - 2;
    dxk = 0.12; dyk = 0.9;
    if idx >= 1 && idx <= numel(dx_lbl), dxk = dx_lbl(idx); dyk = dy_lbl(idx); end
    text(ax, P.theta_deg(k) + dxk, P.xcp_mm(k) + dyk, sprintf('%d', P.vuelo(k)), 'FontSize', E.fs - 1, 'Color', E.azul);
end
xlim(ax, [-2 4.2]); ylim(ax, [-6 48]);
set(ax, 'XTick', -2:1:4, 'YTick', 0:10:40);
txt = {strrep(sprintf('\\delta_x = %.1f mm, \\delta_y = %.1f mm', K.dx_mm, K.dy_mm), '.', ','), strrep(sprintf('h_c = %.2f m', K.hc_m), '.', ',')};
text(ax, 4.1, 47, txt, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'FontSize', E.fs - 1);
leyenda(ax, [h1 h5 h2 h6], {'x_{cp} (cabeceo \theta)', 'ajuste x_{cp}', 'y_{cp} (alabeo \phi)', 'ajuste y_{cp}'}, 'northwest', E);
rotular(ax, E, 'Inclinación media en vuelo estacionario (°)', 'Centro de empuje (mm)', '(a)');

% (b) mapa de empuje
ax = crear_ejes(fig, [8.15 1.05 5.15 4.05], E);
u = U.u_us;
fill(ax, [1725 1775 1775 1725], [0 0 62 62], E.claro(E.verde, 0.15), 'EdgeColor', 'none');
h3 = fill(ax, [u; flipud(u)], [min(U.T_id_min, 62); flipud(min(U.T_id_max, 62))], E.claro(E.verde, 0.45), 'EdgeColor', 'none');
h4 = plot(ax, u, U.T_firmware, '-', 'Color', E.gris, 'LineWidth', E.lw);
plot(ax, [1300 2000], K.peso_N*[1 1], ':', 'Color', E.negro, 'LineWidth', E.lw_ref);
text(ax, 1310, K.peso_N + 2.2, 'peso, 3,6 kg', 'FontSize', E.fs - 1);
plot(ax, [1899 1899], [0 62], '--', 'Color', E.rojo, 'LineWidth', E.lw_ref);
text(ax, 1893, 12, {'mapa del', 'firmware:', '1899 µs'}, 'Color', E.rojo, 'FontSize', E.fs - 1, ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
plot(ax, [1770 1770], [0 62], '-.', 'Color', E.azul, 'LineWidth', E.lw_ref);
text(ax, 1760, 3, {'auditoría:', '1770 µs'}, 'Color', E.azul, 'FontSize', E.fs - 1, ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
xlim(ax, [1300 2000]); ylim(ax, [0 62]); set(ax, 'XTick', 1300:100:2000, 'YTick', 0:10:60);
leyenda(ax, [h4 h3], {'firmware, k_F = 5,40 N', 'identificado, 7,17–8,18 N'}, 'northwest', E);
rotular(ax, E, 'Orden colectiva u (µs)', 'Empuje total (N)', '(b)');
guardar_figura(fig, 'fig_ident');
end
